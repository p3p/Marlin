/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(BINARY_FILE_TRANSFER)

#include "transport_layer.h"
#include "sdfile_transfer.h"

uint8_t BinaryStream::next_serial_device_id = 0;
BinaryStream BinaryStream::port[NUM_SERIAL]{};

// blocking serial write
void BinaryStream::write_packet(Packet& packet) {
  bs_write_serial(serial_device_id, (char *)packet.header.data, sizeof(BinaryStream::Packet::Header));
  if (packet.header.size) {
    bs_write_serial(serial_device_id, (char *)(packet.buffer),  packet.header.size);
    bs_write_serial(serial_device_id, (char *)(packet.footer.data), sizeof(BinaryStream::Packet::Footer));
  }
}

// nonblocking serial read
size_t BinaryStream::stream_read(uint8_t* buffer, size_t length) {
  if (rx_stream.state != ReceiveState::PACKET_WAIT && ELAPSED(millis(), rx_packet.timeout)) {
    rx_stream.state = rx_stream.state == ReceiveState::PACKET_RESPONSE ? ReceiveState::PACKET_RESET : ReceiveState::PACKET_TIMEOUT;
    return -1;
  }
  length = min(bs_serial_data_available(serial_device_id), length);
  size_t received = bs_read_serial(serial_device_id, (char*)buffer, length);
  if (received) rx_packet.timeout = millis() + (PACKET_MAX_WAIT);
  return received;
}

void BinaryStream::receive() {
  millis_t transfer_window = millis() + RX_TIMESLICE;

  #if ENABLED(SDSUPPORT)
    PORT_REDIRECT(serial_device_id);
  #endif

  while (PENDING(millis(), transfer_window)) {
    switch (rx_stream.state) {
      case ReceiveState::PACKET_RESET:
        rx_packet.reset();
        rx_stream.state = ReceiveState::PACKET_WAIT;
      case ReceiveState::PACKET_WAIT:
        if (!stream_read(&rx_packet.header.data[1], 1)) { if (!idle()) return; else break; }  // no active packet so don't wait
        rx_packet.packet_type = rx_packet.header.packet_type();

        // packet stream frameing (14 bit)
        if ((rx_packet.header.token & 0xFCFF) != Packet::Header::HEADER_TOKEN) {
          rx_packet.header.data[0] = rx_packet.header.data[1];
          break;
        }
        // base packet type (2 bit)
        rx_packet.bytes_received = 2;
        if (rx_packet.packet_type == Packet::Type::RESPONSE) {
            rx_stream.state = ReceiveState::PACKET_RESPONSE;
        } else {
            rx_stream.state = ReceiveState::PACKET_HEADER;
        }
        break;
      case ReceiveState::PACKET_RESPONSE:
        rx_packet.bytes_received += stream_read((uint8_t*)rx_packet.response.data + rx_packet.bytes_received, sizeof(Packet::Response) - rx_packet.bytes_received);
        if (rx_packet.bytes_received != sizeof(Packet::Response)) break;

        if (Checksum::crc8(0, (uint8_t *)rx_packet.response.data, sizeof(Packet::Response) - sizeof(Packet::Response::checksum)) == rx_packet.response.checksum) process_response();
        rx_stream.state = ReceiveState::PACKET_RESET; // silently reject corrupt responses, will timeout and request resend
        break;
      case ReceiveState::PACKET_HEADER:
        rx_packet.bytes_received += stream_read((uint8_t*)rx_packet.header.data + rx_packet.bytes_received, sizeof(Packet::Header) - rx_packet.bytes_received);
        if (rx_packet.bytes_received != sizeof(Packet::Header)) break;

        // if the header validated then we are pretty sure packet is good
        if (rx_packet.header.checksum == Checksum::crc8(0, (uint8_t *)rx_packet.header.data, sizeof(Packet::Header) - sizeof(Packet::Header::checksum))) {

          // is the packet expected? FAF packets do not get flow control
          if (rx_packet.header.sync == rx_stream.sync || rx_packet.packet_type == Packet::DATA_FAF) {

            // can we actually fit the payload in the buffer
            if (rx_packet.header.size > RX_STREAM_BUFFER_SIZE) {
              rx_stream.state = ReceiveState::PACKET_ERROR;
              break;
            }

            rx_packet.bytes_received = 0;
            // the packet has a payload
            if (rx_packet.header.size) {
              rx_packet.buffer = static_cast<char *>(rx_buffer); // multipacket buffering not implemented, always allocate whole receive buffer to packet
              rx_stream.state = ReceiveState::PACKET_DATA;
              break;
            }
            // no payload just process it
            rx_stream.state = ReceiveState::PACKET_PROCESS;
          }
          // Currently in retry state, There could be packets already rx buffered, just drop them
          else if (rx_stream.retries) {
            SERIAL_ECHO_MSG("retry waiting for resync");
            rx_stream.state = ReceiveState::PACKET_RESET;
          }
          // Remote resent previous packet? the ack response must have been lost in transit
          else if (rx_packet.header.sync == rx_stream.sync - 1) {
            SERIAL_ECHO_MSG("ok response must have been lost, resending ack");
            transmit_response(BinaryStreamControl::Packet::ACK, rx_packet.header.sync);    // transmit acknoledge and drop the payload
            rx_stream.state = ReceiveState::PACKET_RESET;
          }
          // request resend of the expected packet to see if remote can fix packet stream
          else {
            SERIAL_ECHO_MSG("Datastream packet out of order");
            rx_stream.state = ReceiveState::PACKET_RESEND;
          }
        }
        // The header was corrupted, the start token matched something but the data could be meaningless
        // in order to speed up stream recovery we trust that the packet type is correct, sending
        // a retry respone when necessary rather than just timeout remote.

        // FAF packets do not get flow control / nor feedback on corruption, just drop it
        else if (rx_packet.packet_type == Packet::DATA_FAF) {
          SERIAL_ECHO_MSG("Ignore dodgy FAF packet");
          rx_stream.state = ReceiveState::PACKET_RESET;
        // the stream is in resend state waiting for a specific sync, drop it, remote will resend and wait repeatedly until Acked
        } else if (rx_stream.retries) {
          SERIAL_ECHO_START();
          SERIAL_ECHOLNPAIR("Packet header(", rx_packet.header.sync, "?) corrupt, ignore as in reset state");
          rx_stream.state = ReceiveState::PACKET_RESET;
        // the packet header is corrupt request resend of the expected packet
        } else {
          SERIAL_ECHO_START();
          SERIAL_ECHOLNPAIR("Packet header(", rx_packet.header.sync, "?) corrupt");
          rx_stream.state = ReceiveState::PACKET_RESEND;
        }
        break;
      case ReceiveState::PACKET_DATA: {
        size_t received = stream_read((uint8_t*)rx_packet.buffer + rx_packet.bytes_received, rx_packet.header.size - rx_packet.bytes_received);
        rx_packet.checksum = Checksum::crc16(rx_packet.checksum, (uint8_t *)rx_packet.buffer + rx_packet.bytes_received, received);
        rx_packet.bytes_received += received;
        if (rx_packet.bytes_received != rx_packet.header.size) break;

        rx_stream.state = ReceiveState::PACKET_FOOTER;
        rx_packet.bytes_received = 0;
        break;
      }
      case ReceiveState::PACKET_FOOTER:
        rx_packet.bytes_received += stream_read((uint8_t*)rx_packet.footer.data + rx_packet.bytes_received, sizeof(Packet::Footer) - rx_packet.bytes_received);
        if (rx_packet.bytes_received != sizeof(Packet::Footer)) break;

        if (rx_packet.footer.checksum == rx_packet.checksum) {
          rx_stream.state = ReceiveState::PACKET_PROCESS;
        } else {
          // FAF packets are just ignored when corrupted
          SERIAL_ECHO_START();
          SERIAL_ECHOLNPAIR("Packet(", rx_packet.header.sync, ") payload corrupt (", rx_packet.checksum, ")");
          rx_stream.state = rx_packet.packet_type == Packet::DATA_FAF ? ReceiveState::PACKET_RESET : ReceiveState::PACKET_RESEND;
        }
        break;
      case ReceiveState::PACKET_PROCESS:
        if (rx_packet.packet_type != Packet::DATA_FAF) rx_stream.sync ++;
        rx_stream.retries = 0;
        rx_stream.bytes += rx_packet.header.size;
        dispatch();
        rx_stream.state = ReceiveState::PACKET_RESET;
        break;
      case ReceiveState::PACKET_RESEND:
        if (rx_stream.retries < MAX_RETRIES || MAX_RETRIES == 0) {
          rx_stream.retries++;
          rx_stream.state = ReceiveState::PACKET_RESET;
          SERIAL_ECHO_START();
          SERIAL_ECHOLNPAIR("Resend request ", int(rx_stream.retries), " packet sync: ", rx_packet.header.sync, " stream sync: ", rx_stream.sync);
          transmit_response(BinaryStreamControl::Packet::NACK, rx_stream.sync);
        } else {
          SERIAL_ECHO_MSG("Too many resend requests, Sync lost");
          rx_stream.state = ReceiveState::PACKET_ERROR;
        }
        break;
      case ReceiveState::PACKET_TIMEOUT:
        SERIAL_ECHO_MSG("Datastream timeout");
        rx_stream.state = ReceiveState::PACKET_RESEND;
        break;
      case ReceiveState::PACKET_ERROR:
        SERIAL_ECHO_MSG("Fatal Sync Error");
        transmit_response(BinaryStreamControl::Packet::ERROR, rx_packet.header.sync);
        reset(); // reset everything, resync required
        rx_stream.state = ReceiveState::PACKET_RESET;
        break;
    }
  }
}
/**
 *  Once packets are received and validated they are dispatched to the appropriate protocol
 */
void BinaryStream::dispatch() {
  switch(static_cast<Protocol>(rx_packet.header.protocol_id)) {
    case Protocol::CONTROL:
      switch(rx_packet.header.packet_id) {
        case BinaryStreamControl::Packet::SYNC: {
          // reset the connection for sync
          reset();
          if (rx_packet.header.packet_type() == BinaryStream::Packet::Type::DATA_FAF) {
            send_packet(new(tx_buffer) BinaryStream::PacketPacker(Packet::DATA, (uint8_t)Protocol::CONTROL, BinaryStreamControl::Packet::SYNC, BinaryStreamControl::Packet::Sync{VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, RX_BUFFER_SIZE, sizeof(rx_buffer)}));
          } else {
            transmit_response(BinaryStreamControl::Packet::ACK, rx_packet.header.sync);
            // todo: we syned so check version compliance and updade max payload size
          }
          break;
        }
        case BinaryStreamControl::Packet::CLOSE: // revert back to ASCII mode
          transmit_response(BinaryStreamControl::Packet::ACK, rx_packet.header.sync);
          BinaryStream::disable(serial_device_id);
          break;
        #if ENABLED(PLATFORM_M997_SUPPORT)
        case BinaryStreamControl::Packet::RESET_MCU:
          transmit_response(BinaryStreamControl::Packet::ACK, rx_packet.header.sync);
          delay(10);
          flashFirmware(0);
          break;
        #endif
        default:
          transmit_response(BinaryStreamControl::Packet::REJECT, rx_packet.header.sync);
      }
      break;

    case Protocol::FILE_TRANSFER:
      if (SDFileTransferProtocol::process_ready(rx_packet.header.packet_id, rx_packet.header.size)) {               // if the protocol can handle packet type and the amount of data
        transmit_response(BinaryStreamControl::Packet::ACK, rx_packet.header.sync);                                 // acknoledge receipt
        SDFileTransferProtocol::process(this, rx_packet.header.packet_id, rx_packet.buffer, rx_packet.header.size); // and send payload to be processed
      } else {
        transmit_response(BinaryStreamControl::Packet::NYET, rx_packet.header.sync);                                // otherwise nyet the packet so host resends
      }
    break;

    default:
      transmit_response(BinaryStreamControl::Packet::REJECT, rx_packet.header.sync);
  }
}

uint32_t tx_timeout = 0;
void BinaryStream::transmit_update() {
  switch (tx_stream.state) {
    case TransmitState::IDLE:
      if (tx_queue.available()) {
        tx_active = tx_queue.pop();
        build_packet(tx_active);
        transmit_packet();
      }
      break;
    case TransmitState::BUSY: // todo: this may be useful for spreading a transmit over multiple update slices, will require response caching
      break;
    case TransmitState::WAITING: // NYET or no response timeout
      if (millis() > tx_timeout) {
        SERIAL_ECHOLN("BinaryStream::transmit_update::WAITING: Transmit timeout (no ack) resend last packet");
        write_packet(tx_packet);
        tx_timeout = millis() + 1000;
      }
      break;
  }
}

void BinaryStream::process_response() {
  if (rx_packet.response.sync_id != tx_stream.sync) {
    // Syncronisation error?
    tx_stream.state = TransmitState::IDLE;
    SERIAL_ECHOLN("Received out of sync response");
    return;
  }

  switch (rx_packet.response.response) {
    case BinaryStreamControl::Packet::ACK:
        tx_stream.state = TransmitState::IDLE;
        tx_active->status = TransmitState::COMPLETE;
        tx_stream.sync ++;
      break;
    case BinaryStreamControl::Packet::NACK:
      // packet payload corrupted retry
      SERIAL_ECHOLN("Received NACK");
      tx_active->status = TransmitState::RETRY;
      write_packet(tx_packet);
      break;
    case BinaryStreamControl::Packet::NYET:
      // remote doesn't want this packet yet
      tx_active->status = TransmitState::RETRY;
      tx_queue.push(tx_active); // return packet to queue
      tx_stream.state = TransmitState::IDLE;
      SERIAL_ECHOLN("Received NYET");
      break;
    case BinaryStreamControl::Packet::REJECT:
      // remote does not understand this protocol
      tx_active->status = TransmitState::ERROR;
      tx_stream.state = TransmitState::IDLE;
      SERIAL_ECHOLN("Received REJECT");
      break;
    case BinaryStreamControl::Packet::ERROR:
      // transport layer error, close and resyncronise
      tx_active->status = TransmitState::ERROR;
      tx_stream.state = TransmitState::IDLE;
      SERIAL_ECHOLN("Received ERROR");
      break;
  }
  tx_active->response = rx_packet.response.response;
}

void BinaryStream::transmit_response(uint8_t response, uint8_t sync) {
  // check the current packet needs a response

  //currently imposible: if the tx is currently busy we cant send
  if (tx_stream.state == TransmitState::BUSY) {
    SERIAL_ECHOLN("Need to send Response but serial is busy");
    return;
  }

  // if tx is idle or waiting we cant squeeze the response through anyway
  if (rx_packet.packet_type == Packet::Type::DATA_FAF || (rx_packet.packet_type == Packet::Type::DATA_NAK && response == BinaryStreamControl::Packet::ACK)) return;
  Packet::Response packet{ Packet::Header::HEADER_TOKEN, response, sync, 0};

  packet.checksum = Checksum::crc8(0, (uint8_t *)packet.data, (sizeof(Packet::Response) - sizeof(Packet::Response::checksum)));
  bs_write_serial(serial_device_id, (char*)packet.data, sizeof(Packet::Response));
}

uint8_t BinaryStream::transmit_packet() {
  if (tx_stream.state != TransmitState::IDLE) {
    SERIAL_ECHOLNPAIR("trying to transmit while last packet is not acked", tx_stream.state);
    return 1;
  }
  tx_stream.state = tx_active->status = TransmitState::BUSY;
  write_packet(tx_packet);
  tx_timeout = millis() + 1000;
  tx_stream.state = tx_active->status = TransmitState::WAITING;
  return 0;
}

uint8_t BinaryStream::build_packet(PacketInfo* packet_info) {
  if (tx_stream.state != TransmitState::IDLE) {
    SERIAL_ECHOLNPAIR("trying to build new packet while last packet is not acked", tx_stream.state);
    return 1;
  }
  tx_packet.reset();
  tx_packet.header.token = Packet::Header::HEADER_TOKEN | (BinaryStream::Packet::Type::DATA << 8);
  tx_packet.header.sync = tx_stream.sync;
  tx_packet.header.protocol_id = packet_info->protocol_id;
  tx_packet.header.packet_id = packet_info->packet_id;
  tx_packet.header.size = packet_info->payload_length;
  tx_packet.header.checksum = Checksum::crc8(0, (uint8_t *)tx_packet.header.data, (sizeof(Packet::Header) - sizeof(Packet::Header::checksum)));

  if (packet_info->payload_length && packet_info->payload != nullptr) {
    tx_packet.buffer = packet_info->payload;
    tx_packet.footer.checksum = Checksum::crc16(0, (uint8_t *)tx_packet.buffer, tx_packet.header.size);
  }
  return 0;
}

uint8_t BinaryStream::send_packet(PacketInfo* packet_info) {
  tx_queue.push(packet_info);
  return 0;
}

bool BinaryStream::idle() {
  transmit_update();

  // Some Protocols may need periodic updates without new data
  size_t busy = SDFileTransferProtocol::idle(this);
  return busy + tx_stream.state;
}

#endif // BINARY_FILE_TRANSFER

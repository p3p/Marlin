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

void write_packet(int8_t serial_port, BinaryStream::Packet& packet) {
  bs_write_serial(serial_port, (char *)packet.header.data, sizeof(BinaryStream::Packet::Header));
  if (packet.header.size) {
    bs_write_serial(serial_port, (char *)(packet.buffer),  packet.header.size);
    bs_write_serial(serial_port, (char *)(packet.footer.data), sizeof(BinaryStream::Packet::Footer));
  }
}

size_t BinaryStream::stream_read(uint8_t* buffer, size_t length) {
  if (rx_stream.state != ReceiveState::PACKET_WAIT && ELAPSED(millis(), rx_packet.timeout)) {
    rx_stream.state = rx_stream.state == ReceiveState::PACKET_RESPONSE ? ReceiveState::PACKET_RESET : ReceiveState::PACKET_TIMEOUT;
    return -1;
  }
  // don't block
  length = min(bs_serial_data_available(card.transfer_port_index), length);
  size_t received = bs_read_serial(card.transfer_port_index, (char*)buffer, length);
  if (received) rx_packet.timeout = millis() + (PACKET_MAX_WAIT);
  return received;
}

void BinaryStream::dispatch() {
  static BinaryStreamControl::Packet::Sync sync_response{ sizeof(rx_buffer), VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH };
  static PacketInfo sync_packet{Packet::DATA, (uint8_t)Protocol::CONTROL, BinaryStreamControl::Packet::SYNC, (char*)&sync_response, sizeof(sync_response)};

  switch(static_cast<Protocol>(rx_packet.header.protocol_id)) {
    case Protocol::CONTROL:
      switch(rx_packet.header.packet_id) {
        case BinaryStreamControl::Packet::SYNC: {
          transmit_response(BinaryStreamControl::Packet::ACK, rx_packet.header.sync);      
          // reset the connection for sync
          reset();
          send_packet(&sync_packet);
          break;
        }
        case BinaryStreamControl::Packet::CLOSE: // revert back to ASCII mode
          transmit_response(BinaryStreamControl::Packet::ACK, rx_packet.header.sync);
          card.flag.binary_mode = false; //todo: odd flag to be using now
          break;
        default:
          transmit_response(BinaryStreamControl::Packet::REJECT, rx_packet.header.sync);
      }
      break;
    case Protocol::FILE_TRANSFER:
      if (SDFileTransferProtocol::process_ready(rx_packet.header.packet_id, rx_packet.header.size)) {            // if the protocol can handle packet type and the amount of data
        transmit_response(BinaryStreamControl::Packet::ACK, rx_packet.header.sync);                                    // acknoledge receipt
        SDFileTransferProtocol::process(this, rx_packet.header.packet_id, rx_packet.buffer, rx_packet.header.size); // and send payload to be processed
      } else {
        transmit_response(BinaryStreamControl::Packet::NYET, rx_packet.header.sync);
      }
    break;
    default:
      transmit_response(BinaryStreamControl::Packet::REJECT, rx_packet.header.sync);
  }
}

void BinaryStream::transmit_complete(uint8_t response) {
  switch(static_cast<Protocol>(tx_packet.header.protocol_id)) {
    case Protocol::CONTROL:
      // handled elsewhere
      break;
    case Protocol::FILE_TRANSFER:
      SDFileTransferProtocol::transmit_complete(tx_packet.header.sync, response);
      break;
    default:
      SERIAL_ECHO_MSG("Unsupported Binary Protocol");
      break;
  }
}

uint32_t tx_timeout = 0;
void BinaryStream::transmit_update() {
  switch (tx_stream.state) {
    case TransmitState::IDLE:
      if (tx_queue.available()) {
        build_packet(tx_queue.pop());
        transmit_packet();
      }
      break;
    case TransmitState::BUSY: // todo: this is useful for spreading a transmit over update slices
      break;
    case TransmitState::WAITING:
      if (millis() > tx_timeout) {
        SERIAL_ECHOLN("BinaryStream::transmit_update::WAITING: Transmit timeout (no ack) resend last packet");
        write_packet(card.transfer_port_index, tx_packet);
        tx_timeout = millis() + 1000;
      }
      break;
  }
}

void BinaryStream::process_response() {
  switch (response.response) {
    case BinaryStreamControl::Packet::ACK:
      if (response.sync_id == tx_stream.sync) {
        tx_stream.state = TransmitState::IDLE;
        tx_stream.sync ++;
        SERIAL_ECHOLN("Received ACK");
        return;
      }
      SERIAL_ECHOLN("Received bad ACK");
      // wrong packet acked? desyncronised stream error?
      tx_stream.state = TransmitState::IDLE;
      break;
    case BinaryStreamControl::Packet::NACK:
      // packet payload corrupted retry
      SERIAL_ECHOLN("Received NACK");
      write_packet(card.transfer_port_index, tx_packet);
      break;
    case BinaryStreamControl::Packet::NYET:
      // remote rx buffer is full, wait and retry
      tx_stream.state = TransmitState::IDLE;
      SERIAL_ECHOLN("Received NYET");
      break;
    case BinaryStreamControl::Packet::REJECT:
      // remote does not understand this protocol 
      tx_stream.state = TransmitState::IDLE;
      SERIAL_ECHOLN("Received REJECT");
      break;
    case BinaryStreamControl::Packet::ERROR:
      // transport layer error, close and resyncronise
      tx_stream.state = TransmitState::IDLE;
      SERIAL_ECHOLN("Received ERROR");
      break;
  }
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
  ResponsePacket packet{Packet::Header::HEADER_TOKEN, response, sync, 0};

  uint16_t packet_checksum = 0;
  for( size_t i = 0; i < (sizeof(ResponsePacket) - 1); i++) {
    packet_checksum = checksum( packet_checksum, packet.data[i]);
  }

  packet.checksum = checksum16to8(packet_checksum);
  bs_write_serial(card.transfer_port_index, (char*)packet.data, sizeof(ResponsePacket));
}

uint8_t BinaryStream::transmit_packet() {
  if (tx_stream.state != TransmitState::IDLE) {
    SERIAL_ECHOLNPAIR("trying to transmit while last packet is not acked", tx_stream.state);
    return 1;
  }
  SERIAL_ECHOLN("Sending packet");
  tx_stream.state = TransmitState::BUSY;
  write_packet(card.transfer_port_index, tx_packet);
  tx_timeout = millis() + 1000;
  tx_stream.state = TransmitState::WAITING;
  return 0;
}

uint8_t BinaryStream::build_packet(PacketInfo* packet_info) {
  if (tx_stream.state != TransmitState::IDLE) {
    SERIAL_ECHOLNPAIR("trying to build new packet while last packet is not acked", tx_stream.state);
    return 1;
  }
  SERIAL_ECHOLNPAIR("Building packet (", tx_stream.sync, ", ", (int)packet_info->protocol_id, ", ", packet_info->packet_id, ", " , packet_info->payload_length, ")");

  tx_packet.reset();
  tx_packet.header.token = Packet::Header::HEADER_TOKEN + BinaryStream::Packet::Type::DATA;
  tx_packet.header.sync = tx_stream.sync;
  tx_packet.header.protocol_id = packet_info->protocol_id;
  tx_packet.header.packet_id = packet_info->packet_id;
  tx_packet.header.size = packet_info->payload_length;

  for (size_t i = 0; i < (sizeof(Packet::Header) - sizeof(tx_packet.header.checksum)); i++) {
    tx_packet.checksum = checksum( tx_packet.checksum, tx_packet.header.data[i]);
  }
  tx_packet.header.checksum = checksum16to8(tx_packet.checksum);

  if (packet_info->payload_length && packet_info->payload != nullptr) {
    tx_packet.buffer = (char*)packet_info->payload;
    tx_packet.footer.checksum = checksum( tx_packet.checksum, tx_packet.header.data[sizeof(Packet::Header) - 1]);

    for (size_t i = 0; i < packet_info->payload_length; i++) {
      tx_packet.footer.checksum = checksum( tx_packet.footer.checksum, tx_packet.buffer[i]);
    }
  }
  return 0;
}

uint8_t BinaryStream::send_packet(PacketInfo* packet_info) {
  tx_queue.push(packet_info);
  return 0;
}

void BinaryStream::idle() {
  transmit_update(); 

  // Some Protocols may need periodic updates without new data
  SDFileTransferProtocol::idle();
}

BinaryStream binaryStream[NUM_SERIAL];

#endif // BINARY_FILE_TRANSFER

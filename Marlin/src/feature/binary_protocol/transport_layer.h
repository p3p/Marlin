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
#pragma once

#define BINARY_STREAM_RX_BUFFER_SIZE 512

#include "../../inc/MarlinConfig.h"
#include "../../sd/cardreader.h"

[[gnu::always_inline]] inline size_t bs_serial_data_available(const uint8_t index) {
  switch (index) {
    case 0: return MYSERIAL0.available();
    #if HAS_MULTI_SERIAL
      case 1: return MYSERIAL1.available();
    #endif
  }
  return 0;
}

[[gnu::always_inline]] inline int bs_read_serial(const uint8_t index) {
  switch (index) {
    case 0: return MYSERIAL0.read();
    #if HAS_MULTI_SERIAL
      case 1: return MYSERIAL1.read();
    #endif
  }
  return -1;
}

[[gnu::always_inline]] inline size_t bs_read_serial(const uint8_t index, const char* buffer, const size_t length) {
  switch (index) {
    case 0:
      return MYSERIAL0.readBytes((char*)buffer, length);
    #if HAS_MULTI_SERIAL
      case 1: {
        return MYSERIAL1.readBytes((char *)buffer, length);
    }
    #endif
  }
  return -1;
}

[[gnu::always_inline]] inline size_t bs_write_serial(const uint8_t index, const char* buffer, const size_t length) {
  switch (index) {
    case 0:
      return MYSERIAL0.write((char*)buffer, length);
    #if HAS_MULTI_SERIAL
      case 1: {
        return MYSERIAL1.write((char *)buffer, length);
    }
    #endif
  }
  return -1;
}

class BinaryStreamControl {
public:
  struct Packet {
    enum Type { ACK, NACK, NYET, REJECT, ERROR, SYNC, QUERY, CLOSE };
    struct [[gnu::packed]] Response {
      uint8_t sync;
    };
    struct [[gnu::packed]] Error {
      uint8_t sync, code;
    };
    struct [[gnu::packed]] Sync {
      uint16_t size, version_major, version_minor, version_patch;
    };
  };
};

class BinaryStream {
public:
  enum class Protocol : uint8_t { CONTROL, FILE_TRANSFER };
  enum ReceiveState { PACKET_RESET, PACKET_WAIT, PACKET_HEADER, PACKET_DATA, PACKET_FOOTER,
                                     PACKET_PROCESS, PACKET_RESEND, PACKET_RESPONSE, PACKET_TIMEOUT, PACKET_ERROR };
  enum TransmitState { IDLE, BUSY, WAITING, RETRY };

  struct ResponsePacket { // 5 byte minimal response packet.
    union {
      struct [[gnu::packed]] {
        uint16_t token;
        uint8_t response;
        uint8_t sync_id;
        uint8_t checksum;
      };
      char data[5];
    };
  } response;

  struct PacketInfo {
    uint8_t type = 0;
    uint8_t protocol_id = 0;
    uint8_t packet_id = 0;
    char* payload = nullptr;
    uint16_t payload_length = 0;
    PacketInfo* next = nullptr;
    // ***** uint8_t response_code
    //       uint8_t status_code

    void set(uint8_t type, uint8_t protocol_id, uint8_t packet_id, char* payload, uint16_t payload_length) {
      this->type = type;
      this->protocol_id = protocol_id;
      this->packet_id = packet_id;
      this->payload = payload;
      this->payload_length = payload_length;
    }
  };

  struct TxQueue {
    PacketInfo* head = nullptr;
    PacketInfo* tail = nullptr;

    bool available() {
        return head != nullptr;
    }
    // no memory ownership is taken, caller needs to manage it
    void push(PacketInfo* packet) {
      if (tail == nullptr) {
        head = tail = packet;
      } else {
        tail->next = packet;
        tail = packet;
      }
    }
    PacketInfo* pop() {
      if (head == nullptr) return nullptr;

      PacketInfo* ret = head;
      if (head == tail) {
        head = tail = nullptr;
        return ret;
      }

      head = head->next;
      ret->next = nullptr;
      return ret;
    }
  } tx_queue;

  struct Packet { // 10 byte protocol overhead, ascii with checksum and line number has a minimum of 7 increasing with line
    enum Type { RESPONSE, DATA, DATA_NAK, DATA_FAF };
    struct Header {
      static constexpr uint16_t HEADER_TOKEN = 0xB5AC;
      union {
        struct [[gnu::packed]] {
          uint16_t token;       // packet start token
          uint8_t sync;         // stream sync, resend id and packet loss detection
          uint8_t protocol_id;  // 4 bit protocol,
          uint8_t packet_id;    // 4 bit packet type
          uint16_t size;        // data length
          uint8_t checksum;     // header checksum
        };
        uint8_t data[8];
      };
      uint8_t packet_type() { return token & 0x3; }
      void reset() { token = 0; sync = 0; protocol_id = 0; packet_id = 0; size = 0; checksum = 0; }
    };

    union Footer {
      struct [[gnu::packed]] {
        uint16_t checksum; // full packet checksum
      };
      uint8_t data[2];
      void reset() { checksum = 0; }
    };

    Header header;
    Footer footer;
    uint32_t bytes_received;
    uint16_t checksum;
    uint8_t header_checksum;
    millis_t timeout;
    uint16_t packet_type;
    char* buffer;

    void reset() {
      header.reset();
      footer.reset();
      bytes_received = 0;
      checksum = 0;
      header_checksum = 0;
      timeout = millis() + PACKET_MAX_WAIT;
      buffer = nullptr;
    }
  } rx_packet{}, tx_packet{};

  struct StreamState {
    uint8_t sync;
    uint8_t retries;
    size_t bytes;
    size_t errors;
    uint8_t state;

    void reset() {
      state = 0;
      sync = 0;
      retries = 0;
      bytes = 0;
      errors = 0;
    }
  } rx_stream{}, tx_stream{};

  void reset() {
    rx_stream.reset();
    tx_stream.reset();
    buffer_next_index = 0;
  }

  // fletchers 16 checksum
  static uint32_t checksum(uint32_t cs, uint8_t value) {
    uint16_t cs_low = (((cs & 0xFF) + value) % 255);
    return ((((cs >> 8) + cs_low) % 255) << 8)  | cs_low;
  }

  static uint8_t checksum16to8(uint16_t cs) {
    uint8_t cs_low = ((cs & 0xFF) % 255);
    return (((cs >> 8) + cs_low) % 255);
  }

  // read the next byte from the data stream keeping track of
  // whether the stream times out from data starvation
  // takes the data variable by reference in order to return status
  bool stream_read(uint8_t* data);
  size_t stream_read(uint8_t* buffer, size_t length);

  void receive() {
    millis_t transfer_window = millis() + RX_TIMESLICE;

    #if ENABLED(SDSUPPORT)
      PORT_REDIRECT(card.transfer_port_index);
    #endif

    while (PENDING(millis(), transfer_window)) {
      switch (rx_stream.state) {
        case ReceiveState::PACKET_RESET:
          rx_packet.reset();
          rx_stream.state = ReceiveState::PACKET_WAIT;
        case ReceiveState::PACKET_WAIT:
          if (!stream_read(&rx_packet.header.data[1], 1)) { idle(); return; }  // no active packet so don't wait
          rx_packet.packet_type = rx_packet.header.token - rx_packet.header.HEADER_TOKEN;
          // packet stream frameing (14 bit)
          if (rx_packet.packet_type > Packet::Type::DATA_FAF) {
            rx_packet.header.data[0] = rx_packet.header.data[1];
            break;
          }
          // base packet type (2 bit)
          if (rx_packet.packet_type == Packet::Type::RESPONSE) {
              rx_packet.bytes_received = 2;
              response.data[0] = rx_packet.header.data[0];
              response.data[1] = rx_packet.header.data[1];
              rx_stream.state = ReceiveState::PACKET_RESPONSE;
          } else {
              rx_packet.bytes_received = 2;
              rx_stream.state = ReceiveState::PACKET_HEADER;
          }
          break;
        case ReceiveState::PACKET_RESPONSE:
          rx_packet.bytes_received += stream_read((uint8_t*)response.data + rx_packet.bytes_received, sizeof(ResponsePacket) - rx_packet.bytes_received);
          if (rx_packet.bytes_received != sizeof(ResponsePacket)) break;
          for (size_t i = 0; i < sizeof(ResponsePacket) - sizeof(ResponsePacket::checksum); ++i) {
            rx_packet.checksum = checksum(rx_packet.checksum, response.data[i]);
          }

          if (checksum16to8(rx_packet.checksum) == response.checksum) {
            process_response();
          }
          rx_stream.state = ReceiveState::PACKET_RESET;
          break;
        case ReceiveState::PACKET_HEADER:
          rx_packet.bytes_received += stream_read((uint8_t*)rx_packet.header.data + rx_packet.bytes_received, sizeof(Packet::Header) - rx_packet.bytes_received);
          if (rx_packet.bytes_received != sizeof(Packet::Header)) break;
          for (size_t i = 0; i < sizeof(Packet::Header); ++i) {
            rx_packet.checksum = checksum(rx_packet.checksum, rx_packet.header.data[i]);
            if (i == (sizeof(Packet::Header) - sizeof(Packet::Header::checksum)) - 1) rx_packet.header_checksum = checksum16to8(rx_packet.checksum);
          }

          // checksum validated so pretty sure packet is good
          if (rx_packet.header.checksum == rx_packet.header_checksum) {

            // Is the packet expected? FAF packets do not get flow control
            if (rx_packet.header.sync == rx_stream.sync || rx_packet.packet_type == Packet::DATA_FAF) {

              // can we actually fit the payload in the buffer
              if (rx_packet.header.size > RX_STREAM_BUFFER_SIZE) {
                rx_stream.state = ReceiveState::PACKET_ERROR;
                break;
              }

              buffer_next_index = 0;
              rx_packet.bytes_received = 0;
              // the packet has a payload
              if (rx_packet.header.size) {
                rx_stream.state = ReceiveState::PACKET_DATA;
                rx_packet.buffer = static_cast<char *>(&rx_buffer[0]); // multipacket buffering not implemented, always allocate whole buffer to packet
              }
              // no payload just process it
              else {
                rx_stream.state = ReceiveState::PACKET_PROCESS;
              }
            }
            // Currently in retry state, There could be packets already rx buffered, just drop them
            else if (rx_stream.retries) {
              SERIAL_ECHO_MSG("retry waiting for resync");
              rx_stream.state = ReceiveState::PACKET_RESET;
            }
            // remote resent previous packet? the ack response must have been lost
            else if (rx_packet.header.sync == rx_stream.sync - 1) {
              SERIAL_ECHO_MSG("ok response must have been lost, resending ack");
              transmit_response(BinaryStreamControl::Packet::ACK, rx_packet.header.sync);    // transmit acknoledge and drop the payload
              rx_stream.state = ReceiveState::PACKET_RESET;
            }
            // This shouldnt happen, so just request resend to see if remote can fix it
            else {
              SERIAL_ECHO_MSG("Datastream packet out of order");
              rx_stream.state = ReceiveState::PACKET_RESEND;
            }
          }
          // FAF packets do not get flow control / nor feedback on corruption, just drop it
          else if (rx_packet.packet_type == Packet::DATA_FAF) {
            SERIAL_ECHO_MSG("Ignore dodgy FAF packet");
            rx_stream.state = ReceiveState::PACKET_RESET;
          // the stream is in resend state waiting for a specific sync, drop it, remote will resend and wait repeatedly until Acked
          } else if (rx_stream.retries) {
            SERIAL_ECHO_START();
            SERIAL_ECHOLNPAIR("Packet header(", rx_packet.header.sync, "?) corrupt, ignore as in reset state");
            rx_stream.state = ReceiveState::PACKET_RESET;
          // the packet header is corrupt request resend
          } else {
            SERIAL_ECHO_START();
            SERIAL_ECHOLNPAIR("Packet header(", rx_packet.header.sync, "?) corrupt");
            rx_stream.state = ReceiveState::PACKET_RESEND;
          }
          break;
        case ReceiveState::PACKET_DATA:
          {
            size_t old_received = rx_packet.bytes_received;
            rx_packet.bytes_received += stream_read((uint8_t*)rx_packet.buffer + rx_packet.bytes_received, rx_packet.header.size - rx_packet.bytes_received);
            for (size_t i = old_received; i < rx_packet.bytes_received; ++i) {
              rx_packet.checksum = checksum(rx_packet.checksum, rx_packet.buffer[i]);
            }
            if (rx_packet.bytes_received != rx_packet.header.size) break;
          }
          rx_stream.state = ReceiveState::PACKET_FOOTER;
          rx_packet.bytes_received = 0;
          break;
        case ReceiveState::PACKET_FOOTER:
          rx_packet.bytes_received += stream_read((uint8_t*)rx_packet.footer.data + rx_packet.bytes_received, sizeof(Packet::Footer) - rx_packet.bytes_received);
          if (rx_packet.bytes_received != sizeof(Packet::Footer)) break;

          if (rx_packet.footer.checksum == rx_packet.checksum) {
            rx_stream.state = ReceiveState::PACKET_PROCESS;
          } else {
            // FAF packets are just ignored when corrupted
            SERIAL_ECHO_START();
            SERIAL_ECHOLNPAIR("Packet(", rx_packet.header.sync, ") payload corrupt");
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
          }
          else
            rx_stream.state = ReceiveState::PACKET_ERROR;
          break;
        case ReceiveState::PACKET_TIMEOUT:
          SERIAL_ECHO_MSG("Datastream timeout");
          rx_stream.state = ReceiveState::PACKET_RESEND;
          break;
        case ReceiveState::PACKET_ERROR:
          transmit_response(BinaryStreamControl::Packet::ERROR, rx_packet.header.sync);
          reset(); // reset everything, resync required
          rx_stream.state = ReceiveState::PACKET_RESET;
          break;
      }
    }
  }

  void dispatch();
  void transmit_complete(uint8_t response);
  void transmit_update();
  void process_response();
  void transmit_response(uint8_t response, uint8_t data);
  uint8_t transmit_packet();
  uint8_t build_packet(PacketInfo* packet_info);
  uint8_t send_packet(PacketInfo* packet_info);
  void idle();

  static const uint16_t PACKET_MAX_WAIT = 500, RX_TIMESLICE = 20, MAX_RETRIES = 0, VERSION_MAJOR = 0, VERSION_MINOR = 2, VERSION_PATCH = 0, RX_STREAM_BUFFER_SIZE = BINARY_STREAM_RX_BUFFER_SIZE;
  uint16_t buffer_next_index;
  char rx_buffer[RX_STREAM_BUFFER_SIZE];
};

extern BinaryStream binaryStream[NUM_SERIAL];

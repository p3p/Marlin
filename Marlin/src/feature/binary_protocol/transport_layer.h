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

struct FletchersChecksum {
  // fletchers 16 checksum
  static uint32_t process(uint32_t cs, uint8_t value) {
    uint16_t cs_low = (((cs & 0xFF) + value) % 255);
    return ((((cs >> 8) + cs_low) % 255) << 8)  | cs_low;
  }

  // convert the 16bit checksum down to 8 bit
  static uint8_t convert16to8(uint16_t cs) {
    uint8_t cs_low = ((cs & 0xFF) % 255);
    return (((cs >> 8) + cs_low) % 255);
  }
};


struct BinaryStreamControl {
  struct Packet {
    enum Type { ACK, NACK, NYET, REJECT, ERROR, SYNC, QUERY, CLOSE };
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
    char* buffer;
    Footer footer;

    uint32_t bytes_received;
    uint16_t checksum;
    uint8_t header_checksum;
    millis_t timeout;
    uint16_t packet_type;

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

  BinaryStream() : serial_device_id(next_serial_device_id++) {}

  bool stream_read(uint8_t* data);
  size_t stream_read(uint8_t* buffer, size_t length);

  void receive();
  void dispatch();
  void transmit_complete(uint8_t response);
  void transmit_update();
  void process_response();
  void transmit_response(uint8_t response, uint8_t data);
  uint8_t transmit_packet();
  uint8_t build_packet(PacketInfo* packet_info);
  uint8_t send_packet(PacketInfo* packet_info);
  void idle();

  static void update() {
    for (size_t i = 0; i < NUM_SERIAL; ++i) {
      if (port[i].active) {
        port[i].receive();
      }
    }
  }
  static void enable(size_t port_id) {
    port[port_id].active = true;
  }
  static bool enabled(size_t port_id) {
    return port[port_id].active;
  }
  static void disable(size_t port_id) {
    port[port_id].active = false;
  }

  static const uint16_t PACKET_MAX_WAIT = 500, RX_TIMESLICE = 20, MAX_RETRIES = 0, VERSION_MAJOR = 0, VERSION_MINOR = 2, VERSION_PATCH = 0, RX_STREAM_BUFFER_SIZE = BINARY_STREAM_RX_BUFFER_SIZE;
  static uint8_t next_serial_device_id;
  const uint8_t serial_device_id;

  uint16_t buffer_next_index;
  char rx_buffer[RX_STREAM_BUFFER_SIZE];
  bool active;

  static BinaryStream port[NUM_SERIAL];
};

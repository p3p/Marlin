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
#define BINARY_STREAM_TX_BUFFER_SIZE 128

#include "../../inc/MarlinConfig.h"
#include "../../sd/cardreader.h"

#include <new>

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

#define CRC_USE_TABLES
struct Checksum {
  // fletchers 16 checksum
  [[gnu::optimize("O3"), gnu::always_inline ]] static constexpr uint16_t fletchers16(uint16_t cs, const uint8_t * data, size_t cnt) {
    while (cnt--) {
      uint16_t cs_low = (((cs & 0xFF) + *data++) % 255);
      cs = ((((cs >> 8) + cs_low) % 255) << 8)  | cs_low;
    }
    return cs;
  }

  #ifdef CRC_USE_TABLES
    // poly 0x31
    static constexpr uint8_t crc_tab8[256]{ 0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97, 0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E, 0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4, 0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D, 0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11, 0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8, 0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52, 0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB, 0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA, 0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13, 0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9, 0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50, 0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C, 0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95, 0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F, 0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6, 0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED, 0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54, 0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE, 0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17, 0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B, 0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2, 0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28, 0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91, 0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0, 0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69, 0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93, 0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A, 0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56, 0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF, 0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15, 0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC };
    [[gnu::optimize("O3"), gnu::always_inline ]] static constexpr uint8_t crc8(uint8_t crc, const uint8_t * data, size_t cnt) {
      while (cnt--) {
        crc = crc_tab8[crc ^ *data++];
      }
      return crc;
    }
    // ccitt poly : 0x1021
    static constexpr uint16_t crc_tab16[256]{ 0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF, 0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE, 0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D, 0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4, 0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC, 0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823, 0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B, 0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A, 0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49, 0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70, 0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78, 0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067, 0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256, 0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634, 0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3, 0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92, 0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1, 0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0 };
    [[gnu::optimize("O3"), gnu::always_inline ]] static constexpr uint16_t crc16(uint16_t crc, const uint8_t * data, size_t cnt) {
      while (cnt--) {
        crc = (crc << 8) ^ crc_tab16[ ((crc >> 8) ^ *data++) & 0x00FF ];
      }
      return crc;
    }
  #else
    // poly 0x31
    [[gnu::optimize("O3"), gnu::always_inline ]] static constexpr uint8_t crc8(uint8_t crc, const uint8_t * data, size_t cnt) {
      while (cnt--) {
        crc = crc ^ *data++;
        for (uint8_t i = 0; i < 8; i++)
          crc = (uint8_t)((crc & 0x80) ? ((crc << 1) ^ 0x31) : (crc << 1));
      }
      return crc;
    }

    // ccitt poly : 0x1021
    [[gnu::optimize("O3"), gnu::always_inline ]] static constexpr uint16_t crc16(uint16_t crc, const uint8_t * data, size_t cnt) {
      while (cnt--) {
        crc = (uint16_t)(crc ^ (uint16_t)(((uint16_t)*data++) << 8));
        for (uint8_t i = 0; i < 8; i++)
          crc = (uint16_t)((crc & 0x8000) ? ((uint16_t)(crc << 1) ^ 0x1021) : (crc << 1));
      }
      return crc;
    }
  #endif
};

struct BinaryStreamControl {
  struct Packet {
    enum Type : uint8_t { ACK, NACK, NYET, REJECT, ERROR, SYNC, QUERY, CLOSE, RESET_MCU};
    struct [[gnu::packed]] Error {
      uint8_t sync, code;
    };
    struct [[gnu::packed]] Sync {
      uint16_t version_major, version_minor, version_patch, serial_buffer_size, payload_buffer_size;
    };
  };
};

class BinaryStream {
public:
  enum class Protocol : uint8_t { CONTROL, FILE_TRANSFER };
  enum ReceiveState : uint8_t { PACKET_RESET, PACKET_WAIT, PACKET_HEADER, PACKET_DATA, PACKET_FOOTER,
                                     PACKET_PROCESS, PACKET_RESEND, PACKET_RESPONSE, PACKET_TIMEOUT, PACKET_ERROR };
  enum TransmitState : uint8_t { IDLE, BUSY, WAITING, RETRY, COMPLETE, ERROR };

  struct PacketInfo {
    uint8_t type = 0;
    uint8_t protocol_id = 0;
    uint8_t packet_id = 0;
    char* payload = nullptr;
    uint16_t payload_length = 0;
    uint8_t response = 0;
    uint8_t status = 0;
    PacketInfo* next = nullptr;
  };
  PacketInfo *tx_active = nullptr;

  template <typename T>
  struct PacketPacker : public PacketInfo {
      PacketPacker(uint8_t type, uint8_t protocol_id, uint8_t packet_id, T _payload_obj)
        : PacketInfo{type, protocol_id, packet_id, reinterpret_cast<char*>(&payload_obj), sizeof(T), 0, 0}, payload_obj{_payload_obj} {
          static_assert(sizeof(PacketPacker<T>) <= 128, "buffer overrun"); // todo: generalise : currently sdfile protocol tx buffer size
        }
      T payload_obj;
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
  } tx_queue; //todo: should be part of the protocol?

  struct Packet { // 10 byte protocol overhead, ascii with checksum and line number has a minimum of 7 increasing with line
    enum Type { RESPONSE, DATA, DATA_NAK, DATA_FAF };
    struct Header {
      static constexpr uint16_t HEADER_TOKEN = 0xACB5; // transmitted little endian [B5,AC]
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
      uint8_t packet_type() { return (token >> 8) & 0x03; }
      void reset() { token = 0; sync = 0; protocol_id = 0; packet_id = 0; size = 0; checksum = 0; }
    };

    union Footer {
      struct [[gnu::packed]] {
        uint16_t checksum; // full packet checksum
      };
      uint8_t data[2];
      void reset() { checksum = 0; }
    };

    struct Response { // 5 byte minimal response packet
      union {
        struct [[gnu::packed]] {
          uint16_t token;
          uint8_t response;
          uint8_t sync_id;
          uint8_t checksum;
        };
        char data[5];
      };
    };

    union {
      Header header;
      Response response;
    };
    char* buffer;
    Footer footer;

    uint32_t bytes_received;
    uint16_t checksum;
    millis_t timeout;
    uint16_t packet_type;

    void reset() {
      header.reset();
      footer.reset();
      bytes_received = 0;
      checksum = 0;
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
  void write_packet(Packet& packet);
  bool idle();


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

  static const uint16_t PACKET_MAX_WAIT = 500, RX_TIMESLICE = 20, MAX_RETRIES = 0, VERSION_MAJOR = 0, VERSION_MINOR = 2, VERSION_PATCH = 0,
                        RX_STREAM_BUFFER_SIZE = BINARY_STREAM_RX_BUFFER_SIZE, TX_STREAM_BUFFER_SIZE = BINARY_STREAM_TX_BUFFER_SIZE;
  static uint8_t next_serial_device_id;
  const uint8_t serial_device_id;

  char rx_buffer[RX_STREAM_BUFFER_SIZE];
  [[gnu::aligned(4)]] char tx_buffer[TX_STREAM_BUFFER_SIZE]; // requires alignment for placement new not to cause a hard fault
  bool active;

  static BinaryStream port[NUM_SERIAL];
};

/**
 *

struct SerialConnection {
  static constexpr uint8_t max_protocols = 8;

  struct Protocol {
    virtual bool ready(SerialConnection* connection) = 0;
    virtual void idle(SerialConnection* connection) = 0;
    virtual void receive(SerialConnection* connection) = 0;
    virtual char* name();

    // tx_queue_get
               _available
  };

  struct ProtocolDescriptor {
    uint8_t channel = 0;
    Protocol* protocol = nullptr;
  };

  struct ProtocolList {
    ProtocolDescriptor protocol_list[max_protocols];
    uint8_t protocol_map[max_protocols];
    uint8_t next_index = 0;
    bool registerProtocol(uint8_t channel, Protocol* protocol) {
      if (next_index < max_protocols) {
        protocol_map[next_index] = channel;
        protocol_list[next_index++] = ProtocolDescriptor{channel, protocol};
      } else {
        return false;
      }
    }
  } protocol_list{};

  void update(uint8_t protocol_id) {
    Protocol* protocol = protocol_list.protocol_list[protocol_list.protocol_map[protocol_id]].protocol;
    if (protocol->ready(this)) {
      protocol->idle(this);
    }
  }

  void receive(uint8_t protocol_id) {
    protocol_list.protocol_list[protocol_list.protocol_map[protocol_id]].protocol->receive(this);
  }

  BinaryStream data_stream{};

};

*/
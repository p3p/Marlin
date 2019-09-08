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

#include "../../inc/MarlinConfig.h"
#include "transport_layer.h"

#define BINARY_STREAM_COMPRESSION
#if ENABLED(BINARY_STREAM_COMPRESSION)
  #include "../../libs/heatshrink/heatshrink_decoder.h"
  static heatshrink_decoder hsd;
  static uint8_t decode_buffer[512] = {};
#endif
#include "../../sd/cardreader.h"

class SDFileTransferProtocol  {
private:
  struct Packet {
    struct [[gnu::packed]] Open {
      static bool validate(char* buffer, size_t length) {
        return (length > sizeof(Open) && buffer[length - 1] == '\0');
      }
      static Open& decode(char* buffer) {
        data = &buffer[2];
        return *reinterpret_cast<Open*>(buffer);
      }
      bool compression_enabled() { return compression & 0x1; }
      bool dummy_transfer() { return dummy & 0x1; }
      static char* filename() { return data; }
      private:
        uint8_t dummy, compression;
        static char* data;  // variable length strings complicate things
    };
    struct [[gnu::packed]] QueryResponse {
      uint16_t version_major, version_minor, version_patch;
      uint8_t compression_type, window, lookahead; 
    };
    struct [[gnu::packed]] ActionResponse {
      enum class Response : uint8_t { SUCCESS, BUSY, FAIL, IOERROR, INVALID };
      Response response;
    };
  };
  
  static bool file_open(char* filename) {
    if (!dummy_transfer) {
      card.mount();
      card.openFile(filename, false);
      if (!card.isFileOpen()) return false;
    }
    transfer_active = true;
    data_waiting = 0;
    #if ENABLED(BINARY_STREAM_COMPRESSION)
      heatshrink_decoder_reset(&hsd);
    #endif
    return true;
  }

  static bool file_write(char* buffer, const size_t length) {
    #if ENABLED(BINARY_STREAM_COMPRESSION)
      if (compression) {
        size_t total_processed = 0, processed_count = 0;
        HSD_poll_res presult;

        while (total_processed < length) {
          heatshrink_decoder_sink(&hsd, reinterpret_cast<uint8_t*>(&buffer[total_processed]), length - total_processed, &processed_count);
          total_processed += processed_count;
          do {
            presult = heatshrink_decoder_poll(&hsd, &decode_buffer[data_waiting], sizeof(decode_buffer) - data_waiting, &processed_count);
            data_waiting += processed_count;
            if (data_waiting == sizeof(decode_buffer)) {
              if (!dummy_transfer)
                if (card.write(decode_buffer, data_waiting) < 0) {
                  return false;
                }
              data_waiting = 0;
            }
          } while (presult == HSDR_POLL_MORE);
        }
        return true;
      }
    #endif
    return (dummy_transfer || card.write(buffer, length) >= 0);
  }

  static bool file_close() {
    if (!dummy_transfer) {
      #if ENABLED(BINARY_STREAM_COMPRESSION)
        // flush any buffered data
        if (data_waiting) {
          if (card.write(decode_buffer, data_waiting) < 0) return false;
          data_waiting = 0;
        }
      #endif
      card.closefile();
      card.release();
    }
    #if ENABLED(BINARY_STREAM_COMPRESSION)
      heatshrink_decoder_finish(&hsd);
    #endif
    transfer_active = false;
    return true;
  }

  static void transfer_abort() {
    if (!dummy_transfer) {
      card.closefile();
      card.removeFile(card.filename);
      card.release();
      #if ENABLED(BINARY_STREAM_COMPRESSION)
        heatshrink_decoder_finish(&hsd);
      #endif
    }
    transfer_active = false;
    return;
  }

  enum class FileTransfer : uint8_t { QUERY, ACTION, ACTION_RESPONSE, OPEN, CLOSE, WRITE, ABORT };

  static size_t data_waiting, transfer_timeout, idle_timeout;
  static bool transfer_active, dummy_transfer, compression;

  static Packet::ActionResponse response_data;
  static Packet::QueryResponse query_data;
  static BinaryStream::PacketInfo tx_packet;

public:

  static void idle() {
    // If a transfer is interrupted and a file is left open, abort it after TIMEOUT ms
    const millis_t ms = millis();
    if (transfer_active && ELAPSED(ms, idle_timeout)) {
      idle_timeout = ms + IDLE_PERIOD;
      if (ELAPSED(ms, transfer_timeout)) transfer_abort();
    }
  }

  static void transmit_complete(uint8_t sync, uint8_t response) {
    SERIAL_ECHOLNPAIR("PFT packet: ", sync, " got response: ", response);
    // react to failed packet transfers?
  }

  
  static void transmit_response(BinaryStream *protocol, const Packet::ActionResponse::Response response_type) {
    response_data.response = response_type;
    tx_packet.set(BinaryStream::Packet::DATA, (uint8_t)BinaryStream::Protocol::FILE_TRANSFER, (uint8_t)FileTransfer::ACTION_RESPONSE, (char*)&response_data, sizeof(response_data));
    protocol->send_packet(&tx_packet);
  }

  // is the protocol ready to receive this packet without blocking
  static bool process_ready(uint8_t packet_type, uint16_t length) {
    return true;
  }

  static void process(BinaryStream *protocol, uint8_t packet_type, char* buffer, const uint16_t length) {
    using Response = Packet::ActionResponse::Response;
    transfer_timeout = millis() + TIMEOUT;
    switch (static_cast<FileTransfer>(packet_type)) {
      case FileTransfer::QUERY: {
        #if ENABLED(BINARY_STREAM_COMPRESSION)
            query_data = {VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, 1, HEATSHRINK_STATIC_WINDOW_BITS, HEATSHRINK_STATIC_LOOKAHEAD_BITS};
        #else
            query_data = {VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, 0, 0, 0};
        #endif
        tx_packet.set(BinaryStream::Packet::DATA, (uint8_t)BinaryStream::Protocol::FILE_TRANSFER, (uint8_t)FileTransfer::QUERY, (char*)&query_data, sizeof(Packet::QueryResponse));
        protocol->send_packet(&tx_packet);
        break;
      }
      case FileTransfer::OPEN:
        if (transfer_active) {
          transmit_response(protocol, Response::BUSY);
        } else {
          if (Packet::Open::validate(buffer, length)) {
            auto packet = Packet::Open::decode(buffer);
            compression = packet.compression_enabled();
            dummy_transfer = packet.dummy_transfer();
            if (file_open(packet.filename())) {
              transmit_response(protocol, Response::SUCCESS);
              break;
            }
          }
          transmit_response(protocol, Response::FAIL);
        }
        break;
      case FileTransfer::CLOSE:
        if (transfer_active) {
          if (file_close())
            transmit_response(protocol, Response::SUCCESS);
          else
            transmit_response(protocol, Response::IOERROR);
        }
        else
          transmit_response(protocol, Response::INVALID);
        break;
      case FileTransfer::WRITE:
        if (!transfer_active)
          transmit_response(protocol,Response::INVALID);
        else if (!file_write(buffer, length))
          transmit_response(protocol, Response::IOERROR);
        break;
      case FileTransfer::ABORT:
        transfer_abort();
        transmit_response(protocol, Response::SUCCESS);
        break;
      default:
        transmit_response(protocol, Response::INVALID);
        break;
    }
  }

  static const uint16_t VERSION_MAJOR = 0, VERSION_MINOR = 1, VERSION_PATCH = 0, TIMEOUT = 10000, IDLE_PERIOD = 1000;
};
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

const uint16_t SDFileTransferProtocol_BLOCK_SIZE = 64;

#define sd_busy() (IS_SD_PRINTING() || IS_SD_FILE_OPEN())

class SDFileTransferProtocol  {
private:
  struct Packet {

    struct [[gnu::packed]] Open {
      static bool validate(char* buffer, size_t length) {
        return (length > sizeof(Open) && buffer[length - 1] == '\0');
      }
      static Open& decode(char* buffer) {
        data = &buffer[sizeof(Open)];
        return *reinterpret_cast<Open*>(buffer);
      }
      bool compression_enabled() { return compression & 0x1; }
      bool dummy_transfer() { return dummy & 0x1; }
      static char* filename() { return data; }
      uint8_t dummy = 0, compression = 0;
      static char* data;
    };

    struct [[gnu::packed]] File {
      enum Meta : uint8_t {FOLDER, FILE, EOL};
      static bool validate(char* buffer, size_t length) {
        return (length > sizeof(File) && buffer[length - 1] == '\0');
      }
      static File& decode(char* buffer) {
        data = &buffer[sizeof(File)];
        return *reinterpret_cast<File*>(buffer);
      }
      uint8_t file_id() { return index; }
      uint8_t meta() { return meta_data; }
      static char* filename() { return data; }
      uint8_t index = 0, meta_data = 0;
      uint32_t size;
      static char* data;
    };

    struct [[gnu::packed]] FileCmd {
      enum FileCmdType : uint8_t {CD, DELETE, MKDIR, RMDIR};
      static bool validate(char* buffer, size_t length) {
        return (length > sizeof(FileCmd) && buffer[length - 1] == '\0');
      }
      static FileCmd& decode(char* buffer) {
        data = &buffer[sizeof(FileCmd)];
        return *reinterpret_cast<FileCmd*>(buffer);
      }
      FileCmdType command;
      static char* filename() { return data; }
      static char* data;
    };

    struct [[gnu::packed]] QueryResponse {
      uint16_t version_major, version_minor, version_patch;
      uint8_t compression_type, window, lookahead;
    };

    struct [[gnu::packed]] ActionResponse {
      enum class Response : uint8_t { SUCCESS, BUSY, FAIL, IOERROR, INVALID };
      Response response;
    };

    template <size_t S>
    struct [[gnu::packed]] Data {
      uint8_t data[S];
    };
  };

  static bool file_open(char* filename, bool read = false) {
    if (!dummy_transfer) {
      if (!file.open(&root, filename, read ? O_READ : O_CREAT | O_WRITE | O_TRUNC)) {
        return false;
      }
    }
    data_waiting = 0;
    data_transfered = 0;
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
                if (file.write(decode_buffer, data_waiting) < 0) {
                  return false;
                }
              data_waiting = 0;
            }
          } while (presult == HSDR_POLL_MORE);
        }
        return true;
      }
    #endif
    return (dummy_transfer || file.write(buffer, length) >= 0);
  }

  //todo: support outbound comprssion?
  static int16_t file_read(char* buffer, const size_t length) {
    return file.read(buffer, length);
  }

  static bool file_write_flush() {
    #if ENABLED(BINARY_STREAM_COMPRESSION)
      // flush any buffered data
      if (data_waiting) {
        if (file.write(decode_buffer, data_waiting) < 0) return false;
        data_waiting = 0;
      }
      heatshrink_decoder_finish(&hsd);
    #endif
    return true;
  }

  static bool file_close() {
    if (!dummy_transfer) {
      file_write_flush();
      if(!file.close()) return false;
    }
    return true;
  }

  static void transfer_abort() {
    if (!dummy_transfer) {
      if (protocol_state == RX_TRANSFER_ACTIVE) {
        char dos_name[13];
        file.getDosName(dos_name);
        file.close();
        card.removeFile(dos_name);
        protocol_state = IDLE;
      } else file.close();
      #if ENABLED(BINARY_STREAM_COMPRESSION)
        heatshrink_decoder_finish(&hsd);
      #endif
    }
    return;
  }

  enum FileTransfer : uint8_t { QUERY, ACTION, ACTION_RESPONSE, OPEN, CLOSE, WRITE, ABORT, REQUEST, LIST, CD, PWD, FILE, MOUNT, UNMOUNT };
  enum ProtocolState : uint8_t { IDLE, RX_TRANSFER_ACTIVE, TX_WAIT, TX_TRANSFER_SEND, TX_TRANSFER_FINISH, TX_LS_NEXT };

  static size_t data_waiting, data_transfered, transfer_timeout;
  static uint8_t protocol_state, queued_state;
  static bool dummy_transfer, compression;
  static char tx_buffer[SDFileTransferProtocol_BLOCK_SIZE + 16];
  static SdFile file, root;

public:

  static bool idle(BinaryStream *protocol) {
    static BinaryStream::PacketInfo* active_packet = nullptr;
    switch(protocol_state) {
      case IDLE:
        break;
      case RX_TRANSFER_ACTIVE:
        // If a transfer is interrupted and a file is left open, abort it after TIMEOUT ms
        if (ELAPSED(millis(), transfer_timeout)) {
          transfer_abort();
          protocol_state = IDLE;
        }
        break;
      case TX_TRANSFER_SEND: {
        active_packet = new (tx_buffer) BinaryStream::PacketPacker{BinaryStream::Packet::DATA, (uint8_t)BinaryStream::Protocol::FILE_TRANSFER, (uint8_t)FileTransfer::WRITE, Packet::Data<SDFileTransferProtocol_BLOCK_SIZE>{}};
        int16_t data_read = file_read(active_packet->payload, SDFileTransferProtocol_BLOCK_SIZE);
        if (data_read >= 0) {
          data_transfered += active_packet->payload_length = data_read;
          protocol->send_packet(active_packet);
          queued_state = data_read != SDFileTransferProtocol_BLOCK_SIZE ? TX_TRANSFER_FINISH : TX_TRANSFER_SEND;
          protocol_state = TX_WAIT;
        } else {
          abort();
          protocol_state = IDLE;
        }
        break;
      }
      case TX_TRANSFER_FINISH:
        if (active_packet->status == BinaryStream::TransmitState::COMPLETE) {
          file.close();
          protocol_state = IDLE;
        }
        break;
      case TX_LS_NEXT: {
        auto packet = new (tx_buffer) BinaryStream::PacketPacker{ BinaryStream::Packet::DATA, (uint8_t)BinaryStream::Protocol::FILE_TRANSFER, (uint8_t)FileTransfer::FILE, Packet::File{} };
        packet->payload_obj.data = (char *)&packet->payload_obj + sizeof(Packet::File);
        packet->payload_obj.data[0] = '\0';
        dir_t p;

        if(root.readDir(&p, packet->payload_obj.data) > 0) {
          size_t namelen = strlen(packet->payload_obj.data);

          if (namelen & 0/*LFNSUPPORT*/) {
            packet->payload_length += namelen; // todo: long filename support is fubar, creating a file is not lfn compatible?
          } else {
            root.dirName(p, packet->payload_obj.data);
            packet->payload_length += strlen(packet->payload_obj.data);
          }
          packet->payload_obj.meta_data = !DIR_IS_SUBDIR(&p);
          packet->payload_obj.size = p.fileSize;
          protocol->send_packet(packet);
          active_packet = packet;
          protocol_state = TX_WAIT;
          queued_state = TX_LS_NEXT;
        } else {
          packet->payload_obj.meta_data = Packet::File::EOL;
          protocol->send_packet(packet);
          active_packet = packet;
          protocol_state = TX_WAIT;
        }
        break;
      }
      case TX_WAIT:
        if (active_packet->status == BinaryStream::TransmitState::COMPLETE) {
          protocol_state = queued_state;
          queued_state = IDLE;
        }
        break;
    }

    return protocol_state != IDLE;
  }

  static void transmit_response(BinaryStream *protocol, const Packet::ActionResponse::Response response_type) {
    protocol->send_packet(new (tx_buffer) BinaryStream::PacketPacker{BinaryStream::Packet::DATA, (uint8_t)BinaryStream::Protocol::FILE_TRANSFER, (uint8_t)FileTransfer::ACTION_RESPONSE, Packet::ActionResponse{response_type}});
  }

  // is the protocol ready to receive this packet without blocking
  static bool process_ready(uint8_t packet_type, uint16_t length) {
    return true;
  }

  static void process(BinaryStream *protocol, uint8_t packet_type, char* buffer, const uint16_t length) {
    /*
      DELETE
      MKDIR
      RMDIR

      GET
      PUT
    */


    using Response = Packet::ActionResponse::Response;
    transfer_timeout = millis() + TIMEOUT;
    switch (static_cast<FileTransfer>(packet_type)) {
      case FileTransfer::QUERY: {
        protocol->send_packet(new (tx_buffer) BinaryStream::PacketPacker{BinaryStream::Packet::DATA, (uint8_t)BinaryStream::Protocol::FILE_TRANSFER, (uint8_t)FileTransfer::QUERY,
        #if ENABLED(BINARY_STREAM_COMPRESSION)
            Packet::QueryResponse{VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, 1, HEATSHRINK_STATIC_WINDOW_BITS, HEATSHRINK_STATIC_LOOKAHEAD_BITS}
        #else
            Packet::QueryResponse{VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, 0, 0, 0}
        #endif
        });
        break;
      }
      case FileTransfer::MOUNT:
        if (file.isOpen()) {
          transmit_response(protocol, Response::BUSY);
          break;
        }
        card.mount();
        if (card.isMounted()) {
          root = card.getroot();
          root.rewind();
          transmit_response(protocol, Response::SUCCESS);
        } else
          transmit_response(protocol, Response::IOERROR);
        break;
      case FileTransfer::UNMOUNT:
        if (file.isOpen() || sd_busy()) {
          transmit_response(protocol, Response::BUSY);
          break;
        }
        card.release();
        if (card.isMounted())
          transmit_response(protocol, Response::IOERROR);
        else
          transmit_response(protocol, Response::SUCCESS);
        break;
      case FileTransfer::LIST: {
        if (protocol_state != IDLE || sd_busy()) {
          transmit_response(protocol, Response::BUSY);
          break;
        }
        root.rewind();
        protocol_state = TX_LS_NEXT;
        break;
      }
      case FileTransfer::CD:
        if (sd_busy()) {
          transmit_response(protocol, Response::BUSY);
          break;
        }
        if (Packet::FileCmd::validate(buffer, length)) {
          auto packet = Packet::FileCmd::decode(buffer);
          if(!strcmp(packet.filename(), "/"))
            root = card.getroot();
          else {
            SdFile dir;
            if (dir.open(&root, packet.filename(), O_READ)) {
              root = dir;
            } else {
              transmit_response(protocol, Response::IOERROR);
              break;
            }
          }
          transmit_response(protocol, Response::SUCCESS);
        }
        break;
      case FileTransfer::PWD: {
        if (sd_busy()) {
          transmit_response(protocol, Response::BUSY);
          break;
        }
        // auto packet = new (tx_buffer) BinaryStream::PacketPacker{ BinaryStream::Packet::DATA, (uint8_t)BinaryStream::Protocol::FILE_TRANSFER, (uint8_t)FileTransfer::FILE, Packet::File{} };
        // packet->payload_obj.data = (char *)&packet->payload_obj + sizeof(Packet::File);
        // packet->payload_obj.meta_data = Packet::File::FOLDER;
        // char* workdir_filename = packet->payload_obj.data;
        // card.getAbsWorkDirName(workdir_filename);
        // packet->payload_length += strlen(workdir_filename);
        // protocol->send_packet(packet);
        transmit_response(protocol, Response::IOERROR);
        break;
      }

      case FileTransfer::OPEN:
        if (file.isOpen() || sd_busy()) {
          transmit_response(protocol, Response::BUSY);
          break;
        }
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
        break;
      case FileTransfer::REQUEST:
        if (protocol_state != IDLE || file.isOpen() || sd_busy()) {
          transmit_response(protocol, Response::BUSY);
          break;
        }
        if (Packet::Open::validate(buffer, length)) {
          auto packet = Packet::Open::decode(buffer);
          compression = packet.compression_enabled();
          dummy_transfer = packet.dummy_transfer();
          if (file_open(packet.filename(), true /*reading*/)) {
            transmit_response(protocol, Response::SUCCESS);
            protocol_state = TX_TRANSFER_SEND;
            break;
          }
        }
        transmit_response(protocol, Response::FAIL);
        break;
      case FileTransfer::CLOSE:
        if (file.isOpen()  && !sd_busy()) {
          if (file_close())
            transmit_response(protocol, Response::SUCCESS);
          else
            transmit_response(protocol, Response::IOERROR);
        }
        else
          transmit_response(protocol, Response::INVALID);
        break;
      case FileTransfer::WRITE:
        if (!file.isOpen() || sd_busy())
          transmit_response(protocol,Response::INVALID);
        else if (!file_write(buffer, length))
          transmit_response(protocol, Response::IOERROR);
        break;
      case FileTransfer::ABORT:
        if (!sd_busy()) {
          transfer_abort();
          transmit_response(protocol, Response::SUCCESS);
        } else {
          transmit_response(protocol, Response::BUSY);
        }
        break;
      default:
        transmit_response(protocol, Response::INVALID);
        break;
    }
  }

  static const uint16_t VERSION_MAJOR = 0, VERSION_MINOR = 1, VERSION_PATCH = 0, TIMEOUT = 10000, IDLE_PERIOD = 1000;
};
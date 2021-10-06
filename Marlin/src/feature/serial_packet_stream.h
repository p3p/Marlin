/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#include <stdint.h>
#include "../core/serial_hook.h"
#include "../core/macros.h"

#include <TransportLayer.h>
#include <Service.h>

template <typename T, std::size_t S> class RingBufferStream {
public:
  RingBufferStream() {index_read = index_write = 0;}

  std::size_t available() const {return mask(index_write - index_read);}

  std::size_t available_contiguous() const { return index_write < index_read ? buffer_size - index_read : available(); }
  T* get_contiguous_region(const std::size_t length) {
    if (available_contiguous() >= length) return &buffer[index_read];
    return nullptr;
  }

  std::size_t free() const {return size() - available();}
  bool empty() const {return index_read == index_write;}
  bool full() const {return next(index_write) == index_read;}
  void clear() {index_read = index_write = 0;}

  bool peek(T *const value) const {
    if (value == nullptr || empty()) return false;
    *value = buffer[index_read];
    return true;
  }

  inline std::size_t read(T* dst, std::size_t length) {
    length = std::min(length, available());
    const std::size_t length1 = std::min(length, buffer_size - index_read);
    memcpy(dst, (char*)buffer + index_read, length1);
    memcpy(dst + length1, (char*)buffer, length - length1);
    index_read = mask(index_read + length);
    return length;
  }

  inline std::size_t write(T* src, std::size_t length) {
    length = std::min(length, free());
    const std::size_t length1 = std::min(length, buffer_size - index_write);
    memcpy((char*)buffer + index_write, src, length1);
    memcpy((char*)buffer, src + length1, length - length1);
    index_write = mask(index_write + length);
    return length;
  }

  std::size_t read(T *const value) {
    if (value == nullptr || empty()) return 0;
    *value = buffer[index_read];
    index_read = next(index_read);
    return 1;
  }

  std::size_t write(const T value) {
    std::size_t next_head = next(index_write);
    if (next_head == index_read) return 0;     // buffer full
    buffer[index_write] = value;
    index_write = next_head;
    return 1;
  }

  constexpr std::size_t size() const {
    return buffer_size - 1;
  }

private:
  inline std::size_t mask(std::size_t val) const {
    return val & buffer_mask;
  }

  inline std::size_t next(std::size_t val) const {
    return mask(val + 1);
  }

  static const std::size_t buffer_size = S;
  static const std::size_t buffer_mask = buffer_size - 1;
  volatile T buffer[buffer_size];
  std::size_t index_write;
  std::size_t index_read;
};


class SerialStreamService : public SerialPacketStream::Service {
public:
  SerialStreamService() { }
  ~SerialStreamService() { };
  Status rx_ready(const std::size_t count) override final {
    if (rx_descriptor.inserted == 0) return Status::READY;
    status = Status::STALLED;
    return Status::NOT_YET;
  };
  ServiceRxBuffer* get_rx_buffer() override final { return &rx_descriptor; }
  void process_rx_async() override final {
    rx_stream.write(rx_descriptor.buffer, rx_descriptor.inserted);
    rx_descriptor.inserted = 0;
  }
  void process_rx_sync(const char * buffer, const std::size_t length) override final {}; // todo: we could use thi sinstead and remove the local rx_buffer
  void on_connect() override final { tl_connected = true; }
  void on_disconnect() override final { tl_connected = false; }
  bool tx_waiting() override final { return tx_ready; }
  ServiceTxBuffer* get_tx_buffer() override final { tx_ready = false; return &tx_descriptor; }

  void update() override final {
    if (tx_ready == false && tx_descriptor.status == ServiceTxBuffer::Status::INVALID && tl_connected == true) {
      if (tx_stream.available()) {
        tx_descriptor.length = tx_stream.read(tx_buffer, _MIN(tx_stream.available(), sizeof(tx_buffer))); // todo: rather than use an intermediary buffer, send the next contigious block in tx_stream
        tx_descriptor.frame_flags = SerialPacketStream::Flag::FLOW_CONTROL;
        tx_descriptor.data = tx_buffer;
        tx_ready = true;
      }
    } else if ((tx_descriptor.status == ServiceTxBuffer::Status::ACKNOWLEDGED && tx_descriptor.frame_flags & SerialPacketStream::Flag::FLOW_CONTROL) ||
                (tx_descriptor.status == ServiceTxBuffer::Status::COMPLETE && !(tx_descriptor.frame_flags & SerialPacketStream::Flag::FLOW_CONTROL))) {
      tx_descriptor.status = ServiceTxBuffer::Status::INVALID;
    } else if ( tx_descriptor.status == ServiceTxBuffer::Status::FAILED ) {
      tx_descriptor.status = ServiceTxBuffer::Status::INVALID;
    }
  }

  size_t write(const char c) {
    if (tl_connected) return tx_stream.write(c);
    return 0;
  }

  int16_t read() {
    char value;
    uint32_t ret = rx_stream.read(&value);
    return (ret ? value : -1);
  }

  size_t available() {
    return rx_stream.available();
  }

private:
  bool tl_connected = false;
  bool tx_ready = false;

  alignas(uintptr_t) char tx_buffer[512]{};
  alignas(uintptr_t) char rx_buffer[512]{};
  RingBufferStream<char, 1024> tx_stream;
  RingBufferStream<char, 1024> rx_stream;
  ServiceTxBuffer tx_descriptor{};
  ServiceRxBuffer rx_descriptor{ rx_buffer, sizeof(rx_buffer), 0 };
};

template <typename SerialT>
struct SerialPacketStreamSerial : public SerialBase <SerialPacketStreamSerial < SerialT >> {
  typedef SerialBase< SerialPacketStreamSerial<SerialT> > BaseClassT;
  SerialFeature features(serial_index_t index) const  { return SerialFeature::SerialPacketStream | CALL_IF_EXISTS(SerialFeature, &out, features, index);  }

  NO_INLINE void write(uint8_t c)     { serial_stream_service.write(c); }
  void flush()                        { out.flush();  }
  void begin(long br)                 { out.begin(br);}
  void end()                          { out.end(); }

  void msgDone()                      { out.msgDone(); }
  // Existing instances implement Arduino's operator bool, so use that if it's available
  bool connected()                    { return Private::HasMember_connected<SerialT>::value ? CALL_IF_EXISTS(bool, &out, connected) : (bool)out; }
  void flushTX()                      { CALL_IF_EXISTS(void, &out, flushTX); }

  int available(serial_index_t index) { return available(); }
  int read(serial_index_t index)      { return read(); }

  int available()                     { return serial_stream_service.available(); }
  int read()                          { return serial_stream_service.read(); }

  SerialPacketStreamSerial(const bool e, SerialT & out) : BaseClassT(e), out(out), transport_layer (
    [&out](){ return out.available(0); },
    [&out](char * buffer, const std::size_t length){ return out.readBytes(buffer, length); },
    [](){ return 2048; },
    [&out](const char * buffer, const std::size_t length){ return out.write(buffer, length); },
    [](){ },
    [](){ return millis();}
  ) {
    transport_layer.register_service(&serial_stream_service);
  }

  void process() {
    transport_layer.process();
    transport_layer.services_update();
  }

private:
  SerialT & out;
  SerialPacketStream::TransportLayer transport_layer;
  SerialStreamService serial_stream_service;
};

#pragma once

#include "../user_interface.h"

#include "SPISlavePeripheral.h"

/**
  * Instructions for create a FAT image:
  * 1) Install mtools
  * 2) Create the imagem file:
  *    $ mformat -v "EMBEDDED FS" -t 1 -h 1 -s 10000 -S 2 -C -i fs.img -c 1 -r 1 -L 1
  *    -s NUM is the number of sectors
  * 3) Copy files to the image:
  *    $ mcopy -i fs.img CFFFP_flow_calibrator.gcode ::/
  * 4) Set the path for SD_SIMULATOR_FAT_IMAGE
  */
 //#define SD_SIMULATOR_FAT_IMAGE "/full/path/to/fs.img"
 #ifndef SD_SIMULATOR_FAT_IMAGE
   #error "You need set SD_SIMULATOR_FAT_IMAGE with a path for a FAT filesystem image."
 #endif

class SDCard: public SPISlavePeripheral {
public:
  SDCard(pin_type clk, pin_type mosi, pin_type miso, pin_type cs, pin_type sd_detect = -1) : SPISlavePeripheral(clk, mosi, miso, cs), sd_detect(sd_detect) {}
  virtual ~SDCard() {};

  pin_type sd_detect;

  void update() {}
  void ui_callback(UiWindow* window);

  void onByteReceived(uint8_t _byte) override;
  void onEndTransaction() override {
    SPISlavePeripheral::onEndTransaction();
  };
  void onResponseSent() override;

  int8_t currentCommand = -1;
  int32_t arg = 0;
  int32_t byteCount = 0;
  uint8_t crc = 0;
  uint8_t buf[1024];
  FILE *fp = nullptr;
  uint16_t waitForBytes = 0;
  uint8_t commandWaitingForData = -1;
  uint32_t argWaitingForData = -1;
  uint16_t bufferIndex = 0;
};

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
   #warning "You need set SD_SIMULATOR_FAT_IMAGE with a path for a FAT filesystem image."
   #define SD_SIMULATOR_FAT_IMAGE "fs.img"
 #endif

class SDCard: public SPISlavePeripheral {
public:
  SDCard(pin_type clk, pin_type mosi, pin_type miso, pin_type cs, pin_type sd_detect = -1) : SPISlavePeripheral(clk, mosi, miso, cs), sd_detect(sd_detect) {}
  virtual ~SDCard() {};

  pin_type sd_detect;

  void update() {}
  void ui_callback(UiWindow* window);

  void onByteReceived(uint8_t _byte) override;
  void onRequestedDataReceived(uint8_t token, uint8_t* _data, size_t count) override;

  int32_t currentArg = 0;
  uint8_t buf[1024];
  FILE *fp = nullptr;
};

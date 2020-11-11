
#include "SDCard.h"
#include "../../../../sd/SdInfo.h"

void SDCard::onByteReceived(uint8_t _byte) {
  SPISlavePeripheral::onByteReceived(_byte);

  // 1 byte (cmd) + 4 byte (arg) + 1 byte (crc)
  // response: 0 success
  //
  if (currentCommand > -1) {
    //data
    if (++byteCount == 5) {
      crc = _byte;
      printf("CMD: %d, arg: %d, crc: %d, byteCount: %d\n", currentCommand, arg, crc, byteCount);
      switch (currentCommand) {
        case CMD0:
          setResponse(R1_IDLE_STATE);
          if (fp) fclose(fp);
          fp = fopen(SD_SIMULATOR_FAT_IMAGE, "rb+");
          break;
        case CMD8:
          if (true/*_type == SD_CARD_TYPE_SD1*/) {
            setResponse((R1_ILLEGAL_COMMAND | R1_IDLE_STATE));
          }
          else {
            memset(buf, 0xAA, 4);
            setResponse(buf, 4);
          }
          break;
        case CMD55:
          setResponse(R1_READY_STATE);
          break;
        case CMD58:
          buf[0] = R1_READY_STATE;
          memset(buf+1, 0xC0, 3);
          setResponse(buf, 4);
          break;
        case ACMD41:
          setResponse(R1_READY_STATE);
          break;
        case CMD17: //read block
          buf[0] = R1_READY_STATE;
          buf[1] = DATA_START_BLOCK;
          if (true  /*_type != SD_CARD_TYPE_SDHC*/) {
            arg >>= 9;
          }
          fseek(fp, 512 * arg, SEEK_SET);
          fread(buf + 2, 512, 1, fp);
          buf[512 + 2] = 0; //crc
          setResponse(buf, 512 + 3);
          break;
        case CMD24: //write block
          if (true  /*_type != SD_CARD_TYPE_SDHC*/) {
            arg >>= 9;
          }
          setResponse(R1_READY_STATE);
          waitForBytes = 512 + 1 + 2 + 1; //token + ff (from this response) + data + 2 crc
          commandWaitingForData = currentCommand;
          argWaitingForData = arg;
          bufferIndex = 0;
          break;
        case CMD13:
          setResponse(R1_READY_STATE);
          break;
        default:
          break;
      }
      // currentCommand = -1;
    }
    else if (byteCount <= 4) {
      arg <<= 8;
      arg |= _byte;
    }

    return;
  }
  else if (waitForBytes) {
    waitForBytes--;
    buf[bufferIndex++] = _byte;
    if (waitForBytes == 0) {
      switch (commandWaitingForData) {
      case CMD24:
        fseek(fp, 512 * argWaitingForData, SEEK_SET);
        fwrite(buf + 2, 512, 1, fp);
        fflush(fp);
        buf[0] = DATA_RES_ACCEPTED;
        buf[1] = 0xFF; // ack for finish write
        setResponse(buf, 2);
        break;
      default:
        break;
      }
      commandWaitingForData = -1;
      argWaitingForData = 0;
    }
    return;
  }
  // else if (_byte == 0) {
  //   setResponse(0xFF);
  //   return;
  // }

  currentCommand = _byte - 0x40;
}

void SDCard::onResponseSent() {
  SPISlavePeripheral::onResponseSent();
  currentCommand = -1;
  byteCount = 0;
  arg = 0;
}

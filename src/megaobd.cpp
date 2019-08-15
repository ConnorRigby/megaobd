#include "OBD9141sim.h"
#include <AltSoftSerial.h>
#include <CRC32.h>

// OBD constants
#define OBD_TX_PIN 9
#define OBD_RX_PIN 8
AltSoftSerial OBD_SERIAL;
OBD9141sim OBD;

// Megasquirt Constants
#define MEGASQUIRT_SERIAL Serial
const byte CMD_A_PAYLOAD[7] = {0x0, 0x1, 0x41, 0xD3, 0xD9, 0x9E, 0x8B};

enum STATUS_RECV_STATE {
  STATUS_RECV_SIZE_MSB,
  STATUS_RECV_SIZE_LSB,
  STATUS_RECV_STATUS,
  STATUS_RECV_DATA,
  STATUS_RECV_CRC32,
  STATUS_RECV_COMPLETE,
  STATUS_RECV_ERROR_CRC32
};

#define BUFFER_MAX_SIZE 0xFFFF

struct payload {
  STATUS_RECV_STATE state;
  uint16_t size;
  uint16_t read;
  uint32_t crc32;
  uint8_t status;
  uint8_t buffer[];
};

typedef enum status_field {
  STATUS_UINT8,
  STATUS_INT8,
  STATUS_UINT16,
  STATUS_INT16
} status_field_type_t;

struct status {
  uint16_t secs;
  uint16_t pw1;
  uint16_t pw2;
  uint16_t rpm;
  int16_t advance;
  // TODO(Connor) - Change this to a union
  uint8_t squirt;
  // TODO(Connor) - Change this to a union
  uint8_t engine;
  uint8_t afrtgt1raw;
  uint8_t afrtgt2raw;
  int16_t barometer;
  int16_t map;
  int16_t mat;
  int16_t clt;
  int16_t tps;
  uint16_t maf;
  uint16_t vss1;
};

struct payload msPayload;
struct status msStatus;

CRC32 CRC;

/** 
 * Writes the A command to the serial port.
 */
void writeMegasquirtPayload();

/**
 * Process data in the msPayload struct
 */
void processMegasquirtPayload();

void realTimeValue(void*, size_t, status_field_type_t);

void setup() {
  // Initialize OBD stuff
  OBD.begin(OBD_SERIAL, OBD_RX_PIN, OBD_TX_PIN);
  OBD.keep_init_state(true); 
  OBD.initialize();
  
  // Initialize MegaSquirt transport
  MEGASQUIRT_SERIAL.begin(115200);
  *msPayload.buffer = malloc(0xFFFF);
  writeMegasquirtPayload();
}

void loop() {
  // Process MegaSquirt data
  while(MEGASQUIRT_SERIAL.available()) {
    switch(msPayload.state) {
      case STATUS_RECV_SIZE_MSB:
        msPayload.buffer[0] = MEGASQUIRT_SERIAL.read();
        msPayload.state = STATUS_RECV_SIZE_LSB;
      break;

      case STATUS_RECV_SIZE_LSB:
        msPayload.size = (msPayload.buffer[0] << 8) | MEGASQUIRT_SERIAL.read();
        msPayload.state = STATUS_RECV_STATUS;
      break;

      case STATUS_RECV_STATUS:
        msPayload.status = MEGASQUIRT_SERIAL.read();
        msPayload.state = STATUS_RECV_DATA;
      break;

      case STATUS_RECV_DATA:
        msPayload.buffer[msPayload.read++] = MEGASQUIRT_SERIAL.read();
        // Data read complete
        if(msPayload.read == msPayload.size) {
          msPayload.state = STATUS_RECV_CRC32;
        }
      break;

      case STATUS_RECV_CRC32:
        msPayload.buffer[msPayload.read++] = MEGASQUIRT_SERIAL.read();
        // read 4 more bytes
        if(msPayload.read = (msPayload.size + 4)) {
          msPayload.crc32 = msPayload.buffer[msPayload.read-3] | (msPayload.buffer[msPayload.read-2] << 8) | (msPayload.buffer[msPayload.read-1] << 16) | (msPayload.buffer[msPayload.read] << 24);
          uint32_t calculated = CRC.calculate(msPayload.buffer, msPayload.size);
          // Check crc32
          if(msPayload.crc32 == calculated) {
            msPayload.state = STATUS_RECV_COMPLETE;
          } else {
            msPayload.state = STATUS_RECV_ERROR_CRC32;
          }
        }
      break;

      case STATUS_RECV_COMPLETE:
        processMegasquirtPayload();
      break;

      default:
        // Handle error here
      break;
    }
  }
  
  // Process OBD requests
  OBD.loop();
  
  /** 
  Miata ECU supports the following type 1 PIDS:
    0x00 - PIDs supported (01-20) [01, 03-07, 0C-11, 13-15, 1C, 20]
    0x01 - Monitor status since DTCs cleared
    0x03 - Fuel system status
    0x04 - Calculated engine load value
    0x05 - Engine coolant temperature
    0x06 - Short term fuel % trim - Bank 1
    0x07 - Long term fuel % trim - Bank 1
    0x0C - Engine RPM
    0x0D - Vehicle speed
    0x0E - Timing advance
    0x0F - Intake air temperature
    0x10 - MAF air flow rate
    0x11 - Throttle position
    0x13 - Oxygen sensors present
    0x14 - Bank 1, Sensor 1: O2S Voltage, Short term fuel trim
    0x15 - Bank 1, Sensor 2: O2S Voltage, Short term fuel trim
    0x1C - OBD standards this vehicle conforms to
    0x20 - PIDs supported (21-40)
    0x21 - Distance traveled with MIL on
  */
  // Helper vars
  uint8_t a;
  uint8_t b;

  // Supported PIDS: [01, 03-07, 0C-11, 13-15, 1C, 20]
  // 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E 1F 20
  // 1  0  1  1  1  1  1  0  0  0  0  1  1  1  1  1  1  0  1  1  1  0  0  0  0  0  0  1  0  0  0  1
  // 10111110000111111011100000010001
  OBD.setAnswer(0x01, 0x00, 32, 0b10111110000111111011100000010001);

  // Note that test availability is indicated by set (1) bit and completeness is indicated by reset (0) bit. 
  // ByteA: MIL + number of active codes
  // ByteB: Spark mode and tests available/complete
  // ByteC: CARB tests available
  // ByteD: CARB tests complete 
  OBD.setAnswer(0x01, 0x01, 32, 0b0000000000000111111111110000);

  // Fuel System status
  // Should be built from the `msStatus.squirt` 
  // OBD.setAnswer(0x01, 0x03, );

  // Engine status
  // Should be built from the `msStatus.engine`
  // OBD.setAnswer(0x01, 0x04, );

  // Coolant
  // This may need to be converted to C?
  OBD.setAnswer(0x01, 0x05, (uint8_t)(msStatus.clt + 40));

  // Short term fuel trim
  // OBD.setAnswer(0x01, 0x06, (uint8_t));

  // Long term fuel trim
  // OBD.setAnswer(0x01, 0x07, (uint8_t));

  // Engine RPM
  a = msStatus.rpm & 0xFF;
  b = msStatus.rpm >> 8;
  uint16_t rpm = (256 * a + b) / 4;
  OBD.setAnswer(0x01, 0x0C, (uint16_t)rpm);

  // VSS
  // OBD.setAnswer(0x01, 0x0D, (uint8_t) );

  // Advance
  uint8_t advance = (msStatus.advance / 2) - 64;
  OBD.setAnswer(0x01, 0x0E, (uint8_t)advance);

  // IAT (may need to convert to C?)
  OBD.setAnswer(0x01, 0x0F, (uint8_t)(msStatus.mat - 40));

  // MAF
  a = msStatus.maf & 0xFF;
  b = msStatus.maf >> 8;
  uint16_t maf = (256 * a + b) / 100;
  OBD.setAnswer(0x01, 0x10, (uint16_t)maf);

  // TPS
  uint8_t tps = (100/255) * msStatus.tps;
  OBD.setAnswer(0x01, 0x11, (uint8_t)tps);

  // Number of O2 sensors
  // These bytes might be backwards, but the result is the same?
  OBD.setAnswer(0x01, 0x13, (uint8_t)0b11001100);

  // O2 Sensors (2 is not used)
  // TODO 0x14 second byte should be the same as pid 0x06
  // TODO the first byte should be voltage of the sensor for both answers
  OBD.setAnswer(0x01, 0x14, (uint16_t)0x00FF);
  OBD.setAnswer(0x01, 0x15, (uint16_t)0x00FF);

  // OBD compliance
  // 0x01 = CARB
  OBD.setAnswer(0x01, 0x1C, (uint8_t)0x01);

  // PIDs 21-40
  // Miata only supports 21
  OBD.setAnswer(0x01, 0x20, 32, 0b10000000000000000000000000000000);

  OBD.setAnswer(0x01, 0x21, (uint16_t)0);

  // End loop with telling MegaSquirt we want another status.
  writeMegasquirtPayload();
}

void processMegasquirtPayload() {
  switch(msPayload.status) {
    // Realtime data
    case 0x01: 
      // This is a huge, ugly assign.
      
      // uint16_t msStatus.secs 
      realTimeValue(&msStatus.secs, 0, STATUS_UINT16);
      
      // uint16_t msStatus.pw1;
      realTimeValue(&msStatus.pw1, 2, STATUS_UINT16);

      // uint16_t msStatus.pw2;
      realTimeValue(&msStatus.pw2, 4, STATUS_UINT16);

      // uint16_t msStatus.rpm;
      realTimeValue(&msStatus.rpm, 6, STATUS_UINT16);

      // int16_t msStatus.advance;
      realTimeValue(&msStatus.advance, 8, STATUS_INT16);
      // uint8_t msStatus.squirt;

      realTimeValue(&msStatus.squirt, 10, STATUS_UINT8);
      // uint8_t msStatus.engine;
      realTimeValue(&msStatus.engine, 11, STATUS_UINT8);
      // uint8_t msStatus.afrtgt1raw;

      // uint8_t msStatus.afrtgt2raw;
      
      // int16_t msStatus.barometer;
      realTimeValue(&msStatus.barometer, 16, STATUS_INT16);
      // int16_t msStatus.map;
      realTimeValue(&msStatus.map, 18, STATUS_INT16);
      // int16_t msStatus.mat;
      realTimeValue(&msStatus.mat, 20, STATUS_INT16);
      // int16_t msStatus.clt;
      realTimeValue(&msStatus.clt, 22, STATUS_INT16);
      // int16_t msStatus.tps;
      realTimeValue(&msStatus.tps, 24, STATUS_INT16);
      // uint16_t maf;
      realTimeValue(&msStatus.maf, 210, STATUS_UINT16);
      // uint16_t vss1;
      realTimeValue(&msStatus.vss1, 336, STATUS_INT16);

    break;
    
    default:
      // Unknown status 
    break;
  }
}

void realTimeValue(void *valuePtr, size_t offset, status_field_type_t type) {
  switch(type) {
    case STATUS_UINT8: {
      uint8_t *fieldPtr = (uint8_t*)valuePtr;
      *fieldPtr = msPayload.buffer[offset];
      break;
    }

    case STATUS_INT8: {
      int8_t *fieldPtr = (int8_t*)valuePtr;
      *fieldPtr = msPayload.buffer[offset];
      break;
    }

    case STATUS_UINT16: {
      uint16_t *fieldPtr = (uint16_t*)valuePtr;
      *fieldPtr = (msPayload.buffer[offset] << 8) | msPayload.buffer[offset+1];
      break;
    }

    case STATUS_INT16: {
      uint16_t tmp = (msPayload.buffer[offset] << 8) | msPayload.buffer[offset+1];
      int16_t *fieldPtr = (int16_t*)valuePtr;
      *fieldPtr = (int16_t) tmp;
      break;
    }
  }
}

void writeMegasquirtPayload() {
  uint16_t i;
  for(i = 0; i < 7; i++) {
    MEGASQUIRT_SERIAL.write(CMD_A_PAYLOAD[i]);
  }

  // Reset state machine
  msPayload.state = STATUS_RECV_SIZE_MSB;
  msPayload.size = 0;
  msPayload.crc32 = 0;
  msPayload.read = 0;

  // Clear buffer
  for(i=0; i < BUFFER_MAX_SIZE; i++) {
    msPayload.buffer[i] = 0;
  }
}
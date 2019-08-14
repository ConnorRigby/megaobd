#include "OBD9141sim.h"
#include <AltSoftSerial.h>

// OBD constants
#define OBD_TX_PIN 9
#define OBD_RX_PIN 8
AltSoftSerial OBD_SERIAL;
OBD9141sim OBD;

// Megasquirt Constants
#define MEGASQUIRT_SERIAL Serial

void setup() {
  // Initialize OBD stuff
  OBD.begin(OBD_SERIAL, OBD_RX_PIN, OBD_TX_PIN);
  OBD.keep_init_state(true); 
  OBD.initialize();
  
  // Initialize MegaSquirt transport
  MEGASQUIRT_SERIAL.begin(115200);
}

void loop() {
  // Process MegaSquirt data
  while(MEGASQUIRT_SERIAL.available()) {
    
  }
  
  // Process OBD requests
  OBD.loop();
  
}

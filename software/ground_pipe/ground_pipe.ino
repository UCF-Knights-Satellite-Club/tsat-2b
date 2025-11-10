#include <RFM69.h>
#include <RFM69_ATC.h>

// -----[ Network Config ]-----

#define NODEID 1
#define NETWORKID 100
#define FREQUENCY RF69_915MHZ
#define ENCRYPTKEY "TSAT-2B/25"

// Auto Transmission Control
// Saves power
#define ENABLE_ATC

// -----[ Statics ]-----

#ifdef ENABLE_ATC
RFM69_ATC radio;
#else
RFM69 radio;
#endif

void setup() {
  Serial.begin(115200);

  Serial.println("Initializing radio!");

  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  radio.setHighPower(); // needed for RFM69HCW
  radio.encrypt(ENCRYPTKEY);
}

void loop() {
  if (radio.receiveDone()) {
    // Print out the packet
    for (int i = 0; i < radio.DATALEN; i++) {
      Serial.printf("%d ", radio.DATA[i]);
    }
    Serial.println();
  }
  delay(100);
}

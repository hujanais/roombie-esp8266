#include "roombie.h"

#define SERIAL_RX     5  // pin for SoftwareSerial RX
#define SERIAL_TX     4  // pin for SoftwareSerial TX
#define D3            0  // pin for D3 for baudrate reset.

Roombie roombie(SERIAL_RX, SERIAL_TX, D3); // RX, TX, Baudreset
String cmd;
SensorData pSensorData;
uint8_t rawBytes[128];
String space = " ";

void setup() {
  Serial.begin(115200);

  // put your setup code here, to run once:
  roombie.init();
}

void loop() {
  // do soem cleanup.
  memset(rawBytes, 0, sizeof(rawBytes));
  
  cmd = Serial.readStringUntil('\n');

  if (cmd == "") {
    // do nothing
  } else if (cmd.indexOf("READSENSOR") != -1) {
    unsigned int pktId, pktLen;
    sscanf(cmd.c_str(), "READSENSOR %d %d", &pktId, &pktLen);
    if (roombie.readSensor(pktId, pktLen, rawBytes) == true) {
      for (int i = 0; i < pktLen; i++) {
        Serial.print(rawBytes[i], HEX);
        Serial.print(space);
      }
      Serial.println();
    } else {
      Serial.println("readsensor timed-out");
    }
  } else if (cmd == "ALL") {
    roombie.readAllSensors(&pSensorData, rawBytes);
    for (int i = 0; i < 80; i++) {
      Serial.print(rawBytes[i], HEX);
      Serial.print(space);
    }
    Serial.println();
    Serial.print(pSensorData.isRightBump);
    Serial.print(space);
    Serial.print(pSensorData.isLeftBump);
    Serial.print(space);
    Serial.print(pSensorData.isWheelRightDrop);
    Serial.print(space);
    Serial.print(pSensorData.isWheelLeftDrop);
    Serial.print(space);
    Serial.print(pSensorData.chargingState);
    Serial.print(space);
    Serial.print(pSensorData.voltage);
    Serial.print(space);
    Serial.print(pSensorData.current);
    Serial.print(space);
    Serial.print(pSensorData.temperature);
    Serial.print(space);
    Serial.print(pSensorData.batteryCharge);
    Serial.print(space);
    Serial.print(pSensorData.batteryCapacity);
    Serial.println("----------------");
  } else if (cmd == "POWERDOWN") {
    roombie.powerDown133();
  } else if (cmd.indexOf("COMMAND") != -1) {
    cmd.replace("COMMAND ", ""); // remove the TEST keyword
    char cmdBuffer[256];
    cmd.toCharArray(cmdBuffer, 256);
    roombie.doCommand(cmdBuffer);
  } else {
    // command not found.
    Serial.println(cmd + " does not exist. ");
  }
}

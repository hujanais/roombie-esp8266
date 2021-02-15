#include "roombie.h"
#include "wss.h"

#define SERIAL_RX     5  // pin for SoftwareSerial RX
#define SERIAL_TX     4  // pin for SoftwareSerial TX
#define D3            0  // pin for D3 for baudrate reset.

// Roomba related variables
Roombie *roombie;
String cmd;
SensorData pSensorData;
uint8_t rawBytes[128];
String space = " ";

// Websocket related variables
WSS wss;
DynamicJsonDocument outgoingDoc(1024);
DynamicJsonDocument json(1024);
DynamicJsonDocument innerjson(1024);


unsigned long startTime = 0;
unsigned long responseTime = 0;
unsigned int refreshRate = 1000;
bool isAlive = false;

void setup() {
  Serial.begin(115200);

  // initialize electronics
  pinMode(D3, OUTPUT);
  digitalWrite(D3, HIGH);
  pinMode(SERIAL_RX, INPUT);
  pinMode(SERIAL_TX, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // initialize the wifi and websocket.
  wss.init(handleWebSocketEvent, onMessage);

  roombie = new Roombie(SERIAL_RX, SERIAL_TX, D3); // RX, TX, Baudreset)

  // initialize the roomba.
  roombie->init();

  roombie->startOI128();
}

/**
   The main program loop.  The websocket requires that this loop is running as fast as possible.
   Do not put in any methods that will block this loop.
*/
void loop() {
  wss.loop();

  // do some cleanup.
  memset(rawBytes, 0, sizeof(rawBytes));

  // run this operation once every XX seconds
  if (millis() - startTime >= refreshRate) {
    responseTime = millis();
    isAlive = roombie->readAllSensors(&pSensorData, outgoingDoc, rawBytes);
/*    Serial.println(millis() - responseTime);
    for (int i = 0; i < 80; i++) {
      Serial.print(rawBytes[i], HEX);
      Serial.print(space);
    } */

     startTime = millis();

    // add additional json data here.
    outgoingDoc["isAlive"] = isAlive;

    // send the data to the internet
    wss.sendMessage(outgoingDoc);

    /*    innerjson["level2"] = "hello";
        json["level1"] = innerjson;
        wss.sendMessage(json);
    */
  }
}

/*
   This is the websocket handler that handles the json data from the node server
   expected payload: {command: 'do-something'}
*/
void onMessage(DynamicJsonDocument jsonDoc) {
  DynamicJsonDocument outgoingDoc(512);
  if (jsonDoc["command"] == "WAKEUP") {
    roombie->wakeup();
  } else if (jsonDoc["command"] == "RESET") {
    roombie->factoryReset7();
  } else if (jsonDoc["command"] == "DOCK") {
    roombie->doDock();
  } else if (jsonDoc["command"] == "CLEAN") {
    roombie->doClean();
  } else if (jsonDoc["command"] == "FULL") {
    roombie->goFullMode132();
  } else if (jsonDoc["command"] == "PASSIVE") {
    roombie->goPassiveMode128();
  } else if (jsonDoc["command"] == "SAFE") {
    roombie->goSafeMode131();
  } else if (jsonDoc["command"] == "OFF") {
    roombie->powerDown133();
  } else {
    char cmdBuffer[256];
    jsonDoc["command"].as<String>().toCharArray(cmdBuffer, 256);
    Serial.println(cmdBuffer);
    roombie->doCommand(cmdBuffer);
  }
}

void handleWebSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  wss.handleWebSocketEvent(type, payload, length);
}

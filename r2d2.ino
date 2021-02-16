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
DynamicJsonDocument sensorDataDoc(1024);
DynamicJsonDocument json(1024);

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

  // Start the OI mode.
  roombie->startOI();
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
    startTime = millis();

    isAlive = roombie->readAllSensors(&pSensorData, sensorDataDoc, rawBytes);

    // add additional json data here.
    sensorDataDoc["isAlive"] = isAlive;

    // send the data to the internet
    json["data"] = sensorDataDoc;
    wss.sendMessage(sensorDataDoc);
  }
}

/*
   This is the websocket handler that handles the json data from the node server
   expected payload: {command: 'do-something'}
*/
void onMessage(DynamicJsonDocument jsonDoc) {
  String cmd = jsonDoc["command"].as<String>();

  if (cmd == "WAKEUP") {
    roombie->wakeup();
  } else if (cmd == "RESET") {
    roombie->factoryReset();
  } else if (cmd == "DOCK") {
    roombie->doDock();
  } else if (cmd == "CLEAN") {
    roombie->doClean();
  } else if (cmd == "FULL") {
    roombie->goFullMode();
  } else if (cmd == "PASSIVE") {
    roombie->goPassiveMode();
  } else if (cmd == "SAFE") {
    roombie->goSafeMode();
  } else if (cmd == "OFF") {
    roombie->powerDown();
  } else if (cmd.indexOf("DRIVE") != -1) {
    short velocity, radius;
    sscanf(cmd.c_str(), "DRIVE %d", &velocity, &radius);
    roombie->drive(velocity, 0);
  } else if (cmd.indexOf("PWM") != -1) {
    short mainBrush, sideBrush, vacuum;
    sscanf(cmd.c_str(), "PWM %d %d %d", &mainBrush, &sideBrush, &vacuum);
    roombie->pwmMotors(mainBrush, sideBrush, vacuum);
  } else if (cmd.indexOf("CMD") != -1) {
    char cmdBuffer[256];
    cmd.toCharArray(cmdBuffer, 256);
    roombie->doCommand(cmdBuffer);
  } else {
    Serial.println(cmd + " does not exist.");
  }
}

void handleWebSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  wss.handleWebSocketEvent(type, payload, length);
}

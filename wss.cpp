#include "Arduino.h"
#include "wss.h"
#include <ArduinoJson.h>

#define SSID "WARCHALKED"
#define PASSWORD "0D8478C1E7"

//char* host = "192.168.1.106";  //replace this ip address with the ip address of your Node.Js server
//const int wsport = 8888;

// char path[] = "/esp";   //identifier of this device

WSS::WSS()
{
  webSocket = new WebSocketsClient();
  Serial.println("ctor WSS");
}

void WSS::init(WebSocketClientEvent handler, OnMessage cb) {
  _onMessage = cb;

  WiFi.begin(SSID, PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  delay(1000);

  initWebSocket("192.168.1.106", 8888, "/esp", handler);
}

/**
   This must be called in the loop of the called.
*/
void WSS::loop() {
  webSocket->loop();
}

void WSS::initWebSocket(char* host, int wsPort, char wsPath[], WebSocketClientEvent handler) {
  // server address, port and URL
  webSocket->begin(host, wsPort, wsPath);

  // event handler
  webSocket->onEvent(handler);

  // use HTTP Basic Authorization this is optional remove if not needed
  // webSocket.setAuthorization("user", "Password");

  // try ever 5000 again if connection has failed
  webSocket->setReconnectInterval(5000);

  // start heartbeat (optional)
  // ping server every 15000 ms
  // expect pong from server within 3000 ms
  // consider connection disconnected if pong is not received 2 times
  webSocket->enableHeartbeat(15000, 3000, 2);
}

void WSS::sendMessage(DynamicJsonDocument jsonDoc) {
  String jsonStr = "";
  serializeJson(jsonDoc, jsonStr);
  webSocket->sendTXT(jsonStr);
}

void WSS::handleWebSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  DynamicJsonDocument incomingDoc(256);

  // convert the payload into a jsonstring.
  String jsonString = String((char *)payload);

  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("[WSc] Disconnected!\n");
      break;
    case WStype_CONNECTED: {
        Serial.println("WSS Connected");
      }
      break;
    case WStype_TEXT:
      // websocket incoming message is a jsonString.
      deserializeJson(incomingDoc, jsonString);
      _onMessage(incomingDoc);
      break;
    case WStype_BIN:
      //      Serial.println("[WSc] get binary length: %u\n", length);
      //      hexdump(payload, length);

      // send data to server
      // webSocket.sendBIN(payload, length);
      break;
    case WStype_PING:
      // pong will be send automatically so nothing need to be handled here.
      // Serial.println("[WSc] get ping\n");
      break;
    case WStype_PONG:
      // answer to a ping we send. nothing needed here as well.
      // Serial.println("[WSc] get pong\n");
      break;
  }
}

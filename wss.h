#ifndef wsseb5736d7-8f28-46d7-9e72-fa178b3641f8_h
#define wsseb5736d7-8f28-46d7-9e72-fa178b3641f8_h

#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <WebSocketsClient.h> // https://github.com/Links2004/arduinoWebSockets/tree/master/examples/esp8266 => this implements Ping/Pong
#include <ArduinoJson.h>

class WSS
{	
	public:
		WSS();
		
		typedef void (*WebSocketClientEvent)(WStype_t type, uint8_t * payload, size_t length);
		typedef void (*OnMessage)(DynamicJsonDocument jsonDoc);
		
		void init(WebSocketClientEvent handler, OnMessage cb);
		void loop();

		// cheap callback function.
		void handleWebSocketEvent(WStype_t type, uint8_t * payload, size_t length);
		
		// Send a new websocket message.
		void sendMessage(DynamicJsonDocument jsonDoc);
	
	private:
		String data = "";
		OnMessage _onMessage;

		WebSocketsClient *webSocket;

		void initWebSocket(char* host, int wsPort, char wsPath[], WebSocketClientEvent cb);
};

#endif

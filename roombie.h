#ifndef roombieeb5736d7-8f28-46d7-9e72-fa178b3641f8_h
#define roombieeb5736d7-8f28-46d7-9e72-fa178b3641f8_h

#include "Arduino.h"
#include <ArduinoJson.h>
#include <SoftwareSerial.h>

/// Read timeout in milliseconds.
/// If we have to wait more than this to read a char when we are expecting one, then something is wrong.
#define ROOMBA_READ_TIMEOUT 2000

#define INVALIDRESULT -9999

struct SensorData {
	bool isRightBump;				// 7
	bool isLeftBump;				// 7
	bool isWheelRightDrop;			// 7
	bool isWheelLeftDrop;			// 7
	bool isWall;					// 8
	bool isCliffLeft;				// 9
	bool isCliffFrontLeft;			// 10
	bool isCliffFrontRight;			// 11
	bool isCliffRight;				// 12
	bool isVirtualWall;				// 13
	bool isSideBrushOverCurrent;	// 14
	bool isMainBrushOverCurrent;	// 14
	bool isRightWheelOverCurrent;	// 14
	bool isLeftWheelOverCurrent;	// 14
	bool isDirtDetect;				// 15
	short irCharacterOmni;			// 17
	short buttons;					// 18	// needs work
	short distance;					// 19	// don't use
	short angle;					// 20	// don' use
	unsigned short chargingState;	// 21
	unsigned short voltage;			// 22
	short current;					// 23
	short temperature;				// 24
	unsigned short batteryCharge;	// 25
	unsigned short batteryCapacity;	// 26
	unsigned short wallSignal;		// 27
	unsigned short cliffLeftSignal;	// 28
	unsigned short cliffFrontLeftSignal;	// 29
	unsigned short cliffFrontRightSignal;	// 30
	unsigned short cliffRightSignal;	// 31
  unsigned short chargingSource; // 34
	unsigned short oiMode;			// 35
	unsigned short songNumber;		// 36
	unsigned short songPlaying;		// 37
	unsigned short numOfStreamPkts;	// 38
	short requestedVelocity;		// 39
	short requestedRadius;			// 40
	short requestedRightVelocity;	// 41
	short requestedLeftVelocity;	// 42
	short leftEncoderCounts;		// 43
	short rightEncoderCounts;		// 44
	bool isLeftLightBumper;			// 45
	bool isFrontLeftLightBumper;	// 45
	bool isCenterLeftLightBumper;	// 45
	bool isCenterRightLightBumper;	// 45
	bool isFrontRightLightBumper;	// 45
	bool isRightLightBumper;		// 45
	unsigned short leftLightBumper;			// 46
	unsigned short frontLeftLightBumper;		// 47
	unsigned short centerLeftLightBumper;	// 48
	unsigned short centerRightLightBumper;	// 49
	unsigned short frontRightLightBumper;	// 50
	unsigned short rightLightBumper;			// 51
	unsigned short irCharacterLeft; 	// 52
	unsigned short irCharacterRight; // 53
	short leftMotorCurrent;	// 54
	short rightMotorCurrent; // 55
	short mainBrushCurrent;	// 56
	short sideBrushCurrent;	// 57
	unsigned short stasis;	// 58 0-3
};

class Roombie
{
	public:
		Roombie(int rxPin, int txPin, int baudPin);
		void init();
		void wakeup();
		void startOI();
		void stopOI();
		void powerDown();
		void factoryReset();
		void goSafeMode();
		void goFullMode();
		void goPassiveMode();
		void doClean();
		void doMaxClean();
		void doSpotClean();
		void doDock();
		void doCommand(char* cmd);

    void doEMO();
    void pwmMotors(short mainBrushPercentage, short sideBrushPercentage, unsigned short vacuumPercentage);
    void drive(short velocityPercent, short radius);
    void turnCW(short velocity);
    void turnCCW(short velocity);
    
		bool readSensor(uint8_t packetId, uint8_t pktLen, uint8_t* buf);
		bool readAllSensors(struct SensorData *pSensorData, DynamicJsonDocument& outgoingDoc, uint8_t* buf);
        
	private:
		SoftwareSerial *pSerial;
		int _baudPin;
		
		void _setBaudRate();
		bool _readSerial(uint8_t* dest, uint8_t len);
		unsigned int _convertBytesToUInt(uint8_t* input, uint8_t numOfBytes);
		int _convertBytesToInt(uint8_t* input, uint8_t numOfBytes);
};

#endif

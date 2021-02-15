#include "Arduino.h"
#include "roombie.h"
/// Couldn't have done it without,
/// https://github.com/johnboiles/esp-roomba-mqtt and https://github.com/Mjrovai/Roomba_BT_Ctrl

#include <string.h>
#include <stdio.h>

Roombie::Roombie(int rxPin, int txPin, int baudPin)
{
  pSerial = new SoftwareSerial(rxPin, txPin);
  _baudPin = baudPin;
}

// change baudrate to 19200.
void Roombie::init()
{
  pSerial->begin(19200);
  pSerial->listen();

  delay(2000);  // wait 2 seconds before pulsing the baud pins
  _setBaudRate();
}

/*  After turning on Roomba, wait 2 seconds and then pulse the Baud Rate Changelow three times. from Roomba OpenInterface version 6 docs.
   Each pulse should last between 50 and 500 milliseconds
*/
void Roombie::_setBaudRate()
{
  Serial.println("SetBaudRate to 19200");
  digitalWrite(D3, HIGH);  // set D3 to 5V
  delay(2000);
  for (int i = 0; i < 3; i++) {
    digitalWrite(D3, LOW);
    delay(100);
    digitalWrite(D3, HIGH);
    delay(100);
  }
}

/* handle raw space delimited string
   i.e. 140 0 5 72 32	set a song
   from arduino
   String cmd = Serial.readStringUntil('\n');
   char cmdBuffer[256];
   cmd.toCharArray(cmdBuffer, 256);
   doCommand(cmdBuffer);
*/
void Roombie::doCommand(char* cmd)
{
  const char s[2] = " ";	// space delimited
  char *token;

  /* get the first token */
  token = strtok(cmd, s);

  /* walk through other tokens */
  while ( token != NULL ) {
    pSerial->write(atoi(token));
    token = strtok(NULL, s);
  }
}

void Roombie::wakeup() {
  digitalWrite(D3, HIGH);
  delay(100);
  digitalWrite(D3, LOW);
  delay(500);
  digitalWrite(D3, HIGH);
  delay(2000);
}

void Roombie::startOI128()
{
  pSerial->write(128);
}

void Roombie::stopOI173()
{
  pSerial->write(173);
}

// powers down and set roomba to passive mode so charging can work.
void Roombie::powerDown133()
{
  pSerial->write(133);
}

void Roombie::factoryReset7()
{
  pSerial->write(7);
}

void Roombie::goSafeMode131()
{
  pSerial->write(131);
}

void Roombie::goFullMode132()
{
  pSerial->write(132);
}

void Roombie::goPassiveMode128()
{
  pSerial->write(128);
}

void Roombie::doClean()
{
  pSerial->write(135);
}

void Roombie::doMaxClean()
{
  pSerial->write(136);
}

void Roombie::doSpotClean()
{
  pSerial->write(134);
}

void Roombie::doDock()
{
  pSerial->write(143);
}

/**
 * Special case to turn clock-wise on the spot.
 */
void Roombie::turnCW(short velocity){
  // Turn in place clockwise = -1 = 0xFFFF
  drive(velocity, 0xFF);
}

/**
 * Special case to turn counter-clockwise on the spot.
 */
void Roombie::turnCCW(short velocity) {
  // Turn in place counter-clockwise = 1= 0x0001
  drive(velocity, 0x01);
}


/**
 * Serial sequence: [137] [Velocity high byte] [Velocity low byte] [Radius high byte] [Radius low byte]
 * Velocity (-500 to 500 mm/s).  speed can only be in steps of 28.5 mm/s.
 * Radius (-2000 to 2000 mm)
 * Available in modes: Safe or Full
 */
void Roombie::drive(short velocity, short radius) {
  // clamp the velocity and radius
  if (velocity < -500) velocity = -500;
  if (velocity > 500) velocity = 500; 
  if (radius < -2000) radius = -2000;
  if (radius > 2000) radius = 2000; 
  
  pSerial->write(137);
  pSerial->write(velocity >> 8);  // MSB
  pSerial->write(velocity);       // LSB
  pSerial->write(radius >> 8);    // MSB
  pSerial->write(radius);         // LSB
}


/*
 	Read all roomba sensor data.  Becareful that this is a slow process
 	that can take 100s of ms.
  example:
  SensorData sensorData;
  uint8_t buf[128];
  DynamicJsonDocument outgoingDoc[1024]; // just make sure the size if big enough for your payload
  roombie.readAllSensors(&pSensorData, buf);
  buf is just the byte-raw data for debugging.
*/
bool Roombie::readAllSensors(struct SensorData *pSensorData, DynamicJsonDocument& outgoingDoc, uint8_t* buffer) {
  bool isSuccess = false;

  // reads all sensors and returns 80 packets
  pSerial->write(142);
  pSerial->write(100);
  uint8_t pktLen = 80;
  short val = 0;

  if (_readSerial(buffer, pktLen) == true) {
    uint8_t subBuffer[2];
    subBuffer[0] = buffer[0];	// 7
    val = _convertBytesToUInt(subBuffer, 1);
    pSensorData->isRightBump = ((val & 0x1) != 0);
    pSensorData->isLeftBump = ((val & 0x2) != 0);
    pSensorData->isWheelRightDrop = ((val & 0x4) != 0);
    pSensorData->isWheelLeftDrop = ((val & 0x8) != 0);

    subBuffer[0] = buffer[1];	// 8
    val = _convertBytesToUInt(subBuffer, 1);
    pSensorData->isWall = ((val & 0x1) != 0);

    subBuffer[0] = buffer[2];	// 9
    val = _convertBytesToUInt(subBuffer, 1);
    pSensorData->isCliffLeft = ((val & 0x1) != 0);

    subBuffer[0] = buffer[3];	// 10
    val = _convertBytesToUInt(subBuffer, 1);
    pSensorData->isCliffFrontLeft = ((val & 0x1) != 0);

    subBuffer[0] = buffer[4];	// 11
    val = _convertBytesToUInt(subBuffer, 1);
    pSensorData->isCliffFrontRight = ((val & 0x1) != 0);

    subBuffer[0] = buffer[5];	// 12
    val = _convertBytesToUInt(subBuffer, 1);
    pSensorData->isCliffRight = ((val & 0x1) != 0);

    subBuffer[0] = buffer[6];	// 13
    val = _convertBytesToUInt(subBuffer, 1);
    pSensorData->isVirtualWall = (val & 0x01 != 0);

    subBuffer[0] = buffer[7];	// 14
    val = _convertBytesToUInt(subBuffer, 1);
    pSensorData->isSideBrushOverCurrent = ((val & 0x1) != 0);
    pSensorData->isMainBrushOverCurrent = ((val & 0x2) != 0);
    pSensorData->isRightWheelOverCurrent = (val & 0x4 != 0);
    pSensorData->isLeftWheelOverCurrent = ((val & 0x8) != 0);

    subBuffer[0] = buffer[8];	// 15
    val = _convertBytesToUInt(subBuffer, 1);
    pSensorData->isDirtDetect = ((val & 0x1) != 0);

    // buffer[9] for pktId 16 is not used.

    subBuffer[0] = buffer[10];	// 17
    pSensorData->isSideBrushOverCurrent = _convertBytesToUInt(subBuffer, 1);

    subBuffer[0] = buffer[11];	// 18
    pSensorData->buttons = _convertBytesToUInt(subBuffer, 1);

    subBuffer[0] = buffer[12];	// 19
    subBuffer[1] = buffer[13];
    pSensorData->distance = _convertBytesToInt(subBuffer, 2);

    subBuffer[0] = buffer[14];	// 20
    subBuffer[1] = buffer[15];
    pSensorData->angle = _convertBytesToInt(subBuffer, 2);

    subBuffer[0] = buffer[16];	// 21
    pSensorData->chargingState = _convertBytesToUInt(subBuffer, 1);

    subBuffer[0] = buffer[17];	// 22
    subBuffer[1] = buffer[18];
    pSensorData->voltage = _convertBytesToUInt(subBuffer, 2);

    subBuffer[0] = buffer[19];	// 23
    subBuffer[1] = buffer[20];
    pSensorData->current = _convertBytesToInt(subBuffer, 2);

    subBuffer[0] = buffer[21];	// 24
    pSensorData->temperature = _convertBytesToInt(subBuffer, 1);

    subBuffer[0] = buffer[22];	// 25
    subBuffer[1] = buffer[23];
    pSensorData->batteryCharge = _convertBytesToUInt(subBuffer, 2);

    subBuffer[0] = buffer[24];	// 26
    subBuffer[1] = buffer[25];
    pSensorData->batteryCapacity = _convertBytesToUInt(subBuffer, 2);

    subBuffer[0] = buffer[26];	// 27
    subBuffer[1] = buffer[27];
    pSensorData->wallSignal = _convertBytesToUInt(subBuffer, 2);

    subBuffer[0] = buffer[28];	// 28
    subBuffer[1] = buffer[29];
    pSensorData->cliffLeftSignal = _convertBytesToUInt(subBuffer, 2);

    subBuffer[0] = buffer[30];	// 29
    subBuffer[1] = buffer[31];
    pSensorData->cliffFrontLeftSignal = _convertBytesToUInt(subBuffer, 2);

    subBuffer[0] = buffer[32];	// 30
    subBuffer[1] = buffer[33];
    pSensorData->cliffFrontRightSignal = _convertBytesToUInt(subBuffer, 2);

    subBuffer[0] = buffer[34];	// 31
    subBuffer[1] = buffer[35];
    pSensorData->cliffRightSignal = _convertBytesToUInt(subBuffer, 2);

    subBuffer[0] = buffer[39];	// 34
    pSensorData->chargingSource = _convertBytesToUInt(subBuffer, 1);

    subBuffer[0] = buffer[40];	// 35
    pSensorData->oiMode = _convertBytesToUInt(subBuffer, 1);

    subBuffer[0] = buffer[41];	// 36
    pSensorData->songNumber = _convertBytesToUInt(subBuffer, 1);

    subBuffer[0] = buffer[42];	// 37
    pSensorData->songPlaying = _convertBytesToUInt(subBuffer, 1);

    subBuffer[0] = buffer[43];	// 38
    pSensorData->numOfStreamPkts = _convertBytesToUInt(subBuffer, 1);

    subBuffer[0] = buffer[44];	// 39
    subBuffer[1] = buffer[45];
    pSensorData->requestedVelocity = _convertBytesToInt(subBuffer, 2);

    subBuffer[0] = buffer[46];	// 40
    subBuffer[1] = buffer[47];
    pSensorData->requestedRadius = _convertBytesToInt(subBuffer, 2);

    subBuffer[0] = buffer[48];	// 41
    subBuffer[1] = buffer[49];
    pSensorData->requestedRightVelocity = _convertBytesToInt(subBuffer, 2);

    subBuffer[0] = buffer[50];	// 42
    subBuffer[1] = buffer[51];
    pSensorData->requestedLeftVelocity = _convertBytesToInt(subBuffer, 2);

    subBuffer[0] = buffer[52];	// 43
    subBuffer[1] = buffer[53];
    pSensorData->leftEncoderCounts = _convertBytesToInt(subBuffer, 2);

    subBuffer[0] = buffer[54];	// 44
    subBuffer[1] = buffer[55];
    pSensorData->rightEncoderCounts = _convertBytesToInt(subBuffer, 2);

    subBuffer[0] = buffer[56];	// 45
    val = _convertBytesToUInt(subBuffer, 1);
    pSensorData->isLeftLightBumper = ((val & 0x1) != 0);
    pSensorData->isFrontLeftLightBumper = ((val & 0x2) != 0);
    pSensorData->isCenterLeftLightBumper = ((val & 0x4) != 0);
    pSensorData->isCenterRightLightBumper = (val & 0x8 != 0);
    pSensorData->isFrontRightLightBumper = ((val & 0x10) != 0);
    pSensorData->isRightLightBumper = ((val & 0x20) != 0);

    subBuffer[0] = buffer[57];	// 46
    subBuffer[1] = buffer[58];
    pSensorData->leftLightBumper = _convertBytesToUInt(subBuffer, 2);

    subBuffer[0] = buffer[59];	// 47
    subBuffer[1] = buffer[60];
    pSensorData->frontLeftLightBumper = _convertBytesToUInt(subBuffer, 2);

    subBuffer[0] = buffer[61];	// 48
    subBuffer[1] = buffer[62];
    pSensorData->centerLeftLightBumper = _convertBytesToUInt(subBuffer, 2);

    subBuffer[0] = buffer[63];	// 49
    subBuffer[1] = buffer[64];
    pSensorData->centerRightLightBumper = _convertBytesToUInt(subBuffer, 2);

    subBuffer[0] = buffer[65];	// 50
    subBuffer[1] = buffer[66];
    pSensorData->frontRightLightBumper = _convertBytesToUInt(subBuffer, 2);

    subBuffer[0] = buffer[67];	// 51
    subBuffer[1] = buffer[68];
    pSensorData->rightLightBumper = _convertBytesToUInt(subBuffer, 2);

    subBuffer[0] = buffer[69];	// 52
    pSensorData->irCharacterLeft = _convertBytesToUInt(subBuffer, 1);

    subBuffer[0] = buffer[70];	// 53
    pSensorData->irCharacterRight = _convertBytesToUInt(subBuffer, 1);

    subBuffer[0] = buffer[71];	// 54
    subBuffer[1] = buffer[72];
    pSensorData->leftMotorCurrent = _convertBytesToInt(subBuffer, 2);

    subBuffer[0] = buffer[73];	// 55
    subBuffer[1] = buffer[74];
    pSensorData->rightMotorCurrent = _convertBytesToInt(subBuffer, 2);

    subBuffer[0] = buffer[75];	// 56
    subBuffer[1] = buffer[76];
    pSensorData->mainBrushCurrent = _convertBytesToInt(subBuffer, 2);

    subBuffer[0] = buffer[77];	// 57
    subBuffer[1] = buffer[78];
    pSensorData->sideBrushCurrent = _convertBytesToInt(subBuffer, 2);

    subBuffer[0] = buffer[79];	// 58
    pSensorData->stasis = _convertBytesToUInt(subBuffer, 1);

    // pack data into json for transmission
    outgoingDoc["isRightBump"] = pSensorData->isRightBump;        // 7
    outgoingDoc["isLeftBump"] = pSensorData->isLeftBump;        // 7
    outgoingDoc["isWheelRightDrop"] = pSensorData->isWheelRightDrop;      // 7
    outgoingDoc["isWheelLeftDrop"] = pSensorData->isWheelLeftDrop;     // 7
    outgoingDoc["isWall"] = pSensorData->isWall;          // 8
    outgoingDoc["isCliffLeft"] = pSensorData->isCliffLeft;       // 9
    outgoingDoc["isCliffFrontLeft"] = pSensorData->isCliffFrontLeft;      // 10
    outgoingDoc["isCliffFrontRight"] = pSensorData->isCliffFrontRight;     // 11
    outgoingDoc["isCliffRight"] = pSensorData->isCliffRight;        // 12
    outgoingDoc["isVirtualWall"] = pSensorData->isVirtualWall;       // 13
    outgoingDoc["isSideBrushOverCurrent"] = pSensorData->isSideBrushOverCurrent;  // 14
    outgoingDoc["isMainBrushOverCurrent"] = pSensorData->isMainBrushOverCurrent;  // 14
    outgoingDoc["isRightWheelOverCurrent"] = pSensorData->isRightWheelOverCurrent; // 14
    outgoingDoc["isLeftWheelOverCurrent"] = pSensorData->isLeftWheelOverCurrent;  // 14
    outgoingDoc["isDirtDetect"] = pSensorData->isDirtDetect;        // 15
    outgoingDoc["irCharacterOmni"] = pSensorData->irCharacterOmni;     // 17
    outgoingDoc["buttons"] = pSensorData->buttons;         // 18 // needs work
    outgoingDoc["distance"] = pSensorData->distance;          // 19 // don't use
    outgoingDoc["angle"] = pSensorData->angle;         // 20 // don' use
    outgoingDoc["chargingState"] = pSensorData->chargingState; // 21
    outgoingDoc["voltage"] = pSensorData->voltage;     // 22
    outgoingDoc["current"] = pSensorData->current;         // 23
    outgoingDoc["temperature"] = pSensorData->temperature;       // 24
    outgoingDoc["batteryCharge"] = pSensorData->batteryCharge; // 25
    outgoingDoc["batteryCapacity"] = pSensorData->batteryCapacity; // 26
    outgoingDoc["wallSignal"] = pSensorData->wallSignal;    // 27
    outgoingDoc["cliffLeftSignal"] = pSensorData->cliffLeftSignal; // 28
    outgoingDoc["cliffFrontLeftSignal"] = pSensorData->cliffFrontLeftSignal;  // 29
    outgoingDoc["cliffFrontRightSignal"] = pSensorData->cliffFrontRightSignal; // 30
    outgoingDoc["cliffRightSignal"] = pSensorData->cliffRightSignal;  // 31
    outgoingDoc["chargingSource"] = pSensorData->chargingSource;       // 34
    outgoingDoc["oiMode"] = pSensorData->oiMode;      // 35
    outgoingDoc["songNumber"] = pSensorData->songNumber;    // 36
    outgoingDoc["songPlaying"] = pSensorData->songPlaying;   // 37
    outgoingDoc["numOfStreamPkts"] = pSensorData->numOfStreamPkts; // 38
    outgoingDoc["requestedVelocity"] = pSensorData->requestedVelocity;   // 39
    outgoingDoc["requestedRadius"] = pSensorData->requestedRadius;     // 40
    outgoingDoc["requestedRightVelocity"] = pSensorData->requestedRightVelocity;  // 41
    outgoingDoc["requestedLeftVelocity"] = pSensorData->requestedLeftVelocity; // 42
    outgoingDoc["leftEncoderCounts"] = pSensorData->leftEncoderCounts;   // 43
    outgoingDoc["rightEncoderCounts"] = pSensorData->rightEncoderCounts;    // 44
    outgoingDoc["isLeftLightBumper"] = pSensorData->isLeftLightBumper;     // 45
    outgoingDoc["isFrontLeftLightBumper"] = pSensorData->isFrontLeftLightBumper;  // 45
    outgoingDoc["isCenterLeftLightBumper"] = pSensorData->isCenterLeftLightBumper; // 45
    outgoingDoc["isCenterRightLightBumper"] = pSensorData->isCenterRightLightBumper;  // 45
    outgoingDoc["isFrontRightLightBumper"] = pSensorData->isFrontRightLightBumper; // 45
    outgoingDoc["isRightLightBumper"] = pSensorData->isRightLightBumper;    // 45
    outgoingDoc["leftLightBumper"] = pSensorData->leftLightBumper;     // 46
    outgoingDoc["frontLeftLightBumper"] = pSensorData->frontLeftLightBumper;    // 47
    outgoingDoc["centerLeftLightBumper"] = pSensorData->centerLeftLightBumper; // 48
    outgoingDoc["centerRightLightBumper"] = pSensorData->centerRightLightBumper;  // 49
    outgoingDoc["frontRightLightBumper"] = pSensorData->frontRightLightBumper; // 50
    outgoingDoc["rightLightBumper"] = pSensorData->rightLightBumper;      // 51
    outgoingDoc["irCharacterLeft"] = pSensorData->irCharacterLeft;   // 52
    outgoingDoc["irCharacterRight"] = pSensorData->irCharacterRight; // 53
    outgoingDoc["leftMotorCurrent"] = pSensorData->leftMotorCurrent;  // 54
    outgoingDoc["rightMotorCurrent"] = pSensorData->rightMotorCurrent; // 55
    outgoingDoc["mainBrushCurrent"] = pSensorData->mainBrushCurrent;  // 56
    outgoingDoc["sideBrushCurrent"] = pSensorData->sideBrushCurrent;  // 57
    outgoingDoc["stasis"] = pSensorData->stasis;  // 58 0-3

    isSuccess = true;
  } else {
    isSuccess = false;
  }

  return isSuccess;
}

/*
   Read the selected sensor.
   packetId. the packetId
   dest. The buffer to hold the incoming bytearray
   len. the expected number of bytes for this packetId.
   For example usage,
  	uint8_t buf[128];
    int pktLen = 0;
    if (roombie.readSensor(pktId, &pktLen, buf) == true)
*/
bool Roombie::readSensor(uint8_t packetId, uint8_t pktLen, uint8_t* buf) {
  pSerial->write(142);
  pSerial->write(packetId);

  return _readSerial(buf, pktLen);
}

// Reads at most len bytes and stores them to dest
// If successful, returns true.
// If there is a timeout, returns false
// Blocks until all bytes are read
// Caller must ensure there is sufficient space in dest
// High byte sent first.
bool Roombie::_readSerial(uint8_t* dest, uint8_t len)
{
  while (len-- > 0)
  {
    unsigned long startTime = millis();
    while (!pSerial->available())
    {
      // Look for a timeout
      if (millis() > startTime + ROOMBA_READ_TIMEOUT)
        return false; // Timed out
    }
    uint8_t bVal = pSerial->read();
    *dest++ = bVal;
  }
  return true;
}

/*
   Convert byte array to unsigned integer. [MSB,X,X,X,X,LSB]
*/
unsigned int Roombie::_convertBytesToUInt(uint8_t* input, uint8_t numOfBytes)
{
  unsigned int iVal = 0;
  for (int i = 0; i < numOfBytes; i++) {
    iVal += (input[i] << (8 * (numOfBytes - 1 - i)));
  }

  return iVal;
}

/*
   Convert byte array to signed integer. [MSB,X,X,X,X,LSB]
*/
int Roombie::_convertBytesToInt(uint8_t* input, uint8_t numOfBytes)
{
  int result = 0;
  for (int i = 0; i < numOfBytes; i++) {
    if (i == 0) {
      // this is the MSB
      uint8_t msb = input[0];

      // test for last bit. if last bit is 1 then this is a negative number
      if ((msb >> 7)) {
        result = (msb ^ 0xFF) + 1;                          // get 2's compliment
        result = (result << (8 * (numOfBytes - 1 - i)));    // bitshift to get the correct value based on byte location
        result *= -1;                                       // well this is a negative number then.
      } else {
        result += (msb << (8 * (numOfBytes - 1 - i)));
      }
    } else {
      result += (input[i] << (8 * (numOfBytes - 1 - i)));
    }
  }

  return result;
}

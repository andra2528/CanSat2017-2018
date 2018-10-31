// Used Arduino examples as starting point

#include <Adafruit_MPL3115A2.h>
#include <SD.h>
#include<SPI.h>
#include <String.h>
#include <Wire.h>

Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

float voltage;
String data;
float P0, missionTime,h,tempC, pressure = 0;
int ascending_check, packageNumber = 0;

File theSensorData; //File for sensor data

void setup()
{
  Serial.begin(9600);

  baro.begin();
  P0 = baro.getPressure();

  pinMode(10, OUTPUT); //Declare 10 an output and reserve it
  SD.begin(8); //Initialize the SD card reader
  pinMode(6, OUTPUT); //Initialize the LED pin as an output.
  pinMode(A3, INPUT);

  // Two beeps if data is written successfully on the card
  if (SD.begin(8))
  {
    digitalWrite(6, HIGH);
    delay(100);
    digitalWrite(6, LOW);
    delay(100);
    digitalWrite(6, HIGH);
    delay(100);
    digitalWrite(6, LOW);
  }

  if (!baro.begin())
  {
    //Three beeps if the sensor isn't working.
    for(int soundNo = 1; soundNo <= 3; soundNo ++)
    {
      digitalWrite(6, HIGH);
      delay(100);
      digitalWrite(6, LOW);
      delay(100);

    }
  }
}

void loop()
{
  voltage = (float)analogRead(A3)/1023*5*2;

  missionTime = millis(); //time in milliseconds

  // The pressure is in Pascals.
  pressure = baro.getPressure();
  tempC = baro.getTemperature();

  // calculate altitude using the Barometric Equation
  h = ((pow(P0/pressure, 1.0/5.257) - 1) * (tempC + 273.15))/0.0065;

  packageNumber ++;

  //Beep continuosly when landing.
  if (h >= 50)
    ascending_check = 1;

  if ((h < 10) && (ascending_check == 1))
  {
     while(1)
     {
       digitalWrite(6, HIGH);
       delay(200);
       digitalWrite(6, LOW);
       delay(200);
     }
   }

  theSensorData = SD.open("Data.txt", FILE_WRITE);
  if (theSensorData)
  {
    digitalWrite(6, HIGH);
    delay(50);
    digitalWrite(6, LOW);

    //write package number, temperature, height, pressure and time data to card
    //close the file
    theSensorData.print(packageNumber);
    theSensorData.print(",");
    theSensorData.print(tempC);
    theSensorData.print(",");
    theSensorData.print(h);
    theSensorData.print(",");
    theSensorData.print(pressure);
    theSensorData.print(",");
    theSensorData.print(missionTime);
    theSensorData.print(",");
    theSensorData.println(voltage);

    theSensorData.close();
  }

  delay(1000);

  data = String(String(packageNumber) + ',' + String(tempC) + ',' + String(h) + ',' + String(pressure) + ',' + String(missionTime) + ',' + String(voltage));

  // send data to ground
  sendFrame(Serial, data);
  Serial.println(",");
  delay(250);

}

// the following code was taken from: https://drive.google.com/open?id=1_CPOG4x81SkBJj8z7y7SU-G0MbjUABFo

//Lawrence France 9454745
//27/02/18
//xbee_send_function

//Adaptation of best_send_function for CanSat workshop on Radio Communications 28/02/17
//Sketch to send an API frame with an Xbee via Serial port

//Xbee send function
//Need collected sensor payload as a single comma delimited string, with serial stream designated
void sendFrame(Stream &_serial, String data) {

  //setup serial stream
  Stream* serial;
  serial = &_serial;

  // API frame structure
  // start    lengthx2    type    ID    addressx8   option    datax   checksum

  //constant variables
  uint8_t START_BYTE      = 0x7e;
  uint8_t frameType       = 0x00;
  uint8_t frameID         = 0x01;
  uint8_t addressHigh[]   = {0x00, 0x13, 0xa2, 0x00};
  uint8_t addressLow[]    = {0x41, 0x62, 0xd2, 0xb2};
  uint8_t optionByte      = 0x00;

  //constant payload
  String teamCode("DDDD");
  String which_system("EEEE");

  //calculating sum of addHigh for checksum
  uint8_t addressHighSum = 0;
  for (int i = 0; i < sizeof(addressHigh); i++) {
    addressHighSum += addressHigh[i];
  }
  //Serial.println(addressHighSum, HEX);

  //calculating sum of addLow for checksum
  uint8_t addressLowSum = 0;
  for (int i = 0; i < sizeof(addressLow); i++) {
    addressLowSum += addressLow[i];
  }
  //Serial.println(addressLowSum, HEX);

  //concatenate payload with constant 7622, data (or container) at beginning
  String fullPayload = String(String(teamCode) + ',' + String(which_system) + ',' + data);

  //calculating sum of data for checksum
  uint8_t payloadSum = 0;
  uint8_t payloadBytes[fullPayload.length()];
  fullPayload.getBytes(payloadBytes,fullPayload.length());
  for (int i = 0; i<fullPayload.length(); i++) {
    payloadSum += payloadBytes[i];
  }
  //Serial.println(payloadSum, HEX);

  //calculate length bytes (between length bytes and checksum)
  //(frame is type, ID, address, option, data  )
  int frameLength = 2 + sizeof(addressHigh) + sizeof(addressLow) + 1 + fullPayload.length();
  //Serial.println(frameLength);

  //split length integer into 2 bytes
  uint8_t msbLen = (frameLength >> 8) & 0xff;
  uint8_t lsbLen = frameLength & 0xff;
  //Serial.println(msbLen, HEX); Serial.println(lsbLen, HEX);

  //checksum is FF minus 8-bit sum of bytes themselves (not count of bytes) between length and checksum bytes
  uint8_t checkSum = 0xff - ((frameType + frameID + addressHighSum + addressLowSum + optionByte + payloadSum) & 0xff);
  //Serial.println(checkSum, HEX);

  //write API frame to serial B-)
  serial->write(START_BYTE);
  serial->write(msbLen);
  serial->write(lsbLen);
  serial->write(frameType);
  serial->write(frameID);
  serial->write(addressHigh, sizeof(addressHigh));
  serial->write(addressLow, sizeof(addressLow));
  serial->write(optionByte);
  serial->write(payloadBytes, fullPayload.length());
  serial->write(checkSum);
}

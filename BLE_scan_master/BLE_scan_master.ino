/*
   Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleScan.cpp
   Ported to Arduino ESP32 by Evandro Copercini
*/



#include <Arduino.h>
#include <HardwareSerial.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "BLEBeacon.h"
#include "BLEEddystoneTLM.h"
#include "BLEEddystoneURL.h"

#include "pwr2distance.h"

//BLE definitions
BLEScan* pBLEScan;
int scanTime = 1; //In seconds
uint16_t beaconUUID = 0xFEAA;
#define ENDIAN_CHANGE_U16(x) ((((x)&0xFF00)>>8) + (((x)&0xFF)<<8))

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice);
};


//UART defenitions
#define SLAVES_AMOUNT 2
HardwareSerial Slave0(1);            //RX TX 
HardwareSerial Slave1(2);            //RX TX
char slaveChars[SLAVES_AMOUNT];
String slaveStrings[SLAVES_AMOUNT]; 
enum state_type {
  idle,
  id_reading,
  rssi_reading
};
state_type slaveStates[SLAVES_AMOUNT] = {idle, idle};

void reception(HardwareSerial* Slave, int num);


//Data definitions
int rssis[SLAVES_AMOUNT + 1] = {0, 0, 0};
double distances[SLAVES_AMOUNT + 1] = {1, 1, 1}; //in meters

//Output definitions
const int ledPin1 =  14;
const int ledPin2 =  26;
const int ledPin3 =  33;
const int ledPin4 =  27;
const int ledPin5 =  32;
const int ledPin6 =  25;


void setup() {
  Serial.begin(115200);
  Serial.println("Scanning...");

//  ledcSetup(0, 5000, 12);
//  ledcAttachPin(13, 0);
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  pinMode(ledPin4, OUTPUT);
  pinMode(ledPin5, OUTPUT);
  pinMode(ledPin6, OUTPUT);
  
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(160);
  pBLEScan->setWindow(159);  // less or equal setInterval value

//  Slave0.begin(115200);
//  Slave1.begin(115200);
  Slave0.begin(115200, SERIAL_8N1, 2, 4); 
  Slave1.begin(115200, SERIAL_8N1, 19, 22); 
}


void loop() {
  BLEScanResults foundDevices = pBLEScan->start(scanTime);
  
  reception(&Slave0, 0);
  reception(&Slave1, 1);

  for (int i = 0; i < SLAVES_AMOUNT + 1; ++i) distances[i] = pwr2dist(rssis[i]);

  Serial.printf("pwr: %d, %d, %d; dist: %lf, %lf, %lf\n", 
                rssis[0], 
                rssis[1], 
                rssis[2],
                distances[0],
                distances[1],
                distances[2]);
  
  //ledcWrite(0, val); //0 < val < 4096  
  displayPWRonLEDs();
}

void MyAdvertisedDeviceCallbacks::onResult(BLEAdvertisedDevice advertisedDevice) {
  std::string strServiceData = advertisedDevice.getServiceData();
  uint8_t cServiceData[100];
  strServiceData.copy((char *)cServiceData, strServiceData.length(), 0);
  
  if (advertisedDevice.haveManufacturerData() == true) {
    std::string strManufacturerData = advertisedDevice.getManufacturerData();
    uint8_t cManufacturerData[100];
    strManufacturerData.copy((char *)cManufacturerData, strManufacturerData.length(), 0);
    
    if (strManufacturerData.length() == 25 && cManufacturerData[0] == 0x4C  && cManufacturerData[1] == 0x00) {
      BLEBeacon oBeacon = BLEBeacon();
      oBeacon.setData(strManufacturerData);
      
      if (ENDIAN_CHANGE_U16(oBeacon.getMajor()) == 1) {
        int rssi = advertisedDevice.getRSSI();
        rssis[0] = rssi;
        
        Serial.printf("ID: %04X Major: %d Minor: %d UUID: %s Power: %d\n",
          oBeacon.getManufacturerId(),
          ENDIAN_CHANGE_U16(oBeacon.getMajor()),
          ENDIAN_CHANGE_U16(oBeacon.getMinor()),
          oBeacon.getProximityUUID().toString().c_str(),
          rssi);
      }
    }
  }
} 

void reception(HardwareSerial* Slave, int num) {
  //Slave->listen();

  if (Slave->available()) {
    slaveChars[num] = Slave->read();
    
//    Serial.print(String(num) + " got:(" + String((int)slaveChars[num]) + ")");
//    Serial.write(slaveChars[num]);
//    Serial.print("\n");
    
    switch(slaveChars[num]) {
      case ' ':
        //Serial.print("[" + slaveStrings[num] + "]\n");
        if (slaveStrings[num] == "ID") {
          slaveStates[num] = id_reading;
          //Serial.print(String(num) + " id\n");
        }
        else if (slaveStrings[num] == "PWR") {
          slaveStates[num] = rssi_reading;
          //Serial.print(String(num) + " rssi\n");

        } else {
          //Serial.print(String(num) + " number reading");
          //for (int i = 0; i < slaveStrings[num].length(); i++) Serial.print(byte(slaveStrings[num][i]) + "|");
          //Serial.print("\n");
                   
          switch (slaveStates[num]) {
            case idle:
              break;
              
            case id_reading:
              //converting id from string to int
              break;

            case rssi_reading:
              int rssi = slaveStrings[num].toInt();//converting rssi form sting to int
              rssis[num + 1] = rssi; //writing rssi value to array
              Serial.print(String(rssis[num + 1]) + " was written to the " + num + "\n");
              break;
          }
        }

        slaveStrings[num] = "";
        break;
        
      case '\n':
      case '\r':
        //Serial.print("[" + slaveStrings[num] + "]\n");
        slaveStrings[num] = "";
        slaveStates[num] = idle;
        break;

      default:
        slaveStrings[num] += String(slaveChars[num]);
      }
    }
}

void displayPWRonLEDs() {
  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, LOW);
  digitalWrite(ledPin3, LOW);
  digitalWrite(ledPin4, LOW);
  digitalWrite(ledPin5, LOW);
  digitalWrite(ledPin6, LOW);

  if (rssis[0] > -50) {
    digitalWrite(ledPin1, HIGH);
    digitalWrite(ledPin2, HIGH);
    digitalWrite(ledPin3, HIGH);
    digitalWrite(ledPin4, HIGH);
    digitalWrite(ledPin5, HIGH);
    digitalWrite(ledPin6, HIGH);
  
  } else {
    if (-rssis[0]<-rssis[1] & -rssis[0]<-rssis[2]){
      digitalWrite(ledPin1, HIGH);
      if (-rssis[1]<-rssis[2]) {
        digitalWrite(ledPin5, HIGH);
      }
      else{
        digitalWrite(ledPin4, HIGH);
      }
    }  
    if (-rssis[1]<-rssis[0] & -rssis[1]<-rssis[2]){
      digitalWrite(ledPin2, HIGH);
      if (rssis[0]>rssis[2]) {
        digitalWrite(ledPin5, HIGH);
      }
      else{
        digitalWrite(ledPin6, HIGH);
      }
      }
    if (-rssis[2]<-rssis[0] & -rssis[2]<-rssis[1]){
      digitalWrite(ledPin3, HIGH);
      if (-rssis[1]<-rssis[0]) {
        digitalWrite(ledPin6, HIGH);
      }
      else{
        digitalWrite(ledPin4, HIGH);
      }
    }
  }
}

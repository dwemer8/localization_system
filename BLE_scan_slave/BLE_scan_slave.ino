/*
   Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleScan.cpp
   Ported to Arduino ESP32 by Evandro Copercini
*/



#include <Arduino.h>
#include <SoftwareSerial.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "BLEBeacon.h"
#include "BLEEddystoneTLM.h"
#include "BLEEddystoneURL.h"

SoftwareSerial Master(13, 15); //RX, TX

BLEScan* pBLEScan;
int scanTime = 1; //In seconds
uint16_t beconUUID = 0xFEAA;
#define ENDIAN_CHANGE_U16(x) ((((x)&0xFF00)>>8) + (((x)&0xFF)<<8))

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      std::string strServiceData = advertisedDevice.getServiceData();
       uint8_t cServiceData[100];
       strServiceData.copy((char *)cServiceData, strServiceData.length(), 0);

       if (advertisedDevice.haveManufacturerData()==true) {
          std::string strManufacturerData = advertisedDevice.getManufacturerData();
          
          uint8_t cManufacturerData[100];
          strManufacturerData.copy((char *)cManufacturerData, strManufacturerData.length(), 0);
          
          if (strManufacturerData.length()==25 && cManufacturerData[0] == 0x4C  && cManufacturerData[1] == 0x00 ) {
            BLEBeacon oBeacon = BLEBeacon();
            oBeacon.setData(strManufacturerData);
            if (ENDIAN_CHANGE_U16(oBeacon.getMajor()) == 1) {
              Master.printf("ID %04X PWR %d\n", 
                oBeacon.getManufacturerId(), 
                advertisedDevice.getRSSI()
                );
                            
              Serial.printf("ID: %04X Major: %d Minor: %d UUID: %s Power: %d\n",
                oBeacon.getManufacturerId(),
                ENDIAN_CHANGE_U16(oBeacon.getMajor()),
                ENDIAN_CHANGE_U16(oBeacon.getMinor()),
                oBeacon.getProximityUUID().toString().c_str(),
                advertisedDevice.getRSSI()
                );
              }
            }
          }
       } 
};


void setup() {
  Serial.begin(115200);
  Serial.print("Port started");
  Master.begin(115200);
  Master.print("Port started");

//  BLEDevice::init("");
//  pBLEScan = BLEDevice::getScan(); //create new scan
//  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
//  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
//  pBLEScan->setInterval(100);
//  pBLEScan->setWindow(99);  // less or equal setInterval value
}

void loop() {
  //BLEScanResults foundDevices = pBLEScan->start(scanTime);
  Master.print("ID " + String(0) + " PWR " + String(-200) + "\n");
  Serial.print("ID " + String(0) + " PWR " + String(-200) + "\n");
  delay(1000);
}

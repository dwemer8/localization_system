/*
   Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleScan.cpp
   Ported to Arduino ESP32 by Evandro Copercini
*/

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <HardwareSerial.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "BLEBeacon.h"
#include "BLEEddystoneTLM.h"
#include "BLEEddystoneURL.h"

//BLE definitions
BLEScan* pBLEScan;
int scanTime = 1; //In seconds
uint16_t beaconUUID = 0xFEAA;
#define ENDIAN_CHANGE_U16(x) ((((x)&0xFF00)>>8) + (((x)&0xFF)<<8))

//class for receiving signal from beacon
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice);
};

//UART definitions
HardwareSerial SerialPort(1); // use UART1

void setup() {
  Serial.begin(115200);
  Serial.print("Port started");

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster

  /*!< Scan interval. This is defined as the time interval from
  when the Controller started its last LE scan until it begins the subsequent LE scan.
  Range: 0x0004 to 0x4000 Default: 0x0010 (10 ms)
  Time = N * 0.625 msec
  Time Range: 2.5 msec to 10.24 seconds*/
  pBLEScan->setInterval(160);

  /*!< Scan window. The duration of the LE scan. LE_Scan_Window
  shall be less than or equal to LE_Scan_Interval
  Range: 0x0004 to 0x4000 Default: 0x0010 (10 ms) 4 to 16384 (10240 ms)
  Time = N * 0.625 msec
  Time Range: 2.5 msec to 10240 msec */
  pBLEScan->setWindow(159);  // less or equal setInterval value, 

  SerialPort.begin(115200, SERIAL_8N1, 2, 4); 
}

void loop() {
  BLEScanResults foundDevices = pBLEScan->start(scanTime);
}

void MyAdvertisedDeviceCallbacks::onResult(BLEAdvertisedDevice advertisedDevice) {
   std::string strServiceData = advertisedDevice.getServiceData();
   uint8_t cServiceData[100];
   strServiceData.copy((char *)cServiceData, strServiceData.length(), 0);

   if (advertisedDevice.haveManufacturerData() == true) {
      std::string strManufacturerData = advertisedDevice.getManufacturerData();
      
      uint8_t cManufacturerData[100];
      strManufacturerData.copy((char *)cManufacturerData, strManufacturerData.length(), 0);
      
      if (strManufacturerData.length()==25 && cManufacturerData[0] == 0x4C  && cManufacturerData[1] == 0x00) {
        BLEBeacon oBeacon = BLEBeacon();
        oBeacon.setData(strManufacturerData);
        
        if (ENDIAN_CHANGE_U16(oBeacon.getMajor()) == 1) {
          int manufacturerID = oBeacon.getManufacturerId(), 
              rssi = advertisedDevice.getRSSI();
          //Sending data to the master
          SerialPort.printf("ID %04X PWR %d \n", //padding with zeroes, 4-symbols length, hex; decimal
            manufacturerID, 
            rssi
            );

          //Debug output
          Serial.printf("ID: %04X Major: %d Minor: %d UUID: %s Power: %d\n",
            manufacturerID,
            ENDIAN_CHANGE_U16(oBeacon.getMajor()),
            ENDIAN_CHANGE_U16(oBeacon.getMinor()),
            oBeacon.getProximityUUID().toString().c_str(),
            rssi
            );
         }
      }
   }
}

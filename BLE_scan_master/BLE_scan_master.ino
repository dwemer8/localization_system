/*
   Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleScan.cpp
   Ported to Arduino ESP32 by Evandro Copercini
*/

#include <map>
#include <String>
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
#include "trilateration.h"

//Definitions -------------------------------------------------------------------------
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
int phi2direction(double phi);
int r2ring(double r);


//Output definitions
const int ledRB =  14;// right bottom
const int ledRM =  27;// right middle
const int ledRT =  33;// rigth top
const int ledLT =  25;// left top
const int ledLM =  26;// left middle
const int ledLB =  32;// left bottom
const int leds[] = {ledRB, ledRM, ledRT, ledLT, ledLM, ledLB};
const int LEDS_AMOUNT = 6;
const int freq = 5000;
const int resolution = 8;

void displayPositionOnLEDs(pointPolar2D worker);
void displayPositionOnLEDs(point3D worker);


//Main part -------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("Scanning...");

  for (int i = 0; i < LEDS_AMOUNT; ++i) {
    ledcSetup(i, freq, resolution); //channel, freq, res
    ledcAttachPin(leds[i], i); //GPIO, channel
    ledcWrite(i, 255); //channel, 0 <= power < 2^resolution
    //  pinMode(leds[i], OUTPUT);
  }
  
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(160);
  pBLEScan->setWindow(159);  // less or equal setInterval value

  Slave0.begin(115200, SERIAL_8N1, 2, 4); 
  Slave1.begin(115200, SERIAL_8N1, 19, 22); 
}


void loop() {
  BLEScanResults foundDevices = pBLEScan->start(scanTime);
  
  reception(&Slave0, 0);
  reception(&Slave1, 1);

  for (int i = 0; i < SLAVES_AMOUNT + 1; ++i) distances[i] = pwr2dist(rssis[i]);

  point3D worker = trilaterate3DspheresInPlane(distances[0], distances[1], distances[2]);
  pointCartesian2D workerCart2D = {worker.x, worker.y};
  pointPolar2D workerPol2D = cart2pol(workerCart2D);
  
  Serial.printf("pwr: %d, %d, %d; "
                "dist: %.3lf, %.3lf, %.3lf; "
                "worker_cart: %.3lf, %.3lf, %.3lf; "
                "worker_polar: %.3lf, %.3lf; \n", 
                rssis[0], 
                rssis[1], 
                rssis[2],
                distances[0],
                distances[1],
                distances[2],
                worker.x,
                worker.y,
                worker.z,
                workerPol2D.r,
                workerPol2D.phi
                );
  
  displayPositionOnLEDs(worker);
  //displayPWRonLEDs();
}

//Functions -------------------------------------------------------------------------
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

int phi2direction(double phi) {
  if (-150 <= phi && phi <= -90) {
    return 5;
  } else if (-90 <= phi && phi <= -30) {
    return 0;
  } else if (-30 <= phi && phi <= 30) {
    return 1;
  } else if (30 <= phi && phi <= 90) {
    return 2;
  } else if (90 <= phi && phi <= 150) {
    return 3;
  } else {
    return 4;
  }
}

int r2ring(double r) {
  if (r < 2) {
    return -1;
  } else if (r < 5) {
    return 0;
  } else if (r < 10) {
    return 1;
  } else {
    return 2;
  }
}

void displayPositionOnLEDs(point3D worker) {
  pointCartesian2D workerCart2D = {worker.x, worker.y};
  displayPositionOnLEDs(cart2pol(workerCart2D));
}

void displayPositionOnLEDs(pointPolar2D worker) {
  int ring = r2ring(worker.r), 
      dir = phi2direction(worker.phi);
  Serial.printf("ring: %d, dir: %d ",
                ring,
                dir);

  int ledPowers[LEDS_AMOUNT] = {0, 0, 0, 0, 0, 0};
  if (ring == -1) {
    for (int i = 0; i < LEDS_AMOUNT; i++) ledPowers[i] = 255;
    
  } else {
    ledPowers[dir] = (int)((2 - ring)/2.0*255.0);
  }
  
  Serial.printf("LEDs: ");
  for (int i = 0; i < LEDS_AMOUNT; ++i) {
    ledcWrite(i, ledPowers[i]);
    
    Serial.printf("%d=%d ",
                  i,
                  ledPowers[i]);
  }
  Serial.printf("\n");
}

void displayPWRonLEDs() {
  digitalWrite(ledRB, LOW);
  digitalWrite(ledRM, LOW);
  digitalWrite(ledRT, LOW);
  digitalWrite(ledLT, LOW);
  digitalWrite(ledLM, LOW);
  digitalWrite(ledLB, LOW);

  if (rssis[0] > -50) {
    digitalWrite(ledRB, HIGH);
    digitalWrite(ledRM, HIGH);
    digitalWrite(ledRT, HIGH);
    digitalWrite(ledLT, HIGH);
    digitalWrite(ledLM, HIGH);
    digitalWrite(ledLB, HIGH);
  
  } else {
    if (-rssis[0]<-rssis[1] & -rssis[0]<-rssis[2]){
      digitalWrite(ledRB, HIGH);
      if (-rssis[1]<-rssis[2]) {
        digitalWrite(ledLM, HIGH);
      }
      else{
        digitalWrite(ledLT, HIGH);
      }
    }  
    if (-rssis[1]<-rssis[0] & -rssis[1]<-rssis[2]){
      digitalWrite(ledRM, HIGH);
      if (rssis[0]>rssis[2]) {
        digitalWrite(ledLM, HIGH);
      }
      else{
        digitalWrite(ledLB, HIGH);
      }
      }
    if (-rssis[2]<-rssis[0] & -rssis[2]<-rssis[1]){
      digitalWrite(ledRT, HIGH);
      if (-rssis[1]<-rssis[0]) {
        digitalWrite(ledLB, HIGH);
      }
      else{
        digitalWrite(ledLT, HIGH);
      }
    }
  }
}

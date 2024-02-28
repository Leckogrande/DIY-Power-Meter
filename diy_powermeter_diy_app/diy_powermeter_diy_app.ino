//all the code related to BLE is from:
//https://github.com/mo-thunderz/BleMobileAppMitAppInventor
//by user "mo-thunderz"

//I also used the documentation and example codes of the HX711 library and the Adafruit_MPU6050 library.


//import all librarys necessary
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "HX711.h"

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
BLECharacteristic* pCharacteristic_2 = NULL;
BLEDescriptor *pDescr;
BLE2902 *pBLE2902;

Adafruit_MPU6050 mpu; //creates the object for the MPU6050
HX711 scaleR;         //creates the object for the right scale
HX711 scaleL;         //creates the object for the left scale

//create all variables needed
float crank_lenght = 0.175;  //TODO set the crank lenght
unsigned char bleBuffer[8]; 
short power = 0;
float FR = 0;
float Mr = 0;
float PR = 0;
float FL = 0;
float ML = 0;
float PL = 0;

float nowTime = 0;
float prevTime = 0;
float nowAlpha = 0;
float alpha = 0;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

//set all pins
#define sdaPin 12
#define sclPin 14
uint8_t dataPinR = 23;
uint8_t clockPinR = 19;
uint8_t dataPinL = 5;
uint8_t clockPinL = 18;
uint32_t start, stop;

const int numDataPoints = 240;   // Number of data points for rolling average
int dataPoints[numDataPoints];  // Array to store the data points
int currentIndex = 0;           // Index to keep track of the current position

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR1_UUID          "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR2_UUID          "e3223119-9445-4e96-a4a1-85358c4046a2"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class CharacteristicCallBack: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override { 
    std::string pChar2_value_stdstr = pChar->getValue();
    String pChar2_value_string = String(pChar2_value_stdstr.c_str());
    int pChar2_value_int = pChar2_value_string.toInt();
    Serial.println("pChar2: " + String(pChar2_value_int)); 
  }
};

void setup() {
  Serial.begin(115200);
  
  scaleR.begin(dataPinR, clockPinR); 
  scaleR.set_offset(4287106963);
  scaleR.set_scale(-19);       //TODO you need to calibrate this yourself after mounting the crank to the bike. use the example code of the HX711 library
  scaleR.tare();   //reset the scale to zero = 0
  scaleL.begin(dataPinL, clockPinL);
  scaleL.set_offset(4287106963);
  scaleL.set_scale(-19);       //TODO you need to calibrate this yourself after mounting the crank to the bike. use the example code of the HX711 library
  scaleL.tare();   //reset the scale to zero = 0

  Wire.begin(sdaPin, sclPin); //starts the I2C connection

  // Create the BLE Device
  BLEDevice::init("DIY Powermeter");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHAR1_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );                   

  pCharacteristic_2 = pService->createCharacteristic(
                      CHAR2_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  
                    );  

  // Create a BLE Descriptor
  
  pDescr = new BLEDescriptor((uint16_t)0x2901);
  pDescr->setValue("A very interesting variable");
  pCharacteristic->addDescriptor(pDescr);
  
  pBLE2902 = new BLE2902();
  pBLE2902->setNotifications(true);
  
  // Add all Descriptors here
  pCharacteristic->addDescriptor(pBLE2902);
  pCharacteristic_2->addDescriptor(new BLE2902());
  
  // After defining the desriptors, set the callback functions
  pCharacteristic_2->setCallbacks(new CharacteristicCallBack());
  
  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");

  while (!Serial)
    delay(10); //pause until serial console opens

  Serial.println("Adafruit MPU6050 test!");  // Try to initialize connection to MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("");
  delay(100);
}

void loop() {
  sensors_event_t a, g, temp;   //creates objects for the MPU6050 data
    mpu.getEvent(&a, &g, &temp);  //gets data from MPU6050

    float w = (-1)*g.gyro.z - 0.04;       //uses angular velocity of the z-axis, changes direction, sets an offset because of sensor drift
    FR = scaleR.get_units(1) /1000 *9.81; //reads out the right scale and converts it into newtons
    FL = scaleL.get_units(1) /1000 *9.81; //reads out the left scale and converts it into newtons
    
    //angular velocity and the forces must be always positive
    if (w<0){
     w = 0;
    }
    if (FR<0){
     FR = 0;
    }
    if (FL<0){
     FL = 0;
    }

    Mr = FR * crank_lenght;  //calculates the torque on the right crank
    PR =  Mr * w;      //calculates the power of the right crank
    ML = FL * crank_lenght;  //calculates the torque on the left crank
    PL =  ML * w;      //calculates the power of the left crank

    prevTime = nowTime;
    nowTime = millis();
    float dt = nowTime - prevTime;         //time between this measurement and the last
    float nowAlpha = w *180/PI * dt /1000; //calulate by how much the angle of the crank has changed since last measurement
    alpha = alpha + nowAlpha;              //calculates total angle of the crank
  
    if (alpha >= 360.0) {                 //if angle of crank reaches 360 degree set it back to 0, adds up total revolutions, sets timestamps for CyclePowerService
      alpha -= 360.0;
    }

    power = PL + PR;   //add left and right power

    if (power>3000){   //filters measurements caused by bad contact to the wheatstone bridges
     power = 3000;
    }
    

    dataPoints[currentIndex] = power;  // Update the array with the new data point
    currentIndex = (currentIndex + 1) % numDataPoints;  // Move to the next position in the array
    float rollingAverage = 0;
    for (int i = 0; i < numDataPoints; i++) {  // Calculate the rolling average
     rollingAverage += dataPoints[i];
    }
    rollingAverage /= numDataPoints;
     
    //prints data for calibration and troubleshooting
    Serial.print("Sent Power: ");
    Serial.print(power);
    Serial.print("W  ");
    Serial.print("PR PL: ");
    Serial.print(PR);
    Serial.print("W ");
    Serial.print(PL);
    Serial.print("W ");
    Serial.print("FR FL: ");
    Serial.print(FR);
    Serial.print("N ");
    Serial.print(FL);
    Serial.print("N ");
    Serial.print("Omega: ");
    Serial.print(w);
    Serial.print("rad/s ");
    Serial.print("Alpha: ");
    Serial.print(alpha);
    Serial.print("° ");
    Serial.print("dt: ");
    Serial.print(dt);
    Serial.println("ms");

    // notify changed value
    if (deviceConnected) {
        pCharacteristic->setValue(value);
        pCharacteristic->notify();
        value = rollingAverage;
        //delay(10);
    }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
}



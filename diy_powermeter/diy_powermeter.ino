//import all librarys necessary
#include "BLEDevice.h"
#include "BLEUtils.h"
#include "BLEServer.h"
#include <BLE2902.h>
#include "Arduino.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "HX711.h"
#include <esp_sleep.h>


//part of the code is commented to be able to set up WebSerial if needed to debug without wired connection
/*
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

AsyncWebServer server(80); // WebSerial is accessible at "<IP Address>/webserial" in browser

const char* ssid = "xxxxx";  // Your WiFi SSID
const char* password = "xxxxx";  // Your WiFi Password
*/


// Define the UUIDs for the services and characteristics
#define CYCLING_POWER_SERVICE_UUID        "1818"
#define CYCLING_POWER_MEASUREMENT_UUID    "2A63"
#define CYCLING_POWER_FEATURE_UUID        "2A65"
#define SENSOR_LOCATION_UUID              "2A5D"

// Flags (including the Cadence and Pedal Power Balance)
uint16_t flags = (1 << 0) | (1 << 1) | (1 << 5);
// Define the values for the Cycling Power Feature and Sensor Location as constants
const uint32_t CYCLING_POWER_FEATURES = (1 << 0) | (1 << 3); // Supports Pedal Power Balance and Cadence
const uint8_t SENSOR_LOCATION = 0x01; // Example: Left crank

// Declare BLE Server and Characteristics globally
BLEServer *pServer;
BLECharacteristic *pCyclingPowerMeasurementCharacteristic;
BLECharacteristic *pCyclingPowerFeatureCharacteristic;
BLECharacteristic *pSensorLocationCharacteristic;


Adafruit_MPU6050 mpu; //creates the object for the MPU6050
HX711 scaleR;         //creates the object for the right scale
HX711 scaleL;         //creates the object for the left scale

//create all variables needed
float crank_lenght = 0.175;  //TODO set the crank lenght
float scaleR_calibration_value = 5.099433; //TODO you need to calibrate this yourself after mounting the crank to the bike. use the calibration example code of the HX711 library
float scaleR_calibration_offset = 4294667228;
float scaleL_calibration_value = -14.402885;
float scaleL_calibration_offset = 4294216973;
float c1 = 0.8253;  //TODO you need to compare the powermeter against a commercial one using different rpm and power to make a linear regression model to get a correction function which uses these variables
float c2 = 0.41861;
float c3 = 0.00372;
#define MOTION_THRESHOLD 2 // Adjust threshold if needed
#define MOTION_DURATION 1  // Adjust duration if needed
int sleep_timer = 120000; //Adjust sleep timer if needed in ms

unsigned char bleBuffer[8]; 
int16_t power = 0; // in Watts, signed 16-bit
uint8_t balance = 0; // Pedal power balance, 0-100, representing 0%-100%
uint16_t rpm = 0; // Revolutions per minute, 16-bit unsigned
int16_t power_corrected = 0;
float FR = 0;
float Mr = 0;
float PR = 0;
float FL = 0;
float ML = 0;
float PL = 0;

bool deviceConnected = false;
volatile bool movementDetected = true;
volatile uint32_t lastMovementTime = 0;
float nowTime = 0;
float prevTime = 0;
float nowAlpha = 0;
float alpha = 0;
int cal_counter = 0;

uint16_t revolutions = 0;
uint16_t timestamp = 0;
int t_diff = 0;
int t_prev = 0;
uint32_t start, stop;

//set all pins
#define sdaPin 12
#define sclPin 14
uint8_t dataPinR = 23;
uint8_t clockPinR = 19;
uint8_t dataPinL = 5;
uint8_t clockPinL = 18;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Device connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Device disconnected");
    }
};

void setup() 
{
  Serial.begin(115200); 
  
  /*
  //sets up Wifi for WebSerial
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.printf("WiFi Failed!\n");
    return;
  }
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  WebSerial.begin(&server);
  server.begin();
  */

  //sets up scales
  scaleR.begin(dataPinR, clockPinR); 
  scaleR.set_offset(scaleR_calibration_offset);
  scaleR.set_scale(scaleR_calibration_value);       
  scaleR.tare();   //reset the scale to zero = 0
  scaleL.begin(dataPinL, clockPinL);
  scaleL.set_offset(scaleL_calibration_offset);
  scaleL.set_scale(scaleL_calibration_value);
  scaleL.tare();   //reset the scale to zero = 0
  
  Wire.begin(sdaPin, sclPin); //starts the I2C connection

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

  mpu.setInterruptPinPolarity(false); // sets polaity of int pin
  mpu.setInterruptPinLatch(true);   //sets latching behavior of int pin
  mpu.setMotionInterrupt(true);   // enables motion interrupt
  mpu.setMotionDetectionThreshold(MOTION_THRESHOLD);
  mpu.setMotionDetectionDuration(MOTION_DURATION);

  Serial.println("");
  delay(100);

  esp_sleep_enable_ext0_wakeup(GPIO_NUM_13,1); //defines which GPIO pin will receive the Motion Interrupt to wake up

 

  BLEDevice::init("DIY Powermeter");

  // Create BLE Server
  pServer = BLEDevice::createServer();

  // Create BLE Service for Cycling Power
  BLEService *pCyclingPowerService = pServer->createService(CYCLING_POWER_SERVICE_UUID);

  // Create a BLE Characteristic for Cycling Power Measurement
  pCyclingPowerMeasurementCharacteristic = pCyclingPowerService->createCharacteristic(
                                            CYCLING_POWER_MEASUREMENT_UUID,
                                            BLECharacteristic::PROPERTY_NOTIFY
                                          );
  BLE2902 *pCpmDesc = new BLE2902();
  pCpmDesc->setNotifications(true);
  pCyclingPowerMeasurementCharacteristic->addDescriptor(pCpmDesc);

  // Create a BLE Characteristic for Cycling Power Feature
  pCyclingPowerFeatureCharacteristic = pCyclingPowerService->createCharacteristic(
                                          CYCLING_POWER_FEATURE_UUID,
                                          BLECharacteristic::PROPERTY_READ
                                        );
  pCyclingPowerFeatureCharacteristic->setValue((uint8_t*)&CYCLING_POWER_FEATURES, sizeof(CYCLING_POWER_FEATURES));

  // Create a BLE Characteristic for Sensor Location
  pSensorLocationCharacteristic = pCyclingPowerService->createCharacteristic(
                                      SENSOR_LOCATION_UUID,
                                      BLECharacteristic::PROPERTY_READ
                                    );
  pSensorLocationCharacteristic->setValue((uint8_t*)&SENSOR_LOCATION, sizeof(SENSOR_LOCATION));

  // Start the service
  pCyclingPowerService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(CYCLING_POWER_SERVICE_UUID);
  pAdvertising->start();

  Serial.println("Cycling Power Meter service started");

}

void loop() 
{
  scaleR.tare();   //reset the scale to zero = 0
  scaleL.tare();   //reset the scale to zero = 0 
  Serial.println("scales tared and ready to connect...");
  delay(1000);

  while (pServer->getConnectedCount() > 0) {

    sensors_event_t a, g, temp;   //creates objects for the MPU6050 data
    mpu.getEvent(&a, &g, &temp);  //gets data from MPU6050

    prevTime = nowTime;
    nowTime = millis();
    float dt = nowTime - prevTime;         //time between this measurement and the last

    float t = temp.temperature -10;
    float w = g.gyro.z + 0.02;       //reads out angular velocity of the z-axis, sets an offset because of sensor drift
    FR = scaleR.get_units(1) /1000 *9.81; //reads out the right scale and converts it into newtons
    FL = scaleL.get_units(1) /1000 *9.81; //reads out the left scale and converts it into newtons
    
    //records time since last movement
    if (abs(w) <= 0.1){
      lastMovementTime = lastMovementTime + dt;
    } else {
      lastMovementTime = 0;
    }
    
    //checks if the set sleeptime has been reached
    if ( lastMovementTime >= sleep_timer){
      movementDetected = false;
    } else {
      movementDetected = true;
    }
    
    if (movementDetected == false){
    Serial.println("No movement detected. Going to sleep...");
    delay(100);  // Delay for a short time to allow any Serial.print() messages to be sent
    esp_deep_sleep_start();  // Enter deep sleep mode
    }
    
    float nowAlpha = w *180/PI * dt /1000; //calulate by how much the angle of the crank has changed since last measurement
    alpha = alpha + nowAlpha;              //calculates total angle of the crank
  
    if (alpha >= 360.0) {                 //if angle of crank reaches 360 degree set it back to 0, adds up total revolutions, sets timestamps for CyclePowerService
      alpha -= 360.0;
      t_diff = millis() - t_prev;
      revolutions = revolutions + 1;   
      timestamp = timestamp + t_diff *1000 /1024;
      t_prev = millis();
      cal_counter = 0;
      
    }

     // Ensure that the timestamp rolls over correctly
    timestamp = timestamp % 65536; // 65536 is 2^16, max uint16_t value

    //protocol to re-tare the scales
    if (alpha <= -360.0) {                 //if angle of crank reaches -360 degree set it back to 0, increases counter until 5 revolutions to start re-tareing
      alpha += 360.0;
      cal_counter = cal_counter + 1;   
      if (cal_counter == 5){
        delay(2000);
        scaleR.tare();   //reset the scale to zero = 0
        scaleL.tare();   //reset the scale to zero = 0
        cal_counter = 0;
      }
    }


    //angular velocity must be positive
    if (w<0){
     w = 0;
    }

    Mr = FR * crank_lenght;  //calculates the torque on the right crank
    PR =  Mr * w;      //calculates the power of the right crank
    ML = FL * crank_lenght;  //calculates the torque on the left crank
    PL =  ML * w;      //calculates the power of the left crank

    power = PL + PR;   //add left and right power
    rpm = w * 60/(2*PI);
  
    if (PL<0){   //removes negative power values which would cause errorsin the power balance
     PL = 0;
    }
    if (PR<0){   //removes negative power values which would cause errors in the power balance
     PR = 0;
    }
    
    balance = (PL/(PL + PR + 0.001))*100;
    
    if (power<0){   //removes negative power values which would cause errors
     power = 0;
    }
    if (power>1500){   //filters measurements caused by bad contact to the wheatstone bridges
     power = 1500;
    }

    power_corrected = (c1*power+c2*rpm+c3*power*rpm);  //correction function derived from linear regression model
    
    //to correct for possible wrong power numbers due to the correction function
    if (rpm<5){   
     power_corrected = 0;
    }
    if (power<10){   
     power_corrected = 0;
    }
     
    //prints data for calibration and troubleshooting
    Serial.print("Sent Power: ");
    Serial.print(power_corrected);
    Serial.print("W  ");
    Serial.print("Balance: ");
    Serial.print(balance);
    Serial.print("%  ");
    Serial.print("rpm: ");
    Serial.print(rpm);
    Serial.print("rpm ");
    Serial.print("uncorrected Power: ");
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
    Serial.print("Temp: ");
    Serial.print(t);
    Serial.print("°C ");
    Serial.print("Omega: ");
    Serial.print(w);
    Serial.print("rad/s ");
    Serial.print("Alpha: ");
    Serial.print(alpha);
    Serial.print("° ");
    Serial.print("revolutions: ");
    Serial.print(revolutions);
    Serial.print("  ");
    Serial.print("timestamp: ");
    Serial.print(timestamp);
    Serial.print("  ");
    Serial.print("lastMovementTime: ");
    Serial.print(lastMovementTime);
    Serial.print("ms ");
    Serial.print("dt: ");
    Serial.print(dt);
    Serial.println("ms");

    /*
    //WebSerial.print("Sent Power: ");
    //WebSerial.println(power_corrected);
    //WebSerial.print("W  ");
    //WebSerial.print("PR PL: ");
    //WebSerial.print(PR);
    //WebSerial.print("W ");
    //WebSerial.print(PL);
    //WebSerial.print("W ");
    //WebSerial.print("FR FL: ");
    WebSerial.print(FR);
    WebSerial.print("N ");
    WebSerial.print(FL);
    WebSerial.print("N ");
    //WebSerial.print("Omega: ");
    WebSerial.print(w);
    WebSerial.println("rad/s ");
    //WebSerial.print("Alpha: ");
    //WebSerial.print(alpha);
    //WebSerial.print("° ");
    //WebSerial.print("dt: ");
    //WebSerial.print(dt);
    //WebSerial.println("ms");

    delay(500);
    */
   
    // Prepare the data packet as a std::string
    std::string value;

    // Add the flags
    value += static_cast<char>(flags & 0xFF); // Flags LSB
    value += static_cast<char>((flags >> 8) & 0xFF); // Flags MSB

    // Add the power
    value += static_cast<char>(power & 0xFF); // Power LSB
    value += static_cast<char>((power >> 8) & 0xFF); // Power MSB

    // Add pedal balance if present
    if (flags & (1 << 0)) {
        value += static_cast<char>(balance);
    }

    // Add crank revolution data if present
    if (flags & (1 << 5)) {
        value += static_cast<char>(revolutions & 0xFF); // Revolutions LSB
        value += static_cast<char>((revolutions >> 8) & 0xFF); // Revolutions MSB
    }

    // Add last crank event time if present
    if (flags & (1 << 5)) {
        value += static_cast<char>(timestamp & 0xFF); // Last Crank Event Time LSB
        value += static_cast<char>((timestamp >> 8) & 0xFF); // Last Crank Event Time MSB
    }

    // Set the characteristic value and notify
    pCyclingPowerMeasurementCharacteristic->setValue((uint8_t*)value.data(), value.length());
    pCyclingPowerMeasurementCharacteristic->notify();
  }
}


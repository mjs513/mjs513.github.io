#include <ArduinoBLE.h>

// Setup ICM-20948 IMU 
#include "ICM_20948.h"  //sparkfun library
ICM_20948_I2C IMU;

//Setup Madgwick filter and AHRS
unsigned long lastUpdate, now;  // sample period expressed in milliseconds for filters
float quant[4], ypr[3], _dt;
//float q0, q1, q2, q3; 
float grx, gry, grz;
float ax, ay, az, gx, gy, gz, mx, my, mz;

//trig conversions
float rad2deg = 180.0f / PI;
float deg2rad = PI / 180.0f;

// Magnetic declination. If used, must be selected for your location.
// Note: May not be used in this sketch yet, needs to be checked.
//#define MAG_DEC -13.1603      //degrees for Flushing, NY
#define MAG_DEC 0

//standard gravity in m/s/s
#define gravConst  9.80665

// end madgwick config

//Setup BLE
#define BLE_SENSE_UUID(val) ("6fbe1da7-" val "-44de-92c4-bb6e04fb0212")
const int VERSION = 0x00000000;
BLEService                     service                       (BLE_SENSE_UUID("0000"));
BLEUnsignedIntCharacteristic   versionCharacteristic         (BLE_SENSE_UUID("1001"), BLERead);
BLECharacteristic              accelerationCharacteristic    (BLE_SENSE_UUID("3001"), BLENotify, 3 * sizeof(float)); // Array of 3 floats, G
BLECharacteristic              gyroscopeCharacteristic       (BLE_SENSE_UUID("3002"), BLENotify, 3 * sizeof(float)); // Array of 3 floats, dps
BLECharacteristic              magneticFieldCharacteristic   (BLE_SENSE_UUID("3003"), BLENotify, 3 * sizeof(float)); // Array of 3 floats, uT
BLECharacteristic              ahrsCharacteristic            (BLE_SENSE_UUID("3004"), BLENotify, 3 * sizeof(float)); // Array of 3 floats, uT

// String to calculate the local and device name
String name;

// buffer to read samples into, each sample is 16-bits
short sampleBuffer[256];

// number of samples read
volatile int samplesRead;

void setup() {
  Serial.begin(9600);

  //while (!Serial);
  Serial.println("Started");
  
  Wire.begin();
  Wire.setClock(400000);
  
  bool initialized = false;
  while (!initialized)
  {
    IMU.begin(Wire, 1);
    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(IMU.statusString());
    if (IMU.status != ICM_20948_Stat_Ok)
    {
      Serial.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }


  if (!BLE.begin()) {
    Serial.println("Failled to initialized BLE!");

    while (1);
  }

  String address = BLE.address();

  Serial.print("address = ");
  Serial.println(address);

  address.toUpperCase();

  name = "BLETeensy-";
  name += address[address.length() - 5];
  name += address[address.length() - 4];
  name += address[address.length() - 2];
  name += address[address.length() - 1];

  Serial.print("name = ");
  Serial.println(name);

  BLE.setLocalName(name.c_str());
  BLE.setDeviceName(name.c_str());
  BLE.setAdvertisedService(service);

  service.addCharacteristic(versionCharacteristic);
  service.addCharacteristic(accelerationCharacteristic);
  service.addCharacteristic(gyroscopeCharacteristic);
  service.addCharacteristic(magneticFieldCharacteristic);
  service.addCharacteristic(ahrsCharacteristic);
  
  versionCharacteristic.setValue(VERSION);

  BLE.addService(service);

  BLE.advertise();
}

//elapsedMillis send_data;
bool dataRdy;
void loop() {
  while (BLE.connected()) {
    dataRdy = IMU.dataReady();
    if (dataRdy)
    {  
      now = micros();
      _dt = ((now - lastUpdate) * 0.000001);
      IMU.getAGMT();
      ax = IMU.accX()/1000.0f;    ay = IMU.accY()/1000.0f;    az = IMU.accZ()/1000.0f; 
      gx = IMU.gyrX();            gy = IMU.gyrY();  gz = IMU.gyrZ();
      mx = IMU.magX();            my = IMU.magY();            mz = IMU.magZ(); 

      MadgwickAHRSupdate(gx * deg2rad, gy * deg2rad, gz * deg2rad, ax, ay, az, mx, my, mz);
      lastUpdate = now;
      getYawPitchRoll(ypr);

      if (ahrsCharacteristic.subscribed()) {
        Serial.printf("Yaw: %f, Pitch: %f, Roll: %f\n", ypr[0], ypr[1], ypr[2]);
        ahrsCharacteristic.writeValue(ypr, sizeof(ypr));
      }       

      delay(5);
      
      if (accelerationCharacteristic.subscribed() && dataRdy) {
        float acceleration[3] = { ax, ay, az };
        for(uint8_t i = 0; i < 3; i++) {
          Serial.print(acceleration[i]); Serial.print(", ");     
        }  
        accelerationCharacteristic.writeValue(acceleration, sizeof(acceleration));
      }
      delay(5);      

      if (gyroscopeCharacteristic.subscribed() && dataRdy) {
        float dps[3] = { gx, gy, gz};
        for(uint8_t i = 0; i < 3; i++) {
          Serial.print(dps[i]); Serial.print(", ");     
        }            
        gyroscopeCharacteristic.writeValue(dps, sizeof(dps));
      }
      delay(5);   

      if (magneticFieldCharacteristic.subscribed() && dataRdy) {
        float magneticField[3] = { mx, my, mz };
        for(uint8_t i = 0; i < 3; i++) {
          Serial.print(magneticField[i]); Serial.print(", ");     
        }      
        magneticFieldCharacteristic.writeValue(magneticField, sizeof(magneticField));
      }
      Serial.println(); 
      delay(20);   
    }
  }
}



#ifndef SENSOR_H
#define SENSOR_H

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include <utility/imumaths.h>
#include "quaternion.h"

// Inertial measurement unit (IMU)
class IMU {
    Adafruit_BNO055 imu = Adafruit_BNO055();
    bool calibration_state = true;
    int calibration_timer = 10;

public:
    IMU(bool state) : calibration_state(state) {}
    ~IMU() {}
    
    void init() {
        imu.begin();        //make sure serial.begin() is set to 115200 before this point.
        delay(1000);
        imu.setExtCrystalUse(true);
    }

    void calibrate() {
        static int counter = 0;
        uint8_t system, gyros, accel, mg = 0;
        
        switch(calibration_state) {
            case true:
                imu.getCalibration(&system, &gyros, &accel, &mg);
                Serial.println("Calibrating:");
                while ((gyros != 3) || (accel != 3)) {
                    imu.getCalibration(&system, &gyros, &accel, &mg);
                    Serial.print(counter);
                    Serial.print("s");
                    Serial.print(" -> ");
                    if (counter == calibration_timer) {
                        counter = 0;
                        Serial.println("Gyro calibration: ");
                        Serial.print(gyros);
                        Serial.print(" of 3."); 
                        Serial.println("Accelerometer calibration: ");
                        Serial.print(accel);
                        Serial.println(" of 3.");
                    }
                    counter ++;
                    delay(1000);
                }
                if((gyros == 3) && (accel == 3)) {
                    Serial.println("Calibrated!");
                    calibration_state = false;
                }
                break;
            case false:
                Serial.println("Calibration turned off.");
                break;
        }
    }   
  
    Vector3 getOrientation() {
        imu::Vector<3> ori = imu.getVector(Adafruit_BNO055::VECTOR_EULER);
        return Vector3(ori.x(), ori.y(), ori.z());
    }

    Vector3 getAcceleration() {
        imu::Vector<3> acc = imu.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        return Vector3(acc.x(), acc.y(), acc.z());
    }
    
    Vector3 getAngularVelocity() {
        imu::Vector<3> gyro = imu.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        return Vector3(gyro.x(), gyro.y(), gyro.z());
    }
    
    Vector3 getMagnetometer() {
        imu::Vector<3> mag = imu.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
        return Vector3(mag.x(), mag.y(), mag.z());
    }
    
    Quaternion getQuaternion() {
        imu::Quaternion quat = imu.getQuat();
        return Quaternion(quat.w(), quat.x(), quat.y(), quat.z());
    }
};

class BMP {
    Adafruit_BMP280 bmp;
    
    public:
    void bmpInit(uint8_t address) {
        if (!bmp.begin(address)) {
            Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
            Serial.println("SensorID was: 0x") Serial.println(bmp.sensorID(), 16);
            Serial.print("ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
            Serial.print("ID of 0x56-0x58 represents a BMP 280,\n");
            Serial.print("ID of 0x60 represents a BME 280.\n");
            Serial.print("ID of 0x61 represents a BME 680.\n");
            while (1);
        }

        /* Default settings from datasheet */
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                        Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                        Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                        Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                        Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    }

    Vector3 getBMPReading() {
    // degC, Pa, m
      return Vector3(bmp.readTemperature(), bmp.readPressure(), bmp.readAltitude(1013.25));
    }
};
#endif
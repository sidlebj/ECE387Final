//Brandon Sidle Lightsaber Driver
//I2C library credit goes to Jeff Rowberg
//@ https://github.com/jrowberg/i2cdevlib


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

//keeps track of when to hum
bool on = false;

//sound triggers
int turnon = 3;
int hum = 4;
int turnoff = 5;
int clash = 6;
int swing = 7;
int button = 8;
int led = 9;

//minimum amount of acceleration to trigger a swing/clash
int minClash = 35000;
int minSwing = 25000;

#define LED_PIN 13
bool blinkState = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    //Begin Accelerometer Code
    // initialize serial communication
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);

    //begin sound driver
    pinMode(turnon, OUTPUT);
    pinMode(hum, OUTPUT);
    pinMode(turnoff, OUTPUT);
    pinMode(clash, OUTPUT);
    pinMode(swing, OUTPUT);
    pinMode(button, INPUT);
    pinMode(led, OUTPUT);

    //sound triggers are active LOW, so keep HIGH when not in use
    digitalWrite(turnon, HIGH);
    digitalWrite(hum, HIGH);
    digitalWrite(turnoff, HIGH);
    digitalWrite(clash, HIGH);
    digitalWrite(swing, HIGH);
}

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    // display tab-separated accel/gyro x/y/z values
    Serial.print("a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

    //begin sound driver
    if(on){
      //hum while on
      digitalWrite(hum, LOW);
      digitalWrite(led, HIGH);
      //drive swing/clash sensor
      if(abs(ax) > 30000 || abs(ay) > 30000 || 
      abs(az) > 30000){
        digitalWrite(hum, HIGH);
        digitalWrite(clash, LOW);
        delay(25);
        digitalWrite(clash, HIGH);
      }  
      
      if((abs(ax) > 25000 && abs(ax) < 30000) || 
      (abs(ay) > 25000 && abs(ay) < 30000) || 
      (abs(az) > 25000 && abs(az) < 30000)){
        digitalWrite(hum, HIGH);
        digitalWrite(swing, LOW);
        delay(25);
        digitalWrite(swing, HIGH);
      }
      //drive the off button
      if(digitalRead(button) == LOW){
        on = !on;
        digitalWrite(hum, HIGH);
        //turn off sound
        digitalWrite(turnoff, LOW);
        delay(25);
        digitalWrite(turnoff, HIGH);
        }
    } else {
      //if not on, make sure it doesn't hum
      digitalWrite(led, LOW);
      digitalWrite(hum, HIGH);
    }

    //drive the on button
    if(digitalRead(button) == LOW){
        on = !on;
        //turn on sound
        digitalWrite(turnon, LOW);
        delay(25);
        digitalWrite(turnon, HIGH);
      }
      Serial.println(on);
}

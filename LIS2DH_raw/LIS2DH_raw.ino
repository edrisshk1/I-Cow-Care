// I2C demonstration Arduino sketch for LIS2DH
// 23/9/2013 original by Conor Forde <me@conorforde.com> at https://github.com/Snowda/LIS2DH
//
// Changelog:
//     2014-09-23 - initial release

#include "Wire.h"

// LIS2DH must be installed as libraries, or else the .cpp/.h files
// as classes must be in the include path of your project
#include "LIS2DH12.h"

LIS2DH acceleration;

int16_t ax, ay, az;

#define LED_PIN 13
bool blinkState = false;

void setup() {
    Wire.begin();

    // initialize serial communication
    // (19200 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(19200);

    // initialize device
    Serial.println("Initializing Accelerometer...");
    acceleration.init();

    // verify connection
    Serial.println("Testing device connection...");
    Serial.println(acceleration.whoAmI() ? "LIS2DH connection successful" : "LIS2DH connection failed");

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);

    Serial.print("x:\t");
    Serial.print("y:\t");
    Serial.println("z:\t");
}

void loop() {
    // read raw accel/gyro measurements from device
    acceleration.getMotion(&ax, &ay, &az);

    // display tab-separated accel/gyro x/y/z values
    Serial.print(ax);
    Serial.print("\t");
    Serial.print(ay);
    Serial.print("\t");
    Serial.println(az);

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    delay(50) ;
}
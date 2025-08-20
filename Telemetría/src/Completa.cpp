#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BME280.h>
#include <SparkFun_ADXL345.h> // SparkFun ADXL345 Library
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

int n;
int latt, longi;
SoftwareSerial gpsSerial(8, 9); // RX, TX
TinyGPSPlus gps;
float lattitude, longitude;

/*
3.3V a VCC y SDO
SCL A5
SDA A4
*/
ADXL345 adxl = ADXL345(10); // USE FOR SPI COMMUNICATION, ADXL345(CS_PIN);


Adafruit_BME280 bme; // use I2C interface
/*
3.3V a VCC y SDO
SCL A5
SDA A4
*/
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

void setup()
{
    Serial.begin(9600);

    gpsSerial.begin(9600);

    if (!bme.begin())
    {
        Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
        while (1)
            delay(10);
    }
    adxl.powerOn();                     // Power on the ADXL345
    adxl.setRangeSetting(16);           // Give the range settings
                                        // Accepted values are 2g, 4g, 8g or 16g
                                        // Higher Values = Wider Measurement Range
                                        // Lower Values = Greater Sensitivity
    adxl.setSpiBit(0);                  // Dont move or change the 0, it wont give any measures
    adxl.setActivityXYZ(1, 0, 0);       // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
    adxl.setInactivityXYZ(0, 1, 0);     // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
    adxl.setTapDetectionOnXYZ(0, 0, 1); // Detect taps in the directions turned ON "adxl.setTapDetectionOnX(X, Y, Z);" (1 == ON, 0 == OFF)
}

void loop()
{
    Serial.println(n++);
    // Accelerometer Readings
    int data = gpsSerial.read();

    int x, y, z;
    adxl.readAccel(&x, &y, &z); // Read the accelerometer values and store them in variables declared above x,y,z

    sensors_event_t temp_event, pressure_event, humidity_event;
    bme_temp->getEvent(&temp_event);
    bme_pressure->getEvent(&pressure_event);
    bme_humidity->getEvent(&humidity_event);

    Serial.print(F("Ikaro"));

    Serial.print(F(" | "));
    Serial.print((pressure_event.pressure) * 100, 0);
    Serial.print(F(" | "));
    Serial.print((humidity_event.relative_humidity), 1);
    Serial.print(F(" | "));
    Serial.print(latt / 1.0, 5);
    Serial.print(F(" | "));
    Serial.print(longi);
    Serial.print(F(" | "));
    Serial.print(x / 1.0, 2);
    Serial.print(F(" | "));
    Serial.print(y / 1.0, 2);
    Serial.print(F(" | "));
    Serial.print(z / 1.0, 2);
    Serial.print(F(" | "));

    Serial.println();
    if (gps.encode(data))
    {
        lattitude = (gps.location.lat());
        longitude = (gps.location.lng());
        latt=lattitude;
        longi=longitude;
    }
    delay(1000);
}

#include <Axis.h>
#include <math.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.
   
   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).
   
   Connections (For default I2C)
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 5V DC
   Connect GROUND to common ground

   History
   =======
   2014/JULY/25  - First version (KTOWN)
*/
   
/* Assign a unique base ID for this sensor */   
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000


/* Or, use Hardware SPI:
  SCK -> SPI CLK
  SDA -> SPI MOSI
  G_SDO + XM_SDO -> tied together to SPI MISO
  then select any two pins for the two CS lines:
*/

#define LSM9DS0_XM_CS 10
#define LSM9DS0_GYRO_CS 9
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);

/* Or, use Software SPI:
  G_SDO + XM_SDO -> tied together to the MISO pin!
  then select any pins for the SPI lines, and the two CS pins above
*/

#define LSM9DS0_SCLK 13
#define LSM9DS0_MISO 12
#define LSM9DS0_MOSI 11

//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_SCLK, LSM9DS0_MISO, LSM9DS0_MOSI, LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);


Adafruit_NeoPixel strip = Adafruit_NeoPixel(32, 6, NEO_GRB + NEO_KHZ800);

// Customizes the lights on the rings into an order that is more logical for this program
int topRing[16]    = {  6,  5,  4,  3,  2,  1,  0, 15, 14, 13, 12, 11, 10,  9,  8,  7 };
int bottomRing[16] = { 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 16, 17, 18, 19, 20, 21 };
int ringLength = sizeof(topRing)/sizeof(*topRing);

Axis xAxis = Axis(-0.77, 0.29, 0.00);
Axis yAxis = Axis(-0.12 ,0.99, 0.45);
Axis zAxis = Axis(-0.52, 0.49, -0.41);


/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t accel, mag, gyro, temp;
  
  lsm.getSensor(&accel, &mag, &gyro, &temp);
  
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(accel.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(accel.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(accel.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(accel.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(accel.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(accel.resolution); Serial.println(F(" m/s^2"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(mag.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(mag.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(mag.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(mag.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(mag.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(mag.resolution); Serial.println(F(" uT"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(gyro.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(gyro.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(gyro.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(gyro.max_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Min Value:    ")); Serial.print(gyro.min_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Resolution:   ")); Serial.print(gyro.resolution); Serial.println(F(" rad/s"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(temp.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(temp.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(temp.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(temp.max_value); Serial.println(F(" C"));
  Serial.print  (F("Min Value:    ")); Serial.print(temp.min_value); Serial.println(F(" C"));
  Serial.print  (F("Resolution:   ")); Serial.print(temp.resolution); Serial.println(F(" C"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  
  delay(500);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureSensor(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) 
{
//  while (!Serial);  // wait for flora/leonardo
  Serial.begin(9600);
  Serial.println(F("LSM9DS0 9DOF Sensor Test")); Serial.println("");

  /* Initialise the sensor */
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  Serial.println(F("Found LSM9DS0 9DOF"));
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
  /* Setup the sensor gain and integration time */
  configureSensor();
  
  /* We're ready to go! */
  Serial.println("");
  strip.begin();
  strip.show();
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/

void loop(void) 
{  
  /* Get a new sensor event */ 
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(0, &mag, 0, 0);

  strip.setBrightness(10);
//  pointNorth(mag);
  rainbowCycle(mag);
  
  delay(50);
}

bool isWithinTolerance(float target, float tolerance, float value) {
  return ((value <= target+tolerance) && (value >= target-tolerance));
}


void setStripColor(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    delay(wait);
  }
  strip.show();
}

//Points North with two white pixels
void pointNorth(sensors_event_t mag)
{
  float rawPixel = findNorthPixel(mag);
  int targetPixel = floor(rawPixel);
  int secondPixel = getSecondPixel(rawPixel);
  
  Serial.print("Angle: "); Serial.print(floor((atan2(yAxis.scale_value(mag.magnetic.y), xAxis.scale_value(mag.magnetic.x))*(180.0/3.141))));
  Serial.print(" Target Pixel: "); Serial.print(targetPixel); 
  Serial.print(" Top pixel: "); Serial.print(topRing[targetPixel]); 
  Serial.print(" Bottom Pixel: "); Serial.print(bottomRing[targetPixel]); 
  Serial.print("\n");

  setStripColor(strip.Color(0,0,0),0);
  strip.setPixelColor(topRing[targetPixel], strip.Color(255,255,255));
  strip.setPixelColor(topRing[secondPixel], strip.Color(255,255,255));
  strip.setPixelColor(bottomRing[targetPixel], strip.Color(255,255,255));
  strip.setPixelColor(bottomRing[secondPixel], strip.Color(255,255,255));

  strip.show();
}

float findNorthPixel(sensors_event_t mag)
{
  float anglePerLight = 360.0/ringLength;

  //Take the arc tangent of the scaled x and y components which will return radians. Then convert the radians to degrees. 
  //Then divide that by the angle allocated to each light. That will give you a number betweer -8 and 7. Add 8 to scale that to a base zero system.
  float rawPixel = ((atan2(yAxis.scale_value(mag.magnetic.y), xAxis.scale_value(mag.magnetic.x))*(180.0/3.141))/anglePerLight) + 8;
  return rawPixel;
}

int getSecondPixel(float rawPixel)
{
  float pixelRemainder = fmod(rawPixel, 1.0);
  int targetPixel = floor(rawPixel);
  int secondPixel;
  if(pixelRemainder >= 0.5 && targetPixel < ringLength - 1)
    secondPixel = targetPixel + 1;
  else if(pixelRemainder >= 0.5 && targetPixel == ringLength - 1)
    secondPixel = 0;
  else if(pixelRemainder < 0.5 && targetPixel != 0)
    secondPixel = targetPixel - 1;
  else if(pixelRemainder < 0.5 && targetPixel == 0)
    secondPixel = ringLength - 1;
  else
    secondPixel = 0;

  return secondPixel;
}

// Displays a rainbow on the top and bottom ring.
// Will cycle and keep the red portion pointing north.
// Taken from Adadfruit and modified.
void rainbowCycle(sensors_event_t mag) {

  int targetPixel = floor(findNorthPixel(mag));
  
  setStripColor(strip.Color(0,0,0),0);
  uint16_t i;
  for(i=0; i< ringLength; i++) {
    strip.setPixelColor(topRing[(i + targetPixel) % ringLength], Wheel((i * 256 / ringLength) & 255));
    strip.setPixelColor(bottomRing[(i + targetPixel) % ringLength], Wheel((i * 256 / ringLength) & 255));
  }
  strip.show();
}

// From Adafruit
// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}



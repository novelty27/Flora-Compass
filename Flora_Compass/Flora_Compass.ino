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
      Serial.println("Hit");
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
//     colorWipe(strip.Color(0,0,0),0);
  int topRing[16]    = { 14, 13, 12, 11, 10,  9,  8,  7,  6,  5,  4,  3,  2,  1,  0, 15 };
  int bottomRing[16] = { 30, 31, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29 };

  /* Get a new sensor event */ 
  sensors_event_t accel, mag, gyro, temp;

//  lsm.getEvent(&accel, &mag, &gyro, &temp); 
  lsm.getEvent(0, &mag, 0, 0); 
//
//  // print out accelleration data
//  Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" ");
//  Serial.print("  \tY: "); Serial.print(accel.acceleration.y);       Serial.print(" ");
//  Serial.print("  \tZ: "); Serial.print(accel.acceleration.z);     Serial.println("  \tm/s^2");

//   print out magnetometer data

  Serial.print("Magn. X: "); Serial.print(mag.magnetic.x);     Serial.print(" ");
  Serial.print("  \tY: ");   Serial.print(mag.magnetic.y);     Serial.print(" ");
  Serial.print("  \tZ: ");   Serial.print(mag.magnetic.z);     Serial.print("  \tgauss\n");
//  Serial.print(mag.magnetic.x);     Serial.print("\t");
//  Serial.print(mag.magnetic.y);     Serial.print("\t");
//  Serial.print(mag.magnetic.z);     Serial.print("\n");
//  Serial.print("MaxX: "); Serial.print(maxX); Serial.print(" MinX: "); Serial.print(minX);
//  Serial.print(" MaxY: "); Serial.print(maxY); Serial.print(" MinY: "); Serial.print(minY);
//  Serial.print(" MaxZ: "); Serial.print(maxZ); Serial.print(" MinZ: "); Serial.print(minZ);
//    
//  Serial.print("\n\n\n\n\n");

  
//  // print out gyroscopic data
//  Serial.print("Gyro  X: "); Serial.print(gyro.gyro.x); Serial.print(" ");
//  Serial.print("  \tY: "); Serial.print(gyro.gyro.y);       Serial.print(" ");
//  Serial.print("  \tZ: "); Serial.print(gyro.gyro.z);     Serial.println("  \tdps");
//
//  // print out temperature data
//  Serial.print("Temp: "); Serial.print(temp.temperature); Serial.println(" *C");
//
//  float maxX = 0.29;
//  float minX = -0.77;
//  float midX = (maxX + minX)/2.0;
//  float maxY = 0.99;
//  float minY = -0.12;
//  float midY = (maxY + minY)/2.0;
//  float maxZ = 0.49;
//  float minZ = -0.52;
//  float midZ = (maxZ + minZ)/2.0;

  strip.setBrightness(10);
  
  int topRed = 0;
  int bottomRed = 0;
  int topGreen = 0;
  
  int bottomGreen = 0;
  int topBlue = 0;
  int bottomBlue = 0;

  float trueX = 0;
  float trueY = .45;
  float trueZ = -.41;

  float xTolerance = abs(.1);
  float yTolerance = abs(.1);
  float zTolerance = abs(.1);
  

  //(x <= -.5) means the y axis is pointing up and x axis is parallel to the ground. The bottom of the chip is pointing NE
  if(isWithinTolerance(trueX, xTolerance, mag.magnetic.x)) {
    topRed = 255;
    bottomRed = 255;
  }
  else{
    topRed = 0;
    bottomRed = 0;      
  }

  // (y >= .7) means the y axis in the east/west plane and x axis pointing north, bottom of chip pointing east
  if(isWithinTolerance(trueY, yTolerance, mag.magnetic.y)) {
    topGreen = 255;
    bottomGreen = 255;
  }
  else {
    topGreen = 0;
    bottomGreen = 0;      
  }

  if(isWithinTolerance(trueZ, zTolerance, mag.magnetic.z)) {
    topBlue = 255;
    bottomBlue = 255;
  } 
  else {
    topBlue = 0;
    bottomBlue = 0;
  }
  
  
  uint32_t topColor = strip.Color(topRed, topGreen, topBlue);
  uint32_t bottomColor = strip.Color(bottomRed, bottomGreen, bottomBlue);
    
  for(int i = 0; i < 2*sizeof(topRing)/sizeof(topRing[0]); i++) {
    //need to apply color to top and bottom ring
    if(i < sizeof(topRing)/sizeof(topRing[0])){
      strip.setPixelColor(i, topColor);
    }
    else {
      strip.setPixelColor(i, bottomColor);
    }
  }
//  strip.setPixelColor(0, strip.Color(255,255,255));
  strip.show();
  delay(25);
}

bool isWithinTolerance(float target, float tolerance, float value) {
  return ((value <= target+tolerance) && (value >= target-tolerance));
}

void setStripColor(uint32_t c, uint8_t wait) {
  Serial.print("Number of pixels: "); Serial.println(strip.numPixels());
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    delay(wait);
  }
  strip.show();
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);   
    //return strip.Color(255 - WheelPos * 3, 0, 0);
  } 
  else if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    //return strip.Color(0, WheelPos * 3, 0);
  } 
  else {
    WheelPos -= 170;
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<1256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}


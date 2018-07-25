// version 07 librairie wire updated
/********************************************
   GRAPHIC STANDBY HORIZON
 ********************************************
   By FSA
   credits for graphic libraries: adafruit, Henning Karlsen ...
   displays horizon on a 480*320 TFT display
   Arduino Mega 2560 / TFT shield / MPU 6050 sensor
   Version 0.7 
  This version program is functional as it is.
  Do not use for flight. Sensor not accurate enough, no performance guarantees.
  Improvements can be made on filtering
  There is an issue with some sensors MPU6050/Arduinos which do not respect I2C timing and for which there is no easy fix.
  When this problem is fixed, remove all the lines with "delay(tempo);"
  
  to be improved, the display could also show a Gmeter and a sideslip indicator.
  program has been optimised and does not use any Z axis measurement for display
  
*/
#include <TFT_HX8357.h> // graphic library
#include <Wire.h>  // wire library

long accelX, accelY, accelZ;
float angleX = 0, angleY = 0 , angleZ = 0; // we initialise and suppose that the sensor is fitted with Z axis along gravity vector.
float d1 , d2 ; // used for display
//  will correspond to pitch,roll,yaw
// here, we are only interested in Pitch & Roll for display
long dt, lastmillis, newmillis ; // used to compute time intervals
float gForceX, gForceY, gForceZ;
float gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;
float calX , calY, calZ; // used for calibration datar 
long millisec; // used as a timer during calibration
int acel_deadzone = 8;   //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone = 1;   //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
boolean gyroaligned = false ;
 uint16_t x0 = 80, y0 = 0, wid = 320, heig = 312, rad = 80; //
  uint16_t x1 = x0 + wid; // col =  ; // top left corner, width, height, radius, colour
  uint16_t y1 = 150 , y2 = 150 ; // used for display attitude
const int MPU=0x68;
int ax,ay,az,gx,gy,gz,temperatura;
int tempo = 1 ; //delay to remove hangout in wire library, in millisecs

TFT_HX8357 tft = TFT_HX8357();       // Invoke custom library

#define CENTRE 240
#define TFT_GREY 0x7BEF

long timer = 0;
uint32_t runTime = 0;
int error; //

// Color definitions
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0
#define WHITE    0xFFFF

void setup() {
  randomSeed(analogRead(0));// initialises random generator
  // start the MPU 6050
//  Serial.begin(115200);  only needed if testing on the console
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
    Wire.write(0); 
    Wire.endTransmission(true); 
  // Setup the LCD
  tft.init();
  tft.setRotation(3);
  int buf[478];
  int x, x2;
  int y, y2;
  int r;
  int col = 0; // current display colour
  runTime = millis();
 
  // Clear the screen and draw the frame
  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0, 0, 480, 13, TFT_RED);
  tft.fillRect(0, 305, 480, 320, TFT_GREY);
  tft.setTextColor(TFT_WHITE, TFT_GREY);
  tft.setTextSize(2);
  tft.drawCentreString("* STANDBY INSTRUMENT DISPLAY *", CENTRE, 80, 1);
  tft.drawCentreString("* VERSION 0.7 *", CENTRE, 100, 1);
  tft.drawCentreString("INITIALISATION ", CENTRE, 120, 1);
  setupMPU();
  error = Wire.endTransmission();   if (error == 0)  {
    tft.drawCentreString("MPU6050 CONNECTED", CENTRE, 140, 1);
  }
  else {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawCentreString("MPU6050 NOT CONNECTED", CENTRE, 140, 1);
    tft.setTextColor(TFT_WHITE, TFT_GREY);
  }
  tft.drawRect(0, 14, 479, 305 - 14, TFT_BLUE);  // check I2C connection
  Wire.begin(); // start again
  // Draw some filled, rounded rectangles to check colour
  for (int i = 1; i < 6; i++)
  {
    switch (i)
    { case 1:        col = TFT_MAGENTA; x = 10;        break;
      case 2:        col = TFT_RED; x = 106;        break;
      case 3:        col = TFT_GREEN; x = 202;        break;
      case 4:        col = TFT_BLUE; x = 298 ;        break;
      case 5:        col = TFT_YELLOW; x = 394 ;        break;
    }    tft.fillRoundRect(x, 230, 75, 60, 3, col);  }
  for (int i = 15; i < 304; i += 5)  {    tft.drawLine(1, i, (i * 1.6) - 10, 303, TFT_RED);  }
  for (int i = 304; i > 15; i -= 5)  {    tft.drawLine(477, i, (i * 1.6) - 11, 15, TFT_RED);  }
  for (int i = 304; i > 15; i -= 5)  {    tft.drawLine(1, i, 491 - (i * 1.6), 15, TFT_CYAN);  }
  for (int i = 15; i < 304; i += 5)  {    tft.drawLine(477, i, 490 - (i * 1.6), 303, TFT_CYAN);  }

  delay(2000); // pause a few seconds
  // Clear the screen again and draw the frame
  tft.fillScreen(TFT_BLACK);
 //  uint16_t x0,y0,wid,heig,rad,col ; // top left corner, width, height, radius, colour
  // draw upper part of the sky

  x0 = 80, y0 = 0, wid = 320, heig = 312, rad = 80;x1 = x0 + wid;y1 = 150 ; y2 = 150 ;// to set values
  tft.fillRect(x0, y0, wid, y1, TFT_CYAN); // draw upper part of the sky - a box
  tft.fillTriangle(x0, y1, x1, y2, x1, y1, TFT_CYAN); // draw blue triangular part of the sky
  tft.fillTriangle(x0, y1, x1, y2, x0, y2, YELLOW); // draw amber triangular part of earth
  tft.fillRect(x0, y2, wid, heig - y2, YELLOW); // draw lower part of the earth

  gyroX = 0; gyroY = 0; gyroZ = 0; // reset values
  calX = 366, calY = -70 , calZ=168 ; // calibration data resets , later could be memorized
}

void loop() {
  if (gyroaligned == false) calibration ();
  recordAccelRegisters();
  recordGyroRegisters();
   if (gyroaligned == true) {complimentaryfilter(); displaypfd();} // normal operation
}

void setupMPU() {
  Wire.beginTransmission(MPU); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission(false);
  Wire.beginTransmission(MPU); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4)
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s
  Wire.endTransmission(false);
  Wire.beginTransmission(MPU); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(false);
}
void recordAccelRegisters() {
  int f =5 ; // number of filter cycles, the higher the smoothest, but takes time
  accelX=0;accelY=0;accelZ=0; // reset
  for ( int i = 0; i < f; i ++)  {  
  Wire.beginTransmission(MPU); //I2C address of the MPU
   delay(tempo);// to get rid of hangouts
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission(false);
   
  Wire.requestFrom(MPU, 6); //Request Accel Registers (3B - 40)
    ax = Wire.read()<<8|Wire.read();  //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
    ay = Wire.read()<<8|Wire.read();  //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    az = Wire.read()<<8|Wire.read();  //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
   // temperatura = Wire.read()<<8|Wire.read();  //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
   // gx = Wire.read()<<8|Wire.read();  //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
   // gy = Wire.read()<<8|Wire.read();  //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
   // gz = Wire.read()<<8|Wire.read();  //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
   Wire.endTransmission(false);
  accelX = accelX + ax; //Store first two bytes into accelX
  accelY = accelY + ay; //Store middle two bytes into accelY
   }
    gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0;
 // gForceZ = accelZ / 16384.0;
}

void recordGyroRegisters() {
  int f =5 ; // number of filter cycles, the higher the smoothest, but takes time
  gyroX=0;gyroY=0;gyroZ=0; // reset
  for ( int i = 0; i < f; i ++)  {  
  Wire.beginTransmission(MPU); //I2C address of the MPU
  delay(tempo);// to get rid of hangouts
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 4); //Request Gyro Registers (43 - 48)
  while (Wire.available() < 4);
  gyroX = gyroX + (Wire.read() << 8 | Wire.read()); //Store first two bytes into accelX
  gyroY = gyroY + (Wire.read() << 8 | Wire.read()); //Store middle two bytes into accelY
  // gyroZ = gyroZ + (Wire.read() << 8 | Wire.read()); //Store last two bytes into accelZ // not used
  }
  gyroX = gyroX / f ; gyroY = gyroY/f ; // gyroZ = gyroZ / f;// filtering
  
  rotX = (gyroX + calX) / 131.0; //  value measured + added calibration value
  rotY = (gyroY + calY) / 131.0;
  rotZ = (gyroZ + calZ) / 131.0;
}

void printData() {  // print on the serial interface for check, not to be used later
  Serial.print("Gyro (deg)");
  Serial.print(" X=");
  Serial.println(rotX);
  //Serial.print(" Y=");
  //Serial.print(rotY);
  //Serial.print(" Z=");
  //Serial.println(rotZ);
  //Serial.print(" Accel (g)");
  //Serial.print(" X=");
  //Serial.print(gForceX);
  //Serial.print(" Y=");
  //Serial.print(gForceY);
  //Serial.print(" Z=");
  //Serial.println(gForceZ);
  Serial.print("Complimentary filter : dt= ");
  Serial.print(dt);
  //Serial.print(" lastmillis= ");
  //Serial.print(lastmillis);
  //Serial.print(" newmillis= ");
  //Serial.print(newmillis);
  Serial.print(" angleX= ");
  Serial.print(angleX);
  Serial.print(" rotX= ");
  Serial.print(rotX);
  Serial.print(" gForceX= ");
  Serial.println(gForceX);
   
}
void calibration() {
  // gyros calibration, mpu6050 should not be moved during XX secs, therefore gyros rate should be zero
  // method is to zero the gyros rate
  tft.drawCentreString("GYROS CALIBRATION", CENTRE, 160, 1);
  tft.drawCentreString("DO NOT MOVE ", CENTRE, 180, 1);
  delay(5); // pause a few milliseconds
  Wire.beginTransmission(MPU); //I2C address of the MPU
   delay(tempo);// to get rid of hangouts
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();

  Wire.requestFrom(MPU, 6); //Request Gyro Registers (43 - 48)
  while (Wire.available() < 6);
  gyroX = Wire.read() << 8 | Wire.read(); //Store first two bytes into gyroX
  gyroY = Wire.read() << 8 | Wire.read(); //Store middle two bytes into gyroY
  gyroZ = Wire.read() << 8 | Wire.read(); //Store last two bytes into gyroZ

  rotX = (gyroX + calX) / 131.0;
  rotY = (gyroY + calY) / 131.0;
  rotZ = (gyroZ + calZ) / 131.0; // compute first to decide value change

  if (rotX < -0.3) calX = calX + 10 ; // fast change
  if (rotX > 0.3) calX = calX - 10 ;
  if (rotY < -0.3) calY = calY + 10 ;
  if (rotY > 0.3) calY = calY - 10 ;
  if (rotZ < -0.3) calZ = calZ + 10 ;
  if (rotZ > 0.3) calZ = calZ - 10 ;

  if (rotX < -0.01) calX = calX + 1 ;  // slow change
  if (rotX > 0.01) calX = calX - 1 ;
  if (rotY < -0.01) calY = calY + 1 ;
  if (rotY > 0.01) calY = calY - 1 ;
  if (rotZ < -0.01) calZ = calZ + 1 ;
  if (rotZ > 0.01) calZ = calZ - 1 ;

  rotX = (gyroX + calX) / 131.0;
  rotY = (gyroY + calY) / 131.0;
  rotZ = (gyroZ + calZ) / 131.0; // all should get closer to zero
  // display values on screen
  tft.setCursor (1, 1);
  tft.println("rot X "); tft.print(rotX);
  tft.setCursor (1, 40);
  tft.println("rot Y "); tft.print(rotY);
  tft.setCursor (1, 80);
  tft.println("rot Z "); tft.print(rotZ);
  tft.setCursor (1, 120);
  tft.println("Gforce X "); tft.print(gForceX);
  tft.setCursor (1, 160);
  tft.println("Gforce Y "); tft.print(gForceY);
  tft.setCursor (1, 200);
  tft.println("Gforce Z "); tft.print(gForceZ);
  tft.setCursor (100, 1);
  tft.print("cal X "); tft.print(calX);
  tft.setCursor (100, 40);
  tft.print("cal Y "); tft.print(calY);
  tft.setCursor (100, 80);
  tft.print("cal Z "); tft.print(calZ);
  tft.setCursor (300, 1);
  tft.print("gyro X "); tft.print(gyroX);
  tft.setCursor (300, 40);
  tft.print("gyro Y "); tft.print(gyroY);
  tft.setCursor (300, 80);
  tft.print("gyro Z "); tft.print(gyroZ);

  // check if completed :
  //if ((abs(rotX) < 0.02) && (abs(rotY) < 0.02) && (abs(rotZ) < 0.02) && (gyroaligned == false)) {
  if ((abs(rotX) < 0.02) && (abs(rotY) < 0.02) && (gyroaligned == false)) {  
    // note : 0.02 is the adjustement sensitivity
    // if calibration cannot be reached, increase this value (up to 0.3)
    gyroaligned = true;
    tft.setTextColor(GREEN, TFT_GREY);
    tft.drawCentreString("DONE !", CENTRE, 200, 1);
    delay (2000); // pause to display result
    tft.fillScreen(TFT_BLACK);// erase display 
  }

  // calX is the compensation which will be used to zero the drift
  // rotation rate is in degrees/second
}

void complimentaryfilter() {
  // filter using 98% gyro data and 2% gravity data
  // angle = 0.98 * (angle + gyroData*dt) + 0.02 Acc data
  newmillis = millis();
  dt = newmillis - lastmillis ; // dt in milliseconds
  lastmillis = newmillis ;
  angleX = angleX + (0.98 * (rotX * dt / 1000)) + 0.02 * (gForceY );// angleX displays the roll indication, positive for roll to the left
 // angleX =  -20;
  angleY = angleY + (0.98 * (rotY * dt / 1000) + 0.02 * (gForceX ));// angleY displays the pitch indication
 // angleY = 10 ;
}
void displaypfd () {  // normal display after calibration
  tft.setCursor (1, 1); // ********************* can be removed, just for check
  tft.println("roll"); tft.print(angleX); // remove
//  tft.setCursor (1, 40);
//  tft.println("rot Y "); tft.print(rotY);
//  tft.setCursor (1, 80);
//  tft.println("rot Z "); tft.print(rotZ);
//  tft.setCursor (1, 80);
//  tft.println("Gforce X "); tft.print(gForceX);
//  tft.setCursor (1, 120);
//  tft.println("Gforce Y "); tft.print(gForceY);
//  tft.setCursor (1, 200);
//  tft.println("Gforce Z "); tft.print(gForceZ);
//  tft.setCursor (1,1);  tft.println("ang X"); tft.print(angleX);tft.print(" ");
//  tft.setCursor (1,40);  tft.println("dt  "); tft.print(dt); tft.print(" ");
//  tft.setCursor (140, 80);  tft.print("(rotX * dt / 1000)  "); tft.print((rotX * dt / 1000)); tft.print("   ");
//  tft.setCursor (140, 120);  tft.print("0.02 * (gForceY )  "); tft.print(0.02 * (gForceY )); tft.print("   "); 
// tft.setCursor (1,80);  tft.println("ang Y"); tft.print(angleY);tft.print(" ");
// tft.setCursor (1, 170);  tft.println("d1"); tft.print(d1);tft.print(" ");
// tft.setCursor (1, 200);  tft.println("d2"); tft.print(d2);tft.print(" ");
//  tft.setCursor (1, 250);
//  tft.print("y1 "); tft.println(y1);
//  tft.setCursor (1, 270);
//  tft.print("y2 "); tft.println(y2);
// uint16_t x0 = 80, y0 = 0, wid = 320, heig = 312, rad = 80; //
//  uint16_t x1 = x0 + wid; // col =  ; // top left corner, width, height, radius, colour
//  uint16_t y1 = 150 ; y2 = 150 ; // used for display attitude
 d1=160 * tan( angleX*0.017453) ; // to convert to radians, x PI / 180=0.017453293  d1 is divided by 2 : 160*2 = 320, height of the display
  if (angleX < 0) d1=-d1;
 d2=160 * tan( angleY*0.017453) ;
 y1 = 160 - d1-d2 ; // 160 is center of the display height
 y2 = 160 + d1-d2 ;
 // x1 = 80; x2= x1+wid = 340; // not changed 

  tft.fillRect(x0, 0, wid, y1, TFT_CYAN); // draw upper part of the sky - a box
  if (angleX >=0) {tft.fillTriangle(x0, y1, x0, y2, x1, y1, TFT_CYAN);tft.fillTriangle(x0, y2, x1, y1, x1, y2, YELLOW);} // draw blue triangular part of the sky and draw amber triangular part of earth
  if (angleX < 0) {tft.fillTriangle(x0, y1, x1, y1, x1, y2, TFT_CYAN);tft.fillTriangle(x0, y1, x0, y2, x1, y2, YELLOW);} 
  tft.fillRect(x0, y2, wid, (320-y2) , YELLOW); // draw lower part of the earth
  
  tft.drawRect(236,156,8,8,BLACK); // optional symbol (center box)
    };

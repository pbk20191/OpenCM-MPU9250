// I2C device class (I2Cdev) demonstration Arduino sketch for MPU9250 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU9250 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project

//#include "MPU9250.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h

#include <Wire.h>
#include <I2Cdev.h>
#include <helper_3dmath.h>
#include <MPU9250.h>
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU9250 mpu;
//MPU9250 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-9250's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */
//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
// #define OUTPUT_READABLE_YAWPITCHROLL
// #define OUTPUT_TEAPOT  // processing
// #define Serialchart 
//#define Serialplotter
//#define Debug

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)


// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)



// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo


#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#include <RC100.h>
#define YAWmotor  5
#define PITCHmotor 9
#define ROLLmotor 13
#define FR 4
#define FL 3
#define BR 2
#define BL 1
#define speed 512
#define L_Button1 16
#define R_Button2 17
#define R_LED1 18
#define G_LED2 19
#define B_LED3 20

Dynamixel AX(3);

RC100 Controller;

boolean motor=false;
boolean blinkState = HIGH;
boolean over_flow=HIGH;
// MPU control/status vars
boolean dmpReady = false;  // set true if DMP init was successful
uint8 mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8 devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16 packetSize;    // expected DMP packet size (default is 42 bytes)
uint16 fifoCount;     // count of all bytes currently in FIFO
uint8 fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yaw,pitch,roll;
float zeropoint[3]= {0.0f, 0.0f, 0.0f }; 
float motorangle[3];
// packet structure for InvenSense teapot demo
uint8 teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile boolean mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)

        Wire.begin(0,1); //SDA,SCL
   AX.begin(3);
   Controller.begin(1);
        AX.goalPosition(YAWmotor,512);
        AX.goalPosition(PITCHmotor,512);
        AX.goalPosition(ROLLmotor,512);


    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)


//Serial1.begin(115200);


#ifdef Debug    
    SerialUSB.begin();

while(!SerialUSB.available());

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    SerialUSB.println("Initializing I2C devices...");
#endif

    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

#ifdef Debug
    // verify connection
    SerialUSB.println("Testing device connections...");
    SerialUSB.println(mpu.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");
  #ifndef Serialplotter
    // wait for ready
    SerialUSB.println("\nSend any character to begin DMP programming and demo: ");
    while (SerialUSB.available() && SerialUSB.read()); // empty buffer
    while (!SerialUSB.available());                 // wait for data
  #endif
    while (SerialUSB.available() && SerialUSB.read()); // empty buffer again

    // load and configure the DMP
    SerialUSB.println("Initializing DMP...");
#endif   
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
#ifdef Debug        
        SerialUSB.println("Enabling DMP...");
#endif        
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
#ifdef Debug        
        SerialUSB.println("Enabling interrupt detection (Arduino external interrupt 0)...");
#endif
        attachInterrupt(INTERRUPT_PIN, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
#ifdef Debug
        SerialUSB.println("DMP ready! Waiting for first interrupt...");
#endif
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
#ifdef Debug
        SerialUSB.print("DMP Initialization failed (code ");
        SerialUSB.print(devStatus);
        SerialUSB.println(")");
#endif        
    }
    // configure LED for output
    pinMode(BOARD_LED_PIN, OUTPUT);
    pinMode(R_LED1,OUTPUT);
   digitalWrite(R_LED1,over_flow);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) { //SerialUSB.println("WHAT");
        // other program behavior stuff here
        // .
  if(Controller.available())
  {
    switch(Controller.readData())
    {
      case RC100_BTN_U: {forward(); break;  }
      case RC100_BTN_D: {backward(); break;  }
      case RC100_BTN_L: {leftward(); break;  }
      case RC100_BTN_R: {rightward(); break;  } 
      case RC100_BTN_6: {turnleft(); break;  }  
      case RC100_BTN_5: {turnright(); break;  }
      case RC100_BTN_1: { zeropoint[0] = yaw;  break;  }
      case RC100_BTN_2: { zeropoint[0]=yaw; zeropoint[1] = pitch; zeropoint[2] = roll; break;  }
      case ((RC100_BTN_3)+(RC100_BTN_4)): {   motor=!motor; break;  }
      case ((RC100_BTN_U)+(RC100_BTN_L)): {forleftward(); break;  }
      case ((RC100_BTN_U)+(RC100_BTN_R)): {forrightward(); break;  }
      case ((RC100_BTN_D)+(RC100_BTN_L)): {backleftward(); break;  }
      case ((RC100_BTN_D)+(RC100_BTN_R)): {backrightward(); break;  }
      default: {stop(); break;}
    }
  } 
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        digitalWrite(R_LED1,!over_flow);
#ifdef Debug        
        SerialUSB.println("FIFO overflow!");
#endif        

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
yaw=atan2(2.0f*(q.x*q.y+q.w*q.z),1-2.0f*(q.y*q.y+q.z*q.z))* 180/M_PI;
pitch=asin(2.0f*(q.w*q.y-q.x*q.z))* 180/M_PI;
roll=atan2(2.0f*(q.w*q.x+q.y*q.z),1-2.0f*(q.x*q.x+q.y*q.y))* 180/M_PI;

if(motor)
{
   motorangle[0]=yaw-zeropoint[0];
    if(motorangle[0]>180){motorangle[0]-=360.0f;}
    if(motorangle[0]<-180){motorangle[0]+=360.0f;}
AX.goalPosition(YAWmotor,1023-motormap(motorangle[0])); 
motorangle[1]=pitch-zeropoint[1];
    if(motorangle[1]>90){motorangle[1]-=180.0f;}
    if(motorangle[1]<-90){motorangle[1]+=180.0f;}
AX.goalPosition(PITCHmotor,motormap(motorangle[1]));
motorangle[2]=roll-zeropoint[2];
    if(motorangle[2]>180){motorangle[2]-=360.0f;}
    if(motorangle[2]<-180){motorangle[2]+=360.0f;}
AX.goalPosition(ROLLmotor,motormap(motorangle[2]));
}




#ifdef Debug
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
          //  mpu.dmpGetQuaternion(&q, fifoBuffer);
            SerialUSB.print("quat\t");
            SerialUSB.print(q.w);
            SerialUSB.print("\t");
            SerialUSB.print(q.x);
            SerialUSB.print("\t");
            SerialUSB.print(q.y);
            SerialUSB.print("\t");
            SerialUSB.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
         //   mpu.dmpGetQuaternion(&q, fifoBuffer);
         //   mpu.dmpGetEuler(euler, &q);
            SerialUSB.print("euler\t");
            SerialUSB.print(euler[0] * 180/M_PI);
            SerialUSB.print("\t");
            SerialUSB.print(euler[1] * 180/M_PI);
            SerialUSB.print("\t");
            SerialUSB.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
       //     mpu.dmpGetQuaternion(&q, fifoBuffer);
      //      mpu.dmpGetGravity(&gravity, &q);
      //      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            SerialUSB.print("ypr\t");
            SerialUSB.print(ypr[0] * 180/M_PI);
            SerialUSB.print("\t");
            SerialUSB.print(ypr[1] * 180/M_PI);
            SerialUSB.print("\t");
            SerialUSB.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
     //       mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            SerialUSB.print("areal\t");
            SerialUSB.print(aaReal.x);
            SerialUSB.print("\t");
            SerialUSB.print(aaReal.y);
            SerialUSB.print("\t");
            SerialUSB.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
     //       mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            SerialUSB.print("aworld\t");
            SerialUSB.print(aaWorld.x);
            SerialUSB.print("\t");
            SerialUSB.print(aaWorld.y);
            SerialUSB.print("\t");
            SerialUSB.println(aaWorld.z);
        #endif


        #ifdef OUTPUT_TEAPOT //processing
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
  	  SerialUSB.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        #ifdef Serialchart
      //      mpu.dmpGetQuaternion(&q, fifoBuffer);
         //   mpu.dmpGetEuler(euler, &q);
            SerialUSB.print(yaw,2);
            SerialUSB.print(',');
            SerialUSB.print(pitch,2);
            SerialUSB.print(',');
            SerialUSB.print(roll,2);
            SerialUSB.println();
        #endif

        #ifdef Serialplotter
     //       mpu.dmpGetQuaternion(&q, fifoBuffer);
   //         mpu.dmpGetEuler(euler, &q);
            SerialUSB.print(180);
            SerialUSB.print(" ");
            SerialUSB.print(-180);
            SerialUSB.print(" ");
            SerialUSB.print(yaw,2);
            SerialUSB.print(" ");            
            SerialUSB.print(pitch,2);
            SerialUSB.print(" ");            
            SerialUSB.print(roll,2);
            SerialUSB.println();
        #endif
 #endif
         // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(BOARD_LED_PIN, blinkState);

    }
}







void stop()
{
    AX.goalSpeed(FL, 0);          
    AX.goalSpeed(BR, 0); 
    AX.goalSpeed(BL, 0);          
    AX.goalSpeed(FR, 0);
}

void forward()
{
  AX.goalSpeed(FR,(speed|0x400));
  AX.goalSpeed(FL,speed);
  AX.goalSpeed(BR,(speed | 0x400));
  AX.goalSpeed(BL,speed);
}

void backward()
{
    AX.goalSpeed(FR,speed);
    AX.goalSpeed(FL,(speed|0x400));
    AX.goalSpeed(BR,speed);
    AX.goalSpeed(BL,speed|0x400);
  
}

void rightward()
{
    AX.goalSpeed(FR,speed|0x400);
    AX.goalSpeed(FL,speed);
    AX.goalSpeed(BR,speed);
    AX.goalSpeed(BL,speed|0x400);
}

void leftward()
{
    AX.goalSpeed(FR,speed);
    AX.goalSpeed(FL,speed|0x400);
    AX.goalSpeed(BR,speed|0x400);
    AX.goalSpeed(BL,speed);
}

void forleftward()
{
    AX.goalSpeed(FL, 0);          
    AX.goalSpeed(BR, speed|0x400); 
    AX.goalSpeed(BL, speed);          
    AX.goalSpeed(FR, 0);
}

void backrightward()
{
    AX.goalSpeed(FL, 0);          
    AX.goalSpeed(BR, speed); 
    AX.goalSpeed(BL, speed|0x400);          
    AX.goalSpeed(FR, 0);
}

void forrightward()
{
    AX.goalSpeed(FL, speed);          
    AX.goalSpeed(BR, 0); 
    AX.goalSpeed(BL, 0);          
    AX.goalSpeed(FR, speed|0x400);
}
void backleftward()
{
    AX.goalSpeed(FL, speed|0x400);          
    AX.goalSpeed(BR, 0); 
    AX.goalSpeed(BL, 0);          
    AX.goalSpeed(FR, speed);
}

void turnleft()
{
    AX.goalSpeed(FL, speed|0x400);          
    AX.goalSpeed(BR, speed|0x400); 
    AX.goalSpeed(BL, speed|0x400);          
    AX.goalSpeed(FR, speed|0x400);
}

void turnright()
{
    AX.goalSpeed(FL, speed);          
    AX.goalSpeed(BR, speed); 
    AX.goalSpeed(BL, speed);          
    AX.goalSpeed(FR, speed);
}

int motormap(float angle)
{
int motor;	   angle=constrain(angle,-150,150);  motor=map(angle,-150,150,0,1023);
return motor;
}



//--------------------------------------------------------------
//--    BNO055
//--    IMU 9DOF mySensor Fusion
//--------------------------------------------------------------
//--    BQ
//--------------------------------------------------------------
//--    BNO055 Library functions cheat sheet by
//--        Alvaro Ferran (alvaroferran)
//--------------------------------------------------------------
//--    Released on September 2015
//--    under the GPL v2
//--------------------------------------------------------------


#include "BNO055.h"
#include <Wire.h>

#define A 0X28  //I2C address selection pin LOW
#define B 0x29  //                          HIGH

BNO055 mySensor(A);


void setup(){
    Wire.begin();
    Serial.begin(9600);
    mySensor.init();
}


void loop(){
    //Euler angles
    mySensor.readEul();
    Serial.print("Yaw (dg): "); Serial.print(mySensor.euler.x); Serial.print("  Roll (dg): "); Serial.print(mySensor.euler.y); Serial.print("  Pitch (dg): "); Serial.println(mySensor.euler.z);

    //Quaternions
    // mySensor.readQuat();
    // Serial.print("Q0: "); Serial.print(mySensor.quat.q0); Serial.print("  Q1: "); Serial.print(mySensor.quat.q1); Serial.print("  Q2: "); Serial.print(mySensor.quat.q2); Serial.print("  Q3: "); Serial.println(mySensor.quat.q3);

    //Accelerometer
    // mySensor.readAccel();
    // Serial.print("X (mg): "); Serial.print(mySensor.accel.x); Serial.print("  Y (mg): "); Serial.print(mySensor.accel.y); Serial.print("  Z (mg): "); Serial.println(mySensor.accel.z);

    //Gyroscope
    // mySensor.readGyro();
    // Serial.print("X (dg/s): "); Serial.print(mySensor.gyro.x); Serial.print("  Y (dg/s): "); Serial.print(mySensor.gyro.y); Serial.print("  Z (dg/s): "); Serial.println(mySensor.gyro.z);

    //Compass
    // mySensor.readMag();
    // Serial.print("X (mG): "); Serial.print(mySensor.mag.x); Serial.print("  Y (mG): "); Serial.print(mySensor.mag.y); Serial.print("  Z (mG): "); Serial.println(mySensor.mag.z);

    //Linear acceleration
    // mySensor.readLinAcc();
    // Serial.print("X (mg): "); Serial.print(mySensor.linAcc.x); Serial.print(" Y (mg): "); Serial.print(mySensor.linAcc.y); Serial.print(" Z (mg): "); Serial.println(mySensor.linAcc.z);

    //Gravity
    // mySensor.readGrav();
    // Serial.print("X (mg): "); Serial.print(mySensor.grav.x); Serial.print(" Y (mg): "); Serial.print(mySensor.grav.y); Serial.print(" Z (mg): "); Serial.println(mySensor.grav.z);

    //Temperature
    // mySensor.readTemp();
    // Serial.print("Temp (C): "); Serial.print(mySensor.temp.c);Serial.print(" Temp (F): "); Serial.println(mySensor.temp.f);

    //Linear acceleration independent of orientation
    // mySensor.readAbsAcc();
    // Serial.print("X (mg): "); Serial.print(mySensor.absAcc.x); Serial.print(" Y (mg): "); Serial.print(mySensor.absAcc.y); Serial.print(" Z (mg): "); Serial.println(mySensor.absAcc.z);

    //Dead Reckoning
    // mySensor.deadReckoning();    //Uses world coordinates, x component of acceleration will be in the user's x axis, independently of orientation
    // mySensor.deadReckoning(1);   //Uses local coordinates, x component of acceleration will be in the sensor's x axis
    // Serial.print("Location X : "); Serial.print(mySensor.position.x); Serial.print(" Location Y: "); Serial.print(mySensor.position.y); Serial.print(" Location Z: "); Serial.println(mySensor.position.z);
    // Serial.print("Orientation Q0: "); Serial.print(mySensor.position.q0); Serial.print(" Orientation  Q1: "); Serial.print(mySensor.position.q1); Serial.print(" Orientation  Q2: "); Serial.print(mySensor.position.q2); Serial.print(" Orientation  Q3: "); Serial.println(mySensor.position.q3);
}

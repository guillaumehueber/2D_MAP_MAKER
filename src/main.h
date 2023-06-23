#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <camera/dcmi_camera.h>
#include <msgbus/messagebus.h>
#include <parameter/parameter.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ch.h>
#include <hal.h>
#include <memory_protection.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <motors.h>
#include <arm_math.h>

#include "sensors/imu.h"
#include "sensors/mpu9250.h"
#include "sensors/VL53L0X/VL53L0X.h"

//Constants for accelerometer to turn raw data into usable acceleration values
#define STANDARD_GRAVITY    9.80665f            //Gravity constant
#define RES_2G		2.0f                        //The MPU is set up for the accelerometer to give a resolution of +/- 2G
#define RES_250DPS	250.0f                      //The MPU is set up for the gyroscope  to give a resolution of +/- 250DPS
#define MAX_INT16	32768.0f                    //Max value for and integer of 16bits
#define ACC_RAW2G	 (RES_2G / MAX_INT16)	    //2G scale for 32768 raw value
#define GYRO_RAW2DPS (RES_250DPS / MAX_INT16)	//250DPS (degrees per second) scale for 32768 raw value
#define DEG_360 360
#define WAIT_STABLE_MS 2000                     //Wait this time at startup before 
#define TIME_PERIOD_MS 10                       //Integration period and states at what frequency the threads update

//Standard deviation values Accelerometer Constants. These values don't have units
#define ACC_X_STDEV 37                          //Standard deviation for the X axis for acceleration
#define ACC_Y_STDEV 35                          //Standard deviation for the Y axis for acceleration
#define GYRO_Z_STDEV 18                         //Standard deviation for the Z axis for gyroscope

//Constants for the TOF(Time of Flight) distance sensor
#define DIST_OFFSET 40                          //The TOF gives a value of 40mm when it should give 0mm
#define DIST_MAX 2000                           //Max distance measured with acceptable precision

//Constants to know how sensitive the robot is to acceleration and the distance it has to move
#define ACC_MOVED_THR 10                        //Detects movement above this threshold [m/s^2]
#define DISP_X 270                              //Board distance to move in the X axis
#define DISP_Y 155                              //Board distance to move in the Y axis

//Constant for the motor
#define MOTOR_SPEED 150

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;



#ifdef __cplusplus
}
#endif

#endif

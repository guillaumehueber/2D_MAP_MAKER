#ifndef COMPUTE_IMU_H
#define COMPUTE_IMU_H

#include "main.h"

struct acc_data{
    int16_t avg;
    uint8_t stdev;
    float clean_acc;
};
//Necessary data to clean gyro data
struct gyro_data{
    int16_t avg;
    uint8_t stdev;
    float clean_rate;
    float raw_angle;
    float angle;
};

//Enum to know what we want from the getter function
typedef enum{
    ACC = 0,
    GYRO_CLEAN,
    ANGLE_RAW,
    ANGLE_CLEAN,
}imu_acc_or_gyro;


//Getter function to get global static values from compute_imu.c to motor_controller.c
float getter_imu_values(imu_axis_t axis, imu_acc_or_gyro condition);

//Starts the Inertial Motion Unit (IMU), calibrates it and the values with it.
void init_imu_and_values(void);

//Used to recalibrate IMU by recalculating a new average when we finished a turn
//Takes an average on 50 values so the calibration process should take 200ms
//This function is also used to prevent an overflow on the raw angle
void recalibrate_IMU(void);


#endif /* COMPUTE_IMU_H */
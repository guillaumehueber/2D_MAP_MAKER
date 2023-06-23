#include <ch.h>
#include <hal.h>
#include <stdio.h>
#include <stdlib.h>
#include <arm_math.h>

#include "main.h"
#include "compute_imu.h"

//Initialise the global static variables for the message bus for the IMU values
static messagebus_topic_t *imu_topic;
static imu_msg_t imu_values;

//Set as static global variables as they're used by more than two function in this file
//They also have values we need in motor_controller.c so we use the getter.
static struct acc_data acc[NB_AXIS];
static struct gyro_data gyro[NB_AXIS];

static void timer12_start(void){
    //General Purpose Timer configuration   
    //timer 12 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD12, &gpt12cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD12, 0xFFFF);
}


float deg2rad(float deg){
    return (deg / DEG_360) * (2 * PI);
}

float rad2deg(float rad){
    return (rad * DEG_360) / (2 * PI);
}

void recalibrate_IMU(void){
    // Calibrate IMU.
	calibrate_acc();
	calibrate_gyro();
    //Get offset from the average we made in the IMU calibration
	for(int i = 0; i < NB_AXIS; i++){
        acc[i].avg = get_acc_offset(i);
        gyro[i].avg = get_gyro_offset(i);
        if(gyro[i].raw_angle > DEG_360) gyro[i].raw_angle -= DEG_360; //To prevent eventual overflow 
    }
}

//Clean values using soft threshold from standard deviation found earlier
float clean_values(int16_t raw, int avg, int8_t stdev){
    
    raw -= avg;
    if(abs(raw) < stdev)    raw = 0;
    else if(raw > 0)        raw -= stdev;
    else                    raw += stdev;
    return raw;
}

//Get Data from IMU: acceleration and gyro data 
float getter_imu_values(imu_axis_t axis, imu_acc_or_gyro condition){

    if(condition == ACC)            return acc[axis].clean_acc;
    if(condition == GYRO_CLEAN)     return gyro[axis].angle;
    if(condition == ANGLE_CLEAN)    return gyro[axis].angle;
    if(condition == ANGLE_RAW)      return gyro[axis].raw_angle;
    if(condition == GYRO_CLEAN)     return gyro[axis].clean_rate;
    else                            return 0;
}

static THD_WORKING_AREA(clean_acc_gyro_WA, 256);

static THD_FUNCTION(clean_acc_gyro, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    
    //time variables to measure the time the thread takes
    //volatile to not be optimized out by the compiler if not used
    volatile uint16_t time, time_thread_imu;

    while(TRUE){
        time = chVTGetSystemTime();
        time_thread_imu = 0;

        //Wait for new values on the IMU message bus
        messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));

        chSysLock();
        //reset the timer counter
        GPTD12.tim->CNT = 0;
        chSysUnlock();

        for(int i = 0; i < NB_AXIS; i++){
            
            //Get acceleration values and clean them.

            acc[i].clean_acc = clean_values(get_acc(i), acc[i].avg, acc[i].stdev) * STANDARD_GRAVITY * ACC_RAW2G;
            
            //Get rotation values and clean them.

            gyro[i].clean_rate = clean_values(get_gyro(i), gyro[i].avg, gyro[i].stdev) * deg2rad(GYRO_RAW2DPS);
            gyro[i].clean_rate = rad2deg(gyro[i].clean_rate);
            //Changing gyro_rate from rad/s to deg/s
            //Integrate gyro_rate to get the angle
            //Divide by 1000 as time is in mS and rate is in deg/s
            gyro[i].raw_angle += (gyro[i].clean_rate * TIME_PERIOD_MS) / 1000;

            //Keep angle in between [0; 360]
            gyro[i].angle = fmodf(gyro[i].raw_angle, DEG_360); //Using modulo function for float values       
            if(gyro[i].angle < 0) gyro[i].angle += DEG_360;
        }

        //Make sure the thread function doesn't take longer than TIME_PERIOD_MS
        //Otherwise, we integrate over less time than really takes place.
        //Last time checked, it took 3 mS. So 7ms free.
        chSysLock();
        time_thread_imu = GPTD12.tim->CNT;
        chSysUnlock();

        if(time_thread_imu > (TIME_PERIOD_MS * 1000)){
            chprintf((BaseSequentialStream *)&SD3, "Warning ! Main function takes longer than integration period !\r\n");
        }

        //Call back this thread 10ms later
        chThdSleepUntilWindowed(time, time + MS2ST(TIME_PERIOD_MS)); //reduced the sample rate to 100Hz
    }
}


void init_imu_and_values(){

    //starts timer 12
    timer12_start();
    //inits mpu9250
	mpu_init();
    // inits imu
	imu_start();

    //Update imu_topic for its message bus. 
    //This is defined as a static global variable as used in two functions but only in compute_imu.c
    imu_topic = messagebus_find_topic_blocking(&bus, "/imu");

	//Wait 2 sec to be sure the e-puck is in a stable position
    chThdSleepMilliseconds(WAIT_STABLE_MS);
    recalibrate_IMU();

    //Initialise all values to 0;
	for(int i = 0; i < NB_AXIS; i++){
        acc[i].avg = 0;
        acc[i].stdev = 0;
        acc[i].clean_acc = 0;

        gyro[i].avg = 0;
        gyro[i].stdev = 0;
        gyro[i].clean_rate = 0;
        gyro[i].raw_angle = 0;
        gyro[i].angle = 0;
    }

    //Input standard deviation values calculated on the computer for the soft threshold
    acc[X_AXIS].stdev = ACC_X_STDEV;
    acc[Y_AXIS].stdev = ACC_Y_STDEV;
    gyro[Z_AXIS].stdev = GYRO_Z_STDEV;

    //Start thread to get raw values from acc and gyro, clean them and compute what is needed
    chThdCreateStatic(clean_acc_gyro_WA, sizeof(clean_acc_gyro_WA), NORMALPRIO, clean_acc_gyro, NULL);
}
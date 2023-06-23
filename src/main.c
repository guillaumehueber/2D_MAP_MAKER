///*************************************************************///
//  ------------------- 2DMapMaker Project -------------------
//         Written by Guillaume HUEBER and Arnaud TADIC
//
// Cleans IMU values to compute angle and acceleration.
// Gets the front distance from the TOF sensor.
// Turns 360° with the motors every time robot detects new pos.
// Sends all this data back to a Python program which plots the map.
// All this code runs on the RTOS ChibiOS with multiple threads
//
// Main function initialises the ChibiOS, the message bus as
// well as the IMU, motors and their respective threads.
//
///*************************************************************///

#include "main.h"
#include "communications.h"
#include "compute_imu.h"
#include "motor_controller.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static void serial_start(void){
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}


int main(void){

    halInit();
    // Starts ChibiOS system
    chSysInit(); // Has to be done BEFORE starting new threads !

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar); // Has to be done BEFORE the init of sensors using the message bus !
    
    //Starts the serial communication
    serial_start();
    //Starts the USB communication
    usb_start();
    
    //Inits the imu, values and thread as well as timer
    init_imu_and_values();
    //Inits the motors, values and motor control thread
    init_motor_controller();


    /* Infinite loop. */
    while (1) {
        
        //chprintf((BaseSequentialStream *)&SD3, "Gyro rate: %1.f, Raw angle: %1.f, Angle: %.1f\r\n", getter_imu_values(Z_AXIS, GYRO_CLEAN),  getter_imu_values(Z_AXIS, ANGLE_RAW), getter_imu_values(Z_AXIS, ANGLE_CLEAN));

        chThdSleepMilliseconds(100);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void){
    chSysHalt("Stack smashing detected");
}
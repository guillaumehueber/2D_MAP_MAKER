#include "main.h"
#include "communications.h"
#include "compute_imu.h"
#include "motor_controller.h"

void robot_stop(void){
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}
void robot_turn(void){
	left_motor_set_speed(-MOTOR_SPEED);
	right_motor_set_speed(MOTOR_SPEED);
}

float clean_distance(void){
	//Get distance (which is signed to prevent underflow from substracting the offset)
	int16_t distance;
	distance = VL53L0X_get_dist_mm() - DIST_OFFSET;
	if(distance < 0) return 0;  //Do this to prevent negative distance 
	if(distance > DIST_MAX) return DIST_MAX; 
	return distance;
}


static THD_WORKING_AREA(motor_controller_WA, 256);

static THD_FUNCTION(motor_controller, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
	
	//volatile to not be optimized out by the compiler if not used
    volatile uint16_t time_thread_motors;

    bool moved = TRUE;
    bool start_angle_set = FALSE;
    float start_angle = 0;

	//Initialise to 0 all the values for angle and distance.
    struct dist_rot values_for_map;
	values_for_map.pos_x = 0;
    values_for_map.pos_y = 0;
    values_for_map.angle = 0;
    values_for_map.distance = 0;

    float acc[NB_AXIS];

    while(TRUE){
        time_thread_motors = chVTGetSystemTime();

		//Get x-y position from acceleration, without integrating
        //See if its above a threshold but only look for a start and then wait
        if (moved == FALSE){
			//Get values from getter before entering in the for loop to get the same values
			//instead of getting new ones when we call the getter
			acc[X_AXIS] = getter_imu_values(X_AXIS, ACC);
			acc[Y_AXIS] = getter_imu_values(Y_AXIS, ACC);

            for(int i = 0; i < 2; i++){
                if (acc[i] > ACC_MOVED_THR){ // Means we moved in positive direction (for X)
                    if (i == X_AXIS) values_for_map.pos_x += DISP_X;
                    else values_for_map.pos_y -= DISP_Y;// Means we moved in negative direction (for Y)
                    start_angle_set = FALSE;
                    moved = TRUE;
                }
                else if (acc[i] < - ACC_MOVED_THR){ // Means we moved in negative direction (for X)
                    if (i == X_AXIS) values_for_map.pos_x -= DISP_X;
                    else values_for_map.pos_y += DISP_Y; // Means we moved in positive direction (for Y)
                    start_angle_set = FALSE;
                    moved = TRUE;
                }
            }
        }

		//TOF distance value updates only every 100ms as the TOF thread updates only every 100ms        
        values_for_map.distance = clean_distance();
        values_for_map.angle = getter_imu_values(Z_AXIS, ANGLE_CLEAN);

        //If we moved, we can turn
        if (moved){
            if(!start_angle_set){
                start_angle = getter_imu_values(Z_AXIS, ANGLE_RAW);
                start_angle_set = TRUE;
                robot_turn();
            }

            if(getter_imu_values(Z_AXIS, ANGLE_RAW) < (start_angle + DEG_360)){
                SendFloatToComputer((BaseSequentialStream *) &SD3, &values_for_map.pos_x, 4);
            }
            else{
                robot_stop();
				recalibrate_IMU();
                moved = FALSE;
            }
        }
        //Call back this thread 10ms later
        chThdSleepUntilWindowed(time_thread_motors, time_thread_motors + MS2ST(TIME_PERIOD_MS)); 
    }
}

void init_motor_controller(){
	//inits the motors
    motors_init();
	//Start the TOF distance sensor
    VL53L0X_start();
	//Create Thread
    chThdCreateStatic(motor_controller_WA, sizeof(motor_controller_WA), NORMALPRIO, motor_controller, NULL);
}
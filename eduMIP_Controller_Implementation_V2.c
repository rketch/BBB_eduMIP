/** File  Name: eduMIP_Controller_Implementation_V2.c
 *
 *
 * Author: Robert Ketchum (skeleton of file from Robot Control Library at strawsondesign.com)
 *
 * Last Updated: 26/7/2019 -- Cleaned up code, fixed syntax
 *
 * This code balances an eduMIP inverted pendulum robot using an onboard
 * BeagleBone Blue with two controllers, which were created in Matlab.
 *
 * To compile:
 * gcc -Wall eduMIP_Controller_Implementation_V2.c -o eduMIP_Controller_Implementation_V2 -lm -lrt -l:librobotcontrol.so.1
 *
 * To run:
 * sudo ./eduMIP_Controller_Implementation_V2.c
 *
 *
 **/

//Libraries to Include
#include <stdio.h>
#include <math.h>
#include <robotcontrol.h>

// function declarations
void on_pause_press();
void on_pause_release();
static void imu_interupt_function(void); //mpu interrupt routine
static void* my_thread_2(void* angl1);   //background thread

//variable declerations
static rc_mpu_data_t mpu_data; //mpu data

double raw_gyro              = 0;         //gyro data
double gyro_angle[2]         = {0, 0};
double filtered_gyro         = 0;
double filtered_acc          = 0;
double current_angle         = 0;         //complementary filter output. robot pitch angle
double raw_accel             = 0;         //accelerometer pitch angle in [rad]
double pitch_err[3]          = {0, 0, 0}; //robot pitch error angle with relation to vertical
double uk[3]                 = {0, 0, 0}; //duty cycle
double pitch_reference       = 0;         //robot reference angle. Output of controller 1
int    encoder_reading[2]    = {0, 0};    //wheel angle
double wheel_angle[3]        = {0, 0, 0}; //third is average of the two wheels
double ref_wheel_position[3] = {0, 0, 0}; //for outer loop controller
double wheel_error[3]        = {0, 0, 0}; //wheel position error


//controller value declarations
const double inner_den[2]  = {1.538460787577220, -0.538460787577220};
const double inner_num[3]  = {-8.682690225552795, 15.326915352200393, -6.759610173791212};
const double outer_den[2]  = {1.257775576649003, -0.373344081217810};
const double outer_num[3]  = {0.005057780550182, 0.000246771998252, -0.004811008551930};
const double PI            = 3.1415926;
const double WC            = 4;        //complimentary filter cutoff fz [rad/sec]
const double K1            = 0.3;      //inner loop gain
const double REV           = 341.8648; //encoder value for 1 wheel revolution
const double INNER_LOOP_DT = 0.005;    //inner loop step size
const double OUTER_LOOP_DT = 50000;    //outer loop step size in microseconds
const double OFFSET = 0.3457;          //balanced robot angle offset from vertical


#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN 21



int main()
{
        pthread_t thread2 = 0;

        // make sure another instance isn't running
        if(rc_kill_existing_process(2.0)<-2) return -1;

        // start balance stack to control setpoints
        if(rc_pthread_create(&thread2, my_thread_2, (void*) NULL, SCHED_OTHER,  0)){
                fprintf(stderr, "failed to start thread2\n");
                return -1;
        }

        // start signal handler so we can exit cleanly
        if(rc_enable_signal_handler()==-1){
                fprintf(stderr,"ERROR: failed to start signal handler\n");
                return -1;
        }

        // initialize pause button
        if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
                                                RC_BTN_DEBOUNCE_DEFAULT_US)){
                fprintf(stderr,"ERROR: failed to initialize pause button\n");
                return -1;
        }

        //initialize encoder
        if(rc_encoder_eqep_init()){
                fprintf(stderr,"ERROR: failed to run rc_encoder_eqep_init\n");
                return -1;
        }

        //initialize mpu
        rc_mpu_config_t conf = rc_mpu_default_config();
        conf.i2c_bus = I2C_BUS;
        conf.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
        conf.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
        conf.dmp_sample_rate = 200; //hz to execute imu interrput
        conf.dmp_fetch_accel_gyro=1;
        conf.orient = ORIENTATION_Y_UP;
        if(rc_mpu_initialize_dmp(&mpu_data, conf)){
                printf("rc_mpu_initialize_failed\n");
                return -1;
        }
        rc_mpu_set_dmp_callback(&imu_interupt_function);
        int freq_hz = RC_MOTOR_DEFAULT_PWM_FREQ;

        //initialize motor
        if(rc_motor_init_freq(freq_hz)) return -1;

        // Assign functions to be called when button events occur
        rc_button_set_callbacks(RC_BTN_PIN_PAUSE,on_pause_press,on_pause_release);

        // make PID file
        rc_make_pid_file();
        printf("\n prints out orientation angles \n");

        // Keep looping until state changes to EXITING
        rc_set_state(RUNNING);
        while(rc_get_state()!=EXITING){
                // change LED colors based on state
                if(rc_get_state()==RUNNING){
                        rc_led_set(RC_LED_GREEN, 1);
                        rc_led_set(RC_LED_RED, 0);
                }
                else{
                        rc_led_set(RC_LED_GREEN, 0);
                        rc_led_set(RC_LED_RED, 1);
                }
                rc_usleep(1000000);
        }

        //turn off LEDs and close file descriptors
        rc_led_set(RC_LED_GREEN, 0);
        rc_led_set(RC_LED_RED, 0);
        rc_led_cleanup();
        rc_button_cleanup();    // stop button handlers
        rc_remove_pid_file();   // remove pid file LAST
        return 0;
}

static void* my_thread_2(__attribute__ ((unused)) void* ptr){
        while(rc_get_state()!=EXITING){

                //find wheel position from encoder value
                encoder_reading[0] = rc_encoder_read(2);                    //read encoder value for each wheel
                encoder_reading[1] = rc_encoder_read(3);
                wheel_angle[0]     = encoder_reading[0];                    //change to double precision
                wheel_angle[1]     = encoder_reading[1];
                wheel_angle[0]     = wheel_angle[0] / REV;                  //change encoder value to radians
                wheel_angle[1]     = -wheel_angle[1] / REV;
                wheel_angle[2]     = (wheel_angle[0] + wheel_angle[1]) / 2; //take average encoder value
                wheel_angle[2]     = wheel_angle[2] - pitch_err[0];         //subtracts off difference between true vertical and balanced angle
                wheel_error[0]     = -wheel_angle[2];                       //wheel angle error--wants to return wheel to its original position

                //outer loop controller--updates robot reference angle for use in inner loop
                ref_wheel_position[0] = outer_den[0]*ref_wheel_position[1]+outer_den[1]*ref_wheel_position[2]+outer_num[0]*wheel_error[0]+outer_num[1]*wheel_error[1]+outer_num[2]*wheel_error[2];

                //update steps
                ref_wheel_position[2] = ref_wheel_position[1];
                ref_wheel_position[1] = ref_wheel_position[0];
                wheel_error[2] = wheel_error[1];
                wheel_error[1] = wheel_error[0];

                printf("wheel error: %6.2f    Pitch Angle: %6.2f      pitch error: %6.2f       duty cycle: %6.2f \n ", wheel_error[0]*RAD_TO_DEG, current_angle, pitch_err[0]*RAD_TO_DEG, uk[0]);
                rc_usleep(OUTER_LOOP_DT);
        }
        return NULL;
}


void imu_interupt_function(void){
        //find spacial position of robot from accelerometer+gyro data

        //accelerometer data
        raw_accel = atan2(mpu_data.accel[1],mpu_data.accel[2]);                                //accelerometer data

        //gyro data
        raw_gyro      = mpu_data.gyro[0] * DEG_TO_RAD;
        gyro_angle[0] = gyro_angle[1] + INNER_LOOP_DT * raw_gyro;                              //find angle using euler's integration method

        //filtering--high pass gyro data, low pass accelerometer data
        double HIGH_PASS = 1 / (2 * PI * INNER_LOOP_DT * WC + 1);                              //generic high pass filter
        double LOW_PASS  = (2 * PI * INNER_LOOP_DT * WC) * HIGH_PASS;                          //generic low pass filter
        filtered_acc  = LOW_PASS * raw_accel + (1 - LOW_PASS) * filtered_acc;                  //filter accel data
        filtered_gyro = HIGH_PASS * filtered_gyro + HIGH_PASS*(gyro_angle[0] - gyro_angle[1]); //filter gyro data
        current_angle = filtered_gyro + filtered_acc;                                          //add filtered angles together
        gyro_angle[1] = gyro_angle[0];

        //find error angle, duty cycle
        current_angle = current_angle - PI/2;                                                  //angle with relation to vertical
        pitch_err[0]  = -ref_wheel_position[0] - current_angle - OFFSET;                                //robot error angle
        if (((pitch_err[0] - pitch_err[1]) > 0.1) || ((pitch_err[1] - pitch_err[0]) > 0.1)){
                uk[0] = 0;                                                                     //prevent wheel motion during discontinuous angle reads
                }
        else {
                //inner loop controller
                uk[0] = K1*(inner_num[0] * pitch_err[0] + inner_num[1] * pitch_err[1] + inner_num[2] * pitch_err[2]) + inner_den[0] * uk[1] + inner_den[1] * uk[2];
                }
        if (uk[0] > 1){
		//max duty cycle
                uk[0] = 1;
        }
        if (uk[0] < -1){
		//max duty cycle
                uk[0] = -1;
        }
        if (pitch_err[0] * RAD_TO_DEG>70){
                //stop wheels if robot knocked over
		uk[0] = 0;
        }
        if (pitch_err[0] * RAD_TO_DEG<-70){
		//stop wheels if robot knocked over
                uk[0] = 0;
        }

        //update steps
        pitch_err[2] = pitch_err[1];
        pitch_err[1] = pitch_err[0];
        uk[2]        = uk[1];
        uk[1]        = uk[0];

        //apply duty cycle
        rc_motor_set(3, uk[0]);
        rc_motor_set(2,-uk[0]);
        return;
}

void on_pause_release()
{
        //Make the Pause button toggle between paused and running states.
        if(rc_get_state()==RUNNING)     rc_set_state(PAUSED);
        else if(rc_get_state()==PAUSED) rc_set_state(RUNNING);
        return;
}


void on_pause_press()
{
/**
* If the user holds the pause button for 0.5 seconds, set state to EXITING which
* triggers the rest of the program to exit cleanly.
**/
        int i;
        const int samples = 100;                    // check for release 100 times in this period
        const int us_wait = 500000;                 // 0.5 seconds
        for(i=0;i<samples;i++){
        // now keep checking to see if the button is still held down
                rc_usleep(us_wait/samples);
                if(rc_button_get_state(RC_BTN_PIN_PAUSE)==RC_BTN_STATE_RELEASED) return;
        }
        printf("long press detected, shutting down\n");
        rc_set_state(EXITING);
        return;
}

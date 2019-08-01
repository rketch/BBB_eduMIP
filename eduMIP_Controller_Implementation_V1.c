/** File  Name: eduMIP_Controller_Implementation_V1.c
 *
 * Author: Robert Ketchum (skeleton of file from Robot Control Library at strawsondesign.com)
 *
 * Last Updated: 1/1/2019
 *
 * This code balances an eduMIP inverted pendulum robot using an onboard
 * BeagleBone Blue with two controllers, which were created in Matlab.
 *
 * To compile:
 * gcc -Wall eduMIP_Controller_Implementation_V1.c -o eduMIP_Controller_Implementation_V1 -lm -lrt -l:librobotcontrol.so.1
 *
 * To run:
 * sudo ./eduMIP_Controller_Implementation_V1
 *
 *
 */

//Libraries to Include
#include <stdio.h>
#include <math.h>
#include <robotcontrol.h>

// function declarations
void on_pause_press();
void on_pause_release();
static void imu_interupt_function(void); //mpu interrupt routine
static void* my_thread_2(void* angl1); //background thread

//variable declerations
static rc_mpu_data_t mpu_data; //mpu data
double angl1=0; //intantaneous gyro angle
double angl0=0; //initial gyro angle
double outhigh0=0; //filtered accelerometer data 1 step back
double outlow0=0; //filtered gyro data 1 step back
double pi=3.1415926; //pi
double step=0.005; //inner loop step size
double wc=4; //complimentary filter cutoff fz (rad/sec)
double compfilter=0; //complementary filter robot spatial angle
double acceldata=0; //accelerometer data
double thetak=0; //instantaneous robot angle with relation to vertical
double thetak1=0; //step back error angle
double thetak2=0; //2x step back error angle
double uk=0; //intantaneous motor duty cycle
double uk1=0; //step back duty cycle
double uk2=0; //2x step back duty cycle
double refangle=0; //robot reference angle
double errortheta=0.0; //robot error angle
double k1=.3; //inner loop gain
int phi1=0; //wheel 1 encoder value (int)
int phi2=0; //wheel 2 encoder value (int)
double phi2d=0; //wheel 1 encoder (double)
double phi1d=0; //wheel 2 encoder (double)
double rad= 2*3.1415926; //radians in a revolution
double avephi=0; //average encoder value
double phiref=0; //reference (initial) wheel position
double phierr=0; //wheel position error
double ref2=0; //2x step back robot reference angle
double ref1=0; //1x step back robot reference angle
double phib2=0; //2x step back wheel position error
double phib1=0; //1x step back wheel position error

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
                double max= 2148/rad; //encoder value for 1 revolution
                phi1=rc_encoder_read(2); //read encoder value for each wheel
                phi2=rc_encoder_read(3);
                phi1d=phi1; //change to double precision
                phi2d=phi2;
                phi1d=phi1d/max; //change encoder value to radians
                phi2d=-phi2d/max;
                avephi=(phi1d+phi2d)/2; //take average encoder value
                avephi=avephi-thetak; //subtracts off robot angle
                phierr=phiref-avephi; //wheel angle error

                //outer loop controller--updates robot reference angle for use in inner loop
                refangle=1.257775576649003*ref1-0.373344081217810*ref2+0.005057780550182*phierr+0.000246771998252*phib1-0.004811008551930*phib2;
                ref2=ref1; //update steps
                ref1=refangle;
                phib2=phib1;
                phib1=phierr;
                printf("phierr: %6.2f |refangle: %6.2f |errortheta: %6.2f |uk: %6.2f \n ", phierr*RAD_TO_DEG, refangle, errortheta*RAD_TO_DEG, uk);
                rc_usleep(50000);
        }
        return NULL;
}


void imu_interupt_function(void){
        //find spacial position of robot from accelerometer+gyro data
        acceldata=atan2(mpu_data.accel[1],mpu_data.accel[2]); //accelerometer data

        //gyro data
        double omega=mpu_data.gyro[0]*DEG_TO_RAD; //gyro data
        angl1=angl0+step*omega; //find angle using euler's integration method

        //filtering--high pass gyro data, low pass accelerometer data
        double ahigh= 1/(2*pi*step*wc+1); //high pass
        double alow= (2*pi*step*wc)*ahigh; //low pass
        double outlow1=alow*acceldata+(1-alow)*outlow0; //filter accel data
        outlow0=outlow1;
        double outhigh1=ahigh*outhigh0+ahigh*(angl1-angl0); //filter gyro data
        outhigh0=outhigh1;
        compfilter=outhigh1+outlow1; //add filtered angles together
        angl0=angl1;

        //find error angle, duty cycle
        thetak=compfilter-pi/2; //angle with relation to vertical
        errortheta=-refangle-thetak-0.3457; //robot error angle
        if (((errortheta-thetak1)>0.1)||((thetak1-errortheta)>0.1)){
                uk=0;//to prevent wheel motion during discontinuous angle reads
                }
        else {
                //inner loop controller
                uk=k1*(-8.682690225552795*errortheta+15.326915352200393*thetak1-6.759610173791212*thetak2)+1.538460787577220*uk1-0.538460787577220*uk2;
                }
        if (uk>1){
                uk=1; //max duty cycle
        }
        if (uk<-1){
                uk=-1;
        }
        if (errortheta*RAD_TO_DEG>70){
                uk=0; //stop wheels if robot knocked over
        }
        if (errortheta*RAD_TO_DEG<-70){
                uk=0;
        }

        //update steps
        thetak2=thetak1;
        thetak1=errortheta;
        uk2=uk1;
        uk1=uk;

        //apply duty cycle
        rc_motor_set(3,uk);
        rc_motor_set(2,-uk);
        return;
}

void on_pause_release()
{
        //Make the Pause button toggle between paused and running states.
        if(rc_get_state()==RUNNING)     rc_set_state(PAUSED);
        else if(rc_get_state()==PAUSED) rc_set_state(RUNNING);
        return;
}


/**
* If the user holds the pause button for 0.5 seconds, set state to EXITING which
* triggers the rest of the program to exit cleanly.
**/
void on_pause_press()
{
        int i;
        const int samples = 100; // check for release 100 times in this period
        const int us_wait = 500000; // 0.5 seconds
        // now keep checking to see if the button is still held down
        for(i=0;i<samples;i++){
                rc_usleep(us_wait/samples);
                if(rc_button_get_state(RC_BTN_PIN_PAUSE)==RC_BTN_STATE_RELEASED) return;
        }
        printf("long press detected, shutting down\n");
        rc_set_state(EXITING);
        return;
}

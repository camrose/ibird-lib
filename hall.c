// Modifications and additions to this file made by Andrew Pullin are copyright, 2013
// Copyrights are acknowledged for portions of this code extant before modifications by Andrew Pullin 
// Any application of BSD or other license to copyright content without the authors express approval
// is invalid and void.

// July 14, 2011
// authors: rfearing, apullin
// code for position feedback using hall sensors on left and right motor

#include "p33Fxxxx.h"
#include "led.h"
#include "pid.h"
#include "hall.h"
#include "gyro.h"
#include "motor_ctrl.h"
#include "timer.h"
#include "adc_pid.h"
#include "pwm.h"
#include "hall.h"
#include "p33Fxxxx.h"
#include "incap.h" // input capture
#include <stdlib.h> // for malloc

//Private Functions
static void SetupTimer1(void);
static void SetupTimer2(void);
static void SetupInputCapture(void);
static void hallUpdateBEMF(void);
static void hallUpdatePID(pidPos *pid);
int medianFilter3(int*);

//Function to be installed into T1, and setup function
//static void hallServiceRoutine(void);

///////////////////////////////////
/////// Local variables ///////////
//////////////////////////////////
int t2_ticks;
// unsigned long tic, toc;
long old_wing_time, wing_time, wing_delta; // time of last event
long motor_count[1]; // 0 = left 1 = right counts on sensor

//MoveQueue hallMoveq;
//moveCmdT hallCurrentMove, hallIdleMove, hallManualMove;

int hallbemf[NUM_HALL_PIDS]; //used to store the true, unfiltered speed
int hallbemfLast[NUM_HALL_PIDS]; // Last post-median-filter value
int hallbemfHist[NUM_HALL_PIDS][3]; //This is ONLY for applying the median filter to

//This is an array to map legCtrl controller to PWM output channels
int hallOutputChannels[NUM_HALL_PIDS];

// PID control structure
pidPos hallPIDObjs;
//pidObj hallPIDObjs[NUM_HALL_PIDS];

// structure for reference velocity for leg
hallVelLUT hallPIDVel;

// may be glitch in longer missions at rollover
unsigned long lastMoveTime;
int seqIndex;

static void hallGetSetpoint();
static void hallSetControl();

//////////// 1khz timer //////////
//extern volatile unsigned long t1_ticks;
volatile unsigned long hall_t1_ticks;

///////////////////////////////////
/////// Private Functions /////////
///////////////////////////////////

//Hall effect sensor has ~ 3 kHz rate max, so choose clock high enough
// choose clock period = 6.4 us, divide FCY by 256

// highest interrupt priority. runs at 1 kHZ
static void SetupTimer1(void)
{
    unsigned int T1CON1value, T1PERvalue;
    T1CON1value = T1_ON & T1_SOURCE_INT & T1_PS_1_1 & T1_GATE_OFF &
            T1_SYNC_EXT_OFF & T1_IDLE_CON;

    T1PERvalue = 0x9C40; //clock period = 0.001s = (T1PERvalue/FCY) (1KHz)
    //T1PERvalue = 0x9C40/2;
    //getT1_ticks() = 0;
    OpenTimer1(T1CON1value, T1PERvalue);
    ConfigIntTimer1(T1_INT_PRIOR_6 & T1_INT_ON);
    //int retval;
    //retval = sysServiceConfigT1(T1CON1value, T1PERvalue, T1_INT_PRIOR_6 & T1_INT_ON);
    //TODO: Put a soft trap here, conditional on retval
}
 
//Timer 2 counts
// Set up just to provide a tick counter at 400 Hz
static void SetupTimer2(void) {
    unsigned int T2CON1value, T2PERvalue;
    t2_ticks = 0;
    T2CON1value = T2_ON & T2_SOURCE_INT & T2_PS_1_256 & T2_GATE_OFF;
    T2PERvalue = 0xffff; // max period
    OpenTimer2(T2CON1value, T2PERvalue);
    ConfigIntTimer2(T2_INT_PRIOR_5 & T2_INT_ON);
    //int retval;
    //retval = sysServiceConfigT2(T2CON1value, T2PERvalue, T2_INT_PRIOR_5 & T2_INT_ON);
    //TODO: Put a soft trap here, conditional on retval
}


static void SetupInputCapture() {
    // RB4 and RB5 will be used for inputs
    _TRISB4 = 1; // set for input
    //_TRISB5 = 1; // set for input

    // left leg
    motor_count[0] = 0;
    old_wing_time = 0;
    ConfigIntCapture7(IC_INT_ON & IC_INT_PRIOR_2);
    EnableIntIC7;
    /* In Edge Detection Mode (ICM = 001), the interrupt is generated on every capture
    event and the Input Capture Interrupt (ICI<1:0>) bits are ignored. */
    OpenCapture7(IC_IDLE_STOP & IC_TIMER2_SRC &
            IC_INT_1CAPTURE & IC_EVERY_EDGE);
}

//unsigned long toc;


/////////////////////////
///////   ISR's  /////////
/////////////////////////

// Input Capture 7: wings

//handler for wings
void __attribute__((__interrupt__, no_auto_psv)) _IC7Interrupt(void) {
    // Insert ISR code here
    motor_count[0]++; // increment count for right side - neglect overflow/wrap around

    wing_time = (long) IC7BUF + ((long) (t2_ticks) << 16);
    //left_time = (long) IC7BUF + (getT2_ticks() << 16);
    wing_delta = wing_time - old_wing_time;
    old_wing_time = wing_time;

    LED_RED = ~LED_RED;

    IFS1bits.IC7IF = 0; // Clear CN interrupt
}

/// Replaced by sys_service module
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void) {

    t2_ticks++; // updates about every 400 ms

    //Clear Timer2 interrupt flag
    _T2IF = 0;
}




//////////////////////////////////
/////// Public Functions /////////
//////////////////////////////////


//Main hall effect sensor setup, called from main()
void hallSetup() {
    //Init of PID controller objects
    int i;
    int temp_in = 0;
    for (i = 0; i < NUM_HALL_PIDS; i++) {
        hallInitPIDObjPos(&(hallPIDObjs), DEFAULT_HALL_KP, DEFAULT_HALL_KI,
                DEFAULT_HALL_KD, DEFAULT_HALL_KAW, DEFAULT_HALL_FF);
        hallPIDObjs.minVal = 0;
        hallPIDObjs.satValNeg = 0;
        hallPIDObjs.maxVal = FULLTHROT;
        hallPIDObjs.satValPos = SATTHROT;
    }

    // Controller to PWM channel correspondance
    hallOutputChannels[0] = MC_CHANNEL_PWM1;

    //Init for velocity profile objects
    hallInitPIDVelProfile();

    lastMoveTime = 0;
    //  initialize PID structures before starting Timer1
    hallPIDSetInput(temp_in, lastMoveTime);

    for (i = 0; i < NUM_HALL_PIDS; i++) {
        hallbemfLast[i] = 0;
        hallbemfHist[i][0] = 0;
        hallbemfHist[i][1] = 0;
        hallbemfHist[i][2] = 0;
    }
    
    //System setup
    SetupTimer1();
    SetupTimer2(); // used for leg hall effect sensors
    SetupInputCapture(); // setup input capture for hall effect sensors


    //int retval;
    //retval = sysServiceInstallT1(hallServiceRoutine);

    // returns pointer to queue with 8 move entries
//    hallMoveq = mqInit(8);
//    hallIdleMove = malloc(sizeof (moveCmdStruct));
//    hallIdleMove->inputL = 0;
//    hallIdleMove->inputR = 0;
//    hallIdleMove->duration = 0;
//    hallCurrentMove = hallIdleMove;
//
//    hallManualMove = malloc(sizeof (moveCmdStruct));
//    hallManualMove->inputL = 0;
//    hallManualMove->inputR = 0;
//    hallManualMove->duration = 0;

}

// ----------   all the initializations  -------------------------
// set expire time for first segment in pidSetInput - use start time from MoveClosedLoop
// set points and velocities for one revolution of leg
// called from pidSetup()

void hallInitPIDVelProfile() {
    int i, j;
    for (j = 0; j < NUM_PIDS; j++) {
        hallPIDVel.index = 0; // point to first velocity
        hallPIDVel.interpolate = 0;
        hallPIDVel.wing_strokes = 0; // set initial leg count
        // set control intervals during stride - try to get close to 21.3 ratio (use 42 counts)
        hallPIDVel.interval[0] = (3 * STRIDE_TICKS / NUM_VELS / 3);
        hallPIDVel.delta[0] = 11;
        hallPIDVel.interval[1] = (3 * STRIDE_TICKS / NUM_VELS / 3);
        hallPIDVel.delta[1] = 11;
        hallPIDVel.interval[2] = (3 * STRIDE_TICKS / NUM_VELS / 3);
        hallPIDVel.delta[2] = 11;
        hallPIDVel.interval[3] = (3 * STRIDE_TICKS / NUM_VELS / 3);
        hallPIDVel.delta[3] = 12;
        for (i = 0; i < NUM_VELS; i++) { // interpolate values between setpoints, <<4 for resolution
            hallPIDVel.vel[i] = (hallPIDVel.delta[i] << 8) / hallPIDVel.interval[i];
        }
        hallPIDObjs.p_input = 0; // initialize first set point
    }
}

// set values from packet - leave previous motor_count, p_input, etc.
// called from cmd.c

void hallSetVelProfile(hallVelCmdParams params) {
    int i;
    for (i = 0; i < NUM_VELS; i++) {
        hallPIDVel.interval[i] = params->interval[i];
        hallPIDVel.delta[i] = params->delta[i];
        hallPIDVel.vel[i] = params->vel[i];
    }
}


// called from set thrust closed loop, etc. Thrust

void hallPIDSetInput(unsigned int input, unsigned int run_time) {
    unsigned long temp;
    hallPIDObjs.v_input = input;
    hallPIDObjs.run_time = run_time;
    hallPIDObjs.start_time = hall_t1_ticks;
    //zero out running PID values
    hallPIDObjs.i_error = 0;
    hallPIDObjs.p = 0;
    hallPIDObjs.i = 0;
    hallPIDObjs.d = 0;
    //Seed the median filter

    temp = hall_t1_ticks; // need atomic read due to interrupt
    lastMoveTime = temp + (unsigned long) run_time; // only one run time for both sides
    // set initial time for next move set point

    /*   need to set index =0 initial values */
    /* position setpoints start at 0 (index=0), then interpolate until setpoint 1 (index =1), etc */
    hallPIDVel.expire = temp + (long) hallPIDVel.interval[0]; // end of first interval
    hallPIDVel.interpolate = 0;
    /*        pidObjs[pid_num].p_input += pidVel[pid_num].delta[0];        //update to first set point
     ***  this should be set only after first .expire time to avoid initial transients */
    hallPIDVel.index = 0; // reset setpoint index
    // set first move at t = 0
    //        pidVel[0].expire = temp;   // right side
    //        pidVel[1].expire = temp;   // left side

}

void hallSetGains(hallGainParams params) {
    hallPIDObjs.Kp = params->Kp;
    hallPIDObjs.Ki = params->Ki;
    hallPIDObjs.Kd = params->Kd;
    hallPIDObjs.Kaw = params->Kaw;
    hallPIDObjs.Kff = params->Kff;
}

void hallPIDOn() {
    hallZeroPos();
    hallPIDObjs.onoff = 1;
}

void hallPIDOff() {
    hallPIDObjs.onoff = 0;
}

// zero position setpoint for both motors (avoids big offset errors)

void hallZeroPos() {
    // disable interrupts to reset both motor_counts
    DisableIntIC7;
    motor_count[0] = 0;
    EnableIntIC7;
    // reset position setpoint as well
    hallPIDObjs.p_input = 0;
    hallPIDVel.wing_strokes = 0; // strides also reset
}


/*********************** Motor Control Interrupt *********************************************/
/*****************************************************************************************/

/*****************************************************************************************/


/////////        Hall Control ISR       ////////
/////////  Installed to Timer1 @ 1Khz  ////////
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
//static void hallServiceRoutine(void)
{
    // debug Hall effect sensors //
    LED_RED = _RB4;
    //LED_RED = _RB5;

    hall_t1_ticks++;

    if (hall_t1_ticks > lastMoveTime) // turn off if done running
    { //        hallPIDSetInput(0, 0, 0);    don't reset state when done run, keep for recording telemetry
        hallPIDObjs.onoff = 0;
        if (hall_t1_ticks > 1000) {
            Nop();
        }
        //hallGetSetpoint();
        //        hallPIDSetInput(1, 0, 0);
    } else // update velocity setpoints if needed - only when running
    {
        hallGetSetpoint();
    }

    hallUpdateBEMF();
    hallSetControl();
    _T1IF = 0;
}

static void hallGetSetpoint() {
    int j, index;
    index = hallPIDVel.index;
    // update desired position between setpoints, scaled by 256
    hallPIDVel.interpolate += hallPIDVel.vel[index];

    if (hall_t1_ticks >= hallPIDVel.expire) // time to reach previous setpoint has passed
    {
        hallPIDVel.interpolate = 0;
        hallPIDObjs.p_input += hallPIDVel.delta[index]; //update to next set point
        hallPIDVel.expire += hallPIDVel.interval[(index + 1) % NUM_VELS]; // expire time for next interval
        // got to next index point
        hallPIDVel.index++;

        if (hallPIDVel.index >= NUM_VELS) {
            hallPIDVel.index = 0;
            hallPIDVel.wing_strokes++; // one full leg revolution
            // need to correct for 426 counts per leg stride
            // 5 rev @ 42 counts/rev = 210, actual set point 5 rev @ 42.6 counts, so add 3 to p_input
                if ((hallPIDVel.wing_strokes % 5) == 0) {
                    hallPIDObjs.p_input += 3;
                }
        } // loop on index
    }
}

static void hallSetControl() {
    long j;
    long k;
    // 0 = right side
   
    hallPIDObjs.p_error = hallPIDObjs.p_input + (hallPIDVel.interpolate >> 8) - motor_count[0];
    //hallPIDObjs.p_error = hallPIDObjs.p_input - motor_count[0];
    //hallPIDObjs[j].v_error = hallPIDObjs[j].v_input - measurements[j];
    hallPIDObjs.v_error = hallPIDObjs.v_input - hallbemf[0];
    //Update values

//    if (hallPIDObjs.p_error < -100) {
//        k = hallPIDObjs.p_input;
//        j = motor_count[0];
//        Nop();
//    }

    hallUpdatePID(&(hallPIDObjs));
    if (hallPIDObjs.onoff) {
        //Might want to change this in the future, if we want to track error
        //even when the motor is off.
        //Set PWM duty cycle
        SetDCMCPWM(MC_CHANNEL_PWM1, hallPIDObjs.output, 0); //PWM1.L
    }//end of if (on / off)
    else { //if PID loop is off
        SetDCMCPWM(MC_CHANNEL_PWM1, 0, 0);
    }
}

static void hallUpdatePID(pidPos *pid) {
    pid->p = (long) pid->Kp * pid->p_error;
    pid->i = (long) pid->Ki * pid->i_error;
    pid->d = (long) pid->Kd * (long) pid->v_error;
    // better check scale factors
    /* just use simpled PID, offset is already subtracted in PID GetState */
    // scale so doesn't over flow
    pid->preSat = pid->Kff + pid->p +
            ((pid->i + pid->d) >> 4); // divide by 16
    pid->output = pid->preSat;
    //Clamp output above 0 since don't have H bridge
    if (pid->preSat < pid->minVal) {
        pid->output = pid->minVal;
    }

    pid-> i_error = (long) pid-> i_error + (long) pid->p_error; // integrate error
    // saturate output - assume only worry about >0 for now
    // apply anti-windup to integrator
    if (pid->preSat > pid->satValPos) {
        pid->output = pid->satValPos;
        pid->i_error = (long) pid->i_error + (long) (pid->Kaw) * ((long) (pid->satValPos) - (long) (pid->preSat)) / ((long) GAIN_SCALER);
    }

}

//This duplicates functionalist in leg_ctrl
static void hallUpdateBEMF() {
    //Back EMF measurements are made automatically by coordination of the ADC, PWM, and DMA.
    //Copy to local variables. Not strictly neccesary, just for clarity.
    //This **REQUIRES** that the divider on the battery & BEMF circuits have the same ratio.
    hallbemf[0] = adcGetVBatt() - adcGetBEMFL();
    //NOTE: at this point, we should have a proper correspondance between
    //   the order of all the structured variable; bemf[i] associated with
    //   pidObjs[i], bemfLast[i], etc.
    //   Any "jumbling" of the inputs can be done in the above assignments.

    //Negative ADC measures mean nothing and should never happen anyway
    if (hallbemf[0] < 0) {
        hallbemf[0] = 0;
    }

    //Apply median filter
    int i;
    for (i = 0; i < NUM_HALL_PIDS; i++) {
        hallbemfHist[i][2] = hallbemfHist[i][1]; //rotate first
        hallbemfHist[i][1] = hallbemfHist[i][0];
        hallbemfHist[i][0] = hallbemf[i]; //include newest value
        hallbemf[i] = medianFilter3(hallbemfHist[i]); //Apply median filter
    }

    // IIR filter on BEMF: y[n] = 0.2 * y[n-1] + 0.8 * x[n]
    hallbemf[0] = (2 * (long) hallbemfLast[0] / 10) + 8 * (long) hallbemf[0] / 10;
    hallbemfLast[0] = hallbemf[0]; //bemfLast will not be used after here, OK to set
}


////   Public functions
////////////////////////

int hallGetOutput() {
    return hallPIDObjs.output;
}

long hallGetError() {
    int r = (int) hallPIDObjs.p_input + (hallPIDVel.interpolate >> 8);
    return r;
}

void hallInitPIDObjPos(pidPos *pid, int Kp, int Ki, int Kd, int Kaw, int Kff) {
    pid->p_input = 0;
    pid->v_input = 0;
    pid->p = 0;
    pid->i = 0;
    pid->d = 0;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Kaw = Kaw;
    pid->Kff = Kff;
    pid->output = 0;
    pid->onoff = 0;
    pid->p_error = 0;
    pid->v_error = 0;
    pid->i_error = 0;
}

// called from steeringSetup()

void hallInitPIDObj(pidObj *pid, int Kp, int Ki, int Kd, int Kaw, int Kff) {
    pid->input = 0;
    pid->dState = 0;
    pid->iState = 0;
    pid->output = 0;
    pid->y_old = 0;
    pid->p = 0;
    pid->i = 0;
    pid->d = 0;
    //This is just a guess for the derivative filter time constant
    pid->N = 5;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Kaw = Kaw;
    pid->Kff = 0;
    pid->onoff = 0;
    pid->error = 0;
}

//todo: a getter should not return a pointer; this is unsafe behavior.
long* hallGetMotorCounts() {
    return motor_count;
}
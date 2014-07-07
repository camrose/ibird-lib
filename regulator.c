/*
 * Copyright (c) 2009 - 2012, Regents of the University of California
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of the University of California, Berkeley nor the names
 *   of its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * I-Bird Attitude Regulation Module
 *
 * by Stanley S. Baek
 *
 * v.0.4
 *
 * Revisions:
 *  Stanley S. Baek     2009-10-30      Initial release
 *  Humphrey Hu		    2011-07-20      Changed to fixed point
 *  Humphrey Hu         2012-02-20      Returned to floating point, restructured
 *  Humphrey Hu         2012-06-30      Switched to using quaternion representation
 *
 * Notes:
 *  I-Bird body axes are:
 *      x - Forward along anteroposterior axis
 *      y - Left along left-right axis
 *      z - Up along dorsoventral axis
 *  Rotations in body axes are:
 *      yaw - Positive z direction
 *      pitch - Positive y direction
 *      roll - Positive x direction
 *  PID loops should have references set to 0, since they take the externally
 *      calculated error as an input.
 */

// Software modules
#include "regulator.h"
#include "controller.h"
#include "dfilter.h"
#include "attitude.h"
#include "cv.h"
#include "xl.h"
#include "rate.h"
#include "slew.h"
#include "adc_pid.h"
#include "led.h"
#include "hall.h"
#include "line_sensor.h"

// Hardware/actuator interface
#include "motor_ctrl.h"
#include "sync_servo.h"

// Other
#include "quat.h"
#include "sys_clock.h"
#include "bams.h"
#include "utils.h"
#include "ppbuff.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

typedef struct {
    float thrust;
    float steer;
    float elevator;
} RegulatorOutput;

typedef struct {
    float yaw_err;
    float pitch_err;
    float roll_err;
} RegulatorError;

#define YAW_SAT_MAX         (1.0)
#define YAW_SAT_MIN         (-1.0)
#define PITCH_SAT_MAX       (1.0)
#define PITCH_SAT_MIN       (-1.0)
#define ROLL_SAT_MAX        (1.0)
#define ROLL_SAT_MIN        (0.0) // Doubling as thrust right now

#define DEFAULT_SLEW_LIMIT  (1.0)

// =========== Static Variables ================================================
// Control loop objects
CtrlPidParamStruct yawPid, pitchPid, thrustPid;
DigitalFilterStruct yawRateFilter, pitchRateFilter, rollRateFilter;

// State info
static unsigned char is_ready = 0, is_logging = 0, temp_rot_active = 0, fig_eight = 0, line_track = 0;
static unsigned char yaw_filter_ready = 0, pitch_filter_ready = 0, roll_filter_ready = 0;
static RegulatorMode reg_mode;
static RegulatorOutput rc_outputs;
static Quaternion reference, limited_reference, pose, temp_rot;
static int eight_stage = 0;

// Telemetry buffering
static RegulatorStateStruct reg_states[2];
static PingPongBuffer reg_state_buff;

// =========== Function Stubs =================================================
static float runYawControl(float yaw);
static float runPitchControl(float pitch);
static float runRollControl(float roll);
static int updateBEMF();

static void calculateError(RegulatorError *error);
static void calculateError2(Quaternion *ref_temp, RegulatorError *error);
static void filterError(RegulatorError *error);
static void calculateOutputs(RegulatorError *error, RegulatorOutput *output);
static void applyOutputs(RegulatorOutput *output);
static void logTrace(RegulatorError *error, RegulatorOutput *output);

// =========== Local Variables ===============================================

int bemf[NUM_MOTOR_PIDS]; //used to store the true, unfiltered speed
int bemfLast[NUM_MOTOR_PIDS]; // Last post-median-filter value
int bemfHist[NUM_MOTOR_PIDS][3]; //This is ONLY for applying the median filter to
int medianFilter3(int*);
float crankAngle;
int zone;
int bemfVals[4];
int crankCalibrated;
unsigned long curr_time;
unsigned long prev_time;

// =========== Public Functions ===============================================

void rgltrSetup(float ts) {
    int i;
    
    // Set up drivers
    servoSetup();
    mcSetup();
    
    // Set up dependent modules
    attSetup(ts);
    
    
    rateSetup(ts);
    
    
    slewSetup(ts);    
    slewSetLimit(DEFAULT_SLEW_LIMIT);
    slewEnable();
    
    reg_mode = REG_OFF;  

    ctrlInitPidParams(&yawPid, ts);
    ctrlInitPidParams(&pitchPid, ts);
    ctrlInitPidParams(&thrustPid, ts);    

    ppbuffInit(&reg_state_buff);
    ppbuffWriteActive(&reg_state_buff, &reg_states[0]);
    ppbuffWriteInactive(&reg_state_buff, &reg_states[1]);
        
    reference.w = 1.0;
    reference.x = 0.0;
    reference.y = 0.0;
    reference.z = 0.0;   
    
    limited_reference.w = 1.0;
    limited_reference.x = 0.0;
    limited_reference.y = 0.0;
    limited_reference.z = 0.0;   
    
    is_logging = 0;
    is_ready = 1;

    for (i = 0; i < NUM_MOTOR_PIDS; i++) {
        bemfLast[i] = 0;
        bemfHist[i][0] = 0;
        bemfHist[i][1] = 0;
        bemfHist[i][2] = 0;
    }
    crankAngle = 0;
    zone = 0;
    bemfVals[0] = 0;
    bemfVals[1] = 0;
    bemfVals[2] = 0;
    bemfVals[3] = 0;
    crankCalibrated = 0;
    curr_time = 0;
    prev_time = 0;
}

void rgltrSetMode(unsigned char flag) {

    if(flag == REG_OFF) {
        LED_RED = 1;
        rgltrSetOff();
        //hallPIDOff();
    } else if(flag == REG_TRACK) {
        LED_RED = 0;
        rgltrSetTrack();
    } else if(flag == REG_REMOTE_CONTROL) {
        rgltrSetRemote();
    } else if(flag == REG_TRACK_HALL) {
        rgltrSetHall();
    }
        
}

void rgltrSetOff(void) {
    reg_mode = REG_OFF;
    ctrlStop(&yawPid);
    ctrlStop(&pitchPid);
    ctrlStop(&thrustPid);
    hallPIDOff();
    servoStop();
}

void rgltrSetTrack(void) {
    reg_mode = REG_TRACK;
    ctrlStart(&yawPid);
    ctrlStart(&pitchPid);
    ctrlStart(&thrustPid);
    servoStart();
}

void rgltrSetRemote(void) {
    reg_mode = REG_REMOTE_CONTROL;
    ctrlStop(&yawPid);
    ctrlStop(&pitchPid);
    ctrlStop(&thrustPid);
    hallPIDOff();
    servoStart();
}

void rgltrSetHall(void) {
    reg_mode = REG_TRACK_HALL;
    ctrlStart(&yawPid);
    ctrlStart(&pitchPid);
    hallPIDOn();
    servoStart();
}

void rgltrSetYawRateFilter(RateFilterParams params) {

    dfilterInit(&yawRateFilter, params->order, params->type, 
                params->xcoeffs, params->ycoeffs);
    yaw_filter_ready = 1;
    
} 


void rgltrSetPitchRateFilter(RateFilterParams params) {

    dfilterInit(&pitchRateFilter, params->order, params->type,
                params->xcoeffs, params->ycoeffs);
    pitch_filter_ready = 1;
                
} 

void rgltrSetRollRateFilter(RateFilterParams params) {

    dfilterInit(&rollRateFilter, params->order, params->type,
                params->xcoeffs, params->ycoeffs);
    roll_filter_ready = 1;
    
}

void rgltrSetOffsets(float *offsets) {

    ctrlSetPidOffset(&yawPid, offsets[0]);
    ctrlSetPidOffset(&pitchPid, offsets[1]);
    ctrlSetPidOffset(&thrustPid, offsets[2]);

}

void rgltrSetYawPid(PidParams params) {
    
    ctrlSetPidParams(&yawPid, params->ref, params->kp, params->ki, params->kd);
    ctrlSetPidOffset(&yawPid, params->offset);
    ctrlSetRefWeigts(&yawPid, params->beta, params->gamma);
    ctrlSetSaturation(&yawPid, YAW_SAT_MAX, YAW_SAT_MIN);

}

void rgltrSetPitchPid(PidParams params) {
    
    ctrlSetPidParams(&pitchPid, params->ref, params->kp, params->ki, params->kd);
    ctrlSetPidOffset(&pitchPid, params->offset);
    ctrlSetRefWeigts(&pitchPid, params->beta, params->gamma);
    ctrlSetSaturation(&pitchPid, PITCH_SAT_MAX, PITCH_SAT_MIN);

}

void rgltrSetRollPid(PidParams params) {

    ctrlSetPidParams(&thrustPid, params->ref, params->kp, params->ki, params->kd);
    ctrlSetPidOffset(&thrustPid, params->offset);
    ctrlSetRefWeigts(&thrustPid, params->beta, params->gamma);
    ctrlSetSaturation(&thrustPid, ROLL_SAT_MAX, ROLL_SAT_MIN);

}

void rgltrSetYawRef(float ref) {
    ctrlSetRef(&yawPid, ref);
}

void rgltrSetPitchRef(float ref) {
    ctrlSetRef(&pitchPid, ref);
}

void rgltrSetRollRef(float ref) {
    ctrlSetRef(&thrustPid, ref);
}

void rgltrGetQuatRef(Quaternion *ref) {
    if(ref == NULL) { return; }
    quatCopy(ref, &reference);
}

void rgltrSetQuatRef(Quaternion *ref) {
    if(ref == NULL) { return; }
    quatCopy(&reference, ref);
}

void rgltrSetTempRot(Quaternion *rot) {
    if(rot == NULL) { return; }
    quatCopy(&temp_rot, rot);
    temp_rot_active = 1;
}

void rgltrSetRemoteControlValues(float thrust, float steer, float elevator) {
    rc_outputs.thrust = thrust;
    rc_outputs.steer = steer;
    rc_outputs.elevator = elevator;
}

void rgltrStartLogging(void) {
    is_logging = 1;
}

void rgltrStopLogging(void) {
    is_logging = 0;
}

void rgltrGetState(RegulatorState dst) {

    RegulatorState src;

    src = ppbuffReadActive(&reg_state_buff);
    if(src == NULL) { // Return 0's if no unread data
        memset(dst, 0, sizeof(RegulatorStateStruct));
        return; 
    }
    
    memcpy(dst, src, sizeof(RegulatorStateStruct));    
    
}

void rgltrStartEight(void) {
    fig_eight = 1;
    eight_stage = 0;
}

void rgltrStopEight(void) {
    fig_eight = 0;
    eight_stage = 0;
}

void rgltrRunController(void) {
    
    RegulatorError error;        
    RegulatorOutput output;
    int updated_bemf;
    Quaternion ref_temp;
    RateStruct rate_set;
    LineCam line;

    if(!is_ready) { return; }    

    if(fig_eight) {
        if (eight_stage == 0) {
            ref_temp.w = 0.8214;
            ref_temp.x = 0.1786;
            ref_temp.y = 0.3830;
            ref_temp.z = -0.3830;
            rgltrSetQuatRef(&ref_temp);
            eight_stage = 1;
        } else if (eight_stage == 1) {
            ref_temp.w = 0.8214;
            ref_temp.x = 0.1786;
            ref_temp.y = 0.3830;
            ref_temp.z = -0.3830;
            calculateError2(&ref_temp,&error);
            if (error.yaw_err < 0.04 && error.yaw_err > -0.04) {
                rate_set.pitch_rate = 0;
                rate_set.yaw_rate = 1.5;
                rate_set.roll_rate = 0;
                rateSetGlobalSlew(&rate_set);
                rateEnable();
                eight_stage = 2;
            }
        } else if (eight_stage == 2) {
            ref_temp.w = 0.6409;
            ref_temp.x = -0.2988;
            ref_temp.y = 0.2988;
            ref_temp.z = 0.6409;
            calculateError2(&ref_temp,&error);
            if (error.yaw_err < 0.04 && error.yaw_err > -0.04) {
                rate_set.pitch_rate = 0;
                rate_set.yaw_rate = -1.5;
                rate_set.roll_rate = 0;
                rateSetGlobalSlew(&rate_set);
                rateEnable();
                eight_stage = 3;
            }
        } else if (eight_stage == 3) {
            ref_temp.w = 0.6409;
            ref_temp.x = 0.2988;
            ref_temp.y = 0.2988;
            ref_temp.z = -0.6409;
            calculateError2(&ref_temp,&error);
            if (error.yaw_err < 0.04 && error.yaw_err > -0.04) {
                eight_stage = 4;
            }
        } else if (eight_stage == 4) {
            ref_temp.w = 0.6409;
            ref_temp.x = -0.2988;
            ref_temp.y = 0.2988;
            ref_temp.z = 0.6409;
            calculateError2(&ref_temp,&error);
            if (error.yaw_err < 0.04 && error.yaw_err > -0.04) {
                rateDisable();
                ref_temp.w = 0.0;
                ref_temp.x = -0.4226;
                ref_temp.y = 0.0;
                ref_temp.z = 0.9063;
                rgltrSetQuatRef(&ref_temp);
                eight_stage = 5;
            }
        } else if (eight_stage == 5) {
            
        }
    } else if (line_track) {
        if (lsHasNewFrame()) {
            line = lsGetFrame();
            
        }
    }

    attEstimatePose();  // Update attitude estimate
    //updated_bemf = updateBEMF();

    rateProcess();      // Update limited_reference
    slewProcess(&reference, &limited_reference); // Apply slew rate limiting

    attGetQuat(&pose);
    calculateError(&error);
    calculateOutputs(&error, &output);

    applyOutputs(&output);

    if(is_logging) {
        logTrace(&error, &output);
    }
}


// =========== Private Functions ===============================================

static float runYawControl(float yaw) {

    if(yaw_filter_ready) {
       return ctrlRunPid(&yawPid, yaw, &yawRateFilter);
    } else {
       return ctrlRunPid(&yawPid, yaw, NULL);
    }
}


static float runPitchControl(float pitch) {   

    if(pitch_filter_ready) {
        return ctrlRunPid(&pitchPid, pitch, &pitchRateFilter);
    } else {
        return ctrlRunPid(&pitchPid, pitch, NULL);
    }        

}

static float runRollControl(float roll) {

    if(roll_filter_ready) {
        return ctrlRunPid(&thrustPid, roll, &rollRateFilter);
    } else {
        return ctrlRunPid(&thrustPid, roll, NULL);
    }

}

static void applyTempRot(Quaternion *input, Quaternion *output) {
    if (temp_rot_active == 1) {
        quatMult(&temp_rot, input, output);
        quatNormalize(output);
    } else {
        quatCopy(output, input);
    }
}

static void calculateError2(Quaternion *ref_temp, RegulatorError *error) {

    Quaternion conj_quat, err_quat;
    bams16_t a_2;
    float scale;

    // qref = qpose*qerr
    // qpose'*qref = qerr
    quatConj(&pose, &conj_quat);
    //quatMult(&limited_reference, &conj_quat, &err_quat);
    quatMult(&conj_quat, ref_temp, &err_quat);

    // q = [cos(a/2), sin(a/2)*[x, y, z]]
    // d[x, y, z] = [q]*a/sin(a/2)
    if(err_quat.w == 1.0) { // a = 0 case
        error->yaw_err = 0.0;
        error->pitch_err = 0.0;
        error->roll_err = 0.0;
    } else {
        a_2 = bams16Acos(err_quat.w); // w = cos(a/2)
        scale = bams16ToFloatRad(a_2*2)/bams16Sin(a_2); // a/sin(a/2)
        error->yaw_err = err_quat.z*scale;
        error->pitch_err = err_quat.y*scale;
        error->roll_err = err_quat.x*scale;
    }

}

static void calculateError(RegulatorError *error) {

    Quaternion conj_quat, err_quat;
    bams16_t a_2;
    float scale;
    
    // qref = qpose*qerr
    // qpose'*qref = qerr
    quatConj(&pose, &conj_quat);
    //quatMult(&limited_reference, &conj_quat, &err_quat);
    quatMult(&conj_quat, &limited_reference, &err_quat);

    // q = [cos(a/2), sin(a/2)*[x, y, z]]
    // d[x, y, z] = [q]*a/sin(a/2)        
    if(err_quat.w == 1.0) { // a = 0 case
        error->yaw_err = 0.0;
        error->pitch_err = 0.0;
        error->roll_err = 0.0;
    } else {
        a_2 = bams16Acos(err_quat.w); // w = cos(a/2)             
        scale = bams16ToFloatRad(a_2*2)/bams16Sin(a_2); // a/sin(a/2)
        error->yaw_err = err_quat.z*scale;
        error->pitch_err = err_quat.y*scale;
        error->roll_err = err_quat.x*scale;
    }
    
}

static void filterError(RegulatorError *error) {

    if(yaw_filter_ready) {
        error->yaw_err = dfilterApply(&yawRateFilter, error->yaw_err);
    }
    if(pitch_filter_ready) {
        error->pitch_err = dfilterApply(&pitchRateFilter, error->pitch_err);
    }
    if(roll_filter_ready) {
        error->roll_err = dfilterApply(&rollRateFilter, error->roll_err);
    }
    
}

static void calculateOutputs(RegulatorError *error, RegulatorOutput *output) {

    if(reg_mode == REG_REMOTE_CONTROL) {

        output->steer = rc_outputs.steer;
        output->thrust = rc_outputs.thrust;
        output->elevator = rc_outputs.elevator;

    } else if(reg_mode == REG_TRACK){

        output->steer = runYawControl(error->yaw_err);
        output->elevator = runPitchControl(error->pitch_err);        
        output->thrust = runRollControl(error->pitch_err);

    } else if(reg_mode == REG_TRACK_HALL){
        
        output->steer = runYawControl(error->yaw_err);
        output->elevator = runPitchControl(error->pitch_err);
        output->thrust = (float) ((float) hallGetOutput())/(2.0*(float) PTPER);

    } else {

        output->steer = 0.0;
        output->thrust = 0.0;
        output->elevator = 0.0;

    }

}

static void applyOutputs(RegulatorOutput *output) {

    mcSteer(output->steer);
    mcThrust(output->thrust);
    servoSet(output->elevator);

}

//Poor implementation of a median filter for a 3-array of values

int medianFilter3(int* a) {
    int b[3] = {a[0], a[1], a[2]};
    int temp;

    //Implemented through 3 compare-exchange operations, increasing index
    if (b[0] > b[1]) {
        temp = b[1];
        b[1] = b[0];
        b[0] = temp;
    }
    if (a[0] > a[2]) {
        temp = b[2];
        b[2] = b[0];
        b[0] = temp;
    }
    if (a[1] > a[2]) {
        temp = b[2];
        b[2] = b[1];
        b[1] = temp;
    }

    return b[1];
}

int updateBEMF() {
    int updated_bemf = 0;
    //Back EMF measurements are made automatically by coordination of the ADC, PWM, and DMA.
    //Copy to local variables. Not strictly neccesary, just for clarity.
    //This **REQUIRES** that the divider on the battery & BEMF circuits have the same ratio.
    bemf[0] = adcGetVBatt() - adcGetBEMFL();
    //bemf[0] = ADC1BUF0;
    bemf[1] = adcGetVBatt() - adcGetBEMFR();
    //NOTE: at this point, we should have a proper correspondance between
    //   the order of all the structured variable; bemf[i] associated with
    //   pidObjs[i], bemfLast[i], etc.
    //   Any "jumbling" of the inputs can be done in the above assignments.

    //Negative ADC measures mean nothing and should never happen anyway
    if (bemf[0] < 0) {
        bemf[0] = 0;
    }
    if (bemf[1] < 0) {
        bemf[1] = 0;
    }

//    int i;
//    for (i = 0; i < 1; i++) {
//        if (bemfHist[i][0] != bemf[i]) {
//            bemfHist[i][2] = bemfHist[i][1]; //rotate first
//            bemfHist[i][1] = bemfHist[i][0];
//            bemfHist[i][0] = bemf[i]; //include newest value
//            updated_bemf = 1;
//            prev_time = curr_time;
//            curr_time = sclockGetLocalMillis();
//        }
//    }
    
    //Apply median filter
    int i;
    for (i = 0; i < 1; i++) {
        bemfHist[i][2] = bemfHist[i][1]; //rotate first
        bemfHist[i][1] = bemfHist[i][0];
        bemfHist[i][0] = bemf[i]; //include newest value
        bemf[i] = medianFilter3(bemfHist[i]); //Apply median filter
    }

    // IIR filter on BEMF: y[n] = 0.2 * y[n-1] + 0.8 * x[n]
    bemf[0] = (2 * (long) bemfLast[0] / 10) + 8 * (long) bemf[0] / 10;
    bemfLast[0] = bemf[0]; //bemfLast will not be used after here, OK to set

    return updated_bemf;
//
//    // IIR filter on BEMF: y[n] = 0.2 * y[n-1] + 0.8 * x[n]
//    bemf[0] = (5 * (long) bemfLast[0] / 10) + 5 * (long) bemf[0] / 10;
//    bemf[1] = (5 * (long) bemfLast[1] / 10) + 5 * (long) bemf[1] / 10;
//    bemfLast[0] = bemf[0]; //bemfLast will not be used after here, OK to set
//    bemfLast[1] = bemf[1];
}




static void logTrace(RegulatorError *error, RegulatorOutput *output) {

    int xldat[3];
    int gyrodat[3];
    long* motor_counts;
    RegulatorStateStruct *storage;
    
    storage = ppbuffReadInactive(&reg_state_buff);
    
    if(storage != NULL) {
        quatCopy(&storage->ref, &limited_reference);
        quatCopy(&storage->pose, &pose);        
        //storage->error.w = 0.0;
        //storage->error.x = error->roll_err;
        //storage->error.y = error->pitch_err;
        //storage->error.z = error->yaw_err;
        gyroGetIntXYZ(gyrodat);
        memcpy(&storage->gyro_data, gyrodat, 3*sizeof(int));
        xlGetXYZ(xldat);
        memcpy(&storage->xl_data, xldat, 3*sizeof(int));
        storage->u[0] = output->thrust;
        //storage->u[0] = hallGetOutput();
        storage->u[1] = output->steer;
        storage->u[2] = output->elevator;
        storage->bemf[0] = hallGetBEMF();
        motor_counts = hallGetMotorCounts();
        storage->bemf[1] = motor_counts[0];
        //storage->bemf[1] = (int) (hallGetError()/1000);
        //storage->crank = crankAngle;
        //storage->crank = (float) hallGetError();
        storage->time = sclockGetLocalMillis();
    }
    ppbuffFlip(&reg_state_buff);

}

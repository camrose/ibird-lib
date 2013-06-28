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

#define max( a, b ) ( ((a) > (b)) ? (a) : (b) )

// =========== Static Variables ================================================
// Control loop objects
CtrlPidParamStruct yawPid, pitchPid, thrustPid;
DigitalFilterStruct yawRateFilter, pitchRateFilter, rollRateFilter;

// State info
static unsigned char is_ready = 0, is_logging = 0, temp_rot_active = 0;
static unsigned char yaw_filter_ready = 0, pitch_filter_ready = 0, roll_filter_ready = 0;
static RegulatorMode reg_mode;
static RegulatorOutput rc_outputs;
static Quaternion reference, limited_reference, pose, temp_rot;

// Telemetry buffering
static RegulatorStateStruct reg_states[2];
static PingPongBuffer reg_state_buff;

// =========== Function Stubs =================================================
static float runYawControl(float yaw);
static float runPitchControl(float pitch);
static float runRollControl(float roll);
static void updateBEMF();

static void calculateError(RegulatorError *error);
static void filterError(RegulatorError *error);
static void calculateOutputs(RegulatorError *error, RegulatorOutput *output);
static void applyOutputs(RegulatorOutput *output);
static void logTrace(RegulatorError *error, RegulatorOutput *output);

// =========== Local Variables ===============================================

int bemf[NUM_MOTOR_PIDS]; //used to store the true, unfiltered speed
int bemfLast[NUM_MOTOR_PIDS]; // Last post-median-filter value
int bemfHist[NUM_MOTOR_PIDS][3]; //This is ONLY for applying the median filter to
int medianFilter3(int*);
int max_bemf;
unsigned char halt_wings_closed;
unsigned char wings_stopped;

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

    max_bemf = 0;
    halt_wings_closed = 0;
    wings_stopped = 0;
}

void rgltrSetMode(unsigned char flag) {

    if(flag == REG_OFF) {
        rgltrSetOff();
    } else if(flag == REG_TRACK) {
        rgltrSetTrack();
    } else if(flag == REG_REMOTE_CONTROL) {
        rgltrSetRemote();
    }
        
}

void rgltrSetOff(void) {
    reg_mode = REG_OFF;
    ctrlStop(&yawPid);
    ctrlStop(&pitchPid);
    ctrlStop(&thrustPid);
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
    servoStart();
}

void rgltrStopWings(unsigned char stop) {
    halt_wings_closed = stop;
    if (stop == 0) {
        wings_stopped = 0;
    }
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

void rgltrRunController(void) {
    
    RegulatorError error;        
    RegulatorOutput output;
    int max_bemf_temp;

    if(!is_ready) { return; }    

    attEstimatePose();  // Update attitude estimate
    updateBEMF();

    max_bemf_temp = max(bemfHist[0][0], max(bemfHist[0][1],bemfHist[0][2]));

    if (max_bemf_temp > max_bemf && max_bemf_temp < max_bemf + 10) {
        max_bemf = max_bemf_temp;
    }

    rateProcess();      // Update limited_reference
    slewProcess(&reference, &limited_reference); // Apply slew rate limiting

    attGetQuat(&pose);
    calculateError(&error);    
    calculateOutputs(&error, &output);

    if ((halt_wings_closed == 1 && (bemfHist[0][0] > max_bemf - 5 || bemfHist[0][0] < max_bemf + 5)) || wings_stopped == 1) {
        output.thrust = 0.0;
        wings_stopped = 1;
    }

    applyOutputs(&output);        
    
    if(is_logging) {
        logTrace(&error, &output);
    }
}


// =========== Private Functions ===============================================

static float runYawControl(float yaw) {

    /*float u;

    u = yawPid.offset;

    if (u > yawPid.umax) {
        u = yawPid.umax;
    } else if (u < yawPid.umin) {
        u = yawPid.umin;
    }

    return u;*/
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

void updateBEMF() {
    //Back EMF measurements are made automatically by coordination of the ADC, PWM, and DMA.
    //Copy to local variables. Not strictly neccesary, just for clarity.
    //This **REQUIRES** that the divider on the battery & BEMF circuits have the same ratio.
    //bemf[0] = adcGetVBatt() - adcGetBEMFL();
    bemf[0] = ADC1BUF0;
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

    //Apply median filter
    int i;
    for (i = 0; i < NUM_MOTOR_PIDS; i++) {
        bemfHist[i][2] = bemfHist[i][1]; //rotate first
        bemfHist[i][1] = bemfHist[i][0];
        bemfHist[i][0] = bemf[i]; //include newest value
        bemf[i] = medianFilter3(bemfHist[i]); //Apply median filter
    }

    // IIR filter on BEMF: y[n] = 0.2 * y[n-1] + 0.8 * x[n]
    bemf[0] = (5 * (long) bemfLast[0] / 10) + 5 * (long) bemf[0] / 10;
    bemf[1] = (5 * (long) bemfLast[1] / 10) + 5 * (long) bemf[1] / 10;
    bemfLast[0] = bemf[0]; //bemfLast will not be used after here, OK to set
    bemfLast[1] = bemf[1];
}

static void logTrace(RegulatorError *error, RegulatorOutput *output) {

    RegulatorStateStruct *storage;
    
    storage = ppbuffReadInactive(&reg_state_buff);
    
    if(storage != NULL) {
        quatCopy(&storage->ref, &limited_reference);
        quatCopy(&storage->pose, &pose);        
        storage->error.w = 0.0;
        storage->error.x = error->roll_err;
        storage->error.y = error->pitch_err;
        storage->error.z = error->yaw_err;
        storage->u[0] = output->thrust;
        storage->u[1] = output->steer;
        storage->u[2] = output->elevator;
        //memcpy(storage->bemf, bemf, 2*sizeof(int));
        storage->bemf[0] = ADC1BUF0;
        storage->bemf[1] = bemf[1];
        storage->time = sclockGetLocalTicks();        
    }
    ppbuffFlip(&reg_state_buff);

}

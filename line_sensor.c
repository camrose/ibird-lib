/**
* Copyright (c) 2011-2012, Regents of the University of California
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
* Line Sensor Support
*
* by Cameron Rose
*
* v.beta
*
* Revisions:
*  Cameron Rose     2014-06-12      Initial implementation
*  
* 
* Notes:
*
* TODO:
*/

// ==== REFERENCES ==========================================
#include "line_sensor.h"
#include "utils.h"
#include "timer.h"
#include "adc_pid.h"
#include "sys_clock.h"
#include "carray.h"
#include "counter.h"
#include <stdlib.h>

// ==== CONSTANTS =========================================== 

#define FCY                     (40000000)
#define LS_CLOCK                (_LATE6)
#define LS_SI                   (_LATE3)
#define LOW                     (0)
#define HIGH                    (1)
#define MAX_LINE                (4000)


// ==== STATIC VARIABLES ====================================

static unsigned char is_ready;
static unsigned char pin_is_high;
unsigned char si_is_high;
static unsigned int cnt;
static unsigned int max_cnt;
//static unsigned int cnt_line;
LineCam current_frame;
static unsigned char has_new_frame;
static unsigned char new_line_sent;

unsigned int max_sig;

static CircArray empty_frame_pool, full_frame_pool;

static LineCam getEmptyFrame(void);
static void enqueueEmptyFrame(LineCam frame);
static LineCam getOldestFullFrame(void);
static void enqueueFullFrame(LineCam frame);


static Counter frame_counter;
static Counter px_counter;

// ==== FUNCTION STUBS ======================================

void setupTimer7(unsigned int fs);
void _T7Interrupt(void);



// ==== FUNCTION BODIES =====================================

void lsSetup(LineCam frames, unsigned int num_frames, unsigned int fs) {
    setupTimer7(fs);
    pin_is_high = 0;
    LS_CLOCK = LOW;
    cnt = 0;
    max_cnt = 501;
    unsigned int i;
    current_frame = NULL;

    frame_counter = cntrCreate(); // Frame counter allocation
    if(frame_counter == NULL) { return; }
    px_counter = cntrCreate();
    if(px_counter == NULL) { return; }
    
    empty_frame_pool = carrayCreate(num_frames); // Initialize frame pool
    if(empty_frame_pool == NULL) { return; }
    full_frame_pool = carrayCreate(num_frames); // Initialize frame pool
    if(full_frame_pool == NULL) { return; }

    for(i = 0; i < num_frames; i++) {
        lsReturnFrame(&frames[i]);
    }

    max_sig = 1;

    is_ready = 1;
    has_new_frame = 0;
    new_line_sent = 0;
}

void lsStartCapture(unsigned char flag) {

    if(flag) {
        WriteTimer7(0);
        cntrSet(px_counter, 0);    // Reset row counter
        cntrSet(frame_counter, 0);  // Reset frame counter
        LS_CLOCK = LOW;
        LS_SI = LOW;
        si_is_high = 0;
        pin_is_high = 0;
        cnt = 0;
        EnableIntT7;
    } else {
        DisableIntT7;
        LS_CLOCK = LOW;
        LS_SI = LOW;
        si_is_high = 0;
        pin_is_high = 0;
        cnt = 0;
    }
    

}

void lsSetExposure(unsigned int et, unsigned int fs) {
    DisableIntT7;
    _T7IF = 0;
    CloseTimer7();
    LS_CLOCK = LOW;
    LS_SI = LOW;
    max_cnt = et;
    setupTimer7(fs);   
}

unsigned char lsHasNewFrame(void) {
    return has_new_frame;
}

LineCam lsGetFrame(void) {
    return getOldestFullFrame();
}

void lsReturnFrame(LineCam frame) {
    if(frame == NULL) { return; }
    enqueueEmptyFrame(frame);
}

unsigned int lsGetFrameNum(void) {
    return cntrRead(frame_counter);
}

// =========== Private Functions ===============================================



void __attribute__((interrupt, no_auto_psv)) _T7Interrupt(void) {
    unsigned int px_num;
    unsigned int line;
    
    if (cnt == 0){
        LS_SI = HIGH;
        si_is_high = 1;
        LS_CLOCK = HIGH;
        pin_is_high = 1;
        cnt++;
    } else {
        if(pin_is_high) {
            if (si_is_high){
                LS_SI = LOW;
            }
            px_num = cntrRead(px_counter);
            if (px_num < 129) {
                if(current_frame == NULL) {
                    current_frame = getEmptyFrame(); // Load new frame
                }
                if(current_frame == NULL) { return; }

                line = adcGetLine();
//                if (line > max_sig) {
//                    max_sig = line;
//                }
                current_frame->pixels[px_num] = (unsigned char)(255*((float)line/4000.0));
                cntrIncrement(px_counter);
            } else if (px_num == 129) {
                cntrIncrement(px_counter);
                current_frame->frame_num = cntrRead(frame_counter); // write frame number
                current_frame->timestamp = sclockGetLocalTicks();
                enqueueFullFrame(current_frame); // Add to output queue
                current_frame = NULL;
                cntrIncrement(frame_counter);
            } else {
                cntrIncrement(px_counter);
            }
            LS_CLOCK = LOW; // End pulse
            pin_is_high = 0;

        } else {
            LS_CLOCK = HIGH; // Begin pulse
            pin_is_high = 1;
        }
        cnt++;
        if (cnt > max_cnt) {
            cnt = 0;
            cntrSet(px_counter, 0);
        }
//    } else {
//        cnt++;
//        if (cnt > max_cnt) {
//            cnt = 0;
//        }
//    }
    }

    _T7IF = 0;

}

void setupTimer7(unsigned int frequency) {

    unsigned int con_reg, period;

    // prescale 1:64
    con_reg =     T7_ON &
    T7_IDLE_CON &
    T7_GATE_OFF &
    T7_PS_1_1 &
    T7_SOURCE_INT;

    // period value = Fcy/(prescale*Ftimer)
    period = FCY/(frequency);

    OpenTimer7(con_reg, period);
    ConfigIntTimer7(T7_INT_PRIOR_6 & T7_INT_OFF);

    _T7IF = 0;
}

/**
 * Get the next available empty frame. If no frames are available, automatically
 * dequeues and returns the oldest full frame.
 *
 * @return Next available frame for writing
 */
static LineCam getEmptyFrame(void) {

    LineCam frame;

    frame = carrayPopHead(empty_frame_pool);
    if(frame == NULL) {
        frame = getOldestFullFrame(); // If no more empty frames, get oldest full
    }
    return frame;

}

/**
 * Enqueues a frame for writing into.
 *
 * @param frame CamFrame object to enqueue
 */
static void enqueueEmptyFrame(LineCam frame) {

    carrayAddTail(empty_frame_pool, frame);

}

/**
 * Returns the oldest full frame in the outgoing buffer.
 *
 * @return Oldest full frame object
 */
static LineCam getOldestFullFrame(void) {

    LineCam frame;

    frame = carrayPopHead(full_frame_pool);
    if(carrayIsEmpty(full_frame_pool)) {
        has_new_frame = 0;
    }

    return frame;

}

/**
 * Enqueues a full frame object in the outgoing buffer.
 *
 * @param frame CamFrame object to enqueue
 */
static void enqueueFullFrame(LineCam frame) {

    carrayAddTail(full_frame_pool, frame);
    has_new_frame = 1;

}
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
#include "dsp.h"
#include "larray.h"
#include <stdlib.h>
#include <string.h>

// ==== CONSTANTS ===========================================

#define FCY                     (40000000)
#define LS_CLOCK                (_LATE6)
#define LS_SI                   (_LATE7)
#define LOW                     (0)
#define HIGH                    (1)
#define MAX_LINE                (4000)
#define PX_TO_M                 (35.8209)
#define CENTER_TO_WIDTH         (0.2227)
#define EDGE_THRESH             (1000)
#define MAX_MARKERS             (5)

#define SWAP(x,y) if (d[y] < d[x]) { int tmp = d[x]; d[x] = d[y]; d[y] = tmp; }


// ==== STATIC VARIABLES ====================================

static unsigned char is_ready;
static unsigned int cnt;
static unsigned int max_cnt;
//static unsigned int cnt_line;
static LineCam current_frame;
static unsigned char has_new_frame;
static unsigned char new_line_sent;
static unsigned char found_marker;

static CircArray empty_frame_pool, full_frame_pool;
static LinArray marker_list;

static LineCam getEmptyFrame(void);
static void enqueueEmptyFrame(LineCam frame);
static LineCam getOldestFullFrame(void);
static void enqueueFullFrame(LineCam frame);
static void convolve(int numElems1,int numElems2,
        int* dstV,int* srcV1,int* srcV2);
static void sort6(unsigned char* d);

static void swap2(int* a, int* b, int first, int second);
static void siftDown(int* a, int* b, int start, int end);
static void heapify(int* a, int* b, int count);
static void heapsort(int* a, int* b, int count);

static unsigned char copyMarker(Marker dst, Marker src);

static int indices[109];

static Counter frame_counter;
static Counter px_counter;

static MarkerStruct default_marker;

// ==== FUNCTION STUBS ======================================

void setupTimer7(unsigned int fs);
void _T7Interrupt(void);



// ==== FUNCTION BODIES =====================================

void lsSetup(LineCam frames, unsigned int num_frames, unsigned int fs) {
    unsigned char* default_hold;
    setupTimer7(fs);
    LS_CLOCK = LOW;
    LS_SI = LOW;
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

    for (i = 0; i < 109; i++) {
        indices[i] = i + 10;
    }

    marker_list = larrayCreate(MAX_MARKERS);
    if(marker_list == NULL) {return;}

    default_marker.num_edges = 2;
    default_marker.threshold = EDGE_THRESH;
    default_marker.px_to_m = 25.9;
    default_marker.edges.edges = (unsigned char *)calloc(default_marker.num_edges, sizeof(unsigned char));
    default_marker.edges.edges[0] = 0;
    default_marker.edges.edges[1] = 0;

    larrayReplace(marker_list, 0, &default_marker);

    is_ready = 1;
    has_new_frame = 0;
    new_line_sent = 0;
    found_marker = 0;
}

void lsStartCapture(unsigned char flag) {

    if(flag) {
        WriteTimer7(0);
        cntrSet(px_counter, 0);    // Reset row counter
        cntrSet(frame_counter, 0);  // Reset frame counter
        LS_CLOCK = LOW;
        LS_SI = LOW;
        cnt = 0;
        EnableIntT7;
    } else {
        DisableIntT7;
        LS_CLOCK = LOW;
        LS_SI = LOW;
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

unsigned char lsGetEdges(Marker marker) {
    LineCam frame = NULL;
    int deriv_gauss[9] = {-7,-25,-50,-48,0,48,50,25,7};
    int img_gauss[136];
    int img[128];
    int indices_sort[109];
    int img_sort[109];
    int edge_hold[marker->num_edges];

    int i;

    while (frame == NULL) {
        frame = lsGetFrame();
    }
    for (i=0;i<129;i++){
        img[i] = (int)(frame->pixels[i]);
    }
    convolve(128,9,img_gauss,img,deriv_gauss);

    VectorCopy(109,img_sort,&img_gauss[14]);
    VectorCopy(109,indices_sort,indices);

    heapsort(img_sort, indices_sort, 109);

    for (i=0; i<(marker->num_edges/2); i++) {
        if(abs(img_sort[i])>EDGE_THRESH) {
            marker->edges.edges[i] = indices_sort[i];
        } else {
            marker->edges.edges[i] = 0;
        }
        if(abs(img_sort[108-i])>EDGE_THRESH) {
            marker->edges.edges[marker->num_edges-1-i] = indices_sort[108-i];
        } else {
            marker->edges.edges[marker->num_edges-1-i] = 0;
        }
    }

    for(i = 0; i < marker->num_edges; i++) {
        edge_hold[i] = marker->edges.edges[i];
    }
    heapsort(edge_hold, indices_sort, marker->num_edges);
    for(i = 0; i < marker->num_edges; i++) {
        marker->edges.edges[i] = edge_hold[i];
    }

    marker->edges.frame_num = frame->frame_num;
    marker->edges.timestamp = frame->timestamp;
    
    lsReturnFrame(frame);
    if (marker->edges.edges[0] == 0) {
        return 0;
    }
    return 1;
}

unsigned char lsGetMarker(Marker marker) {
    float center;
    int marker_width;

    if (lsGetEdges(marker) == 0) {
        found_marker = 0;
        return 0;
    }

    center = ((float)(marker->edges.edges[marker->num_edges-1] + marker->edges.edges[0]))/2.0;
    marker_width = marker->edges.edges[marker->num_edges-1] - marker->edges.edges[0];

    marker->edges.location = center;
    if (marker_width > 0) {
        marker->edges.distance = marker->px_to_m/marker_width;
    } else {
        marker->edges.distance = -1;
    }
    found_marker = 1;

    return 1;
}

unsigned char lsFoundMarker() {
    return found_marker;
}

unsigned char lsGetNumMarkers() {
    return marker_list->size;
}

unsigned char lsGetMarkerNumber(Marker m, unsigned char index) {
    Marker m_hold;
    m_hold = (Marker)larrayRetrieve(marker_list,index);
    copyMarker(m, m_hold);
    if (m == NULL) {
        return 0;
    } else {
        return lsGetMarker(m);
    }
}

unsigned char lsAddMarker(Marker m, unsigned char index) {
    LinArrayItem item;
    m->edges.edges = (unsigned char *)calloc(m->num_edges,sizeof(unsigned char));
    if (index > -1) {
        item = larrayReplace(marker_list,index,m);
    } else {
        item = larrayReplace(marker_list,marker_list->size,m);
    }
    if (item == NULL) {
        return 0;
    } else {
        return 1;
    }
}

// =========== Private Functions ===============================================



void __attribute__((interrupt, no_auto_psv)) _T7Interrupt(void) {
    unsigned int px_num;
    unsigned int line;
    
    if (cnt == 0){
        LS_SI = HIGH;
        LS_CLOCK = HIGH;
        cnt++;
    } else {
        if(LS_CLOCK) {
            if (LS_SI){
                LS_SI = LOW;
            }
            px_num = cntrRead(px_counter);
            if (px_num < 129) {
                if(current_frame == NULL) {
                    current_frame = getEmptyFrame(); // Load new frame
                }
                if(current_frame == NULL) { return; }

                line = adcGetLine();

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
        } else {
            LS_CLOCK = HIGH; // Begin pulse
        }
        cnt++;
        if (cnt > max_cnt) {
            cnt = 0;
            cntrSet(px_counter, 0);
        }
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

static void convolve(int numElems1,int numElems2,
        int* dstV,int* srcV1,int* srcV2) {
    int n;
    for (n = 0; n < numElems1 + numElems2 - 1; n++) {
        int kmin, kmax, k;
        dstV[n] = 0;
        kmin = (n >= numElems2 - 1) ? n - (numElems2 - 1) : 0;
        kmax = (n < numElems1 - 1) ? n : numElems1 - 1;
        for (k = kmin; k <= kmax; k++) {
            dstV[n] += srcV1[k] * srcV2[n - k];
        }
    }
}

static void swap2(int* a, int* b, int first, int second) {
    int tmp = a[first];
    a[first] = a[second];
    a[second] = tmp;
    tmp = b[first];
    b[first] = b[second];
    b[second] = tmp;
}

static void siftDown(int* a, int* b, int start, int end) {
    int child, swap, root;
    root = start;
    while (root*2+1 <= end) {
        child = root*2 + 1;
        swap = root;
        if (a[swap] < a[child]) {swap = child;}
        if (child+1 <= end && a[swap] < a[child+1]) {swap = child+1;}
        if (swap == root) {return;}
        else {
            swap2(a,b,root,swap);
            root = swap;
        }
    }
}

static void heapify(int* a, int* b, int count) {
    int start = (int)(floor((count-2)/2));
    while (start >= 0) {
        siftDown(a, b, start, count-1);
        start = start-1;
    }
}

static void heapsort(int* a, int* b, int count) {
    int end;
    heapify(a, b, count);
    end = count-1;
    while (end > 0) {
        swap2(a, b, end, 0);
        end = end - 1;
        siftDown(a, b, 0, end);
    }
}

// Swap sort for 6 items using the Bose-Nelson algorithm
static void sort6(unsigned char* d) {
//#define SWAP(x,y) if (d[y] < d[x]) { int tmp = d[x]; d[x] = d[y]; d[y] = tmp; }
    SWAP(1, 2);
    SWAP(0, 2);
    SWAP(0, 1);
    SWAP(4, 5);
    SWAP(3, 5);
    SWAP(3, 4);
    SWAP(0, 3);
    SWAP(1, 4);
    SWAP(2, 5);
    SWAP(2, 4);
    SWAP(1, 3);
    SWAP(2, 3);
//#undef SWAP
}

static unsigned char copyMarker(Marker dst, Marker src) {
    Marker ret;
    ret = memcpy(dst,src,sizeof(MarkerStruct));
    if (ret == NULL) { return 0; }
    return 1;
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
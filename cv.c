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
* Online computer vision module
*
* by Humphrey Hu
*
* v. beta
*
* Revisions:
*  Humphrey Hu      2011-07-01      Initial implementation
*  Humphrey Hu      2012-02-16      Complete rewrite to use camera driver
*/

#include "attitude.h"
#include "cv.h"
#include "cam.h"
#include "counter.h"
#include "utils.h"
#include "bams.h" // For fast trig
#include "sqrti.h" // For fast integer square root

#include <math.h>
#include <stdlib.h>
#include <string.h>

// State info
static unsigned char is_ready, is_running, hp_on;

static CamFrame background_frame;

// Private functions
static void cvReadFrameParams(CamFrame frame, FrameInfo info);
static void cvCalculateMeans(CamFrame frame, FrameInfo info);
static void cvBackgroundSubtractFrame(CamFrame frame, FrameInfo info);
static void cvCentroidFrame(CamFrame frame, FrameInfo info);
static void cvMaxPixelFrame(CamFrame frame, FrameInfo info);
static void cvRotateFrame(CamFrame frame, bams16_t theta);
static void cvSobel(CamFrame frame, FrameInfo info);
static void cvHighPassPeak(CamFrame frame, FrameInfo info);
static void cvBinary(CamFrame frame, FrameInfo info);

// Helpers
static void cvShiftHoriz(CamFrame frame, int num);
static void cvShiftVert(CamFrame frame, int num);
static void colShift(CamFrame frame, unsigned int col, 
                    unsigned int row_dst, unsigned int row_src, unsigned int num);
static void colSet(CamFrame frame, unsigned int col, unsigned int row_dst,
                    unsigned char val, unsigned int num);
                    
// Setup the CV module
void cvSetup(void) {        
    
    is_ready = 0;
    is_running = 0;    
       
    background_frame = NULL;
    hp_on = 0;
    
    is_ready = 1;

}

void cvSetHP() {

    hp_on = ~hp_on;

}

void cvProcessFrame(CamFrame frame, FrameInfo info) {    

    if(!is_ready) { return; } // Module readiness quick fail       

    cvReadFrameParams(frame, info);     
    //cvRotateFrame(frame, -attGetYawBAMS());
    
    if(hp_on) {
        cvSobel(frame, info);
        //cvBinary(frame, info);
    }
        
    
}

CamFrame cvSetBackgroundFrame(CamFrame frame) {

    CamFrame old;

    old = background_frame;
    background_frame = frame;

    return old;
    
}

// =========== Private Functions ===============================================
/**
 * Read basic frame parameters and write to info struct.
 * @param frame - Input frame
 * @param info - Info struct to populate
 */
void cvReadFrameParams(CamFrame frame, FrameInfo info) {

    if(frame == NULL || info == NULL) { return; };

    // Reset fields
    memset(info, 0, sizeof(FrameInfoStruct));

    // Write frame center location
    info->offset[0] = DS_IMAGE_COLS/2;
    info->offset[1] = DS_IMAGE_ROWS/2;
    
    // Copy frame number
    info->frame_num = frame->frame_num;    

//    unsigned int i;
//    for(i = 0; i < DS_IMAGE_ROWS; i++) {
//        memset(frame->pixels[i], 0, DS_IMAGE_COLS/2);
//        memset(frame->pixels[i] + DS_IMAGE_COLS/2, 0xFF, DS_IMAGE_COLS/2);
//    }

}

/**
 * Subtract the set background frame from the input frame.
 *
 * @param frame - Input frame
 * @param info - Info struct to populate
 */
void cvBackgroundSubtractFrame(CamFrame frame, FrameInfo info) {

    unsigned char cap_val, bg_val;
    unsigned int i, j; 

    if(frame == NULL || info == NULL) { return; }
    if(background_frame == NULL) { return; }

    for(i = 0; i < DS_IMAGE_ROWS; i++) {        
        for(j = 0; j < DS_IMAGE_COLS; j++) {
        
            cap_val = frame->pixels[i][j];
            bg_val = background_frame->pixels[i][j];
            if(cap_val > bg_val) {
                frame->pixels[i][j] = cap_val - bg_val;
            } else {
                frame->pixels[i][j] = 0;
            }
            
        }
    }

}

/**
 * Find the maximum luminosity pixel in the input frame.
 *
 * @param frame - Input frame
 * @param info - Info struct to populate
 */
void cvMaxPixelFrame(CamFrame frame, FrameInfo info) {
    
    unsigned int i, j, max_val, max_loc[2];
    unsigned char val;    
    
    max_val = 0;    
    max_loc[0] = 0;
    max_loc[1] = 1;

    for(i = 0; i < DS_IMAGE_ROWS; i++) {        
        for(j = 0; j < DS_IMAGE_COLS; j++) {
            val = frame->pixels[i][j];
            if(val > max_val) {
                max_val = val;
                max_loc[0] = j;
                max_loc[1] = i;
            }
        }
    }

    info->max[0] = max_loc[0];
    info->max[1] = max_loc[1];
    info->max_lum = max_val;

}

void cvCalculateMeans(CamFrame frame, FrameInfo info) {

    unsigned char i, j, val;
    unsigned long row_acc, col_acc[DS_IMAGE_COLS];
    unsigned long l_acc;
    
    l_acc = 0;
    memset(col_acc, 0, 4*DS_IMAGE_COLS);
    
    for(i = 0; i < DS_IMAGE_ROWS; i++) {
        row_acc = 0;
        for(j = 0; j < DS_IMAGE_COLS; j++) {
            val = frame->pixels[i][j];
            l_acc += val;
            row_acc += val;
            col_acc[j] += val;
        }
        info->row_means[i] = (unsigned char) (row_acc/DS_IMAGE_COLS);
    }

    for(j = 0; j < DS_IMAGE_COLS; j++) {
        info->col_means[j] = (unsigned char) (col_acc[j]/DS_IMAGE_ROWS);
    }

    info->mass = l_acc;
    info->avg_lum = l_acc/(DS_IMAGE_ROWS*DS_IMAGE_COLS);
    
}

/**
 * Find the frame luminosity centroid of the input frame.
 *
 * @param frame - Input frame
 * @param info - Info struct to populate
 */
void cvCentroidFrame(CamFrame frame, FrameInfo info) {
    
    unsigned long x_acc, y_acc, temp;
    unsigned int i;

    // Check for completely blank case
    if(info->avg_lum == 0) {
        info->centroid[0] = info->offset[0];
        info->centroid[1] = info->offset[1];
        return;
    }
    
    // Else calculate centroid
    x_acc = 0;
    y_acc = 0;
    
    for(i = 0; i < DS_IMAGE_ROWS; i++) {            
        y_acc += i*info->row_means[i];
    }   
    for(i = 0; i < DS_IMAGE_COLS; i++) {
        x_acc += i*info->col_means[i];
    }
    y_acc = y_acc*DS_IMAGE_COLS;
    x_acc = x_acc*DS_IMAGE_ROWS;

    temp = x_acc/info->mass;
    info->centroid[0] = (unsigned int) temp;
    temp = y_acc/info->mass;
    info->centroid[1] = (unsigned int) temp;
    

}

static void cvRotateFrame(CamFrame frame, bams16_t theta) {

    float alpha, beta;
    int horiz_shift, vert_shift;
    
    alpha = bams16Tan(theta/2);
    beta = bams16Sin(theta);

    // Calculate total horizontal shift of upper row
    horiz_shift = (int) (alpha*(DS_IMAGE_ROWS/2));
    // Calculate total horizontal shift of right column
    vert_shift = (int) (beta*(DS_IMAGE_COLS/2));
    
    cvShiftHoriz(frame, horiz_shift);    
    cvShiftVert(frame, vert_shift);
    cvShiftHoriz(frame, horiz_shift);

}

// Frame is pointer to CamFrame object
// num is number of pixels top row (row 0) is shifted to the right
static void cvShiftHoriz(CamFrame frame, int num) {

    int shift, half_height, i, width, height;    
    unsigned char *row;
    
    height = (int) DS_IMAGE_ROWS; // frame->num_rows;
    width = (int) DS_IMAGE_COLS; // frame->num_cols;
    half_height = (int) height/2;
    
    for(i = 0; i < height; i++) {
            
        row = frame->pixels[i];
        shift = ((i - half_height)*num)/half_height;
        
        if(shift == 0) {
            // Do nothing
        } else if(shift > 0) {
            
            memmove(row + shift, row, width - shift);
            memset(row, 0, shift);
        
        } else { // shift < 0
        
            shift = -shift;
            memmove(row, row + shift, width - shift);
            memset(row + width - shift, 0, shift);
            
        }

        int j;
        j = 0;

    }

}

// Num is number of pixels leftmost column (col 0) is shifted down
static void cvShiftVert(CamFrame frame, int num) {

    int shift, half_width, i, width, height;        
    
    height = (int) DS_IMAGE_ROWS; // frame->num_rows;
    width = (int) DS_IMAGE_COLS; // frame->num_cols;
    half_width = (int) width/2;
    
    for(i = 0; i < width; i++) {
                    
        shift = ((i - half_width)*num)/half_width;
        
        if(shift == 0) {
            // Do nothing
        } else if(shift > 0) {
            
            colShift(frame, i, 0, shift, height - shift);
            colSet(frame, i, height - shift, 0, shift); 
            
        } else { // shift < 0
        
            shift = -shift;
            colShift(frame, i, shift, 0, height - shift); 
            colSet(frame, i, 0, 0, shift);
            
        }

        int j;
        j = 0;

    }

}

static void colShift(CamFrame frame, unsigned int col,
                    unsigned int row_dst, unsigned int row_src, unsigned int num) {

    int i, shift, step;
    unsigned int cnt;    
        
    shift = row_dst - row_src;
    
    if(shift == 0) {
        return;
    } else if(shift > 0) { // Start from tail
        step = -1;
        i = num - 1;
    } else { // shift < 0, start from head
        step = 1;
        i = 0;
    }
    
    cnt = num;
    while(cnt--) {
        frame->pixels[row_dst + i][col] = 
            frame->pixels[row_src + i][col];
        i = i + step;
    }
    
}

static void colSet(CamFrame frame, unsigned int col, unsigned int row_dst,
                unsigned char val, unsigned int num) {

    unsigned int cnt, i;
    
    cnt = num;
    i = 0;
    
    while(cnt--) {
        frame->pixels[row_dst + i][col] = val;
        i++;
    }
                
}

const char sobel_hor_kernel[3] = {-1, 0, 1};
const char sobel_ver_kernel[3] = {1, 2, 1};
#define SOBEL_HOR_SCALE         (1)
#define SOBEL_VER_SCALE         (4)
#define SOBEL_IMAGE_COLS       (DS_IMAGE_COLS - 2)
#define SOBEL_IMAGE_ROWS       (DS_IMAGE_ROWS - 2)

static void cvSobel(CamFrame frame, FrameInfo info) {

    unsigned int i, j, k;
    int h_acc, v_acc, h_grad[SOBEL_IMAGE_COLS], v_grad[SOBEL_IMAGE_COLS];
    unsigned char val, temp;
    unsigned char h_buffs[3][SOBEL_IMAGE_COLS],
                    v_buffs[3][SOBEL_IMAGE_COLS];
    unsigned char indices[3] = {2, 0, 1};

    for(j = 0; j < SOBEL_IMAGE_COLS; j++) {

        h_acc = (-frame->pixels[0][j] + frame->pixels[0][j + 2])/SOBEL_HOR_SCALE;
        if(h_acc < 0) { h_acc = -h_acc; }
        h_buffs[0][j] = h_acc;
        h_acc = (-frame->pixels[1][j] + frame->pixels[1][j + 2])/SOBEL_HOR_SCALE;
        if(h_acc < 0) { h_acc = -h_acc; }
        h_buffs[1][j] = h_acc;

        v_buffs[0][j] = (frame->pixels[0][j] + 2*frame->pixels[0][j + 1] +
                frame->pixels[0][j + 2])/SOBEL_VER_SCALE;
        v_buffs[1][j] = (frame->pixels[1][j] + 2*frame->pixels[1][j + 1] +
                frame->pixels[1][j + 2])/SOBEL_VER_SCALE;
    }

    for(i = 2; i < DS_IMAGE_ROWS; i++) {
        for(j = 0; j < SOBEL_IMAGE_COLS; j++) {            
            h_acc = (-frame->pixels[i][j] + frame->pixels[i][j + 2])/SOBEL_HOR_SCALE;
            v_acc = (frame->pixels[i][j] + 2*frame->pixels[i][j + 1] +
                    frame->pixels[i][j + 2])/SOBEL_VER_SCALE;

            if(h_acc < 0) { h_acc = -h_acc; } // Accumulators range [-255, 255]           

            h_buffs[indices[0]][j] = (unsigned char) h_acc;
            v_buffs[indices[0]][j] = (unsigned char) v_acc;
        
        } // Row complete
        
        temp = indices[0];          // Rotate the indices left
        indices[0] = indices[1];
        indices[1] = indices[2];
        indices[2] = temp;                

        memset(h_grad, 0, 2*SOBEL_IMAGE_COLS);
        memset(v_grad, 0, 2*SOBEL_IMAGE_COLS);

        for(j = 0; j< SOBEL_IMAGE_COLS; j++) {
            h_grad[j] += h_buffs[indices[0]][j] + 2*h_buffs[indices[1]][j] +
                        h_buffs[indices[2]][j];
            v_grad[j] += v_buffs[indices[0]][j] - v_buffs[indices[2]][j];
        }

        for(j = 0; j < SOBEL_IMAGE_COLS; j++) {
            v_acc = v_grad[j]/SOBEL_HOR_SCALE;
            h_acc = h_grad[j]/SOBEL_VER_SCALE;
            if(v_acc < 0) { v_acc = -v_acc; }
            frame->pixels[i - 2][j] = (v_acc + h_acc)/2;
        }

    }
    
}

#define BIN_THRESHOLD       (30)

static void cvBinary(CamFrame frame, FrameInfo info) {

    unsigned int i, j;
    unsigned char val;    

    for(i = 0; i < DS_IMAGE_ROWS; i++) {
        for(j = 0; j < DS_IMAGE_COLS; j++) {
            val = frame->pixels[i][j];
            if(val > BIN_THRESHOLD) {
                frame->pixels[i][j] = 0xFF;
            } else {
                frame->pixels[i][j] = 0;
            }
        }
    }

}

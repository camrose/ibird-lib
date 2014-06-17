/******************************************************************************
* Name: line_sensor.h
* Desc: line sensor support
* Date: 2014-06-12
* Author: Cameron Rose
******************************************************************************/
#ifndef __LINE_SENSOR_H
#define __LINE_SENSOR_H

typedef struct {
    unsigned long timestamp;
    unsigned int frame_num;
    unsigned char pixels[129];
} LineCamStruct;
typedef LineCamStruct* LineCam;

void lsSetup(LineCam frames, unsigned int num_frames, unsigned int fs);
void lsStartCapture(unsigned char flag);
unsigned char lsHasNewFrame(void);
LineCam lsGetFrame();
void lsSetExposure(unsigned int et, unsigned int fs);
void lsReturnFrame(LineCam frame);
unsigned int lsGetFrameNum(void);

#endif // __LINE_SENSOR_H


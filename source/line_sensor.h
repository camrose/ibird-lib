/******************************************************************************
* Name: line_sensor.h
* Desc: line sensor support
* Date: 2014-06-12
* Author: Cameron Rose
******************************************************************************/
#ifndef __LINE_SENSOR_H
#define __LINE_SENSOR_H

#define LINE_VIEW_ANGLE     (0.558)  // view angle in radians (32 degrees)
#define LINE_FRAME_WIDTH    (127)    // frame width in pixels

typedef struct {
    unsigned long timestamp;
    unsigned int frame_num;
    unsigned char pixels[128];
} LineCamStruct;
typedef LineCamStruct* LineCam;

typedef struct {
    unsigned long timestamp;
    unsigned int frame_num;
    unsigned char *edges;
    float location;
    float distance;
} EdgesStruct;
typedef EdgesStruct* Edges;

typedef struct {
    unsigned char num_edges;
    unsigned int threshold;
    float px_to_m;
    EdgesStruct edges;
} MarkerStruct;
typedef MarkerStruct* Marker;

void lsSetup(LineCam frames, unsigned int num_frames, unsigned int fs);
void lsStartCapture(unsigned char flag);
unsigned char lsHasNewFrame(void);
LineCam lsGetFrame();
void lsSetExposure(unsigned int et, unsigned int fs);
void lsReturnFrame(LineCam frame);
unsigned int lsGetFrameNum(void);
unsigned char lsGetEdges(Marker marker);
unsigned char lsGetMarker(Marker marker);
unsigned char lsFoundMarker();
unsigned char lsGetNumMarkers();
unsigned char lsGetMarkerNumber(Marker m, unsigned char index);
unsigned char lsAddMarker(Marker m, unsigned char index);

#endif // __LINE_SENSOR_H


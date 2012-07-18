/* 
 * File:   cmd.h
 * Author: Cameron
 *
 * Created on July 17, 2012, 5:22 PM
 */

#ifndef __CMD_H
#define	__CMD_H

#include "mac_packet.h"

unsigned int cmdSetup(unsigned int queue_size);
void cmdProcessBuffer(void);
unsigned int cmdQueuePacket(MacPacket packet);





#endif	/* CMD_H */


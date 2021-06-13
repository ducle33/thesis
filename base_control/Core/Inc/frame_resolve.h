/*
*   File name: PID.c
*   Author: Linh Nguyen
*   Last modified: 1/6/2021
*   Description: This contains all interface functions for frame resolving.
*/



#ifndef _FRAME_RESOLVE_H_
#define _FRAME_RESOLVE_H_



uint32_t checksum(uint8_t *d);

uint8_t parseChecksum(uint8_t *d, uint32_t crc);

void parseTxFrame(uint8_t *d /* uint8_t *d */, float vel_linear, float vel_angular);

void float2byte(float *f, uint8_t *d, uint8_t s);

float * resolveRxFrame(uint8_t * d);


#endif
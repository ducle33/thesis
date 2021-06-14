/*
*   File name: frame_resolve.c
*   Author: Linh Nguyen
*   Last modified: 12/6/2021
*   Description: This source file contain all auxillary fucntions
*                for resolving received frame from DMA.  
*/



#include "main.h"
#include <stdio.h>


// Aux fucntion
void float2byte(float *f, uint8_t *d, uint8_t s)
{
    uint32_t * tmp;
    tmp = (uint32_t*)f;
    uint8_t i = 0;
    for (i = 0; i<s; i++)
    {      
        d[i] = *tmp>>(24-8*i);
    }
}

// Calc checksum
uint32_t checksum(uint8_t *d)
{
    uint32_t _crc = 0;
    uint8_t i = 0;
    for (i = 2; i < 10; i++)
    {
        _crc+= d[i];
    }
    return _crc;
}

// Parse frames for transmition
void parseTxFrame(uint8_t *d /* uint8_t *d */, float vel_linear, float vel_angular)
{
    // Header
    d[0] = 1;
    d[1] = 2; 
  
    // Resolve float to 4 bytes binary.
    float2byte(&vel_linear, &d[2], 4);
    float2byte(&vel_angular, &d[6], 4);

    uint32_t crc = checksum(d);   
   
    d[10] = (crc >> 24) & 0xFF;
    d[11] = (crc >> 16)  & 0xFF; 
    d[12] = (crc >> 8)  & 0xFF; 
    d[13] = (crc )  & 0xFF; 
    
    // EOF
    d[14] = '\r';
    d[15] = '\n';
}


void resolveRxFrame(uint8_t * d, float * linear, float * angular)
{
    uint32_t tmp1 = 0;
    uint32_t tmp2 = 0;
    uint32_t rx_crc, local_crc;
    
    rx_crc = d[10] << 24 | d[11]<<16 | d[12]<<8 | d[13]; 
    tmp1 = d[2] << 24 | d[3]<<16 | d[4]<<8 | d[5];
    tmp2 = d[6] << 24 | d[7]<<16 | d[8]<<8 | d[9];
    
    local_crc = checksum(d);

    if (local_crc == rx_crc)
    {
        *linear = *(float *)&tmp1;
        *angular = *(float *)&tmp2;
    }
}

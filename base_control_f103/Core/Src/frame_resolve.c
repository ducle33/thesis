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

void double2byte(double *d, uint8_t *buffer, uint8_t s)
{
    union {
        double _d;
        uint8_t bytes[8];
    } container;
    container._d = *d;
    uint8_t ii;
    for (ii=0; ii<8; ii++) 
        buffer[ii] = container.bytes[7-ii];

}

// Calc checksum
uint32_t checksum(uint8_t *d, uint8_t from, uint8_t to)
{
    uint32_t _crc = 0;
    uint8_t i = 0;
    for (i = from; i < to; i++)
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

    uint32_t crc = checksum(d, 2 , 10);   
   
    d[10] = (crc >> 24) & 0xFF;
    d[11] = (crc >> 16)  & 0xFF; 
    d[12] = (crc >> 8)  & 0xFF; 
    d[13] = (crc )  & 0xFF; 
    
    // EOF
    d[14] = '\r';
    d[15] = '\n';
}

void parseTxStateFrame(uint8_t *d /*18bytes*/, double x, double y, double th)
{
    // Header
    d[0] = 1;
    d[1] = 2; 
  
    // Resolve float to 4 bytes binary.
    double2byte(&x, &d[2], 8);
    double2byte(&y, &d[10], 8);
    double2byte(&th, &d[18], 8);

    uint32_t crc = checksum(d, 2, 26);   
   
    d[26] = (crc >> 24) & 0xFF;
    d[27] = (crc >> 16)  & 0xFF; 
    d[28] = (crc >> 8)  & 0xFF; 
    d[29] = (crc )  & 0xFF; 
    
    // EOF
    d[30] = '\r';
    d[31] = '\n';
}


void resolveRxFrame(uint8_t * d, float * linear, float * angular)
{
    uint32_t tmp1 = 0;
    uint32_t tmp2 = 0;
    uint32_t rx_crc, local_crc;
    
    rx_crc = d[10] << 24 | d[11]<<16 | d[12]<<8 | d[13]; 
    tmp1 = d[2] << 24 | d[3]<<16 | d[4]<<8 | d[5];
    tmp2 = d[6] << 24 | d[7]<<16 | d[8]<<8 | d[9];
    
    local_crc = checksum(d, 2, 10);

    if (local_crc == rx_crc)
    {
        *linear = *(float *)&tmp1;
        *angular = *(float *)&tmp2;
    }
}


#ifndef _RING_BUFFER_H_ 
#define _RING_BUFFER_H_


#include <stdio.h>

typedef struct 
{
    size_t  size;
    uint8_t *pBuffer;
    uint8_t *head;
    uint8_t *tail;
    uint8_t filled;
    uint8_t isFull;
} BuffefHandler_t;





#endif
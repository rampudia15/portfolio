#ifndef __API_H__
#define __API_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "InfoFIFO.h"
#include "memmap.h"

void initFifoInfo(int bufferSize){
    BUFFER_SIZE = bufferSize;
    
}

int createFifo(int* shmaddr, int size){
    
    struct Buffer FIFO;
    FIFO.queueLength = BUFFER_SIZE;
    FIFO.writeIndex = 0;
    FIFO.readIndex = 0;
    
    struct Buffer *shamddr = malloc(size);
    shmaddr = &FIFO;
    
    return 1;
    
}

int* openFifo(int* shmaddr){
    int *ptr = shmaddr;
    return ptr;
}


int readFifo(struct Buffer *buffer, int *rx){
    
    if(buffer->queueLength == 0) {
        printf("Buffer is empty\n");
        *rx = NULL;
        return 0;
    } else {
        *rx = buffer->queue[buffer->readIndex];
        buffer->queueLength--;
        buffer->readIndex++;
    }
    
    if(buffer->readIndex == BUFFER_SIZE) { 
        buffer->readIndex = 0;
    }
    
    return 1;
}

int writeFifo(struct Buffer *buffer, int *tx){
    
    if (buffer->queueLength == BUFFER_SIZE){ 
        printf("Buffer is full!\n");
        return 0;
    }
    else {
        buffer->queue[buffer->writeIndex] = *tx;
        buffer->queueLength++;
        buffer->writeIndex++;
    }
    if (buffer->writeIndex == BUFFER_SIZE){
        buffer->writeIndex=0;
    }
    return 1;
}


#endif


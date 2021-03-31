// =========================================================
// AUTHOR     : Ricardo Ampudia
// CREATE DATE     : 02/2020
// PURPOSE     :   RTS 
// DESCRIPTION  : Small library to enable communication of 
// two independent processes with a shared memory using a 
// FIFO buffer.
//==========================================================

#ifndef __API_H__
#define __API_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "InfoFIFO.h"
#include "memmap.h"

//Initializes buffer configuration
void initFifoInfo(int bufferSize){
    BUFFER_SIZE = bufferSize;
    
}

//Creates FIFO channel for communication
int createFifo(int* shmaddr, int size){
    
    struct Buffer FIFO;
    FIFO.queueLength = BUFFER_SIZE;
    FIFO.writeIndex = 0;
    FIFO.readIndex = 0;
    
    struct Buffer *shamddr = malloc(size);
    shmaddr = &FIFO;
    
    return 1;
    
}

//Returns the pointer to the shared memory address
int* openFifo(int* shmaddr){
    int *ptr = shmaddr;
    return ptr;
}


//Read stored data in the buffer
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

//Writes data into the buffer
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


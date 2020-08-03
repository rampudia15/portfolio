#ifndef __InfoFIFO_H__
#define __InfoFIFO_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_BUFFER_SIZE 16

int BUFFER_SIZE = 16;

struct Buffer {
    int queue[MAX_BUFFER_SIZE]; //Array that holds queue values
    int queueLength; //Current length of the queue
    int writeIndex;
    int readIndex;
};


#endif

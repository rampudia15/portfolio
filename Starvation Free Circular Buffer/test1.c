#Example that allows two tasks to communicate using the circular buffer functions found in api.h

#if 1

#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include <kernel.h>
#include <kernel2.h>
#include <kernelConst.h>
#include <api.h>
#include <InfoFIFO.h>
#include <fcntl.h>
#include <unistd.h>


uint32_t *shmaddr;
uint32_t *shmaddr_flag;
uint32_t numFifos=0;
uint32_t  apiDebug;
uint32_t A[10];

void task1(){
  int i,j;
  int tx = 1;
  volatile struct Info_FIFO *ififo;
  int data[16];
  int result=0;

  ififo=openFifo(shmaddr); //openFifo(ptr);
  
  for(j=0;j<9;j++){
    A[j]=j;
  }

  for(j=0;j<10000000;j++){
    for(i=0;i<16;i++){
      data[i]=j*16+i;
      //      fprintf(stdout, "task1 data wrt %d \n",j*16+i);
    }
     
     //if(tx<50){
        fprintf(stdout, "<-- task1 sent %d to queue -->\n", tx);
        result=writeFifo(ififo, &tx); //result=writeFifo(ptr,data);
        tx++;
     //}

  }
}

void task2(){
  int i,j;
  int rx;
  int data[16];
  int result=0;
  volatile struct Info_FIFO *ififo;

  printf("task 2 shmaddr %x data %x\n",shmaddr,*shmaddr);

  ififo=openFifo(shmaddr);
  
  for(j=0;j<10000000;j++){
      
      result=readFifo(ififo, &rx);
      fprintf(stdout, "<-- task2 read %d from queue -->\n", rx);
      
    for(i=0;i<16;i++){
      //fprintf(stdout, "task2 data read %d \n",data[i]);
    }
    //fprintf(stdout, "task2 %d %d\n",j,result);
  }
}

void task3(){
  int i,j;  
  while(1){
    for(j=0;j<9;j++){
      printf("A %d %d\n\n", j, A[j]);
    }
  }
  for(j=0;j<10000000;j++){
    for(i=0;i<100;i++){
    }
    fprintf(stdout, "task3 %d\n",j);
  }
}

void task4(){
  int i,j;
  for(j=0;j<10000000;j++){
    for(i=0;i<100;i++){
    }
    fprintf(stdout, "task4 %d\n",j);
  }
}


int main()
{
  volatile uint32_t *ptr, *ptr2;
  printf("main\n");

  while(*shmaddr_flag==0);
  printf("flag has been set\n");

  int i,j;

  uint32_t result;

  shmaddr=FIFO_ADMINISTRATION;
  initFifoInfo(16); //initFifoInfo(bufferSize);

  numFifos=0; //0
  apiDebug=0;
  ptr=0x900000; // debug address

  *ptr=9999;
  printf("main1 %x\n",shmaddr);  

  result = createFifo(shmaddr,16*4); //createFifo(shmaddr, size)
  
  InitTaskInfo(shmaddr); 

  CreateNewTask(1, (uint32_t)&task1, 100000, 0);
  //CreateNewTask(2, (uint32_t)&task3, 100000, 0);

  TaskStart(1); 
  //TaskStart(2);

  InitKernel();

  for(j=0;j<1000;j++){
    for(i=0;i<1000;i++){
    }
  }
  return 0;

}

int main2()
{ 
  int i,j;

  uint32_t result;
  volatile uint32_t *ptr, *ptr2;

  shmaddr_flag=FIFO_ADMINISTRATION+0x10000;

  *shmaddr_flag=0;

  printf("begin main2 wait1\n");
  // wait loop
  for(i=0;i<100;i++){
  }
  printf("end main2 wait\n");

  *shmaddr_flag=1;

  ptr=0x900000; // debug address

  *ptr=9999;
  

  for(i=0;i<10000;i++){};

  printf("main2 %x\n",shmaddr); 

  numFifos=0; //0
  apiDebug=0;

  InitTaskInfo2(shmaddr);

  CreateNewTask2(1, (uint32_t)&task2, 10000000, 0); 
  //CreateNewTask2(2, (uint32_t)&task4, 10000000, 0);

  //TaskStart2(1); 
  TaskStart2(1);


  InitKernel2();



  for(j=0;j<1000;j++){
    for(i=0;i<1000;i++){
    }
  }
  return 0;

}

#endif

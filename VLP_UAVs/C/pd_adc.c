/*This program aquires ADC samples from a PD and executes a FFT 
in real time in the Crazyflie 2.1. The result is a vector with the 
received power in [uW] corresponding to 4 different light sources 
transmitting at different frequencies.

The instructions to add a new task (such as this one) to the 
firmware of the Crazyflie are found in the following URL:
https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/development/systemtask/

To perform the FFT in the Crazyflie, the project object file of the
arm_cfft_init_f32.o function must be added manually as explained
here:
https://forum.bitcraze.io/viewtopic.php?f=6&t=4844&p=22159#p22159

Ricardo Ampudia <r.ampudia15@gmail.com>
Last modification: August 2021 */

#include "config.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "static_mem.h"
#include "task.h"

#include "log.h"
#include "param.h"
#include "deck_constants.h"
#include "deck_analog.h"
#include "deck_digital.h"

#include "pd_adc.h"

#include "deck.h"
#include "math.h"
#include "arm_math.h"

#define SAMPLES 64
#define PI 3.1415926f
#define N_LEDS 4
#define M_ORDER 14
#define A_PD 5.2
#define PT 1.35
#define SAMPLING_FREQ 1000
#define ADC_RES 12
#define WINDOW 5

//Declarations
static float32_t Pr[N_LEDS]; //Power received = {P_f1, P_f2, ...}
static float32_t Pr_tmp[N_LEDS]; //Power received = {P_f1, P_f2, ...}
static float32_t D[N_LEDS]; //Distance from LEDs to PD = {d1, d2, ...}
static float32_t pos[3]; //pos = {x,y,z}
static float32_t adc_signal[SAMPLES]; //Raw signal from the ADC readings
static float32_t P_signal[SAMPLES]; //Input signal in uW
static float32_t V[SAMPLES];

//Declarations for MLE
static float32_t A_f32[2*(N_LEDS-1)];
static float32_t B_f32[N_LEDS-1];
static float32_t AT_f32[2*3];
static float32_t ATMA_f32[2*2];
static float32_t ATMAI_f32[2*2];
static float32_t ALL_A_f32[2*3];
static float32_t POS_f32[2];

//Signal parameters
int f[N_LEDS] = {60,120,240,480}; //{freq1, freq2, ...}
int freq_index[N_LEDS]; //Holds the index of the DFT bin corresponding to f[N_LEDS]

//FFT declarations
static float32_t rfft_output[SAMPLES*2];
static float32_t test_output[SAMPLES];
static float32_t maxvalue;
static uint32_t maxindex;

//Tx positions in 3D
float Tx1[N_LEDS] = {0.25, 1, 0};
float Tx2[N_LEDS] = {1, 1.75, 0};
float Tx3[N_LEDS] = {1.75, 1, 0};
float Tx4[N_LEDS] = {1, 0.25, 0};

//Allocate memory for the task
static void pdAdcTask(void*);
STATIC_MEM_TASK_ALLOC(pdAdcTask, PD_ADC_TASK_STACKSIZE);

static bool isInit = false;

// Initialize task
void pdAdcTaskInit() {
  STATIC_MEM_TASK_CREATE(pdAdcTask, pdAdcTask, PD_ADC_TASK_NAME, NULL, PD_ADC_TASK_PRI);
  isInit = true;
  DEBUG_PRINT("Initialized the PD ADC task\n");
}

//Test task initialized correctly
bool pdAdcTaskTest() {
  return isInit;
}

//Sample data from the PD and execute FFT
static void pdAdcTask(void* parameters) {

//Declarations
int arr_pos = 0;
float h; //Height in m

DEBUG_PRINT("FFT test function is running!");
const TickType_t xDelay =  1 / portTICK_PERIOD_MS;
const TickType_t xDelay2 =  20 / portTICK_PERIOD_MS;

//Initialize frequency index
for(int i=0; i<N_LEDS; i++){
    freq_index[i] = f[i]*SAMPLES/SAMPLING_FREQ;
}

//Initialize X,Y.Z coordinate arrays for LEDs
//for (int i=0; i<N_LEDS; i++) {
float X[N_LEDS] = {Tx1[0], Tx2[0], Tx3[0], Tx4[0]};
float Y[N_LEDS] = {Tx1[1], Tx2[1], Tx3[1], Tx4[1]};
float Z[N_LEDS] = {Tx1[2], Tx2[2], Tx3[2], Tx4[2]};

//Start ADC
adcInit();

//Begin loop
while (true) {

    //Acquire ADC samples
   	for(int i=0; i<SAMPLES; i++){
        adc_signal[i] = analogRead(DECK_GPIO_TX2);
   		vTaskDelay(xDelay);
   	}

   	//Rescale ADC
    rescale_ADC(adc_signal, P_signal, SAMPLES, VREF, ADC_RES);

    //Perform FFT on the input signal to obtain Pr (uW) from the LEDs
    fft_routine_cmsis(P_signal, SAMPLES, Pr_tmp, N_LEDS);

    /* The positioning method is executed offline in Matlab, but some of the
    functions necessary for an online implementation are already available 
    using the CMSIS DSP lib for real-time execution: */
    
    //Aquire height from complementary filter (not programmed)
    //h = 1.75; 

    //Obtain distance vector from Pr
    //distance_calc(Pr_tmp, D, M_ORDER, A_PD, PT, h, N_LEDS);

    //Trilateration via most Likelihood Estimation
    //calc_position_MLE(X, Y, h, D, pos, N_LEDS);
    //DEBUG_PRINT("Pos = [%f, %f, %f] \n\r", pos[0], pos[1], pos[2]);

    //Save final result to log variable
	Pr[0] = Pr_tmp[0];
	Pr[1] = Pr_tmp[1];
	Pr[2] = Pr_tmp[2];
	Pr[3] = Pr_tmp[3];
  
    vTaskDelay(xDelay2);
  }
}

//Re-scale ADC and perfrom conversion to obtain Power Received (uW)
void rescale_ADC(float *adc, float *P, int samples, float Vref, int adc_res){
	//Create simulated input signal
	for (int i=0;i<samples;i++) {
		V[i] = adc[i]*Vref/pow(2,adc_res); //Convert ADC signal to voltage
		P[i] = (V[i] - 0.0001)/0.045; //Power conversion from PD configuration
    }
}

//Perform the FFT to retriee the Pr from different light sources
void fft_routine_cmsis(float *P, int samples, float *Pr, int n_leds){

	//Instances
	arm_rfft_fast_instance_f32 S;

	//Perform RFFT
	arm_rfft_fast_init_f32(&S, samples);
	arm_rfft_fast_f32(&S, P, rfft_output,0);

	//Obtain the magnitude of the complex coefficients
	arm_cmplx_mag_f32(rfft_output,test_output, samples*2);

	test_output[0] = 0; //DC component

	//Analyze the frequency window of interest (f[n] +/-3 bins)and obtain the magnitude of the peak
	float temp_freq[4];
	int k=0;
	for(int i=0; i<n_leds; i++){
		k=0;
		for (int j=freq_index[i]-2; j<freq_index[i]+2; j++){
			temp_freq[k]=2*test_output[j]/samples; //Retrieve magnitude of the bins in the frequency window
			k++;
		}
			arm_max_f32(temp_freq, 4, &maxvalue, &maxindex); //Obtain max value within window
			Pr[i] = maxvalue*PI/4; //Square wave factor
	}
}

//Calculate distance to a source considering Lambertian radiation and parallel Tx-Rx
void distance_calc(float *Pr, float *D, float m, float A_pd, float Pt, float h, int n_leds){
	for (int i=0; i<n_leds; i++){
		D[i] = (m+1)*A_pd*Pt*pow(h,(m+1))/(2*PI*Pr[i]);
		D[i] = pow(D[i], 1.0/(m+3));
	}
	//DEBUG_PRINT("D = [%f, %f, %f, %f ] \n\r", D[0], D[1], D[2], D[3]);
}

//Obtain the 2D position using the Most Likelihood Estimation method
void calc_position_MLE(float *X, float *Y, float h, float *D, float *pos, int n_leds){

	//Construct A matrix for MLE (array format)
	for (int i=0; i<n_leds-1; i++){
		A_f32[2*i] = 2*(X[i]-X[n_leds-1]);
		A_f32[2*i+1] = 2*(Y[i]-Y[n_leds-1]);
	}
	//Construct b matrix for MLE (array format)
	for (int i=0; i<n_leds-1; i++){
		B_f32[i] = pow(X[i], 2) + pow(Y[i], 2) - pow(D[i], 2) - (pow(X[n_leds-1], 2) + pow(Y[n_leds-1], 2) - pow(D[n_leds-1], 2));
	}

	//Initialize buffers for matrix operations
	/*float32_t AT_f32[2*3];
	float32_t ATMA_f32[2*2];
	float32_t ATMAI_f32[2*2];
	float32_t ALL_A_f32[2*3];
	float32_t POS_f32[2];*/

	//Initialize matrix instances
	arm_matrix_instance_f32 A;      // Matrix A Instance
	arm_matrix_instance_f32 AT;     // Matrix AT(A transpose) instance
	arm_matrix_instance_f32 ATMA;   // Matrix ATMA( AT multiply with A) instance
	arm_matrix_instance_f32 ATMAI;  // Matrix ATMAI(Inverse of ATMA) instance
	arm_matrix_instance_f32 ALL_A;  // Matrix ALL_a(ATMAI * AT) instance
	arm_matrix_instance_f32 B;      // Matrix B instance
	arm_matrix_instance_f32 POS;      // Matrix POS(Unknown Matrix) instance
	uint32_t srcRows, srcColumns;

	//A
	srcRows = 3;
	srcColumns = 2;
	arm_mat_init_f32(&A, srcRows, srcColumns, (float32_t *)A_f32);

	//AT (A transpose)
	srcRows = 2;
	srcColumns = 3;
	arm_mat_init_f32(&AT, srcRows, srcColumns, AT_f32);
	arm_mat_trans_f32(&A, &AT);

	//ATMA (AT*A)
	srcRows = 2;
	srcColumns = 2;
	arm_mat_init_f32(&ATMA, srcRows, srcColumns, ATMA_f32);
	arm_mat_mult_f32(&AT, &A, &ATMA);

	//ATMAI (inv(AT*A))
	srcRows = 2;
	srcColumns = 2;
	arm_mat_init_f32(&ATMAI, srcRows, srcColumns, ATMAI_f32);
	arm_mat_inverse_f32(&ATMA, &ATMAI);

	//ALL_A (inv(AT*A)*AT)
	srcRows = 2;
	srcColumns = 3;
	arm_mat_init_f32(&ALL_A, srcRows, srcColumns, ALL_A_f32);
	arm_mat_mult_f32(&ATMAI, &AT, &ALL_A);

	//B
	srcRows = 3;
	srcColumns = 1;
	arm_mat_init_f32(&B, srcRows, srcColumns, (float32_t *)B_f32);

	//X=(inv(AT*A)*AT)*B
	srcRows = 2;
	srcColumns = 1;
	arm_mat_init_f32(&POS, srcRows, srcColumns, POS_f32);
	arm_mat_mult_f32(&ALL_A, &B, &POS);

	//Save results to pos
	pos[0] = POS.pData[0];
	pos[1] = POS.pData[1];
	pos[2] = h;
}

//Create log group to transmit to Crazyflie
LOG_GROUP_START(PD_ADC)
LOG_ADD(LOG_FLOAT, Pr_1, &Pr[0])
LOG_ADD(LOG_FLOAT, Pr_2, &Pr[1])
LOG_ADD(LOG_FLOAT, Pr_3, &Pr[2])
LOG_ADD(LOG_FLOAT, Pr_4, &Pr[3])
LOG_ADD(LOG_FLOAT, D_1, &D[0])
LOG_ADD(LOG_FLOAT, D_2, &D[1])
LOG_ADD(LOG_FLOAT, D_3, &D[2])
LOG_ADD(LOG_FLOAT, D_4, &D[3])
LOG_ADD(LOG_FLOAT, pos_x, &pos[0])
LOG_ADD(LOG_FLOAT, pos_y, &pos[1])
LOG_ADD(LOG_FLOAT, pos_z, &pos[2])
LOG_GROUP_STOP(PD_ADC)

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* NOTE

	...

*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



// DEVELOPMENT



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// INCLUDE

#include "main.h"

#include <stdio.h>
#include <math.h>
#include <limits.h>

#include "cmsis_os.h"
#include "arm_math.h"
#include "arm_nnfunctions.h"

#include "stm32l4xx_hal_pwr.h"

#include "ecg_data.h"
#include "model_parameters.h"

#include "datalog_application.h"		// REMOVE ME
#include "TargetFeatures.h"				// REMOVE ME

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GENERAL DEFINE

	#define ENABLE_BLE
//	#define ENABLE_USB
//	#define ENABLE_LED


#define DATAQUEUE_SIZE  				((uint32_t) 50)

#define LED_ON 							BSP_LED_On(LED1)
#define LED_OFF 						BSP_LED_Off(LED1)
#define LED_TGG 						BSP_LED_Toggle(LED1)

#define timer_start()    				*((volatile uint32_t*)0xE0001000) = 0x40000001  						// Enable CYCCNT register
#define timer_stop()   					*((volatile uint32_t*)0xE0001000) = 0x40000000  						// Disable CYCCNT register
#define timer_get()   					*((volatile uint32_t*)0xE0001004)               						// Get value from CYCCNT register

// HAL_GetTick();
// osKernelSysTick();

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////






//void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
//{
////	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);
////	STLBLE_PRINTF("\r\nOVERFLOW");
//	while(1);
//}



typedef enum
{

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// SENSORS

	 SENSOR_0 = 0						// ECG sensor
//	 ,SENSOR_1
//	 ,SENSOR_2
//	 ,NEW_SENSOR_N

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// VARIOUS

	,THREAD_BLE							// BLE								Task name = 	bleThread
	,THREAD_MASTER						// Master Thread					Task name = 	MasterThread
	,MESSAGE_MASTER						// Edit FIFO
	,MESSAGE_SENSOR						// ...
	,MESSAGE_DELAYED					// ...
	,SEMAPHORE_BLE						// Timer semaphore
	,SEMAPHORE_MUTEX					// Semaphore mutex

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// SENSOR_0

	,THREAD_GET_SENSOR_0				// Get ECG data						Task name = 	get_thread_0
	,THREAD_proc1_SENSOR_0				// ECG peak detection								proc1_thread_0
	,THREAD_proc2_SENSOR_0				// ECG data classification							proc2_thread_0
	,THREAD_THRSH_SENSOR_0				//													thrsh_thread_1
	,THREAD_SEND_SENSOR_0				// Send ECG data									send_thread_0

	,MESSAGE_proc1_SENSOR_0				// Peak FIFO
	,MESSAGE_proc2_SENSOR_0				// CNN FIFO
	,MESSAGE_THRSH_SENSOR_0				// Threshold FIFO
	,MESSAGE_SEND_SENSOR_0				// Send FIFO

	,SEMAPHORE_SENSOR_0					// Get data semaphore

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	// SENSOR_1
//
//	,THREAD_GET_SENSOR_1				// Get temperature data				Task name = 	get_thread_1
////	,THREAD_proc1_SENSOR_1				// ...												proc1_thread_1
////	,THREAD_proc2_SENSOR_1				// ...												proc2_thread_1
//	,THREAD_THRSH_SENSOR_1				// 													thrsh_thread_1
//	,THREAD_SEND_SENSOR_1				// Send temperature data							send_thread_1
//
////	,MESSAGE_proc1_SENSOR_1				// ...
////	,MESSAGE_proc2_SENSOR_1				// ...
//	,MESSAGE_THRSH_SENSOR_1				// Threshold FIFO
//	,MESSAGE_SEND_SENSOR_1				// Send FIFO
//
//	,SEMAPHORE_SENSOR_1					// Get data semaphore
//
//	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	// SENSOR_2
//
//	,THREAD_GET_SENSOR_2				// Get pressure data				Task name = 	get_thread_2
////	,THREAD_proc1_SENSOR_2				// ...												proc1_thread_2
////	,THREAD_proc2_SENSOR_2				// ...												proc2_thread_2
//	,THREAD_THRSH_SENSOR_2				// 													thrsh_thread_2
//	,THREAD_SEND_SENSOR_2				// Send pressure data								send_thread_2
//
////	,MESSAGE_proc1_SENSOR_2				// ...
////	,MESSAGE_proc2_SENSOR_2				// ...
//	,MESSAGE_THRSH_SENSOR_2				// Threshold FIFO
//	,MESSAGE_SEND_SENSOR_2				// Send FIFO
//
//	,SEMAPHORE_SENSOR_2					// Get data semaphore
//
//	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

} systemTypeDef;

typedef enum
{
	  TASK_SETUP = 0
	 ,SAMP_PERIOD_SETUP
	 ,SEND_PERIOD_SETUP
	 ,PERIOD_CHECK
} masterCommandTypeDef;

typedef enum
{
	 READ_LAST_VALUE = 0
	,READ_FROM_FIFO
} messageTypeTypeDef;

typedef enum
{
	 DO_NOT_SEND = 0
	,SEND
} thresholdTypeTypeDef;

typedef enum							// To be removed
{
	 ECG	 = 20						// ECG			   data type
	,DATA_1D = 7						// One-dimensional data type
	,DATA_3D = 10						// Tri-dimensional data type

} sensorDataTypeTypeDef;

typedef enum
{
	 INT8	 = 1
	,INT16	 = 2
	,INT32	 = 4
	,FLOAT

} DataTypeTypeDef;

typedef enum
{
	 RUN = 0
	,LPRUN
	,SLEEP_WFI
	,SLEEP_WFE
	,LPSLEEP_WFI
	,LPSLEEP_WFE
	,STOP0_WFI
	,STOP0_WFE
	,STOP1_WFI
	,STOP1_WFE
	,STOP2_WFI
	,STOP2_WFE
	,STANDBY
	,SHUTDOWN
} powerModeTypeTypeDef;

typedef struct
{
	int 				message_id;
	int 				sensor_id;
	uint16_t 			charHandle;

	uint32_t 			ms_counter;

	void 				*data;
	int					data_len;
} SensorsData;

struct sensorSetupStruct
{
	char 				name[20];
	char 				code[20];
	int				 	data_type;
	int 				data_len;
	int					sample_packing;
	int					sensor_type;
	uint16_t 			charHandle;

	int 				en_threads[5];

	osThreadId 			get_threadId;
	osThreadId 			proc1_threadId;
	osThreadId 			proc2_threadId;
	osThreadId 			thrsh_threadId;
	osThreadId 			send_threadId;

	osMessageQId 		proc1_queueId;
	osMessageQId 		proc2_queueId;
	osMessageQId 		thrsh_queueId;
	osMessageQId 		send_queueId;

	osMessageQId 		out_get_queueId;
	osMessageQId 		out_proc1_queueId;
	osMessageQId 		out_proc2_queueId;
	osMessageQId 		out_thrsh_queueId;
	osMessageQId 		out_send_queueId;

	osSemaphoreId 		get_semId;

	osTimerId 			samp_timerId;
	int					samp_period_ms;
	void 				(*samp_timer_Callback) (void const *arg);

	osTimerId 			send_timerId;
	int					send_period_ms;
	void 				(*send_timer_Callback) (void const *arg);

	SensorsData 		*last_value;

	int			 		(*data_threshold) (SensorsData *data);

	void 				(*get_data) (SensorsData *data);
//	void 				(*init) (void);
//	void 				(*deInit) (void);
//	void 				(*enable) (void);
//	void 				(*disable) (void);
};

typedef struct
{
	unsigned int 		msDelay;
	osTimerId			timer_id;
	int 				sensor_id;
	osMessageQId 		receiver;
	SensorsData 		*message;
} DelayedSensorsDataMessage;



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MASTER

//	#define		TESTCODE

#define 		INIT_MASTER_TIMER_PERIOD	1000

static void 	master_thread(				void const *argument);

osThreadId									master_threadId;
#ifndef TESTCODE
osThreadDef(								THREAD_MASTER, master_thread, osPriorityAboveNormal, 0, configMINIMAL_STACK_SIZE * 2);
#else
osThreadDef(								THREAD_MASTER, master_thread, osPriorityAboveNormal, 0, configMINIMAL_STACK_SIZE * 4);
#endif

osMessageQId 								master_queueId;
osMessageQDef(								MESSAGE_MASTER, DATAQUEUE_SIZE, int);

osPoolId 									thread_poolId;
osPoolDef(									thread_pool, DATAQUEUE_SIZE, int);

void 			master_timer_Callback(		void const *arg);
void 			master_timer_Start(			void);
void 			master_timer_Stop(			void);

osTimerId 									master_timerId;
osTimerDef(									master_timer, master_timer_Callback);

uint32_t									timer_period_master;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// BLE

#define INIT_BLE_TIMER_PERIOD				500

static void 	ble_thread(					void const *argument);
static void 	InitBlueNRGStack(			void);

osThreadId									ble_threadId;
osThreadDef(								THREAD_BLE, ble_thread, osPriorityNormal, 	0, 	configMINIMAL_STACK_SIZE * 4);

osSemaphoreId 								ble_semId;
osSemaphoreDef(								SEMAPHORE_BLE);

void 			ble_timer_Callback(			void const *arg);
void 			ble_timer_Start(			void);
void 			ble_timer_Stop(				void);

osTimerId 									ble_timerId;
osTimerDef(									ble_timer, ble_timer_Callback);

uint32_t									timer_period_ble;

void			delayedMessage_Callback(	void const *arg);



// CUSTOM PARAMETERS & FUNCTIONS:



#include "sensor_service.h"
#include "bluenrg_utils.h"

static void 	GetBatteryInfoData(			void);

uint32_t 									StartTime;
uint32_t 									EndTime;
uint32_t									CycleTime;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ADC

#define CS_PIN 			GPIO_PIN_12 		// CS pin
#define CS_PER 			GPIOG 				// CS peripheral

#define START_PIN		GPIO_PIN_2	 		// START pin
#define START_PER		GPIOC 				// START peripheral

#define RESET_PIN		GPIO_PIN_0 			// RESET pin
#define RESET_PER		GPIOC 				// RESET peripheral

#define DRDY_PIN		GPIO_PIN_1 			// DRDY pin
#define DRDY_PER 		GPIOC 				// DRDY peripheral

//ADS1298
#define ADS_CMD_WAKEUP		0x02			// Wake-up from standby mode
#define ADS_CMD_STANDBY		0x04			// Enter standby mode
#define ADS_CMD_RESET		0x06			// Reset the device
#define ADS_CMD_START		0x08			// Start/restart (synchronize) conversions
#define ADS_CMD_STOP		0x0A			// Stop the conversion
#define ADS_CMD_OFFSETCAL	0x1A
#define ADS_CMD_RDATAC		0x10			// Enable Read Data Continuous mode
#define ADS_CMD_SDATAC		0x11			// Stop reading continuously
#define ADS_CMD_RDATA		0x12			// Read data by command; supports multiple read back
#define ADS_CMD_RREG		0x20			// 0x001r rrrr - 0x000n nnnn: Read n nnnn registers starting at address r rrrr
#define ADS_CMD_WREG		0x40			// 0x010r rrrr - 0x000n nnnn:Write n nnnn registers starting at address r rrrr

#define FMOD_1024			0x00			// 125sps
#define FMOD_512			0x01			// 250sps
#define FMOD_256			0x02			// 500sps (default)
#define FMOD_128			0x03			// 1ksps
#define FMOD_64				0x04			// 2ksps
#define FMOD_32				0x05			// 4ksps
#define FMOD_16				0x06			// 8ksps
#define F_125_SPS			FMOD_1024		// 125sps
#define F_250_SPS			FMOD_512		// 250sps
#define F_500_SPS			FMOD_256		// 500sps (default)
#define F_1000_SPS			FMOD_128		// 1ksps
#define F_2000_SPS			FMOD_64			// 2ksps
#define F_4000_SPS			FMOD_32			// 4ksps
#define F_8000_SPS			FMOD_16			// 8ksps

#define ADS_REG_ID			0x0
#define ADS_REG_CONFIG1		0x1
#define ADS_REG_CONFIG2 	0x2
#define ADS_REG_CONFIG3		0x3
#define ADS_REG_LOFF		0x4
#define ADS_REG_CH1SET		0x5
#define ADS_REG_CH2SET		0x6
#define ADS_REG_CH3SET		0x7
#define ADS_REG_CH4SET		0x8
#define ADS_REG_CH5SET		0x9
#define ADS_REG_CH6SET		0xA
#define ADS_REG_CH7SET		0xB
#define ADS_REG_CH8SET		0xC
#define ADS_REG_RLDSENSP 	0xD
#define ADS_REG_RLDSENSN	0xE
#define ADS_REG_LOFFSENSP	0xF
#define ADS_REG_LOFFSENSN	0x10
#define ADS_REG_LOFF_FLIP	0x11
#define ADS_REG_LOFF_STATP	0x12
#define ADS_REG_LOFF_STATN	0x13
#define ADS_REG_GPIO		0x14
#define ADS_REG_PACE		0x15
#define ADS_REG_RESP		0x16
#define ADS_REG_CONFIG4		0x17
#define ADS_REG_WCT1		0x18
#define ADS_REG_WCT2		0x19
#define ADS_REG_MAXADD		0x19

#define ADS_VAL_REF_ON		0xA0			// Enable internal reference buffer
#define ADS_VAL_VREF_4V		0x90			// Set internal reference to 4V (with AVDD = 5V only)
#define ADS_VAL_CLK_OUT		0x88			// Enables clock output on the clk pin
#define ADS_VAL_CLK_EN      0x08		    // if clksel pin is 1, CLK_EN = 0 3state, CLK_EN = 1 output clock
#define ADS_VAL_INT_TEST	0x82			// Test signals are generated internally, amplitude: +/�(VREFP � VREFN)/2420
#define ADS_VAL_TEST_FREQ_1	0x81			// Test signal frequency at 1Hz

#define ADS_PGA_GAIN_6		0x00			// set PGA gain to 6
#define ADS_PGA_GAIN_1		0x10			// set PGA gain to 1
#define ADS_PGA_GAIN_2		0x20			// set PGA gain to 2
#define ADS_PGA_GAIN_3		0x30			// set PGA gain to 3
#define ADS_PGA_GAIN_4		0x40			// set PGA gain to 4
#define ADS_PGA_GAIN_8		0x50			// set PGA gain to 8
#define ADS_PGA_GAIN_12		0x60			// set PGA gain to 12

#define ADS_MUX_ELECTRODE	0x00			// Normal electrode input
#define ADS_MUX_SHORTED		0x01			// Input shorted for noise or offset measurement
#define ADS_MUX_RLD_MEASURE	0x02			// Used for RLD measurements
#define ADS_MUX_MVDD		0x03			// Supply voltage measurement
#define ADS_MUX_TEMPERATURE	0x04			// Temperature sensor
#define ADS_MUX_TEST		0x05			// Test signal
#define ADS_MUX_RLD_DRP		0x06			// Positive input is the driver (RLD routing to input channel)
#define ADS_MUX_RLD_DRN		0x07			// Negative input is the driver
#define ADS_MUX_RLD_DRPN	0x08 			// Both inputs are connected to RLDIN
#define ADS_MUX_CH3_TO_CH	0x09     		// Route channel 3 to the selected channel input

#define ADS_CHPD			0x80



void 			power_up_ADS(				);
void 			start_ADS(					void);
int 			send_sw_command_ADS(		unsigned char command);
int 			write_single_register_ADS(	unsigned char address, unsigned char reg_data);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// VARIOUS



#define NUMBER_OF_SENSORS					1
#define INIT_TASK_STATE						0b00


void 			init_all_task(				void);
void 			*dataMalloc(				int type, int size);
void 			dataCopy(					int type, int *cnt, SensorsData *src, SensorsData *dst);

void 			Error_Handler(				void);

static void 	gp_read_from_sensor(		int sensorId);
static void 	gp_data_threshold(			int sensorId);
static void 	gp_send_to_gateway(			int sensorId);

//extern void 	vPortSetupTimerInterrupt( 	void);



osPoolId 									message_poolId;
osPoolDef(									MESSAGE_SENSOR, 		DATAQUEUE_SIZE, 	SensorsData);

osSemaphoreId 								mutex_semId;
osSemaphoreDef(								SEMAPHORE_MUTEX);

uint32_t									soc;
int32_t										current= 0;
uint32_t									voltage;

uint32_t 									timer_0;
uint32_t 									timer_1;
uint32_t 									timer_2;
uint32_t 									timer_3;
uint32_t 									timer_4;
uint32_t 									timer_5;
uint32_t 									timer_6;
uint32_t 									timer_7;
uint32_t 									timer_8;
uint32_t 									timer_9;

uint32_t 									timer_workload;
int 										workLoad = 0;
#define			MAX_WORKLOAD				255



extern uint32_t 							ulTimerCountsForOneTick;
extern uint8_t 								set_connectable;
extern int 									connected;
static volatile uint32_t 					HCI_ProcessEvent = 		0;

uint32_t									taskState = 			INIT_TASK_STATE;

uint8_t 									BufferToWrite[256];
int32_t 									BytesToWrite;
uint8_t 									bdaddr[6];
uint32_t 									ConnectionBleStatus = 	0;
uint32_t 									exec;
USBD_HandleTypeDef 							USBD_Device;

uint32_t									samp_period_ms_index;
uint32_t									samp_period_ms_value;

uint32_t									send_period_ms_index;
uint32_t									send_period_ms_value;

struct sensorSetupStruct 					sensorSetup[NUMBER_OF_SENSORS];



osMessageQId 								delayed_queueId;
osMessageQDef(								MESSAGE_DELAYED, DATAQUEUE_SIZE, DelayedSensorsDataMessage);



// CUSTOM PARAMETERS & FUNCTIONS:



void			powerControl(				int mode, int frequency);
static void 	GetBatteryInfoData(			void);
void			delayedOsMessagePut(		unsigned int msDelay, int sensor_id, osMessageQId receiver, SensorsData *message);


osTimerDef(delayedTimer, delayedMessage_Callback);
void *STC3115_handle = 						NULL;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SPI

	SPI_HandleTypeDef hspi3;
	static void MX_GPIO_Init(void);
	static void MX_SPI3_Init(void);
	void SystemClock_Config_SPI(void);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SENSOR_0

	static void 	get_thread_0(				void const *argument);
	static void 	proc1_thread_0(				void const *argument);
	static void 	proc2_thread_0(				void const *argument);
	static void 	thrsh_thread_0(				void const *argument);
	static void 	send_thread_0(				void const *argument);

	osMessageQDef(								MESSAGE_proc1_SENSOR_0, DATAQUEUE_SIZE, 	int);
	osMessageQDef(								MESSAGE_proc2_SENSOR_0, DATAQUEUE_SIZE, 	int);
	osMessageQDef(								MESSAGE_THRSH_SENSOR_0, DATAQUEUE_SIZE, 	int);
	osMessageQDef(								MESSAGE_SEND_SENSOR_0, 	DATAQUEUE_SIZE, 	int);

	osSemaphoreDef(								SEMAPHORE_SENSOR_0);

	osThreadDef(								THREAD_GET_SENSOR_0,	get_thread_0,		osPriorityAboveNormal,	0,	configMINIMAL_STACK_SIZE * 1); // 1
	osThreadDef(								THREAD_proc1_SENSOR_0,	proc1_thread_0,		osPriorityAboveNormal,	0,	configMINIMAL_STACK_SIZE * 1); // 1
	osThreadDef(								THREAD_proc2_SENSOR_0,	proc2_thread_0,		osPriorityNormal,		0,	configMINIMAL_STACK_SIZE * 6); // 2
	osThreadDef(								THREAD_THRSH_SENSOR_0,	thrsh_thread_0,		osPriorityAboveNormal,	0,	configMINIMAL_STACK_SIZE * 1); // 1
	osThreadDef(								THREAD_SEND_SENSOR_0,	send_thread_0,		osPriorityAboveNormal,	0,	configMINIMAL_STACK_SIZE * 4); // 4

	void 			samp_timer_0_Callback(		void const *arg);
	void 			send_timer_0_Callback(		void const *arg);



// CUSTOM PARAMETERS & FUNCTIONS:



//	#include "ecg_data.h"
//	#include "ds_cnn.h"

	void 			ecg_read_from_sensor(		SensorsData *data);
	int			 	ecg_data_threshold(			SensorsData *data);

	static void 	ecg_peak_detector(			int sensorId);
	static void 	ecg_cnn_classifier(			int sensorId);

//	static void 	enable_1(					void);
//	static void 	disable_1(					void);
//	static void 	setOdr_1(					void);

	#define ECG_SAMP_PERIOD_VALUE				1000
	#define HB_SEND_PERIOD_VALUE				1000






	int16_t 									ecg_data[ECG_SAMPLE] = 	ECG_DATA;
//	int16_t 									ecg_data[100] = 		{1};
//	int											ecg_delta = 			ECG_DELTA;
	int											ecg_debounce = 			ECG_RATE / 10;
	int											ecg_rate = 				60000 / ECG_SAMP_PERIOD_VALUE;
	int32_t										ecg_count = 			0;
	int32_t										ecg_prev_count = 		0;
	int32_t										ecg_count_peak = 		-ecg_debounce;
	int16_t										ecg_prev_data = 		ecg_data[0];
//	int16_t										ecg_heartbeat = 		0;
	uint32_t									ecg_state = 			0;

	int											ttemp = 666;






	#define 	LOG_PRINT









	#define CNN_20_100
//	#define CNN_18_100
//	#define CNN_16_100
//	#define CNN_12_100
//	#define CNN_8_100
//	#define CNN_2_5
//	#define CNN_4_5
//	#define CNN_20_50
//	#define CNN_20_4_100
//	#define CNN_4_4_100
//	#define CNN_20_20_25



#ifdef		CNN_20_100

#define		CONV1_INDIM 		198
#define		CONV1_OUTDIM 		192
#define		CONV1_IF 			1
#define		CONV1_OF 			20
#define		CONV1_KDIM 			7
#define		CONV1_STRIDE 		1
#define		CONV1_PADDING 		0
#define		CONV1_BIAS_LSHIFT 	0
#define		CONV1_OUT_RSHIFT	9
#define 	CONV1_OUT_SCALE		469.7					// TO CALCULATE

#define 	RELU1_INDIM			3840

#define 	MAXPOOL1_INDIM		192
#define 	MAXPOOL1_OUTDIM		96
#define 	MAXPOOL1_IF			20
#define 	MAXPOOL1_KDIM		2
#define 	MAXPOOL1_STRIDE		2
#define 	MAXPOOL1_PADDING	0

#define		CONV2_INDIM 		96
#define		CONV2_OUTDIM 		90
#define		CONV2_IF 			20
#define		CONV2_OF 			20
#define		CONV2_KDIM 			7
#define		CONV2_STRIDE 		1
#define		CONV2_PADDING 		0
#define		CONV2_BIAS_LSHIFT 	0
#define		CONV2_OUT_RSHIFT 	8
#define 	CONV2_OUT_SCALE		289.72					// TO CALCULATE

#define 	RELU2_INDIM			1800

#define 	MAXPOOL2_INDIM		90
#define 	MAXPOOL2_OUTDIM		45
#define 	MAXPOOL2_IF			20
#define 	MAXPOOL2_KDIM		2
#define 	MAXPOOL2_STRIDE		2
#define 	MAXPOOL2_PADDING	0

#define 	FC1_INDIM			900
#define 	FC1_OUTDIM			100
#define 	FC1_BIAS_LSHIFT		0
#define 	FC1_OUT_RSHIFT		8
#define 	FC1_OUT_SCALE		260.5					// TO CALCULATE

#define 	RELU3_INDIM			100

#define 	FC2_INDIM			100
#define 	FC2_OUTDIM			5
#define 	FC2_BIAS_LSHIFT		0
#define 	FC2_OUT_RSHIFT		7
#define 	FC2_OUT_SCALE		132.5					// TO CALCULATE

#define		COLBUFFER_DIM		FC1_INDIM				// maximum between:    FULLY-> FCN_INDIM    CONV-> 2 * CONVN_IF * CONVN_KDIM
#define		CNNBUFFER			CONV1_OF * CONV1_OUTDIM // maximum between:    FULLY-> FCN_INDIM    CONV-> CONVN_IF * CONVN_INDIM       CONV-> CONVN_OF * CONVN_OUTDIM

	#define	DUMMYINPUT
	#define	DUMMYWTBIAS

	#define	OUTPUT_SHIFT

//	#define	CONV2_BASIC
//	#define	FULLY1_BASIC
//	#define	FULLY2_BASIC

	#define	CONV1_RAM
	#define	CONV2_RAM
//	#define	FC1_RAM
	#define	FC2_RAM

#endif



#ifdef		CNN_18_100

#define		CONV1_INDIM 		198
#define		CONV1_OUTDIM 		192
#define		CONV1_IF 			1
#define		CONV1_OF 			18
#define		CONV1_KDIM 			7
#define		CONV1_STRIDE 		1
#define		CONV1_PADDING 		0
#define		CONV1_BIAS_LSHIFT 	0
#define		CONV1_OUT_RSHIFT	9
#define 	CONV1_OUT_SCALE		469.7

#define 	RELU1_INDIM			3456

#define 	MAXPOOL1_INDIM		192
#define 	MAXPOOL1_OUTDIM		96
#define 	MAXPOOL1_IF			18
#define 	MAXPOOL1_KDIM		2
#define 	MAXPOOL1_STRIDE		2
#define 	MAXPOOL1_PADDING	0

#define		CONV2_INDIM 		96
#define		CONV2_OUTDIM 		90
#define		CONV2_IF 			18
#define		CONV2_OF 			18
#define		CONV2_KDIM 			7
#define		CONV2_STRIDE 		1
#define		CONV2_PADDING 		0
#define		CONV2_BIAS_LSHIFT 	0
#define		CONV2_OUT_RSHIFT 	8
#define 	CONV2_OUT_SCALE		289.72

#define 	RELU2_INDIM			1620

#define 	MAXPOOL2_INDIM		90
#define 	MAXPOOL2_OUTDIM		45
#define 	MAXPOOL2_IF			18
#define 	MAXPOOL2_KDIM		2
#define 	MAXPOOL2_STRIDE		2
#define 	MAXPOOL2_PADDING	0

#define 	FC1_INDIM			810
#define 	FC1_OUTDIM			100
#define 	FC1_BIAS_LSHIFT		0
#define 	FC1_OUT_RSHIFT		8
#define 	FC1_OUT_SCALE		260.5

#define 	RELU3_INDIM			100

#define 	FC2_INDIM			100
#define 	FC2_OUTDIM			5
#define 	FC2_BIAS_LSHIFT		0
#define 	FC2_OUT_RSHIFT		7
#define 	FC2_OUT_SCALE		132.5

#define		COLBUFFER_DIM		FC1_INDIM				// maximum between:    FULLY-> FCN_INDIM    CONV-> 2 * CONVN_IF * CONVN_KDIM
#define		CNNBUFFER			CONV1_OF * CONV1_OUTDIM // maximum between:    FULLY-> FCN_INDIM    CONV-> CONVN_IF * CONVN_INDIM       CONV-> CONVN_OF * CONVN_OUTDIM

//	#define	DUMMYINPUT
//	#define	DUMMYWTBIAS

//	#define	OUTPUT_SHIFT
	#define	OUTPUT_DIV
//	#define OUTPUT_32BIT

	#define	CONV1_BASIC
	#define	CONV2_BASIC
	#define	FULLY1_BASIC
	#define	FULLY2_BASIC

	#define	CONV1_RAM
	#define	CONV2_RAM
//	#define	FC1_RAM
	#define	FC2_RAM

#endif



#ifdef		CNN_16_100

#define		CONV1_INDIM 		198
#define		CONV1_OUTDIM 		192
#define		CONV1_IF 			1
#define		CONV1_OF 			16
#define		CONV1_KDIM 			7
#define		CONV1_STRIDE 		1
#define		CONV1_PADDING 		0
#define		CONV1_BIAS_LSHIFT 	0
#define		CONV1_OUT_RSHIFT	9
#define 	CONV1_OUT_SCALE		469.7					// TO CALCULATE

#define 	RELU1_INDIM			3072

#define 	MAXPOOL1_INDIM		192
#define 	MAXPOOL1_OUTDIM		96
#define 	MAXPOOL1_IF			16
#define 	MAXPOOL1_KDIM		2
#define 	MAXPOOL1_STRIDE		2
#define 	MAXPOOL1_PADDING	0

#define		CONV2_INDIM 		96
#define		CONV2_OUTDIM 		90
#define		CONV2_IF 			16
#define		CONV2_OF 			16
#define		CONV2_KDIM 			7
#define		CONV2_STRIDE 		1
#define		CONV2_PADDING 		0
#define		CONV2_BIAS_LSHIFT 	0
#define		CONV2_OUT_RSHIFT 	8
#define 	CONV2_OUT_SCALE		289.72					// TO CALCULATE

#define 	RELU2_INDIM			1440

#define 	MAXPOOL2_INDIM		90
#define 	MAXPOOL2_OUTDIM		45
#define 	MAXPOOL2_IF			16
#define 	MAXPOOL2_KDIM		2
#define 	MAXPOOL2_STRIDE		2
#define 	MAXPOOL2_PADDING	0

#define 	FC1_INDIM			720
#define 	FC1_OUTDIM			100
#define 	FC1_BIAS_LSHIFT		0
#define 	FC1_OUT_RSHIFT		8
#define 	FC1_OUT_SCALE		260.5					// TO CALCULATE

#define 	RELU3_INDIM			100

#define 	FC2_INDIM			100
#define 	FC2_OUTDIM			5
#define 	FC2_BIAS_LSHIFT		0
#define 	FC2_OUT_RSHIFT		7
#define 	FC2_OUT_SCALE		132.5					// TO CALCULATE

#define		COLBUFFER_DIM		FC1_INDIM				// maximum between:    FULLY-> FCN_INDIM    CONV-> 2 * CONVN_IF * CONVN_KDIM
#define		CNNBUFFER			CONV1_OF * CONV1_OUTDIM // maximum between:    FULLY-> FCN_INDIM    CONV-> CONVN_IF * CONVN_INDIM       CONV-> CONVN_OF * CONVN_OUTDIM

	#define	DUMMYINPUT
	#define	DUMMYWTBIAS

//	#define	OUTPUT_SHIFT
	#define	OUTPUT_DIV
//	#define OUTPUT_32BIT

	#define	CONV1_BASIC
//	#define	CONV2_BASIC
//	#define	FULLY1_BASIC
//	#define	FULLY2_BASIC

	#define	CONV1_RAM
	#define	CONV2_RAM
//	#define	FC1_RAM
	#define	FC2_RAM

#endif



#ifdef		CNN_12_100

#define		CONV1_INDIM 		198
#define		CONV1_OUTDIM 		192
#define		CONV1_IF 			1
#define		CONV1_OF 			12
#define		CONV1_KDIM 			7
#define		CONV1_STRIDE 		1
#define		CONV1_PADDING 		0
#define		CONV1_BIAS_LSHIFT 	0
#define		CONV1_OUT_RSHIFT	9
#define 	CONV1_OUT_SCALE		469.7					// TO CALCULATE

#define 	RELU1_INDIM			2304

#define 	MAXPOOL1_INDIM		192
#define 	MAXPOOL1_OUTDIM		96
#define 	MAXPOOL1_IF			12
#define 	MAXPOOL1_KDIM		2
#define 	MAXPOOL1_STRIDE		2
#define 	MAXPOOL1_PADDING	0

#define		CONV2_INDIM 		96
#define		CONV2_OUTDIM 		90
#define		CONV2_IF 			12
#define		CONV2_OF 			12
#define		CONV2_KDIM 			7
#define		CONV2_STRIDE 		1
#define		CONV2_PADDING 		0
#define		CONV2_BIAS_LSHIFT 	0
#define		CONV2_OUT_RSHIFT 	8
#define 	CONV2_OUT_SCALE		289.72					// TO CALCULATE

#define 	RELU2_INDIM			1080

#define 	MAXPOOL2_INDIM		90
#define 	MAXPOOL2_OUTDIM		45
#define 	MAXPOOL2_IF			12
#define 	MAXPOOL2_KDIM		2
#define 	MAXPOOL2_STRIDE		2
#define 	MAXPOOL2_PADDING	0

#define 	FC1_INDIM			540
#define 	FC1_OUTDIM			100
#define 	FC1_BIAS_LSHIFT		0
#define 	FC1_OUT_RSHIFT		8
#define 	FC1_OUT_SCALE		260.5					// TO CALCULATE

#define 	RELU3_INDIM			100

#define 	FC2_INDIM			100
#define 	FC2_OUTDIM			5
#define 	FC2_BIAS_LSHIFT		0
#define 	FC2_OUT_RSHIFT		7
#define 	FC2_OUT_SCALE		132.5					// TO CALCULATE

#define		COLBUFFER_DIM		FC1_INDIM				// maximum between:    FULLY-> FCN_INDIM    CONV-> 2 * CONVN_IF * CONVN_KDIM
#define		CNNBUFFER			CONV1_OF * CONV1_OUTDIM // maximum between:    FULLY-> FCN_INDIM    CONV-> CONVN_IF * CONVN_INDIM       CONV-> CONVN_OF * CONVN_OUTDIM

	#define	DUMMYINPUT
	#define	DUMMYWTBIAS

//	#define	OUTPUT_SHIFT
	#define	OUTPUT_DIV
//	#define OUTPUT_32BIT

	#define	CONV1_BASIC
//	#define	CONV2_BASIC
//	#define	FULLY1_BASIC
//	#define	FULLY2_BASIC

	#define	CONV1_RAM
	#define	CONV2_RAM
//	#define	FC1_RAM
	#define	FC2_RAM

#endif



#ifdef		CNN_8_100

#define		CONV1_INDIM 		198
#define		CONV1_OUTDIM 		192
#define		CONV1_IF 			1
#define		CONV1_OF 			8
#define		CONV1_KDIM 			7
#define		CONV1_STRIDE 		1
#define		CONV1_PADDING 		0
#define		CONV1_BIAS_LSHIFT 	0
#define		CONV1_OUT_RSHIFT	9
#define 	CONV1_OUT_SCALE		469.7					// TO CALCULATE

#define 	RELU1_INDIM			1536

#define 	MAXPOOL1_INDIM		192
#define 	MAXPOOL1_OUTDIM		96
#define 	MAXPOOL1_IF			8
#define 	MAXPOOL1_KDIM		2
#define 	MAXPOOL1_STRIDE		2
#define 	MAXPOOL1_PADDING	0

#define		CONV2_INDIM 		96
#define		CONV2_OUTDIM 		90
#define		CONV2_IF 			8
#define		CONV2_OF 			8
#define		CONV2_KDIM 			7
#define		CONV2_STRIDE 		1
#define		CONV2_PADDING 		0
#define		CONV2_BIAS_LSHIFT 	0
#define		CONV2_OUT_RSHIFT 	8
#define 	CONV2_OUT_SCALE		289.72					// TO CALCULATE

#define 	RELU2_INDIM			720

#define 	MAXPOOL2_INDIM		90
#define 	MAXPOOL2_OUTDIM		45
#define 	MAXPOOL2_IF			8
#define 	MAXPOOL2_KDIM		2
#define 	MAXPOOL2_STRIDE		2
#define 	MAXPOOL2_PADDING	0

#define 	FC1_INDIM			360
#define 	FC1_OUTDIM			100
#define 	FC1_BIAS_LSHIFT		0
#define 	FC1_OUT_RSHIFT		8
#define 	FC1_OUT_SCALE		260.5					// TO CALCULATE

#define 	RELU3_INDIM			100

#define 	FC2_INDIM			100
#define 	FC2_OUTDIM			5
#define 	FC2_BIAS_LSHIFT		0
#define 	FC2_OUT_RSHIFT		7
#define 	FC2_OUT_SCALE		132.5					// TO CALCULATE

#define		COLBUFFER_DIM		FC1_INDIM				// maximum between:    FULLY-> FCN_INDIM    CONV-> 2 * CONVN_IF * CONVN_KDIM
#define		CNNBUFFER			CONV1_OF * CONV1_OUTDIM // maximum between:    FULLY-> FCN_INDIM    CONV-> CONVN_IF * CONVN_INDIM       CONV-> CONVN_OF * CONVN_OUTDIM

	#define	DUMMYINPUT
	#define	DUMMYWTBIAS

//	#define	OUTPUT_SHIFT
	#define	OUTPUT_DIV
//	#define OUTPUT_32BIT

	#define	CONV1_BASIC
//	#define	CONV2_BASIC
//	#define	FULLY1_BASIC
//	#define	FULLY2_BASIC

	#define	CONV1_RAM
	#define	CONV2_RAM
//	#define	FC1_RAM
	#define	FC2_RAM

#endif



#ifdef		CNN_4_5

#define		CONV1_INDIM 		198
#define		CONV1_OUTDIM 		192
#define		CONV1_IF 			1
#define		CONV1_OF 			4
#define		CONV1_KDIM 			7
#define		CONV1_STRIDE 		1
#define		CONV1_PADDING 		0
#define		CONV1_BIAS_LSHIFT 	0
#define		CONV1_OUT_RSHIFT	9
#define 	CONV1_OUT_SCALE		469.7					// TO CALCULATE

#define 	RELU1_INDIM			768

#define 	MAXPOOL1_INDIM		192
#define 	MAXPOOL1_OUTDIM		96
#define 	MAXPOOL1_IF			4
#define 	MAXPOOL1_KDIM		2
#define 	MAXPOOL1_STRIDE		2
#define 	MAXPOOL1_PADDING	0

#define		CONV2_INDIM 		96
#define		CONV2_OUTDIM 		90
#define		CONV2_IF 			4
#define		CONV2_OF 			4
#define		CONV2_KDIM 			7
#define		CONV2_STRIDE 		1
#define		CONV2_PADDING 		0
#define		CONV2_BIAS_LSHIFT 	0
#define		CONV2_OUT_RSHIFT 	8
#define 	CONV2_OUT_SCALE		289.72					// TO CALCULATE

#define 	RELU2_INDIM			360

#define 	MAXPOOL2_INDIM		90
#define 	MAXPOOL2_OUTDIM		45
#define 	MAXPOOL2_IF			4
#define 	MAXPOOL2_KDIM		2
#define 	MAXPOOL2_STRIDE		2
#define 	MAXPOOL2_PADDING	0

#define 	FC1_INDIM			180
#define 	FC1_OUTDIM			5
#define 	FC1_BIAS_LSHIFT		0
#define 	FC1_OUT_RSHIFT		8
#define 	FC1_OUT_SCALE		260.5					// TO CALCULATE

#define 	RELU3_INDIM			5

#define 	FC2_INDIM			5
#define 	FC2_OUTDIM			5
#define 	FC2_BIAS_LSHIFT		0
#define 	FC2_OUT_RSHIFT		7
#define 	FC2_OUT_SCALE		132.5					// TO CALCULATE

#define		COLBUFFER_DIM		FC1_INDIM				// maximum between:    FULLY-> FCN_INDIM    CONV-> 2 * CONVN_IF * CONVN_KDIM
#define		CNNBUFFER			CONV1_OF * CONV1_OUTDIM // maximum between:    FULLY-> FCN_INDIM    CONV-> CONVN_IF * CONVN_INDIM       CONV-> CONVN_OF * CONVN_OUTDIM

	#define	DUMMYINPUT
	#define	DUMMYWTBIAS

//	#define	OUTPUT_SHIFT
//	#define	OUTPUT_DIV
//	#define OUTPUT_32BIT

//	#define	CONV1_BASIC
//	#define	CONV2_BASIC
//	#define	FULLY1_BASIC
//	#define	FULLY2_BASIC

	#define	CONV1_RAM
	#define	CONV2_RAM
	#define	FC1_RAM
	#define	FC2_RAM

#endif



#ifdef		CNN_2_5

#define		CONV1_INDIM 		198
#define		CONV1_OUTDIM 		192
#define		CONV1_IF 			1
#define		CONV1_OF 			2
#define		CONV1_KDIM 			7
#define		CONV1_STRIDE 		1
#define		CONV1_PADDING 		0
#define		CONV1_BIAS_LSHIFT 	0
#define		CONV1_OUT_RSHIFT	9
#define 	CONV1_OUT_SCALE		469.7					// TO CALCULATE

#define 	RELU1_INDIM			768

#define 	MAXPOOL1_INDIM		192
#define 	MAXPOOL1_OUTDIM		96
#define 	MAXPOOL1_IF			2
#define 	MAXPOOL1_KDIM		2
#define 	MAXPOOL1_STRIDE		2
#define 	MAXPOOL1_PADDING	0

#define		CONV2_INDIM 		96
#define		CONV2_OUTDIM 		90
#define		CONV2_IF 			4
#define		CONV2_OF 			4
#define		CONV2_KDIM 			7
#define		CONV2_STRIDE 		1
#define		CONV2_PADDING 		0
#define		CONV2_BIAS_LSHIFT 	0
#define		CONV2_OUT_RSHIFT 	8
#define 	CONV2_OUT_SCALE		289.72					// TO CALCULATE

#define 	RELU2_INDIM			360

#define 	MAXPOOL2_INDIM		90
#define 	MAXPOOL2_OUTDIM		45
#define 	MAXPOOL2_IF			4
#define 	MAXPOOL2_KDIM		2
#define 	MAXPOOL2_STRIDE		2
#define 	MAXPOOL2_PADDING	0

#define 	FC1_INDIM			90
#define 	FC1_OUTDIM			5
#define 	FC1_BIAS_LSHIFT		0
#define 	FC1_OUT_RSHIFT		8
#define 	FC1_OUT_SCALE		260.5					// TO CALCULATE

#define 	RELU3_INDIM			5

#define 	FC2_INDIM			5
#define 	FC2_OUTDIM			5
#define 	FC2_BIAS_LSHIFT		0
#define 	FC2_OUT_RSHIFT		7
#define 	FC2_OUT_SCALE		132.5					// TO CALCULATE

#define		COLBUFFER_DIM		FC1_INDIM				// maximum between:    FULLY-> FCN_INDIM    CONV-> 2 * CONVN_IF * CONVN_KDIM
#define		CNNBUFFER			CONV1_OF * CONV1_OUTDIM // maximum between:    FULLY-> FCN_INDIM    CONV-> CONVN_IF * CONVN_INDIM       CONV-> CONVN_OF * CONVN_OUTDIM

	#define	DUMMYINPUT
	#define	DUMMYWTBIAS

//	#define	OUTPUT_SHIFT
//	#define	OUTPUT_DIV
//	#define OUTPUT_32BIT

//	#define	CONV1_BASIC
//	#define	CONV2_BASIC
//	#define	FULLY1_BASIC
//	#define	FULLY2_BASIC

	#define	CONV1_RAM
	#define	CONV2_RAM
	#define	FC1_RAM
	#define	FC2_RAM

#endif



#ifdef		CNN_20_50

#define		CONV1_INDIM 		198
#define		CONV1_OUTDIM 		192
#define		CONV1_IF 			1
#define		CONV1_OF 			20
#define		CONV1_KDIM 			7
#define		CONV1_STRIDE 		1
#define		CONV1_PADDING 		0
#define		CONV1_BIAS_LSHIFT 	0
#define		CONV1_OUT_RSHIFT	9
#define 	CONV1_OUT_SCALE		469.7					// TO CALCULATE

#define 	RELU1_INDIM			3840

#define 	MAXPOOL1_INDIM		192
#define 	MAXPOOL1_OUTDIM		96
#define 	MAXPOOL1_IF			20
#define 	MAXPOOL1_KDIM		2
#define 	MAXPOOL1_STRIDE		2
#define 	MAXPOOL1_PADDING	0

#define		CONV2_INDIM 		96
#define		CONV2_OUTDIM 		90
#define		CONV2_IF 			20
#define		CONV2_OF 			20
#define		CONV2_KDIM 			7
#define		CONV2_STRIDE 		1
#define		CONV2_PADDING 		0
#define		CONV2_BIAS_LSHIFT 	0
#define		CONV2_OUT_RSHIFT 	8
#define 	CONV2_OUT_SCALE		289.72					// TO CALCULATE

#define 	RELU2_INDIM			1800

#define 	MAXPOOL2_INDIM		90
#define 	MAXPOOL2_OUTDIM		45
#define 	MAXPOOL2_IF			20
#define 	MAXPOOL2_KDIM		2
#define 	MAXPOOL2_STRIDE		2
#define 	MAXPOOL2_PADDING	0

#define 	FC1_INDIM			900
#define 	FC1_OUTDIM			25
#define 	FC1_BIAS_LSHIFT		0
#define 	FC1_OUT_RSHIFT		8
#define 	FC1_OUT_SCALE		260.5					// TO CALCULATE

#define 	RELU3_INDIM			25

#define 	FC2_INDIM			25
#define 	FC2_OUTDIM			5
#define 	FC2_BIAS_LSHIFT		0
#define 	FC2_OUT_RSHIFT		7
#define 	FC2_OUT_SCALE		132.5					// TO CALCULATE

#define		COLBUFFER_DIM		FC1_INDIM				// maximum between:    FULLY-> FCN_INDIM    CONV-> 2 * CONVN_IF * CONVN_KDIM
#define		CNNBUFFER			CONV1_OF * CONV1_OUTDIM // maximum between:    FULLY-> FCN_INDIM    CONV-> CONVN_IF * CONVN_INDIM       CONV-> CONVN_OF * CONVN_OUTDIM

	#define	DUMMYINPUT
	#define	DUMMYWTBIAS

//	#define	OUTPUT_SHIFT

//	#define	CONV2_BASIC
//	#define	FULLY1_BASIC
//	#define	FULLY2_BASIC

	#define	CONV1_RAM
	#define	CONV2_RAM
	#define	FC1_RAM
	#define	FC2_RAM

#endif



#ifdef		CNN_20_4_100

#define		CONV1_INDIM 		198
#define		CONV1_OUTDIM 		192
#define		CONV1_IF 			1
#define		CONV1_OF 			20
#define		CONV1_KDIM 			7
#define		CONV1_STRIDE 		1
#define		CONV1_PADDING 		0
#define		CONV1_BIAS_LSHIFT 	0
#define		CONV1_OUT_RSHIFT	9
#define 	CONV1_OUT_SCALE		469.7					// TO CALCULATE

#define 	RELU1_INDIM			3840

#define 	MAXPOOL1_INDIM		192
#define 	MAXPOOL1_OUTDIM		96
#define 	MAXPOOL1_IF			20
#define 	MAXPOOL1_KDIM		2
#define 	MAXPOOL1_STRIDE		2
#define 	MAXPOOL1_PADDING	0

#define		CONV2_INDIM 		96
#define		CONV2_OUTDIM 		90
#define		CONV2_IF 			20
#define		CONV2_OF 			4
#define		CONV2_KDIM 			7
#define		CONV2_STRIDE 		1
#define		CONV2_PADDING 		0
#define		CONV2_BIAS_LSHIFT 	0
#define		CONV2_OUT_RSHIFT 	8
#define 	CONV2_OUT_SCALE		289.72					// TO CALCULATE

#define 	RELU2_INDIM			360

#define 	MAXPOOL2_INDIM		90
#define 	MAXPOOL2_OUTDIM		45
#define 	MAXPOOL2_IF			4
#define 	MAXPOOL2_KDIM		2
#define 	MAXPOOL2_STRIDE		2
#define 	MAXPOOL2_PADDING	0

#define 	FC1_INDIM			180
#define 	FC1_OUTDIM			25
#define 	FC1_BIAS_LSHIFT		0
#define 	FC1_OUT_RSHIFT		8
#define 	FC1_OUT_SCALE		260.5					// TO CALCULATE

#define 	RELU3_INDIM			25

#define 	FC2_INDIM			25
#define 	FC2_OUTDIM			5
#define 	FC2_BIAS_LSHIFT		0
#define 	FC2_OUT_RSHIFT		7
#define 	FC2_OUT_SCALE		132.5					// TO CALCULATE

#define		COLBUFFER_DIM		2 * CONV1_IF * CONV1_KDIM 	// maximum between:    FULLY-> FCN_INDIM    CONV-> 2 * CONVN_IF * CONVN_KDIM
#define		CNNBUFFER			CONV1_OF * CONV1_OUTDIM		// maximum between:    FULLY-> FCN_INDIM    CONV-> CONVN_IF * CONVN_INDIM       CONV-> CONVN_OF * CONVN_OUTDIM

	#define	DUMMYINPUT
	#define	DUMMYWTBIAS

//	#define	OUTPUT_SHIFT

//	#define	CONV2_BASIC
//	#define	FULLY1_BASIC
//	#define	FULLY2_BASIC

	#define	CONV1_RAM
	#define	CONV2_RAM
	#define	FC1_RAM
	#define	FC2_RAM

#endif



#ifdef		CNN_4_4_100

#define		CONV1_INDIM 		198
#define		CONV1_OUTDIM 		192
#define		CONV1_IF 			1
#define		CONV1_OF 			4
#define		CONV1_KDIM 			7
#define		CONV1_STRIDE 		1
#define		CONV1_PADDING 		0
#define		CONV1_BIAS_LSHIFT 	0
#define		CONV1_OUT_RSHIFT	9
#define 	CONV1_OUT_SCALE		469.7					// TO CALCULATE

#define 	RELU1_INDIM			768

#define 	MAXPOOL1_INDIM		192
#define 	MAXPOOL1_OUTDIM		96
#define 	MAXPOOL1_IF			4
#define 	MAXPOOL1_KDIM		2
#define 	MAXPOOL1_STRIDE		2
#define 	MAXPOOL1_PADDING	0

#define		CONV2_INDIM 		96
#define		CONV2_OUTDIM 		90
#define		CONV2_IF 			4
#define		CONV2_OF 			4
#define		CONV2_KDIM 			7
#define		CONV2_STRIDE 		1
#define		CONV2_PADDING 		0
#define		CONV2_BIAS_LSHIFT 	0
#define		CONV2_OUT_RSHIFT 	8
#define 	CONV2_OUT_SCALE		289.72					// TO CALCULATE

#define 	RELU2_INDIM			360

#define 	MAXPOOL2_INDIM		90
#define 	MAXPOOL2_OUTDIM		45
#define 	MAXPOOL2_IF			4
#define 	MAXPOOL2_KDIM		2
#define 	MAXPOOL2_STRIDE		2
#define 	MAXPOOL2_PADDING	0

#define 	FC1_INDIM			180
#define 	FC1_OUTDIM			25
#define 	FC1_BIAS_LSHIFT		0
#define 	FC1_OUT_RSHIFT		8
#define 	FC1_OUT_SCALE		260.5					// TO CALCULATE

#define 	RELU3_INDIM			25

#define 	FC2_INDIM			25
#define 	FC2_OUTDIM			5
#define 	FC2_BIAS_LSHIFT		0
#define 	FC2_OUT_RSHIFT		7
#define 	FC2_OUT_SCALE		132.5					// TO CALCULATE

#define		COLBUFFER_DIM		FC1_INDIM 	// maximum between:    FULLY-> FCN_INDIM    CONV-> 2 * CONVN_IF * CONVN_KDIM
#define		CNNBUFFER			CONV1_OF * CONV1_OUTDIM		// maximum between:    FULLY-> FCN_INDIM    CONV-> CONVN_IF * CONVN_INDIM       CONV-> CONVN_OF * CONVN_OUTDIM

	#define	DUMMYINPUT
	#define	DUMMYWTBIAS

//	#define	OUTPUT_SHIFT

//	#define	CONV2_BASIC
//	#define	FULLY1_BASIC
//	#define	FULLY2_BASIC

	#define	CONV1_RAM
	#define	CONV2_RAM
	#define	FC1_RAM
	#define	FC2_RAM

#endif



#ifdef		CNN_20_20_25

#define		CONV1_INDIM 		198
#define		CONV1_OUTDIM 		192
#define		CONV1_IF 			1
#define		CONV1_OF 			20
#define		CONV1_KDIM 			7
#define		CONV1_STRIDE 		1
#define		CONV1_PADDING 		0
#define		CONV1_BIAS_LSHIFT 	0
#define		CONV1_OUT_RSHIFT	9
#define 	CONV1_OUT_SCALE		469.7					// TO CALCULATE

#define 	RELU1_INDIM			3840

#define 	MAXPOOL1_INDIM		192
#define 	MAXPOOL1_OUTDIM		96
#define 	MAXPOOL1_IF			20
#define 	MAXPOOL1_KDIM		2
#define 	MAXPOOL1_STRIDE		2
#define 	MAXPOOL1_PADDING	0

#define		CONV2_INDIM 		96
#define		CONV2_OUTDIM 		90
#define		CONV2_IF 			20
#define		CONV2_OF 			20
#define		CONV2_KDIM 			7
#define		CONV2_STRIDE 		1
#define		CONV2_PADDING 		0
#define		CONV2_BIAS_LSHIFT 	0
#define		CONV2_OUT_RSHIFT 	8
#define 	CONV2_OUT_SCALE		289.72					// TO CALCULATE

#define 	RELU2_INDIM			1800

#define 	MAXPOOL2_INDIM		90
#define 	MAXPOOL2_OUTDIM		45
#define 	MAXPOOL2_IF			20
#define 	MAXPOOL2_KDIM		2
#define 	MAXPOOL2_STRIDE		2
#define 	MAXPOOL2_PADDING	0

#define 	FC1_INDIM			900
#define 	FC1_OUTDIM			25
#define 	FC1_BIAS_LSHIFT		0
#define 	FC1_OUT_RSHIFT		8
#define 	FC1_OUT_SCALE		260.5					// TO CALCULATE

#define 	RELU3_INDIM			25

#define 	FC2_INDIM			25
#define 	FC2_OUTDIM			5
#define 	FC2_BIAS_LSHIFT		0
#define 	FC2_OUT_RSHIFT		7
#define 	FC2_OUT_SCALE		132.5					// TO CALCULATE

#define		COLBUFFER_DIM		FC1_INDIM				// maximum between:    FULLY-> FCN_INDIM    CONV-> 2 * CONVN_IF * CONVN_KDIM
#define		CNNBUFFER			CONV1_OF * CONV1_OUTDIM // maximum between:    FULLY-> FCN_INDIM    CONV-> CONVN_IF * CONVN_INDIM       CONV-> CONVN_OF * CONVN_OUTDIM

	#define	DUMMYINPUT
	#define	DUMMYWTBIAS

//	#define	OUTPUT_SHIFT

//	#define	CONV2_BASIC
//	#define	FULLY1_BASIC
//	#define	FULLY2_BASIC

	#define	CONV1_RAM
	#define	CONV2_RAM
	#define	FC1_RAM
	#define	FC2_RAM

#endif









//#ifndef DUMMYINPUT
//	/*N*/ 		q7_t input_N[] = 		ECG_DATA_0_1;
	/*R*/ 		q7_t input_R[] = 		{-16, -16, -16, -16, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -15, -14, -14, -14, -14, -14, -13, -14, -14, -14, -14, -15, -14, -14, -14, -13, -12, -12, -12, -12, -12, -12, -12, -12, -12, -13, -13, -13, -13, -13, -13, -13, -13, -14, -13, -13, -11, -11, -10, -11, -10, -11, -11, -12, -13, -14, -14, -15, -15, -15, -15, -15, -14, -15, -15, -15, -15, -15, -16, -16, -16, -15, -14, -11, -7, -2, 3, 6, 6, 3, -3, -11, -16, -17, -12, -1, 11, 24, 38, 50, 61, 69, 74, 77, 78, 76, 73, 69, 63, 55, 45, 36, 26, 16, 6, -4, -13, -19, -24, -25, -25, -25, -23, -22, -21, -20, -20, -19, -18, -18, -17, -17, -17, -17, -17, -18, -18, -18, -18, -18, -18, -18, -18, -17, -17, -17, -18, -18, -18, -19, -19, -19, -19, -18, -18, -18, -19, -19, -19, -20, -20, -19, -20, -20, -21, -20, -20, -20, -20, -20, -21, -21, -21, -21, -21, -22, -22, -22, -21, -22, -22, -22, -22, -23, -24, -23, -24, -24, -24, -24, -24, -24, -24, -25, -25, -25, -26, -26, -26, -26, -26, -26, -26};
//	/*V*/ 		q7_t input_V[] = 		{-18, -19, -19, -19, -19, -19, -19, -19, -19, -20, -21, -21, -23, -23, -23, -23, -23, -24, -24, -24, -25, -25, -26, -27, -27, -28, -29, -29, -29, -29, -29, -29, -29, -28, -28, -28, -28, -28, -28, -26, -26, -24, -23, -21, -21, -20, -19, -18, -18, -16, -15, -14, -13, -12, -11, -10, -10, -9, -9, -9, -8, -8, -8, -8, -7, -8, -6, -6, -6, -5, -5, -4, -4, -3, -1, 1, 2, 5, 8, 11, 14, 16, 18, 20, 21, 20, 17, 13, 5, -5, -18, -31, -44, -56, -66, -75, -83, -89, -93, -96, -98, -99, -99, -96, -93, -89, -86, -83, -80, -77, -73, -68, -63, -59, -55, -51, -47, -45, -41, -37, -33, -30, -28, -24, -20, -16, -9, -2, 5, 10, 14, 15, 15, 14, 13, 13, 13, 14, 15, 16, 16, 17, 17, 17, 18, 18, 19, 19, 19, 20, 21, 22, 22, 23, 23, 23, 23, 23, 24, 25, 27, 27, 28, 29, 30, 31, 31, 31, 33, 33, 34, 35, 35, 36, 37, 38, 38, 39, 39, 39, 38, 38, 37, 37, 36, 35, 35, 34, 33, 32, 30, 28, 27, 25, 23, 22, 20, 19};
//#else
				q7_t *input;
//				q7_t input[CONV1_INDIM];
//#endif



	q7_t 		*conv1_input;
	q7_t 		*conv1_output;

#if not defined CONV1_RAM && not defined DUMMYWTBIAS
	const q7_t 	conv1_wt[] = 		CONV1_WEIGHT;
	const q7_t 	conv1_bias[] = 		CONV1_BIAS;
#endif

#if defined CONV1_RAM && not defined DUMMYWTBIAS
	q7_t 		conv1_wt[] = 		CONV1_WEIGHT;
	q7_t 		conv1_bias[] = 		CONV1_BIAS;
#endif

#if not defined CONV1_RAM && defined DUMMYWTBIAS
	const q7_t 	conv1_wt[CONV1_IF * CONV1_OF * CONV1_KDIM] = {0};
	const q7_t 	conv1_bias[CONV1_OF] = {0};
#endif

#if defined CONV1_RAM && defined DUMMYWTBIAS
	q7_t 		conv1_wt[CONV1_IF * CONV1_OF * CONV1_KDIM];
	q7_t 		conv1_bias[CONV1_OF];
#endif

#ifdef OUTPUT_32BIT
	q31_t 		*conv1_output_o32;
#endif



	q7_t		*relu1_inout;



	q7_t		*maxpool1_inout;



	q7_t 		*conv2_input;
	q7_t 		*conv2_output;

#if not defined CONV2_RAM && not defined DUMMYWTBIAS
	const q7_t 	conv2_wt[] = 		CONV2_WEIGHT;
	const q7_t 	conv2_bias[] = 		CONV2_BIAS;
#endif

#if defined CONV2_RAM && not defined DUMMYWTBIAS
	q7_t 		conv2_wt[] = 		CONV2_WEIGHT;
	q7_t 		conv2_bias[] = 		CONV2_BIAS;
#endif

#if not defined CONV2_RAM && defined DUMMYWTBIAS
	const q7_t 	conv2_wt[CONV2_IF * CONV2_OF * CONV2_KDIM] = {0};
	const q7_t 	conv2_bias[CONV2_OF] = {0};
#endif

#if defined CONV2_RAM && defined DUMMYWTBIAS
	q7_t 		conv2_wt[CONV2_IF * CONV2_OF * CONV2_KDIM];
	q7_t 		conv2_bias[CONV2_OF];
#endif

#ifdef OUTPUT_32BIT
	q31_t 		*conv2_output_o32;
#endif



	q7_t		*relu2_inout;



	q7_t		*maxpool2_inout;



	q7_t		*fc1_input;
	q7_t		*fc1_output;

#if not defined FC1_RAM && not defined DUMMYWTBIAS
	const q7_t 	fc1_wt[] = 			FC1__PACKED_PARAMS_WEIGHT;
	const q7_t 	fc1_bias[] = 		FC1__PACKED_PARAMS_BIAS;
#endif

#if defined FC1_RAM && not defined DUMMYWTBIAS
	q7_t 		fc1_wt[] = 			FC1__PACKED_PARAMS_WEIGHT;
	q7_t 		fc1_bias[] = 		FC1__PACKED_PARAMS_BIAS;
#endif

#if not defined FC1_RAM && defined DUMMYWTBIAS
	const q7_t 	fc1_wt[FC1_INDIM * FC1_OUTDIM] = {0};
	const q7_t 	fc1_bias[FC1_OUTDIM] = {0};
#endif

#if defined FC1_RAM && defined DUMMYWTBIAS
	q7_t 	fc1_wt[FC1_INDIM * FC1_OUTDIM];
	q7_t 	fc1_bias[FC1_OUTDIM];
#endif

#ifdef OUTPUT_32BIT
	q31_t 		*fc1_output_o32;
#endif



	q7_t		*relu3_inout;



	q7_t		*fc2_input;
	q7_t		*fc2_output;

#if not defined FC2_RAM && not defined DUMMYWTBIAS
	const q7_t 	fc2_wt[] = 			FC2__PACKED_PARAMS_WEIGHT;
	const q7_t 	fc2_bias[] = 		FC2__PACKED_PARAMS_BIAS;
#endif

#if defined FC2_RAM && not defined DUMMYWTBIAS
	q7_t 		fc2_wt[] = 			FC2__PACKED_PARAMS_WEIGHT;
	q7_t 		fc2_bias[] = 		FC2__PACKED_PARAMS_BIAS;
#endif

#if not defined FC2_RAM && defined DUMMYWTBIAS
	const q7_t 	fc2_wt[FC2_INDIM * FC2_OUTDIM] = {0};
	const q7_t 	fc2_bias[FC2_OUTDIM] = {0};
#endif

#if defined FC2_RAM && defined DUMMYWTBIAS
	q7_t 	fc2_wt[FC2_INDIM * FC2_OUTDIM];
	q7_t 	fc2_bias[FC2_OUTDIM];
#endif

#ifdef OUTPUT_32BIT
	q31_t 		*fc2_output_o32;
#endif



	q15_t 			col_buffer[COLBUFFER_DIM];

	q7_t 			cnn_a_buffer[CNNBUFFER];
	q7_t 			cnn_b_buffer[CNNBUFFER];



	int 			fc2_output_maxindex;
	q7_t 			fc2_output_max;
	char 			classes[] = {'N', 'V', 'R', 'A', 'L'};



	#define			DSP_BUFFERDIM		512
	#define			CNN_HALFWINDOWDIM	(int) (CONV1_INDIM / 2)

	int32_t 		ecg_rawSignal		[DSP_BUFFERDIM * 2] = {0};
	unsigned int	ecg_rawSignal_rPeak;
	unsigned int	ecg_heartbeat;



	#define 		DSP_ALGORITHM_1
//	#define			DSP_ALGORITHM_OLD



#ifdef ALGORITHM_X
	#define			DSP_DC_FILTER
	#define			DSP_LP_FILTER
	#define			DSP_HP_FILTER
	#define 		DSP_DERIVATIVE
	#define 		DSP_SQUARED
	#define 		DSP_INTEGRAL
#endif

#ifdef DSP_ALGORITHM_1
	#define			DSP_DC_FILTER
	#define			DSP_LP_FILTER
//	#define			DSP_HP_FILTER
	#define 		DSP_DERIVATIVE
	#define 		DSP_SQUARED
//	#define 		DSP_INTEGRAL

	#define			LP_FILTER_GAIN
	#define			HP_FILTER_GAIN
	#define			DERIVATIVE_GAIN 		/ ((float) (8 * 36))

#define				SIGNAL_RISE_THRESHOLD	2000000

const unsigned int	signal_peak_delay = 	ECG_RATE / ((float) 5);
#endif

#ifdef DSP_DC_FILTER
	int32_t 		ecg_dcfilter			[DSP_BUFFERDIM * 2] = {0};
	int				ecg_dcfilter_delay = 	0;
#define				DC_FILTER_WINDOW		1
#define				DC_FILTER_DELAY 		0
#endif

#ifdef DSP_LP_FILTER
	int32_t 		ecg_lowpass				[DSP_BUFFERDIM * 2] = {0};
	int				ecg_lowpass_delay = 	0;
#define				LP_FILTER_WINDOW		12
#define				LP_FILTER_DELAY 		5
#endif

#ifdef DSP_HP_FILTER
	int32_t 		ecg_highpass			[DSP_BUFFERDIM * 2] = {0};
	int				ecg_highpass_delay = 	0;
#define				HP_FILTER_WINDOW 		32
#define				HP_FILTER_DELAY 		16
#endif

#ifdef DSP_DERIVATIVE
	int32_t 		ecg_derivative 			[DSP_BUFFERDIM * 2] = {0};
	int				ecg_derivative_delay = 	0;
#define				DERIVATIVE_WINDOW 		5
#define				DERIVATIVE_DELAY 		2
#endif

#ifdef DSP_SQUARED
	int32_t 		ecg_squared				[DSP_BUFFERDIM * 2] = {0};
	int				ecg_squared_delay = 	0;
#define				SQUARED_WINDOW 			1
#define				SQUARED_DELAY 			0
#endif

#ifdef DSP_INTEGRAL
	int32_t 		ecg_integral			[DSP_BUFFERDIM * 2] = {0};
	int				ecg_integral_index;
	int				ecg_integral_delay = 	0;
#define				INTEGRAL_WINDOW			20
#define				INTEGRAL_DELAY 			10
#define				INTEGRAL_GAIN 			/ ((float) INTEGRAL_WINDOW)
#endif

	int32_t			*ecg_signal;
	int				ecg_signal_index = 		0;
	int				ecg_signal_delay = 		0;

#define		 		PRE_BUFFERING 			CNN_HALFWINDOWDIM

	#define			FORCE_DSP_START_FROMZERO
	int 			dsp_start = 			0;



	int8_t			cnn_rawSignal[DSP_BUFFERDIM * 2];

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// SENSOR_1
//
//	static void 	get_thread_1(				void const *argument);
////	static void 	proc1_thread_1(				void const *argument);
////	static void 	proc2_thread_1(				void const *argument);
//	static void 	thrsh_thread_1(				void const *argument);
//	static void 	send_thread_1(				void const *argument);
//
////	osMessageQDef(								MESSAGE_proc1_SENSOR_1, DATAQUEUE_SIZE, 	int);
////	osMessageQDef(								MESSAGE_proc2_SENSOR_1, DATAQUEUE_SIZE, 	int);
//	osMessageQDef(								MESSAGE_THRSH_SENSOR_1, DATAQUEUE_SIZE, 	int);
//	osMessageQDef(								MESSAGE_SEND_SENSOR_1, 	DATAQUEUE_SIZE, 	int);
//
//	osSemaphoreDef(								SEMAPHORE_SENSOR_1);
//
//	osThreadDef(								THREAD_GET_SENSOR_1,	get_thread_1,		osPriorityAboveNormal,	0,	configMINIMAL_STACK_SIZE*2);
////	osThreadDef(								THREAD_proc1_SENSOR_1,	proc1_thread_1,		osPriorityAboveNormal,	0,	configMINIMAL_STACK_SIZE*2);
////	osThreadDef(								THREAD_proc2_SENSOR_1,	proc2_thread_1,		osPriorityNormal,		0,	configMINIMAL_STACK_SIZE*55);
//	osThreadDef(								THREAD_THRSH_SENSOR_1,	thrsh_thread_1,		osPriorityNormal,		0,	configMINIMAL_STACK_SIZE*2);
//	osThreadDef(								THREAD_SEND_SENSOR_1,	send_thread_1,		osPriorityNormal,		0,	configMINIMAL_STACK_SIZE*4);
//
//	void 			samp_timer_1_Callback(		void const *arg);
//	void 			send_timer_1_Callback(		void const *arg);
//
//
//
//// CUSTOM PARAMETERS & FUNCTIONS:
//
//
//
//	void 			temp_read_from_sensor(		SensorsData *data);
//	int 		 	temp_data_threshold(		SensorsData *data);
//
////	static void 	enable_1(					void);
////	static void 	disable_1(					void);
////	static void 	setOdr_1(					void);
//
//
//
//	#define TEMP_SAMP_PERIOD_VALUE				1000
//	#define TEMP_SEND_PERIOD_VALUE				1000
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// SENSOR_2
//
//	static void 	get_thread_2(				void const *argument);
////	static void 	proc1_thread_2(				void const *argument);
////	static void 	proc2_thread_2(				void const *argument);
//	static void 	thrsh_thread_2(				void const *argument);
//	static void 	send_thread_2(				void const *argument);
//
////	osMessageQDef(								MESSAGE_proc1_SENSOR_2, DATAQUEUE_SIZE, 	int);
////	osMessageQDef(								MESSAGE_proc2_SENSOR_2, DATAQUEUE_SIZE, 	int);
//	osMessageQDef(								MESSAGE_THRSH_SENSOR_2, DATAQUEUE_SIZE, 	int);
//	osMessageQDef(								MESSAGE_SEND_SENSOR_2, 	DATAQUEUE_SIZE, 	int);
//
//	osSemaphoreDef(								SEMAPHORE_SENSOR_2);
//
//	osThreadDef(								THREAD_GET_SENSOR_2,	get_thread_2,		osPriorityAboveNormal,	0,	configMINIMAL_STACK_SIZE*2);
////	osThreadDef(								THREAD_proc1_SENSOR_2,	proc1_thread_2,		osPriorityAboveNormal,	0,	configMINIMAL_STACK_SIZE*2);
////	osThreadDef(								THREAD_proc2_SENSOR_2,	proc2_thread_2,		osPriorityNormal,		0,	configMINIMAL_STACK_SIZE*55);
//	osThreadDef(								THREAD_THRSH_SENSOR_2,	thrsh_thread_2,		osPriorityNormal,		0,	configMINIMAL_STACK_SIZE*2);
//	osThreadDef(								THREAD_SEND_SENSOR_2,	send_thread_2,		osPriorityNormal,		0,	configMINIMAL_STACK_SIZE*4);
//
//	void 			samp_timer_2_Callback(		void const *arg);
//	void 			send_timer_2_Callback(		void const *arg);
//
//
//
//// CUSTOM PARAMETERS & FUNCTIONS:
//
//
//
//	void 			pres_read_from_sensor(		SensorsData *data);
//	int 			pres_data_threshold(		SensorsData *data);
//
////	static void 	enable_2(					void);
////	static void 	disable_2(					void);
////	static void 	setOdr_2(					void);
//
//
//
//	#define PRES_SAMP_PERIOD_VALUE				1000
//	#define PRES_SEND_PERIOD_VALUE				1000
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////






int main(void)
{

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// MAIN

	HAL_Init();

//	SystemClock_Config();
	SystemClock_Config_SPI();
//	SystemClock_Config_lp();
//	powerControl(RUN,16);
//	powerControl(LPRUN, 4);

	MX_GPIO_Init();
	MX_SPI3_Init();

//    BSP_LED_Init(LED1);
//
	HAL_PWREx_EnableVddUSB();													// Enable USB power on Pwrctrl CR2 register
	USBD_Init(								&USBD_Device, &VCP_Desc, 0);		// Init Device Library
	USBD_RegisterClass(						&USBD_Device, USBD_CDC_CLASS);		// Add Supported Class
	USBD_CDC_RegisterInterface(				&USBD_Device, &USBD_CDC_fops);		// Add Interface Callbacks for AUDIO and CDC Class
	USBD_Start(								&USBD_Device);						// Start Device Process

 	InitTargetPlatform(TARGET_SENSORTILE);

	InitBlueNRGStack();

	Add_ConfigW2ST_Service();
	Add_HWServW2ST_Service();

	timer_start();

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// MASTER

	master_threadId = 					osThreadCreate(		osThread(	THREAD_MASTER), 		NULL);
	master_queueId = 					osMessageCreate(	osMessageQ(	MESSAGE_MASTER), 		NULL);
	delayed_queueId = 					osMessageCreate(	osMessageQ(	MESSAGE_DELAYED), 		NULL);

	timer_period_master =				INIT_MASTER_TIMER_PERIOD;

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// BLE

	ble_threadId = 						osThreadCreate(		osThread(	THREAD_BLE), 		NULL);
	ble_semId = 						osSemaphoreCreate(	osSemaphore(SEMAPHORE_BLE), 	1);

	timer_period_ble =					INIT_BLE_TIMER_PERIOD;

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// ALL SENSORS

	thread_poolId = 					osPoolCreate(		osPool(		thread_pool));
	message_poolId = 					osPoolCreate(		osPool(		MESSAGE_SENSOR));

	mutex_semId = 						osSemaphoreCreate(	osSemaphore(SEMAPHORE_MUTEX), 		1);

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// SENSOR_0

	sprintf(sensorSetup[SENSOR_0].name,				"ECG sensor");
	sprintf(sensorSetup[SENSOR_0].code,				"...");
	sensorSetup[SENSOR_0].data_type =				INT16;
	sensorSetup[SENSOR_0].data_len = 				1;
	sensorSetup[SENSOR_0].sample_packing = 			1;
	sensorSetup[SENSOR_0].sensor_type =				ECG;


	sensorSetup[SENSOR_0].en_threads[0] =			1;
	sensorSetup[SENSOR_0].en_threads[1] =			1;
	sensorSetup[SENSOR_0].en_threads[2] =			1;
	sensorSetup[SENSOR_0].en_threads[3] =			1;
	sensorSetup[SENSOR_0].en_threads[4] =			1;

	sensorSetup[SENSOR_0].get_threadId =			osThreadCreate(		osThread(	THREAD_GET_SENSOR_0		), 	NULL);
	sensorSetup[SENSOR_0].proc1_threadId =			osThreadCreate(		osThread(	THREAD_proc1_SENSOR_0	), 	NULL);
	sensorSetup[SENSOR_0].proc2_threadId =			osThreadCreate(		osThread(	THREAD_proc2_SENSOR_0	), 	NULL);
	sensorSetup[SENSOR_0].thrsh_threadId =			osThreadCreate(		osThread(	THREAD_THRSH_SENSOR_0	), 	NULL);
	sensorSetup[SENSOR_0].send_threadId =			osThreadCreate(		osThread(	THREAD_SEND_SENSOR_0	), 	NULL);

	sensorSetup[SENSOR_0].proc1_queueId = 			osMessageCreate(	osMessageQ(	MESSAGE_proc1_SENSOR_0	), NULL);
	sensorSetup[SENSOR_0].proc2_queueId = 			osMessageCreate(	osMessageQ(	MESSAGE_proc2_SENSOR_0	), NULL);
	sensorSetup[SENSOR_0].thrsh_queueId = 			osMessageCreate(	osMessageQ(	MESSAGE_THRSH_SENSOR_0	), NULL);
	sensorSetup[SENSOR_0].send_queueId = 			osMessageCreate(	osMessageQ(	MESSAGE_SEND_SENSOR_0	), NULL);

	sensorSetup[SENSOR_0].get_semId = 				osSemaphoreCreate(	osSemaphore(SEMAPHORE_SENSOR_0), 	1);

	sensorSetup[SENSOR_0].samp_timer_Callback = 	samp_timer_0_Callback;
	osTimerDef( 									TIMER_SAMP_SENSOR_0, sensorSetup[SENSOR_0].samp_timer_Callback);
	sensorSetup[SENSOR_0].samp_timerId = 			osTimerCreate(osTimer(TIMER_SAMP_SENSOR_0), osTimerPeriodic, &exec);

	sensorSetup[SENSOR_0].send_timer_Callback = 	send_timer_0_Callback;
	osTimerDef( 									TIMER_SEND_SENSOR_0, sensorSetup[SENSOR_0].send_timer_Callback);
	sensorSetup[SENSOR_0].send_timerId = 			osTimerCreate(osTimer(TIMER_SEND_SENSOR_0), osTimerPeriodic, &exec);

	sensorSetup[SENSOR_0].last_value = 				(SensorsData *) pvPortMalloc(sizeof(SensorsData));
	sensorSetup[SENSOR_0].last_value->message_id = 	READ_LAST_VALUE;
	sensorSetup[SENSOR_0].last_value->data_len =	sensorSetup[SENSOR_0].data_len;
	sensorSetup[SENSOR_0].last_value->data = 		dataMalloc(sensorSetup[SENSOR_0].data_type, sensorSetup[SENSOR_0].data_len);

	sensorSetup[SENSOR_0].charHandle = 				addSensorCharacteristc(SENSOR_0, sensorSetup[SENSOR_0].sensor_type);

	sensorSetup[SENSOR_0].data_threshold = 			ecg_data_threshold;

	sensorSetup[SENSOR_0].get_data = 				ecg_read_from_sensor;
//	sensorSetup[SENSOR_0].enable = 					enable_0;
//	sensorSetup[SENSOR_0].disable = 				disable_0;

	sensorSetup[SENSOR_0].samp_period_ms =			ECG_SAMP_PERIOD_VALUE;
	sensorSetup[SENSOR_0].send_period_ms =			HB_SEND_PERIOD_VALUE;



#if defined(DSP_DC_FILTER) && !defined(FORCE_DSP_START_FROMZERO)
	if (dsp_start < DC_FILTER_WINDOW) 				dsp_start = DC_FILTER_WINDOW;
#endif
#if defined(DSP_LP_FILTER) && !defined(FORCE_DSP_START_FROMZERO)
	if (dsp_start < LP_FILTER_WINDOW) 				dsp_start = LP_FILTER_WINDOW;
#endif
#if defined(DSP_HP_FILTER) && !defined(FORCE_DSP_START_FROMZERO)
	if (dsp_start < HP_FILTER_WINDOW) 				dsp_start = HP_FILTER_WINDOW;
#endif
#if defined(DSP_DERIVATIVE) && !defined(FORCE_DSP_START_FROMZERO)
	if (dsp_start < DERIVATIVE_WINDOW) 				dsp_start = DERIVATIVE_WINDOW;
#endif
#if defined(DSP_SQUARED) && !defined(FORCE_DSP_START_FROMZERO)
	if (dsp_start < SQUARED_WINDOW) 				dsp_start = SQUARED_WINDOW;
#endif
#if defined(DSP_INTEGRAL) && !defined(FORCE_DSP_START_FROMZERO)
	if (dsp_start < INTEGRAL_WINDOW) 				dsp_start = INTEGRAL_WINDOW;
#endif
#ifndef FORCE_DSP_START_FROMZERO
	dsp_start = 									(CNN_HALFWINDOWDIM - 1) - dsp_start;
#endif



#ifdef DSP_DC_FILTER
	ecg_signal_delay +=								DC_FILTER_DELAY;
	ecg_dcfilter_delay =							ecg_signal_delay;
#endif
#ifdef DSP_LP_FILTER
	ecg_signal_delay +=								LP_FILTER_DELAY;
	ecg_lowpass_delay =								ecg_signal_delay;
#endif
#ifdef DSP_HP_FILTER
	ecg_signal_delay +=								HP_FILTER_DELAY;
	ecg_highpass_delay =							ecg_signal_delay;
#endif
#ifdef DSP_DERIVATIVE
	ecg_signal_delay +=								DERIVATIVE_DELAY;
	ecg_derivative_delay =							ecg_signal_delay;
#endif
#ifdef DSP_SQUARED
	ecg_signal_delay +=								SQUARED_DELAY;
	ecg_squared_delay =								ecg_signal_delay;
#endif
#ifdef DSP_INTEGRAL
	ecg_signal_delay +=								INTEGRAL_DELAY;
	ecg_integral_delay =							ecg_signal_delay;
#endif


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	// SENSOR_1
//
//	sprintf(sensorSetup[SENSOR_1].name,			"Temperature sensor");
//	sprintf(sensorSetup[SENSOR_1].code,			"...");
//	sensorSetup[SENSOR_1].data_type =			FLOAT;
//	sensorSetup[SENSOR_1].sensor_type =			DATA_1D;
//
//	sensorSetup[SENSOR_1].en_threads[0] =		1;
//	sensorSetup[SENSOR_1].en_threads[1] =		0;
//	sensorSetup[SENSOR_1].en_threads[2] =		0;
//	sensorSetup[SENSOR_1].en_threads[3] =		1;
//	sensorSetup[SENSOR_1].en_threads[4] =		1;
//
//	sensorSetup[SENSOR_1].get_threadId =		osThreadCreate(		osThread(	THREAD_GET_SENSOR_1		), 	NULL);
////	sensorSetup[SENSOR_1].proc1_threadId =		osThreadCreate(		osThread(	THREAD_proc1_SENSOR_1	), 	NULL);
////	sensorSetup[SENSOR_1].proc2_threadId =		osThreadCreate(		osThread(	THREAD_proc2_SENSOR_1	), 	NULL);
//	sensorSetup[SENSOR_1].thrsh_threadId =		osThreadCreate(		osThread(	THREAD_THRSH_SENSOR_1	), 	NULL);
//	sensorSetup[SENSOR_1].send_threadId =		osThreadCreate(		osThread(	THREAD_SEND_SENSOR_1	), 	NULL);
//
////	sensorSetup[SENSOR_1].proc1_queueId = 		osMessageCreate(	osMessageQ(	MESSAGE_proc1_SENSOR_1	), NULL);
////	sensorSetup[SENSOR_1].proc2_queueId = 		osMessageCreate(	osMessageQ(	MESSAGE_proc2_SENSOR_1	), NULL);
//	sensorSetup[SENSOR_1].thrsh_queueId = 		osMessageCreate(	osMessageQ(	MESSAGE_THRSH_SENSOR_1	), NULL);
//	sensorSetup[SENSOR_1].send_queueId = 		osMessageCreate(	osMessageQ(	MESSAGE_SEND_SENSOR_1	), NULL);
//
//	sensorSetup[SENSOR_1].get_semId = 			osSemaphoreCreate(	osSemaphore(SEMAPHORE_SENSOR_1), 	1);
//
//	sensorSetup[SENSOR_1].samp_timer_Callback = samp_timer_1_Callback;
//	osTimerDef( 								TIMER_SAMP_SENSOR_1, sensorSetup[SENSOR_1].samp_timer_Callback);
//	sensorSetup[SENSOR_1].samp_timerId = 		osTimerCreate(osTimer(TIMER_SAMP_SENSOR_1), osTimerPeriodic, &exec);
//
//	sensorSetup[SENSOR_1].send_timer_Callback = send_timer_1_Callback;
//	osTimerDef( 								TIMER_SEND_SENSOR_1, sensorSetup[SENSOR_1].send_timer_Callback);
//	sensorSetup[SENSOR_1].send_timerId = 		osTimerCreate(osTimer(TIMER_SEND_SENSOR_1), osTimerPeriodic, &exec);
//
//	sensorSetup[SENSOR_1].last_value = 			(SensorsData *) pvPortMalloc(sizeof(SensorsData) * 1);
//	sensorSetup[SENSOR_1].sample_packing = 		1;
//	sensorSetup[SENSOR_1].last_value->data = 	dataMalloc(sensorSetup[SENSOR_1].data_type, sensorSetup[SENSOR_1].sample_packing);
//
//	sensorSetup[SENSOR_1].charHandle = 			addSensorCharacteristc(SENSOR_1, sensorSetup[SENSOR_1].sensor_type);
//
//	sensorSetup[SENSOR_1].data_threshold = 		temp_data_threshold;
//
//	sensorSetup[SENSOR_1].get_data = 			temp_read_from_sensor;
////	sensorSetup[SENSOR_1].enable = 				enable_1;
////	sensorSetup[SENSOR_1].disable = 			disable_1;
//
//	sensorSetup[SENSOR_1].samp_period_ms =		TEMP_SAMP_PERIOD_VALUE;
//	sensorSetup[SENSOR_1].send_period_ms =		TEMP_SEND_PERIOD_VALUE;
//
//
//	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	// SENSOR_2
//
//	sprintf(sensorSetup[SENSOR_2].name,			"Pressure sensor");
//	sprintf(sensorSetup[SENSOR_2].code,			"...");
//	sensorSetup[SENSOR_2].data_type =			FLOAT;
//	sensorSetup[SENSOR_2].sensor_type =			DATA_1D;
//
//	sensorSetup[SENSOR_2].en_threads[0] =		1;
//	sensorSetup[SENSOR_2].en_threads[1] =		0;
//	sensorSetup[SENSOR_2].en_threads[2] =		0;
//	sensorSetup[SENSOR_2].en_threads[3] =		1;
//	sensorSetup[SENSOR_2].en_threads[4] =		1;
//
//	sensorSetup[SENSOR_2].get_threadId =		osThreadCreate(		osThread(	THREAD_GET_SENSOR_2		), 	NULL);
////	sensorSetup[SENSOR_2].proc1_threadId =		osThreadCreate(		osThread(	THREAD_proc1_SENSOR_2	), 	NULL);
////	sensorSetup[SENSOR_2].proc2_threadId =		osThreadCreate(		osThread(	THREAD_proc2_SENSOR_2	), 	NULL);
//	sensorSetup[SENSOR_2].thrsh_threadId =		osThreadCreate(		osThread(	THREAD_THRSH_SENSOR_2	), 	NULL);
//	sensorSetup[SENSOR_2].send_threadId =		osThreadCreate(		osThread(	THREAD_SEND_SENSOR_2	), 	NULL);
//
////	sensorSetup[SENSOR_2].proc1_queueId = 		osMessageCreate(	osMessageQ(	MESSAGE_proc1_SENSOR_2	), NULL);
////	sensorSetup[SENSOR_2].proc2_queueId = 		osMessageCreate(	osMessageQ(	MESSAGE_proc2_SENSOR_2	), NULL);
//	sensorSetup[SENSOR_2].thrsh_queueId = 		osMessageCreate(	osMessageQ(	MESSAGE_THRSH_SENSOR_2	), NULL);
//	sensorSetup[SENSOR_2].send_queueId = 		osMessageCreate(	osMessageQ(	MESSAGE_SEND_SENSOR_2	), NULL);
//
//	sensorSetup[SENSOR_2].get_semId = 			osSemaphoreCreate(	osSemaphore(SEMAPHORE_SENSOR_2), 	1);
//
//	sensorSetup[SENSOR_2].samp_timer_Callback = samp_timer_2_Callback;
//	osTimerDef( 								TIMER_SAMP_SENSOR_2, sensorSetup[SENSOR_2].samp_timer_Callback);
//	sensorSetup[SENSOR_2].samp_timerId = 		osTimerCreate(osTimer(TIMER_SAMP_SENSOR_2), osTimerPeriodic, &exec);
//
//	sensorSetup[SENSOR_2].send_timer_Callback = send_timer_2_Callback;
//	osTimerDef( 								TIMER_SEND_SENSOR_2, sensorSetup[SENSOR_2].send_timer_Callback);
//	sensorSetup[SENSOR_2].send_timerId = 		osTimerCreate(osTimer(TIMER_SEND_SENSOR_2), osTimerPeriodic, &exec);
//
//	sensorSetup[SENSOR_2].last_value = 			(SensorsData *) pvPortMalloc(sizeof(SensorsData) * 1);
//	sensorSetup[SENSOR_2].sample_packing = 		1;
//	sensorSetup[SENSOR_2].last_value->data = 	dataMalloc(sensorSetup[SENSOR_2].data_type, sensorSetup[SENSOR_2].sample_packing);
//
//	sensorSetup[SENSOR_2].charHandle = 			addSensorCharacteristc(SENSOR_2, sensorSetup[SENSOR_2].sensor_type);
//
//	sensorSetup[SENSOR_2].data_threshold = 		pres_data_threshold;
//
//	sensorSetup[SENSOR_2].get_data = 			pres_read_from_sensor;
////	sensorSetup[SENSOR_2].enable = 				enable_2;
////	sensorSetup[SENSOR_2].disable = 			disable_2;
//
//	sensorSetup[SENSOR_2].samp_period_ms =		TEMP_SAMP_PERIOD_VALUE;
//	sensorSetup[SENSOR_2].send_period_ms =		TEMP_SEND_PERIOD_VALUE;
//
//
//	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//for(;;)
//{
//	timer_0 = timer_get();
//	while(timer_0 + 4*1000000 > timer_get());
//	LED_TGG;
//}

#define TESTCODE

#ifdef TESTCODE_
	for(;;)
	{
//		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_SET);
		  LED_ON;
		  HAL_Delay(1000);
//		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_RESET);
		  LED_OFF;
		  HAL_Delay(1000);
	}
#endif

#ifdef TESTCODE_

	STLBLE_PRINTF("\r\nTEST code");

	input = input_R;



	timer_0 = timer_get();

	conv1_input = input; conv1_output = cnn_a_buffer;
	arm_convolve_HWC_q7_basic_nonsquare_div(conv1_input, CONV1_INDIM, 1, CONV1_IF, conv1_wt, CONV1_OF, CONV1_KDIM, 1, CONV1_PADDING, 0, CONV1_STRIDE, 1, conv1_bias, CONV1_BIAS_LSHIFT, CONV1_OUT_SCALE, conv1_output, CONV1_OUTDIM, 1, col_buffer, NULL);

	timer_1 = timer_get();

	relu1_inout = cnn_a_buffer;
	arm_relu_q7(relu1_inout, RELU1_INDIM);

	timer_2 = timer_get();

	maxpool1_inout = cnn_a_buffer;
	arm_maxpool_q7_HWC_1D(maxpool1_inout, MAXPOOL1_INDIM, MAXPOOL1_IF, MAXPOOL1_KDIM, MAXPOOL1_PADDING, MAXPOOL1_STRIDE, MAXPOOL1_OUTDIM);

	timer_3 = timer_get();

	conv2_input = cnn_a_buffer; conv2_output = cnn_b_buffer;

#if defined(CONV2_BASIC) && defined(OUTPUT_SHIFT)
	arm_convolve_HWC_q7_basic_nonsquare(conv2_input, CONV2_INDIM, 1, CONV2_IF, conv2_wt, CONV2_OF, CONV2_KDIM, 1, CONV2_PADDING, 0, CONV2_STRIDE, 1, conv2_bias, CONV2_BIAS_LSHIFT, CONV2_OUT_RSHIFT, conv2_output, CONV2_OUTDIM, 1, col_buffer, NULL);
#endif
#if defined(CONV2_BASIC) && !defined(OUTPUT_SHIFT)
	arm_convolve_HWC_q7_basic_nonsquare_div(conv2_input, CONV2_INDIM, 1, CONV2_IF, conv2_wt, CONV2_OF, CONV2_KDIM, 1, CONV2_PADDING, 0, CONV2_STRIDE, 1, conv2_bias, CONV2_BIAS_LSHIFT, CONV2_OUT_SCALE, conv2_output, CONV2_OUTDIM, 1, col_buffer, NULL);
#endif
#if !defined(CONV2_BASIC) && defined(OUTPUT_SHIFT)
	arm_convolve_HWC_q7_fast_nonsquare(conv2_input, CONV2_INDIM, 1, CONV2_IF, conv2_wt, CONV2_OF, CONV2_KDIM, 1, CONV2_PADDING, 0, CONV2_STRIDE, 1, conv2_bias, CONV2_BIAS_LSHIFT, CONV2_OUT_SCALE, conv2_output, CONV2_OUTDIM, 1, col_buffer, NULL);
#endif
#if !defined(CONV2_BASIC) && !defined(OUTPUT_SHIFT)
	arm_convolve_HWC_q7_fast_nonsquare_div(conv2_input, CONV2_INDIM, 1, CONV2_IF, conv2_wt, CONV2_OF, CONV2_KDIM, 1, CONV2_PADDING, 0, CONV2_STRIDE, 1, conv2_bias, CONV2_BIAS_LSHIFT, CONV2_OUT_RSHIFT, conv2_output, CONV2_OUTDIM, 1, col_buffer, NULL);
#endif

	timer_4 = timer_get();

	relu2_inout = cnn_b_buffer;
	arm_relu_q7(relu2_inout, RELU2_INDIM);

	timer_5 = timer_get();

	maxpool2_inout = cnn_b_buffer;
	arm_maxpool_q7_HWC_1D(maxpool2_inout, MAXPOOL2_INDIM, MAXPOOL2_IF, MAXPOOL2_KDIM, MAXPOOL2_PADDING, MAXPOOL2_STRIDE, MAXPOOL2_OUTDIM);

	timer_6 = timer_get();

	fc1_input = cnn_b_buffer; fc1_output = cnn_a_buffer;
#if defined(FULLY1_BASIC) && defined(OUTPUT_SHIFT)
	arm_fully_connected_q7(fc1_input, fc1_wt, FC1_INDIM, FC1_OUTDIM, FC1_BIAS_LSHIFT, FC1_OUT_RSHIFT, fc1_bias, fc1_output, col_buffer);
#endif
#if defined(FULLY1_BASIC) && !defined(OUTPUT_SHIFT)
	arm_fully_connected_q7_div(fc1_input, fc1_wt, FC1_INDIM, FC1_OUTDIM, FC1_BIAS_LSHIFT, FC1_OUT_SCALE, fc1_bias, fc1_output, col_buffer);
#endif
#if !defined(FULLY1_BASIC) && defined(OUTPUT_SHIFT)
	arm_fully_connected_q7_opt(fc1_input, fc1_wt, FC1_INDIM, FC1_OUTDIM, FC1_BIAS_LSHIFT, FC1_OUT_RSHIFT, fc1_bias, fc1_output, col_buffer);
#endif
#if !defined(FULLY1_BASIC) && !defined(OUTPUT_SHIFT)
	arm_fully_connected_q7_opt_div(fc1_input, fc1_wt, FC1_INDIM, FC1_OUTDIM, FC1_BIAS_LSHIFT, FC1_OUT_SCALE, fc1_bias, fc1_output, col_buffer);
#endif

	timer_7 = timer_get();

	relu3_inout = cnn_a_buffer;
	arm_relu_q7(relu3_inout, RELU3_INDIM);

	timer_8 = timer_get();

	fc2_input = cnn_a_buffer; fc2_output = cnn_b_buffer;
#if defined(FULLY2_BASIC) && defined(OUTPUT_SHIFT)
	arm_fully_connected_q7(fc2_input, fc2_wt, FC2_INDIM, FC2_OUTDIM, FC2_BIAS_LSHIFT, FC2_OUT_RSHIFT, fc2_bias, fc2_output, col_buffer);
#endif
#if defined(FULLY2_BASIC) && !defined(OUTPUT_SHIFT)
	arm_fully_connected_q7_div(fc2_input, fc2_wt, FC2_INDIM, FC2_OUTDIM, FC2_BIAS_LSHIFT, FC2_OUT_SCALE, fc2_bias, fc2_output, col_buffer);
#endif
#if !defined(FULLY2_BASIC) && defined(OUTPUT_SHIFT)
	arm_fully_connected_q7_opt(fc2_input, fc2_wt, FC2_INDIM, FC2_OUTDIM, FC2_BIAS_LSHIFT, FC2_OUT_RSHIFT, fc2_bias, fc2_output, col_buffer);
#endif
#if !defined(FULLY2_BASIC) && !defined(OUTPUT_SHIFT)
	arm_fully_connected_q7_opt_div(fc2_input, fc2_wt, FC2_INDIM, FC2_OUTDIM, FC2_BIAS_LSHIFT, FC2_OUT_SCALE, fc2_bias, fc2_output, col_buffer);
#endif

	timer_9 = timer_get();



	STLBLE_PRINTF("\r\n%d", timer_1 - timer_0 - 1); // CONV1 time:
	STLBLE_PRINTF("\r\n%d", timer_2 - timer_1 - 1); // RELU1 time:
	STLBLE_PRINTF("\r\n%d", timer_3 - timer_2 - 1); // POOL1 time:
	STLBLE_PRINTF("\r\n%d", timer_4 - timer_3 - 1); // CONV2 time:
	STLBLE_PRINTF("\r\n%d", timer_5 - timer_4 - 1); // RELU2 time:
	STLBLE_PRINTF("\r\n%d", timer_6 - timer_5 - 1); // POOL2 time:
	STLBLE_PRINTF("\r\n%d", timer_7 - timer_6 - 1); // FULL1 time:
	STLBLE_PRINTF("\r\n%d", timer_8 - timer_7 - 1); // RELU3 time:
	STLBLE_PRINTF("\r\n%d", timer_9 - timer_8 - 1); // FULL2 time:

	STLBLE_PRINTF("\r\n\nDIV NN time:   %d", timer_9 - timer_0 - 9);

	for (;;)
#endif






	osKernelStart();					// Start scheduler

	for (;;);							// We should never get here as control is now taken by the scheduler
}

static void master_thread(void const *argument)
{
	(void) argument;

	Sensor_IO_SPI_CS_Init_All();		// Configure and disable all the Chip Select pins

	for (int i = 0; i < NUMBER_OF_SENSORS; i++)
	{
		if(sensorSetup[i].en_threads[0]) osThreadSuspend(sensorSetup[i].get_threadId);
		if(sensorSetup[i].en_threads[1]) osThreadSuspend(sensorSetup[i].proc1_threadId);
		if(sensorSetup[i].en_threads[2]) osThreadSuspend(sensorSetup[i].proc2_threadId);
		if(sensorSetup[i].en_threads[3]) osThreadSuspend(sensorSetup[i].thrsh_threadId);
		if(sensorSetup[i].en_threads[4]) osThreadSuspend(sensorSetup[i].send_threadId);
	}

//	osThreadDef_t *thread_def_test = osThread(THREAD_proc2_SENSOR_0);
//	vApplicationStackOverflowHook(NULL, thread_def_test->name); // (const portCHAR *)

	osEvent evt;
	int *message;

	init_all_task();

	master_timer_Start();
	ble_timer_Start();

	timer_workload = HAL_GetTick();



	for (;;)
	{
		evt = osMessageGet(master_queueId, osWaitForever);

		if (evt.status == osEventMessage)
		{
			message = (int *) evt.value.p;

			switch (*message)
			{
				case TASK_SETUP:

					for (int i = 0; i < NUMBER_OF_SENSORS; i++)
					{
						switch ((taskState & (3 << (2*i))) >> (2*i))
						{
							case (0):
								osTimerStop( sensorSetup[i].samp_timerId);
								osTimerStop( sensorSetup[i].send_timerId);

								if(sensorSetup[i].en_threads[0]) osThreadSuspend(sensorSetup[i].get_threadId);
								if(sensorSetup[i].en_threads[1]) osThreadSuspend(sensorSetup[i].proc1_threadId);
								if(sensorSetup[i].en_threads[2]) osThreadSuspend(sensorSetup[i].proc2_threadId);
								if(sensorSetup[i].en_threads[3]) osThreadSuspend(sensorSetup[i].thrsh_threadId);
								if(sensorSetup[i].en_threads[4]) osThreadSuspend(sensorSetup[i].send_threadId);
							break;

							case (1):
								osTimerStart(sensorSetup[i].samp_timerId, sensorSetup[i].samp_period_ms);
								osTimerStop( sensorSetup[i].send_timerId);

								sensorSetup[i].out_get_queueId = 	sensorSetup[i].thrsh_queueId;
								sensorSetup[i].out_thrsh_queueId = 	sensorSetup[i].send_queueId;

								if (i != SENSOR_0) sensorSetup[i].sample_packing = 1;
								if (i == SENSOR_0) sensorSetup[i].sample_packing = 8;

								if(sensorSetup[i].en_threads[0]) osThreadResume(sensorSetup[i].get_threadId);
								if(sensorSetup[i].en_threads[1]) osThreadSuspend(sensorSetup[i].proc1_threadId);
								if(sensorSetup[i].en_threads[2]) osThreadSuspend(sensorSetup[i].proc2_threadId);
								if(sensorSetup[i].en_threads[3]) osThreadResume(sensorSetup[i].thrsh_threadId);
								if(sensorSetup[i].en_threads[4]) osThreadResume(sensorSetup[i].send_threadId);
							break;

							case (2):
//								powerControl(RUN, 8);

								osTimerStart(sensorSetup[i].samp_timerId, sensorSetup[i].samp_period_ms);
								if (i == SENSOR_0) osTimerStart(sensorSetup[i].send_timerId, sensorSetup[i].send_period_ms);

								sensorSetup[i].out_get_queueId = 	sensorSetup[i].proc1_queueId;
								sensorSetup[i].out_proc1_queueId = 	NULL;
								sensorSetup[i].out_thrsh_queueId = 	sensorSetup[i].send_queueId;

								sensorSetup[i].sample_packing = 1;

								if(sensorSetup[i].en_threads[0]) osThreadResume(sensorSetup[i].get_threadId);
								if(sensorSetup[i].en_threads[1]) osThreadResume(sensorSetup[i].proc1_threadId);
								if(sensorSetup[i].en_threads[2]) osThreadSuspend(sensorSetup[i].proc2_threadId);
								if(sensorSetup[i].en_threads[3]) osThreadResume(sensorSetup[i].thrsh_threadId);
								if(sensorSetup[i].en_threads[4]) osThreadResume(sensorSetup[i].send_threadId);

							break;

							case (3):
								osTimerStart(sensorSetup[i].samp_timerId, sensorSetup[i].samp_period_ms);
								osTimerStop( sensorSetup[i].send_timerId);

								sensorSetup[i].out_get_queueId = 	sensorSetup[i].proc1_queueId;
								sensorSetup[i].out_proc1_queueId = 	sensorSetup[i].proc2_queueId;
								sensorSetup[i].out_proc2_queueId = 	sensorSetup[i].thrsh_queueId;
								sensorSetup[i].out_thrsh_queueId = 	sensorSetup[i].send_queueId;

								sensorSetup[i].sample_packing = 1;

								if(sensorSetup[i].en_threads[0]) osThreadResume(sensorSetup[i].get_threadId);
								if(sensorSetup[i].en_threads[1]) osThreadResume(sensorSetup[i].proc1_threadId);
								if(sensorSetup[i].en_threads[2]) osThreadResume(sensorSetup[i].proc2_threadId);
								if(sensorSetup[i].en_threads[3]) osThreadResume(sensorSetup[i].thrsh_threadId);
								if(sensorSetup[i].en_threads[4]) osThreadResume(sensorSetup[i].send_threadId);
							break;

							default:
								osTimerStop( sensorSetup[i].samp_timerId);
								osTimerStop( sensorSetup[i].send_timerId);

								if(sensorSetup[i].en_threads[0]) osThreadSuspend(sensorSetup[i].get_threadId);
								if(sensorSetup[i].en_threads[1]) osThreadSuspend(sensorSetup[i].proc1_threadId);
								if(sensorSetup[i].en_threads[2]) osThreadSuspend(sensorSetup[i].proc2_threadId);
								if(sensorSetup[i].en_threads[3]) osThreadSuspend(sensorSetup[i].thrsh_threadId);
								if(sensorSetup[i].en_threads[4]) osThreadSuspend(sensorSetup[i].send_threadId);
							break;
						}
					}

					break;

				case SAMP_PERIOD_SETUP:

					sensorSetup[samp_period_ms_index].samp_period_ms = samp_period_ms_value;
					init_all_task();

					break;

				case SEND_PERIOD_SETUP:

					sensorSetup[send_period_ms_index].send_period_ms = send_period_ms_value;
					init_all_task();

					break;

				case PERIOD_CHECK:

//					if(0) GetBatteryInfoData();

					workLoad = (HAL_GetTick() - timer_workload) / (timer_period_master / (float) (MAX_WORKLOAD));
					timer_workload = HAL_GetTick();
//
//					STLBLE_PRINTF("\r\n\r\nWorkload: %d", workLoad);

					break;

				default:

					// ...

					break;

			}

			osPoolFree(thread_poolId, message);
		}
	}
}

static void ble_thread(	void const *argument)
{

	uint32_t StartTime;

	StartTime = osKernelSysTick();

	for (;;)
	{
		osSemaphoreWait(ble_semId, osWaitForever);

		if (!connected)
		{
			if (!TargetBoardFeatures.LedStatus)
			{
				if (osKernelSysTick()-StartTime > (float)1000)
				{
					StartTime = osKernelSysTick();
				}
			}
			else
			{
				if (osKernelSysTick()-StartTime > (float)100)
				{
					StartTime = osKernelSysTick();
				}
			}
		}

		if (HCI_ProcessEvent)							// Handle BLE event
		{
			HCI_ProcessEvent=0;
			HCI_Process();
		}

		if (set_connectable)							// Update the BLE advertise data and make the Board connectable
		{
			setConnectable();
			set_connectable = FALSE;
		}

//	    __WFI();
	}
}









//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SENSOR_0

	static void get_thread_0(	void const *argument) {	gp_read_from_sensor(	SENSOR_0 ); }
	static void proc1_thread_0(	void const *argument) { ecg_peak_detector(		SENSOR_0 ); }
	static void proc2_thread_0(	void const *argument) { ecg_cnn_classifier(		SENSOR_0 ); }
	static void thrsh_thread_0(	void const *argument) { gp_data_threshold(		SENSOR_0 ); }
	static void send_thread_0(	void const *argument) { gp_send_to_gateway(		SENSOR_0 ); }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// SENSOR_1
//
//	static void get_thread_1(	void const *argument) {	gp_read_from_sensor(	SENSOR_1 ); }
////	static void proc1_thread_1(	void const *argument) { _proc0(					SENSOR_1 ); }
////	static void proc2_thread_1(	void const *argument) { _proc1(					SENSOR_1 ); }
//	static void thrsh_thread_1(	void const *argument) { gp_data_threshold(		SENSOR_1 ); }
//	static void send_thread_1(	void const *argument) { gp_send_to_gateway(		SENSOR_1 ); }
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// SENSOR_2
//
//	static void get_thread_2(	void const *argument) {	gp_read_from_sensor(	SENSOR_2 ); }
////	static void proc1_thread_2(	void const *argument) { _proc0(					SENSOR_2 ); }
////	static void proc2_thread_2(	void const *argument) { _proc1(					SENSOR_2 ); }
//	static void thrsh_thread_2(	void const *argument) { gp_data_threshold(		SENSOR_2 ); }
//	static void send_thread_2(	void const *argument) { gp_send_to_gateway(		SENSOR_2 ); }
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////









//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GENERAL PURPOSE FUNCTIONS

static void gp_read_from_sensor(int sensorId)
{
//	SensorsData *mptr;

	int buf_len;
	char buf[50];
	buf_len = sprintf(buf, "SPI Test\n");

	for (;;)
	{
		osSemaphoreWait(sensorSetup[sensorId].get_semId, osWaitForever);

//		mptr = (SensorsData *) pvPortMalloc(sizeof(SensorsData));
//		mptr->data_len = sensorSetup[sensorId].data_len;
//		mptr->data = dataMalloc(sensorSetup[sensorId].data_type, mptr->data_len);
//
//		if (mptr != NULL)
//		{
//			sensorSetup[sensorId].get_data(mptr);
//			mptr->ms_counter = osKernelSysTick();
//			mptr->message_id = READ_FROM_FIFO;
//		}
//		if (osMessagePut(sensorSetup[sensorId].out_get_queueId, (uint32_t) mptr, osWaitForever) != osOK) Error_Handler();

		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi3, (uint8_t *)buf, buf_len, 100);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_SET);

		STLBLE_PRINTF("\r\nSENT");
	}
}

static void gp_data_threshold(int sensorId)
{
	osEvent evt;
	SensorsData *mptr;
	SensorsData *rptr;

	int sendCnt = 0;

	for (;;)
	{
		evt = 	osMessageGet(sensorSetup[sensorId].thrsh_queueId, osWaitForever);

		if (evt.status == osEventMessage)
		{

			rptr = (SensorsData *) evt.value.p;

			if(!sendCnt)
			{
				mptr = 					(SensorsData *) pvPortMalloc(sizeof(SensorsData));
				mptr->data_len = 		rptr->data_len * sensorSetup[sensorId].sample_packing;
				mptr->data = 			dataMalloc(sensorSetup[sensorId].data_type, mptr->data_len);
				sendCnt = 				sensorSetup[sensorId].sample_packing;
			}

			dataCopy(sensorSetup[sensorId].data_type, &sendCnt, rptr, mptr);

			if(!sendCnt)
			{
				if(sensorSetup[sensorId].data_threshold(mptr) == SEND)
				{
					mptr->ms_counter = rptr->ms_counter;
					if (osMessagePut(sensorSetup[sensorId].out_thrsh_queueId, (uint32_t) mptr, osWaitForever) != osOK) Error_Handler();
				}
			}

		}

		if(rptr->message_id == READ_FROM_FIFO)
		{
			vPortFree(rptr->data);
			vPortFree(rptr);
		}
	}
}

static void gp_send_to_gateway(int sensorId)
{
	osEvent evt;
	SensorsData *rptr;

	for (;;)
	{
		evt = 	osMessageGet(sensorSetup[sensorId].send_queueId, osWaitForever);

		if (evt.status == osEventMessage)
		{
			rptr = (SensorsData *) evt.value.p;



//			*((int16_t*) rptr->data) = workLoad;
//			rptr->data_len = 1;
//
//			charUpdate(
//						sensorSetup[sensorId].charHandle,
//						sensorSetup[sensorId].data_type,
//						sensorSetup[sensorId].sensor_type,
//						rptr->ms_counter,
//						rptr->data,
//						rptr->data_len
//					  );



			charUpdate(
						sensorSetup[sensorId].charHandle,
						sensorSetup[sensorId].data_type,
						sensorSetup[sensorId].sensor_type,
						rptr->ms_counter,
						rptr->data,
						rptr->data_len
					  );



		}

		vPortFree(rptr->data);
		vPortFree(rptr);
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////









//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SENSOR_0: FUNCTIONS & TIMER

void ecg_read_from_sensor(SensorsData *packet)
{
	packet->data_len = 				1;
	*((int16_t*) packet->data) = 	ecg_data[ecg_count % ECG_SAMPLE];
	ecg_prev_count = 				ecg_count++;
}

static void ecg_peak_detector(int sensorId)
{
	osEvent evt;
	SensorsData *mptr;
	SensorsData *rptr;

	while (ecg_signal_index < 512 /*CNN_HALFWINDOWDIM*/)
	{

		evt = osMessageGet(sensorSetup[sensorId].proc1_queueId, osWaitForever);

		if (evt.status == osEventMessage)
		{
			rptr = (SensorsData *) evt.value.p;

			cnn_rawSignal[(ecg_signal_index % DSP_BUFFERDIM)] = (*((int16_t*) rptr->data) >> 8);
			cnn_rawSignal[(ecg_signal_index % DSP_BUFFERDIM) + DSP_BUFFERDIM] = (*((int16_t*) rptr->data) >> 8);

			ecg_rawSignal[(ecg_signal_index % DSP_BUFFERDIM)] = *((int16_t*) rptr->data);
			ecg_rawSignal[(ecg_signal_index % DSP_BUFFERDIM) + DSP_BUFFERDIM] = *((int16_t*) rptr->data);
			ecg_signal = ecg_rawSignal;



#ifdef DSP_DC_FILTER
			if (ecg_signal_index >= dsp_start + 1)
			{
				ecg_dcfilter[ecg_signal_index % DSP_BUFFERDIM] = 	ecg_signal[ecg_signal_index % DSP_BUFFERDIM]
																	- ecg_signal[(ecg_signal_index - 1) % DSP_BUFFERDIM]
																	+ 0.995 * ecg_dcfilter[(ecg_signal_index - 1) % DSP_BUFFERDIM];

				ecg_dcfilter[(ecg_signal_index % DSP_BUFFERDIM) + DSP_BUFFERDIM] = ecg_dcfilter[ecg_signal_index % DSP_BUFFERDIM];
			}

			ecg_signal = ecg_dcfilter;
#endif



#ifdef DSP_LP_FILTER
			if (ecg_signal_index >= dsp_start + 12)
			{
				ecg_lowpass[ecg_signal_index % DSP_BUFFERDIM] = 	ecg_signal[ecg_signal_index % DSP_BUFFERDIM] LP_FILTER_GAIN
																	- 2 * ecg_signal[(ecg_signal_index - 6) % DSP_BUFFERDIM] LP_FILTER_GAIN
																	+ ecg_signal[(ecg_signal_index - 12) % DSP_BUFFERDIM] LP_FILTER_GAIN
																	+ 2 * ecg_lowpass[(ecg_signal_index - 1) % DSP_BUFFERDIM]
																	- ecg_lowpass[(ecg_signal_index - 2) % DSP_BUFFERDIM];

				ecg_lowpass[(ecg_signal_index % DSP_BUFFERDIM) + DSP_BUFFERDIM] = ecg_lowpass[ecg_signal_index % DSP_BUFFERDIM];
			}
			else if (ecg_signal_index >= dsp_start + 6)
			{
				ecg_lowpass[ecg_signal_index % DSP_BUFFERDIM] = 	ecg_signal[ecg_signal_index % DSP_BUFFERDIM] LP_FILTER_GAIN
																	- 2 * ecg_signal[(ecg_signal_index - 6) % DSP_BUFFERDIM] LP_FILTER_GAIN
																	+ 2 * ecg_lowpass[(ecg_signal_index - 1) % DSP_BUFFERDIM]
																	- ecg_lowpass[(ecg_signal_index - 2) % DSP_BUFFERDIM];

				ecg_lowpass[(ecg_signal_index % DSP_BUFFERDIM) + DSP_BUFFERDIM] = ecg_lowpass[ecg_signal_index % DSP_BUFFERDIM];
			}
			else if (ecg_signal_index >= dsp_start + 2)
			{
				ecg_lowpass[ecg_signal_index % DSP_BUFFERDIM] = 	ecg_signal[ecg_signal_index % DSP_BUFFERDIM] LP_FILTER_GAIN
																	+ 2 * ecg_lowpass[(ecg_signal_index - 1) % DSP_BUFFERDIM]
																	- ecg_lowpass[(ecg_signal_index - 2) % DSP_BUFFERDIM];

				ecg_lowpass[(ecg_signal_index % DSP_BUFFERDIM) + DSP_BUFFERDIM] = ecg_lowpass[ecg_signal_index % DSP_BUFFERDIM];
			}
			else if (ecg_signal_index >= dsp_start + 1)
			{
				ecg_lowpass[ecg_signal_index % DSP_BUFFERDIM] = 	ecg_signal[ecg_signal_index % DSP_BUFFERDIM] LP_FILTER_GAIN
																	+ 2 * ecg_lowpass[(ecg_signal_index - 1) % DSP_BUFFERDIM];

				ecg_lowpass[(ecg_signal_index % DSP_BUFFERDIM) + DSP_BUFFERDIM] = ecg_lowpass[ecg_signal_index % DSP_BUFFERDIM];
			}
			else if (ecg_signal_index >= dsp_start)
			{
				ecg_lowpass[ecg_signal_index % DSP_BUFFERDIM] = 	ecg_signal[ecg_signal_index % DSP_BUFFERDIM] LP_FILTER_GAIN;

				ecg_lowpass[(ecg_signal_index % DSP_BUFFERDIM) + DSP_BUFFERDIM] = ecg_lowpass[ecg_signal_index % DSP_BUFFERDIM];
			}

			ecg_signal = ecg_lowpass;
#endif



#ifdef DSP_HP_FILTER
			if (ecg_signal_index >= dsp_start + 32)
			{
				ecg_highpass[ecg_signal_index % DSP_BUFFERDIM] = 	- ecg_signal[ecg_signal_index % DSP_BUFFERDIM] HP_FILTER_GAIN
																	+ 32 * ecg_signal[(ecg_signal_index - 16) % DSP_BUFFERDIM] HP_FILTER_GAIN
																	+ ecg_signal[(ecg_signal_index - 32) % DSP_BUFFERDIM] HP_FILTER_GAIN
																	- ecg_highpass[(ecg_signal_index - 1) % DSP_BUFFERDIM];

				ecg_highpass[(ecg_signal_index % DSP_BUFFERDIM) + DSP_BUFFERDIM] = ecg_highpass[ecg_signal_index % DSP_BUFFERDIM];
			}
			else if (ecg_signal_index >= dsp_start + 16)
			{
				ecg_highpass[ecg_signal_index % DSP_BUFFERDIM] = 	- ecg_signal[ecg_signal_index % DSP_BUFFERDIM] HP_FILTER_GAIN
																	+ 32 * ecg_signal[(ecg_signal_index - 16) % DSP_BUFFERDIM] HP_FILTER_GAIN
																	- ecg_highpass[(ecg_signal_index - 1) % DSP_BUFFERDIM];

				ecg_highpass[(ecg_signal_index % DSP_BUFFERDIM) + DSP_BUFFERDIM] = ecg_highpass[ecg_signal_index % DSP_BUFFERDIM];
			}
			else if (ecg_signal_index >= dsp_start + 1)
			{
				ecg_highpass[ecg_signal_index % DSP_BUFFERDIM] = 	- ecg_signal[ecg_signal_index % DSP_BUFFERDIM] HP_FILTER_GAIN
																	- ecg_highpass[(ecg_signal_index - 1) % DSP_BUFFERDIM];

				ecg_highpass[(ecg_signal_index % DSP_BUFFERDIM) + DSP_BUFFERDIM] = ecg_highpass[ecg_signal_index % DSP_BUFFERDIM];
			}
			else if (ecg_signal_index >= dsp_start)
			{
				ecg_highpass[ecg_signal_index % DSP_BUFFERDIM] = 	- ecg_signal[ecg_signal_index % DSP_BUFFERDIM] HP_FILTER_GAIN;

				ecg_highpass[(ecg_signal_index % DSP_BUFFERDIM) + DSP_BUFFERDIM] = ecg_highpass[ecg_signal_index % DSP_BUFFERDIM];
			}

			ecg_signal = ecg_highpass;
#endif

#ifdef DSP_DERIVATIVE
			if (ecg_signal_index >= dsp_start + 4)
			{
				ecg_derivative[ecg_signal_index % DSP_BUFFERDIM] = 	- ecg_signal[(ecg_signal_index - 4) % DSP_BUFFERDIM] DERIVATIVE_GAIN
																	- 2 * ecg_signal[(ecg_signal_index - 3) % DSP_BUFFERDIM] DERIVATIVE_GAIN
																	+ 2 * ecg_signal[(ecg_signal_index - 1) % DSP_BUFFERDIM] DERIVATIVE_GAIN
																	+ ecg_signal[ecg_signal_index % DSP_BUFFERDIM] DERIVATIVE_GAIN;

				ecg_derivative[(ecg_signal_index % DSP_BUFFERDIM) + DSP_BUFFERDIM] = ecg_derivative[ecg_signal_index % DSP_BUFFERDIM];
			}
			else if (ecg_signal_index >= dsp_start + 3)
			{
				ecg_derivative[ecg_signal_index % DSP_BUFFERDIM] = 	- 2 * ecg_signal[(ecg_signal_index - 3) % DSP_BUFFERDIM] DERIVATIVE_GAIN
																	+ 2 * ecg_signal[(ecg_signal_index - 1) % DSP_BUFFERDIM] DERIVATIVE_GAIN
																	+ ecg_signal[ecg_signal_index % DSP_BUFFERDIM] DERIVATIVE_GAIN;

				ecg_derivative[(ecg_signal_index % DSP_BUFFERDIM) + DSP_BUFFERDIM] = ecg_derivative[ecg_signal_index % DSP_BUFFERDIM];
			}
			else if (ecg_signal_index >= dsp_start + 1)
			{
				ecg_derivative[ecg_signal_index % DSP_BUFFERDIM] = 	2 * ecg_signal[(ecg_signal_index - 1) % DSP_BUFFERDIM] DERIVATIVE_GAIN
																	+ ecg_signal[ecg_signal_index % DSP_BUFFERDIM] DERIVATIVE_GAIN;

				ecg_derivative[(ecg_signal_index % DSP_BUFFERDIM) + DSP_BUFFERDIM] = ecg_derivative[ecg_signal_index % DSP_BUFFERDIM];
			}
			else if (ecg_signal_index >= dsp_start)
			{
				ecg_derivative[ecg_signal_index % DSP_BUFFERDIM] = 	ecg_signal[ecg_signal_index % DSP_BUFFERDIM] DERIVATIVE_GAIN;

				ecg_derivative[(ecg_signal_index % DSP_BUFFERDIM) + DSP_BUFFERDIM] = ecg_derivative[ecg_signal_index % DSP_BUFFERDIM];
			}

			ecg_signal = ecg_derivative;
#endif

#ifdef DSP_SQUARED
			if (ecg_signal_index >= dsp_start)
			{
				ecg_squared[ecg_signal_index % DSP_BUFFERDIM] = 	ecg_signal[ecg_signal_index % DSP_BUFFERDIM] * ecg_signal[ecg_signal_index % DSP_BUFFERDIM];

				ecg_squared[(ecg_signal_index % DSP_BUFFERDIM) + DSP_BUFFERDIM] = ecg_squared[ecg_signal_index % DSP_BUFFERDIM];
			}

			ecg_signal = ecg_squared;
#endif

#ifdef DSP_INTEGRAL
			if (ecg_signal_index >= dsp_start)
			{
				ecg_integral[ecg_signal_index % DSP_BUFFERDIM] = 0;
				for (ecg_integral_index = 0; ecg_integral_index < INTEGRAL_WINDOW; ecg_integral_index++)
				{
					if (ecg_signal_index >= dsp_start + ecg_integral_index)
					{
						ecg_integral[ecg_signal_index % DSP_BUFFERDIM] += ecg_signal[(ecg_signal_index - ecg_integral_index) % DSP_BUFFERDIM];
					}
					else break;
				}
				ecg_integral[ecg_signal_index % DSP_BUFFERDIM] /= ecg_integral_index;

				ecg_integral[(ecg_signal_index % DSP_BUFFERDIM) + DSP_BUFFERDIM] = ecg_integral[ecg_signal_index % DSP_BUFFERDIM];
			}

			ecg_signal = ecg_integral;
#endif

			ecg_signal_index++;


			vPortFree(rptr->data);
			vPortFree(rptr);
		}
	}

	for (;;)
	{
		evt = osMessageGet(sensorSetup[sensorId].proc1_queueId, osWaitForever);

		if (evt.status == osEventMessage)
		{
			rptr = (SensorsData *) evt.value.p;



//			timer_0 = timer_get();

			cnn_rawSignal[(ecg_signal_index % DSP_BUFFERDIM)] = (*((int16_t*) rptr->data) >> 8);
			cnn_rawSignal[(ecg_signal_index % DSP_BUFFERDIM) + DSP_BUFFERDIM] = (*((int16_t*) rptr->data) >> 8);

			ecg_rawSignal[(ecg_signal_index % DSP_BUFFERDIM)] = *((int16_t*) rptr->data);
			ecg_rawSignal[(ecg_signal_index % DSP_BUFFERDIM) + DSP_BUFFERDIM] = *((int16_t*) rptr->data);
			ecg_signal = ecg_rawSignal;



#ifdef DSP_DC_FILTER
			// DC FILTER
			// out(n) = in(n) -in(n-1) + 0.995*out(n-1)
			ecg_dcfilter[ecg_signal_index % DSP_BUFFERDIM] =   	ecg_signal[ecg_signal_index % DSP_BUFFERDIM]
															    - ecg_signal[(ecg_signal_index - 1) % DSP_BUFFERDIM]
																+ 0.995 * ecg_dcfilter[(ecg_signal_index - 1) % DSP_BUFFERDIM];

			ecg_dcfilter[(ecg_signal_index % DSP_BUFFERDIM) + DSP_BUFFERDIM] = ecg_dcfilter[ecg_signal_index % DSP_BUFFERDIM];

			ecg_signal = ecg_dcfilter;
#endif

#ifdef DSP_LP_FILTER
			// LOW PASS FILTER
			// out(n) = 1/36[in(n) - 2in(n - 6) + in(n - 12) + 2out(n - 1)] - out(n - 2)
			ecg_lowpass[ecg_signal_index % DSP_BUFFERDIM] = 	ecg_signal[ecg_signal_index % DSP_BUFFERDIM] LP_FILTER_GAIN
																- 2 * ecg_signal[(ecg_signal_index - 6) % DSP_BUFFERDIM] LP_FILTER_GAIN
																+ ecg_signal[(ecg_signal_index - 12) % DSP_BUFFERDIM] LP_FILTER_GAIN
																+ 2 * ecg_lowpass[(ecg_signal_index - 1) % DSP_BUFFERDIM]
																- ecg_lowpass[(ecg_signal_index - 2) % DSP_BUFFERDIM];

			ecg_lowpass[(ecg_signal_index % DSP_BUFFERDIM) + DSP_BUFFERDIM] = ecg_lowpass[ecg_signal_index % DSP_BUFFERDIM];

			ecg_signal = ecg_lowpass;
#endif

#ifdef DSP_HP_FILTER
			// HIGH PASS FILTER
			// out(n) = 1/32[in(n) + 32in(n - 16) + in(n - 32)] + out(n - 1)
			ecg_highpass[ecg_signal_index % DSP_BUFFERDIM] = 	- ecg_signal[ecg_signal_index % DSP_BUFFERDIM] HP_FILTER_GAIN
																+ 32 * ecg_signal[(ecg_signal_index - 16) % DSP_BUFFERDIM] HP_FILTER_GAIN
																+ ecg_signal[(ecg_signal_index - 32) % DSP_BUFFERDIM] HP_FILTER_GAIN
																- ecg_highpass[(ecg_signal_index - 1) % DSP_BUFFERDIM];

			ecg_highpass[(ecg_signal_index % DSP_BUFFERDIM) + DSP_BUFFERDIM] = ecg_highpass[ecg_signal_index % DSP_BUFFERDIM];

			ecg_signal = ecg_highpass;
#endif

#ifdef DSP_DERIVATIVE
			// DERIVATIVE
			// out(n) = 1/8[in(n) + 2in(n - 1) - 2in(n - 3)- in(n - 4)]
			ecg_derivative[ecg_signal_index % DSP_BUFFERDIM] = 	- ecg_signal[(ecg_signal_index - 4) % DSP_BUFFERDIM] DERIVATIVE_GAIN
																- 2 * ecg_signal[(ecg_signal_index - 3) % DSP_BUFFERDIM] DERIVATIVE_GAIN
																+ 2 * ecg_signal[(ecg_signal_index - 1) % DSP_BUFFERDIM] DERIVATIVE_GAIN
																+ ecg_signal[ecg_signal_index % DSP_BUFFERDIM] DERIVATIVE_GAIN;

			ecg_derivative[(ecg_signal_index % DSP_BUFFERDIM) + DSP_BUFFERDIM] = ecg_derivative[ecg_signal_index % DSP_BUFFERDIM];

			ecg_signal = ecg_derivative;
#endif

#ifdef DSP_SQUARED
			// SQUARED
			// out(n) = in(n) * in(n)
			if (ecg_signal_index >= dsp_start)
			{
				ecg_squared[ecg_signal_index % DSP_BUFFERDIM] = 	ecg_signal[ecg_signal_index % DSP_BUFFERDIM] * ecg_signal[ecg_signal_index % DSP_BUFFERDIM];

				ecg_squared[(ecg_signal_index % DSP_BUFFERDIM) + DSP_BUFFERDIM] = ecg_squared[ecg_signal_index % DSP_BUFFERDIM];
			}

			ecg_signal = ecg_squared;
#endif

#ifdef DSP_INTEGRAL
			// INTEGRAL
			// out(n) = 20[in(n) + in(n - 1) + in(n - 2) + ... + in(n - 17) + in(n - 18) + in(n - 19)]
			if (ecg_signal_index >= dsp_start)
			{
				ecg_integral[ecg_signal_index % DSP_BUFFERDIM] = 0;
				for (ecg_integral_index = 0; ecg_integral_index < INTEGRAL_WINDOW; ecg_integral_index++)
				{
					ecg_integral[ecg_signal_index % DSP_BUFFERDIM] += ecg_signal[(ecg_signal_index - ecg_integral_index) % DSP_BUFFERDIM];
				}
				ecg_integral[ecg_signal_index % DSP_BUFFERDIM] = ecg_integral[ecg_signal_index % DSP_BUFFERDIM] INTEGRAL_GAIN;

				ecg_integral[(ecg_signal_index % DSP_BUFFERDIM) + DSP_BUFFERDIM] = ecg_integral[ecg_signal_index % DSP_BUFFERDIM];
			}

			ecg_signal = ecg_integral;
#endif



#ifdef DSP_ALGORITHM_1
			switch (ecg_state & 0b11)
			{
				case (0):
					if(ecg_signal[ecg_signal_index % DSP_BUFFERDIM] >= SIGNAL_RISE_THRESHOLD) ecg_state++;
				break;

				case (1):
					LED_ON;
					if(ecg_signal[ecg_signal_index % DSP_BUFFERDIM] < ecg_signal[(ecg_signal_index - 1) % DSP_BUFFERDIM]) ecg_state++;
				break;

				case (2):
					if(ecg_signal[ecg_signal_index % DSP_BUFFERDIM] > ecg_signal[(ecg_signal_index - 1) % DSP_BUFFERDIM])
					{
						*((int16_t*) sensorSetup[SENSOR_0].last_value->data) = ecg_rate / ((float) ((ecg_signal_index - 1 - ecg_signal_delay) - ecg_rawSignal_rPeak));
						sensorSetup[SENSOR_0].last_value->ms_counter = osKernelSysTick();
						ecg_rawSignal_rPeak = ecg_signal_index - 1 - ecg_signal_delay;
						LED_OFF;

						if(sensorSetup[sensorId].out_proc1_queueId != NULL)
						{
							mptr = 					(SensorsData *) pvPortMalloc(sizeof(SensorsData));
							mptr->data_len = 		sensorSetup[sensorId].data_len;
							mptr->data = 			cnn_rawSignal + ((ecg_rawSignal_rPeak - CNN_HALFWINDOWDIM) % DSP_BUFFERDIM);
//							delayedOsMessagePut((ECG_SAMP_PERIOD_VALUE * CNN_HALFWINDOWDIM) - ecg_signal_delay, sensorId, sensorSetup[sensorId].out_proc1_queueId, mptr);
							if (osMessagePut(sensorSetup[sensorId].out_proc1_queueId, (uint32_t) mptr, osWaitForever) != osOK) Error_Handler();
						}

						ecg_state++;
					}
				break;

				case (3):
					if((ecg_signal_index - signal_peak_delay) == ecg_rawSignal_rPeak) ecg_state++;
				break;

				default:
					while(1);
				break;
			}
#endif

#ifdef DSP_ALGORITHM_OLD
			switch (ecg_state & 0b11)
			{
				case (0):
					if(((*((int16_t*) rptr->data) - ecg_prev_data) > ecg_delta) && ((ecg_prev_count - ecg_count_peak) > ecg_debounce)) ecg_state++;
				break;

				case (1):
					LED_ON;
					if(*((int16_t*) rptr->data) < ecg_prev_data)
					{
						ecg_heartbeat = ecg_rate/((ecg_prev_count - 1) - ecg_count_peak);
						ecg_count_peak = ecg_prev_count - 1;
						ecg_rawSignal_rPeak = ecg_signal_index;
						ecg_state++;
					}
				break;

				case (2):
					*((int16_t*) sensorSetup[sensorId].last_value->data) = 	ecg_heartbeat;
					sensorSetup[sensorId].last_value->ms_counter = 			osKernelSysTick();
					sensorSetup[sensorId].last_value->message_id = 			READ_LAST_VALUE;

//					STLBLE_PRINTF("\r\nHB: %d", ecg_heartbeat);

					mptr = 					(SensorsData *) pvPortMalloc(sizeof(SensorsData) * 1);
					mptr->data_len = 		sensorSetup[sensorId].sample_packing;
					mptr->data = 			cnn_rawSignal + ((ecg_rawSignal_rPeak - CNN_HALFWINDOWDIM) % DSP_BUFFERDIM);

//					STLBLE_PRINTF("\r\nA %x", mptr->data);

					delayedOsMessagePut(ECG_SAMP_PERIOD_VALUE * CNN_HALFWINDOWDIM, sensorId, sensorSetup[sensorId].out_proc1_queueId, mptr);



//					mptr = (SensorsData *) pvPortMalloc(sizeof(SensorsData) * 1);
//					mptr->data = dataMalloc(sensorSetup[sensorId].data_type, mptr->data_len);
//					if (osMessagePut(sensorSetup[sensorId].out_proc1_queueId, (uint32_t) mptr, osWaitForever) != osOK) Error_Handler();

					ecg_state++;
				break;

				case (3):
					if((*((int16_t*) rptr->data) - ecg_prev_data) > 0)
					{
						ecg_state++;
						LED_OFF;
					}
				break;

				default:
					while(1);
				break;
			}
#endif

			ecg_signal_index++;
			ecg_prev_data = *((int16_t*) rptr->data);


			vPortFree(rptr->data);
			vPortFree(rptr);
		}

//		timer_1 = timer_get();
//
//		STLBLE_PRINTF("\rPEAK time:   %d         ", timer_1 - timer_0 - 1);
	}
}

static void ecg_cnn_classifier(int sensorId)
{
	osEvent evt;
	SensorsData *rptr;
	SensorsData *mptr;



//	#define SIMPLETEST

#ifdef SIMPLETEST
	/*
	 * 	--- EXAMPLE: CONV2D LAYER ---
	 *
	 * 	IN_DIM = 	10
	 * 	OUT_DIM = 	7
	 * 	WT_DIM = 	4
	 * 	IF = 		2
	 * 	OF = 		2
	 *
	 *	IF1	IF2		K1_IF1	K1_IF2	K2_IF1	K2_IF2		B1	B2		OF1		OF2
	 * 	0	0		1		10		2		20			0	0		0606	1212
	 * 	1	10		1		10		2		20						1010	2020
	 * 	2	20		1		10		2		20						1414	2828
	 * 	3	30		1		10		2		20						1818	3636
	 * 	4	40														2222	4444
	 * 	5	50														2626	5252
	 * 	6	60														3030	6060
	 * 	7	70
	 * 	8	80
	 * 	9	90
	 *
	 *
	 *
	 * 	--- CMSIS INPUT ---
	 *
	 *	input	[IN_DIM * IF] = 		{0,	 0,  1, 10,  2, 20,  3, 30,  4,  40,  5, 50,  6, 60,  7, 70,  8, 80,  9, 90};
	 * 	weights	[WT_DIM * IF * OF] = 	{1, 10,  1, 10,  1, 10,  1, 10,  2,  20,  2, 20,  2, 20,  2, 20};
	 * 	bias	[OF] = 					{0};
	 * 	output	[OUT_DIM * OF] = 		{0};
	 * 	stride = 						1
	 * 	padding = 						0
	 * 	bias_shift = 					0
	 * 	output_shift = 					0
	 *
	 *
	 *
	 * 	--- CMSIS OUTPUT ---
	 *
	 * 	output = {606, 1212, 1010, 2020, 1414, 2828, 1818, 3636, 2222, 4444, 2626, 5252, 3030, 6060}
	 */



	#define TEST__INDIM			10
	#define TEST__WTDIM			4
	#define TEST__IF			2
	#define TEST__OF			2
	#define TEST__STRIDE		1
	#define	TEST__PADDING		0
	#define TEST__OUTDIM		(TEST__INDIM-TEST__WTDIM+1)	// with stride equal to 1 and padding equal to zero
	#define TEST__OUTRSHIFT		0
	#define TEST__BIASLSHIFT	0


	q7_t 	test__data	[TEST__INDIM*TEST__IF] = 			{0, 0, 1, 10, 2, 20, 3, 30, 4, 40, 5, 50, 6, 60, 7, 70, 8, 80, 9, 90};
	q7_t 	test__wt	[TEST__WTDIM*TEST__IF*TEST__OF] = 	{1, 10, 1, 10, 1, 10, 1, 10, 2, 20, 2, 20, 2, 20, 2, 20};
	q7_t 	test__bias	[TEST__OF] = 						{0};
	int32_t out__test	[TEST__OUTDIM*TEST__OF] = 			{0};

	q7_t* test__buffer;
	q7_t* test__col_buffer;
	q7_t test__scratch_pad[TEST__OF*TEST__INDIM + 2*TEST__IF*TEST__WTDIM + TEST__INDIM*TEST__IF] = {0};

	test__buffer = test__scratch_pad;
	test__col_buffer = test__buffer + TEST__OF*TEST__INDIM;

	arm_convolve_HWC_q7_basic_nonsquare_o32(				(q7_t*)test__data,
															TEST__INDIM, 1,
															TEST__IF,
															test__wt,
															TEST__OF,
															TEST__WTDIM, 1,
															TEST__PADDING, 0,
															TEST__STRIDE, 1,
															test__bias,
															TEST__BIASLSHIFT,
															TEST__OUTRSHIFT,
															test__buffer,
															TEST__OUTDIM, 1,
															(q15_t*)test__col_buffer, NULL,
															out__test);

	for(int i = 0; i < TEST__OUTDIM*TEST__OF; i+=TEST__OF)
	{
		for(int j = 0; j < TEST__OF; j++)
		{
			STLBLE_PRINTF("%d, ", out__test[i+j]);
		}
		STLBLE_PRINTF("\r\n");
	}

	while(1);
#endif



#ifdef OUTPUT_32BIT
	q31_t 		buffer_o32[CONV1_OF*CONV1_OUTDIM];
#endif

	for (;;)
	{
		evt = osMessageGet(sensorSetup[sensorId].proc2_queueId, osWaitForever);
		LED_ON;

		if (evt.status == osEventMessage)
		{

			rptr = (SensorsData *) evt.value.p;
			input = (q7_t*) rptr->data;



#ifdef LOG_PRINT
			timer_0 = timer_get();
#endif

#ifdef OUTPUT_SHIFT_OLD
			conv1_input = input; conv1_output = cnn_a_buffer;
			arm_convolve_HWC_q7_basic_nonsquare(conv1_input, CONV1_INDIM, 1, CONV1_IF, conv1_wt, CONV1_OF, CONV1_KDIM, 1, CONV1_PADDING, 0, CONV1_STRIDE, 1, conv1_bias, CONV1_BIAS_LSHIFT, CONV1_OUT_RSHIFT, conv1_output, CONV1_OUTDIM, 1, col_buffer, NULL);

			relu1_inout = cnn_a_buffer;
			arm_relu_q7(relu1_inout, RELU1_INDIM);

			maxpool1_inout = cnn_a_buffer;
			arm_maxpool_q7_HWC_1D(maxpool1_inout, MAXPOOL1_INDIM, MAXPOOL1_IF, MAXPOOL1_KDIM, MAXPOOL1_PADDING, MAXPOOL1_STRIDE, MAXPOOL1_OUTDIM);

			conv2_input = cnn_a_buffer; conv2_output = cnn_b_buffer;
			arm_convolve_HWC_q7_basic_nonsquare(conv2_input, CONV2_INDIM, 1, CONV2_IF, conv2_wt, CONV2_OF, CONV2_KDIM, 1, CONV2_PADDING, 0, CONV2_STRIDE, 1, conv2_bias, CONV2_BIAS_LSHIFT, CONV2_OUT_RSHIFT, conv2_output, CONV2_OUTDIM, 1, col_buffer, NULL);

			relu2_inout = cnn_b_buffer;
			arm_relu_q7(relu2_inout, RELU2_INDIM);

			maxpool2_inout = cnn_b_buffer;
			arm_maxpool_q7_HWC_1D(maxpool2_inout, MAXPOOL2_INDIM, MAXPOOL2_IF, MAXPOOL2_KDIM, MAXPOOL2_PADDING, MAXPOOL2_STRIDE, MAXPOOL2_OUTDIM);

			fc1_input = cnn_b_buffer; fc1_output = cnn_a_buffer;
			arm_fully_connected_q7(fc1_input, fc1_wt, FC1_INDIM, FC1_OUTDIM, FC1_BIAS_LSHIFT, FC1_OUT_RSHIFT, fc1_bias, fc1_output, col_buffer);

			relu3_inout = cnn_a_buffer;
			arm_relu_q7(relu3_inout, RELU3_INDIM);

			fc2_input = cnn_a_buffer; fc2_output = cnn_b_buffer;
			arm_fully_connected_q7(fc2_input, fc2_wt, FC2_INDIM, FC2_OUTDIM, FC2_BIAS_LSHIFT, FC2_OUT_RSHIFT, fc2_bias, fc2_output, col_buffer);
#endif



			conv1_input = input; conv1_output = cnn_a_buffer;
			arm_convolve_HWC_q7_basic_nonsquare_div(conv1_input, CONV1_INDIM, 1, CONV1_IF, conv1_wt, CONV1_OF, CONV1_KDIM, 1, CONV1_PADDING, 0, CONV1_STRIDE, 1, conv1_bias, CONV1_BIAS_LSHIFT, CONV1_OUT_SCALE, conv1_output, CONV1_OUTDIM, 1, col_buffer, NULL);

			timer_1 = timer_get();

			relu1_inout = cnn_a_buffer;
			arm_relu_q7(relu1_inout, RELU1_INDIM);

			timer_2 = timer_get();

			maxpool1_inout = cnn_a_buffer;
			arm_maxpool_q7_HWC_1D(maxpool1_inout, MAXPOOL1_INDIM, MAXPOOL1_IF, MAXPOOL1_KDIM, MAXPOOL1_PADDING, MAXPOOL1_STRIDE, MAXPOOL1_OUTDIM);

			timer_3 = timer_get();

			conv2_input = cnn_a_buffer; conv2_output = cnn_b_buffer;

#if defined(CONV2_BASIC) && defined(OUTPUT_SHIFT)
			arm_convolve_HWC_q7_basic_nonsquare(conv2_input, CONV2_INDIM, 1, CONV2_IF, conv2_wt, CONV2_OF, CONV2_KDIM, 1, CONV2_PADDING, 0, CONV2_STRIDE, 1, conv2_bias, CONV2_BIAS_LSHIFT, CONV2_OUT_RSHIFT, conv2_output, CONV2_OUTDIM, 1, col_buffer, NULL);
#endif
#if defined(CONV2_BASIC) && !defined(OUTPUT_SHIFT)
			arm_convolve_HWC_q7_basic_nonsquare_div(conv2_input, CONV2_INDIM, 1, CONV2_IF, conv2_wt, CONV2_OF, CONV2_KDIM, 1, CONV2_PADDING, 0, CONV2_STRIDE, 1, conv2_bias, CONV2_BIAS_LSHIFT, CONV2_OUT_SCALE, conv2_output, CONV2_OUTDIM, 1, col_buffer, NULL);
#endif
#if !defined(CONV2_BASIC) && defined(OUTPUT_SHIFT)
			arm_convolve_HWC_q7_fast_nonsquare(conv2_input, CONV2_INDIM, 1, CONV2_IF, conv2_wt, CONV2_OF, CONV2_KDIM, 1, CONV2_PADDING, 0, CONV2_STRIDE, 1, conv2_bias, CONV2_BIAS_LSHIFT, CONV2_OUT_SCALE, conv2_output, CONV2_OUTDIM, 1, col_buffer, NULL);
#endif
#if !defined(CONV2_BASIC) && !defined(OUTPUT_SHIFT)
			arm_convolve_HWC_q7_fast_nonsquare_div(conv2_input, CONV2_INDIM, 1, CONV2_IF, conv2_wt, CONV2_OF, CONV2_KDIM, 1, CONV2_PADDING, 0, CONV2_STRIDE, 1, conv2_bias, CONV2_BIAS_LSHIFT, CONV2_OUT_RSHIFT, conv2_output, CONV2_OUTDIM, 1, col_buffer, NULL);
#endif

			timer_4 = timer_get();

			relu2_inout = cnn_b_buffer;
			arm_relu_q7(relu2_inout, RELU2_INDIM);

			timer_5 = timer_get();

			maxpool2_inout = cnn_b_buffer;
			arm_maxpool_q7_HWC_1D(maxpool2_inout, MAXPOOL2_INDIM, MAXPOOL2_IF, MAXPOOL2_KDIM, MAXPOOL2_PADDING, MAXPOOL2_STRIDE, MAXPOOL2_OUTDIM);

			timer_6 = timer_get();

			fc1_input = cnn_b_buffer; fc1_output = cnn_a_buffer;
#if defined(FULLY1_BASIC) && defined(OUTPUT_SHIFT)
			arm_fully_connected_q7(fc1_input, fc1_wt, FC1_INDIM, FC1_OUTDIM, FC1_BIAS_LSHIFT, FC1_OUT_RSHIFT, fc1_bias, fc1_output, col_buffer);
#endif
#if defined(FULLY1_BASIC) && !defined(OUTPUT_SHIFT)
			arm_fully_connected_q7_div(fc1_input, fc1_wt, FC1_INDIM, FC1_OUTDIM, FC1_BIAS_LSHIFT, FC1_OUT_SCALE, fc1_bias, fc1_output, col_buffer);
#endif
#if !defined(FULLY1_BASIC) && defined(OUTPUT_SHIFT)
			arm_fully_connected_q7_opt(fc1_input, fc1_wt, FC1_INDIM, FC1_OUTDIM, FC1_BIAS_LSHIFT, FC1_OUT_RSHIFT, fc1_bias, fc1_output, col_buffer);
#endif
#if !defined(FULLY1_BASIC) && !defined(OUTPUT_SHIFT)
			arm_fully_connected_q7_opt_div(fc1_input, fc1_wt, FC1_INDIM, FC1_OUTDIM, FC1_BIAS_LSHIFT, FC1_OUT_SCALE, fc1_bias, fc1_output, col_buffer);
#endif

			timer_7 = timer_get();

			relu3_inout = cnn_a_buffer;
			arm_relu_q7(relu3_inout, RELU3_INDIM);

			timer_8 = timer_get();

			fc2_input = cnn_a_buffer; fc2_output = cnn_b_buffer;
#if defined(FULLY2_BASIC) && defined(OUTPUT_SHIFT)
			arm_fully_connected_q7(fc2_input, fc2_wt, FC2_INDIM, FC2_OUTDIM, FC2_BIAS_LSHIFT, FC2_OUT_RSHIFT, fc2_bias, fc2_output, col_buffer);
#endif
#if defined(FULLY2_BASIC) && !defined(OUTPUT_SHIFT)
			arm_fully_connected_q7_div(fc2_input, fc2_wt, FC2_INDIM, FC2_OUTDIM, FC2_BIAS_LSHIFT, FC2_OUT_SCALE, fc2_bias, fc2_output, col_buffer);
#endif
#if !defined(FULLY2_BASIC) && defined(OUTPUT_SHIFT)
			arm_fully_connected_q7_opt(fc2_input, fc2_wt, FC2_INDIM, FC2_OUTDIM, FC2_BIAS_LSHIFT, FC2_OUT_RSHIFT, fc2_bias, fc2_output, col_buffer);
#endif
#if !defined(FULLY2_BASIC) && !defined(OUTPUT_SHIFT)
			arm_fully_connected_q7_opt_div(fc2_input, fc2_wt, FC2_INDIM, FC2_OUTDIM, FC2_BIAS_LSHIFT, FC2_OUT_SCALE, fc2_bias, fc2_output, col_buffer);
#endif

			timer_9 = timer_get();



#ifdef OUTPUT_32BIT_OLD
			conv1_input = input; conv1_output = cnn_a_buffer; conv1_output_o32 = buffer_o32;
			arm_convolve_HWC_q7_basic_nonsquare_o32(conv1_input, CONV1_INDIM, 1, CONV1_IF, conv1_wt, CONV1_OF, CONV1_KDIM, 1, CONV1_PADDING, 0, CONV1_STRIDE, 1, conv1_bias, CONV1_BIAS_LSHIFT, 0, conv1_output, CONV1_OUTDIM, 1, col_buffer, NULL, conv1_output_o32);
			for(int i = 0; i < CONV1_OF*CONV1_OUTDIM; i++) conv1_output[i] = round(conv1_output_o32[i]/CONV1_OUT_SCALE);

			relu1_inout = cnn_a_buffer;
			arm_relu_q7(relu1_inout, RELU1_INDIM);

			maxpool1_inout = cnn_a_buffer;
			arm_maxpool_q7_HWC_1D(maxpool1_inout, MAXPOOL1_INDIM, MAXPOOL1_IF, MAXPOOL1_KDIM, MAXPOOL1_PADDING, MAXPOOL1_STRIDE, MAXPOOL1_OUTDIM);

			conv2_input = cnn_a_buffer; conv2_output = cnn_b_buffer; conv2_output_o32 = buffer_o32;
			arm_convolve_HWC_q7_basic_nonsquare_o32(conv2_input, CONV2_INDIM, 1, CONV2_IF, conv2_wt, CONV2_OF, CONV2_KDIM, 1, CONV2_PADDING, 0, CONV2_STRIDE, 1, conv2_bias, CONV2_BIAS_LSHIFT, 0, conv2_output, CONV2_OUTDIM, 1, col_buffer, NULL, conv2_output_o32);
			for(int i = 0; i < CONV2_OF*CONV2_OUTDIM; i++) conv2_output[i] = round(conv2_output_o32[i]/CONV2_OUT_SCALE);

			relu2_inout = cnn_b_buffer;
			arm_relu_q7(relu2_inout, RELU2_INDIM);

			maxpool2_inout = cnn_b_buffer;
			arm_maxpool_q7_HWC_1D(maxpool2_inout, MAXPOOL2_INDIM,MAXPOOL2_IF,MAXPOOL2_KDIM,MAXPOOL2_PADDING,MAXPOOL2_STRIDE,MAXPOOL2_OUTDIM);

			fc1_input = cnn_b_buffer; fc1_output = cnn_a_buffer; fc1_output_o32 = buffer_o32;
			arm_fully_connected_q7_o32(fc1_input, fc1_wt, FC1_INDIM, FC1_OUTDIM, FC1_BIAS_LSHIFT, FC1_OUT_RSHIFT, fc1_bias, fc1_output, col_buffer, fc1_output_o32);
			for(int i = 0; i < FC1_OUTDIM; i++) fc1_output[i] = round(fc1_output_o32[i]/FC1_OUT_SCALE);

			relu3_inout = cnn_a_buffer;
			arm_relu_q7(relu3_inout, RELU3_INDIM);

			fc2_input = cnn_a_buffer; fc2_output = cnn_b_buffer; fc2_output_o32 = buffer_o32;
			arm_fully_connected_q7_o32(fc1_output, fc2_wt, FC2_INDIM, FC2_OUTDIM, FC2_BIAS_LSHIFT, FC2_OUT_RSHIFT, fc2_bias, fc2_output, col_buffer, fc2_output_o32);
			for(int i = 0; i < FC2_OUTDIM; i++) fc2_output[i] = round(fc2_output_o32[i]/FC2_OUT_SCALE);
#endif

#ifdef LOG_PRINT
			STLBLE_PRINTF("\r\nCONV1 time:   %d", timer_1 - timer_0 - 1);
			STLBLE_PRINTF("\r\nRELU1 time:   %d", timer_2 - timer_1 - 1);
			STLBLE_PRINTF("\r\nPOOL1 time:   %d", timer_3 - timer_2 - 1);
			STLBLE_PRINTF("\r\nCONV2 time:   %d", timer_4 - timer_3 - 1);
			STLBLE_PRINTF("\r\nRELU2 time:   %d", timer_5 - timer_4 - 1);
			STLBLE_PRINTF("\r\nPOOL2 time:   %d", timer_6 - timer_5 - 1);
			STLBLE_PRINTF("\r\nFULL1 time:   %d", timer_7 - timer_6 - 1);
			STLBLE_PRINTF("\r\nRELU3 time:   %d", timer_8 - timer_7 - 1);
			STLBLE_PRINTF("\r\nFULL2 time:   %d", timer_9 - timer_8 - 1);

			STLBLE_PRINTF("\r\n\nDIV NN time:   %d", timer_9 - timer_0 - 9);
			STLBLE_PRINTF("\r\n\nWorkload:   %d", workLoad);
#endif

			fc2_output_maxindex = 0;
			fc2_output_max = fc2_output[fc2_output_maxindex];

			for (int i = 1; i < FC2_OUTDIM; ++i)
			{
				if (fc2_output[i] > fc2_output_max)
				{
					fc2_output_max = fc2_output[i];
					fc2_output_maxindex = i;
				}
			}

#ifdef LOG_PRINT
			STLBLE_PRINTF("\r\nCLASS: %c\r\n", classes[fc2_output_maxindex]);
#endif

			mptr = 					(SensorsData *) pvPortMalloc(sizeof(SensorsData));
			mptr->data_len = 		1;
			mptr->data = 			dataMalloc(sensorSetup[sensorId].data_type, mptr->data_len);
			mptr->ms_counter =		osKernelSysTick();
			mptr->message_id =		READ_FROM_FIFO;
			*((int16_t*) mptr->data) = fc2_output_maxindex;

			if (osMessagePut(sensorSetup[sensorId].out_proc2_queueId, (uint32_t) mptr, osWaitForever) != osOK) Error_Handler();

			vPortFree(rptr);

		}

		LED_OFF;
	}
}

int ecg_data_threshold(SensorsData *data)
{
	return SEND;
//	return DO_NOT_SEND;
}



void samp_timer_0_Callback(void const *arg) { osSemaphoreRelease(sensorSetup[SENSOR_0].get_semId); }

void send_timer_0_Callback(void const *arg) { if (osMessagePut(sensorSetup[SENSOR_0].thrsh_queueId, (uint32_t) sensorSetup[SENSOR_0].last_value, osWaitForever) != osOK) Error_Handler(); }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////






////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// SENSOR_1: FUNCTIONS & TIMER
//
//void temp_read_from_sensor(SensorsData *packet)
//{
//	packet->data_len = 	1;
//	*((float*) packet->data) = 36.5;
//}
//
//int temp_data_threshold(SensorsData *data) { return SEND; }
//
//
//
//void samp_timer_1_Callback(void const *arg) { osSemaphoreRelease(sensorSetup[SENSOR_1].get_semId); }
//
//void send_timer_1_Callback(void const *arg) { if (osMessagePut(sensorSetup[SENSOR_1].thrsh_queueId, (uint32_t) sensorSetup[SENSOR_1].last_value, osWaitForever) != osOK) Error_Handler(); }
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////






////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// SENSOR_2: FUNCTIONS
//
//void pres_read_from_sensor(SensorsData *packet)
//{
//	packet->data_len = 	1;
//	*((float*) packet->data) = 107.011;
//}
//
//int pres_data_threshold(SensorsData *data) { return SEND; }
//
//
//
//void samp_timer_2_Callback(void const *arg) { osSemaphoreRelease(sensorSetup[SENSOR_2].get_semId); }
//
//void send_timer_2_Callback(void const *arg) { if (osMessagePut(sensorSetup[SENSOR_2].thrsh_queueId, (uint32_t) sensorSetup[SENSOR_2].last_value, osWaitForever) != osOK) Error_Handler(); }
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////









void init_all_task()
{

	int *message;

	message = (int *) osPoolAlloc(thread_poolId);
	*message = TASK_SETUP;

	if (osMessagePut(master_queueId, (uint32_t) message, osWaitForever) != osOK) Error_Handler();
}

void *dataMalloc(int type, int size)
{
	switch (type)
	{
	case INT8:
		return pvPortMalloc(sizeof(int8_t) * size);

	case INT16:
		return pvPortMalloc(sizeof(int16_t) * size);

	case INT32:
		return pvPortMalloc(sizeof(int32_t) * size);

	case FLOAT:
		return pvPortMalloc(sizeof(float) * size);
	}
}

void dataCopy(int type, int *cnt, SensorsData *src, SensorsData *dst)
{
	(*cnt)--;
	switch (type)
	{
	case INT8:
		memcpy((int8_t*) dst->data + ((*cnt) * src->data_len), (int8_t*) src->data, src->data_len * sizeof(int8_t));
		break;

	case INT16:
		memcpy((int16_t*) dst->data + ((*cnt) * src->data_len), (int16_t*) src->data, src->data_len * sizeof(int16_t));
		break;

	case INT32:
		memcpy((int32_t*) dst->data + ((*cnt) * src->data_len), (int32_t*) src->data, src->data_len * sizeof(int32_t));
		break;

	case FLOAT:
//		for (int packing_cnt = src->data_len; packing_cnt; --packing_cnt) *(((float*) dst->data) + ((*cnt) * src->data_len) + packing_cnt) = *((float*) src->data + packing_cnt);
		memcpy((float*) dst->data + ((*cnt) * src->data_len), (float*) src->data, src->data_len * sizeof(float));
		break;
	}
}



void master_timer_Callback(void const *arg)
{
	int *message;

	message = (int *) osPoolAlloc(thread_poolId);
	*message = PERIOD_CHECK;
	osMessagePut(master_queueId, (uint32_t) message, osWaitForever);
}

void master_timer_Start(void)
{
	exec = 1;
	master_timerId = osTimerCreate(osTimer(master_timer), osTimerPeriodic, &exec);			// Create periodic timer
	if (master_timerId) osTimerStart(master_timerId, (float) timer_period_master);       	// start timer
}

void master_timer_Stop(void) { osTimerStop(master_timerId); }



void ble_timer_Callback(void const *arg) { osSemaphoreRelease(ble_semId); }

void ble_timer_Start(void)
{
	exec = 1;
	ble_timerId = 		osTimerCreate(osTimer(ble_timer), osTimerPeriodic, &exec);			// Create periodic timer
	if (ble_timerId) 	osTimerStart(ble_timerId, (float) timer_period_master);
}

void ble_timer_Stop(void) { osTimerStop(ble_timerId); }

void delayedMessage_Callback(void const *arg)
{
	osEvent evt;
	DelayedSensorsDataMessage *delayedMessage;


	evt = osMessageGet(delayed_queueId, osWaitForever);

	if (evt.status == osEventMessage)
	{
		delayedMessage = (DelayedSensorsDataMessage *) evt.value.p;
		if (osMessagePut(delayedMessage->receiver, (uint32_t) delayedMessage->message, osWaitForever) != osOK) Error_Handler();
		osTimerDelete(delayedMessage->timer_id);
		vPortFree(delayedMessage);
	}
}









static void GetBatteryInfoData(void)
{
//  uint32_t soc;
//  int32_t current= 0;
//	uint32_t voltage;

  uint8_t v_mode;

  BSP_GG_Task(STC3115_handle,&v_mode);		// Update Gas Gouge Status

  /* Read the Gas Gouge Status */
  BSP_GG_GetVoltage(STC3115_handle, &voltage);
  BSP_GG_GetCurrent(STC3115_handle, &current);
  BSP_GG_GetSOC(STC3115_handle, &soc);

  #ifdef DEBUG_USB_NOTIFY_TRAMISSION
  STLBLE_PRINTF("Charge= %ld%% Voltage=%ld mV Current= %ld mA\r\n", soc, voltage, current);
  #endif
}



void delayedOsMessagePut(unsigned int msDelay, int sensor_id, osMessageQId receiver, SensorsData *message)
{
	DelayedSensorsDataMessage *delayedMessage;
	delayedMessage = (DelayedSensorsDataMessage *) pvPortMalloc(sizeof(DelayedSensorsDataMessage));

	delayedMessage->timer_id = osTimerCreate(osTimer(delayedTimer), osTimerOnce, &exec);
	delayedMessage->msDelay = msDelay;
	delayedMessage->sensor_id = sensor_id;
	delayedMessage->receiver = receiver;
	delayedMessage->message = message;

	if(osMessagePut(delayed_queueId, (uint32_t) delayedMessage, osWaitForever) != osOK) Error_Handler();

	osTimerStart(delayedMessage->timer_id, delayedMessage->msDelay);
}



static void InitBlueNRGStack(void)
{
	#ifdef DEBUG_USB_CONNECTION_INFO
	{
		STLBLE_PRINTF("STMicroelectronics %s:\r\n"
			"\tVersion %c.%c.%c\r\n"
			"\tSensorTile"
			"\r\n",
		STLBLE_PACKAGENAME, STLBLE_VERSION_MAJOR,STLBLE_VERSION_MINOR,STLBLE_VERSION_PATCH);

	STLBLE_PRINTF("\t(HAL %ld.%ld.%ld_%ld)\r\n" "\tCompiled %s %s"
	#if defined (__IAR_SYSTEMS_ICC__)
		" (IAR)\r\n"
	#elif defined (__CC_ARM)
		" (KEIL)\r\n"
	#elif defined (__GNUC__)
		" (openstm32)\r\n"
	#endif
			,HAL_GetHalVersion() >>24,
			(HAL_GetHalVersion() >>16)&0xFF,
			(HAL_GetHalVersion() >> 8)&0xFF,
			 HAL_GetHalVersion()      &0xFF,
			__DATE__,__TIME__);
	}
	#endif

	const char 		BoardName[8] = {NAME_STLBLE,0};
	uint16_t 		service_handle, dev_name_char_handle, appearance_char_handle;
	int 			ret;
	uint8_t  		hwVersion;
	uint16_t 		fwVersion;

	#ifdef STATIC_BLE_MAC
	uint8_t tmp_bdaddr[6] = {STATIC_BLE_MAC};
    for (uint8_t i = 0; i < 6; i++) bdaddr[i] = tmp_bdaddr[i];
	#endif


	BNRG_SPI_Init();  															// Initialize the BlueNRG SPI driver
	HCI_Init();																	// Initialize the BlueNRG HCI
	BlueNRG_RST();																// Reset BlueNRG hardware
	getBlueNRGVersion(&hwVersion, &fwVersion);									// Get the BlueNRG HW and FW versions

	if (hwVersion > 0x30) TargetBoardFeatures.bnrg_expansion_board = IDB05A1;	// X-NUCLEO-IDB05A1 expansion board is used
	else TargetBoardFeatures.bnrg_expansion_board = IDB04A1;					// X-NUCLEO-IDB0041 expansion board is used

	BlueNRG_RST();																// Reset BlueNRG again otherwise it will fail.

	#ifndef STATIC_BLE_MAC														// Create a Unique BLE MAC
    bdaddr[0] = (STM32_UUID[1]>>24)&0xFF;
    bdaddr[1] = (STM32_UUID[0]    )&0xFF;
    bdaddr[2] = (STM32_UUID[2] >>8)&0xFF;
    bdaddr[3] = (STM32_UUID[0]>>16)&0xFF;
    bdaddr[4] = (((STLBLE_VERSION_MAJOR-48)*10) + (STLBLE_VERSION_MINOR-48)+100)&0xFF;
    bdaddr[5] = 0xC0;															// For a Legal BLE Random MAC

	#else
    ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);
    if (ret)
    {
		#ifdef DEBUG_USB_CONNECTION_INFO
    	{
    		STLBLE_PRINTF("\r\nSetting Pubblic BD_ADDR failed\r\n");
    	}
		#endif
		goto fail;
	}
	#endif

	ret = aci_gatt_init();
	if (ret)
	{
		#ifdef DEBUG_USB_CONNECTION_INFO
    	{
    		STLBLE_PRINTF("\r\nGATT_Init failed\r\n");
        }
    	#endif
		goto fail;
	}

	if (TargetBoardFeatures.bnrg_expansion_board == IDB05A1)
	{
		ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
	}
	else
	{
		ret = aci_gap_init_IDB04A1(GAP_PERIPHERAL_ROLE_IDB04A1, &service_handle, &dev_name_char_handle, &appearance_char_handle);
	}

	if (ret != BLE_STATUS_SUCCESS)
	{
		#ifdef DEBUG_USB_CONNECTION_INFO
		{
			STLBLE_PRINTF("\r\nGAP_Init failed\r\n");
        }
    	#endif
		goto fail;
	}

	#ifndef  STATIC_BLE_MAC
	ret = hci_le_set_random_address(bdaddr);
	if (ret)
	{
		#ifdef DEBUG_USB_CONNECTION_INFO
		{
			STLBLE_PRINTF("\r\nSetting the Static Random BD_ADDR failed\r\n");
		}
		#endif
		goto fail;
	}
	#endif

	ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0, 7/*strlen(BoardName)*/, (uint8_t *) BoardName);
	if (ret)
	{
		#ifdef DEBUG_USB_CONNECTION_INFO
		{
			STLBLE_PRINTF("\r\naci_gatt_update_char_value failed\r\n");
		}
		#endif
		while (1);
	}

	ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED, OOB_AUTH_DATA_ABSENT, NULL, 7, 16, USE_FIXED_PIN_FOR_PAIRING, 123456, BONDING);
	if (ret != BLE_STATUS_SUCCESS)
	{
		#ifdef DEBUG_USB_CONNECTION_INFO
		{
			STLBLE_PRINTF("\r\nGAP setting Authentication failed\r\n");
		}
		#endif
		goto fail;
	}

	#ifdef DEBUG_USB_CONNECTION_INFO
	{
		STLBLE_PRINTF("SERVER: BLE Stack Initialized \r\n"
		"\t\tBoard type=%s HWver=%d, FWver=%d.%d.%c\r\n"
		"\t\tBoardName= %s\r\n"
		"\t\tBoardMAC = %x:%x:%x:%x:%x:%x\r\n\n",
		"SensorTile",
		hwVersion,
		fwVersion>>8,
		(fwVersion>>4)&0xF,
		(hwVersion > 0x30) ? ('a'+(fwVersion&0xF)-1) : 'a',
		BoardName,
		bdaddr[5],bdaddr[4],bdaddr[3],bdaddr[2],bdaddr[1],bdaddr[0]);
	}
	#endif

	aci_hal_set_tx_power_level(0,3);											// Set output power level

	return;

	fail:
		return;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
		case BNRG_SPI_EXTI_PIN:
			HCI_Isr();
			HCI_ProcessEvent=1;
			break;

		default:

			break;
	}
}



void powerControl(int mode, int frequency)
{

	switch (mode)
	{
		/**
		  * @brief Configure the main internal regulator output voltage.
		  * @param  VoltageScaling: specifies the regulator output voltage to achieve
		  *         a tradeoff between performance and power consumption.
		  *          This parameter can be one of the following values:
		  *            @arg @ref PWR_REGULATOR_VOLTAGE_SCALE1 Regulator voltage output range 1 mode,
		  *                                                typical output voltage at 1.2 V,
		  *                                                system frequency up to 80 MHz.
		  *            @arg @ref PWR_REGULATOR_VOLTAGE_SCALE2 Regulator voltage output range 2 mode,
		  *                                                typical output voltage at 1.0 V,
		  *                                                system frequency up to 26 MHz.
		  * @note  When moving from Range 1 to Range 2, the system frequency must be decreased to
		  *        a value below 26 MHz before calling HAL_PWREx_ControlVoltageScaling() API.
		  *        When moving from Range 2 to Range 1, the system frequency can be increased to
		  *        a value up to 80 MHz after calling HAL_PWREx_ControlVoltageScaling() API.
		  * @note  When moving from Range 2 to Range 1, the API waits for VOSF flag to be
		  *        cleared before returning the status. If the flag is not cleared within
		  *        50 microseconds, HAL_TIMEOUT status is reported.
		  * @retval HAL Status
		  */
		case RUN:

			HAL_PWREx_DisableLowPowerRunMode();
			if(frequency < 26)
			{
				HAL_RCC_DeInit();
				SystemClock_Config_adv(frequency);
				ulTimerCountsForOneTick = ( ( SystemCoreClock ) / configTICK_RATE_HZ );

				HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);
//				HAL_PWREx_EnableLowPowerRunMode();
			}
			else
			{
				HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

				HAL_RCC_DeInit();
				SystemClock_Config_adv_mod(frequency);
				ulTimerCountsForOneTick = ( ( SystemCoreClock ) / configTICK_RATE_HZ );
			}

		break;



		/**
		  * @brief Enter Low-power Run mode
		  * @note  In Low-power Run mode, all I/O pins keep the same state as in Run mode.
		  * @note  When Regulator is set to PWR_LOWPOWERREGULATOR_ON, the user can optionally configure the
		  *        Flash in power-down monde in setting the RUN_PD bit in FLASH_ACR register.
		  *        Additionally, the clock frequency must be reduced below 2 MHz.
		  *        Setting RUN_PD in FLASH_ACR then appropriately reducing the clock frequency must
		  *        be done before calling HAL_PWREx_EnableLowPowerRunMode() API.
		  * @retval None
		  */
		case LPRUN:
			if(frequency == 1){
				HAL_RCC_DeInit();
				SystemClock_Config_lp();
//				ulTimerCountsForOneTick = ( ( SystemCoreClock ) / configTICK_RATE_HZ );
				HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);
			}
			else{
				HAL_RCC_DeInit();
				SystemClock_Config_lp_2();
//				ulTimerCountsForOneTick = ( ( SystemCoreClock ) / configTICK_RATE_HZ );
				HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

//				HAL_RCC_DeInit();
//				SystemClock_Config_adv(frequency);
//				ulTimerCountsForOneTick = ( ( SystemCoreClock ) / configTICK_RATE_HZ );
//
//				HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);
				HAL_PWREx_EnableLowPowerRunMode();
			}
		break;



		/**
		  * @brief Enter Sleep or Low-power Sleep mode.
		  * @note  In Sleep/Low-power Sleep mode, all I/O pins keep the same state as in Run mode.
		  * @param Regulator: Specifies the regulator state in Sleep/Low-power Sleep mode.
		  *          This parameter can be one of the following values:
		  *            @arg @ref PWR_MAINREGULATOR_ON Sleep mode (regulator in main mode)
		  *            @arg @ref PWR_LOWPOWERREGULATOR_ON Low-power Sleep mode (regulator in low-power mode)
		  * @note  Low-power Sleep mode is entered from Low-power Run mode. Therefore, if not yet
		  *        in Low-power Run mode before calling HAL_PWR_EnterSLEEPMode() with Regulator set
		  *        to PWR_LOWPOWERREGULATOR_ON, the user can optionally configure the
		  *        Flash in power-down monde in setting the SLEEP_PD bit in FLASH_ACR register.
		  *        Additionally, the clock frequency must be reduced below 2 MHz.
		  *        Setting SLEEP_PD in FLASH_ACR then appropriately reducing the clock frequency must
		  *        be done before calling HAL_PWR_EnterSLEEPMode() API.
		  * @note  When exiting Low-power Sleep mode, the MCU is in Low-power Run mode. To move in
		  *        Run mode, the user must resort to HAL_PWREx_DisableLowPowerRunMode() API.
		  * @param SLEEPEntry: Specifies if Sleep mode is entered with WFI or WFE instruction.
		  *           This parameter can be one of the following values:
		  *            @arg @ref PWR_SLEEPENTRY_WFI enter Sleep or Low-power Sleep mode with WFI instruction
		  *            @arg @ref PWR_SLEEPENTRY_WFE enter Sleep or Low-power Sleep mode with WFE instruction
		  * @note  When WFI entry is used, tick interrupt have to be disabled if not desired as
		  *        the interrupt wake up source.
		  * @retval None
		  */
		case SLEEP_WFI:
			HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		break;

		case SLEEP_WFE:
			HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFE);
		break;

		case LPSLEEP_WFI:
			HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		break;

		case LPSLEEP_WFE:
			HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFE);
		break;



		/**
		  * @brief Enter Stop 0 mode.
		  * @note  In Stop 0 mode, main and low voltage regulators are ON.
		  * @note  In Stop 0 mode, all I/O pins keep the same state as in Run mode.
		  * @note  All clocks in the VCORE domain are stopped; the PLL, the MSI,
		  *        the HSI and the HSE oscillators are disabled. Some peripherals with the wakeup capability
		  *        (I2Cx, USARTx and LPUART) can switch on the HSI to receive a frame, and switch off the HSI
		  *        after receiving the frame if it is not a wakeup frame. In this case, the HSI clock is propagated
		  *        only to the peripheral requesting it.
		  *        SRAM1, SRAM2 and register contents are preserved.
		  *        The BOR is available.
		  * @note  When exiting Stop 0 mode by issuing an interrupt or a wakeup event,
		  *         the HSI RC oscillator is selected as system clock if STOPWUCK bit in RCC_CFGR register
		  *         is set; the MSI oscillator is selected if STOPWUCK is cleared.
		  * @note  By keeping the internal regulator ON during Stop 0 mode, the consumption
		  *         is higher although the startup time is reduced.
		  * @param STOPEntry  specifies if Stop mode in entered with WFI or WFE instruction.
		  *          This parameter can be one of the following values:
		  *            @arg @ref PWR_STOPENTRY_WFI  Enter Stop mode with WFI instruction
		  *            @arg @ref PWR_STOPENTRY_WFE  Enter Stop mode with WFE instruction
		  * @retval None
		  */
		case STOP0_WFI:
//			HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
			HAL_PWREx_EnterSTOP0Mode(PWR_STOPENTRY_WFI);
		break;

		case STOP0_WFE:
//			HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE);
			HAL_PWREx_EnterSTOP0Mode(PWR_STOPENTRY_WFE);
		break;



		/**
		  * @brief Enter Stop 1 mode.
		  * @note  In Stop 1 mode, only low power voltage regulator is ON.
		  * @note  In Stop 1 mode, all I/O pins keep the same state as in Run mode.
		  * @note  All clocks in the VCORE domain are stopped; the PLL, the MSI,
		  *        the HSI and the HSE oscillators are disabled. Some peripherals with the wakeup capability
		  *        (I2Cx, USARTx and LPUART) can switch on the HSI to receive a frame, and switch off the HSI
		  *        after receiving the frame if it is not a wakeup frame. In this case, the HSI clock is propagated
		  *        only to the peripheral requesting it.
		  *        SRAM1, SRAM2 and register contents are preserved.
		  *        The BOR is available.
		  * @note  When exiting Stop 1 mode by issuing an interrupt or a wakeup event,
		  *         the HSI RC oscillator is selected as system clock if STOPWUCK bit in RCC_CFGR register
		  *         is set; the MSI oscillator is selected if STOPWUCK is cleared.
		  * @note  Due to low power mode, an additional startup delay is incurred when waking up from Stop 1 mode.
		  * @param STOPEntry  specifies if Stop mode in entered with WFI or WFE instruction.
		  *          This parameter can be one of the following values:
		  *            @arg @ref PWR_STOPENTRY_WFI  Enter Stop mode with WFI instruction
		  *            @arg @ref PWR_STOPENTRY_WFE  Enter Stop mode with WFE instruction
		  * @retval None
		  */
		case STOP1_WFI:
//			HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
			HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);
		break;

		case STOP1_WFE:
//			HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFE);
			HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFE);
		break;



		/**
		  * @brief Enter Stop 2 mode.
		  * @note  In Stop 2 mode, only low power voltage regulator is ON.
		  * @note  In Stop 2 mode, all I/O pins keep the same state as in Run mode.
		  * @note  All clocks in the VCORE domain are stopped, the PLL, the MSI,
		  *        the HSI and the HSE oscillators are disabled. Some peripherals with wakeup capability
		  *        (LCD, LPTIM1, I2C3 and LPUART) can switch on the HSI to receive a frame, and switch off the HSI after
		  *        receiving the frame if it is not a wakeup frame. In this case the HSI clock is propagated only
		  *        to the peripheral requesting it.
		  *        SRAM1, SRAM2 and register contents are preserved.
		  *        The BOR is available.
		  *        The voltage regulator is set in low-power mode but LPR bit must be cleared to enter stop 2 mode.
		  *        Otherwise, Stop 1 mode is entered.
		  * @note  When exiting Stop 2 mode by issuing an interrupt or a wakeup event,
		  *         the HSI RC oscillator is selected as system clock if STOPWUCK bit in RCC_CFGR register
		  *         is set; the MSI oscillator is selected if STOPWUCK is cleared.
		  * @param STOPEntry  specifies if Stop mode in entered with WFI or WFE instruction.
		  *          This parameter can be one of the following values:
		  *            @arg @ref PWR_STOPENTRY_WFI  Enter Stop mode with WFI instruction
		  *            @arg @ref PWR_STOPENTRY_WFE  Enter Stop mode with WFE instruction
		  * @retval None
		  */

		case STOP2_WFI:
			HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
		break;

		case STOP2_WFE:
			HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFE);
		break;



		/**
		  * @brief Enter Standby mode.
		  * @note  In Standby mode, the PLL, the HSI, the MSI and the HSE oscillators are switched
		  *        off. The voltage regulator is disabled, except when SRAM2 content is preserved
		  *        in which case the regulator is in low-power mode.
		  *        SRAM1 and register contents are lost except for registers in the Backup domain and
		  *        Standby circuitry. SRAM2 content can be preserved if the bit RRS is set in PWR_CR3 register.
		  *        To enable this feature, the user can resort to HAL_PWREx_EnableSRAM2ContentRetention() API
		  *        to set RRS bit.
		  *        The BOR is available.
		  * @note  The I/Os can be configured either with a pull-up or pull-down or can be kept in analog state.
		  *        HAL_PWREx_EnableGPIOPullUp() and HAL_PWREx_EnableGPIOPullDown() respectively enable Pull Up and
		  *        Pull Down state, HAL_PWREx_DisableGPIOPullUp() and HAL_PWREx_DisableGPIOPullDown() disable the
		  *        same.
		  *        These states are effective in Standby mode only if APC bit is set through
		  *        HAL_PWREx_EnablePullUpPullDownConfig() API.
		  * @retval None
		  */
		case STANDBY:
			HAL_PWR_EnterSTANDBYMode();
		break;



		/**
		  * @brief Enter Shutdown mode.
		  * @note  In Shutdown mode, the PLL, the HSI, the MSI, the LSI and the HSE oscillators are switched
		  *        off. The voltage regulator is disabled and Vcore domain is powered off.
		  *        SRAM1, SRAM2 and registers contents are lost except for registers in the Backup domain.
		  *        The BOR is not available.
		  * @note  The I/Os can be configured either with a pull-up or pull-down or can be kept in analog state.
		  * @retval None
		  */
		case SHUTDOWN:
			HAL_PWREx_EnterSHUTDOWNMode();
		break;



		default:
			while(1);
		break;
	}
}

///**
//* @brief  Initialize all sensors
//* @param  None
//* @retval None
//*/
//static void initializeAcc( void )
//{
//	if (BSP_ACCELERO_Init( LSM303AGR_X_0, &LSM303AGR_X_0_handle ) != COMPONENT_OK)
//	{
//		  while(1);
//	}
//
//}
//
//
///**
// * @brief  Enable all sensors
// * @param  None
// * @retval None
// */
// void enableAcc( void )
// {
//   BSP_ACCELERO_Sensor_Enable( LSM303AGR_X_0_handle );
// }
//
//
// /**
// * @brief  Set ODR all sensors
// * @param  None
// * @retval None
// */
// void setOdrAcc( void )
// {
//   BSP_ACCELERO_Set_ODR_Value( LSM303AGR_X_0_handle, ACCELERO_ODR);
// }
//
//
// /**
// * @brief  Disable all sensors
// * @param  None
// * @retval None
// */
// void disableAcc( void )
// {
//   BSP_ACCELERO_Sensor_Disable( LSM303AGR_X_0_handle );
// }


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config_SPI(void)
{
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	  __HAL_RCC_PWR_CLK_ENABLE();
	  HAL_PWR_EnableBkUpAccess();

	  /* Enable the LSE Oscilator */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
	  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /* Enable the CSS interrupt in case LSE signal is corrupted or not present */
	  HAL_RCCEx_DisableLSECSS();

	  /* Enable MSI Oscillator and activate PLL with MSI as source */
	  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
	  RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
	  RCC_OscInitStruct.HSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
	  RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_5; // 11
	  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_MSI;
	  RCC_OscInitStruct.PLL.PLLM            = 6;	// Default value = 6
	  RCC_OscInitStruct.PLL.PLLN            = 40;
	  RCC_OscInitStruct.PLL.PLLP            = 7;
	  RCC_OscInitStruct.PLL.PLLQ            = 2;	// 4
	  RCC_OscInitStruct.PLL.PLLR            = 2;	// Minimum value: 4
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	  if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /* Enable MSI Auto-calibration through LSE */
	  HAL_RCCEx_EnableMSIPLLMode();

	  /* Select MSI output as USB clock source */
	  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
	  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_MSI;
	  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

	  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
	  clocks dividers */
	  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1; //2
	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_6|GPIO_PIN_2|GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PG12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC6 PC2 PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_6|GPIO_PIN_2|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}



void power_up_ADS()
{
//    HAL_GPIO_WritePin(START_PER,START_PIN,GPIO_PIN_SET); // START_PORT|=START_PIN;
//    HAL_GPIO_WritePin(CS_PER,CS_PIN,GPIO_PIN_RESET); // CS_PORT&=~CS_PIN;

	HAL_GPIO_WritePin(START_PER,START_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RESET_PER,RESET_PIN,GPIO_PIN_RESET); // RESET_ADS;                      // Reset device
    HAL_GPIO_WritePin(CS_PER,CS_PIN,GPIO_PIN_SET); // CS_PORT&=~CS_PIN;


//    POWERUP_ADS;                    // Power up device
//    //__delay_cycles(T_RST);

//    //CLKSEL
//    P3DIR &=~BIT7; //input direction
//    P3SEL &=~ BIT7;


//  //EXT_CLK
//    P2DIR &=~ BIT2;
//    P2SEL &=~ BIT2;
//    P2IE &= ~BIT2;

//
    HAL_Delay(10); //__delay_cycles(T_RST);
//    //__delay_cycles(T_POR);        // Wait t_por which is 2^16 tclk (2.048MHz) -> 32ms
//    HAL_GPIO_WritePin(RESET_PER,RESET_PIN,GPIO_PIN_RESET); // RESET_ADS;                      // Reset device
//    HAL_Delay(10); //__delay_cycles(T_RST);          // Hold reset for at least 1 sec
//    HAL_GPIO_WritePin(RESET_PER,RESET_PIN,GPIO_PIN_SET); // RELEASE_ADS;                    // Release device from reset
//    HAL_Delay(1000); //__delay_cycles(T_STA);          // Wait start interval before using the device: min 18tclk

	HAL_GPIO_WritePin(START_PER,START_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(RESET_PER,RESET_PIN,GPIO_PIN_SET); // RESET_ADS;                      // Reset device

    send_sw_command_ADS(ADS_CMD_SDATAC);// Since ADS1192 at power-up is in RDATAC mode it is necessary to stop the conversion
}

void start_ADS(void){
	HAL_GPIO_WritePin(CS_PER,CS_PIN,GPIO_PIN_SET); //CS_PORT&=~CS_PIN;

    // __disable_interrupt();  //Disable interrupts globally

    //---------------Electrode Configuration Channel1--------------
//    configuration[1] = F_250_SPS; // CONFIG1
//    configuration[2] =  REF_ON; ////CONFIG2
//    configuration[3] = ELECTRODE | GAIN_12;  // CH1SET
//    configuration[5] = PWUP_RLD | RLD2N | RLD2P; // RLD_SENS

    write_single_register_ADS(	ADS_REG_CONFIG1, 	F_250_SPS 								);
    write_single_register_ADS(	ADS_REG_CONFIG2, 	ADS_VAL_REF_ON	 						);
    write_single_register_ADS(	ADS_REG_CH1SET, 	ADS_CHPD | ADS_MUX_SHORTED	 			);
    write_single_register_ADS(	ADS_REG_CH2SET, 	ADS_CHPD | ADS_MUX_SHORTED	 			);
    write_single_register_ADS(	ADS_REG_CH3SET, 	ADS_CHPD | ADS_MUX_SHORTED	 			);
    write_single_register_ADS(	ADS_REG_CH4SET, 	ADS_PGA_GAIN_12 | ADS_MUX_ELECTRODE 	);
    write_single_register_ADS(	ADS_REG_CH5SET, 	ADS_CHPD | ADS_MUX_SHORTED	 			);
    write_single_register_ADS(	ADS_REG_CH6SET, 	ADS_CHPD | ADS_MUX_SHORTED	 			);
    write_single_register_ADS(	ADS_REG_CH7SET, 	ADS_CHPD | ADS_MUX_SHORTED	 			);
    write_single_register_ADS(	ADS_REG_CH8SET, 	ADS_CHPD | ADS_MUX_SHORTED	 			);

    //------------------check register writing/reading--------------------------
    //read_single_register_ADS1192(CONFIG2, &configuration[6], 0);

    //------------------------------------------------------------------
    send_sw_command_ADS(ADS_CMD_RDATAC); // send_sw_command_ADS1192_all(RDATAC);
//    HAL_GPIO_WritePin(START_PER,START_PIN,GPIO_PIN_SET); // ???? HW_START_ADS;                             // starts ADS via hardware signal
	HAL_GPIO_WritePin(CS_PER,CS_PIN,GPIO_PIN_RESET); // ???? CS_PORT&=~CS_PIN;
//    __enable_interrupt();   //Enable interrupts globally
//    ENABLE_DRDY_INT;                                           // enable interrupt on port 2 (DRDY)

//  CODE !!
}

int write_single_register_ADS(unsigned char reg_address, unsigned char reg_data)
{
    unsigned char opcode;

    if(reg_address > ADS_REG_MAXADD)
        return -1;
    if(reg_address == ADS_REG_GPIO)
        return 0;

    opcode = ADS_CMD_WREG + reg_address;                // Create OPCODE1 to perform read instruction

//    interrupt_status = UCA0IE;              // Save Interrupt enable settings
//    UCA0IE &= ~UCRXIE;                      // Disable interrupt on receive from ADS1192

	HAL_GPIO_WritePin(CS_PER, CS_PIN, GPIO_PIN_RESET); // set_device_CS_ADS1192();// select the same ADS device

//    while(!(UCA0IFG&UCTXIFG));
//    UCA0TXBUF = opcode;      // Send read operation op-code
	HAL_SPI_Transmit(&hspi3, &opcode, 1, 100);

//    delayUS_ASM(32); //__delay_cycles(TX_DEL);

    opcode = 0;

//    while(!(UCA0IFG&UCTXIFG));
//    UCA0TXBUF = 0x00;                       // For a single write the number of registers to be written should be 0
	HAL_SPI_Transmit(&hspi3, &opcode, 1, 100);

//    delayUS_ASM(32); //__delay_cycles(TX_DEL);

//    while(!(UCA0IFG&UCTXIFG));
//    UCA0TXBUF = reg_data;                   // Write register value
	HAL_SPI_Transmit(&hspi3, &reg_data, 1, 100);

//	delayUS_ASM(1);
	HAL_GPIO_WritePin(CS_PER, CS_PIN, GPIO_PIN_SET); // unset_device_CS_ADS1192();            // deselect correct ADS device----------------------------------------------------------

//    UCA0IE = interrupt_status;              // Restore Interrupt enable register

    return 0;
}


int send_sw_command_ADS(unsigned char command)
{
    if(command > 0x12) return -1;

	HAL_GPIO_WritePin(CS_PER, CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &command, 1, 100);
//	delayUS_ASM(1);
	HAL_GPIO_WritePin(CS_PER, CS_PIN, GPIO_PIN_SET);

    return 0;
}



void Error_Handler(void) { while (1) {} }



#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */



void assert_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{}
}
#endif

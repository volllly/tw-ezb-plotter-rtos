////////////////////////////////////////////////////////////////
//   MakeBlock Project / µCos3                                //
//   =========================                                //
//   Copyright© 2018 by Felix Knobl, FH Technikum Wien        //
//   Copyright© 2018 by Paul Volvsek, FH Technikum Wien       //
//   Partial code parts from: Roman Beneder, Martin Horauer   //
//   ======================================================   //
//   Task description:                                        //
//   https://cis.technikum-wien.at/documents/bel/4/ezb/       //
//   semesterplan/tasks/project/project.html                  //
//   ======================================================   //
//   Baud rate changed to: 115200                             //
////////////////////////////////////////////////////////////////

#define DEBUG_USE_IO_EXPANDER

#include <app_cfg.h>
#include <cpu_core.h>
#include <os.h>

#include <bsp.h>
#include <bsp_sys.h>
#include <bsp_int.h>

#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

#include <xmc_common.h>
#include <xmc_uart.h>
#include <xmc_gpio.h>
#include <xmc4500_spi_lib.h>
#include <mcp23s08_drv.h>
#include <xmc_ccu4.h>
#include <GPIO.h>
#include <lib_math.h>

#if SEMI_HOSTING
	#include <debug_lib.h>
#endif

#if JLINK_RTT
	#include <SEGGER_RTT.h>
	#include <SEGGER_RTT_Conf.h>
#endif

// Definitions
#define MAX_MSG_LENGTH	20
#define NUM_MSG			10u
#define SEP				':'

// Stepper driving constants
#if defined DEBUG_USE_IO_EXPANDER

	#define X_STEP_POS		0x03
	#define X_STEP_NEG		0x02
	#define Y_STEP_POS		0x0C
	#define Y_STEP_NEG		0x08

	#define X_MOTOR_STOP	0x00
	#define Y_MOTOR_STOP	0x00

	#define X_DIR_POS		0x01
	#define X_DIR_NEG		0x00

#else

	#define X_DIR_POS		P3_3_set();
	#define X_DIR_NEG		P3_3_reset();

	#define X_STEP			P3_4_set();
	#define X_STOP			P3_4_reset();

	#define Y_DIR_POS		P1_8_reset();
	#define Y_DIR_NEG		P1_8_set();

	#define Y_STEP			P1_0_set();
	#define Y_STOP			P1_0_reset();

#endif

// Servo definitions
#define CCU40_20MS		(uint16_t)9370
#define PEN_MID_POS		(uint16_t)CCU40_20MS * (20 - 1.5) / 20 	// 1.5 is the HIGH time in milliseconds
#define PEN_UP_POS		(uint16_t)CCU40_20MS * (20 - 1.2) / 20	// 1.2 is the HIGH time in milliseconds
#define PEN_DOWN_POS	(uint16_t)CCU40_20MS * (20 - 1.8) / 20	// 1.8 is the HIGH time in milliseconds
#define PEN_DELAY		(uint32_t)3000000

// Endpoint GPIO Macros
#define ENDSTOP_RIGHT	P1_12_read()
#define ENDSTOP_LEFT	P1_14_read()
#define ENDSTOP_BOTTOM	P1_13_read()
#define ENDSTOP_TOP		P1_15_read()

// Stepper delays
#define STEP_PULSE		1000		// Time for HIGH level duration
#define STEP_DELAY		25000		// Time delay between steps

// Task definition variables
static  CPU_STK  AppStartTaskStk[APP_CFG_TASK_START_STK_SIZE];
static  OS_TCB   AppStartTaskTCB;
static  CPU_STK  AppTaskComStk[APP_CFG_TASK_COM_STK_SIZE];
static  OS_TCB   AppTaskComTCB;

// Memory Block
OS_MEM Mem_Partition;
CPU_CHAR MyPartitionStorage[NUM_MSG - 1][MAX_MSG_LENGTH];

// Message Queues
OS_Q UART_ISR;

// Local Prototypes
static void AppTaskStart(void *p_arg);
static void AppTaskCreate(void);
static void AppObjCreate(void);
static void AppTaskCom(void *p_arg);

static void penUp();
static void penDown();
static void startCommand();
static bool finCommand();
static bool moveToLineToCommand(uint32_t xTo, uint32_t yTo);
static void curveToCommand();

typedef enum parameterType
{
	UINT,
	CHAR,
	STRING
} parameterType_t;

static void *getParamById(CPU_CHAR* pUARTMessage, uint8_t paramID, parameterType_t PT);
static void initCCU4X(XMC_CCU4_MODULE_t *pCCU, XMC_CCU4_SLICE_t *pSlice, uint8_t sliceNumber, uint32_t prescaler, const XMC_GPIO_PORT_t *pPort, const uint8_t pin);
static void  setCCU4X(XMC_CCU4_MODULE_t *pCCU, XMC_CCU4_SLICE_t *pSlice, uint8_t sliceNumber, uint16_t periodValue, uint16_t pwmValue);

static const uint32_t CCU4_SHADOW_TRANSFER_SLICES[] = { 1u, 16u, 256u, 4096u };

// Max X and Y Steps for the whole working area
static uint32_t MaxX = 0;
static uint32_t MaxY = 0;

// Current Positions
static uint32_t CurX = 0;
static uint32_t CurY = 0;

// Pen up
static bool isPenUp = false;

// This is the standard entry point for C code.
int main (void)
{
	OS_ERR err;

	// Disable all interrupts
	BSP_IntDisAll();

	// Enable Interrupt UART
	BSP_IntEn(BSP_INT_ID_USIC1_01);
	BSP_IntEn(BSP_INT_ID_USIC1_00);

	// Init SEMI Hosting DEBUG Support
	#if SEMI_HOSTING
		initRetargetSwo();
	#endif

	// Init JLINK RTT DEBUG Support
	#if JLINK_RTT
		SEGGER_RTT_ConfigDownBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
		SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
	#endif

	// Init uC/OS-III
	OSInit(&err);

	if (err != OS_ERR_NONE)
		APP_TRACE_DBG ("Error OSInit: main\n");

	// Create the start task
	OSTaskCreate((OS_TCB *)		&AppStartTaskTCB,
				(CPU_CHAR *)	"Startup Task",
				(OS_TASK_PTR)	AppTaskStart,
				(void *)		0,
				(OS_PRIO)		APP_CFG_TASK_START_PRIO,
				(CPU_STK *)		&AppStartTaskStk[0],
				(CPU_STK_SIZE)	APP_CFG_TASK_START_STK_SIZE / 10u,
				(CPU_STK_SIZE)	APP_CFG_TASK_START_STK_SIZE,
				(OS_MSG_QTY)	0u,
				(OS_TICK)		0u,
				(void *)		0,
				(OS_OPT)		(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
				(OS_ERR *)		&err);

	// Start multitasking (i.e., give control to uC/OS-III)
	OSStart(&err);

	if (err != OS_ERR_NONE)
		APP_TRACE_DBG("Error OSStart: main\n");

	while (1)
	{
		APP_TRACE_DBG("Should never be output! Bug?\n");
	}

	return 0;
}


// Startup (init) task that loads board support functions,
// initializes CPU services, the memory, the systick timer,
// etc. and finally invokes other application tasks.
static void AppTaskStart (void *p_arg)
{
	CPU_INT32U	cpu_clk_freq;
	CPU_INT32U	cnts;
	OS_ERR		err;
	(void)		p_arg;

	// Initialize BSP functions
	BSP_Init();

	// Initialize the uC/CPU services
	CPU_Init();

	// Determine SysTick reference frequency
	cpu_clk_freq = BSP_SysClkFreqGet();

	// Determine nbr SysTick increments
	cnts = cpu_clk_freq / (CPU_INT32U) OSCfg_TickRate_Hz;

	// Init uCOS-III periodic time src (SysTick)
	OS_CPU_SysTickInit (cnts);

	// Initialize memory management module
	Mem_Init();

	// Initialize mathematical module
	Math_Init();

	// Init PWM for servo control
	initCCU4X(CCU40, CCU40_CC40, 0, 8, XMC_GPIO_PORT1, 3);

	// Lift pen up
	penUp();

	_init_spi();

	// Init I/O Expander and configure all IOs as outputs

	#if defined DEBUG_USE_IO_EXPANDER
    	_mcp23s08_reset_ss(MCP23S08_SS);
    	_mcp23s08_reg_xfer(XMC_SPI1_CH0, MCP23S08_IODIR, 0x00, MCP23S08_WR);
	#else
    	// DEBUG: Configure XMC GPIO Pins as outputs to control the motors
    	P3_3_set_mode(OUTPUT_PP_GP);
    	P3_3_reset();

    	P3_4_set_mode(OUTPUT_PP_GP);
    	P3_4_reset();

    	P1_8_set_mode(OUTPUT_PP_GP);
    	P1_8_reset();

    	P1_0_set_mode(OUTPUT_PP_GP);
       	P1_0_reset();
	#endif

    // Configure Endstops GPIO as inputs
	// LEFT
	P1_14_set_mode(INPUT_PU);
	P1_14_set_driver_strength(MEDIUM);

	// RIGHT
	P1_12_set_mode(INPUT_PU);
	P1_12_set_driver_strength(MEDIUM);

    // TOP
	P1_15_set_mode(INPUT_PU);
	P1_15_set_driver_strength(MEDIUM);

	// BOTTOM
	P1_13_set_mode(INPUT_PU);
	P1_13_set_driver_strength(MEDIUM);

	// Compute CPU capacity with no task running
	#if (OS_CFG_STAT_TASK_EN > 0u)
		OSStatTaskCPUUsageInit(&err);

		if (err != OS_ERR_NONE)
			APP_TRACE_DBG("Error OSStatTaskCPUUsageInit: AppTaskStart\n");
	#endif

	APP_TRACE_INFO("Creating Application Objects...\n");

	// create application objects
	AppObjCreate();
	APP_TRACE_INFO("Creating Application Tasks...\n");

	// create application tasks
	AppTaskCreate();

	while (DEF_TRUE)
	{
		// Suspend current task
		OSTaskSuspend((OS_TCB *)0, &err);

		if (err != OS_ERR_NONE)
			APP_TRACE_DBG("Error OSTaskSuspend: AppTaskStart\n");
	}
}

// Creates the application objects
static void AppObjCreate(void)
{
	OS_ERR err;

	// Create Shared Memory for UART ISR
	OSMemCreate((OS_MEM *)		&Mem_Partition,
				(CPU_CHAR *)	"Mem Partition",
				(void *)		&MyPartitionStorage[0][0],
				(OS_MEM_QTY)	NUM_MSG,
				(OS_MEM_SIZE)	MAX_MSG_LENGTH * sizeof (CPU_CHAR),
				(OS_ERR *)		&err);

	if (err != OS_ERR_NONE)
		APP_TRACE_DBG("Error OSMemCreate: AppObjCreate\n");

	// Create UART ISR Message Queue
	OSQCreate(	(OS_Q *)		&UART_ISR,
				(CPU_CHAR *)	"ISR Queue",
				(OS_MSG_QTY)	NUM_MSG,
				(OS_ERR *)		&err);

	if (err != OS_ERR_NONE)
		APP_TRACE_DBG("Error OSQCreate ISR Queue: AppObjCreate\n");
}

// Creates the Application Tasks
static void AppTaskCreate(void)
{
	OS_ERR err;

	// Create AppTaskCom
	OSTaskCreate((OS_TCB *)		&AppTaskComTCB,
				(CPU_CHAR *)	"TaskCOM",
				(OS_TASK_PTR)	AppTaskCom,
				(void *)		0,
				(OS_PRIO)		APP_CFG_TASK_COM_PRIO,
				(CPU_STK *)		&AppTaskComStk[0],
				(CPU_STK_SIZE)	APP_CFG_TASK_COM_STK_SIZE / 10u,
				(CPU_STK_SIZE)	APP_CFG_TASK_COM_STK_SIZE,
				(OS_MSG_QTY)	0u,
				(OS_TICK)		0u,
				(void *)		0,
				(OS_OPT)		(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
				(OS_ERR *)		&err);

	if (err != OS_ERR_NONE)
		APP_TRACE_DBG ("Error OSTaskCreate TaskCOM: AppTaskCreate\n");
}

// Communication Task between UART and LED task
static void AppTaskCom(void *p_arg)
{
	void        *pMessage;
	OS_ERR      err;
	OS_MSG_SIZE messageSize;

	char    	UARTrequestMessage[MAX_MSG_LENGTH];
	char		UARTresponseMessage[MAX_MSG_LENGTH];

	int n = 0;
	char commandType = '\0';
	uint32_t xTo, yTo = 0;

	bool commandSuccess = false;

	(void) p_arg;
	APP_TRACE_INFO ("Entering AppTaskCom...\n");

	while (DEF_TRUE)
	{
		// Empty the buffers
		memset(&UARTrequestMessage, 0, MAX_MSG_LENGTH);
		memset(&UARTresponseMessage, 0, MAX_MSG_LENGTH);

		// ################################
		// ###   PART 1: Receive UART   ###
		// ################################

		// Check if we received a message on UART
		pMessage = OSQPend(&UART_ISR, 0, OS_OPT_PEND_NON_BLOCKING, &messageSize, NULL, &err);

		if (err == OS_ERR_NONE)
		{
			// Obtain the duty cycle we received
			memcpy(UARTrequestMessage, (CPU_CHAR *)pMessage, messageSize - 1);

			// Release the memory partition allocated in the UART service routine <18>
			OSMemPut(&Mem_Partition, pMessage, &err);

			if (err != OS_ERR_NONE)
				APP_TRACE_DBG("Error OSMemPut: AppTaskCom\n");

			#if (APP_TRACE_LEVEL >= TRACE_LEVEL_INFO)
				printf("INFO: UART Message received: '%s'\n", UARTrequestMessage);
			#endif

			printf("Param 0 (C): %c\n", (char)getParamById(UARTrequestMessage, 0, CHAR));
			printf("Param 1 (X): %u\n", (uint32_t)getParamById(UARTrequestMessage, 1, UINT));
			printf("Param 2 (Y): %u\n", (uint32_t)getParamById(UARTrequestMessage, 2, UINT));

			commandType = (char)getParamById(UARTrequestMessage, 0, CHAR);

			switch (commandType)
			{
				case 'S': // START command
					// Lift pen up (if isn't)
					penUp();

					// Perform reference drive
					startCommand();

					// Return X-Max and Y-max to PC
					snprintf(UARTresponseMessage, MAX_MSG_LENGTH - 1, "#S:%i:%i$", MaxX, MaxY);
					break;

				case 'M': // MOVETO command
					// Lift pen up (if isn't)
					penUp();

					// Get the coordinates
					xTo = (uint32_t)getParamById(UARTrequestMessage, 1, UINT);
					yTo = (uint32_t)getParamById(UARTrequestMessage, 2, UINT);

					// Move to X/Y
					commandSuccess = moveToLineToCommand(xTo, yTo);

					if (commandSuccess)
					{
						// Return ACK to PC
						snprintf(UARTresponseMessage, MAX_MSG_LENGTH - 1, "#M:ACK$");
					}
					else
					{
						// Return NACK to PC
						snprintf(UARTresponseMessage, MAX_MSG_LENGTH - 1, "#M:NACK$");
					}

					break;

				case 'L': // DRAW LINE command
					// Lift pen down (if isn't)
					penDown();

					// Get the coordinates
					xTo = (uint32_t)getParamById(UARTrequestMessage, 1, UINT);
					yTo = (uint32_t)getParamById(UARTrequestMessage, 2, UINT);

					// Line to X/Y
					commandSuccess = moveToLineToCommand(xTo, yTo);

					if (commandSuccess)
					{
						// Return ACK to PC
						snprintf(UARTresponseMessage, MAX_MSG_LENGTH - 1, "#L:ACK$");
					}
					else
					{
						// Return NACK to PC
						snprintf(UARTresponseMessage, MAX_MSG_LENGTH - 1, "#L:NACK$");
					}
					break;

				case 'C': // DRAW CURVE command
					// Lift pen down (if isn't)
					penDown();

					// TODO: Draw curve


					// Return ACK to PC
					snprintf(UARTresponseMessage, MAX_MSG_LENGTH - 1, "#C:ACK$");
					break;

				case 'F': // FIN command
					// Lift pen up (if isn't)
					penUp();

					// Return to X/Y 0/0
					commandSuccess = finCommand();

					if (commandSuccess)
					{
						// Return ACK to PC
						snprintf(UARTresponseMessage, MAX_MSG_LENGTH - 1, "#F:ACK$");
					}
					else
					{
						// Return NACK to PC
						snprintf(UARTresponseMessage, MAX_MSG_LENGTH - 1, "#F:NACK$");
					}

					break;

				default:
					break;

			}

			// Transmit response to PC
			for (n = 0; n <= strlen(UARTresponseMessage); n++)
			{
				XMC_UART_CH_Transmit(XMC_UART1_CH1, UARTresponseMessage[n]);
			}

			// Transmit Line Break for debugging
			// XMC_UART_CH_Transmit(XMC_UART1_CH1, '\n');

			printf("Current Plotter Position: X = %u / Y = %u\n", CurX, CurY);

		}

		// Delay to free CPU
		OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STRICT, &err);

		if (err != OS_ERR_NONE)
			APP_TRACE_DBG("Error OSTimeDlyHMSM: AppTaskCom\n");
	}
}

// Returns the parameter with the specified id and data type
static void *getParamById(char* pUARTMessage, uint8_t paramID, parameterType_t PT)
{
	// Example: #<0:1:2:3:4:5...>$

	int separatorCounter = 0;
	int parameterValue = 0;

	char *pSeparatorPos = NULL;
	static char parameterBuffer[20];

	// Clear the buffer since it is static to preserve the data after return
	memset(parameterBuffer, 0, sizeof(parameterBuffer));

	if (paramID == 0)
	{
		// We don't need to loop through the string,
		// since we want the first parameter that starts at position 0

		// Find the first separator
		pSeparatorPos = strchr(pUARTMessage, SEP);
	}
	else
	{
		// Loop through the whole string
		while (*pUARTMessage != '\0')
		{
			// If we found a separator
			if (*pUARTMessage == ':')
			{
				// Increment the separator counter
				separatorCounter++;

				// If we found our separator (parameter)
				if (separatorCounter == paramID)
				{
					// Increment the pointer position to roll over to the start of the parameter
					pUARTMessage++;

					// Find the end of the parameter
					pSeparatorPos = strchr(pUARTMessage, SEP);

					break;
				}
			}

			// Increment the pointer position
			pUARTMessage++;
		}
	}

	// Separator found?
	if (pSeparatorPos != NULL)
	{
		// Copy the parameter value into the buffer
		memcpy(parameterBuffer, pUARTMessage, pSeparatorPos - pUARTMessage);
	}
	else
	{
		// Separator not found (last parameter)
		if (strlen(pUARTMessage) != 0)
		{
			// Copy the parameter value into the buffer
			memcpy(parameterBuffer, pUARTMessage, strlen(pUARTMessage));
		}
		else
		{
			// Parameter error
			return NULL;
		}
	}

	// Which type of parameter do we want?
	switch (PT)
	{
		case UINT:
			// Convert the parameter to integer
			return (uint32_t)strtol(parameterBuffer, NULL, 10);
			break;

		case CHAR:
			return (char)parameterBuffer[0];
			break;

		case STRING:
			return (char *)parameterBuffer;
			break;

		default:
			printf("ERROR: Parameter type unknown!\n");
			return NULL;
			break;
	}
}




static void penUp()
{
	uint32_t delayCounter = PEN_DELAY;

	if (!isPenUp)
	{
		setCCU4X(CCU40, CCU40_CC40, 0, CCU40_20MS, PEN_UP_POS);

		while (delayCounter > 0)
			delayCounter--;

		isPenUp = true;
	}
}

static void penDown()
{
	uint32_t delayCounter = PEN_DELAY;

	if (isPenUp)
	{
		setCCU4X(CCU40, CCU40_CC40, 0, CCU40_20MS, PEN_DOWN_POS);

		while (delayCounter > 0)
			delayCounter--;

		isPenUp = false;
	}
}

// Performs a reference drive.
// Drives to the lower right corner and the drives back to upper left corner and counts the steps
static void startCommand()
{
	uint32_t delayCounter = 0;

	MaxX = 0;
	MaxY = 0;

	#if defined DEBUG_USE_IO_EXPANDER
		// NOP
	#else
		X_DIR_POS;
		Y_DIR_POS;
	#endif

	delayCounter = STEP_DELAY;
	while (delayCounter > 0)
		delayCounter--;

	// Drive to lower right corner
	while (true)
	{
		if (ENDSTOP_RIGHT != 0)
		{
			#if defined DEBUG_USE_IO_EXPANDER
				_mcp23s08_reg_xfer(XMC_SPI1_CH0, MCP23S08_GPIO, X_STEP_POS, MCP23S08_WR);
			#else
				X_STEP;
			#endif
		}

		if (ENDSTOP_BOTTOM != 0)
		{
			#if defined DEBUG_USE_IO_EXPANDER
				_mcp23s08_reg_xfer(XMC_SPI1_CH0, MCP23S08_GPIO, Y_STEP_POS, MCP23S08_WR);
			#else
				Y_STEP;
			#endif
		}

		delayCounter = STEP_PULSE;
		while (delayCounter > 0)
			delayCounter--;

		#if defined DEBUG_USE_IO_EXPANDER
			_mcp23s08_reg_xfer(XMC_SPI1_CH0, MCP23S08_GPIO, X_MOTOR_STOP | Y_MOTOR_STOP, MCP23S08_WR);
		#else
			X_STOP;
			Y_STOP;
		#endif

		delayCounter = STEP_DELAY;
		while (delayCounter > 0)
			delayCounter--;

		if (ENDSTOP_RIGHT == 0 && ENDSTOP_BOTTOM == 0)
		{
			break;
		}
	}

	#if defined DEBUG_USE_IO_EXPANDER

	#else
		X_DIR_NEG;
		Y_DIR_NEG;
	#endif

	delayCounter = STEP_DELAY;
	while (delayCounter > 0)
		delayCounter--;

	// Drive to lower left corner and count the steps
	while (true)
	{
		if (ENDSTOP_LEFT != 0)
		{
			#if defined DEBUG_USE_IO_EXPANDER
				_mcp23s08_reg_xfer(XMC_SPI1_CH0, MCP23S08_GPIO, X_STEP_NEG, MCP23S08_WR);
			#else
				X_STEP;
			#endif

			MaxX++;
		}

		if (ENDSTOP_TOP != 0)
		{
			#if defined DEBUG_USE_IO_EXPANDER
				_mcp23s08_reg_xfer(XMC_SPI1_CH0, MCP23S08_GPIO, Y_STEP_NEG, MCP23S08_WR);
			#else
				Y_STEP;
			#endif

			MaxY++;
		}

		delayCounter = STEP_PULSE;
		while (delayCounter > 0)
			delayCounter--;

		#if defined DEBUG_USE_IO_EXPANDER
			_mcp23s08_reg_xfer(XMC_SPI1_CH0, MCP23S08_GPIO, X_MOTOR_STOP | Y_MOTOR_STOP, MCP23S08_WR);
		#else
			X_STOP;
			Y_STOP;
		#endif

		delayCounter = STEP_DELAY;
		while (delayCounter > 0)
			delayCounter--;

		if (ENDSTOP_LEFT == 0 && ENDSTOP_TOP == 0)
		{
			break;
		}
	}
}

static bool finCommand()
{
	// Move to 0/0
	bool commandSuccess = moveToLineToCommand(0, 0);

	return commandSuccess;
}


// Drives the plotter to specified positions in a line.
// Returns true on success or false, if destination is outside of MaxX/MaxY
static bool moveToLineToCommand(uint32_t xTo, uint32_t yTo)
{
	uint32_t n = 0;
	uint32_t delayCounter = 0;
	uint32_t loopLength = 0;
	uint32_t deltaX = 0;
	uint32_t deltaY = 0;

	int scaleFactor = 0;
	int scaleFactorCounter = 0;

	bool driveXPos = false;
	bool driveYPos = false;

	bool allowDriveX = false;
	bool allowDriveY = false;

	// Check destination
	if (xTo > MaxX || yTo > MaxY)
	{
		return false;
	}

	// Check if we want to go online in one direction
	if (CurX != xTo)
	{
		allowDriveX = true;
	}

	if (CurY != yTo)
	{
		allowDriveY = true;
	}

	// Calculate drive direction X
	if (xTo > CurX)
	{
		// Drive right
		driveXPos = true;
	}

	// Calculate drive direction Y
	if (yTo > CurY)
	{
		// Drive down
		driveYPos = true;
	}

	// Calculate delta distance X and change the stepper direction
	if (driveXPos)
	{
		deltaX = xTo - CurX;

		#ifndef DEBUG_USE_IO_EXPANDER
			X_DIR_POS;
		#endif
	}

	else
	{
		deltaX = CurX - xTo;

		#ifndef DEBUG_USE_IO_EXPANDER
			X_DIR_NEG;
		#endif
	}

	// Calculate delta distance Y and change the stepper direction
	if (driveYPos)
	{
		deltaY = yTo - CurY;

		#ifndef DEBUG_USE_IO_EXPANDER
			Y_DIR_POS;
		#endif
	}
	else
	{
		deltaY = CurY - yTo;

		#ifndef DEBUG_USE_IO_EXPANDER
			Y_DIR_NEG;
		#endif
	}

	// Check larger delta
	if (deltaX > deltaY)
	{
		loopLength = deltaX;

		if (allowDriveY)
			scaleFactor = deltaX / deltaY;
	}
	else
	{
		loopLength = deltaY;

		if (allowDriveX)
			scaleFactor = deltaY / deltaX;
	}

	delayCounter = STEP_DELAY;
	while (delayCounter > 0)
		delayCounter--;

	// Perform the drive
	for (n = 0; n < loopLength; n++)
	{
		if (deltaX > deltaY)
		{
			// Drive each step with motor X
			if (allowDriveX)
			{
				#if defined DEBUG_USE_IO_EXPANDER
					if (driveXPos)
					{
						_mcp23s08_reg_xfer(XMC_SPI1_CH0, MCP23S08_GPIO, X_STEP_POS, MCP23S08_WR);
					}
					else
					{
						_mcp23s08_reg_xfer(XMC_SPI1_CH0, MCP23S08_GPIO, X_STEP_NEG, MCP23S08_WR);
					}
				#else
					X_STEP;
				#endif
			}

			scaleFactorCounter++;

			if (scaleFactorCounter == scaleFactor)
			{
				scaleFactorCounter = 0;

				if (allowDriveY)
				{
					#if defined DEBUG_USE_IO_EXPANDER
						if (driveYPos)
						{
							_mcp23s08_reg_xfer(XMC_SPI1_CH0, MCP23S08_GPIO, Y_STEP_POS, MCP23S08_WR);
						}
						else
						{
							_mcp23s08_reg_xfer(XMC_SPI1_CH0, MCP23S08_GPIO, Y_STEP_NEG, MCP23S08_WR);
						}
					#else
						Y_STEP;
					#endif
				}

			}
		}
		else
		{
			if (allowDriveY)
			{
				#if defined DEBUG_USE_IO_EXPANDER
					if (driveYPos)
					{
						_mcp23s08_reg_xfer(XMC_SPI1_CH0, MCP23S08_GPIO, Y_STEP_POS, MCP23S08_WR);
					}
					else
					{
						_mcp23s08_reg_xfer(XMC_SPI1_CH0, MCP23S08_GPIO, Y_STEP_NEG, MCP23S08_WR);
					}
				#else
					Y_STEP;
				#endif
			}


			scaleFactorCounter++;

			if (scaleFactorCounter == scaleFactor)
			{
				scaleFactorCounter = 0;

				if (allowDriveX)
				{
					#if defined DEBUG_USE_IO_EXPANDER
						if (driveXPos)
						{
							_mcp23s08_reg_xfer(XMC_SPI1_CH0, MCP23S08_GPIO, X_STEP_POS, MCP23S08_WR);
						}
						else
						{
							_mcp23s08_reg_xfer(XMC_SPI1_CH0, MCP23S08_GPIO, X_STEP_NEG, MCP23S08_WR);
						}
					#else
						X_STEP;
					#endif
				}
			}
		}

		delayCounter = STEP_PULSE;
		while (delayCounter > 0)
			delayCounter--;

		#if defined DEBUG_USE_IO_EXPANDER
			_mcp23s08_reg_xfer(XMC_SPI1_CH0, MCP23S08_GPIO, X_MOTOR_STOP | Y_MOTOR_STOP, MCP23S08_WR);
		#else
			X_STOP;
			Y_STOP;
		#endif

		delayCounter = STEP_DELAY;
		while (delayCounter > 0)
			delayCounter--;

	}

	// Update the current position
	CurX = xTo;
	CurY = yTo;

	return true;
}

static void curveToCommand()
{
 // TODO
}

static void initCCU4X(XMC_CCU4_MODULE_t *pCCU, XMC_CCU4_SLICE_t *pSlice, uint8_t sliceNumber, uint32_t prescaler, const XMC_GPIO_PORT_t *pPort, const uint8_t pin)
{
	XMC_CCU4_SLICE_COMPARE_CONFIG_t SLICE_config =
	{
		.timer_mode =				(uint32_t) XMC_CCU4_SLICE_TIMER_COUNT_MODE_EA,
		.monoshot =					(uint32_t) XMC_CCU4_SLICE_TIMER_REPEAT_MODE_REPEAT,
		.shadow_xfer_clear	=		(uint32_t) 0,
		.dither_timer_period =		(uint32_t) 0,
		.dither_duty_cycle =		(uint32_t) 0,
		.prescaler_mode =			(uint32_t) XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,
		.mcm_enable =				(uint32_t) 0,
		.prescaler_initval =		(uint32_t) prescaler,
		.float_limit =				(uint32_t) 0,
		.dither_limit =				(uint32_t) 0,
		.passive_level =			(uint32_t) XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
		.timer_concatenation =		(uint32_t) 0
	};

	XMC_GPIO_CONFIG_t SLICE_OUTPUT_config =
	{
		.mode =				XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT3,
		.output_strength =	XMC_GPIO_OUTPUT_STRENGTH_MEDIUM,
		.output_level =		XMC_GPIO_OUTPUT_LEVEL_LOW,
	};

	// Enable CCU4 PWM output
	XMC_GPIO_Init(pPort, pin, &SLICE_OUTPUT_config);

	// Start of CCU4 configurations
	// Ensure fCCU reaches CCU40
	XMC_CCU4_SetModuleClock(pCCU, XMC_CCU4_CLOCK_SCU);

	// Enable clock, enable prescaler block and configure global control
	XMC_CCU4_Init(pCCU, XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR);

	// Get the slice out of idle mode
	XMC_CCU4_EnableClock(pCCU, sliceNumber);

	// Start the prescaler and restore clocks to slices
	XMC_CCU4_StartPrescaler(pCCU);

	// Initialize the Slice
	XMC_CCU4_SLICE_CompareInit(pSlice, &SLICE_config);

	// Set Period Value
	XMC_CCU4_SLICE_SetTimerPeriodMatch(pSlice, 0xFFFF);

	// Set PWM Value
	XMC_CCU4_SLICE_SetTimerCompareMatch(pSlice, 0xFFFF);

	// Enable shadow transfer
	XMC_CCU4_EnableShadowTransfer(pCCU, CCU4_SHADOW_TRANSFER_SLICES[sliceNumber]);

	// Start the Timer
	XMC_CCU4_SLICE_StartTimer(pSlice);
}

static void setCCU4X(XMC_CCU4_MODULE_t *pCCU, XMC_CCU4_SLICE_t *pSlice, uint8_t sliceNumber, uint16_t periodValue, uint16_t pwmValue)
{
	// Set Period Value
	XMC_CCU4_SLICE_SetTimerPeriodMatch(pSlice, periodValue);

	// Set PWM Value
	XMC_CCU4_SLICE_SetTimerCompareMatch(pSlice, pwmValue);

	// Enable shadow transfer
	XMC_CCU4_EnableShadowTransfer(pCCU, CCU4_SHADOW_TRANSFER_SLICES[sliceNumber]);
}






/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "fsl_port.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "fsl_adc16.h"

#include "pin_mux.h"
#include <stdbool.h>
#include "clock_config.h"

#define DEMO_ADC16_BASE ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U
#define DEMO_ADC16_USER_CHANNEL 4U /* PTB18, ADC0_SE4 */

#define DEMO_ADC16_IRQn ADC0_IRQn
#define DEMO_ADC16_IRQ_HANDLER_FUNC ADC0_IRQHandler

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ADCR_VDD (65535U) /* Maximum value when use 16b resolution */
#define V_BG (1000U)      /* BANDGAP voltage in mV (trim to 1.0V) */
#define V_TEMP25 (716U)   /* Typical VTEMP25 in mV */
#define M (1620U)         /* Typical slope: (mV x 1000)/oC */
#define STANDARD_TEMP (25U)
#define TASK_PRIO (configMAX_PRIORITIES - 1)
#define CONSUMER_LINE_SIZE 3
SemaphoreHandle_t xSemaphore_producer;
SemaphoreHandle_t xSemaphore_consumer;

volatile bool g_Adc16ConversionDoneFlag = false;

volatile uint32_t g_Adc16ConversionValue;
volatile uint32_t g_Adc16InterruptCounter;
int32_t adcrTemp25=0;
int32_t adcr100m=0;


static void adcmonitorfunc(void *parameters){
	for(;;){
	if (g_Adc16ConversionDoneFlag)
	        {
		int32_t currentTemperature = 0;
		    
		    currentTemperature = (int32_t)(STANDARD_TEMP - ((int32_t)g_Adc16ConversionValue - (int32_t)adcrTemp25) * 100 / (int32_t)adcr100m);
		    
		 PRINTF("ADC Value: currentTemperature\r\n");
			        PRINTF("ADC Interrupt Count: %d\r\n", g_Adc16InterruptCounter);
	        }

	        vTaskDelay(pdMS_TO_TICKS(10000));
	}

}


/*******************************************************************************
 * Code
 ******************************************************************************/


void DEMO_ADC16_IRQ_HANDLER_FUNC(void)
{
    g_Adc16ConversionDoneFlag = true;
    /* Read conversion result to clear the conversion completed flag. */
    g_Adc16ConversionValue = ADC16_GetChannelConversionValue(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP);
    g_Adc16InterruptCounter++;
}

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void producer_task(void *pvParameters);
static void consumer_task(void *pvParameters);
static volatile  bool buttnpress=false;
static void ledblinkfunc(void *parameters);
static void inputbtnfun(void * parameters);
static void adcmonitorfunc(void *parameters);

void BOARD_SW3_IRQ_HANDLER(void){
	PORT_ClearPinsInterruptFlags(PORTC, 1<<4);
	buttnpress=true;
}

static void ledblinkfunc(void *parameters){
	for (;;) {

		//PRINTF("Tick count: %ld\r\n", xTaskGetTickCount());
	        PRINTF("Health Check Status is complete \r\n");
	        //PRINTF("Tick count = %lu\r\n", (unsigned long)xTaskGetTickCount());

	        GPIO_TogglePinsOutput(GPIOC, 1U << 1U);
             //for(int i=0;i<600000;i++);
            // PRINTF("Tick=%lu\r\n", xTaskGetTickCount());
                 vTaskDelay(pdMS_TO_TICKS(5000));
	        // Delay for 500 ms
	        //vTaskDelay(pdMS_TO_TICKS(500));
	    }
}

static void inputbtnfun(void * parameters){

for(;;){
PRINTF("User Input Status\r\n");

if (buttnpress ){
	PRINTF("UserInput Recived\r\n");
	buttnpress=false;

}
vTaskDelay(pdMS_TO_TICKS(1000));
}
}

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{
    /* Init board hardware. */

	 adc16_config_t adc16ConfigStruct;
	    adc16_channel_config_t adc16ChannelConfigStruct;

    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    gpio_pin_config_t  ledcfg={
       			kGPIO_DigitalOutput,0

       	};
       	GPIO_PinInit(GPIOC, 1, &ledcfg);

       	gpio_pin_config_t buttoncfg={
       			kGPIO_DigitalInput,0
       	};
       	PORT_SetPinInterruptConfig(PORTC, 4, kPORT_InterruptFallingEdge);
       	GPIO_PinInit(GPIOC, 4, &buttoncfg);
       	EnableIRQ(BOARD_SW3_IRQ);
        EnableIRQ(DEMO_ADC16_IRQn);

           PRINTF("\r\nADC16 interrupt Example.\r\n");



           /*
            * adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
            * adc16ConfigStruct.clockSource = kADC16_ClockSourceAsynchronousClock;
            * adc16ConfigStruct.enableAsynchronousClock = true;
            * adc16ConfigStruct.clockDivider = kADC16_ClockDivider8;
            * adc16ConfigStruct.resolution = kADC16_ResolutionSE12Bit;
            * adc16ConfigStruct.longSampleMode = kADC16_LongSampleDisabled;
            * adc16ConfigStruct.enableHighSpeed = false;
            * adc16ConfigStruct.enableLowPower = false;
            * adc16ConfigStruct.enableContinuousConversion = false;
            */
           ADC16_GetDefaultConfig(&adc16ConfigStruct);
       #ifdef BOARD_ADC_USE_ALT_VREF
           adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
       #endif
           ADC16_Init(DEMO_ADC16_BASE, &adc16ConfigStruct);
           ADC16_EnableHardwareTrigger(DEMO_ADC16_BASE, false); /* Make sure the software trigger is used. */
       #if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
           if (kStatus_Success == ADC16_DoAutoCalibration(DEMO_ADC16_BASE))
           {
               PRINTF("ADC16_DoAutoCalibration() Done.\r\n");
           }
           else
           {
               PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
           }
       #endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */
           PRINTF("Press any key to get user channel's ADC value ...\r\n");

           adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
           adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = true; /* Enable the interrupt. */
       #if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
           adc16ChannelConfigStruct.enableDifferentialConversion = false;
       #endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */

           g_Adc16InterruptCounter = 0U;

           //while (1)
           //{
               //GETCHAR();
               g_Adc16ConversionDoneFlag = false;
               adcrTemp25 = 25810;
                  /* ADCR_100M = ADCR_VDD x M x 100 / VDD */
                  adcr100m = 5839;
               /*
                When in software trigger mode, each conversion would be launched once calling the "ADC16_ChannelConfigure()"
                function, which works like writing a conversion command and executing it. For another channel's conversion,
                just to change the "channelNumber" field in channel configuration structure, and call the function
                "ADC16_ChannelConfigure()"" again.
                Also, the "enableInterruptOnConversionCompleted" inside the channel configuration structure is a parameter for
                the conversion command. It takes affect just for the current conversion. If the interrupt is still required
                for the following conversion, it is necessary to assert the "enableInterruptOnConversionCompleted" every time
                for each command.
               */
               ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
    //xTaskCreate(producer_task, "PRODUCER_TASK", configMINIMAL_STACK_SIZE + 128, NULL, TASK_PRIO, NULL);
    /* Start scheduling. */
    xTaskCreate(ledblinkfunc,"blink",256,NULL,1,NULL);
               xTaskCreate(inputbtnfun,"btn",200,NULL,1,NULL);
              xTaskCreate(adcmonitorfunc,"adc",200+166,NULL,1,NULL);
               vTaskStartScheduler();
           for(;;)

           ;
   //}
}

/*!
 * @brief Task producer_task.
 */
static void producer_task(void *pvParameters)
{
    uint32_t i;
    
    PRINTF("Producer_task created.\r\n");
    xSemaphore_producer = xSemaphoreCreateBinary();
    if (xSemaphore_producer == NULL)
    {
        PRINTF("xSemaphore_producer creation failed.\r\n");
        vTaskSuspend(NULL);
    }

    xSemaphore_consumer = xSemaphoreCreateBinary();
    if (xSemaphore_consumer == NULL)
    {
        PRINTF("xSemaphore_consumer creation failed.\r\n");
        vTaskSuspend(NULL);
    }

    for (i = 0; i < CONSUMER_LINE_SIZE; i++)
    {
        xTaskCreate(consumer_task, "CONSUMER_TASK", configMINIMAL_STACK_SIZE, (void *)i, TASK_PRIO, NULL);
        PRINTF("Consumer_task %d created.\r\n", i);
    }

    while (1)
    {

    	 PRINTF("Tick count = %lu\r\n", (unsigned long)xTaskGetTickCount());

        /* Producer is ready to provide item. */
        xSemaphoreGive(xSemaphore_consumer);
        /* Producer is waiting when consumer will be ready to accept item. */
        if (xSemaphoreTake(xSemaphore_producer, portMAX_DELAY) == pdTRUE)
        {
            PRINTF("Producer released item.\r\n");
        }
        else
        {
            PRINTF("Producer is waiting for customer.\r\n");
        }
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

/*!
 * @brief Task consumer_task.
 */
static void consumer_task(void *pvParameters)
{
    PRINTF("Consumer number: %d\r\n", pvParameters);
    while (1)
    {
        /* Consumer is ready to accept. */
        xSemaphoreGive(xSemaphore_producer);
        /* Consumer is waiting when producer will be ready to produce item. */
        if (xSemaphoreTake(xSemaphore_consumer, portMAX_DELAY) == pdTRUE)
        {
            PRINTF("Consumer %d accepted item.\r\n", pvParameters);
        }
        else
        {
            PRINTF("Consumer %d is waiting for producer.\r\n", pvParameters);
        }
    }
}

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

#include "fsl_debug_console.h"
#include "fsl_smc.h"
#include "fsl_pmc.h"
#include "fsl_adc16.h"
#include "board.h"
#include "fsl_lptmr.h"

#include "pin_mux.h"
#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_ADC16_BASEADDR ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U

#define DEMO_ADC16_IRQ_ID ADC0_IRQn
#define DEMO_ADC16_IRQ_HANDLER_FUNC ADC0_IRQHandler

#define DEMO_LPTMR_BASE LPTMR0

/*
 * These values are used to get the temperature. DO NOT MODIFY
 * The method used in this demo to calculate temperature of chip is mapped to
 * Temperature Sensor for the HCS08 Microcontroller Family document (Document Number: AN3031)
 */
#define ADCR_VDD (65535U) /* Maximum value when use 16b resolution */
#define V_BG (1000U)      /* BANDGAP voltage in mV (trim to 1.0V) */
#define V_TEMP25 (716U)   /* Typical VTEMP25 in mV */
#define M (1620U)         /* Typical slope: (mV x 1000)/oC */
#define STANDARD_TEMP (25U)

#define LED1_INIT() LED_RED_INIT(LOGIC_LED_OFF)
#define LED1_ON() LED_RED_ON()
#define LED1_OFF() LED_RED_OFF()

#define LED2_INIT() LED_GREEN_INIT(LOGIC_LED_OFF)
#define LED2_ON() LED_GREEN_ON()
#define LED2_OFF() LED_GREEN_OFF()
#define kAdcChannelTemperature (26U) /*! ADC channel of temperature sensor */
#define kAdcChannelBandgap (27U)     /*! ADC channel of BANDGAP */

#define UPPER_VALUE_LIMIT (1U) /*! This value/10 is going to be added to current Temp to set the upper boundary*/
#define LOWER_VALUE_LIMIT                                                               \
    (1U) /*! This Value/10 is going to be subtracted from current Temp to set the lower \
            boundary*/
#define UPDATE_BOUNDARIES_TIME                                                       \
    (20U) /*! This value indicates the number of cycles needed to update boundaries. \
              To know the Time it will take, multiply this value times LPTMR_COMPARE_VALUE*/

#define LPTMR_COMPARE_VALUE (700U) /* Low Power Timer interrupt time in miliseconds */

/*!
* @brief Boundaries struct
*/
typedef struct lowPowerAdcBoundaries
{
    int32_t upperBoundary; /*! upper boundary in degree */
    int32_t lowerBoundary; /*! lower boundary in degree */
} lowPowerAdcBoundaries_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/


/*!
 * @brief calibrate parameters: VDD and ADCR_TEMP25
 *
 * @param base The ADC instance number
 */
static void ADC16_CalibrateParams(ADC_Type *base);


/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile static uint32_t adcValue = 0; /*! ADC value */
static uint32_t adcrTemp25 = 0;        /*! Calibrated ADCR_TEMP25 */
static uint32_t adcr100m = 0;
volatile bool conversionCompleted = false; /*! Conversion is completed Flag */

/*******************************************************************************
 * Code
 ******************************************************************************/




static void ADC16CalibratetParams(ADC_Type *ADC){

	uint32_t pmcbandgappvalue=0;
	adc16_channel_config_t chnlcfg;
	adc16_config_t config;


	pmc_bandgap_buffer_config_t pmcbfrcfg;

#if defined(FSL_FEATURE_PMC_HAS_BGEN) && (FSL_FEATURE_PMC_HAS_BGEN)
	pmcbfcfg.enableInLowPowerMode=True;
#endif
#if defined(FSL_FEATURE_PMC_HAS_BGBDS) && (FSL_FEATURE_PMC_HAS_BGBDS)
	pmcbfrcfg.drive=kPMC_BandgapBufferDriveLow;
#endif
PMC_ConfigureBandgapBuffer(PMC, &pmcbfrcfg);

ADC16_GetDefaultConfig(&config);
#if defined(FSL_FEATURE_ADC16_MAX_RESOLUTION)&&FSL_FEATURE_ADC16_MAX_RESOLUTION
	config.resolution=kADC16_Resolution16Bit;
#endif
config.enableLowPower=true;
config.enableContinuousConversion=false;
config.enableHighSpeed=true;
config.longSampleMode=kADC16_LongSampleCycle24;

#if defined(BOARD_ADC_USE_ALT_VREF)&&BOARD_ADC_USE_ALT_VREF
  config.clockSource=kADC16_ReferenceVoltageSourceVref;
#endif
ADC16_Init(base, &config);
chnlcfg.channelNumber=27;
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
  ADC16_DoAutoCalibration(ADC);
#endif

#if defined(FSL_FEATURE_ADC16_HAS_HW_AVERAGE)&&FSL_FEATURE_ADC16_HAS_HW_AVERAGE
  ADC16_SetHardwareAverage(ADC, kADC16_HardwareAverageCount32);

#endif
#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) &&FSL_FEATURE_ADC16_HAS_DIFF_MODE
chnlcfg.enableDifferentialConversion=false;
#endif
chnlcfg.enableInterruptOnConversionCompleted=false;

ADC16_SetChannelConfig(ADC, 0U, &chnlcfg);

while(! ADC16_GetChannelStatusFlags(ADC, 0U)&& timeout--)
{

}

pmcbandgappvalue=ADC16_GetChannelConversionValue(ADC, 0U);
//ADC16_PauseConversion(ADC);

//calculation formula

pmcbfrcfg.enable=false;
PMC_ConfigureBandgapBuffer(PMC, &pmcbfrcfg);
}



/*!
 * @brief main function
 */
int main(void)
{


    /* Init hardware */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    /* Init using Led in Demo app */



    /* Set to allow entering vlps mode */
    SMC_SetPowerModeProtection(SMC, kSMC_AllowPowerModeVlp);

    /* Calibrate param Temperature sensor */
    ADC16CalibratetParams(DEMO_ADC16_BASEADDR);

    /* Initialize Demo ADC */

}

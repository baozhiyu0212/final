/**************************************************************************//**
 * @file
 * @brief Empty Project
 * @author Energy Micro AS
 * @version 3.20.2
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silicon Labs Software License Agreement. See 
 * "http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt"  
 * for details. Before using this software for any purpose, you must agree to the 
 * terms of that agreement.
 *
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_int.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "bsp.h"
#include "em_letimer.h"
#include "em_gpio.h"
#include "bsp_trace.h"
#include "em_acmp.h"
#include "em_timer.h"
#include "em_dma.h"
#include "em_adc.h"
#include "dmactrl.h"
#include "em_i2c.h"
#include "em_leuart.h"

#define adc_start					0x01
#define adc_stop                    0x01 << 1

#define ADCSAMPLES                  150

#define timer_start                 0x01
#define timer_stop                  0x02
#define timer_counts_HPMdelay    	28000000			// counts corresponding to delay of 2 seconds ( required to measure heart rate)


int LOplus;
int LOminus;
int Dstbuffer[500];
int j;

void CMU_Setup(void);
void GPIO_Setup(void);
void setupAdc(void);
void SetupTimer(void);

void CMU_Setup(void)
	{
		CMU_ClockEnable(cmuClock_GPIO, true);
		CMU_ClockEnable(cmuClock_ADC0, true);
		CMU_ClockEnable(cmuClock_TIMER0, true);


	}


void GPIO_Setup(void)
	{
  	GPIO_PinModeSet (gpioPortB, 11, gpioModeInput, 1);        // LO+
  	GPIO_PinModeSet (gpioPortB, 12, gpioModeInput, 1);        // LO-
  	GPIO_PinModeSet (gpioPortD, 6, gpioModeInput, 1);        // OUT

	}


void setupAdc(void)
{
  ADC_Init_TypeDef        adcInit       = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef  adcInitSingle = ADC_INITSINGLE_DEFAULT;


  adcInit.timebase = ADC_TimebaseCalc(0);
  adcInit.prescale = ADC_PrescaleCalc(10, 0);                     //Calculate prescalar for desired value Tconv = (N+Ta)*P/14M
  ADC_Init(ADC0, &adcInit);

  adcInitSingle.input     =  adcSingleInpCh6;                         /* Select input to ADC */
  adcInitSingle.reference    = adcRef2V5;                             /* Reference voltage */
  adcInitSingle.rep      = true;
  adcInitSingle.acqTime	  = adcAcqTime256;
  ADC_InitSingle(ADC0, &adcInitSingle);

}

void SetupTimer(void)
{
TIMER_Init_TypeDef timer0 =												// initialize timer for using a delay to let TSL to stabilize on being powered up.
	     {
	       .enable     = false,
	       .debugRun   = false,
	       .prescale   = timerPrescale1024,
	       .clkSel     = timerClkSelHFPerClk,
	       .fallAction = timerInputActionNone,
	       .riseAction = timerInputActionNone,
	       .mode       = timerModeUp,
	       .dmaClrAct  = false,
	       .quadModeX4 = false,
	       .oneShot    = true,
	       .sync       = false,
	     };

TIMER_Init(TIMER0, &timer0);

//TIMER_TopSet(&timer0, 14000000);

//TIMER0->CNT = 0x00;
TIMER0->CMD = timer_stop;												// insuring timer is off before counting starts





}


/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Chip errata */
  CHIP_Init();

	CMU_Setup();
	GPIO_Setup();
	setupAdc();
  /* Infinite loop */
  while (1) {

	  LOplus = GPIO_PinInGet(gpioPortB, 11);
	  LOminus = GPIO_PinInGet(gpioPortB, 12);

	  ADC0->CMD |= adc_start;                                                  // Enable ADC
	  TIMER0->CMD = timer_start;

	 //while (TIMER0->CNT <= timer_counts_HPMdelay)

	  for ( j = 0; j < ADCSAMPLES ; j++)
	  {
	  			while (!(ADC0->STATUS & ADC_STATUS_SINGLEDV));
	  			Dstbuffer[j] = ADC0->SINGLEDATA;


	  		}



		  ADC0->CMD |= adc_stop;                                           // Disable ADC
		  TIMER0->CMD = timer_stop;




  }
}

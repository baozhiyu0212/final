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

//#include "em_device.h"

//#include "em_chip.h"



/**************************************************************************//**

 * @brief  Main function

 *****************************************************************************/





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

#include "myProject_1.h"













uint32_t sleep_block_counter[EM4+1];



int Interval,a,b,h,count,heartbeat_count,timer_count;



float osc_ratio;

int ULFRCO_Calibrated, LETIMER0_prescalar;

int iflag;



int8_t string4[40];







void Sleep(void);

void blockSleepMode(uint32_t Minimummode);

void unblockSleepMode(uint32_t Minimummode);

void CMU_Setup(void);

void GPIO_Setup(void);

void LETIMER0_Setup(void);

static void ACMPInit(void);



/***************************************************************************/ /**

  * Sleep routines

 ******************************************************************************* */









/* This code is originally Silicon Labs and copyrighted by Silicon Labs in 2015 and Silicon Labs grants

 * permission to anyone to use the software for any purpose, including commercial applications, and to alter it,

 * and to redistribute it freely subject that the origin is not miss represented, altered source version must be

 * plainly marked, and this notice cannot be altered or removed from any source distribution.

 *

 * Routine include :

 *

 * void blockSleepMode(uint32_t Minimummode)

 * void unblockSleepMode(uint32_t Minimummode)

 * void(Sleep)void

*/



void blockSleepMode(uint32_t Minimummode){              // block an energy mode so that it does not go lower

//INT_Disable();

sleep_block_counter[Minimummode]++;

//INT_Enable();

}



void unblockSleepMode(uint32_t Minimummode){            // unblock an energy mode after operation completes

//INT_Disable();

if (sleep_block_counter[Minimummode]>0){

sleep_block_counter[Minimummode]--;

}

else sleep_block_counter[Minimummode] = 0;

//INT_Enable();

}







void Sleep(void){                                               // enter EMX sleep routine

if (sleep_block_counter[EM0] > 0){}

else if (sleep_block_counter[EM1] > 0)

EMU_EnterEM1();

else if (sleep_block_counter[EM2] > 0)

EMU_EnterEM2(true);

else if (sleep_block_counter[EM3] > 0)

EMU_EnterEM3(true);



}





//Peripheral functions

/***************************************************************************/ /**

  * CMU Setup

 ******************************************************************************* */

void CMU_Setup(void){





CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

CMU_ClockEnable(cmuClock_CORELE, true);

CMU_ClockEnable(cmuClock_HFPER, true);

CMU_ClockEnable(cmuClock_TIMER0, true);

CMU_ClockEnable(cmuClock_TIMER1, true);

CMU_ClockEnable(cmuClock_LETIMER0, true);

CMU_ClockEnable(cmuClock_GPIO, true);

CMU_ClockEnable(cmuClock_ACMP0, true);

CMU_ClockEnable(cmuClock_DMA, true);

CMU_ClockEnable(cmuClock_ADC0, true);

CMU_ClockEnable(cmuClock_I2C1, true);







if (LETIMER0_EM_Mode == EM3)

{





CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);               // Select ULFRCO for EM3

//CMU_OscillatorEnable(cmuOsc_LFXO, false, false);





}

//else CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);                // Select LFXO for all other modes





}





void GPIO_Setup(void)

{

  GPIO_PinModeSet (gpioPortB, 11, gpioModeInput, 1);        // LO+

  GPIO_PinModeSet (gpioPortB, 12, gpioModeInput, 1);        // LO-

  GPIO_PinModeSet (gpioPortC, 0, gpioModeInput, 1);        // OUT



}





/***************************************************************************/ /**

  * ACMP initialization

 ******************************************************************************* */

void ACMPInit(void)

{

  const ACMP_Init_TypeDef acmp_init =

  {

    false,                              /* Full bias current*/

    true,                              /* Half bias current */

    7,                                  /* Biasprog current configuration */

    false,                               /* Enable interrupt for falling edge */

    true,                               /* Enable interrupt for rising edge */

    acmpWarmTime256,                    /* Warm-up time in clock cycles, >140 cycles for 10us with 14MHz */

    acmpHysteresisLevel0,               /* Hysteresis configuration */

    0,                                  /* Inactive comparator output value */

    false,                              /* Enable low power mode */

    0x30,                                  /* Vdd reference scaling */

    false,                               /* Enable ACMP */

  };



  ACMP_Init(ACMP0, &acmp_init);

  ACMP_ChannelSet(ACMP0, acmpChannel2V5, acmpChannel0);

  ACMP0->IEN |= ACMP_IEN_EDGE;



 	NVIC_EnableIRQ(ACMP0_IRQn);





 }

















void LETIMER0_Setup(void){



   LETIMER_Init_TypeDef LETIMER0_init;







   int Comp0_init;

   int Comp1_int_init;



   // LETIMER initialization

   LETIMER0_init.bufTop = false;                 // do not load COMP! into COMP0 when REP0 = 0

   LETIMER0_init.comp0Top = true;                // load COMP0 into CNT on underflow

   LETIMER0_init.debugRun = false;               // stop counting when debug stops

   LETIMER0_init.enable = false;

   LETIMER0_init.out0Pol = 0;                    // idle output 0

   LETIMER0_init.out1Pol = 1;

   LETIMER0_init.repMode = letimerRepeatFree;    // count until stopped by software

   LETIMER0_init.rtcComp0Enable = false;        // do not start counting on COMP0 RTC match

   LETIMER0_init.rtcComp1Enable = false;       // do not start counting on COMP1 RTC match

   // underflow output actions

   LETIMER0_init.ufoa0 = letimerUFOANone;

   LETIMER0_init.ufoa1 = letimerUFOANone;





       // initialize period in ULFRCO for EM3 and LFXO for all other energy modes

   LETIMER_Init(LETIMER0, &LETIMER0_init);







Comp0_init = LETIMER0_cmp0ontime * (LETIMER0_ULFRCO_count);









   LETIMER0->CNT = Comp0_init;



   LETIMER_CompareSet(LETIMER0,0,Comp0_init);









   Comp1_int_init = LETIMER0_LEDontime * LETIMER0_ULFRCO_count ;









   LETIMER_CompareSet(LETIMER0,1,Comp1_int_init);                // set compare register value ( pointer , compare register 0 to set, initialization value)





   while((LETIMER0->SYNCBUSY) != 0);                             //wait till synchronization bit goes low



   // setting corresponding flag bits in LETIMER_IEN

   LETIMER0->IEN = LETIMER_IEN_UF;



   blockSleepMode(LETIMER0_EM_Mode);



   NVIC_EnableIRQ(LETIMER0_IRQn);



   }





















  /**************************************************************************//**

   * @brief ACMP0 Interrupt handler

   *****************************************************************************/

  void ACMP0_IRQHandler(void)

  {

    /* Clear interrupt flag */

    ACMP0->IFC = ACMP_IFC_EDGE;



count = count + 1;//For a high ACMP output turn off LED





  }





  void LETIMER0_IRQHandler(void){

  int currentFlags;

  currentFlags = LETIMER0->IF;                                                          // save IF register contents/state in a variable

  LETIMER0->IFC = currentFlags;                                                         // Interrupt flag clear register



if ((currentFlags & LETIMER_IF_UF) != 0)

{



timer_count++;



if(timer_count == 500)

{

  heartbeat_count = 4*count;

  ACMP0->CTRL &= ~ACMP_CTRL_EN;                                     // Turn off ACMP

}





}



  }















  int main(void)

  {

  CHIP_Init();



CMU_Setup();

GPIO_Setup();

ACMPInit();

LETIMER0_Setup();





//ACMP0->CTRL |= ACMP_CTRL_EN;

/* Wait for warmup */

//while (!(ACMP0->STATUS & ACMP_STATUS_ACMPACT)) ;





LETIMER_Enable(LETIMER0, true);

ACMP0->CTRL |= ACMP_CTRL_EN;



      /* Wait for warmup */

         while (!(ACMP0->STATUS & ACMP_STATUS_ACMPACT)) ;

while(1)

{





//ACMP0->CTRL &= ~ACMP_CTRL_EN;

}









  }


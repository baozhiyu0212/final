


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



uint32_t i2c_txBuffer1;								// variables for storing ADC values of two ADCs from TSL2561 device
uint32_t i2c_txBuffer2;								// variables for storing ADC values of two ADCs from TSL2561 device
uint32_t i2c_txBuffer3;
uint32_t i2c_txBuffer4;
uint32_t i2c_txBuffer5;
uint32_t i2c_txBuffer6;								// variables for storing ADC values of two ADCs from TSL2561 device





uint32_t sleep_block_counter[EM4+1];

int Interval,a,b,h,fl;




float osc_ratio;
int ULFRCO_Calibrated;
int8_t string4[40];



void LETIMER0_Calibration(void);
void Sleep(void);
void blockSleepMode(uint32_t Minimummode);
void unblockSleepMode(uint32_t Minimummode);
void CMU_Setup(void);
void GPIO_Setup(void);
void LETIMER0_Setup(void);
void I2CSetup(void);


void performi2ctransfer(void);
void writetoI2C(int reg_address, int data);
void readfromI2C(uint32_t read_regaddress);


void powerup(void);


//void LED_state(int led, bool state);
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


}


/***************************************************************************/ /**
  * GPIO Setup
 ******************************************************************************* */

void GPIO_Setup(void){

	GPIO_PinModeSet(gpioPortC, 5, gpioModeWiredAndPullUpFilter, 1);			/*GPIO pins for I2C - SDA and SCL*/
	    GPIO_PinModeSet(gpioPortC, 4, gpioModeWiredAndPullUpFilter, 1);
	  //  GPIO_PinModeSet(gpioPortD, 1, gpioModeInput, 1);



  }


/***************************************************************************/ /**
  * Powerup routine
 ******************************************************************************* */


void powerup(void)
{


	/*  INT 6
	 * 3V3  4 - PD0
	 * GND  19
	 * SCL  9
	 * SDA  7
	 */

	GPIO_PinModeSet(port_i2c_power, 0, gpioModePushPull, 0);				// enable power to TSL2561
	    GPIO_PinOutSet(port_i2c_power, 0);


	TIMER_Init_TypeDef timer0 =												// initialize timer for using a delay to let TSL to stabilize on being powered up.
		     {
		       .enable     = false,
		       .debugRun   = false,
		       .prescale   = timerPrescale1,
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

	TIMER0->CNT = 0x00;
	TIMER0->CMD = timer_stop;												// insuring timer is off before counting starts

	TIMER0->CMD = timer_start;

	while (TIMER0->CNT <= timer_counts_TSLdelay);

	TIMER0->CMD = timer_stop;



}


/***************************************************************************/ /**
  * Power down routine
 ******************************************************************************* */


void powerdown()
{

	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	GPIO_PinModeSet(port_i2c_SCL, 5, gpioModeDisabled, 0);				/*GPIO pins for I2C - SDA and SCL*/
    GPIO_PinModeSet(port_i2c_SDA, 4, gpioModeDisabled, 0);				//SDA pin configured
  //	GPIO_IntConfig(port_i2c_interrupt, 1, false, true, false);			//configure external interrupt
  //	GPIO_PinModeSet(port_i2c_interrupt, 1, gpioModeDisabled, 0);
    GPIO_PinModeSet(port_i2c_power, 0, gpioModeDisabled, 0);			// disable power to TSL2561
    GPIO_PinOutClear(port_i2c_power, 0);

       //	GPIO_PinModeSet(gpioPortD, 1, gpioModeDisabled, 1);
       	//	      GPIO_IntConfig( gpioPortD, 1, false, true, false );


}





















void I2CSetup(void)
{


	I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;

	I2C_Init(I2C1, &i2cInit);

	if(I2C1->STATE & I2C_STATE_BUSY){
	I2C1->CMD = I2C_CMD_ABORT ;
	}

	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_I2C1, true);
    CMU_ClockEnable(cmuClock_GPIO, true);


	  GPIO_PinModeSet(gpioPortC, 5, gpioModeWiredAndPullUpFilter, 1);
	  GPIO_PinModeSet(gpioPortC, 4, gpioModeWiredAndPullUpFilter, 1);
      GPIO_PinModeSet(gpioPortD, 1, gpioModeInput, 1);


	for (int i = 0; i < 9; i++)
	  {
	    /*
	     * TBD: Seems to be clocking at appr 80kHz-120kHz depending on compiler
	     * optimization when running at 14MHz. A bit high for standard mode devices,
	     * but DVK only has fast mode devices. Need however to add some time
	     * measurement in order to not be dependable on frequency and code executed.
	     */
	    GPIO_PinModeSet(gpioPortC, 5, gpioModeWiredAnd, 0);
	    GPIO_PinModeSet(gpioPortC, 5, gpioModeWiredAnd, 1);
	  }

    I2C1->ROUTE = I2C_ROUTE_SDAPEN |										// route pins used for I2c buses SDA and SCL
    	              I2C_ROUTE_SCLPEN |
    	       (I2C_ROUTE_LOCATION_LOC0);

    if(I2C1->STATE & I2C_STATE_BUSY){										// reset registers and device
    	I2C1->CMD = I2C_CMD_ABORT ;
    	}

    int fl = I2C1->IF;
        I2C1->IFC = fl;



}




/***************************************************************************/ /**
  * I2C Transfer
 ******************************************************************************* */











void performi2ctransfer(void)
{
	blockSleepMode(EM1);

    I2C1->IEN = 0x1AFF;

     // data out rate

   writetoI2C(0x2A, 0x30);

    //  register scale set

   writetoI2C(0x0E, 0x01);

   // active mode


   writetoI2C(0x2A, 0x31);

   unblockSleepMode(EM1);
}







void writetoI2C(int reg_address, int data)
{
	 I2C1->CMD = 0x01;
	 I2C1->TXDATA = 0x3A;
	 while(! (I2C1->IF & I2C_IF_ACK));
	 fl = (I2C1->IF & I2C_IF_ACK);
	 I2C1->IFC = fl;

	 I2C1->TXDATA = reg_address;
	 while(! (I2C1->IF & I2C_IF_ACK));
	 fl = (I2C1->IF & I2C_IF_ACK);
     I2C1->IFC = fl;

     I2C1->TXDATA = data;
     while(! (I2C1->IF & I2C_IF_ACK));
     fl = (I2C1->IF & I2C_IF_ACK);
     I2C1->IFC = fl;

     I2C1->CMD = 0x02;
}


void readfromI2C(uint32_t read_regaddress)
{
	blockSleepMode(EM1);

	I2C1->TXDATA = 0x3A;
	I2C1->CMD = 0x01;
	while(! (I2C1->IF & I2C_IF_ACK));
	fl = (I2C1->IF & I2C_IF_ACK);
	I2C1->IFC = fl;


	I2C1->TXDATA = read_regaddress;
	I2C1->CMD = 0x01;
	while(! (I2C1->IF & I2C_IF_ACK));
	fl = (I2C1->IF & I2C_IF_ACK);
	I2C1->IFC = fl;



	I2C1->TXDATA = 0x3B;
	while(! (I2C1->IF & I2C_IF_ACK));
	fl = (I2C1->IF & I2C_IF_ACK);
	I2C1->IFC = fl;



	while (!(I2C1->IF & I2C_IF_RXDATAV));
	i2c_txBuffer1 = I2C1->RXDATA;
	I2C1->CMD |= I2C_CMD_ACK;

	while (!(I2C1->IF & I2C_IF_RXDATAV));
	i2c_txBuffer2 = I2C1->RXDATA;
	I2C1->CMD |= I2C_CMD_ACK;

	while (!(I2C1->IF & I2C_IF_RXDATAV));
		i2c_txBuffer3 = I2C1->RXDATA;
		I2C1->CMD |= I2C_CMD_ACK;

	while (!(I2C1->IF & I2C_IF_RXDATAV));
		i2c_txBuffer4 = I2C1->RXDATA;
		I2C1->CMD |= I2C_CMD_ACK;

	while (!(I2C1->IF & I2C_IF_RXDATAV));
		i2c_txBuffer5 = I2C1->RXDATA;
		I2C1->CMD |= I2C_CMD_ACK;

	  while (!(I2C1->IF & I2C_IF_RXDATAV));
	  i2c_txBuffer6 = I2C1->RXDATA;
	  I2C1->CMD |= I2C_CMD_NACK;
	  I2C1->CMD = 0x02;

	  unblockSleepMode(EM1);
}























 int main(void){
	//int ULFRCO_Calibrated;

	CHIP_Init(); // initialize all errata


	blockSleepMode(EM3);


	CMU_Setup();
	GPIO_Setup();


	powerup();
	I2CSetup();												// do i2c setup, configure TSL2561 registers and monitor if interrupt occurs
	performi2ctransfer();

	readfromI2C(0x01);

	powerdown();










	while(1){



	}
}




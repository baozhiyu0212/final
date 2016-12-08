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


void CMU_Setup(void);
void GPIO_Setup(void);
void I2CSetup(void);

int fl;
uint32_t i2c_txBuffer2;								// variables for storing ADC values of two ADCs from TSL2561 device
uint32_t i2c_txBuffer3;
uint32_t i2c_txBuffer4;
uint32_t i2c_txBuffer5;


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


void GPIO_Setup(void)
{



    GPIO_PinModeSet(gpioPortC, 5, gpioModeWiredAndPullUpFilter, 1);			/*GPIO pins for I2C - SDA and SCL*/
    GPIO_PinModeSet(gpioPortC, 4, gpioModeWiredAndPullUpFilter, 1);
  //  GPIO_PinModeSet(gpioPortD, 1, gpioModeInput, 1);




          //  	  GPIO_ExtIntConfig( gpioPortD, 1, 1, true, false, true );
          //       NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
          //        NVIC_EnableIRQ(GPIO_ODD_IRQn);
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

int main(void)
{


	CMU_Setup();
	GPIO_Setup();
	I2CSetup();



	int fl = I2C1->IF;

    I2C1->IFC = fl;

    I2C1->IEN = 0x1AFF;





//fl = (I2C1->IF & I2C_IF_ACK);

//I2C1->IFC = fl;

        I2C1->TXDATA = 0x3A;
        I2C1->CMD = 0x01;


        while(! (I2C1->IF & I2C_IF_ACK));

        fl = (I2C1->IF & I2C_IF_ACK);
        I2C1->IFC = fl;


        I2C1->TXDATA = 0x0D;
        while(! (I2C1->IF & I2C_IF_ACK));

        fl = (I2C1->IF & I2C_IF_ACK);
               I2C1->IFC = fl;

         I2C1->CMD = 0x01;

         I2C1->TXDATA = 0x3B;
         while(! (I2C1->IF & I2C_IF_ACK));

         fl = (I2C1->IF & I2C_IF_ACK);
         I2C1->IFC = fl;

         while (!(I2C1->STATUS & I2C_STATUS_RXDATAV));
         i2c_txBuffer2 = I2C1->RXDATA;

  /*       I2C1->CMD |= I2C_CMD_ACK;

         while (!(I2C1->IF & I2C_IF_RXDATAV));
                  i2c_txBuffer3 = I2C1->RXDATA;

                  I2C1->CMD |= I2C_CMD_ACK;

         while (!(I2C1->IF & I2C_IF_RXDATAV));
                i2c_txBuffer4 = I2C1->RXDATA;

                I2C1->CMD |= I2C_CMD_ACK;

         while (!(I2C1->IF & I2C_IF_RXDATAV));
               i2c_txBuffer5 = I2C1->RXDATA;			*/

               I2C1->CMD |= I2C_CMD_NACK;

        I2C1->CMD = 0x02;

}

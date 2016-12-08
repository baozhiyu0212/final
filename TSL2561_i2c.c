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





DMA_CB_TypeDef cb;



/* Transfer Flag */

volatile bool transferActive;



/* ADC Transfer Data */



volatile uint16_t DstPtr[ADCSAMPLES];               //buffer for storing temperature readings with DMA

volatile uint16_t Dstbuffer[ADCSAMPLES];	//buffer for storing temperature readings without DMA



uint32_t i2c_txBuffer1;	// variables for storing ADC values of two ADCs from TSL2561 device

uint32_t i2c_txBuffer2;	// variables for storing ADC values of two ADCs from TSL2561 device

uint32_t i2c_txBuffer3;

uint32_t i2c_txBuffer4;

uint32_t i2c_txBuffer5;

uint32_t i2c_txBuffer6;	// variables for storing ADC values of two ADCs from TSL2561 device





uint32_t tsl_txBuffer2;

uint32_t tsl_txBuffer3;

uint32_t tsl_txBuffer4;

uint32_t tsl_txBuffer5;









uint32_t sleep_block_counter[EM4+1];



int Interval,a,b,c,d,h,fl,count,heartbeat_count,timer_count;

int txcount= 0;

int irfcount = 0;



int sumI, sign;

int sumF;



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

static void ACMPInit(void);

void DMA_ACTIVATE(void);

void setupAdc(void);

float convertToCelsius(int32_t adcSample);



void ClearTSLInterrupt_TSL(void);

void I2CSetup_TSL(void);

void ClearACK();

void writetoTXDATA ( int n );





void I2CSetup_MMA(void);

void performi2ctransfer_MMA(void);

void writetoI2C(int reg_address, int data);

void readfromI2C_MMA(uint32_t read_regaddress);

void performI2CTransferAndMonitor_TSL(void);



void powerup(void);

void powerdown(void);





void initLeuart(void);

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

CMU_ClockEnable(cmuClock_I2C0, true);









if (LETIMER0_EM_Mode == EM3)

{

if(CALIBRATION == 1)

{

LETIMER0_Calibration();                                           // Enter calibration if enable in constants

}



CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);               // Select ULFRCO for EM3

CMU_OscillatorEnable(cmuOsc_LFXO, false, false);





}

else CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);                // Select LFXO for all other modes





}





/***************************************************************************/ /**

  * GPIO Setup

 ******************************************************************************* */



void GPIO_Setup(void){



  GPIO_PinModeSet (port_internal_tempsensor, 6, gpioModeInput, 1);

  GPIO_PinModeSet (port_ADC_excite, 6, gpioModePushPull, 0);



  GPIO_PinModeSet(gpioPortC, 5, gpioModeWiredAndPullUpFilter, 1);	/*GPIO pins for I2C - SDA and SCL*/

GPIO_PinModeSet(gpioPortC, 4, gpioModeWiredAndPullUpFilter, 1);



GPIO_PinModeSet(gpioPortD, 7, gpioModeWiredAndPullUpFilter, 1);	/*GPIO pins for I2C - SDA and SCL*/

    GPIO_PinModeSet(gpioPortD, 6, gpioModeWiredAndPullUpFilter, 1);



    GPIO_PinModeSet (port_LED , 3, gpioModePushPullDrive, 0);

GPIO_PinModeSet (port_LED , 2, gpioModePushPullDrive, 0);



  GPIO_PinModeSet (gpioPortB, 11, gpioModeInput, 1);        // LO+

  GPIO_PinModeSet (gpioPortB, 12, gpioModeInput, 1);        // LO-

  GPIO_PinModeSet (gpioPortC, 0, gpioModeInput, 1);        // OUT













  }





/***************************************************************************/ /**

  * Powerup routine

 ******************************************************************************* */





void powerup(void)

{





/*  MMA

*

* 3V3  4 - PD0

* GND  19

* SCL  9

* SDA  7

*

*

*

*

*

*

* 224 in i2cbuffer which is OUT_Z_MSB is the threshold value for alert

*/







/*  TSL

*

*

* INT 6 - PD1

* 3V3 8 - PD2

* GND

* SCL  PD7 - 17

* SDA  PD6 - 16

*

*/









GPIO_PinModeSet(port_MMA_power, 0, gpioModePushPull, 0);	// enable power to MMA

GPIO_PinOutSet(port_MMA_power, 0);



GPIO_PinModeSet(port_TSL_power, 4, gpioModePushPull, 0);	// enable power to MMA

GPIO_PinOutSet(port_TSL_power, 4);



TIMER_Init_TypeDef timer0 =	// initialize timer for using a delay to let TSL to stabilize on being powered up.

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

TIMER0->CMD = timer_stop;	// insuring timer is off before counting starts



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

GPIO_PinModeSet(port_MMA_SCL, 5, gpioModeDisabled, 0);	/*GPIO pins for MMA I2C - SDA and SCL*/

    GPIO_PinModeSet(port_MMA_SDA, 4, gpioModeDisabled, 0);	//SDA pin configured



GPIO_PinModeSet(port_TSL_SCL, 7, gpioModeDisabled, 0);	/*GPIO pins for TSL I2C - SDA and SCL*/

    GPIO_PinModeSet(port_TSL_SDA, 6, gpioModeDisabled, 0);	//SDA pin configured





  GPIO_IntConfig(port_i2c_interrupt, 1, false, true, false);	//disable external interrupt

  GPIO_PinModeSet(port_i2c_interrupt, 1, gpioModeDisabled, 0);



    GPIO_PinModeSet(port_MMA_power, 0, gpioModeDisabled, 0);	// disable power to MMA

    GPIO_PinOutClear(port_MMA_power, 0);



    GPIO_PinModeSet(port_TSL_power, 4, gpioModeDisabled, 0);	// disable power to TSL2561

    GPIO_PinOutClear(port_TSL_power, 4);

       //	GPIO_PinModeSet(gpioPortD, 1, gpioModeDisabled, 1);

       //	      GPIO_IntConfig( gpioPortD, 1, false, true, false );





}









/**************************************************************************//**

TSL Interrupt Clearing Routine

 *****************************************************************************/



void ClearTSLInterrupt_TSL(void)

{







  I2C0->TXDATA = TSLAddr_write;	// give address to slave fr clearing interrut on TSL

  I2C0->CMD = I2C_START;





  while(! (I2C0->IF & I2C_IF_ACK));



  ClearACK();



     writetoTXDATA( TSL_clearinterrupt );	// clearing TSL interrupt by clearing bit in command register





     I2C0->CMD = I2C_STOP;



     ClearACK();









}





























/**************************************************************************//**

GPIO Interrupt Handling Routine

 *****************************************************************************/

void GPIO_ODD_IRQHandler(void)

   {

INT_Disable();	// disable or clear all interrupts



GPIO_IntClear(gpioclear_extint);



blockSleepMode(EM1);	// enter energy mode EM1 on starting communication with the slave device





I2C0->TXDATA = TSLAddr_write;	// send address of device

    I2C0->CMD = I2C_START;	// send start signal



    while(! (I2C0->IF & I2C_IF_ACK));	// wait for ACK



    ClearACK();	// clear ACK after receipt



    writetoTXDATA( Addr_ADCval );	// access ADC registers on TSL2561 through command reg



    I2C0->CMD = I2C_START;	// send a repeat start

    writetoTXDATA( TSLAddr_read );	// send device address with LSB = 1 for read



    while (!(I2C0->IF & I2C_IF_RXDATAV));	// start reading

    tsl_txBuffer2 = I2C0->RXDATA;



    I2C0->CMD |= I2C_CMD_ACK;



    while (!(I2C0->IF & I2C_IF_RXDATAV));

    tsl_txBuffer3 = I2C0->RXDATA;



    I2C0->CMD |= I2C_CMD_ACK;



    while (!(I2C0->IF & I2C_IF_RXDATAV));

    tsl_txBuffer4 = I2C0->RXDATA;



    I2C0->CMD |= I2C_CMD_ACK;



    while (!(I2C0->IF & I2C_IF_RXDATAV));

    tsl_txBuffer5 = I2C0->RXDATA;



    I2C0->CMD |= I2C_CMD_NACK;



    I2C0->CMD = I2C_STOP;



    unblockSleepMode(EM1);





           a = 256*tsl_txBuffer3 + tsl_txBuffer2;	// calculate output of ADC0 and compare

                 if (a<15 )

                 {

                 GPIO_PinOutSet(port_LED, 2);



                 }



          b = 256*tsl_txBuffer5 +  tsl_txBuffer4;	// calculate output of ADC1 and compare

                if(b>2048)

                {

               	GPIO_PinOutClear(port_LED, 2);



                }







                ClearTSLInterrupt_TSL();

          INT_Enable();



    }

















/**************************************************************************//**

 Routine to clear ACK

 *****************************************************************************/



void ClearACK()

{

int fl = (I2C0->IF & I2C_IF_ACK);

I2C0->IFC = fl;

}









/**************************************************************************//**

 Write address/data to TXDATA

 *****************************************************************************/







void writetoTXDATA ( int n )

{

I2C0->TXDATA = n;

while(! (I2C0->IF & I2C_IF_ACK));

int fl = (I2C0->IF & I2C_IF_ACK);

I2C0->IFC = fl;

}







/***************************************************************************/ /**

  * I2C Setup routine

 ******************************************************************************* */



void I2CSetup_TSL(void)

{

I2C_Init_TypeDef i2cInit1 = I2C_INIT_DEFAULT;



I2C_Init(I2C0, &i2cInit1);



CMU_ClockEnable(cmuClock_HFPER, true);

CMU_ClockEnable(cmuClock_I2C0, true);

    CMU_ClockEnable(cmuClock_GPIO, true);





  GPIO_PinModeSet(port_TSL_SCL, 7, gpioModeWiredAndPullUpFilter, 1);

  GPIO_PinModeSet(port_TSL_SDA, 6, gpioModeWiredAndPullUpFilter, 1);

      GPIO_PinModeSet(port_i2c_interrupt, 1, gpioModeInput, 1);











for (int i = 0; i < 9; i++)

  {

    /* TBD: Seems to be clocking at appr 80kHz-120kHz depending on compiler

    * optimization when running at 14MHz. A bit high for standard mode devices,

    * but DVK only has fast mode devices. Need however to add some time

    * measurement in order to not be dependable on frequency and code executed.

    */

    GPIO_PinModeSet(port_TSL_SCL, 7, gpioModeWiredAnd, 0);

    GPIO_PinModeSet(port_TSL_SCL, 7, gpioModeWiredAnd, 1);

  }





    I2C0->ROUTE = I2C_ROUTE_SDAPEN |	// route pins used for I2c buses SDA and SCL

                  I2C_ROUTE_SCLPEN |

    	      (I2C_ROUTE_LOCATION_LOC1);



    if(I2C0->STATE & I2C_STATE_BUSY){	// reset registers and device

    I2C0->CMD = I2C_CMD_ABORT ;

    }



    int fl = I2C0->IF;

        I2C0->IFC = fl;

}









/***************************************************************************/ /**

  * I2C Transfer

 ******************************************************************************* */



void performI2CTransferAndMonitor_TSL(void)

{

blockSleepMode(EM1);











   I2C0->TXDATA = TSLAddr_write;

   I2C0->CMD = I2C_START;





   while(! (I2C0->IF & I2C_IF_ACK));



   ClearACK();







   writetoTXDATA( CMD_WORD );     /* Implementing word transfer, directly pass the subsequent commands.

 	 	 	 	 	 	 	 	 	 	No need to specify address every time as it is incremented by itself.*/



   writetoTXDATA( 0x00 );



   writetoTXDATA( TimingReg );



   writetoTXDATA( THLOLO );



   writetoTXDATA( THLOHI );



   writetoTXDATA( THHILO );



   writetoTXDATA( THHIHI );



   writetoTXDATA( TSL_IntReg );





   I2C0->CMD = I2C_STOP;











//   ClearACK();



   I2C0->TXDATA = TSLAddr_write;

      I2C0->CMD = I2C_START;





      while(! (I2C0->IF & I2C_IF_ACK));



      ClearACK();







      writetoTXDATA( CMD_WORD );     /* Implementing word transfer, directly pass the subsequent commands.

    	 	 	 	 	 	 	 	 	No need to specify address every time as it is incremented by itself.*/



      writetoTXDATA( PowerUp );

      I2C0->CMD = I2C_STOP;











   GPIO_PinModeSet(port_i2c_interrupt, 1, gpioModeInput, 1);

   GPIO_IntConfig( port_i2c_interrupt, 1, false, true, true );

   //	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);

   	NVIC_EnableIRQ(GPIO_ODD_IRQn);







   unblockSleepMode(EM1);













}





void I2CSetup_MMA(void)

{





I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;



I2C_Init(I2C1, &i2cInit);



if(I2C1->STATE & I2C_STATE_BUSY){

I2C1->CMD = I2C_CMD_ABORT ;

}



CMU_ClockEnable(cmuClock_HFPER, true);

CMU_ClockEnable(cmuClock_I2C1, true);

    CMU_ClockEnable(cmuClock_GPIO, true);





  GPIO_PinModeSet(port_MMA_SCL, 5, gpioModeWiredAndPullUpFilter, 1);

  GPIO_PinModeSet(port_MMA_SDA, 4, gpioModeWiredAndPullUpFilter, 1);





for (int i = 0; i < 9; i++)

  {

    /*

    * TBD: Seems to be clocking at appr 80kHz-120kHz depending on compiler

    * optimization when running at 14MHz. A bit high for standard mode devices,

    * but DVK only has fast mode devices. Need however to add some time

    * measurement in order to not be dependable on frequency and code executed.

    */

    GPIO_PinModeSet(port_MMA_SCL, 5, gpioModeWiredAnd, 0);

    GPIO_PinModeSet(port_MMA_SCL, 5, gpioModeWiredAnd, 1);

  }



    I2C1->ROUTE = I2C_ROUTE_SDAPEN |	// route pins used for I2c buses SDA and SCL

                  I2C_ROUTE_SCLPEN |

    	      (I2C_ROUTE_LOCATION_LOC0);



    if(I2C1->STATE & I2C_STATE_BUSY){	// reset registers and device

    I2C1->CMD = I2C_CMD_ABORT ;

    }



    int fl = I2C1->IF;

        I2C1->IFC = fl;







}





void performi2ctransfer_MMA(void)

{



    I2C1->IEN = 0x1AFF;



     // data out rate



   writetoI2C(0x2A, 0x30);



    //  register scale set



   writetoI2C(0x0E, 0x01);



   // active mode





   writetoI2C(0x2A, 0x31);



}















void writetoI2C(int reg_address, int data)

{

I2C1->CMD = I2C_START;

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



     I2C1->CMD = I2C_STOP;

}





void readfromI2C_MMA(uint32_t read_regaddress)

{



I2C1->TXDATA = 0x3A;

I2C1->CMD = I2C_START;

while(! (I2C1->IF & I2C_IF_ACK));

fl = (I2C1->IF & I2C_IF_ACK);

I2C1->IFC = fl;





I2C1->TXDATA = read_regaddress;

I2C1->CMD = I2C_START;

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

  I2C1->CMD = I2C_STOP;



}















void initLeuart(void)

{



/* Start LFXO, and use LFXO for low-energy modules */

// CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);

  CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFRCO);



  /* Enabling clocks, all other remain disabled */

  CMU_ClockEnable(cmuClock_CORELE, true);     /* Enable CORELE clock */



  CMU_ClockEnable(cmuClock_GPIO, true);       /* Enable GPIO clock */

  CMU_ClockEnable(cmuClock_LEUART0, true);    /* Enable LEUART1 clock */





/* Defining the LEUART0 initialization data */

LEUART_Init_TypeDef leuart0Init =

{

  .enable   = leuartEnableTx,       /* Activate data reception on LEUn_TX pin. */

  .refFreq  = 0,                    /* Inherit the clock frequenzy from the LEUART clock source */

  .baudrate = 9600,                 /* Baudrate = 9600 bps */

  .databits = leuartDatabits8,      /* Each LEUART frame containes 8 databits */

  .parity   = leuartNoParity,       /* No parity bits in use */

  .stopbits = leuartStopbits2,      /* Setting the number of stop bits in a frame to 2 bitperiods */

};



  LEUART_Reset(LEUART0);

  LEUART_Init(LEUART0, &leuart0Init);

  while((LEUART0->SYNCBUSY) != 0);



  /* Route LEUART0 TX pin to DMA location 0 */

  LEUART0->ROUTE = LEUART_ROUTE_TXPEN |

                   LEUART_ROUTE_LOCATION_LOC0;



  /* Enable GPIO for LEUART0. TX is on D4 */

  GPIO_PinModeSet(gpioPortD,                /* GPIO port */

          LEUART_tx_port,                        /* GPIO port number */

                  gpioModePushPull,         /* Pin mode is set to push pull */

                  1);                       /* High idle state */



  /* Enable GPIO for LEUART0. RX is on D5 */

    GPIO_PinModeSet(gpioPortD,                /* GPIO port */

            LEUART_rx_port,                        /* GPIO port number */

                    gpioModePushPull,         /* Pin mode is set to push pull */

                    1);



    LEUART0->CTRL |= LEUART_LOOPBK;

    LEUART0->CMD |= LEUART_RXEN;



    LEUART0->IEN = LEUART_TXBL;

    blockSleepMode(EM2);



  //  NVIC_EnableIRQ(LEUART0_IRQn);



}



/**************************************************************************/ /**

  * DMA callBack routine

 ******************************************************************************* */

void ADCdmaTransferComplete(unsigned int channel, bool primary, void *user)

{

  (void) channel;

  (void) primary;

  (void) user;



  ADC0->CMD |= adc_stop;                                                         // Turn off ADC by writing 1 to stop bit in CMD register

  transferActive = false;

  unblockSleepMode(EM1);                                                            // Unblock previously blocked energy mode



  int sum = 0;



  for(int i=0;i<ADCSAMPLES;i++)                                                     //Calculate sum of data stored in location pointed by DstPtr

  {

sum = sum + DstPtr[i];

  }



  sum = sum/ADCSAMPLES;                                                              // Calculate average

  float average = convertToCelsius(sum);                                             // Calculate average temperature in celsius

  float rounded = ((int)(average*100 + .5)/100.0) ;

  //rounded = -1.33;



  if(rounded < 0)

  {

  sign = 0x6E;

  rounded = -1*rounded;

  }

  else sign = 0x70;



 // seperating the int and float parts of the rounded value



  sumI = (int)rounded; //sumI is the part of int



  sumF = (rounded - sumI)*100;





 initLeuart();



 unblockSleepMode(EM2);













 // turn on LED if temperature is outside pre-set limits



 /* if( average<LowerTemp || average>UpperTemp)



  {

    GPIO_PinOutSet(port_LED , LED_temperature_pin );

  }



  else GPIO_PinOutClear(port_LED , LED_temperature_pin);*/



}



/* This code is originally Silicon Labs and copyrighted by Silicon Labs in 2015 and Silicon Labs grants

 * permission to anyone to use the software for any purpose, including commercial applications, and to alter it,

 * and to redistribute it freely subject that the origin is not miss represented, altered source version must be

 * plainly marked, and this notice cannot be altered or removed from any source distribution.

 *

 * Routine include :

 *

 * float convertToCelsius

*/







float convertToCelsius(int32_t adcSample)

{

  float temp;

  /* Factory calibration temperature from device information page. */

  float cal_temp_0 = (float)((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK)

                             >> _DEVINFO_CAL_TEMP_SHIFT);



  float cal_value_0 = (float)((DEVINFO->ADC0CAL2

                               & _DEVINFO_ADC0CAL2_TEMP1V25_MASK)

                              >> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);



  /* Temperature gradient (from datasheet) */

  float t_grad = -6.27;



  temp = (cal_temp_0 - ((cal_value_0 - adcSample)  / t_grad));



  return temp;

}





/***************************************************************************/ /**

  * DMA setup

 ******************************************************************************* */

void DMA_setup(void)

{

    // DMA init



DMA_Init_TypeDef dma_init ;





dma_init.hprot        =  0;

    dma_init.controlBlock = dmaControlBlock;



    CMU_ClockEnable(cmuClock_HFPER, true);

    CMU_ClockEnable(cmuClock_DMA, true);



    DMA_Init(&dma_init);





    // DMA channel configuration



    DMA_CfgChannel_TypeDef  chnl_Cfg;



    chnl_Cfg.highPri   = false;

    chnl_Cfg.enableInt = true;

    chnl_Cfg.select    = DMAREQ_ADC0_SINGLE;

    chnl_Cfg.cb        = &cb;

    DMA_CfgChannel (DMA_CHANNEL_ADC, &chnl_Cfg);





    // DMA Descriptor configuration

    DMA_CfgDescr_TypeDef  descrCfg;



      descrCfg.dstInc  = dmaDataInc2;

      descrCfg.srcInc  = dmaDataIncNone;

      descrCfg.size    = dmaDataSize2;

      descrCfg.arbRate = dmaArbitrate1;

      descrCfg.hprot   = 0;

      DMA_CfgDescr(DMA_CHANNEL_ADC, true, &descrCfg);





      // call back function setup

      cb.cbFunc  = ADCdmaTransferComplete;

      cb.userPtr = NULL;

      cb.primary = true;



      DMA->IFC |= 0x01;                               // write 1 to clear DMA channel 0 previous interrupts.

      DMA->IEN |= 0x01;                               // write 1 to enable DMA channel 0 interrupt.







  }



// Activate function for DMA



void DMA_ACTIVATE()

{



DMA_ActivateBasic(DMA_CHANNEL_ADC,

                    true,

                    false,

                    (void *)DstPtr,

                    (void *)&(ADC0->SINGLEDATA),

                    ADCSAMPLES - 1);

}









/***************************************************************************/ /**

  * ADC initialization

 ******************************************************************************* */



void setupAdc(void)

{

  ADC_Init_TypeDef        adcInit       = ADC_INIT_DEFAULT;

  ADC_InitSingle_TypeDef  adcInitSingle = ADC_INITSINGLE_DEFAULT;





  adcInit.timebase = ADC_TimebaseCalc(0);

  adcInit.prescale = ADC_PrescaleCalc(2000000, 0);                     //Calculate prescalar for desired value Tconv = (N+Ta)*P/14M

  ADC_Init(ADC0, &adcInit);



  adcInitSingle.input     =  adcSingleInpTemp;                         /* Select input to ADC */

  adcInitSingle.reference    = adcRef1V25;                             /* Reference voltage */

  adcInitSingle.rep      = true;

  adcInitSingle.acqTime	  = adcAcqTime8;

  ADC_InitSingle(ADC0, &adcInitSingle);



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







void LETIMER0_Calibration(void)

  {



  TIMER_Init_TypeDef timer0 =

    {

      .enable     = false,                                 // start counting after TIMER_Init is completed

      .debugRun   = false,                                 // Counter shall keep counting during debug halt

      .prescale   = timerPrescale1,

      .clkSel     = timerClkSelHFPerClk,

      .fallAction = timerInputActionNone,

      .riseAction = timerInputActionNone,

      .mode       = timerModeUp,

      .dmaClrAct  = false,

      .quadModeX4 = false,

      .oneShot    = false,

      .sync       = false,

    };



  TIMER_Init_TypeDef timer1 =

    {

      .enable     = false,                                 // start counting after TIMER_Init is completed

      .debugRun   = false,                                 // Counter shall keep counting during debug halt

      .prescale   = timerPrescale1,

      .clkSel     = timerClkSelCascade,

      .fallAction = timerInputActionNone,

      .riseAction = timerInputActionNone,

      .mode       = timerModeUp,

      .dmaClrAct  = false,

      .quadModeX4 = false,

      .oneShot    = false,

      .sync       = true,

    };



  // Setup both the timers



  TIMER_Init(TIMER0, &timer0);

    TIMER_Init(TIMER1, &timer1);



    // Initialize both count values to 0

    TIMER0->CNT = 0x0000;

    TIMER1->CNT = 0x0000;



    TIMER0->CMD = timer_stop;





  LETIMER_Init_TypeDef LETIMER0_Calinit;

      LETIMER0_Calinit.bufTop = false;                       // do not load COMP! into COMP0 when REP0 = 0

  LETIMER0_Calinit.comp0Top = true;                      // load COMP0 into CNT on underflow

  LETIMER0_Calinit.debugRun = false;                     // stop counting when debug stops

  LETIMER0_Calinit.enable = false;

  LETIMER0_Calinit.out0Pol = 0;                          // idle output 0

  LETIMER0_Calinit.out1Pol = 1;

  LETIMER0_Calinit.repMode = letimerRepeatOneshot;        // count until stopped by software

  LETIMER0_Calinit.rtcComp0Enable = false;                // do not start counting on COMP0 RTC match

  LETIMER0_Calinit.rtcComp1Enable = false;                // do not start counting on COMP1 RTC match

        LETIMER0_Calinit.ufoa0 = letimerUFOANone;	        // underflow output actions

  LETIMER0_Calinit.ufoa1 = letimerUFOANone;



  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);

  CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

  CMU_ClockEnable(cmuClock_LETIMER0, true);



  LETIMER_Init(LETIMER0, &LETIMER0_Calinit);

  LETIMER0->CNT = LETIMER0_LFXO_count;



  LETIMER_Enable(LETIMER0, true);



  TIMER0->CMD = timer_start;

  TIMER1->CMD = timer_start;

  while ( LETIMER0->CNT != 0 );



  TIMER0->CMD = timer_stop;

  TIMER1->CMD = timer_stop;

  int long a,LFXO_Count ;

  int  b;

  a = TIMER1->CNT;

  b = TIMER0->CNT;



  LFXO_Count = a <<16| b;







  // ULFRCO



  TIMER0->CNT = 0x0000;

  TIMER1->CNT = 0x0000;



    TIMER0->CMD = timer_stop;









    LETIMER_Init(LETIMER0, &LETIMER0_Calinit);



    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);

    CMU_OscillatorEnable(cmuOsc_LFXO, false, false);

    CMU_ClockEnable(cmuClock_LETIMER0, true);

    LETIMER0->CNT=LETIMER0_ULFRCO_count;

    LETIMER_Enable(LETIMER0, true);



      TIMER0->CMD = timer_start;

      TIMER1->CMD = timer_start;

      while ( LETIMER0->CNT != 0 ) ;



      TIMER0->CMD = timer_stop;

          TIMER1->CMD = timer_stop;



      int long ULFRCO_Count ;



      a = TIMER1->CNT;

      b = TIMER0->CNT;



      ULFRCO_Count = a<<16 | b;





      float osc_ratio;

      osc_ratio = LFXO_Count/(float)ULFRCO_Count;







      ULFRCO_Calibrated = osc_ratio*LETIMER0_ULFRCO_count;





  }









void LETIMER0_Setup(void){



   LETIMER_Init_TypeDef LETIMER0_init;







   int Comp0_init;

   int Comp1_int_init;

   float Comp1_float_init;

   int LETIMER0_prescalar;

   //int ULFRCO_count_calibrated;



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



   if (LETIMER0_EM_Mode == EM3)

   {

   if(CALIBRATION == 1){







   Comp0_init = LETIMER0_period * ULFRCO_Calibrated ;

   }



   else Comp0_init = LETIMER0_period * LETIMER0_ULFRCO_count ;



   }

   else {

   LETIMER0_prescalar = LETIMER0_period/2;

   Comp0_init = LETIMER0_period * LETIMER0_LFXO_count;

   CMU->LFAPRESC0 &= 0xfffff0ff;

   CMU->LFAPRESC0 |= LETIMER0_prescalar <<8 ;

   LETIMER0_prescalar = 1 << LETIMER0_prescalar;



   Comp0_init = LETIMER0_period * (LETIMER0_LFXO_count /



   LETIMER0_prescalar);



   }



   LETIMER0->CNT = Comp0_init;



   LETIMER_CompareSet(LETIMER0,0,Comp0_init);







   //load comp1 register with a count to generate time in ms

   //load comp0 register with a count to initialize cnt





   // initialize on time in ULFRCO for EM3 and LFXO for all other energy modes

   if (LETIMER0_EM_Mode == EM3)

   {



   if(CALIBRATION == 1){



   	    Comp1_int_init = LETIMER0_LEDontime * ULFRCO_Calibrated;

   	    Comp1_float_init = LETIMER0_LEDontime * ULFRCO_Calibrated;



   	    if(Comp1_float_init > Comp1_int_init)                            /* Compare int nd float values of the multiplication for minimum */

   	    {                                                                 /*  accurate excite time of 4ms*/

     Comp1_int_init++;

   	    }



     else{

     Comp1_int_init +=0;

         }



  }



   else

   {

   Comp1_int_init = LETIMER0_LEDontime * LETIMER0_ULFRCO_count ;



   }

   }





   else

   Comp1_int_init = LETIMER0_LEDontime * (LETIMER0_LFXO_count/LETIMER0_prescalar);



   LETIMER_CompareSet(LETIMER0,1,Comp1_int_init);                // set compare register value ( pointer , compare register 0 to set, initialization value)





   while((LETIMER0->SYNCBUSY) != 0);                             //wait till synchronization bit goes low



   // setting corresponding flag bits in LETIMER_IEN

   LETIMER0->IEN = LETIMER_IEN_UF

   | LETIMER_IEN_COMP1;



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



  /**************************************************************************//**

       * @brief LEUART Interrupt handler

       *****************************************************************************/

  void LEUART0_IRQHandler(void)

  {



// blockSleepMode(EM2);



      int flags = LEUART0->IF;

      LEUART0->IFC = flags;



    if (irfcount == 0)

        {

        irfcount ++;

        int pin = GPIO_PinInGet(gpioPortE, LED_lightness_pin);

          if(pin == 1)

          {

          LEUART0->TXDATA = LED_on_condition;	// passing ascii g to indicate that LED is on

          while (!(LEUART0->IF & LEUART_IF_TXC));

          }

          else if (pin == 0)

          {

          LEUART0->TXDATA = LED_off_condition;	// passing ascii h to indicate that LED is off

          while (!(LEUART0->IF & LEUART_IF_TXC));



          }





        }





  else if(txcount == 0)                                                               // use of else if to make sure that only one data is transmitted at 1 interrupt

            {



            LEUART0->TXDATA = sign;

            while ((LEUART0->IF & LEUART_IF_TXC)==0);

            txcount ++;

            }









    else if(txcount == 1)                                                               // use of else if to make sure that only one data is transmitted at 1 interrupt

          {



          LEUART0->TXDATA = sumI;

          while ((LEUART0->IF & LEUART_IF_TXC)==0);

          txcount ++;

          }





    else if(txcount == 2)

    {

  NVIC_DisableIRQ(LEUART0_IRQn);

          LEUART0->TXDATA = sumF;

          while ((LEUART0->IF & LEUART_IF_TXC)==0);

          txcount = 0;



          irfcount = 0;

          }











  // unblockSleepMode(EM2);









  }



  /**************************************************************************//**

     * @brief LETIMER0 Interrupt handler

     *****************************************************************************/

void LETIMER0_IRQHandler(void){

int currentFlags;

currentFlags = LETIMER0->IF;                                                          // save IF register contents/state in a variable

LETIMER0->IFC = currentFlags;                                                         // Interrupt flag clear register







/* ACMP output check */

// depending upon the flag set, change LED functions for each of the interrupts

if ((currentFlags & LETIMER_IF_COMP1) != 0)

{

if( Interval == 0){

powerup();

}



}

else if ((currentFlags & LETIMER_IF_UF) != 0)

{







timer_count++;



if(timer_count == 2)

{

  heartbeat_count = 5*count;

  timer_count = 0;// Turn off ACMP

}







/* DMA routine start here*/



if( withDMA == 1)                                                    // Activate DMA here if used

{

DMA_ACTIVATE();

}





/*Non DMA routine starts here*/



blockSleepMode(EM1);                                                 // Block energy mode for ADC and DAC



ADC0->CMD |= adc_start;                                                  // Enable ADC



if( withDMA != 1)                                                   // Follow the routine for no DMA

{

for(int j=0;j<ADCSAMPLES;j++)                                       // Take each ADC conversion data and store in memory

{

while (!(ADC0->STATUS & ADC_STATUS_SINGLEDV));

Dstbuffer[j] = ADC0->SINGLEDATA;



}



ADC0->CMD |= adc_stop;                                           // Disable ADC

  transferActive = false;                                           // Update user defined flag

  unblockSleepMode(EM1);                                            // Unblock sleep mode





/* Routine to find the average temperation in celsius and change LED state accordingly */



int sumtemp = 0;



  for(int i=0;i<ADCSAMPLES;i++)

  {

sumtemp = sumtemp + Dstbuffer[i];

  }



  sumtemp = sumtemp/ADCSAMPLES;

  float average_nodma = convertToCelsius(sumtemp);



/*	  if( average_nodma<LowerTemp || average_nodma>UpperTemp)



  {

    GPIO_PinOutSet(port_LED , LED_temperature_pin);

  }



  else GPIO_PinOutClear(port_LED , LED_temperature_pin); */



}







if( Interval == 0)

{

// powerup();

I2CSetup_MMA();	// do i2c setup, configure TSL2561 registers and monitor if interrupt occurs

performi2ctransfer_MMA();

I2CSetup_TSL();	// do i2c setup, configure TSL2561 registers and monitor if interrupt occurs

performI2CTransferAndMonitor_TSL();

Interval++;

}





else if( Interval == 1 )

{

    readfromI2C_MMA(0x01);



    if(i2c_txBuffer5 >= 220)

    {

            GPIO_PinOutSet(port_LED, 3);

    }



    else  GPIO_PinOutClear(port_LED, 3);



    Interval++;



}







else if( Interval == 2)

{

powerdown();

Interval = 0;	// enable power down routine

//unblockSleepMode(EM1);



}



}





}













 int main(void){

//int ULFRCO_Calibrated;



CHIP_Init(); // initialize all errata





blockSleepMode(EM3);





CMU_Setup();

ACMPInit();

GPIO_Setup();



if( withDMA == 1){

    DMA_setup();

}

    setupAdc();



LETIMER0_Setup();



LETIMER_Enable(LETIMER0, true);



ACMP0->CTRL |= ACMP_CTRL_EN;



  /* Wait for warmup */

     while (!(ACMP0->STATUS & ACMP_STATUS_ACMPACT)) ;











while(1){

Sleep(); // call sleep routine



while(1)

  {

    //INT_Disable();

    if ( transferActive )

      {

      Sleep();

      }

  //  INT_Enable();



    /* Exit the loop if transfer has completed */

    if ( !transferActive )

    {

      break;

    }

  }



}

}


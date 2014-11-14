#include "MPC5604B_M07N.h"
#define SERVO_CENTRE 1600
#define SERVO_LIMIT 300
#define START  3
#define END  128
#define L_BOUND 40
#define R_BOUND 90
#define MIN_FINGER 20
#define LB_WIDTH 10
#define UB_WIDTH 30
#define FILTER 50


//Prototypes
void TransmitCharacter(uint8_t ch);
void FLUSH_data_car(void);
void CAMERA_car(void);

// Global variables
volatile uint32_t dly,lly;	// used in delay routines
uint8_t rx_data[4],pt;

//volatile uint8_t Result[128];             	/* Read converstion result from ADC input ANS0 */
volatile uint16_t Result[128];                	/* Read converstion result from ADC input ANS0 */
volatile int32_t diff_result[127];             	/* Read converstion result from ADC input ANS0 */
volatile uint8_t i=0,j=0;	// 8 bit counters
volatile uint32_t adcdata;

// MY VARIABLES
//uint8_t kp=0.2,kd=0,ki=0;
uint8_t kp = 3.8;
int16_t diff=0;
uint32_t int_time=75000; 
uint16_t dc_speed_a = 350;

// Main loop variables
int16_t max_val, min_val;
uint8_t max_I, min_I, centre=64, prev_centre=64;
int8_t error;
int16_t pid_term;
uint16_t correction;
uint8_t option;						// Can be removed
int8_t width=0;

int a = 500;
	


// ROUTINES.........
void printserialhex(uint16_t innum) {
  uint16_t j1,in;
  uint8_t p1,p2;
  in = innum;
   
  j1 = (in & 0x0f);
  if (j1 > 9) p1 = (uint8_t)(j1 + 0x41 - 10);
  else p1 = (uint8_t)(j1 +0x30);
  j1 = (in & 0xf0) >> 4;
  if (j1 > 9) p2 = (uint8_t)(j1 +0x41 - 10);
  else p2 = (uint8_t)(j1 +0x30);
  TransmitCharacter(p2);
  TransmitCharacter(p1);  
}

void printserialsingned(uint16_t innum) {
  uint16_t j1,k1,l1,m1,in;
  uint8_t p1,p2,p3,p4,p5;
 
  if(innum < 0x8000) {
    in = innum;
  	TransmitCharacter('+');    
  } 
  else {
    in = (uint16_t)(~innum);
    //in = 0x7fff - in;
    TransmitCharacter('-');     
  }
  
  j1 = (in / 10);
  p1 = (uint8_t)(in - j1*10 +0x30);
  k1 = (j1 / 10);
  p2 = (uint8_t)(j1 - k1*10 +0x30);
  l1 = (k1 / 10);
  p3 = (uint8_t)(k1 - l1*10 +0x30);
  m1 = (l1 / 10);
  p4 = (uint8_t)(l1 - m1*10 +0x30);
  p5 = (uint8_t)m1 +0x30;
  TransmitCharacter(p5);
  TransmitCharacter(p4);
  TransmitCharacter(p3);
  TransmitCharacter(p2);
  TransmitCharacter(p1);  
  //TransmitCharacter(0x09);
}


void printlistall(void) {
   //TransmitCharacter(0x0a);   
   //TransmitCharacter(0x0d);  
   for(pt=0;pt<128;pt++){
      //pt++;
      //pt++;
      printserialsingned(Result[pt]);
      //printserial(list[pt]);
   }
   printserialsingned(centre);
   
   TransmitCharacter(0x0a);   	// LIne feed
   //TransmitCharacter(0x0d);   // carrige return
}

void init_LinFLEX_0_UART (void) 
{	

	/* enter INIT mode */
	LINFLEX_0.LINCR1.R = 0x0081; 		/* SLEEP=0, INIT=1 */
	
	/* wait for the INIT mode */
	while (0x1000 != (LINFLEX_0.LINSR.R & 0xF000)) {}
		
	/* configure pads */
	SIU.PCR[18].R = 0x0604;     		/* Configure pad PB2 for AF1 func: LIN0TX */
	SIU.PCR[19].R = 0x0100;     		/* Configure pad PB3 for LIN0RX */	
	
	/* configure for UART mode */
	LINFLEX_0.UARTCR.R = 0x0001; 		/* set the UART bit first to be able to write the other bits */
	LINFLEX_0.UARTCR.R = 0x0033; 		/* 8bit data, no parity, Tx and Rx enabled, UART mode */
								 		/* Transmit buffer size = 1 (TDFL = 0 */
								 		/* Receive buffer size = 1 (RDFL = 0) */
	
	/* configure baudrate 115200 */
	/* assuming 64 MHz peripheral set 1 clock */		
	LINFLEX_0.LINFBRR.R = 12;
	LINFLEX_0.LINIBRR.R = 34;
		
	/* enter NORMAL mode */
	LINFLEX_0.LINCR1.R = 0x0080; /* INIT=0 */	
}

void TransmitCharacter(uint8_t ch)
{
	LINFLEX_0.BDRL.B.DATA0 = ch;  			/* write character to transmit buffer */
	while (1 != LINFLEX_0.UARTSR.B.DTF) {}  /* Wait for data transmission completed flag */
	LINFLEX_0.UARTSR.R = 0x0002; 			/* clear the DTF flag and not the other flags */	
}

void TransmitData (char TransData[]) 
{
	uint8_t	j,k;                                 /* Dummy variable */
	k = strlen (TransData);
	for (j=0; j< k; j++) 
	{  /* Loop for character string */
		TransmitCharacter(TransData[j]);  		/* Transmit a byte */		
	}
}

/* This functions polls UART receive buffer. when it is full, it moves received data from the buffer to the memory */
uint8_t ReadData (void)
{
	uint8_t ch;
	/* wait for DRF */
	while (1 != LINFLEX_0.UARTSR.B.DRF) {}  /* Wait for data reception completed flag */
		
	/* wait for RMB */
	while (1 != LINFLEX_0.UARTSR.B.RMB) {}  /* Wait for Release Message Buffer */
	
	/* get the data */
	ch = (uint8_t)LINFLEX_0.BDRM.B.DATA4;
		
	/* clear the DRF and RMB flags by writing 1 to them */
	LINFLEX_0.UARTSR.R = 0x0204;
	
	return ch;	
}

void initModesAndClock(void) {
	ME.MER.R = 0x0000001D;          	/* Enable DRUN, RUN0, SAFE, RESET modes */
	                              		/* Initialize PLL before turning it on: */
										/* Use 1 of the next 2 lines depending on crystal frequency: */
	CGM.FMPLL_CR.R = 0x02400100;    	/* 8 MHz xtal: Set PLL0 to 64 MHz */   
	/*CGM.FMPLL_R = 0x12400100;*/     	/* 40 MHz xtal: Set PLL0 to 64 MHz */   
	ME.RUN[0].R = 0x001F0074;       	/* RUN0 cfg: 16MHzIRCON,OSC0ON,PLL0ON,syclk=PLL */
	
	//ME.RUNPC[0].R = 0x00000010; 	  	/* Peri. Cfg. 0 settings: only run in RUN0 mode */
   										/* Use the next lines as needed for MPC56xxB/S: */  	    	
	//ME.PCTL[48].R = 0x0000;         	/* MPC56xxB LINFlex0: select ME.RUNPC[0] */	
	//ME.PCTL[68].R = 0x0000;         	/* MPC56xxB/S SIUL:  select ME.RUNPC[0] */	
	
	ME.RUNPC[1].R = 0x00000010;     	/* Peri. Cfg. 1 settings: only run in RUN0 mode */
	ME.PCTL[32].R = 0x01;       		/* MPC56xxB ADC 0: select ME.RUNPC[1] */
  	ME.PCTL[57].R = 0x01;       		/* MPC56xxB CTUL: select ME.RUNPC[1] */
  	ME.PCTL[48].R = 0x01;           	/* MPC56xxB/P/S LINFlex 0: select ME.RUNPC[1] */
	ME.PCTL[68].R = 0x01;           	/* MPC56xxB/S SIUL:  select ME.RUNPC[1] */
	ME.PCTL[72].R = 0x01;           	/* MPC56xxB/S EMIOS 0:  select ME.RUNPC[1] */
	                              		/* Mode Transition to enter RUN0 mode: */
	ME.MCTL.R = 0x40005AF0;         	/* Enter RUN0 Mode & Key */
	ME.MCTL.R = 0x4000A50F;         	/* Enter RUN0 Mode & Inverted Key */  
	while (ME.GS.B.S_MTRANS) {}     	/* Wait for mode transition to complete */    
	                          			/* Note: could wait here using timer and/or I_TC IRQ */
	while(ME.GS.B.S_CURRENTMODE != 4) {}/* Verify RUN0 is the current mode */
	
	//while (ME.IS.B.I_MTC != 1) {}    /* Wait for mode transition to complete */    
	//ME.IS.R = 0x00000001;           /* Clear Transition flag */ 
}


void initPeriClkGen(void) {
	CGM.SC_DC[0].R = 0x80; 				/* MPC56xxB/S: Enable peri set 1 sysclk divided by 1 */
  	CGM.SC_DC[2].R = 0x80;         		/* MPC56xxB: Enable peri set 3 sysclk divided by 1*/
}

void disableWatchdog(void) {
	SWT.SR.R = 0x0000c520;     			/* Write keys to clear soft lock bit */
  	SWT.SR.R = 0x0000d928; 
  	SWT.CR.R = 0x8000010A;     			/* Clear watchdog enable (WEN) */
}

void initPads (void) {
	SIU.PCR[2].R = 0x0503;           	/* MPC56xxB: Initialize PA[2] as eMIOS[2] input */
	SIU.PCR[3].R = 0x0600;           	/* MPC56xxB: Initialize PA[3] as eMIOS[3] output */
	SIU.PCR[20].R = 0x2000;          	/* MPC56xxB: Initialize PB[4] as ANP0 */
	SIU.PCR[21].R = 0x2000;          	/* MPC56xxB: Initialize PB[5] as ANP1 */
	SIU.PCR[22].R = 0x2000;          	/* MPC56xxB: Initialize PB[6] as ANP2 */
}

void initADC(void) {
	//ADC.MCR.R = 0x20020000;         	/* Initialize ADC scan mode*/
	ADC.MCR.R = 0x00000000;         	/* Initialize ADC one shot mode*/
	ADC.NCMR[0].R = 0x00000007;      	/* Select ANP1:2 inputs for normal conversion */
	ADC.CTR[0].R = 0x00008606;       	/* Conversion times for 32MHz ADClock */
}

void initCTU(void) {
  	CTU.EVTCFGR[2].R = 0x00008000;  	 /* Config event on eMIOS Ch 2 to trig ANP[0] */
}

// INITIALIZE THE EMIOS MODULE A (0)
void initEMIOS_0(void) {  
	EMIOS_0.MCR.B.GPRE= 63;   			/* Divide 64 MHz sysclk by 63+1 = 64 for 1MHz eMIOS clk*/
	EMIOS_0.MCR.B.GPREN = 1;			/* Enable eMIOS clock */
	EMIOS_0.MCR.B.GTBE = 1;  			/* Enable global time base */
	EMIOS_0.MCR.B.FRZ = 1;    			/* Enable stopping channels when in debug mode */
}

void initEMIOS_0ch3(void) {
	EMIOS_0.CH[3].CADR.R = 250;      	/* Ch 3: Match "A" is 250 */
	EMIOS_0.CH[3].CBDR.R = 500;      	/* Ch 3: Match "B" is 500 */
	EMIOS_0.CH[3].CCR.R= 0x000000E0; 	/* Ch 3: Mode is OPWMB, time base = ch 23 */
	
	EMIOS_0.CH[2].CCR.R= 0x01020082; 	/* Ch 2: Mode is SAIC (SINGLE ACTION INPUT CAPTURE), time base = ch 23 */
}

// FEEDS THE COUNTER BUS B
void initEMIOS_0ch0(void) {        		/* EMIOS 0 CH 0: Modulus Up Counter */
	EMIOS_0.CH[0].CADR.R = 19999;   	/* Period will be 19999+1 = 20000 clocks (20 msec)*/
	EMIOS_0.CH[0].CCR.B.MODE = 0x50; 	/* Modulus Counter Buffered (MCB) */
	EMIOS_0.CH[0].CCR.B.BSL = 0x3;   	/* Use internal counter */
	EMIOS_0.CH[0].CCR.B.UCPRE=0;     	/* Set channel prescaler to divide by 1 */
	EMIOS_0.CH[0].CCR.B.UCPEN = 1;   	/* Enable prescaler; uses default divide by 1*/
	EMIOS_0.CH[0].CCR.B.FREN = 1;   	/* Freeze channel counting when in debug mode*/
}

//FEEDS THE COUNTER BUS A
void initEMIOS_0ch23(void) {        	/* EMIOS 0 CH 23: Modulus Up Counter */
	EMIOS_0.CH[23].CADR.R = 999;      	/* Period will be 999+1 = 1000 clocks (1 msec)*/
	EMIOS_0.CH[23].CCR.B.MODE = 0x50; 	/* Modulus Counter Buffered (MCB) */
	EMIOS_0.CH[23].CCR.B.BSL = 0x3;   	/* Use internal counter */
	EMIOS_0.CH[23].CCR.B.UCPRE=0;     	/* Set channel prescaler to divide by 1 */
	EMIOS_0.CH[23].CCR.B.UCPEN = 1;   	/* Enable prescaler; uses default divide by 1*/
	EMIOS_0.CH[23].CCR.B.FREN = 1;   	/* Freeze channel counting when in debug mode*/
}

// WORKS ON COUNTER BUS B
// SERVO
void initEMIOS_0ch4(void) {        		/* EMIOS 0 CH 4: Output Pulse Width Modulation*/
	EMIOS_0.CH[4].CADR.R = 0;     		/* Leading edge when channel counter bus=0*/
	EMIOS_0.CH[4].CBDR.R = 1400;      	/* Trailing edge when channel counter bus=1400 Middle, 1650 Right Max, 1150 Left Max*/
	EMIOS_0.CH[4].CCR.B.BSL = 0x01;  	/* Use counter bus B */
	EMIOS_0.CH[4].CCR.B.EDPOL = 1;  	/* Polarity-leading edge sets output */
	EMIOS_0.CH[4].CCR.B.MODE = 0x60; 	/* Mode is OPWM Buffered */
	SIU.PCR[28].R = 0x0600;           	/* MPC56xxS: Assign EMIOS_0 ch 6 to pad */
}

// BOTH THESE CHANNELS WORK ON COUNTER BUS A
// DC MOTOR __ L
void initEMIOS_0ch6(void) {        		/* EMIOS 0 CH 6: Output Pulse Width Modulation*/
	EMIOS_0.CH[6].CADR.R = 0;     	/* Leading edge when channel counter bus=0*/
	EMIOS_0.CH[6].CBDR.R = dc_speed_a;     	/* Trailing edge when channel counter bus=500*/
	
	EMIOS_0.CH[6].CCR.B.BSL = 0x0;  	/* Use counter bus A (default) */
	EMIOS_0.CH[6].CCR.B.EDPOL = 1;  	/* Polarity-leading edge sets output */
	EMIOS_0.CH[6].CCR.B.MODE = 0x60; 	/* Mode is OPWM Buffered */
	SIU.PCR[30].R = 0x0600;           	/* MPC56xxS: Assign EMIOS_0 ch 6 to pad */
}

// DC MOTOR __ R
void initEMIOS_0ch7(void) {        		/* EMIOS 0 CH 7: Output Pulse Width Modulation*/
	EMIOS_0.CH[7].CADR.R = 0;    		/* Leading edge when channel counter bus=0*/
	EMIOS_0.CH[7].CBDR.R = dc_speed_a;     	/* Trailing edge when channel's counter bus=999*/
	
	EMIOS_0.CH[7].CCR.B.BSL = 0x0; 		/* Use counter bus A (default) */
	EMIOS_0.CH[7].CCR.B.EDPOL = 1; 		/* Polarity-leading edge sets output*/
	EMIOS_0.CH[7].CCR.B.MODE = 0x60; 	/* Mode is OPWM Buffered */
	SIU.PCR[31].R = 0x0600;           	/* MPC56xxS: Assign EMIOS_0 ch 7 to pad */
}

// delay routines

void Delay_car(void){
  for(dly=0;dly<3;dly++);
}

void Delay(void){
  for(dly=0;dly<250;dly++);
}

void Delay2(void){
  for(dly=0;dly<a;dly++);
}

void Delaylong(void){
  for(dly=0;dly<20000;dly++);
}

void Delaylonglong(void){
  for(lly=0;lly<1;lly++) Delaylong();
}

void Delaycamera(void){
  for(lly=0;lly<10;lly++) Delaylong();
}

void Delaycamera2(void){
  for(lly=0;lly<2;lly++) Delaylong();
}

void Delaywait(void){
  for(lly=0;lly<500;lly++) Delaylong();
}

void Delayled(void){
  for(lly=0;lly<500;lly++) Delaylong();
}


void Integrate_time_car()
{
	/*
		1 instruction  ====  160ns
		Min int time == 33.75us  === 211 insructions
	*/
	
	
	for(dly=0;dly<int_time;dly++);
	//Delaycamera();
	
}



void DC_Motors_on()
{
	SIU.PCR[16].R = 0x0200;				/* Program the drive enable pin of Left Motor as output*/
//	SIU.PGPDO[0].R = 0x00008000;		/* Enable Left the motors */
//	Delaywait();
	
	
	SIU.PCR[17].R = 0x0200;				/* Program the drive enable pin of Right Motor as output*/
//	SIU.PGPDO[0].R = 0x00004000;		/* Enable Right the motors */

//	Delaywait();

	SIU.PGPDO[0].R = 0x0000C000;		/* Enable Right the motors */	
}

void SERVO(uint32_t value)
{
	EMIOS_0.CH[4].CBDR.R = value;      	/* 1500 Middle */
}

// LEFT MOTOR DRIVE WHEN USING MOTOR IN SERIES....
void dc_speed_left(int a)
{
	EMIOS_0.CH[6].CBDR.R = a;     	/* Trailing edge when channel counter bus=500*/
}

void dc_speed_right(int a)
{
	EMIOS_0.CH[7].CBDR.R = a;     	/* Trailing edge when channel counter bus=500*/
}

void CAMERA(void)
{
	SIU.PCR[27].R = 0x0200;				/* Program the Sensor read start pin as output*/
	SIU.PCR[29].R = 0x0200;				/* Program the Sensor Clock pin as output*/
	for(j=0;j<2;j++)
	{
		SIU.PCR[27].R = 0x0200;				/* Program the Sensor read start pin as output*/
		SIU.PCR[29].R = 0x0200;				/* Program the Sensor Clock pin as output*/
		SIU.PGPDO[0].R &= ~0x00000014;		/* All port line low */
		SIU.PGPDO[0].R |= 0x00000010;		/* Sensor read start High */
		Delay();
		SIU.PGPDO[0].R |= 0x00000004;		/* Sensor Clock High */
		Delay();
		SIU.PGPDO[0].R &= ~0x00000010;		/* Sensor read start Low */ 
		Delay();
		SIU.PGPDO[0].R &= ~0x00000004;		/* Sensor Clock Low */
		Delay();
		for (i=0;i<128;i++)
		{
			Delay();
			SIU.PGPDO[0].R |= 0x00000004;	/* Sensor Clock High */
			ADC.MCR.B.NSTART=1;     		/* Trigger normal conversions for ADC0 */
			while (ADC.MCR.B.NSTART == 1) {};
			adcdata = ADC.CDR[0].B.CDATA;
			Delay();
			SIU.PGPDO[0].R &= ~0x00000004;	/* Sensor Clock Low */
			Result[i] = (uint8_t)(adcdata >> 2);		
		}
		//Delaycamera2();
		Delay2();
	}
	//printlistall();
}



void FLUSH_data_car(void)
{
	// First clock pulse
	SIU.PGPDO[0].R |= 0x00000010;		/* Sensor read start High */
	Delay_car();
	SIU.PGPDO[0].R |= 0x00000004;		/* Sensor Clock High */
	Delay_car();
	SIU.PGPDO[0].R &= ~0x00000010;		/* Sensor read start Low */ 
	Delay_car();
	SIU.PGPDO[0].R &= ~0x00000004;		/* Sensor Clock Low */
	
	// Clock pulses from 2 to 129
	for (i=0;i<128;i++)
	{
		Delay_car();Delay_car();
		SIU.PGPDO[0].R |= 0x00000004;	/* Sensor Clock High */
		Delay_car();Delay_car();
		SIU.PGPDO[0].R &= ~0x00000004;	/* Sensor Clock Low */
	}
	
}

void CAMERA_car(void)
{
	SIU.PCR[27].R = 0x0200;				/* Program the Sensor read start pin as output*/
	SIU.PCR[29].R = 0x0200;				/* Program the Sensor Clock pin as output*/

	SIU.PGPDO[0].R &= ~0x00000014;		/* All port line low */
	// FLUSH DATA
	FLUSH_data_car();
	Integrate_time_car();

	SIU.PGPDO[0].R &= ~0x00000014;		/* All port line low */
	
	SIU.PGPDO[0].R |= 0x00000010;		/* Sensor read start High */
	Delay_car();
	SIU.PGPDO[0].R |= 0x00000004;		/* Sensor Clock High */
	Delay_car();
	SIU.PGPDO[0].R &= ~0x00000010;		/* Sensor read start Low */ 
	// Read value no one...
	while (ADC.MCR.B.NSTART == 1) {};
	adcdata = ADC.CDR[0].B.CDATA;
	Result[0] = adcdata;		
	Delay_car();
	SIU.PGPDO[0].R &= ~0x00000004;		/* Sensor Clock Low */
	for (i=1;i<128;i++)
	{
		Delay_car();Delay_car();
		SIU.PGPDO[0].R |= 0x00000004;	/* Sensor Clock High */
		ADC.MCR.B.NSTART=1;     		/* Trigger normal conversions for ADC0 */
		while (ADC.MCR.B.NSTART == 1) {};
		adcdata = ADC.CDR[0].B.CDATA;
		Delay_car();Delay_car();
		SIU.PGPDO[0].R &= ~0x00000004;	/* Sensor Clock Low */
		//Result[i] = (uint8_t)(adcdata>>2);		
		Result[i] = adcdata;		
	}
	
	Delay_car();Delay_car();
	SIU.PGPDO[0].R |= 0x00000004;	/* Sensor Clock High */
	Delay_car();Delay_car();
	SIU.PGPDO[0].R &= ~0x00000004;	/* Sensor Clock Low */
	
	// To print the result
//	printlistall();
}


void main (void) {
	
	initModesAndClock(); 				/* Initialize mode entries and system clock */
	initPeriClkGen();  					/* Initialize peripheral clock generation for DSPIs */
	disableWatchdog(); 					/* Disable watchdog */
	
    initPads();             			/* Initialize pads used in example */
  	initADC();              			/* Init. ADC for normal conversions but don't start yet*/
  	initCTU();              			/* Configure desired CTU event(s) */
  	initEMIOS_0();          			/* Initialize eMIOS channels as counter, SAIC, OPWM */
  	initEMIOS_0ch3();					/* Initialize eMIOS 0 channel 3 as OPWM and channel 2 as SAIC*/ 
  	
  	initEMIOS_0ch0(); 					/* Initialize eMIOS 0 channel 0 as modulus counter*/
	initEMIOS_0ch23(); 					/* Initialize eMIOS 0 channel 23 as modulus counter*/
	initEMIOS_0ch4(); 					/* Initialize eMIOS 0 channel 0 as OPWM, ch 4 as time base */
	initEMIOS_0ch6(); 					/* Initialize eMIOS 0 channel 0 as OPWM, ch 6 as time base */
	initEMIOS_0ch7(); 					/* Initialize eMIOS 0 channel 1 as OPWM, ch 7 as time base */
	
	//init_LinFLEX_0_UART();

	
	
	SIU.PCR[17].R = 0x0200;				/* Program the drive enable pin of Right Motor as output*/
	SIU.PCR[16].R = 0x0200;				/* Program the drive enable pin of Left Motor as output*/
	SIU.PGPDO[0].R = 0x00000000;		/* Disable the motors */


// Routines
	DC_Motors_on();
	

//******************************** INFINITE LOOP ***********************************
for (;;) 
	{

// 0. DEBUGGING CODE
		//option = ReadData();
		//printlistall();


// 1. Clean all variables
		//sensor_value_left =0;
		//sensor_value_right =0 ;
			
// 2. Sense the line
		//option = ReadData();
		//CAMERA();
		CAMERA_car();
		

		
// 3. Calculate Differential
		for (i=0;i<127;i++)		// one less than 128
		{
			diff_result[i] = (short int) (Result[i+1] - Result [i]);	// dy.dx
		}
		
// 4. Find maximum and minimum indices
		max_val =  diff_result[START];	// can be improved to centre + max_delta
		min_val =  diff_result[START];
		max_I = START;
		min_I = START;
		
		for (i=START+1;i<END;i++)	//0 skipped
		{
			if (max_val < diff_result[i])
			{
				max_val = diff_result[i];
				max_I = i;
			}
			
			if (min_val > diff_result[i])
			{
				min_val = diff_result[i];
				min_I = i;
			}
		}
						
// 6. MAIN STATE MACHINE ALGO
		
       width = max_I - min_I;
  
       if (width> LB_WIDTH && width < UB_WIDTH)
       {
            // Everything is assumed normal.
            // IF there is a spike (very less probability)
            // it should be allowed to process, it can't misguide the car.      

            // Find centre
			if (min_I < L_BOUND)
			{
				SERVO (SERVO_CENTRE - SERVO_LIMIT );	
				continue;
			}
			
			if (max_I > R_BOUND)
			{
				SERVO (SERVO_CENTRE + SERVO_LIMIT );	
				continue;
			}
			
			centre = (max_I + min_I) /2;
			
       }
       else
       {
       		continue;
       }
       
		
		
		if ((centre - prev_centre) > FILTER)
		{
			prev_centre = centre;
			centre = centre * 0.20;
		}
		else
		{
			prev_centre = centre;	
		}
       	
	       		
		
		
// 8. Calculate error
		error = centre - 64 ;
		pid_term = (int) ( kp * error );

// 9. Calculate PID term		
		if (pid_term > SERVO_LIMIT)		
		{
			pid_term = SERVO_LIMIT;
		}
		
		else if (pid_term <-SERVO_LIMIT)
		{
			pid_term = -SERVO_LIMIT;
		}
		
// 10. Feed the new value to servo motor
		correction = SERVO_CENTRE + pid_term; 
		SERVO (correction);	
		
	}	
}

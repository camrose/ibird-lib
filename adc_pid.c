/*
 * Name: SetupADC.c
 * Desc: Configure the analog to digital converter.
 * Date: 2009-04-02
 * Author: fgb
 */

#include "adc.h"
#include "adc_pid.h"
#include "p33Fxxxx.h"
#include "ports.h"

//Functions
static void adcSetupPeripheral(void);
//DMA related functions
static void initDma0(void);
void __attribute__((__interrupt__)) _DMA0Interrupt(void);


//Variables to store values as they come out of the DMA buffer
static unsigned int adc_bemfL;
static unsigned int adc_bemfR;
static unsigned int adc_battery;
static unsigned int adc_AN3;

void adcSetup(void){
	adcSetupPeripheral();
	initDma0(); //DMA is needed to read multiple values from the ADC core
}

static void adcSetupPeripheral(void){
	unsigned int AD1CON1value, AD1CON2value, AD1CON3value, AD1CON4value, 
				AD1PCFGHvalue, AD1PCFGLvalue, AD1CSSHvalue, AD1CSSLvalue, 
				AD1CHS0value, AD1CHS123value;

	//Setup:
	//ADC1 : Ch0 - AN11 then AN1
	//		 Ch1 - AN0 then AN3
	//ADC2 : Left unconfigured for user applications
//	AD1CON1value = ADC_MODULE_ON & 			//ADC module is enabled
//				   ADC_IDLE_CONTINUE & 		// ADC will continue in idle mode
//                                   ADC_AD12B_10BIT & 		// ADC in 10 bit mode
//				   ADC_FORMAT_INTG & 		// ADC in integer format (CLARIFY)
//				   ADC_CLK_MPWM & 			// MCPWM interval ends sampling and starts conversion
//                                   ADC_MULTIPLE & 		//Simultaneously sample CH0 and CH1
//				   ADC_ADDMABM_ORDER &		//DMA buffers are written in the order of conversion
//				   ADC_AUTO_SAMPLING_ON & 	//ADC does not need to be triggered manually
//				   ADC_SAMP_ON;				//sample / hold amplifiers are sampling  (maybe incorrect)
//        AD1CON2value = ADC_VREF_AVDD_AVSS & 	//Vref+ = AVdd , Vref- = AVss
//				   ADC_SCAN_ON & 			//Do not scan through ADC channels
//				   ADC_SELECT_CHAN_0 & 	//Sample & convert
//				   ADC_ALT_BUF_OFF &		//Use one 16 word buffer
//				   ADC_ALT_INPUT_OFF & 		// Alternate between MUXA and MUXB
//				   ADC_DMA_ADD_INC_2;		//Increment DMA address after 2 samples, to account for alt. sampling
//        AD1CON3value = ADC_CONV_CLK_SYSTEM & 	//Use System clock, not internal RC osc
//				   ADC_CONV_CLK_3Tcy & 		//Tad = 3 * Tcy
//				   ADC_SAMPLE_TIME_1; 		//Sample Time = 1*Tad
//	AD1CON4value = ADC_DMA_BUF_LOC_1; 		//This may be wrong (TODO)
//
//
//	AD1CHS123value = 0;
//
// 	AD1CHS0value = ADC_CH0_NEG_SAMPLEA_VREFN & 		// Sample A, Vref- = AVss
//				   ADC_CH0_POS_SAMPLEA_AN11; 		// Sample A, CH0 = AN11
//
//
//        AD1CSSHvalue = SCAN_NONE_16_31; 				//Skip AN16-AN131 for Input Scan
//	AD1CSSLvalue = SCAN_NONE_0_15 | (1 << 0) //AN0
//                                  | (1 << 1) //AN1
//                                  | (1 << 11); //AN11
//
//	//Set pins to analog inputs; also check init_default.c
//	AD1PCFGHvalue = ENABLE_ALL_DIG_16_31; //Shouldn't matter, only AN0-15 on 706A
//        AD1PCFGLvalue = ENABLE_AN0_ANA & ENABLE_AN1_ANA & ENABLE_AN11_ANA;
//
//        SetChanADC1(AD1CHS123value, AD1CHS0value);
//        OpenADC1(AD1CON1value, AD1CON2value, AD1CON3value, AD1CON4value, AD1PCFGLvalue, AD1PCFGHvalue, AD1CSSHvalue, AD1CSSLvalue);

        AD1CON1bits.ADON = 0;       //disable
        AD1CON1bits.ADSIDL = 0;     //continue in idle mode
        AD1CON1bits.AD12B = 0;      //10 bit mode
        AD1CON1bits.FORM = 0b00;    //integer (0000 00dd dddd dddd) format output
        AD1CON1bits.SSRC = 0b011;   //Sample clock source based on PWM
        AD1CON1bits.SIMSAM = 1;     //Sample channels simultaneously
        AD1CON1bits.ASAM = 1;       //Auto sampling on
        AD1CON1bits.ADDMABM = 1;

        AD1CON2bits.VCFG = 0b000;   //Vdd is pos. ref and Vss is neg. ref.
        AD1CON2bits.CSCNA = 1;      //scan inputs
        AD1CON2bits.CHPS = 0b00;    //Convert channels 0 and 1
        AD1CON2bits.SMPI = 0b0010;  //Interrupt after 3 conversions (depends on CHPS and SIMSAM)
        AD1CON2bits.BUFM = 0;       //Always fill conversion buffer from first element
        AD1CON2bits.ALTS = 0;       //Do not alternate MUXes for analog input selection

        AD1CON3bits.ADRC = 0;       //Derive conversion clock from system clock
    //    AD1CON3bits.SAMC = 0b00001; //Auto sampling clock period is one Tad
        AD1CON3bits.ADCS = 0b00000010; // Each TAD is 3 Tcy

        AD1PCFGL = 0xF7FC;          //Enable AN0 - AN3 as analog inputs

        AD1CHS0bits.CH0SA = 0b01011;      //Select AN1 for CH0 +ve input
        AD1CHS0bits.CH0NA = 0;      //Select Vref- for CH0 -ve input

        AD1CSSL = 0x0803;

        //AD1CHS123bits.CH123SA = 0b1;  //Select AN3 for CH1 +ve input
        //AD1CHS123bits.CH123NA = 0b00;  //Select Vref- for CH1 -ve input

        AD1CON1bits.ADON = 1;       //enable
        
	IFS0bits.AD1IF = 0; // Clear the A/D interrupt flag bit
	IEC0bits.AD1IE = 0; //Disable A/D interrupt
}


//For testing purposes, should not be enabled
/*
void __attribute__((interrupt,no_auto_psv)) _ADC1Interrupt(void)
{
	//ADC sync indicator
	if(AD1CON1bits.DONE){
		LATB |= (1<<4);
	}else{
		LATB &= ~(1<<4);
	}
	IFS0bits.AD1IF = 0;
}
*/

//Getters for other modules to access values
unsigned int adcGetBEMFL(){
	return adc_bemfL;
}

unsigned int adcGetBEMFR(){
	return adc_bemfR;
}

unsigned int adcGetVBatt(){
	return adc_battery;
}


//////////////////////////////////////////////////////////////////////
///////////////      DMA Section     /////////////////////////////////
//////////////////////////////////////////////////////////////////////

#define  SAMP_BUFF_SIZE	 		1		// Size of the input buffer per analog input

//Buffers need special attribute to be in DMA memory space
static int  BufferA[3][SAMP_BUFF_SIZE] __attribute__((space(dma)));
static int  BufferB[3][SAMP_BUFF_SIZE] __attribute__((space(dma)));

static unsigned int DmaBuffer = 0;



/*****************************************************************************
* Function Name : initDma0
* Description   : Setup function for DMA0, to read ADC1 into a buffer
* Parameters    : None
* Return Value  : None
*****************************************************************************/
static void initDma0(void)
{
	DMA0CONbits.AMODE = 0;			// Configure DMA for Register Indirect w/ post-increment
	DMA0CONbits.MODE  = 2;			// Configure DMA for Continuous Ping-Pong mode
	
	DMA0PAD=(int)&ADC1BUF0;
	//DMA0CNT = (SAMP_BUFF_SIZE*2)-1;					
	DMA0CNT = 2;  //See dsPIC user's manual. 3 analog reads -> DMA0CNT = 3-1 = 2
	//DMA0CNT = 7;

	DMA0REQ=13; //ADC1 requests

	DMA0STA = __builtin_dmaoffset(BufferA);		
	DMA0STB = __builtin_dmaoffset(BufferB);

	IFS0bits.DMA0IF = 0;			//Clear the DMA interrupt flag bit
        IEC0bits.DMA0IE = 1;			//Set the DMA interrupt enable bit

	DMA0CONbits.CHEN=1;
}

/*****************************************************************************
* Function Name : _DMA0Interrupt
* Description   : Interrupt hander for DMA0 , associated with ADC1 here.
				  Motor BEMF vales are set through setter functions.
* Parameters    : None
* Return Value  : None
*****************************************************************************/
void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)
{
	if(DmaBuffer==0) {
		adc_battery = 	BufferA[0][0];	//AN0
		adc_bemfR = 	BufferA[1][0];	//AN1
		adc_bemfL = 	BufferA[2][0];	//AN11

	} else {

		adc_battery = 	BufferB[0][0];	//AN0
		adc_bemfR = 	BufferB[1][0];	//AN1
		adc_bemfL = 	BufferB[2][0];	//AN11
	}
//        adc_battery = 	BufferA[0][0];	//AN0
//        adc_bemfR = 	BufferA[1][0];	//AN1
//        adc_bemfL = 	BufferA[2][0];	//AN11
	DmaBuffer ^= 1;	 //Toggle between buffers
	IFS0bits.DMA0IF = 0;		//Clear the DMA0 Interrupt Flag
}
// End DMA section

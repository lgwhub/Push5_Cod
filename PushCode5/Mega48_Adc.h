//Mega48_Adc.h
//#include "Mega48_Adc.h"

#ifndef _MEGA48_ADC_H
#define _MEGA48_ADC_H

	#define old_adc 0
	
	//#define FIRST_ADC_CH	6
	#define FIRST_ADC_CH	6
	#define LAST_ADC_CH		7
 
 
 //extern uchar chAdc_Resoult1;
extern uchar chAdc_Resoult6;
extern uchar chAdc_Resoult7;
 
	
	#if old_adc == 0
	
	#define MAX_ADC_NUMBER	16
	
	
	
struct adc_struct 
{
	

	unsigned short int sum[LAST_ADC_CH+1-FIRST_ADC_CH];	
	
	
	unsigned char ch;
	unsigned char number;
	unsigned char complete;
	
	unsigned char Time;
};

extern struct adc_struct Adc;


void InitAdc(void);
void AdcProcess(void);	//ÖÜÆÚ10mS
//void CloseAdc(void);
#endif  // old_adc == 0
#endif
///////////////////

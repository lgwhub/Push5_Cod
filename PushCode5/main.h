#ifndef _MAIN_H
#define _MAIN_H
//    ���1MS��λ��ʱͨ���ӳ���
void DelayMs(unsigned int x);
void work(void);  /*  ��PC��Ҳ��������  */

/* ����ʱָʾ����˸  */
void LedStart(void);
/* ����ת��  */

#if CONFIG_WIRELESS_DECODE
void LedContral(void);
#endif
/////////
void AdcProcess(void);
void Init_Hd(void);

#if CONFIG_SAVE5
void EEPROM_Write(unsigned int iAdr,unsigned char bDat);
unsigned char EEPROM_Read(unsigned int iAdr);
#endif



void PwmContral1(uchar x);	//OC1A PWM1 ����
//void PwmContral2(uchar x);	//OC1B PWM2 ����


#endif









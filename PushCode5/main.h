#ifndef _MAIN_H
#define _MAIN_H
//    软件1MS单位延时通用子程序
void DelayMs(unsigned int x);
void work(void);  /*  在PC上也可以运行  */

/* 启动时指示灯闪烁  */
void LedStart(void);
/* 亮灭转换  */

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



void PwmContral1(uchar x);	//OC1A PWM1 控制
//void PwmContral2(uchar x);	//OC1B PWM2 控制


#endif









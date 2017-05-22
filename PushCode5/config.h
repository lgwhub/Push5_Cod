#ifndef _CONFIG_H
#define _CONFIG_H

////////ѡ���Ź�//////
#define EnWdog 				1

//���������ID��
#define CONFIG_SAVE5		1

//���õ������ߵ������
#define CONFIG_CC1100		1
//���õ������ߵ������
#define CONFIG_433SG		0
#define CC1100_PKT_LEN 32

//��ֹ��ƤȦѹ��̫������ת�˵��׺��Զ���תһ�㣬����
#define CONFIG_UNPUSH		1
//ѡ��������
#define Config_Al_Box		1


//IOCFG0_DIV128;				//���Ծ���Ƶ��
#define CONFIG_26MHZ_OSC_OUT		1
		
//FREQ0_ADJ_260018;		//����1.000153  ��ģ�龧��ƫ�� 	
#define USER_NEW_OSC		1



//��������ʱ��
//#define MAX_RUN_TIME_sec	(72)
#define MAX_RUN_TIME_sec	(720)
//85%�����µ�������ʱ��
#define Time10000Ms					1000
//94%�����µ�������ʱ��
#define Time2000Ms					200

//3.3V�ο��µ�����������
//gpParam->bCurrentRate ( 200 - 220 )
#define CURRENT_RATE	(114)
//#define CURRENT_RATE	(206+24)
//#define CURRENT_RATE	(206+34)
//#define CURRENT_RATE	(206*50/33)
//gpParam->bCurrentRate=CURRENT_RATE;			//���3�� 5A

//�����ת�������趨ֵ�����ֵΪ127  18A   max=256*75%=192
//#define CURRENT_FORWARD	124			
//#define CURRENT_FORWARD	(102)
#define CURRENT_FORWARD	(85)
//�����ת���ɵ��趨ֵ�����ֵΪ127  16A
//#define CURRENT_BACKWARD 110	
//#define CURRENT_BACKWARD 91
#define CURRENT_BACKWARD 78
//#define CURRENT_BACKWARD 75


#define CONFIG_OSSCAL						0
#define CONFIG_TEST_FREQ_PIN		0
///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
/////////////////////����Ϊһ�㲻��Ҫ�ı������////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
#define CONFIG_PWM			1
//FreqPwmRate  x2  x4  x8
#define FREQ_PWM_RATE			8
#define PWM_MAX_VAL			250
//Fpwm=Fosc/2/64/PWM_MAX_VAL = 122HZ


//�ⲿ����
#define CONFIG_EXT_4MHZ_OSC		1


#define Fosc	  (8000000/2)
#define	BAUD_DEFINE(baud)	(Fosc/8/(baud))-1	

#define TIMER_50MS	((Fosc*50/1000)/64)
//#define TIMER_50MS	195

//8bit ��ʱ��0��Ҫ1024��Ƶ
//#define TIMER_10MS_8BIT	((Fosc*10/1000)/1024)
//8bit ��ʱ��0��Ҫ256��Ƶ	//ң����������Ҫ
#define TIMER_10MS_8BIT	((Fosc*10/1000)/256)

//������Ϊ�Զ����ܣ��ſ��������ٰ��ſ���Ϊ�ֶ�����
#define CONFIG_FUNCTION_AUTO	1
//���Ӷ������Զ������о�������
//#define CONFIG_AUTOKEY	0

#endif
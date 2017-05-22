//nomal.c
#include "Nomal.h"



//////////
 /* 在数组内加字符串或数组直到0X00 */
uchar PutString(uchar *ps,uchar *pd,uchar max_ps) 
{uchar len=0; /* 结果长度 */
 uchar temp;
while(len<max_ps)
    {temp=*(ps+len);  /* 原 */
     *(pd+len)=temp;  /* 目的 */
     if(temp==0x00)break;
     else{len++;
         };
    };
return len;   /* 指向pd的0X00位  */
}
///

uchar CompareCharChar(uchar *chr1,uchar *chr2,uchar len)
{
 uchar i;
 for(i=0;i<len;i++)
    {
    if(*(chr1+i)!=*(chr2+i))
          {return 0;
          };
    };
return 1;
}

///////////

uchar MakeValAsc8(uchar *StrHead,uchar Val,uchar *StrEnd,uchar *out)	//8位变量转换为十进制的ASC码，有前后缀
{
 /* 在数组内加字符串或数组直到0X00 */
//uchar PutString(uchar *ps,uchar *pd,uchar max_ps) 
uchar i;

i=PutString(StrHead,out,20);
out[i]=HexToAsc(Val/100%10);i++;
out[i]=HexToAsc(Val/10%10);i++;
out[i]=HexToAsc(Val%10);i++;
i+=PutString(StrEnd,&out[i],20);
return i;
}
/*
uchar MakeValAsc16(uchar *StrHead,uint16 Val,uchar *StrEnd,uchar *out,uchar point)	//16位变量转换为十进制的ASC码，有前后缀
{
 // 在数组内加字符串或数组直到0X00
//uchar PutString(uchar *ps,uchar *pd,uchar max_ps) 
uchar i;
uchar temp;
uchar flag;

flag=0;

	i=PutString(StrHead,out,20);
	temp=Val/10000%10;
	if((temp>0)||flag)
			{
				flag=1;
				out[i]=HexToAsc(temp);i++;
			}
	temp=Val/1000%10;
	if((temp>0)||flag)
			{
				flag=1;
				out[i]=HexToAsc(temp);i++;
			}
	temp=Val/100%10;
	if((temp>0)||flag)
			{
				flag=1;
				out[i]=HexToAsc(temp);i++;
			}
	temp=Val/10%10;
	
	if((temp>0)||flag||point)
			{
				out[i]=HexToAsc(temp);i++;
			}
	
	if(point)	//小数点
			{//括号不能省20081117
		out[i]='.';i++;		
		}
	out[i]=HexToAsc(Val%10);i++;
	
	i+=PutString(StrEnd,&out[i],20);
	return i;
}
*/
//////////////
/*
uchar MakeValAsc32(uchar *StrHead,uint32 Val,uchar *StrEnd,uchar *out,uchar point)	//32位变量转换为十进制的ASC码，有前后缀
{
uchar i;
uchar temp;
uchar flag;
	flag=0;
	i=PutString(StrHead,out,20);
	
	temp=Val/1000000000%10;
			if((temp>0)||flag)
					{
						flag=1;
						out[i]=HexToAsc(temp);i++;
					}
	temp=Val/100000000%10;
			if((temp>0)||flag)
					{
						flag=1;
						out[i]=HexToAsc(temp);i++;
					}
	temp=Val/10000000%10;
			if((temp>0)||flag)
					{
						flag=1;
						out[i]=HexToAsc(temp);i++;
					}
	temp=Val/1000000%10;
			if((temp>0)||flag)
					{
						flag=1;
						out[i]=HexToAsc(temp);i++;
					}
	temp=Val/100000%10;
			if((temp>0)||flag)
					{
						flag=1;
						out[i]=HexToAsc(temp);i++;
					}
	temp=Val/10000%10;
			if((temp>0)||flag)
					{
						flag=1;
						out[i]=HexToAsc(temp);i++;
					}
	temp=Val/1000%10;
			if((temp>0)||flag)
					{
						flag=1;
						out[i]=HexToAsc(temp);i++;
					}
	temp=Val/100%10;
			if((temp>0)||flag)
					{
						flag=1;
						out[i]=HexToAsc(temp);i++;
					}
	temp=Val/10%10;
	if((temp>0)||flag||point)
			{
				out[i]=HexToAsc(temp);i++;
			}
	if(point)	//小数点
		{
			out[i]='.';i++;	
		}
	temp=Val%10;
	out[i]=HexToAsc(temp);i++;
	i+=PutString(StrEnd,&out[i],20);
	return i;

}
*/
/*
uchar MakeValHex16(uchar *StrHead,uint Val,uchar *StrEnd,uchar *out)	//16位变量转换为HEX的ASC码，有前后缀
{
 //在数组内加字符串或数组直到0X00
//uchar PutString(uchar *ps,uchar *pd,uchar max_ps) 
uchar i;

i=PutString(StrHead,out,20);
out[i]=HexToAsc(Val/0x1000%0x10);i++;
out[i]=HexToAsc(Val/0x100%0x10);i++;
out[i]=HexToAsc(Val/0x10%0x10);	i++;
out[i]=HexToAsc(Val%0x10);i++;
i+=PutString(StrEnd,&out[i],20);
return i;
}
*/
/********************************************************************************/
/*
uchar IsNumberAsc(uchar x)   //判断是否为数字0~9
{if(x>'9')
     {return 0;
     }
 else if(x<'0')
     {return 0;
     }
 else{return 1;
     };
}
*/
/************************************************************************/
/*                                                                      */
/*                     ASC_2码  转换通用子程序                           */
/*                                                                      */
/*          HexToAsc  半字节二进制转换为1位ASC_2码   子程序              */
/************************************************************************/
uchar HexToAsc(uchar x)   /*  1/2hex>>1asc2   */
{ x&=0x0f;
 if(x>9){x-=0x0a;x+='A';}else{x+='0';};
 return x;
}

/************************************************************************/
/*          AscToHex  1位ASC_2码转换为半字节二进制   子程序              */
/*                        1ASC>>1/2HEX                                  */
/************************************************************************/

uchar AscToHex(uchar x)     
{
x&=0xFF;	
if(x>='a')
    {x-=87;//{x-='a';x+=0x0a;}
    }
else if(x>'9')
    {x-=55;//{x-='A';x+=0x0a;
    }
else{x-='0';
    };
x&=0x0f;
return x;
}

//////////////////////////////////////////////////////////////////////////
/*
uint16 NumberAscStringToInt16(uchar *p)	//最多5位ASC码的数字字符串转换为16位无符号整型
{
uint32 word2;
uchar i;
word2=0;
for(i=0;i<5;i++)
	{
	if(IsNumberAsc(*p))
        	{
        	word2=word2*10+AscToHex(*p);
        	}
        else break;
        p++;
        };
if(word2>65535)word2=65535;
   return  (uint16)word2;    	
}
*/
/////////////////////////////////////////////

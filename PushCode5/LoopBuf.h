#ifndef __LOOP_BUF_H
#define __LOOP_BUF_H
#define UART0_RECV_BUF_SIZE	36
#define UART0_SEND_BUF_SIZE	38
//#define	SizeDataType	unsigned short int
#define	SizeDataType	unsigned char

typedef struct
{
SizeDataType 		len;		//已有长度
SizeDataType		addTail;	//输入数据尾
//uint16 	outHead;	//输出数据头
SizeDataType 		outTail;	//输出数据尾
}_loopbuf;


/////
void ClearLoopBuf(_loopbuf *st,SizeDataType BufSize);				//清除环型缓冲区
unsigned char AddLoopBuf(_loopbuf *st,uchar *buf,SizeDataType BufSize,uchar chr);	//加入到环型缓冲区
unsigned char GetLoopbuf(_loopbuf *st,uchar *buf,SizeDataType BufSize,uchar *chr);	//从环型缓冲区提取



//extern uchar Uart0RecvBuf[UART0_RECV_BUF_SIZE+6];	//串口0接收缓冲区
//extern _loopbuf Uart0RecvStruct;
extern uchar Uart0SendBuf[UART0_SEND_BUF_SIZE+6];	//串口0接收缓冲区
extern _loopbuf Uart0SendStruct;

////
uchar Uart0CharSend(uchar x);

void SendText_UART0(uchar *StrData);
uchar GetOneFromUart0(uchar *buf);

#endif

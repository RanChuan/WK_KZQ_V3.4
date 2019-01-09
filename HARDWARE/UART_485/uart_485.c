#include "uart_485.h"
#include "delay.h"
#include "usart.h"
#include "config.h"
#include "crc8_16.h" 
#include "control.h"

#ifdef USART_485_RX_EN   	//���ʹ���˽���


//���ջ����� 	
u8 RS485_RX_BUF[100];  	//���ջ���,���64���ֽ�.
//���յ������ݳ���
u8 RS485_RX_CNT=0;   		  

u32 err_shui_num=0;     //©ˮ��������


void USART3_IRQHandler(void)
{
	u8 res;	    
 
 	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //���յ�����
	{	  
		res =USART_ReceiveData(USART3); 	//��ȡ���յ�������
//		USART1->DR = res;
		if(RS485_RX_CNT<100)
		{
			RS485_RX_BUF[RS485_RX_CNT]=res;		//��¼���յ���ֵ
			RS485_RX_CNT++;						//������������1 
		}
	}  											 
} 
#endif	

void USART3_Init(u32 bound)
{  
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ʹ��GPIOA,Gʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//ʹ��USART2ʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;				 //PG9�˿�����
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	//PA2
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//��������
  GPIO_Init(GPIOB, &GPIO_InitStructure);
   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PA3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
  GPIO_Init(GPIOB, &GPIO_InitStructure);  

	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3,ENABLE);//��λ����2
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3,DISABLE);//ֹͣ��λ
 
	
 #ifdef USART_485_RX_EN		  	//���ʹ���˽���
	USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8λ���ݳ���
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;///��żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//�շ�ģʽ

    USART_Init(USART3, &USART_InitStructure); ; //��ʼ������
  
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn; //ʹ�ܴ���2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; //�����ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure); //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
 
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�����ж�
   
    USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ��� 

 #endif

 USART_485_TX_EN=0;			//Ĭ��Ϊ����ģʽ

}
//RS485����len���ֽ�.
//buf:�������׵�ַ
//len:���͵��ֽ���(Ϊ�˺ͱ�����Ľ���ƥ��,���ｨ�鲻Ҫ����64���ֽ�)
void RS485_Send_Data(u8 *buf,u8 len)
{
	u8 t;
	USART_485_TX_EN=1;			//����Ϊ����ģʽ
  	for(t=0;t<len;t++)		//ѭ����������
	{		   
		while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	  
		USART_SendData(USART3,buf[t]);
	}	 
 
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);		
	RS485_RX_CNT=0;	  
	USART_485_TX_EN=0;				//����Ϊ����ģʽ	
}
//RS485��ѯ���յ�������
//buf:���ջ����׵�ַ
//len:���������ݳ���
void RS485_Receive_Data(u8 *buf,u8 *len)
{
	delay_ms(2000);
	u8 rxlen=RS485_RX_CNT;
	u8 i=0;
	*len=0;				//Ĭ��Ϊ0

//	if(rxlen==RS485_RX_CNT&&rxlen)//���յ�������,�ҽ��������
//	{
	if (RS485_RX_CNT>=7)
	{
		for(i=0;i<RS485_RX_CNT;i++)
		{
			buf[i]=RS485_RX_BUF[i];	
		}		
		*len=RS485_RX_CNT;	//��¼�������ݳ���
		RS485_RX_CNT=0;		//����
	}
}



//©ˮ������1��������0��������
u8 water_err (void)
{
	err_shui_num++;
	if(err_shui_num<10000){
		return 0;
	}
	err_shui_num=0;
	u8 crc[2]={0};
	u8 length;
	u8 buff[20]={0};
	buff[0]=0x03;
	buff[1]=0x04;
	buff[2]=0x00;
	buff[3]=0x01;
	buff[4]=0x00;
	buff[5]=0x01;
	Get_Crc16(buff,6,crc);
	buff[6]=crc[0];
	buff[7]=crc[1];
	RS485_Send_Data(buff,8);
	buff[0]=0;
	RS485_Receive_Data(buff,&length);
	
	
	if(length != 7){
		return 0;
	}
	Get_Crc16(buff,length-2,crc);	
	if (buff[5]==crc[0] && buff[6]==crc[1])
	{
		if (!buff[4])
		{
			WARN_WATER=0;//�ޱ���
			return 0;
		}
		else
		{
			WARN_WATER=1;//��©ˮ����
			return 1;
		}
	}
	WARN_WATER=2;//û�н�©ˮ����
	return 2;
	
}







// �������飬0,�ɹ���1��ʧ��
u8 yiti_power(u8 cmd) 
{
	u8 data[10]={0};
	u8 recv[10]={0};
	u8 crc[2]={0};
	u8 len=0;
	data[0]=0x01;
	data[1]=0x06;
	data[2]=0x00;
	data[3]=0x01;
	if (cmd==1)
	{
		
	data[4]=0xff;
	data[5]=0x00;
	}
	else
	{
	data[4]=0x00;
	data[5]=0x00;
	}
	Get_Crc16(data,6,crc);
	data[6]=crc[0];
	data[7]=crc[1];
	RS485_Send_Data(data,8);
	delay_ms(200);
	RS485_Receive_Data(recv,&len);
//	if (len==8)
	{
		CS_STATE&=0x00ff;
		CS_STATE|=cmd<<8;
		control_run(DEVICE_ON,cmd);
		return 0;
	}
//	else
	{
		return 1;
	}
}



// �趨ʪ��ֵ��0,�ɹ���1��ʧ��
u8 yiti_set(u8 set) 
{
	u8 data[10]={0};
	u8 recv[10]={0};
	u8 crc[2]={0};
	u8 len=0;
	data[0]=0x01;
	data[1]=0x06;
	data[2]=0x00;
	data[3]=0x03;
	data[4]=set;
	data[5]=0x00;
	Get_Crc16(data,6,crc);
	data[6]=crc[0];
	data[7]=crc[1];
	RS485_Send_Data(data,8);
	delay_ms(200);
	RS485_Receive_Data(recv,&len);
//	if (len==8)
	{
		CS_STATE&=0xff00;
		CS_STATE|=set;
		return 0;
	}
//	else
	{
		return 1;
	}
}








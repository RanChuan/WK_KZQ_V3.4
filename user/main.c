#include "system_init.h"
#include "config.h"




int main()
{	
	u8 buff[50]={0,0,0,0,0,0};

	u16 i=0;
	u8 GREE_DOW[10]={0x90,0x00,0x00,0x0a,0x40,0x00,0x40,0xf0,0xff,0x00};
	void DeviceLEDType (u16 i);
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x2800);

	System_Init();//初始化
	
	
	while(1)
	{
		
		
		//-----命令解析以及执行模块-------//
		if (AUX) 													//
			USART1_GetCmd (buff,50);				//
		else buff[0]=0;										//
		if (buff[0]!=0x00)								//
		{																	//
			Get_cmd (buff);									//
		}																	//
		//----------------------------------
		
		
		
		
		
		//--------运行状态指示灯----------------//
		DeviceLEDType(i);												//
		i++;																		//
		if (i>100) i=0;													//
		//--------------------------------------//
		
		
		
		
		//--------控制板上实体按键---------------//
		Cfg_Model ( );				//配置模式查询函数 //
		//---------------------------------------//
		
		
		
		//----------------漏水报警-----------//
		water_err();												 //
		//-----------------------------------//
		
//		delay_ms(200);
//		Remote_send_USER(2);


		//------------重装载看门狗------------//
		IWDG_Feed( );													//
		//------------------------------------//
	}
}


void DeviceLEDType (u16 i)
{
	u8 t=0;
	if (MY_STYLE==MYSTYLE_KT)
	{
		t=STYLE_KT;
	}
	else if (MY_STYLE==MYSTYLE_CS)
	{
		t=STYLE_CS;
	}
	else if (MY_STYLE==MYSTYLE_JH)
	{
		t=STYLE_JH;
	}
	else if (MY_STYLE==MYSTYLE_JS)
	{
		t=STYLE_CS;
	}
	else if (MY_STYLE==MYSTYLE_YT)
	{
		t=STYLE_CS;
	}
	else
	{
		t=0;
	}


	if (i==50) control_run (t,1);	//
	if (i==100) control_run (t,0);	//

}



#include "system_init.h"
#include "config.h"

//系统初始化，初始化顺序不要随意修改
void System_Init()
{

	NVIC_Configuration();	 
	delay_init();	    					//延时函数初始化
	control_init ( );						//控制初始化
	Config_Init( );							//配置初始化要在控制初始化之后
	Remote_init (38000);				//红外载波频率是38000hz
	Receive_init ( );						//红外接收初始化
	
	
	
	
	USART1_Init(115200);				//RF传递信息串口
	AS32_Param_Init( );					//无线模块初始化
	TIM4_TIMEout_init();				//超时自动关设备定时器
	USART3_Init(9600);					//串口3是漏水报警
	IWDG_Init(7,60000) ;				//独立看门狗初始化
}




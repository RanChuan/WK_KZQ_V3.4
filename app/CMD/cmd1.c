#include "uart_485.h"


#include "cmd.h"
#include "config.h"
#include "drv_as62.h"
#include "stmflash.h"
#include "receive.h"
/*
			这个文件用来支持新的多条协议

*/

u32 RUN_TIME_S=0;//从上电或者上次重启到现在的工作时间，单位秒

extern u16 CS_CANTCMD;//在cmd.c中定义的
							//控制命令超时自动关闭计时器，单位，秒
extern u16 MY_OFFTIME;




/*****************************************************************

							函数作用：对输入数据进行有效性验证
												对输入的数据进行crc校验，比对地址，
												根据命令的类型不同分发给各执行函数

									参数：输入的命令数据



****************************************************************/
void Get_cmd_NEW_MODE (u8 *buff1)
{
	u16 datalength=0;//命令中数据的长度（不是总长度）
	u16 temp=0;
	u8 crc[2];
	
								//数据头校验以及长度获取
	if (buff1[0]==0xff&&buff1[1]==0xff)
	{
		datalength=(buff1[5]<<8)|buff1[6];
	}
	else
		return;
	
								//进行CRC校验
	Get_Crc16(buff1,datalength+7,crc);
	if ((crc[0]==buff1[datalength+7])&&(crc[1]==buff1[datalength+8]))//校验成功，命令有效
	{
	}
	else 
		return ;
	
	
								//进行地址校验
	temp=(buff1[2]<<8)|buff1[3];
	if (temp==MY_ADDR||((buff1[4]>=0x06)&&buff1[4]<=0x08))
	{
		MY_OFFTIME=0;
		switch (buff1[4])
		{
			case 0x01://获取系统参数
				cmd_0x01 (buff1);
				break;
			case 0x02://查询设备故障
				cmd_0x02 (buff1);
				break;
			case 0x03://控制设备
				cmd_0x03 (buff1);
				break;
			case 0x04://获取内存变量
				cmd_0x04 (buff1);
				break;
			case 0x05://获取设备flash
				cmd_0x05 (buff1);
				break;
			case 0x06://配置模式
				cmd_0x06 (buff1);
				break;
			case 0x07:
				cmd_0x07 (buff1);
				break;
			case 0x08:
				cmd_0x08 (buff1);
				break;
			case 0x09:
				cmd_0x09 (buff1);
				break;
			default:
				Return_NEW(buff1[4],ERR_NULLCMD);//不支持的命令
				break;
		}
	}
	else 
		return;
}



			//返回系统参数
void cmd_0x01 (u8 *buff)
{
	u8 data[30];
	u8 crc[2];
	
	data[0]=0xff;
	data[1]=0xff;
	data[2]=MY_ADDR>>8;
	data[3]=MY_ADDR;
	data[4]=buff[4]|0x80;
	data[5]=0x00;
	data[6]=14;
	data[7]=0;		//错误类型
	data[8]=0;
				
					//数据位
	data[9+0]=MY_STYLE;//本机类型
	data[9+1]=IF_WORK;//是否在工作
	data[9+2]=RUN_TIME_S>>24;
	data[9+3]=RUN_TIME_S>>16;
	data[9+4]=RUN_TIME_S>>8;
	data[9+5]=RUN_TIME_S;
	data[9+6]=IF_HAND;
	
				//根据自身的类型填充这2位
	if (MY_STYLE==MYSTYLE_KT)
	{
		data[9+7]=KT_STATE;
		data[9+8]=KT_STATE1;
		data[9+10]=KT_STYLE;
		data[9+11]=WARN_WENDU;
	}
	else if (MY_STYLE==MYSTYLE_CS)
	{
		data[9+7]=CS_STATE;
		data[9+8]=0;
		data[9+9]=CS_CANTCMD;
		data[9+11]=WARN_SHIDU;
		data[9+11]=1;
	}
	else if (MY_STYLE==MYSTYLE_JS)//加湿和除湿使用同一个字节
	{
		data[9+7]=CS_STATE;
		data[9+8]=0;
		data[9+9]=CS_CANTCMD;
		data[9+11]=WARN_SHIDU;
	}
	else if (MY_STYLE==MYSTYLE_YT)//恒湿机高8位是开关状态，低8位是设定的湿度值
	{
		data[9+7]=CS_STATE>>8;
		data[9+8]=CS_STATE;
		data[9+11]=WARN_WATER;
//		data[9+11]=1;
	}
	else if (MY_STYLE==MYSTYLE_JH)
	{
		data[9+7]=JH_STATE;
		data[9+8]=0;
		data[9+11]=WARN_TVOC;
	}
	
	Get_Crc16(data,9+12,crc);
	data[9+12]=crc[0];
	data[9+13]=crc[1];
	AS32_TX_none(data,9+14);//发送返回数据
	
}


						//查询设备故障
void cmd_0x02 (u8 *buff)
{
	u8 data[30];
	u8 crc[2];
	
	data[0]=0xff;
	data[1]=0xff;
	data[2]=MY_ADDR>>8;
	data[3]=MY_ADDR;
	data[4]=buff[4]|0x80;
	data[5]=0x00;
	data[6]=12;
	data[7]=0;		//错误类型
	data[8]=0;
	
							//有没有故障，1，有
	if (MY_STYLE==MYSTYLE_KT)
	{
		if (WARN_WENDU) data[9+0]=1;
	}
	else if (MY_STYLE==MYSTYLE_CS)
	{
		if (WARN_SHIDU) data[9+0]=1;
	}
	else if (MY_STYLE==MYSTYLE_JH)
	{
		if (WARN_TVOC) data[9+0]=1;
		if (WARN_PM2_5) data[9+0]=1;
	}
	
	data[9+1]=WARN_WATER;
	Get_Crc16(data,9+2,crc);
	data[9+2]=crc[0];
	data[9+3]=crc[1];
	AS32_TX_none(data,9+4);//发送返回数据
}

								//控制命令执行
void cmd_0x03 (u8 *buff)
{
	u8 data[30];
	u8 crc[2];
	
	data[0]=0xff;
	data[1]=0xff;
	data[2]=MY_ADDR>>8;
	data[3]=MY_ADDR;
	data[4]=buff[4]|0x80;
	data[5]=0x00;
	data[6]=2;
	data[7]=0;		//错误类型
	data[8]=0;

	
	if (buff[7]!=MY_STYLE)
	{	
		Return_NEW (buff[4],ERR_NOTMYTYPE);
		return ;
	}
	else
	{
		MyDevice_On(buff[8],buff[9]);
	}
	Get_Crc16(data,9,crc);
	data[9]=crc[0];
	data[9+1]=crc[1];
	AS32_TX_none(data,9+2);//发送返回数据

}

							//存取设备内存变量
void cmd_0x04 (u8 *buff)
{
	u8 data[30];
	u8 crc[2];
	
	data[0]=0xff;
	data[1]=0xff;
	data[2]=MY_ADDR>>8;
	data[3]=MY_ADDR;
	data[4]=buff[4]|0x80;
	data[5]=0x00;
	data[6]=2;
	data[7]=0;		//错误类型
	data[8]=0;


	if (buff[8]==1)//写入
	{
		if (buff[7]==1)
		{
			LIMIT_WENDU_H=(buff[9]<<8)|buff[10];
			LIMIT_WENDU_L=(buff[11]<<8)|buff[12];
		}
		else if (buff[7]==2)
		{
			LIMIT_SHIDU_H=(buff[9]<<8)|buff[10];
			LIMIT_SHIDU_L=(buff[11]<<8)|buff[12];
		}
		else if (buff[7]==3)
		{
			LIMIT_TVOC=(buff[9]<<8)|buff[10];
			LIMIT_PM2_5=(buff[11]<<8)|buff[12];
		}
		else
		{
			Return_NEW (buff[4],ERR_DATAVALE);
			return ;			
		}
		Get_Crc16(data,9,crc);
		data[9]=crc[0];
		data[9+1]=crc[1];
		AS32_TX_none(data,9+2);//发送返回数据
	}
	
	
	else if (buff[8]==0)
	{
		data[6]+=4;
		if (buff[7]==1)
		{
			buff[9]=LIMIT_WENDU_H>>8;
			buff[10]=LIMIT_WENDU_H;
			buff[11]=LIMIT_WENDU_L>>8;
			buff[12]=LIMIT_WENDU_L;
		}
		else if (buff[7]==2)
		{
			buff[9]=LIMIT_SHIDU_H>>8;
			buff[10]=LIMIT_SHIDU_H;
			buff[11]=LIMIT_SHIDU_L>>8;
			buff[12]=LIMIT_SHIDU_L;
		}
		else if (buff[7]==3)
		{
			buff[9]=LIMIT_TVOC>>8;
			buff[10]=LIMIT_TVOC;
			buff[11]=LIMIT_PM2_5>>8;
			buff[12]=LIMIT_PM2_5;

		}
		else
		{
			Return_NEW (buff[4],ERR_DATAVALE);
			return ;			
		}
		Get_Crc16(data,13,crc);
		data[13]=crc[0];
		data[14]=crc[1];
		AS32_TX_none(data,15);//发送返回数据
	}
}


						//存取flash
void cmd_0x05 (u8 *buff)
{
	u8 data[300];
	u8 crc[2];
	u32 flash_addr=0;
	u16 want=0;
	u16 want_i=0;
	u16 i=0;
	u16 num_now=0;
	u16 num_all=0;
	u16 buf[100]={0};
	data[0]=0xff;
	data[1]=0xff;
	data[2]=MY_ADDR>>8;
	data[3]=MY_ADDR;
	data[4]=buff[4]|0x80;
	data[5]=0x00;
	data[6]=2;
	data[7]=0;		//错误类型
	data[8]=0;

	if (buff[7]==1)//写入
	{
		
	}
	else if (buff[7]==0)//读取
	{
		flash_addr=(buff[10]<<24)|(buff[11]<<16)|(buff[12]<<8)|buff[13];
		want=(buff[8]<<8)|buff[9];
		
		num_all=want/200;
		if (want%200) num_all++;
		while(want)
		{
			for (i=0;i<100;i++) buf[i]=0;
			if (want>200)
			{
				want_i=200;
				want-=200;
			}
			else
			{
				want_i=want;
				want=0;
			}
			STMFLASH_Read(flash_addr,buf,want_i/2);
			flash_addr+=want_i/2;
			
			data[6]=want_i+2;
			num_now++;
			data[9]=num_now;
			data[10]=num_all;
			for (i=0;i<want_i;i++)
			{
				data[11+i]=buf[i]>>8;
				data[12+i]=buf[i];
			}
			
			Get_Crc16(data,want_i+2+7,crc);
			data[want_i+2+7]=crc[0];
			data[want_i+2+8]=crc[1];
			AS32_TX_none(data,want_i+2+9);//发送返回数据
			
		}
	}
}

						
void cmd_0x06 (u8 *buff)
{
	u8 data[30]={0};
	u8 crc[2]={0};
	data[0]=0xff;
	data[1]=0xff;
	data[2]=MY_ADDR>>8;
	data[3]=MY_ADDR;
	data[4]=buff[4]|0x80;
	data[5]=0x00;
	data[6]=3;
	data[7]=0;		//错误类型
	data[8]=0;
	MY_MODEL=buff[7];
	data[9]=MY_MODEL;
	control_run(MODEL,MY_MODEL);
	if (MY_MODEL==0) {Config_Init( );	}
	Get_Crc16(data,data[6]+7,crc);
	data[data[6]+7]=crc[0];
	data[data[6]+8]=crc[1];
	AS32_TX_none(data,data[6]+9);//发送返回数据
	
}
void cmd_0x07 (u8 *buff)
{
	if (MY_MODEL)
	{
		MY_ADDR=(buff[7]<<8)|buff[8];
		MY_STYLE=buff[9];
		if (MY_STYLE==MYSTYLE_KT)
		{
			KT_STYLE=buff[10];
		}
		Save_Config();
		Return_NEW(buff[4],ERR_SUCCESS);
	}
	else
	{
		Return_NEW(buff[4],ERR_CANNOTCFG);
	}
}
void cmd_0x08 (u8 *buff)
{
	if (MY_MODEL==0)
	{
		Return_NEW(buff[4],ERR_CANNOTCFG);
		return;
	}
	if (buff[7]!=0x02)
	{
		Return_NEW(buff[4],ERR_DATAVALE);
		return;
	}
	if (1)
	{
		if (buff[8]==0x01)
		{
			control_run(CMD_UP,0);
			control_run(DEVICE_ON,1);
			control_run(CMD_DOWN,0);
		}
		else if (buff[8]==0x02)
		{
			control_run(CMD_UP,1);
			control_run(DEVICE_ON,0);
			control_run(CMD_DOWN,1);
		}
		else if (buff[8]==0x03)
		{
			control_run(CMD_UP,1);
			control_run(DEVICE_ON,0);
			control_run(CMD_DOWN,0);
		}
		else if (buff[8]==0x04)
		{
			control_run(CMD_UP,0);
			control_run(DEVICE_ON,0);
			control_run(CMD_DOWN,1);
		}
		KT_USER_CMD=buff[8];
		Clear_Cupture();//清除已存在的捕获缓冲器数据
		Return_NEW(buff[4],ERR_SUCCESS);
	}
	
}
void cmd_0x09 (u8 *buff)
{
	if (buff[7]==2)
	{
		Return_NEW(buff[4],ERR_SUCCESS);
		delay_ms(100);
		NVIC_SystemReset();
	}
	else
	{
		Return_NEW(buff[4],ERR_DATAVALE);
	}
}







void Return_NEW (u8 cmd,u16 err)
{
	u8 data[20];
	u8 crc[2];
	
	data[0]=0xff;
	data[1]=0xff;
	data[2]=MY_ADDR>>8;
	data[3]=MY_ADDR;
	data[4]=cmd|0x80;
	data[5]=0x00;
	data[6]=0x02;
	data[7]=err>>8;		//错误类型
	data[8]=err;
	
	Get_Crc16(data,9,crc);
	data[9]=crc[0];
	data[10]=crc[1];
	AS32_TX_none(data,11);//发送返回数据
	
}






void MyDevice_On (u8 state,u8 updown)
{
	if (MY_STYLE==MYSTYLE_CS)
	{
		if (state==1)
		{
			if (updown==1)//加湿
			{
				Cmd_cs_off();
			}
			else if (updown==2)//除湿
			{
				Cmd_cs_on();
			}
		}
		else
		{
			Cmd_cs_off();
		}
	}
	else if (MY_STYLE==MYSTYLE_KT)
	{
		if (state==1)
		{
			if (updown==1)//加温
			{
				Cmd_kt_up();
			}
			else if (updown==2)//降温
			{
				Cmd_kt_down();
			}
			else
			{
				Cmd_kt_on();
			}
		}
		else
		{
			Cmd_kt_off();
		}
	}
	else if (MY_STYLE==MYSTYLE_JH)
	{
		if (state==1)
		{
			if (updown==1)//加空气微粒，不可能有这个命令，
			{
				Cmd_jh_off();
			}
			else if (updown==2)//降空气微粒
			{
				Cmd_jh_on();
			}
			else
			{
				Cmd_jh_on();
			}
		}
		else
		{
			Cmd_jh_off();
		}
	}
	else if (MY_STYLE==MYSTYLE_JS)
	{
		if (state==1)
		{
			if (updown==1)//加湿气
			{
				Cmd_cs_on();		//由于和除湿机使用不同的板子，可以使用除湿机的控制函数
			}
			else if (updown==2)//降
			{
				Cmd_cs_off();
			}
			else
			{
				Cmd_cs_on();
			}
		}
		else
		{
			Cmd_cs_off();
		}
	}
	else if (MY_STYLE==MYSTYLE_YT)
	{
		yiti_power(state);
		delay_ms(100);
		yiti_set(updown);
	}
	
}



















			//系统开机时间计数中断，每10ms一次中断
void Run_Time_IRQ (void)
{
	static u8 i=0;
	i++;
	if (i>=100)
	{
		RUN_TIME_S++;
		i=0;//修复设备运行时间计时不准确，2018.8.17
	}
}
















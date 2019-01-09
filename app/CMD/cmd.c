#include "cmd.h"
#include "config.h"
#include "drv_as62.h"
#include "uart_485.h"
#include "receive.h"


				//模式1用到的协议
void Cmd_return(u16 num);//返回数据
void Cmd_err (void);//出错
void Cmd_Icant (void); //不支持的命令




					//模式2的数据返回函数，cmd为0x03或0x06
void Cmd_return_MODE2 (u8 cmd,u16 datalength);
void Cmd_err_MODE2 (u8 cmd,u16 err);				//模式2的通信错误返回



							//控制命令超时自动关闭计时器，单位，秒
 u16 MY_OFFTIME=0;

										//除湿机启动停止不可操作标志
										//开启5分钟不可关闭，关闭5分钟不可打开
u16 CS_CANTCMD=0;
static u16 CS_BUSY_TIME=5*60*100;//5分钟CD，递减变量
static u8  CS_LAST_CMD=0;//除湿机在CD结束后执行的命令
#define  CS_OUTTIME 5*60*100


//除湿机，空调，升降温，空气净化器


		//格力空调电辅热升温至30度
const u8 GREE_UP[10]={0x30,0x70,0x00,0x0a,0x40,0x00,0x40,0x00,0x60};
		//格力空调制冷模式降温至16度
const u8 GREE_DOWN[10]={0x90,0x00,0x00,0x0a,0x40,0x00,0x40,0x00,0xa0};
		//格力空调自动模式开机
const u8 GREE_ON[10]={0x90,0x90,0x04,0x8a,0x40,0x00,0x40,0x00,0xf0};
		//格力空调自动模式关机
const u8 GREE_OFF[10]={0x00,0x90,0x00,0x0a,0x40,0x00,0x40,0x00,0xa0};



		//美的空调电辅热升温至30度
const u8 MIDEA_UP[10]={0xb2 ,0x4d ,0xbf ,0x40 ,0xbc ,0x43,0x00,0x00};
		//美的空调制冷模式降温至16度
const u8 MIDEA_DOWN[10]={0xb2 ,0x4d ,0xbf ,0x40 ,0x00 ,0xff,0x00,0x00};
		//美的空调自动模式开机
const u8 MIDEA_ON[10]={0xb2, 0x4d ,0x1f ,0xe0 ,0xd8 ,0x27,0x00,0x00};
		//美的空调自动模式关机
const u8 MIDEA_OFF[10]={0xb2 ,0x4d ,0x7b ,0x84 ,0xe0 ,0x1f,0x00,0x00};




/*************************************************************************************

读取命令格式：
0xf0,0xfe,addr=MY_ADDR,cmd=0x03,fun_h=0,fun_l=0x02,data0=0,data1=0x04,crc_h,crc_l
本机支持的读取命令固定为10个字节
有等号的变量的值只能为等号右边的固定值
fun_0,fun_1是功能标识，代表是控制器
如果检测到格式不对会发出错误信息或者不予应答

读取返回数据格式：
0xf0 ,0xfe ,addr ,cmd=0x03 ,msg0=0x00 ,mag1=0x02 ,long_h=0x00 ,long_l=0x04,
MY_STATE[0],MY_STATE[1],MY_STATE[2],MY_STATE[3],crc_h,crc_l
本机的返回数据固定为14字节，并且只会在读取命令期望读取的数据长度为4时才会返回
本机返回的有等号的变量固定为等号右边的值，不会因为运行条件的变化而变化
没有等号的变量可能是以下这些值：
MY_STATE[0]：0，除湿机关；		1，除湿机开									
MY_STATE[1]：0，空调,关；			1，空调开										
MY_STATE[2]：0，无状态；			1，空调升温；		2，空调降温	
MY_STATE[3]：0，空气净化器关（本机暂时不支持空气净化器的控制，固定为0）


控制命令格式：
0xf0,0xfe,addr=MY_ADDR,cmd=0x06,msg0,msg1,fun_h=0,fun_l=0x02,data0,data1,crc_h,crc_l
本机支持的控制命令固定为12个字节
fun_0,fun_1是功能标识，代表是控制器
有等号的变量是固定的，本机只能接受这种格式的命令
本机通过对msg0和msg1所附带值的判断来控制除湿机和空调的启停
			-------------------------
			|   msg0    |   msg1    |
			-------------------------
			|   0x10    |   0x11    |温度太高，通过此可开空调并开降温
			-------------------------
			|   0x10    |   0x12    |温度太低，通过此可开空调并开升温
			-------------------------
			|   0x20    |   0x21    |湿度太高，通过此可开除湿机
			-------------------------
			|   0x20    |   0x22    |湿度太低，通过此可关除湿机
			-------------------------
			程序中只通过msg0和msg1来控制除湿机
			
没有等号的变量可以有以下这些值：						
			-------------------------							
			|data0			|  data1		|							
			-------------------------							
			|		0				|			0			|空调关				
			-------------------------							
			|		0				|			1			|空调关				
			-------------------------							
			|		0				|			2			|空调关				
			-------------------------							
			|		1				|			0			|空调开				
			-------------------------							
			|		1				|			1			|空调开，升温	
			-------------------------							
			|		1				|			2			|空调开，降温	
			-------------------------							
如果检测到格式不对会发出错误信息或者不予应答

*****************************************************************************************/
#define __FR_DEBUG 0



						//命令解析函数指针
void (*Get_cmd) (u8 *buff1);




						//模式1的命令解析
void Get_cmd_MODE1 (u8 *buff1)
{
	u16 find_num=0;
	u8 *buff=buff1;
	u8 crc[2];
	u16 byte_long=0;//期望返回的字节长度
	u8 if_cmd=0;//如果有可解析命令，默认没有
	u8 if_cfg_cmd=0;//如果是配置命令
	
	while (*buff)
	{
		
		//寻找帧头
		if ((*buff==0xf0)&&(*(buff+1)==0xfe))
		{
			//判断命令类型，取得crc校验数据长度
			if (buff[4]==0x00)
			{
				if (buff[3]==0x06)
				{
					find_num=10;
					
				}
				else if (buff[3]==0x03)
				{
					find_num=8;
				}
			}
			else if (buff[4]==0x10)
			{
				if (buff[5]!=MYSTYLE_KT) find_num=14;
				else find_num=14;
				if_cfg_cmd=1;
			}
			//crc校验
			Get_Crc16(buff,find_num,crc);
			//如果校验成功，设置命令有效标志位
			if ((crc[0]==buff[find_num])&&(crc[1]==buff[find_num+1]))//校验成功，命令有效
			{
				if_cmd=1;
			}
			else
				Cmd_err();
			break;
		}
		buff++;
	}
	
	
	
	//配置命令哟,配置命令只有在配置模式下才会被读取
	if (if_cfg_cmd==1&&if_cmd==1)
	{
		if (buff[3]==0x06)
		{
			if (Set_MyStyle(buff[5])==0xff)//设置类型失败
					return;
			MY_ADDR=buff[9];//设置地址
			if (buff[5]==MYSTYLE_KT) KT_STYLE=buff[10];//如果被配置为空调，设置空调的类型
			WORK_MODE=buff[11];
			CJ_ADDR=(buff[12]<<8)|buff[13];
			Save_Config();
			MY_MODEL=0;			//退出配置模式
			control_run(MODEL,0);
												//关掉所有类型指示灯
			control_run(STYLE_KT,0);
			control_run(STYLE_CS,0);
			control_run(STYLE_JH,0);
		}
		return;
	}
	
	
	
	
	
	
	
	
	//命令有效，进行命令处理
	if (if_cmd)
	{
		//地址不对，退出
		if (buff[2]!=MY_ADDR)
		{
			return ;
		}
		
		
		//读取状态命令
		if (buff[3]==0x03)//读取状态
		{
			if (buff[4]==0x00&&buff[5]==MY_STYLE)//这个命令是对控制器发出的
			{
				byte_long= (buff[6]<<8)+buff[6+1];
				Cmd_return(byte_long);
			}
			else
			{
				//不是，不应答
			}
		}
		//控制命令
		else if (buff[3]==0x06)//命令
		{
			if (buff[6]==0x00&&buff[7]==MY_STYLE)//这个命令是对控制器发出的
			{
				//根据信息表中的值调节除湿机空调
				if (MY_STYLE==MYSTYLE_CS)//控制器类型为除湿机时
				{
					if (buff[4]==0x20)//湿度
					{
						if (buff[5]==0x21)//湿度过高
						{
							Cmd_cs_on();
						}
						else if (buff[5]==0x22)//湿度过低
						{
							Cmd_cs_off();
						}
						else
						{
							Cmd_Icant();
						}
//						return ;//除湿机不关心后面的
					}
				}
				
				else if (MY_STYLE==MYSTYLE_KT)//控制器类型为空调时
				{
//					if (buff[4]==0x10)//温度度
//					{
//						if (buff[5]==0x11)//温度过低
//						{
//							Cmd_kt_up();
//						}
//						else if (buff[5]==0x12)//温度过高
//						{
//							Cmd_kt_down();
//						}
//						else
//						{
//							Cmd_Icant();
//						}
//					}
//					
				}
				else if (MY_STYLE==MYSTYLE_JH)//控制器类型为净化器时
				{
					if (buff[4]==0x30)//空气纯净度
					{
						if (buff[5]==0x31)//杂质太多
						{
							Cmd_jh_on();
						}
						else
						{
							Cmd_Icant();
						}
					}
					if (buff[4]==0x40)//温度度
					{
						if (buff[5]==0x41)//温度过高
						{
							Cmd_jh_on();
						}
						else
						{
							Cmd_Icant();
						}
					}
					
				}
				//根据控制参数控制
				if (MY_STYLE==MYSTYLE_KT)//控制器类型为空调时
				{
					if (buff[8]==0)
					{
						Cmd_kt_off();
					}
					else if (buff[8]==1)
					{
						if (buff[8+1]==1)
						{
							Cmd_kt_up();						//空调升温降温
						}
						else if(buff[8+1]==2)
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
						Cmd_Icant();
					}
				}
				else if (MY_STYLE==MYSTYLE_CS)//控制器类型为除湿机时
				{
					if (buff[8]==0)
					{
						Cmd_cs_off();
					}
					else if (buff[8]==1)
					{
						Cmd_cs_on();
					}
					
				}
				else if (MY_STYLE==MYSTYLE_JH)//控制器类型为净化机时
				{
					if (buff[8]==0)
					{
						Cmd_jh_off();
					}
					else if (buff[8]==1)
					{
						Cmd_jh_on();
					}
					
				}
					
			}
			else
			{
				Cmd_Icant();
			}
			
		
		}
	}
	
}



static u8 ERR_CONT=0;//环境值超标次数
static u8 ERR_TYPE=0;//环境值超标类型，只有空调有，1，高温，2，低温


void Get_cmd_MODE2 (u8 *buff1)
{
	static u8 connect=0;//0没有通讯，1，通信正在进行
	static u8 big_cmd=0;//广播命令
	u16 cmd_length=0;//命令总长度
	u8 *cmd=buff1;
	u8 crc[2];//crc校验
	u8 if_cmd=0;//crc校验成功，命令有效
	u8 cmd_cfg=0;//是不是配置命令
	u16 temp=0;//临时变量
	
	
	connect=1;//不管连没连，都可以控制
	
	
	
	
	
						//处理无线模块来的数据
	if (buff1[0]==0xff&&buff1[1]==0xff)
	{
			cmd_length=((buff1[5]<<8)|buff1[6])+9;
	}
	//配置命令是和以前一样的
	else if (buff1[0]==0xf0&&buff1[1]==0xfe)
	{
		cmd_length=((buff1[6]<<8)|buff1[7])+10;

		if (buff1[4]==0x10)
		{
			if (MY_MODEL==0x01)
			{
				if ((buff1[3]==0x06)||(buff1[3]==0x07))
					cmd_cfg=1;
			}
			else
			{
				Cmd_err_MODE2 (0x06,ERR_CANNOTCFG);
				return;
			}
		}
		
	}
	else
	{
//		Cmd_err_MODE2 (0x06,ERR_FAIL);
		return ;//数据头不对，退出
	}
	//crc校验
	Get_Crc16(buff1,cmd_length-2,crc);
	//如果校验成功，设置命令有效标志位
	if ((crc[0]==buff1[cmd_length-2])&&(crc[1]==buff1[cmd_length-1]))//校验成功，命令有效
	{
		if_cmd=1;
	}
	
	
	
	//解析配置命令
	if (cmd_cfg&&if_cmd)
	{
		if (buff1[3]==0x06)
		{
			temp=(buff1[6]<<8)|buff1[7];//保存配置命令的数据长度
			if (Set_MyStyle(buff1[5])==0xff)//设置类型
				return;
			
										//带了地址配置
			if (temp>=2)
			{
				MY_ADDR=(buff1[8]<<8)|buff1[9];//设置地址
			}
			
										//带了空调类型
			if (temp>=3)
			{
				if (buff1[5]==MYSTYLE_KT) KT_STYLE=buff1[10];//如果被配置为空调，设置空调的类型
			}
			
										//带了命令解析模式
			if (temp>=4)
			{
				WORK_MODE=buff1[11];
			}
			
			if (temp>=6)//配置命令带了采集器地址
			{
				CJ_ADDR=(buff1[12]<<8)|buff1[13];
			}
			
			if (temp>=8) //配置命令带了环境值上限
			{
				if (buff1[5]==MYSTYLE_KT)
				{
					LIMIT_WENDU_H=(buff1[14]<<8)|buff1[15];
				}
				else if (buff1[5]==MYSTYLE_CS)
				{
					LIMIT_SHIDU_H=(buff1[14]<<8)|buff1[15];
				}
				else if (buff1[5]==MYSTYLE_JH)
				{
					LIMIT_TVOC=(buff1[14]<<8)|buff1[15];
				}
			}
			if (temp>=10)		//配置命令带了环境值下限
			{
				if (buff1[5]==MYSTYLE_KT)
				{
					LIMIT_WENDU_L=(buff1[16]<<8)|buff1[17];
				}
				else if (buff1[5]==MYSTYLE_CS)
				{
					LIMIT_SHIDU_L=(buff1[16]<<8)|buff1[17];
				}
				else if (buff1[5]==MYSTYLE_JH)
				{
					LIMIT_PM2_5=(buff1[16]<<8)|buff1[17];
				}
				
			}
			
									
									//保存配置信息
			Save_Config();
			
			
			MY_MODEL=0;			//退出配置模式
			control_run(MODEL,0);
			
												//关掉所有类型指示灯
			control_run(STYLE_KT,0);
			control_run(STYLE_CS,0);
			control_run(STYLE_JH,0);
		}
		else if (buff1[3]==0x07)//学习遥控命令模式
		{
			if (buff1[4]==0x10&&buff1[5]==MY_STYLE)
			{
				if (buff1[8]==0x01)
				{
					control_run(CMD_UP,0);
					control_run(DEVICE_ON,1);
					control_run(CMD_DOWN,0);
				}
				else if (buff1[8]==0x02)
				{
					control_run(CMD_UP,1);
					control_run(DEVICE_ON,0);
					control_run(CMD_DOWN,1);
				}
				else if (buff1[8]==0x03)
				{
					control_run(CMD_UP,1);
					control_run(DEVICE_ON,0);
					control_run(CMD_DOWN,0);
				}
				else if (buff1[8]==0x04)
				{
					control_run(CMD_UP,0);
					control_run(DEVICE_ON,0);
					control_run(CMD_DOWN,1);
				}
//				Remote_send_USER(buff1[8]);
//				while (Remote_GetState());
				KT_USER_CMD=buff1[8];
			}
		}
		return;
	}
	
	
	
	
	
	
	//开始解析命令
	if (if_cmd)
	{
		temp=(buff1[2]<<8)|buff1[3];
			
		if ((temp!=MY_ADDR)&&(temp!=0x0000))
			return;			//如果命令不是给自己发的，退出
		if (temp==0x0000)//广播命令
		{
			if (buff1[31+7]!=MY_STYLE)
				return;
			big_cmd=1;
		}
		else
			big_cmd=0;
		
		if (buff1[4]==0x03)//读取命令，可能是通信开始或结束
		{
			temp=(buff1[5]<<8)|buff1[6];
			if (temp==0x0003)//数据长度始终为3，不对就出错
			{
				if (buff1[9]==0x01)//数据传输开始
				{
					connect=1;
					temp=(buff1[7]<<8)|buff1[8];
																
																//传输开始命令正确，返回自身状态
					Cmd_return_MODE2 (buff1[4],temp);//应答自身的状态
				}
				else if (buff1[9]==0x02)//数据传输结束,传输结束是没有回应的
				{
					
											//传输结束命令正确，结束传输
					connect=0;
				}
				else
				{
					//命令参数出错，既不是传输开始也不是传输结束
					Cmd_err_MODE2 (0x06,ERR_DATAVALE);
					return;
				}
			}
			else
			{
				//数据长度出错，读取命令的长度只能为3
				Cmd_err_MODE2 (0x06,ERR_DATALENGTH);
				return;
			}
		}
		else if (buff1[4]==0x06)
		{
			//控制命令必须在建立了通信关系才有效,广播命令也有效
			if (connect||(big_cmd==1))
			{
				temp=(buff1[5]<<8)|buff1[6];
				if (temp<27+4)
				{
					//控制数据太短
					Cmd_err_MODE2 (0x06,ERR_DATALENGTH);
					return;
				}
				if (buff1[29]==1)//手动模式
				{
					IF_HAND=1;
					if (MY_STYLE==MYSTYLE_KT)
					{
						if (buff1[30]==0)
						{
							Cmd_kt_off();
						}
						else if (buff1[30]==1)
						{
							if (buff1[33]==1)
							{
								Cmd_kt_up();
							}
							else if (buff1[33]==2)
							{
								Cmd_kt_down();
							}
							else if (buff1[33]==0)
							{
								Cmd_kt_on();
							}
							else
							{
								//数据错误
								Cmd_err_MODE2 (0x06,ERR_DATAVALE);
								return;
							}
						}
						else
						{
							//数据错误
							Cmd_err_MODE2 (0x06,ERR_DATAVALE);
							return;
						}
						
					}
					else if (MY_STYLE==MYSTYLE_CS)
					{
						if (buff1[31]==1)
						{
							Cmd_cs_on();
						}
						else if (buff1[31]==0)
						{
							Cmd_cs_off();
						}
						else
						{
							//数据出错
							Cmd_err_MODE2 (0x06,ERR_DATAVALE);
							return;
						}
					}
					else if (MY_STYLE==MYSTYLE_JH)
					{
						if (buff1[32]==1)
						{
							Cmd_jh_on();
						}
						else if (buff1[32]==0)
						{
							Cmd_jh_off();
						}
						else
						{
							//数据出错
							Cmd_err_MODE2 (0x06,ERR_DATAVALE);
							return;
						}
					}
						
				}
				else if (buff1[29]==2)//自动模式
				{
					IF_HAND=2;
					temp=(buff1[27]<<8)|buff1[28];
					if (temp!=CJ_ADDR)
					{
						//采集器地址错误，
						Cmd_err_MODE2 (0x06,ERR_NOCJADDR);
						return;
					}
					if (big_cmd==0)Cmd_Limit_Updata(buff1);//更新环境变量限制

					if (MY_STYLE==MYSTYLE_KT)
					{
						if (ERR_CONT>10)//连续10次超出限制
						{
							if (ERR_TYPE==1)//高温
							{
								Cmd_kt_down();
								WARN_WENDU=0x01;
							}
							else if (ERR_TYPE==2)//地温
							{
								Cmd_kt_up();
								WARN_WENDU=0x02;
							}
							else if (ERR_TYPE==0)//正常
							{
								Cmd_kt_off();
								WARN_WENDU=0;
							}
							
						}
						else
						{
								Cmd_kt_off();
								WARN_WENDU=0;
						}
						if (1)
						{
							if (buff1[7]>LIMIT_WENDU_H)//温度超过上限
							{
								if (ERR_TYPE==1)
								{
									if (ERR_CONT<=10)
										ERR_CONT++;
									
								}
								else
								{
									ERR_TYPE=1;
									ERR_CONT=0;
								}
							}
							else if (buff1[7]<LIMIT_WENDU_L)//温度超过下限
							{
								if (ERR_TYPE==2)
								{
									if (ERR_CONT<=10)
										ERR_CONT++;
									
								}
								else
								{
									ERR_TYPE=2;
									ERR_CONT=0;
								}
							}
							else
							{
								ERR_TYPE=0;
								ERR_CONT=0;
								
							}
						}
						
					}
					
					//除湿机
					else if (MY_STYLE==MYSTYLE_CS)
					{
						if (ERR_CONT>10)
						{
							if (ERR_TYPE==1)//高温
							{
								Cmd_cs_on();
								WARN_SHIDU=1;
							}
							else if (ERR_TYPE==2)//地温
							{
								Cmd_cs_off();//应该加湿
								WARN_SHIDU=2;
							}
							else if (ERR_TYPE==0)//正常
							{
								Cmd_cs_off();
								WARN_SHIDU=0;
							}
							
						}
						else
						{
								Cmd_cs_off();
								WARN_SHIDU=0;
						}
						if (1)
						{
							if (buff1[9]>LIMIT_SHIDU_H)//温度超过上限
							{
								if (ERR_TYPE==1)
								{
									if (ERR_CONT<=10)
										ERR_CONT++;
									
								}
								else
								{
									ERR_TYPE=1;
									ERR_CONT=0;
								}
							}
							else if (buff1[9]<LIMIT_SHIDU_L)//温度超过下限
							{
								if (ERR_TYPE==2)
								{
									if (ERR_CONT<=10)
										ERR_CONT++;
									
								}
								else
								{
									ERR_TYPE=2;
									ERR_CONT=0;
								}
							}
							else
							{
								ERR_TYPE=0;
								ERR_CONT=0;
								
							}
						}
						
						
					}
					else if (MY_STYLE==MYSTYLE_JH)
					{
						if (ERR_CONT>10)
						{
							if (ERR_TYPE==1)//高温
							{
								Cmd_jh_on();
								WARN_TVOC=1;
								WARN_PM2_5=1;
							}
							else if (ERR_TYPE==2)//,不会有这种值
							{
								Cmd_jh_off();//应该加湿
							}
							else if (ERR_TYPE==0)//正常
							{
								Cmd_jh_off();
								WARN_TVOC=0;
								WARN_PM2_5=0;
							}
							
						}
						else
						{
								Cmd_jh_off();
								WARN_TVOC=0;
								WARN_PM2_5=0;
						}							
						
						if (1)
						{
							if (((buff1[11]<<8)|buff1[12])>LIMIT_PM2_5)//pm2.5超过上限
							{
								if (ERR_TYPE==1)
								{
									if (ERR_CONT<=10)
										ERR_CONT++;
									
								}
								else
								{
									ERR_TYPE=1;
									ERR_CONT=0;
								}
							}
							else if (((buff1[14]<<8)|buff1[15])>LIMIT_TVOC)//tvoc超过sh限
							{
								if (ERR_TYPE==1)
								{
									if (ERR_CONT<=10)
										ERR_CONT++;
									
								}
								else
								{
									ERR_TYPE=1;
									ERR_CONT=0;
								}
							}
							else
							{
								ERR_TYPE=0;
								ERR_CONT=0;
								
							}
						}
						
						
					}
					
				}
				else
				{
					//数据错误，既不是手动模式也不是自动模式
					Cmd_err_MODE2 (0x06,ERR_DATAVALE);
					return;
				}
								//返回控制命令正确且已经控制成功,广播命令不返回
				if (big_cmd==0) Cmd_return_MODE2 (0x06,2);

			}
			else
			{
				//是控制命令，但是没有建立连接关系
				Cmd_err_MODE2 (0x06,ERR_NOCONNECT);
				return;
			}
		}
		else
		{
				//操作字出错，既不是控制命令也不是读取命令
			Cmd_err_MODE2 (0x06,ERR_NULLCMD);
		}
	}
	
	
}



			//更新环境变量限制值，1，已更新，0，没更新
			//传进来的参数是整个命令的首地址，
			//确保命令是集中器发来的第二条命令，在不是广播命令
			//并且是自动模式时调用
u8 Cmd_Limit_Updata(u8 * buff)
{
	u16 temp=0;
	u8 ret=0;
	if (MY_STYLE==MYSTYLE_KT)
	{
		if ((buff[10+7])!=0)//温度上限
		{
			LIMIT_WENDU_H=(buff[10+7]);
			ret=1;
		}
		if (buff[12+7]!=0)//温度下限
		{
			LIMIT_WENDU_L=(buff[12+7]);
			ret=1;
		}
	}
	else if (MY_STYLE==MYSTYLE_CS)
	{
		if (buff[27+7]!=0)
		{
			LIMIT_SHIDU_H=buff[27+7];
			ret=1;
		}
		if (buff[29+7]!=0)
		{
			LIMIT_SHIDU_L=buff[29+7];
			ret=1;
		}
	}
	else if (MY_STYLE==MYSTYLE_JH)
	{
		temp=(buff[14+7]<<8)|buff[15+7];//PM2.5
		if (temp!=0)
		{
			LIMIT_PM2_5=temp;
			ret=1;
		}
		temp=(buff[17+7]<<8)|buff[18+7];//TVOC
		if (temp!=0)
		{
			LIMIT_TVOC=temp;
			ret=1;
		}
	}
	return ret;
}






void Cmd_err_MODE2 (u8 cmd,u16 err)
{
	u8 data[20];
	u8 crc[2];
	
	data[0]=0xff;
	data[1]=0xff;
	data[2]=MY_ADDR>>8;
	data[3]=MY_ADDR;
	data[4]=cmd;
	data[5]=0x00;
	data[6]=0x02;
	data[7]=err>>8;		//错误类型
	data[8]=err;
	
	Get_Crc16(data,9,crc);
	data[9]=crc[0];
	data[10]=crc[1];
	AS32_TX_none(data,11);//发送返回数据

}



void Cmd_return_MODE2 (u8 cmd,u16 datalength)
{
	u8 data[40]={0};
	u8 crc[2];
	MY_OFFTIME=0;//自动关闭定时器重置
	if (cmd==0x03)//只有通信开始命令有应答
	{
		if (datalength>30)
		{
			//欲获取的数据长度过长
			Cmd_err_MODE2 (0x03,ERR_WANTLENGTH);
			return;
		}
		
		data[0]=0xff;
		data[1]=0xff;
		data[2]=MY_ADDR>>8;
		data[3]=MY_ADDR;
		data[4]=0x03;
		data[5]=datalength>>8;
		data[6]=datalength;
		data[7]=0x00;		//无错误
		data[8]=0x00;
		
		//以下是数据位
		data[9]=MY_STYLE;
		data[10]=IF_WORK;
		data[11]=IF_HAND;
		data[12]=WORK_MODE;
		data[13]=KT_STATE;
		data[14]=CS_STATE;
		data[15]=JH_STATE;
		data[16]=KT_STATE1;
		
		//报警信息
		data[17]=WARN_WATER;
		data[18]=WARN_WENDU;
		data[19]=WARN_SHIDU;
		data[20]=WARN_TVOC;
		data[21]=WARN_PM2_5;
		
		//所属采集器ID
		data[22]=CJ_ADDR>>8;
		data[23]=CJ_ADDR;
		
		data[24]=KT_STYLE;
		data[25]=CS_IN_CD;//除湿机是否在CD中
		
		if (MY_STYLE==MYSTYLE_KT)
		{
			data[26]=LIMIT_WENDU_H;
			data[27]=LIMIT_WENDU_L;
		}
		else if (MY_STYLE==MYSTYLE_CS)
		{
			data[26]=LIMIT_SHIDU_H;
			data[27]=LIMIT_SHIDU_L;
		}
		else if (MY_STYLE==MYSTYLE_JH)
		{
			data[26]=LIMIT_TVOC;
			data[27]=LIMIT_PM2_5;
		}
		
		Get_Crc16(data,datalength+7,crc);
		data[datalength+7]=crc[0];
		data[datalength+8]=crc[1];
		AS32_TX_none(data,datalength+9);//发送返回数据
		
	}
	else if (cmd==0x06)
	{
		data[0]=0xff;
		data[1]=0xff;
		data[2]=MY_ADDR>>8;
		data[3]=MY_ADDR;
		data[4]=0x06;
		data[5]=0x00;
		data[6]=0x02;
		data[7]=0x00;		//无错误
		data[8]=0x00;
		
		Get_Crc16(data,9,crc);
		data[9]=crc[0];
		data[10]=crc[1];
		AS32_TX_none(data,11);//发送返回数据
	}
	else
	{
		//操作字出错
		Cmd_err_MODE2 (0x03,ERR_NULLCMD);
		return;
		
	}
}









void Cmd_return (u16 num)
{
	u8 data[]={0xf0 ,0xfe ,0xdf ,0x03 ,0x00 ,0x02 ,0x00 ,0x04,0,0,0,0,0,0};
	u8 crc[2];//不另外声明变量会出现硬件错误
#if __FR_DEBUG==1
		printf ("控制器：\r\n\t");
		printf ("收到了命令\r\n");
#endif
	data[5]=MY_STYLE;//控制器类型
	if (MYSTYLE_KT==MY_STYLE)
	{
		data[2]=MY_ADDR;
		data[7]=0x02;
		data[8]=KT_STATE;
		data[9]=KT_STATE1;
		Get_Crc16(data,10,crc);
		data[10]=crc[0];
		data[11]=crc[1];
		AS32_TX_none(data,12);//发送返回数据
		
	}
	else if (MYSTYLE_CS==MY_STYLE)
	{
		data[2]=MY_ADDR;
		data[7]=0x01;
		data[8]=CS_STATE;
		Get_Crc16(data,9,crc);
		data[9]=crc[0];
		data[10]=crc[1];
		AS32_TX_none(data,11);//发送返回数据
		
	}
	else if (MYSTYLE_JH==MY_STYLE)
	{
		data[2]=MY_ADDR;
		data[7]=0x01;
		data[8]=JH_STATE;
		Get_Crc16(data,9,crc);
		data[9]=crc[0];
		data[10]=crc[1];
		AS32_TX_none(data,11);//发送返回数据
		
	}
	else
	{
		Cmd_Icant();
	}
}


//以下是控制命令执行函数

void Cmd_cs_on (void)
{
	MY_OFFTIME=0;
	if (CS_STATE!=0x01)
	{
			control_run (DEVICE_ON,1);
			control_run (CMD_UP,0);
			control_run (CMD_DOWN,0);
			CS_STATE=0x01;
			control_run (POWER_1,1);
			control_run (POWER_2,1);
			
	}

#if __FR_DEBUG==1
	printf ("控制器：\r\n\t");
	printf ("除湿机开已执行\r\n");
#endif
}

void Cmd_cs_off (void)
{
	MY_OFFTIME=0;
	if (CS_STATE!=0x00)
	{
			control_run (DEVICE_ON,0);
			control_run (CMD_UP,0);
			control_run (CMD_DOWN,0);
			CS_STATE=0x00;
			control_run (POWER_1,0);
			control_run (POWER_2,0);
	}
	
	
#if __FR_DEBUG==1
	printf ("控制器：\r\n\t");
	printf ("除湿机关已执行\r\n");
#endif
	
}

void Cmd_err (void)
{
	
	
#if __FR_DEBUG==1
	printf ("控制器：\r\n\t");
	printf ("命令出错！\r\n");
#endif
	
	
}

void Cmd_kt_on (void)
{
	MY_OFFTIME=0;
	//这里插入空调的开机命令

	if (KT_STATE!=0x01)
	{
		control_run (POWER_1,1);
		control_run (POWER_2,1);

		
	while (Remote_GetState ( ));//保证红外发射时空闲的
	if (KT_STYLE==GREE)
		Remote_send ((u8*)GREE_ON,GREE);
	else if (KT_STYLE==MIDEA)
		Remote_send ((u8*)MIDEA_ON,MIDEA);
	else if (KT_STYLE==USER)
		Remote_send_USER(1);
		
		
		control_run (DEVICE_ON,1);
		control_run (CMD_UP,0);
		control_run (CMD_DOWN,0);
		KT_STATE=0x01;
	}
	
#if __FR_DEBUG==1
	printf ("控制器：\r\n\t");
	printf ("空调开已执行\r\n");
#endif
}

void Cmd_kt_off(void)
{
	MY_OFFTIME=0;
	//这里插入空调的关机命令
	if (KT_STATE!=0x00)
	{
		control_run (POWER_1,0);
		control_run (POWER_2,0);

	while (Remote_GetState ( ));
	if (KT_STYLE==GREE)
		Remote_send ((u8*)GREE_OFF,GREE);
	else if (KT_STYLE==MIDEA)
		Remote_send ((u8*)MIDEA_OFF,MIDEA);
	else if (KT_STYLE==USER)
		Remote_send_USER(2);


		
		control_run (DEVICE_ON,0);
		control_run (CMD_UP,0);
		control_run (CMD_DOWN,0);
		KT_STATE=0x00;
	}
	KT_STATE1=0x00;
#if __FR_DEBUG==1
	printf ("控制器：\r\n\t");
	printf ("空调关已执行\r\n");
#endif
}



void Cmd_kt_up (void)
{
	MY_OFFTIME=0;
	//这里插入空调的开机命令
	if (KT_STATE1!=0x01)
	{
		control_run (POWER_1,1);
		control_run (POWER_2,0);

		while (Remote_GetState ( ));
	if (KT_STYLE==GREE)
		Remote_send ((u8*)GREE_UP,GREE);
	else if (KT_STYLE==MIDEA)
		Remote_send ((u8*)MIDEA_UP,MIDEA);
	else if (KT_STYLE==USER)
		Remote_send_USER(3);


		control_run (DEVICE_ON,1);
		control_run (CMD_UP,1);
		control_run (CMD_DOWN,0);
		KT_STATE1=0x01;
	}
	KT_STATE=0x01;
#if __FR_DEBUG==1
	printf ("控制器：\r\n\t");
	printf ("空调升温已执行\r\n");
#endif
}

void Cmd_kt_down (void)
{
	MY_OFFTIME=0;
	//这里插入空调的开机命令
	if (KT_STATE1!=0x02)
	{
		control_run (POWER_1,0);
		control_run (POWER_2,1);

		while (Remote_GetState ( ));
	if (KT_STYLE==GREE)
		Remote_send ((u8*)GREE_DOWN,GREE);
	else if (KT_STYLE==MIDEA)
		Remote_send ((u8*)MIDEA_DOWN,MIDEA);
	else if (KT_STYLE==USER)
		Remote_send_USER(4);
		
		
		
		control_run (DEVICE_ON,1);
		control_run (CMD_UP,0);
		control_run (CMD_DOWN,1);
		KT_STATE1=0x02;
	}
		KT_STATE=0x01;
#if __FR_DEBUG==1
	printf ("控制器：\r\n\t");
	printf ("空调降温已执行\r\n");
#endif
}

void Cmd_Icant (void)
{
#if __FR_DEBUG==1
	printf ("控制器：\r\n\t");
	printf ("不支持的命令\r\n");
#endif
	
}

void Cmd_jh_on (void)//净化器开
{
	MY_OFFTIME=0;
	if (JH_STATE!=0x01)
	{
		control_run (POWER_1,1);
		control_run (POWER_2,1);


		while (Remote_GetState ( ));//保证红外发射时空闲的
		Remote_send_USER(1);


		
		control_run (DEVICE_ON,1);
		control_run (CMD_UP,0);
		control_run (CMD_DOWN,0);
		JH_STATE=0x01;
	}
#if __FR_DEBUG==1
	printf ("控制器：\r\n\t");
	printf ("净化器开已执行\r\n");
#endif

}
void Cmd_jh_off (void)//净化器关
{
	MY_OFFTIME=0;
	if (JH_STATE!=0x00)
	{
		control_run (POWER_1,0);
		control_run (POWER_2,0);

		while (Remote_GetState ( ));//保证红外发射时空闲的
		Remote_send_USER(2);

		
		control_run (DEVICE_ON,0);
		control_run (CMD_UP,0);
		control_run (CMD_DOWN,0);
		JH_STATE=0x00;
	}
#if __FR_DEBUG==1
	printf ("控制器：\r\n\t");
	printf ("净化器关已执行\r\n");
#endif

}



//处理按键消息，进入配置模式，测试控制功能
//配置模式在配置成功过后会自动退出，长按按键也会退出
//在测试功能时，会临时改变系统状态，退出配置模式时恢复
void Cfg_Model (void)
{
	u8 key=0;
	static u8 i=0;//静态循环变量
	key=Get_Key(0);
	if (key==PRESS_SHORT)
	{
		if (MY_MODEL==0)
		{
			MY_MODEL=1;
			control_run(MODEL,1);
		}
		else
		{
			i++;
			switch (i)
			{
													
				case 1:
					MyDevice_On(1,0);
					break;
				case 2:
					MyDevice_On(1,1);
					break;
				case 3:
					MyDevice_On(1,2);
					break;
				case 4:
					MyDevice_On(0,0);
					break;
				default:
					i=0;
					break;
					
			}
		}
	}
	else if (key==PRESS_LONG)
	{
			MY_MODEL=0;								//退出配置模式
			control_run(MODEL,0);
			control_run(STYLE_KT,0);
			control_run(STYLE_CS,0);
			control_run(STYLE_JH,0);
			control_run(DEVICE_ON,0);
			control_run(CMD_UP,0);
			control_run(CMD_DOWN,0);
			MyDevice_On(0,0);
			Config_Init( );						//重新初始化系统
			i=0;
	}
	if (MY_MODEL==1)
	{
		MY_OFFTIME=0;//重置自动关机计数器，防止自动关机
		if (KT_USER_CMD>=1&&KT_USER_CMD<=4)
		{
			Receive_Cmd(ENABLE);
			if (Receive_GetState()) 
			{
				//如果接收了遥控键值，退出学习模式
				Receive_SaveKey(KT_USER_CMD);
				KT_USER_CMD=0;		//退出空调命令学习
				control_run(DEVICE_ON,0);
				control_run(CMD_UP,0);
				control_run(CMD_DOWN,0);
			}
		}
	}
}








void TIM4_TIMEout_init (void )
{
/***********************************************************************************************

															定时器4默认初始化

*****************************************************************************************************/

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = 720-1; //，每10ms计数器加一
	TIM_TimeBaseStructure.TIM_Prescaler =1000; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);  

 
 
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //计数器更新中断

	//在发送红外的时候再打开定时器
	TIM_Cmd(TIM4,ENABLE ); 	//失能定时器1
	
	
	
}











//按键

#define KEY PBin(2)
static u8 KEYS;


//返回按键状态
u8 Get_Key(u8 keynum)
{
	u8 key_ret;
	key_ret=KEYS;
	KEYS=PRESS_NONE;
	return key_ret;
}



void KEY_IRQHandler(void)   //按键检测中断
{
	static u8 key_time;
	static u8 key_valid;
//KEY1																																				
	if (KEY==0)
	{
		if (key_valid==0)//按键有效
		{
			key_time++;
			if (key_time>=80)
			{
				KEYS=PRESS_LONG;
				key_valid=1;//触发长按，按键无效
			}
		}
	}
	else
	{
	if (key_time>1&&key_time<80)
		{
			KEYS=PRESS_SHORT;
		}
		key_time=0;
		key_valid=0;//按键设为有效
	}
}


//除湿机不可操作计时器,每10ms调用一次
void CS_BUSY_IRQ (void)
{
	if (CS_CANTCMD)//
	{
		if (CS_BUSY_TIME) 
		{
			CS_BUSY_TIME--;
			CS_IN_CD=1;
		}
		else
		{
			CS_CANTCMD=0;//冷却已完毕
			CS_IN_CD=0;
			control_run (POWER_1,CS_LAST_CMD);//执行最后一次命令
			control_run (POWER_2,CS_LAST_CMD);

		}
	}
}



void Time_Out_IRQ (void)
{
	static u16 ms10=0;
		ms10++;
		if (ms10>=100)
		{
			ms10=0;
			MY_OFFTIME++;//计数器每秒加一
			if (MY_OFFTIME>30)
			{
				
				Cmd_cs_off();
				Cmd_kt_off();
				Cmd_jh_off();
			}
		}
}





//每10ms进一次中断
void TIM4_IRQHandler ()
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update)!=RESET)
	{
		Time_Out_IRQ();//超时自动关设备中断
		Run_Time_IRQ ();//系统运行计时中断
//		CS_BUSY_IRQ();	//除湿机CD计时中断
		KEY_IRQHandler();	//按键中断
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	}
}


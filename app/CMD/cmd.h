/*****************************************************************************


			控制器对集中器发来的命令的解析逻辑
			作者：冉川
			完成时间：2018.3.15


*******************************************************************************/

#ifndef  __CMD_H
#define  __CMD_H
#include "usart.h"
#include "delay.h"
#include "control.h"
#include "remote.h"
#include "crc8_16.h"

//按键长按短按区分
#define PRESS_SHORT 1
#define PRESS_LONG 2
#define PRESS_NONE 0



//模式2中的错误类型定义

#define ERR_SUCCESS         0x0000			//成功
#define ERR_FAIL						0x0001			//未知错误
#define ERR_DATAVALE				0x0002			//数据错误
#define ERR_DATALENGTH			0x0003			//数据长度错误
#define ERR_WANTLENGTH			0x0004			//想要读取的数据长度错误
#define ERR_NULLCMD					0x0005			//不支持的命令类型，
#define ERR_NOCONNECT				0x0006			//没有连接，一切操作都要在通信开始之后
#define ERR_NOCJADDR				0x0007			//采集器的地址不是本机绑定的采集器地址
#define ERR_CANNOTCFG				0x0008			//在不是可配置状态下收到了配置命令
#define ERR_NOTMYTYPE				0x0009			//所要控制的设备与本机不符
#define ERR_TIMEOUT					0x000a   //超时














/*在cmd.c里定义的函数*/

extern void (*Get_cmd) (u8 *buff1);
u8 Get_Key(u8 keynum);

							//模式1的命令解析
void Get_cmd_MODE1 (u8 *buff1);

							//模式2的命令解析
void Get_cmd_MODE2 (u8 *buff1);

					//更新环境变量限制值
u8 Cmd_Limit_Updata(u8 *);

void CS_BUSY_IRQ (void);

void TIM4_TIMEout_init (void );
void Cfg_Model (void);



void Cmd_kt_on(void);//空调开
void Cmd_kt_off(void);//空调关
void Cmd_cs_on(void);//除湿机开
void Cmd_cs_off(void);//除湿机关
void Cmd_kt_up (void);//空调升温
void Cmd_kt_down (void);//空调降温
void Cmd_jh_on (void);//净化器开
void Cmd_jh_off (void);//净化器关






/*在cmd1.c里定义的函数*/
				//支持新的多条协议

void Get_cmd_NEW_MODE (u8 *buff1);
void Run_Time_IRQ (void);

void Return_NEW (u8,u16 );
					//控制我的设备
void MyDevice_On (u8 state,u8 updown);



void cmd_0x01 (u8 *buff);
void cmd_0x02 (u8 *buff);
void cmd_0x03 (u8 *buff);
void cmd_0x04 (u8 *buff);
void cmd_0x05 (u8 *buff);
void cmd_0x06 (u8 *buff);
void cmd_0x07 (u8 *buff);
void cmd_0x08 (u8 *buff);
void cmd_0x09 (u8 *buff);





#endif


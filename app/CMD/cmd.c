#include "cmd.h"
#include "config.h"
#include "drv_as62.h"
#include "uart_485.h"
#include "receive.h"


				//ģʽ1�õ���Э��
void Cmd_return(u16 num);//��������
void Cmd_err (void);//����
void Cmd_Icant (void); //��֧�ֵ�����




					//ģʽ2�����ݷ��غ�����cmdΪ0x03��0x06
void Cmd_return_MODE2 (u8 cmd,u16 datalength);
void Cmd_err_MODE2 (u8 cmd,u16 err);				//ģʽ2��ͨ�Ŵ��󷵻�



							//�������ʱ�Զ��رռ�ʱ������λ����
 u16 MY_OFFTIME=0;

										//��ʪ������ֹͣ���ɲ�����־
										//����5���Ӳ��ɹرգ��ر�5���Ӳ��ɴ�
u16 CS_CANTCMD=0;
static u16 CS_BUSY_TIME=5*60*100;//5����CD���ݼ�����
static u8  CS_LAST_CMD=0;//��ʪ����CD������ִ�е�����
#define  CS_OUTTIME 5*60*100


//��ʪ�����յ��������£�����������


		//�����յ��縨��������30��
const u8 GREE_UP[10]={0x30,0x70,0x00,0x0a,0x40,0x00,0x40,0x00,0x60};
		//�����յ�����ģʽ������16��
const u8 GREE_DOWN[10]={0x90,0x00,0x00,0x0a,0x40,0x00,0x40,0x00,0xa0};
		//�����յ��Զ�ģʽ����
const u8 GREE_ON[10]={0x90,0x90,0x04,0x8a,0x40,0x00,0x40,0x00,0xf0};
		//�����յ��Զ�ģʽ�ػ�
const u8 GREE_OFF[10]={0x00,0x90,0x00,0x0a,0x40,0x00,0x40,0x00,0xa0};



		//���Ŀյ��縨��������30��
const u8 MIDEA_UP[10]={0xb2 ,0x4d ,0xbf ,0x40 ,0xbc ,0x43,0x00,0x00};
		//���Ŀյ�����ģʽ������16��
const u8 MIDEA_DOWN[10]={0xb2 ,0x4d ,0xbf ,0x40 ,0x00 ,0xff,0x00,0x00};
		//���Ŀյ��Զ�ģʽ����
const u8 MIDEA_ON[10]={0xb2, 0x4d ,0x1f ,0xe0 ,0xd8 ,0x27,0x00,0x00};
		//���Ŀյ��Զ�ģʽ�ػ�
const u8 MIDEA_OFF[10]={0xb2 ,0x4d ,0x7b ,0x84 ,0xe0 ,0x1f,0x00,0x00};




/*************************************************************************************

��ȡ�����ʽ��
0xf0,0xfe,addr=MY_ADDR,cmd=0x03,fun_h=0,fun_l=0x02,data0=0,data1=0x04,crc_h,crc_l
����֧�ֵĶ�ȡ����̶�Ϊ10���ֽ�
�еȺŵı�����ֵֻ��Ϊ�Ⱥ��ұߵĹ̶�ֵ
fun_0,fun_1�ǹ��ܱ�ʶ�������ǿ�����
�����⵽��ʽ���Իᷢ��������Ϣ���߲���Ӧ��

��ȡ�������ݸ�ʽ��
0xf0 ,0xfe ,addr ,cmd=0x03 ,msg0=0x00 ,mag1=0x02 ,long_h=0x00 ,long_l=0x04,
MY_STATE[0],MY_STATE[1],MY_STATE[2],MY_STATE[3],crc_h,crc_l
�����ķ������ݹ̶�Ϊ14�ֽڣ�����ֻ���ڶ�ȡ����������ȡ�����ݳ���Ϊ4ʱ�Ż᷵��
�������ص��еȺŵı����̶�Ϊ�Ⱥ��ұߵ�ֵ��������Ϊ���������ı仯���仯
û�еȺŵı���������������Щֵ��
MY_STATE[0]��0����ʪ���أ�		1����ʪ����									
MY_STATE[1]��0���յ�,�أ�			1���յ���										
MY_STATE[2]��0����״̬��			1���յ����£�		2���յ�����	
MY_STATE[3]��0�������������أ�������ʱ��֧�ֿ����������Ŀ��ƣ��̶�Ϊ0��


���������ʽ��
0xf0,0xfe,addr=MY_ADDR,cmd=0x06,msg0,msg1,fun_h=0,fun_l=0x02,data0,data1,crc_h,crc_l
����֧�ֵĿ�������̶�Ϊ12���ֽ�
fun_0,fun_1�ǹ��ܱ�ʶ�������ǿ�����
�еȺŵı����ǹ̶��ģ�����ֻ�ܽ������ָ�ʽ������
����ͨ����msg0��msg1������ֵ���ж������Ƴ�ʪ���Ϳյ�����ͣ
			-------------------------
			|   msg0    |   msg1    |
			-------------------------
			|   0x10    |   0x11    |�¶�̫�ߣ�ͨ���˿ɿ��յ���������
			-------------------------
			|   0x10    |   0x12    |�¶�̫�ͣ�ͨ���˿ɿ��յ���������
			-------------------------
			|   0x20    |   0x21    |ʪ��̫�ߣ�ͨ���˿ɿ���ʪ��
			-------------------------
			|   0x20    |   0x22    |ʪ��̫�ͣ�ͨ���˿ɹس�ʪ��
			-------------------------
			������ֻͨ��msg0��msg1�����Ƴ�ʪ��
			
û�еȺŵı���������������Щֵ��						
			-------------------------							
			|data0			|  data1		|							
			-------------------------							
			|		0				|			0			|�յ���				
			-------------------------							
			|		0				|			1			|�յ���				
			-------------------------							
			|		0				|			2			|�յ���				
			-------------------------							
			|		1				|			0			|�յ���				
			-------------------------							
			|		1				|			1			|�յ���������	
			-------------------------							
			|		1				|			2			|�յ���������	
			-------------------------							
�����⵽��ʽ���Իᷢ��������Ϣ���߲���Ӧ��

*****************************************************************************************/
#define __FR_DEBUG 0



						//�����������ָ��
void (*Get_cmd) (u8 *buff1);




						//ģʽ1���������
void Get_cmd_MODE1 (u8 *buff1)
{
	u16 find_num=0;
	u8 *buff=buff1;
	u8 crc[2];
	u16 byte_long=0;//�������ص��ֽڳ���
	u8 if_cmd=0;//����пɽ������Ĭ��û��
	u8 if_cfg_cmd=0;//�������������
	
	while (*buff)
	{
		
		//Ѱ��֡ͷ
		if ((*buff==0xf0)&&(*(buff+1)==0xfe))
		{
			//�ж��������ͣ�ȡ��crcУ�����ݳ���
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
			//crcУ��
			Get_Crc16(buff,find_num,crc);
			//���У��ɹ�������������Ч��־λ
			if ((crc[0]==buff[find_num])&&(crc[1]==buff[find_num+1]))//У��ɹ���������Ч
			{
				if_cmd=1;
			}
			else
				Cmd_err();
			break;
		}
		buff++;
	}
	
	
	
	//��������Ӵ,��������ֻ��������ģʽ�²Żᱻ��ȡ
	if (if_cfg_cmd==1&&if_cmd==1)
	{
		if (buff[3]==0x06)
		{
			if (Set_MyStyle(buff[5])==0xff)//��������ʧ��
					return;
			MY_ADDR=buff[9];//���õ�ַ
			if (buff[5]==MYSTYLE_KT) KT_STYLE=buff[10];//���������Ϊ�յ������ÿյ�������
			WORK_MODE=buff[11];
			CJ_ADDR=(buff[12]<<8)|buff[13];
			Save_Config();
			MY_MODEL=0;			//�˳�����ģʽ
			control_run(MODEL,0);
												//�ص���������ָʾ��
			control_run(STYLE_KT,0);
			control_run(STYLE_CS,0);
			control_run(STYLE_JH,0);
		}
		return;
	}
	
	
	
	
	
	
	
	
	//������Ч�����������
	if (if_cmd)
	{
		//��ַ���ԣ��˳�
		if (buff[2]!=MY_ADDR)
		{
			return ;
		}
		
		
		//��ȡ״̬����
		if (buff[3]==0x03)//��ȡ״̬
		{
			if (buff[4]==0x00&&buff[5]==MY_STYLE)//��������ǶԿ�����������
			{
				byte_long= (buff[6]<<8)+buff[6+1];
				Cmd_return(byte_long);
			}
			else
			{
				//���ǣ���Ӧ��
			}
		}
		//��������
		else if (buff[3]==0x06)//����
		{
			if (buff[6]==0x00&&buff[7]==MY_STYLE)//��������ǶԿ�����������
			{
				//������Ϣ���е�ֵ���ڳ�ʪ���յ�
				if (MY_STYLE==MYSTYLE_CS)//����������Ϊ��ʪ��ʱ
				{
					if (buff[4]==0x20)//ʪ��
					{
						if (buff[5]==0x21)//ʪ�ȹ���
						{
							Cmd_cs_on();
						}
						else if (buff[5]==0x22)//ʪ�ȹ���
						{
							Cmd_cs_off();
						}
						else
						{
							Cmd_Icant();
						}
//						return ;//��ʪ�������ĺ����
					}
				}
				
				else if (MY_STYLE==MYSTYLE_KT)//����������Ϊ�յ�ʱ
				{
//					if (buff[4]==0x10)//�¶ȶ�
//					{
//						if (buff[5]==0x11)//�¶ȹ���
//						{
//							Cmd_kt_up();
//						}
//						else if (buff[5]==0x12)//�¶ȹ���
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
				else if (MY_STYLE==MYSTYLE_JH)//����������Ϊ������ʱ
				{
					if (buff[4]==0x30)//����������
					{
						if (buff[5]==0x31)//����̫��
						{
							Cmd_jh_on();
						}
						else
						{
							Cmd_Icant();
						}
					}
					if (buff[4]==0x40)//�¶ȶ�
					{
						if (buff[5]==0x41)//�¶ȹ���
						{
							Cmd_jh_on();
						}
						else
						{
							Cmd_Icant();
						}
					}
					
				}
				//���ݿ��Ʋ�������
				if (MY_STYLE==MYSTYLE_KT)//����������Ϊ�յ�ʱ
				{
					if (buff[8]==0)
					{
						Cmd_kt_off();
					}
					else if (buff[8]==1)
					{
						if (buff[8+1]==1)
						{
							Cmd_kt_up();						//�յ����½���
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
				else if (MY_STYLE==MYSTYLE_CS)//����������Ϊ��ʪ��ʱ
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
				else if (MY_STYLE==MYSTYLE_JH)//����������Ϊ������ʱ
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



static u8 ERR_CONT=0;//����ֵ�������
static u8 ERR_TYPE=0;//����ֵ�������ͣ�ֻ�пյ��У�1�����£�2������


void Get_cmd_MODE2 (u8 *buff1)
{
	static u8 connect=0;//0û��ͨѶ��1��ͨ�����ڽ���
	static u8 big_cmd=0;//�㲥����
	u16 cmd_length=0;//�����ܳ���
	u8 *cmd=buff1;
	u8 crc[2];//crcУ��
	u8 if_cmd=0;//crcУ��ɹ���������Ч
	u8 cmd_cfg=0;//�ǲ�����������
	u16 temp=0;//��ʱ����
	
	
	connect=1;//������û���������Կ���
	
	
	
	
	
						//��������ģ����������
	if (buff1[0]==0xff&&buff1[1]==0xff)
	{
			cmd_length=((buff1[5]<<8)|buff1[6])+9;
	}
	//���������Ǻ���ǰһ����
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
		return ;//����ͷ���ԣ��˳�
	}
	//crcУ��
	Get_Crc16(buff1,cmd_length-2,crc);
	//���У��ɹ�������������Ч��־λ
	if ((crc[0]==buff1[cmd_length-2])&&(crc[1]==buff1[cmd_length-1]))//У��ɹ���������Ч
	{
		if_cmd=1;
	}
	
	
	
	//������������
	if (cmd_cfg&&if_cmd)
	{
		if (buff1[3]==0x06)
		{
			temp=(buff1[6]<<8)|buff1[7];//����������������ݳ���
			if (Set_MyStyle(buff1[5])==0xff)//��������
				return;
			
										//���˵�ַ����
			if (temp>=2)
			{
				MY_ADDR=(buff1[8]<<8)|buff1[9];//���õ�ַ
			}
			
										//���˿յ�����
			if (temp>=3)
			{
				if (buff1[5]==MYSTYLE_KT) KT_STYLE=buff1[10];//���������Ϊ�յ������ÿյ�������
			}
			
										//�����������ģʽ
			if (temp>=4)
			{
				WORK_MODE=buff1[11];
			}
			
			if (temp>=6)//����������˲ɼ�����ַ
			{
				CJ_ADDR=(buff1[12]<<8)|buff1[13];
			}
			
			if (temp>=8) //����������˻���ֵ����
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
			if (temp>=10)		//����������˻���ֵ����
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
			
									
									//����������Ϣ
			Save_Config();
			
			
			MY_MODEL=0;			//�˳�����ģʽ
			control_run(MODEL,0);
			
												//�ص���������ָʾ��
			control_run(STYLE_KT,0);
			control_run(STYLE_CS,0);
			control_run(STYLE_JH,0);
		}
		else if (buff1[3]==0x07)//ѧϰң������ģʽ
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
	
	
	
	
	
	
	//��ʼ��������
	if (if_cmd)
	{
		temp=(buff1[2]<<8)|buff1[3];
			
		if ((temp!=MY_ADDR)&&(temp!=0x0000))
			return;			//�������Ǹ��Լ����ģ��˳�
		if (temp==0x0000)//�㲥����
		{
			if (buff1[31+7]!=MY_STYLE)
				return;
			big_cmd=1;
		}
		else
			big_cmd=0;
		
		if (buff1[4]==0x03)//��ȡ���������ͨ�ſ�ʼ�����
		{
			temp=(buff1[5]<<8)|buff1[6];
			if (temp==0x0003)//���ݳ���ʼ��Ϊ3�����Ծͳ���
			{
				if (buff1[9]==0x01)//���ݴ��俪ʼ
				{
					connect=1;
					temp=(buff1[7]<<8)|buff1[8];
																
																//���俪ʼ������ȷ����������״̬
					Cmd_return_MODE2 (buff1[4],temp);//Ӧ�������״̬
				}
				else if (buff1[9]==0x02)//���ݴ������,���������û�л�Ӧ��
				{
					
											//�������������ȷ����������
					connect=0;
				}
				else
				{
					//������������Ȳ��Ǵ��俪ʼҲ���Ǵ������
					Cmd_err_MODE2 (0x06,ERR_DATAVALE);
					return;
				}
			}
			else
			{
				//���ݳ��ȳ�����ȡ����ĳ���ֻ��Ϊ3
				Cmd_err_MODE2 (0x06,ERR_DATALENGTH);
				return;
			}
		}
		else if (buff1[4]==0x06)
		{
			//������������ڽ�����ͨ�Ź�ϵ����Ч,�㲥����Ҳ��Ч
			if (connect||(big_cmd==1))
			{
				temp=(buff1[5]<<8)|buff1[6];
				if (temp<27+4)
				{
					//��������̫��
					Cmd_err_MODE2 (0x06,ERR_DATALENGTH);
					return;
				}
				if (buff1[29]==1)//�ֶ�ģʽ
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
								//���ݴ���
								Cmd_err_MODE2 (0x06,ERR_DATAVALE);
								return;
							}
						}
						else
						{
							//���ݴ���
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
							//���ݳ���
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
							//���ݳ���
							Cmd_err_MODE2 (0x06,ERR_DATAVALE);
							return;
						}
					}
						
				}
				else if (buff1[29]==2)//�Զ�ģʽ
				{
					IF_HAND=2;
					temp=(buff1[27]<<8)|buff1[28];
					if (temp!=CJ_ADDR)
					{
						//�ɼ�����ַ����
						Cmd_err_MODE2 (0x06,ERR_NOCJADDR);
						return;
					}
					if (big_cmd==0)Cmd_Limit_Updata(buff1);//���»�����������

					if (MY_STYLE==MYSTYLE_KT)
					{
						if (ERR_CONT>10)//����10�γ�������
						{
							if (ERR_TYPE==1)//����
							{
								Cmd_kt_down();
								WARN_WENDU=0x01;
							}
							else if (ERR_TYPE==2)//����
							{
								Cmd_kt_up();
								WARN_WENDU=0x02;
							}
							else if (ERR_TYPE==0)//����
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
							if (buff1[7]>LIMIT_WENDU_H)//�¶ȳ�������
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
							else if (buff1[7]<LIMIT_WENDU_L)//�¶ȳ�������
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
					
					//��ʪ��
					else if (MY_STYLE==MYSTYLE_CS)
					{
						if (ERR_CONT>10)
						{
							if (ERR_TYPE==1)//����
							{
								Cmd_cs_on();
								WARN_SHIDU=1;
							}
							else if (ERR_TYPE==2)//����
							{
								Cmd_cs_off();//Ӧ�ü�ʪ
								WARN_SHIDU=2;
							}
							else if (ERR_TYPE==0)//����
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
							if (buff1[9]>LIMIT_SHIDU_H)//�¶ȳ�������
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
							else if (buff1[9]<LIMIT_SHIDU_L)//�¶ȳ�������
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
							if (ERR_TYPE==1)//����
							{
								Cmd_jh_on();
								WARN_TVOC=1;
								WARN_PM2_5=1;
							}
							else if (ERR_TYPE==2)//,����������ֵ
							{
								Cmd_jh_off();//Ӧ�ü�ʪ
							}
							else if (ERR_TYPE==0)//����
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
							if (((buff1[11]<<8)|buff1[12])>LIMIT_PM2_5)//pm2.5��������
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
							else if (((buff1[14]<<8)|buff1[15])>LIMIT_TVOC)//tvoc����sh��
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
					//���ݴ��󣬼Ȳ����ֶ�ģʽҲ�����Զ�ģʽ
					Cmd_err_MODE2 (0x06,ERR_DATAVALE);
					return;
				}
								//���ؿ���������ȷ���Ѿ����Ƴɹ�,�㲥�������
				if (big_cmd==0) Cmd_return_MODE2 (0x06,2);

			}
			else
			{
				//�ǿ����������û�н������ӹ�ϵ
				Cmd_err_MODE2 (0x06,ERR_NOCONNECT);
				return;
			}
		}
		else
		{
				//�����ֳ����Ȳ��ǿ�������Ҳ���Ƕ�ȡ����
			Cmd_err_MODE2 (0x06,ERR_NULLCMD);
		}
	}
	
	
}



			//���»�����������ֵ��1���Ѹ��£�0��û����
			//�������Ĳ���������������׵�ַ��
			//ȷ�������Ǽ����������ĵڶ�������ڲ��ǹ㲥����
			//�������Զ�ģʽʱ����
u8 Cmd_Limit_Updata(u8 * buff)
{
	u16 temp=0;
	u8 ret=0;
	if (MY_STYLE==MYSTYLE_KT)
	{
		if ((buff[10+7])!=0)//�¶�����
		{
			LIMIT_WENDU_H=(buff[10+7]);
			ret=1;
		}
		if (buff[12+7]!=0)//�¶�����
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
	data[7]=err>>8;		//��������
	data[8]=err;
	
	Get_Crc16(data,9,crc);
	data[9]=crc[0];
	data[10]=crc[1];
	AS32_TX_none(data,11);//���ͷ�������

}



void Cmd_return_MODE2 (u8 cmd,u16 datalength)
{
	u8 data[40]={0};
	u8 crc[2];
	MY_OFFTIME=0;//�Զ��رն�ʱ������
	if (cmd==0x03)//ֻ��ͨ�ſ�ʼ������Ӧ��
	{
		if (datalength>30)
		{
			//����ȡ�����ݳ��ȹ���
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
		data[7]=0x00;		//�޴���
		data[8]=0x00;
		
		//����������λ
		data[9]=MY_STYLE;
		data[10]=IF_WORK;
		data[11]=IF_HAND;
		data[12]=WORK_MODE;
		data[13]=KT_STATE;
		data[14]=CS_STATE;
		data[15]=JH_STATE;
		data[16]=KT_STATE1;
		
		//������Ϣ
		data[17]=WARN_WATER;
		data[18]=WARN_WENDU;
		data[19]=WARN_SHIDU;
		data[20]=WARN_TVOC;
		data[21]=WARN_PM2_5;
		
		//�����ɼ���ID
		data[22]=CJ_ADDR>>8;
		data[23]=CJ_ADDR;
		
		data[24]=KT_STYLE;
		data[25]=CS_IN_CD;//��ʪ���Ƿ���CD��
		
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
		AS32_TX_none(data,datalength+9);//���ͷ�������
		
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
		data[7]=0x00;		//�޴���
		data[8]=0x00;
		
		Get_Crc16(data,9,crc);
		data[9]=crc[0];
		data[10]=crc[1];
		AS32_TX_none(data,11);//���ͷ�������
	}
	else
	{
		//�����ֳ���
		Cmd_err_MODE2 (0x03,ERR_NULLCMD);
		return;
		
	}
}









void Cmd_return (u16 num)
{
	u8 data[]={0xf0 ,0xfe ,0xdf ,0x03 ,0x00 ,0x02 ,0x00 ,0x04,0,0,0,0,0,0};
	u8 crc[2];//�������������������Ӳ������
#if __FR_DEBUG==1
		printf ("��������\r\n\t");
		printf ("�յ�������\r\n");
#endif
	data[5]=MY_STYLE;//����������
	if (MYSTYLE_KT==MY_STYLE)
	{
		data[2]=MY_ADDR;
		data[7]=0x02;
		data[8]=KT_STATE;
		data[9]=KT_STATE1;
		Get_Crc16(data,10,crc);
		data[10]=crc[0];
		data[11]=crc[1];
		AS32_TX_none(data,12);//���ͷ�������
		
	}
	else if (MYSTYLE_CS==MY_STYLE)
	{
		data[2]=MY_ADDR;
		data[7]=0x01;
		data[8]=CS_STATE;
		Get_Crc16(data,9,crc);
		data[9]=crc[0];
		data[10]=crc[1];
		AS32_TX_none(data,11);//���ͷ�������
		
	}
	else if (MYSTYLE_JH==MY_STYLE)
	{
		data[2]=MY_ADDR;
		data[7]=0x01;
		data[8]=JH_STATE;
		Get_Crc16(data,9,crc);
		data[9]=crc[0];
		data[10]=crc[1];
		AS32_TX_none(data,11);//���ͷ�������
		
	}
	else
	{
		Cmd_Icant();
	}
}


//�����ǿ�������ִ�к���

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
	printf ("��������\r\n\t");
	printf ("��ʪ������ִ��\r\n");
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
	printf ("��������\r\n\t");
	printf ("��ʪ������ִ��\r\n");
#endif
	
}

void Cmd_err (void)
{
	
	
#if __FR_DEBUG==1
	printf ("��������\r\n\t");
	printf ("�������\r\n");
#endif
	
	
}

void Cmd_kt_on (void)
{
	MY_OFFTIME=0;
	//�������յ��Ŀ�������

	if (KT_STATE!=0x01)
	{
		control_run (POWER_1,1);
		control_run (POWER_2,1);

		
	while (Remote_GetState ( ));//��֤���ⷢ��ʱ���е�
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
	printf ("��������\r\n\t");
	printf ("�յ�����ִ��\r\n");
#endif
}

void Cmd_kt_off(void)
{
	MY_OFFTIME=0;
	//�������յ��Ĺػ�����
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
	printf ("��������\r\n\t");
	printf ("�յ�����ִ��\r\n");
#endif
}



void Cmd_kt_up (void)
{
	MY_OFFTIME=0;
	//�������յ��Ŀ�������
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
	printf ("��������\r\n\t");
	printf ("�յ�������ִ��\r\n");
#endif
}

void Cmd_kt_down (void)
{
	MY_OFFTIME=0;
	//�������յ��Ŀ�������
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
	printf ("��������\r\n\t");
	printf ("�յ�������ִ��\r\n");
#endif
}

void Cmd_Icant (void)
{
#if __FR_DEBUG==1
	printf ("��������\r\n\t");
	printf ("��֧�ֵ�����\r\n");
#endif
	
}

void Cmd_jh_on (void)//��������
{
	MY_OFFTIME=0;
	if (JH_STATE!=0x01)
	{
		control_run (POWER_1,1);
		control_run (POWER_2,1);


		while (Remote_GetState ( ));//��֤���ⷢ��ʱ���е�
		Remote_send_USER(1);


		
		control_run (DEVICE_ON,1);
		control_run (CMD_UP,0);
		control_run (CMD_DOWN,0);
		JH_STATE=0x01;
	}
#if __FR_DEBUG==1
	printf ("��������\r\n\t");
	printf ("����������ִ��\r\n");
#endif

}
void Cmd_jh_off (void)//��������
{
	MY_OFFTIME=0;
	if (JH_STATE!=0x00)
	{
		control_run (POWER_1,0);
		control_run (POWER_2,0);

		while (Remote_GetState ( ));//��֤���ⷢ��ʱ���е�
		Remote_send_USER(2);

		
		control_run (DEVICE_ON,0);
		control_run (CMD_UP,0);
		control_run (CMD_DOWN,0);
		JH_STATE=0x00;
	}
#if __FR_DEBUG==1
	printf ("��������\r\n\t");
	printf ("����������ִ��\r\n");
#endif

}



//��������Ϣ����������ģʽ�����Կ��ƹ���
//����ģʽ�����óɹ�������Զ��˳�����������Ҳ���˳�
//�ڲ��Թ���ʱ������ʱ�ı�ϵͳ״̬���˳�����ģʽʱ�ָ�
void Cfg_Model (void)
{
	u8 key=0;
	static u8 i=0;//��̬ѭ������
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
			MY_MODEL=0;								//�˳�����ģʽ
			control_run(MODEL,0);
			control_run(STYLE_KT,0);
			control_run(STYLE_CS,0);
			control_run(STYLE_JH,0);
			control_run(DEVICE_ON,0);
			control_run(CMD_UP,0);
			control_run(CMD_DOWN,0);
			MyDevice_On(0,0);
			Config_Init( );						//���³�ʼ��ϵͳ
			i=0;
	}
	if (MY_MODEL==1)
	{
		MY_OFFTIME=0;//�����Զ��ػ�����������ֹ�Զ��ػ�
		if (KT_USER_CMD>=1&&KT_USER_CMD<=4)
		{
			Receive_Cmd(ENABLE);
			if (Receive_GetState()) 
			{
				//���������ң�ؼ�ֵ���˳�ѧϰģʽ
				Receive_SaveKey(KT_USER_CMD);
				KT_USER_CMD=0;		//�˳��յ�����ѧϰ
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

															��ʱ��4Ĭ�ϳ�ʼ��

*****************************************************************************************************/

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = 720-1; //��ÿ10ms��������һ
	TIM_TimeBaseStructure.TIM_Prescaler =1000; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);  

 
 
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //�����������ж�

	//�ڷ��ͺ����ʱ���ٴ򿪶�ʱ��
	TIM_Cmd(TIM4,ENABLE ); 	//ʧ�ܶ�ʱ��1
	
	
	
}











//����

#define KEY PBin(2)
static u8 KEYS;


//���ذ���״̬
u8 Get_Key(u8 keynum)
{
	u8 key_ret;
	key_ret=KEYS;
	KEYS=PRESS_NONE;
	return key_ret;
}



void KEY_IRQHandler(void)   //��������ж�
{
	static u8 key_time;
	static u8 key_valid;
//KEY1																																				
	if (KEY==0)
	{
		if (key_valid==0)//������Ч
		{
			key_time++;
			if (key_time>=80)
			{
				KEYS=PRESS_LONG;
				key_valid=1;//����������������Ч
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
		key_valid=0;//������Ϊ��Ч
	}
}


//��ʪ�����ɲ�����ʱ��,ÿ10ms����һ��
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
			CS_CANTCMD=0;//��ȴ�����
			CS_IN_CD=0;
			control_run (POWER_1,CS_LAST_CMD);//ִ�����һ������
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
			MY_OFFTIME++;//������ÿ���һ
			if (MY_OFFTIME>30)
			{
				
				Cmd_cs_off();
				Cmd_kt_off();
				Cmd_jh_off();
			}
		}
}





//ÿ10ms��һ���ж�
void TIM4_IRQHandler ()
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update)!=RESET)
	{
		Time_Out_IRQ();//��ʱ�Զ����豸�ж�
		Run_Time_IRQ ();//ϵͳ���м�ʱ�ж�
//		CS_BUSY_IRQ();	//��ʪ��CD��ʱ�ж�
		KEY_IRQHandler();	//�����ж�
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	}
}


/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include "TSL1401.h"
#include "main.h"
#include "stdio.h"
/*
*********************************************************************************************************
*                                               CONSTANTS
*********************************************************************************************************
*/

#define TSL_SI    PCout(0)   //SI  PC0
#define TSL_CLK   PCout(1)   //CLK PC1
#define	TSL_SI_0()	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET) 
#define	TSL_SI_1()	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET)  
#define	TSL_CLK_0()	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET) 
#define	TSL_CLK_1()	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET)  
/*
*********************************************************************************************************
*                                            	变量定义
*********************************************************************************************************
*/
uint8_t SciBuf[200];  //存储上传到上位机的信息

/*
*********************************************************************************************************
*                                               函数实现
*********************************************************************************************************
*/
#ifdef __GNUC_
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
//重映射printf的功能
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, 0xFFFF);
 
  return ch;
}


/**************************************************************************
Function: Get_Voltage
Input   : none
Output  : none
函数功能：获取ADC的值
入口参数: 无 
返回  值：无
**************************************************************************/	 
//获取ADC的值
uint16_t Get_Adc(uint8_t ch)   
{
	uint16_t result;
	HAL_ADC_Start(&hadc1);								//启动转换
	HAL_ADC_PollForConversion(&hadc1,50);				//等待转化结束
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
	{
		result=HAL_ADC_GetValue(&hadc1);	//返回最近一次ADC1规则组的转换结果
	}
	return result;
}
/**************************************************************************
函数功能：CCD数据采集
入口参数：无
返回  值：无
**************************************************************************/
void RD_TSL(void) 
{

    uint8_t i=0,tslp=0;
    TSL_CLK_1();
	TSL_SI_0();
	Dly_us();

    TSL_CLK_0();
	TSL_SI_0();
	Dly_us();

	TSL_CLK_0();
	TSL_SI_1();
	Dly_us();

	TSL_CLK_1();
	TSL_SI_1();
	Dly_us();

	TSL_CLK_1();
	TSL_SI_0();
	Dly_us();

  for(i=0;i<128;i++)
  { 
    TSL_CLK_0(); 
    Dly_us();  //调节曝光时间
    ADV[tslp]=(Get_Adc(3))>>4;
    ++tslp;
    TSL_CLK_1();
     Dly_us();
  }  
}
/**************************************************************************
函数功能：延时
入口参数：无
返回  值：无
**************************************************************************/
void Dly_us(void)  //线性CCD延时
{
   int ii;    
   for(ii=0;ii<10;ii++);      
}

/**************************************************************************
函数功能：线性CCD取中值
入口参数：无
返回  值：无
**************************************************************************/
void  Find_CCD_Zhongzhi(void)
{ 
	 static uint8_t i,j,Left,Right,Last_CCD_Zhongzhi;
	 static uint16_t value1_max,value1_min;
	
	   value1_max=ADV[0];  //动态阈值算法，读取最大和最小值
     for(i=5;i<123;i++)   //两边各去掉5个点
     {
        if(value1_max<=ADV[i])
        value1_max=ADV[i];
     }
	   value1_min=ADV[0];  //最小值
     for(i=5;i<123;i++) 
     {
        if(value1_min>=ADV[i])
        value1_min=ADV[i];
     }
   CCD_Yuzhi=(value1_max+value1_min)/2;	  //计算出本次中线提取的阈值
	 for(i = 5;i<118; i++)   //寻找左边跳变沿
  {
		if(ADV[i]>CCD_Yuzhi&&ADV[i+1]>CCD_Yuzhi&&ADV[i+2]>CCD_Yuzhi&&ADV[i+3]<CCD_Yuzhi&&ADV[i+4]<CCD_Yuzhi&&ADV[i+5]<CCD_Yuzhi)
		{	
		  Left=i;
		  break;	
		}
  }
	 for(j = 118;j>5; j--)//寻找右边跳变沿
  {
		if(ADV[j]<CCD_Yuzhi&&ADV[j+1]<CCD_Yuzhi&&ADV[j+2]<CCD_Yuzhi&&ADV[j+3]>CCD_Yuzhi&&ADV[j+4]>CCD_Yuzhi&&ADV[j+5]>CCD_Yuzhi)
		{	
		  Right=j;
		  break;	
		}
  }
	CCD_Zhongzhi=(Right+Left)/2;//计算中线位置
	if(myabs(CCD_Zhongzhi-Last_CCD_Zhongzhi)>70)   //计算中线的偏差，如果太大
	CCD_Zhongzhi=Last_CCD_Zhongzhi;    //则取上一次的值
	Last_CCD_Zhongzhi=CCD_Zhongzhi;  //保存上一次的偏差
}	

/**************************************************************************
Function: Absolute value function
Input   : a：Number to be converted
Output  : unsigned int
函数功能：绝对值函数
入口参数：a：需要计算绝对值的数
返回  值：无符号整型
**************************************************************************/	
int myabs(int a)
{ 		   
	int temp;
	if(a<0)  temp=-a;  
	else temp=a;
	return temp;
}
//CCD控制函数
void CCDCONCTROL(void){
    RD_TSL();
    Find_CCD_Zhongzhi();
}

	 /******************************************************************************
***
* FUNCTION NAME: binToHex_low(u8 num) *
* CREATE DATE : 20170707 *
* CREATED BY : XJU *
* FUNCTION : 将二进制低8位转换16进制*
* MODIFY DATE : NONE *
* INPUT : u8 *
* OUTPUT : NONE *
* RETURN : char *
*******************************************************************************
**/ 	 	 

uint8_t binToHex_low(uint8_t num)
 {uint8_t low_four;
	 low_four=num&0x0f;
	 if(low_four==0)
		  return('0');
	 else if(low_four==1)
		  return('1');
	 else if(low_four==2)
		  return('2');
	 else if(low_four==3)
		  return('3');
	 else if(low_four==4)
		  return('4');
	 else if(low_four==5)
		  return('5');
	 else if(low_four==6)
		  return('6');
	 else if(low_four==7)
		  return('7');
	 else if(low_four==8)
		  return('8');
	 else if(low_four==9)
		  return('9');
	 else if(low_four==10)
		  return('A');
	 else if(low_four==11)
		  return('B');
	 else if(low_four==12)
		  return('C');
	 else if(low_four==13)
		  return('D');
	 else if(low_four==14)
		  return('E');
	 else if(low_four==15)
		  return('F');
 }
 
/******************************************************************************
***
* FUNCTION NAME: binToHex_low(u8 num) *
* CREATE DATE : 20170707 *
* CREATED BY : XJU *
* FUNCTION : 将二进制高8位转换16进制*
* MODIFY DATE : NONE *
* INPUT : u8 *
* OUTPUT : NONE *
* RETURN : char *
*******************************************************************************
**/ 						 
 uint8_t binToHex_high(uint8_t num)
 {
		uint8_t high_four;
		high_four=(num>>4)&0x0f;
		if(high_four==0)
			return('0');
				else if(high_four==1)
					return('1');
					else if(high_four==2)
							return('2');
							else if(high_four==3)
								return('3');
								else if(high_four==4)
								return('4');
									else if(high_four==5)
									return('5');
										else if(high_four==6)
											return('6');
											else if(high_four==7)
											return('7');
												else if(high_four==8)
												return('8');
													else if(high_four==9)
														return('9');
														else if(high_four==10)
															return('A');
															else if(high_four==11)
																return('B');
																else if(high_four==12)
																	return('C');
																	else if(high_four==13)
																		return('D');
																		else if(high_four==14)
																			return('E');
																			else if(high_four==15)
																				return('F');
 }

uint8_t head=0x02;
uint8_t last=0xFD;
void send_pc(void){
	RD_TSL();
	 SciBuf[0] = 0; 
	  SciBuf[1] = 132;
    SciBuf[2] = 0; 
    SciBuf[3] = 0;
	  SciBuf[4] = 0;
    SciBuf[5] = 0; 
		for(int i=0;i<128;i++)
		SciBuf[6+i] = ADV[i];
	int i;
		 HAL_UART_Transmit(&huart6,&head,1,0xFFFF);
         HAL_UART_Transmit(&huart6,&last,1,0xFFFF);
		 for(i=2;i<134;i++)
		 { 
				// printf("%c",binToHex_high(SciBuf[i])); //以字符形式发送高4位对应的16进制
				// printf("%c",binToHex_low(SciBuf[i]));  //以字符形式发送低?位对应的16进制
				uint8_t ch=binToHex_high(SciBuf[i]);
				uint8_t cl=binToHex_low(SciBuf[i]);
				HAL_UART_Transmit(&huart6,&ch,1,0xFFFF);
                HAL_UART_Transmit(&huart6,&cl,1,0xFFFF);
		 }
		 HAL_UART_Transmit(&huart6,&last,1,0xFFFF);
         HAL_UART_Transmit(&huart6,&head,1,0xFFFF);
}
void chazhi(void)   // 判断是否停车，停车为1
{ 
	
  static uint16_t valueleft,valueright,chazhi,panduan;
  static uint8_t i,j;
  valueright = 0;
  valueleft = 0;
  panduan = 0;
  
  panduan = 0;
      for(i=5;i<64;i++)   //两边各去掉5个点
     {
        
        valueleft = valueleft + ADV[i];
     }
	valueleft = valueleft/60;
    
     for(j=64;j<123;j++) 
     {
      
        
       valueright = valueright + ADV[i];
     }
       valueright = valueright/60;
     chazhi = valueleft - valueright;
     if(chazhi>15)
     {
       panduan=1;
       
     
     }
     
     
}
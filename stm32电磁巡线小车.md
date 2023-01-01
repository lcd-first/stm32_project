

# stm32 电磁巡线小车

## 一 可实现功能

1. 使用陀螺仪，根据俯仰角变化在下坡后停车 。
2. 通过三路电感，实现小车巡线，可循 s弯 ，d形弯，8字弯，环岛。
3. 可在不同的地方巡线，有学习能力。
4. 红外光电开关判断，实现小车的启停，启动，舵机。

---



## 二 实物图

![自动车](http://m.qpic.cn/psc?/V100uwA64V5n8T/45NBuzDIW489QBoVep5mcXkNavY2QXbROihU4jmh.fEHZ52QF*OpWvCRjTr75hOwc2LlmLnSL0UjUfDJLYxve2Q1fKjg0F89gH*wpAzWzrw!/b&bo=VAbuAgAAAAABF44!&rf=viewer_4)

![实物图](http://m.qpic.cn/psc?/V100uwA64V5n8T/45NBuzDIW489QBoVep5mcXysGmhXmYjo6fSQCVWkjBQXJv9w2tNf.pg46q.Pd3os.8lz2sbyunGstK2GIQZy4i.2Wff6muoLtoRsS6JLNAo!/b&bo=oAU4BAAAAAABF6k!&rf=viewer_4)

---



## 三 所需材料

![账单1](http://m.qpic.cn/psc?/V100uwA64V5n8T/45NBuzDIW489QBoVep5mcavqUlinYcdzbI7d3OQdnaTpglGGYflNUhr2kiKrvf4rU7V2WdwMnNCIp82LDxCxQJ0yyFrJEwMG3sEqdcNvHfw!/b&bo=HwT*AwAAAAADF9U!&rf=viewer_4)

![账单2](http://m.qpic.cn/psc?/V100uwA64V5n8T/45NBuzDIW489QBoVep5mccXBvFb1sDPDQ5*JaxOZKWm5R*ZThVcZj6zSPyKdiDSh1uBFYFitgIl1YC297Z4nvVwAvSXOzRWAYIfZ.YmcUP0!/b&bo=IwSEAwAAAAADF5I!&rf=viewer_4)

> + *此账单与可遥控机械臂小车账单合并*

---



## 四 程序



```c
// mian.c

#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "mpu6050.h"
#include "usmart.h"   
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "stm32f10x.h"
#include "oscilloscope.h"
#include "adc.h"
#include  "motor.h"
#include  "xhg.h"
#include "slope.h"
#include "duoji.h"

float pitch,roll,yaw; 		//欧拉角

int sign1 = 1, sign2 = 1;
int sign3 = 1;

int error=0,error2=0,error3=0;
int flag6=0;
int main(void)
{ 
	   extern unsigned char time;
  	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先
	
     TIM3_PWM_Init();  
     TIM2_PWM_Init();
	   TIM4_PWM_Init();
     ADC_init();     //  adc 采集
     delay_init();
	   TIM4_duoji();
	    key_y();
	   sensor_init();    // 小黄管
		 GPIO_SetBits(GPIOC,GPIO_Pin_0 |  GPIO_Pin_1|  GPIO_Pin_2 |  GPIO_Pin_6|  GPIO_Pin_7|GPIO_Pin_3);	//触发信号

	/*************上下桥模块*************/
	
		while( 1 )
	{ 
		if(GPIO_ReadInputDataBit(GPIOC , GPIO_Pin_8) == 0&&flag6==0)
		{
			MPU_Init();		//初始化MPU6050 
			while( mpu_dmp_init() );
			flag6=1;
		}
		if(GPIO_ReadInputDataBit(GPIOC , GPIO_Pin_9) == 0&&flag6==0)
		{
			flag6=2;
		}
		if(flag6==1)
		{
			if( mpu_dmp_get_data(&pitch,&roll,&yaw) == 0 )
			{ 
				if( pitch < 10 )
				{
					forward(80,80);
				}
				
				else if(pitch>15&&pitch<25)
				{
					 forward(60,60);
				}
				else if(pitch>25)           //          俯仰角  
				{
					forward(20,20);
					delay_ms(3000);
					stop();
					while(GPIO_ReadInputDataBit(GPIOC , GPIO_Pin_1) == 1) ;    // 第一节车厢				sign3 = 0;
					flag6=2;
					break;
				}
				else
				{
					forward(20,20);
				}
			}
		}
		if(flag6==2)
		{
			break;
		}
	}
	
	
	while(1)
{		
	   
	   //小黄管检测 有严格的顺序， 在蓝方 先右后左   
	   //  pc1 第一节
	   //  pc0 第二节
	   //  pc2 第三节
	     int i;
	  	 float left_speed=85;
	     float right_speed=85;
      pid_cul(left_speed,right_speed);   
	
		if( (GPIO_ReadInputDataBit(GPIOC , GPIO_Pin_7) == 0) && ( sign1 == 1) ) //右侧小黄管检测到挡板
		{   
			 sign1 = 0;
			 stop();
			 	for( i = 0; i<250; i++)
				{
	   				 stop();
					   delay_ms(100);
					if( GPIO_ReadInputDataBit(GPIOC , GPIO_Pin_0) == 0 )//     小车后间   第二小黄管检测到物料放入  
					{	
						break;//跳出延时等待，继续循迹
					}
				}					
		}
			else if( (GPIO_ReadInputDataBit(GPIOC , GPIO_Pin_6) == 0) && ( sign2 == 1 )&&(sign1==0) ) //左侧小黄管检测到挡板
			{	
				delay_ms(30);
				sign2=0;
				for( i = 0; i<250; i++)
				{
					stop();
					delay_ms( 100 );
					if( GPIO_ReadInputDataBit(GPIOC , GPIO_Pin_2) == 0)       //  小车中黄管     检测到物料放入
					{
						break;//跳出延时等待，继续循迹
					}
				}
	  	}
		else if( !(GPIO_ReadInputDataBit( GPIOC , GPIO_Pin_3 )) &&sign1==0&&sign2==0)
		{
			   stop();
			   duoji_juqi();
         while(1);
  	}

  }

}
void key_y()
{
	
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC , ENABLE );  //串口时钟使能
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;//上拉输入模式
	GPIO_InitStruct.GPIO_Pin =   GPIO_Pin_8 |  GPIO_Pin_9 ;           //后两个依次为左右检测挡板小黄管.  //前三个为检测物料小黄管.依次检测第一块物料，第二块物料，第三块物料
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOC , &GPIO_InitStruct );
}


```







```c
// motor.c
// pid 

#include "stm32f10x.h"
#include  "motor.h"
#include "delay.h"
#include "adc.h"
#include "mpu6050.h" 
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "duoji.h"


unsigned char time = 0;

 void TIM4_PWM_Init()
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //时钟使能
	
	TIM_TimeBaseStructure.TIM_Period = 9999; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =7199; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //使能指定的TIM4中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器


}
//定时器3中断服务程序
void TIM4_IRQHandler(void)   //TIM4中断
{
		time++;//1秒中断一次
		 TIM_ClearITPendingBit(TIM4,TIM_IT_Update);	

}


void  TIM3_PWM_Init( )
{ 
//TIM3 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数

 GPIO_InitTypeDef GPIO_InitStructure;
 TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
 TIM_OCInitTypeDef  TIM_OCInitStructure;
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //使能定时器4时钟
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟 
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1; //
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO 
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; //
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO 
  //初始化TIM3
 TIM_TimeBaseStructure.TIM_Period = 99; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
 TIM_TimeBaseStructure.TIM_Prescaler =719; //设置用来作为TIMx时钟频率除数的预分频值 
 TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
 TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位 
 //初始化TIM3 Channel/2/3/4 PWM模式  
 
 TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1 
 TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
 TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高

 TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC1
 TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC2
 TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC3
 TIM_OC4Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC4 
 TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR1上的预装载寄存器
 TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
 TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR3上的预装载寄存器
 TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR4上的预装载寄存器 
 TIM_Cmd(TIM3, ENABLE);  //使能TIM3
}


void TIM2_PWM_Init( void )
{  
 GPIO_InitTypeDef GPIO_InitStructure;
 TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
 TIM_OCInitTypeDef  TIM_OCInitStructure;
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //使能定时器4时钟
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟 
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3; //右前7通道2，左前8通道3，右后6通道1，左后9通道4
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO 
   //初始化TIM4
 TIM_TimeBaseStructure.TIM_Period = 99; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
 TIM_TimeBaseStructure.TIM_Prescaler =719; //设置用来作为TIMx时钟频率除数的预分频值 
 TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
 TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位 
 //初始化TIM2 Channel/2/3/4 PWM模式  
 TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
 TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
 TIM_OC1Init(TIM2, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC1
 TIM_OC2Init(TIM2, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM4 OC2
 TIM_OC3Init(TIM2, &TIM_OCInitStructure); 
 TIM_OC4Init(TIM2, &TIM_OCInitStructure);  
 TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能TIM4在CCR1上的预装载寄存器
 TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable); 
 TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);  
 TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable); 
 TIM_Cmd(TIM2, ENABLE); 
 
}

//TIM1中的PB6 PB7  代表左前轮， PB0 PB1  代表左后轮； TIM4中的PB6 B7代表右后轮，B8 B9代表右前轮
void forward(float speed_left ,float speed_right )
{
 TIM_SetCompare1(TIM3,speed_right);    //        PA6  PA7  右前
 TIM_SetCompare2(TIM3,0);   
	
 TIM_SetCompare3(TIM3,speed_right);    //      PB0  PB1     右后
 TIM_SetCompare4(TIM3,0);          
	
 TIM_SetCompare1(TIM2,speed_left);     //    PA0   PA1  左前
 TIM_SetCompare2(TIM2,0);       
	
 TIM_SetCompare3(TIM2,speed_left);     
 TIM_SetCompare4(TIM2,0);             //           PA2   PA3    左后
}

void turnright(float  speed_left, float  speed_right)
{ 
	//  右轮反转， 左轮正转 加速  
 TIM_SetCompare1(TIM3,0);              //      PA6  PA7  右前
 TIM_SetCompare2(TIM3,speed_right);   
	
 TIM_SetCompare3(TIM3,0);              //      PB0  PB1     右后
 TIM_SetCompare4(TIM3,speed_right);          

 TIM_SetCompare1(TIM2,speed_left);     //      PA0   PA1  左前
 TIM_SetCompare2(TIM2,0);       
	
 TIM_SetCompare3(TIM2,speed_left);     
 TIM_SetCompare4(TIM2,0);             //        PA2   PA3    左后
	
}



void turnleft(float  speed_left, float  speed_right)
{ 
	// 左轮反转， 右轮正转 加速 
 TIM_SetCompare1(TIM3,speed_right);              //      PA6  PA7  右前
 TIM_SetCompare2(TIM3,0);   
	
 TIM_SetCompare3(TIM3,speed_right);              //      PB0  PB1     右后
 TIM_SetCompare4(TIM3,0);          

 TIM_SetCompare1(TIM2,0);     //      PA0   PA1  左前
 TIM_SetCompare2(TIM2,speed_left);       
	
 TIM_SetCompare3(TIM2,0);     
 TIM_SetCompare4(TIM2,speed_left);             //        PA2   PA3    左后
	
}

void stop(void)
{
	
 

	 
 TIM_SetCompare1(TIM3,0);              //      PA6  PA7  右前
 TIM_SetCompare2(TIM3,0);   
	
 TIM_SetCompare3(TIM3,0);              //      PB0  PB1     右后
 TIM_SetCompare4(TIM3,0);          

 TIM_SetCompare1(TIM2,0);     //      PA0   PA1  左前
 TIM_SetCompare2(TIM2,0);       
	
 TIM_SetCompare3(TIM2,0);     
 TIM_SetCompare4(TIM2,0);             //        PA2   PA3    左后; 

	 
}


void  pid_cul(float left_speed, float right_speed)
{
	int max = 2000;
	int symbol;
	int change_speed;
	float left,middle,right;
	
	float kp = 0.078,Kp;   //0.0878,     0.178     0.0378      0.0578    0.0478
	float ki = 0.00031,Ki;
	float kd = 0.00853,Kd;
	
  float error;
	static float last_1_error = 0;
	static float last_2_error = 0;

	left = Get_ADC_Average(ADC_Channel_5); //通道一 PA1
	middle = Get_ADC_Average(ADC_Channel_14);	//通道四 PA4
	right = Get_ADC_Average(ADC_Channel_15); //通道三 PA3
	symbol = left - right;
	
	if( symbol < 0 )
	{
		error = -symbol;
	}
	
	else
	{
		error = symbol;
	}
	
	if(middle<750&&left<750)   //  直右转 
	{
	  	turnright(70,80);
		  delay_ms(250);
	}
	 else if(middle<750&&right<750)   //  直左转 
	 {
		turnleft(80,70);
		delay_ms(250);
	} 
	  if(middle>3000&&right>2200&&time<5)    // right left  值不定  
	{ 
	
		forward(80,80);
		delay_ms(510);
		turnleft(80,80);
		delay_ms(200);
		forward(70,80);
		delay_ms(400);
	  TIM_Cmd(TIM4, ENABLE);  //使能TIM4
	}
  else 	if(middle>3000&&right>2000&&time>12)
	{
	   forward(80,80);
		 delay_ms(150);
		 turnleft(70,80);
	   delay_ms(200);
		 forward(70,80);
		 delay_ms(800);
	}
	
	 else  if(  left-right < -300 )      // 小车已往左偏   往右转
	{
		  Kp = kp * ( error - last_1_error );
		  Ki += ki * error;
	  	Kd = kd * ( error - ( 2 * last_1_error ) +  last_2_error );
			if( error > max )//限幅
			{
				error = last_1_error;
			}
			change_speed = (int)( Kp + Ki + Kd );
		forward((left_speed + change_speed) , (right_speed - change_speed) );
	} 
	 else if(left-right > 300  )     //正值向右偏，小车向左转;
	{
		error = symbol;
	
		Kp = kp * ( error - last_1_error );
		Ki += ki * error;
		Kd = kd * ( error - ( 2 * last_1_error ) +  last_2_error );
		
			if( error > max )//限幅
			{
				error = last_1_error;
			}
			
			change_speed = (int)( Kp + Ki + Kd );


		forward((left_speed -change_speed) , (right_speed + change_speed));
	}
	
	
  	else   
    {
		  forward( left_speed ,right_speed );//以初始速度直行
	  } 
	   last_1_error = error;
     last_2_error = last_1_error;
}





```

```c
// adc.c
// 电感采集 滤波算法 


#include "adc.h"
#include "math.h"
#include "delay.h"
#include "stm32f10x.h"

void ADC_init()
{
		ADC_InitTypeDef ADC_InitStruct;																															//定义adc结构体
	  GPIO_InitTypeDef GPIO_InitStruct;																														//定义gpio结构体

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);																														//选择时钟2分频                             
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC, ENABLE);										//使能gpio和adc

		
	  GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AIN;																										//gpio口初始化
		GPIO_InitStruct.GPIO_Pin=GPIO_Pin_5;       //添加口   需要修改的地方
		GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
		GPIO_Init(GPIOA,&GPIO_InitStruct);	
	
	  GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AIN;																										//gpio口初始化
		GPIO_InitStruct.GPIO_Pin=GPIO_Pin_4|GPIO_Pin_5;       //添加口   需要修改的地方
		GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
		GPIO_Init(GPIOC,&GPIO_InitStruct);	
	

	  ADC_DeInit(ADC1);																																						//adc初始化
	
	  ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;     //独立模式
	  ADC_InitStruct.ADC_ContinuousConvMode=DISABLE;      //不开启扫描
		ADC_InitStruct.ADC_DataAlign=ADC_DataAlign_Right;   //数据右对齐
		ADC_InitStruct.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;  //触发软件
		ADC_InitStruct.ADC_NbrOfChannel=3;                  //ADC的通道数目　　　着重注意一下需要修改
		ADC_InitStruct.ADC_ScanConvMode=DISABLE;
	
	  ADC_Init(ADC1,&ADC_InitStruct);
		ADC_Cmd(ADC1,ENABLE);
		
		ADC_ResetCalibration(ADC1);  //重置指定的ADC的校准寄存器
		
		while(ADC_GetResetCalibrationStatus(ADC1));//等待上一步操作完成
		
		ADC_StartCalibration(ADC1);     //开始制定ADC的校准状态
			
		while(ADC_GetCalibrationStatus(ADC1));  //等待上一步操作完成
	 
	 
	 
	 
	  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_239Cycles5 );
    ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 2, ADC_SampleTime_239Cycles5 );
    ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 3, ADC_SampleTime_239Cycles5 );
  


	 
}

//获取ADC通道的数据

u16 Get_ADC(u8 ch)
{
  	ADC_RegularChannelConfig(ADC1,ch,1,ADC_SampleTime_239Cycles5);  //开始采集 采集哪个口
	  ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	  while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));
	  return ADC_GetConversionValue(ADC1);
}

//软件滤波
#define N 12
u16 Get_ADC_Average(u8 ch)
{
  

	
	
	
	    u32 temp_value=0;
		u8 t;
		for(t=0;t<10;t++)
	    {
			  temp_value+=Get_ADC(ch);
			}
			  return temp_value/N;
}






```

```c
// 舵机
#include "duoji.h"
#include "stm32f10x.h"
#include "delay.h"


void TIM4_duoji()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);  //使能TIM8的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能 GPIO 的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	//配置GPIO
	//设置该引脚为复用输出功能,输出 TIM8 CH1 的 PWM 脉冲波形
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//初始化TIM8
	TIM_TimeBaseStructure.TIM_Period = 1999;						//设置自动重装载的值
	TIM_TimeBaseStructure.TIM_Prescaler = 719;					//设置预分频系数
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //设置向上计数（累加）计数模式
	//至此，定时器就能正常工作了，此时的溢出时间也就是周期为：Tout= ((arr+1)*(psc+1))/Tclk 这里的Tclk为72MHz

	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //时钟的分频因子，仅对电路的稳定性有影响
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);			//初始化TIM8

	//初始化 TIM8 Channel2 PWM 模式
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // 选择PWM模式2，
	/*在向上计数时，一旦TIMx_CNT<TIMx_CCR1时通道1为无效电平，否则为
    有效电平；在向下计数时，一旦TIMx_CNT>TIMx_CCR1时通道1为有效电平，
    否则为无效电平。*/
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	  //设置极性，输出有效电平为：高电平
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //输出使能

	TIM_OC4Init(TIM1, &TIM_OCInitStructure);					  //初始化外设 TIM8 OC1


	
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable); //使能预装载寄存器


   GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1,ENABLE);


	TIM_Cmd(TIM1, ENABLE); //使能 TIM8

	TIM_CtrlPWMOutputs(TIM1, ENABLE); //主输出使能，当使用的是通用定时器时，这句不需要
}

//	
void duoji_juqi()
{

     int i;
	int j;
	
	for(j=0;j<10;j++)
	{ 
		for(i = 1500 ; i >0; i--)// 865 900
		{ 
			TIM_SetCompare4( TIM1 , i );
			delay_us( 500 );
	     }
		for(i = 0 ; i <1500; i++)// 865 900
		{  
			TIM_SetCompare4( TIM1 , i );
			 delay_us( 500 );
	    }
		
      }


}

// 萌新驾到
// 此为不完全程序，如需完整程序请私聊，免费

```



## 五 总结

```c
 1.在购买材料时，尽量买材质好一点的，杜邦线的选择很重要，材质差的杜邦线，会影响很多地方，而这些地方，会很麻烦。另外也多买几份，当作备用件，这对项目的进程有很大的帮助。
 2.在书写程序时，要多独立思考，编译器有很多不懂的bug时，用百度查，可以在后台开一个网易有道词典，可以翻译出现bug原因，通常是程序书写不规范导致。
 3.多采集数据，电磁赛道与很多其他的赛道不同，不同赛道，不同楼层的电感值可能会不同，注意采集，直线,直角，d形弯，环岛，s弯 电磁值的变化，总结规律。
 4.注意保护电感，小车在赛道巡线时，可能会与赛道或其他小车产生碰撞，电感位置的变化也会对小车巡线产生影响。
 5.小车在前期接线，插线时，要注意避免线太乱，否则到项目后期，在小车出现硬件上的问题时，会因为线太乱，影响项目进程。
 
     
```

 














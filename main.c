/*
 * sumo2015_as7.c
 *
 * Created: 2015-10-07 오전 10:08:45
 * Author : Administrator
 */ 
#define _IR_MAIN_
#define _SENSOR_DETECT_
#define _IR_DETECT_
#define _NORMAL_

#define F_CPU 				16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

#include "INIT.h"
#include "CLCD.h"
#include "SONAR_I2C.h"
#include "SENSOR_SPI.h"
#include "PSD_DIST.h"

//SENSOR
#define TRIGGER_CNT			12		// 70 / 9.984 = 7

#define CH_MID_PSD			3
#define CH_RIGHT_SONAR 		1
#define CH_LEFT_SONAR		2

#define DETECT_DIR_N		0
#define DETECT_DIR_L		1
#define DETECT_DIR_M		2
#define DETECT_DIR_R		3

//IR
#define CH_POT				0
#define CH_IR_FL			6
#define CH_IR_FR			5
#define CH_IR_BL			7
#define CH_IR_BR			4

#define IR_DETECT_NO		0
#define IR_DETECT_FL		1
#define IR_DETECT_FR		2
#define IR_DETECT_BL		3
#define IR_DETECT_BR		4
#define IR_DETECT_FLFR		5
#define IR_DETECT_BLBR		6
#define IR_DETECT_FLBL		7
#define IR_DETECT_FRBR		8
#define IR_DETECT_FLFRBL	9
#define IR_DETECT_FLFRBR	10
#define IR_DETECT_FRBLBR	11
#define IR_DETECT_FLBLBR	12
#define IR_DETECT_UNKN		13		

//MOTION
#define PI 3.141592
#define WHEEL_DIA 6.3
#define WHEEL_DIS 17
#define PPR 728.0

#define MOTION_STOP			0
#define MOTION_STRAIGHT		1
#define MOTION_REVERSE		2
#define MOTION_ROTATECW		3
#define MOTION_ROTATECCW	4
#define MOTION_CURVE_FR		5
#define MOTION_CURVE_FL		6
#define MOTION_CURVE_BR		7
#define MOTION_CURVE_BL		8
#define MOTION_LEFT			9
#define MOTION_RIGHT		10
#define MOTION_ESCAPE_STR	11
#define MOTION_ESCAPE_REV	12
#define MOTION_ATTACK		13

#define DETECT_ANGLE1		45
#define DETECT_ANGLE2		23
#define DETECT_SPEED		20

#define CURVE_RATIO			0.4

#define ESCAPE_SPEED		25
#define ESCAPE_DIST			45

#define INT_CNT_ESCAPE		40
#define DUTY_ESCAPE			1200

#define ATTACK_SPEED		ref_def*1.5

//MOTOR
#define RIGHT_MOTOR_F() 	PORTD = (PORTD&0xCF) | (1<<PD4)
#define RIGHT_MOTOR_B() 	PORTD = (PORTD&0xCF) | (1<<PD5)
#define LEFT_MOTOR_B()  	PORTD = (PORTD&0x3F) | (1<<PD6)
#define LEFT_MOTOR_F()  	PORTD = (PORTD&0x3F) | (1<<PD7)

#define RIGHT_MOTOR_E_L()	PORTG &= ~(1<<PG3)
#define RIGHT_MOTOR_E_H()	PORTG |= (1<<PG3)
#define LEFT_MOTOR_E_L() 	PORTG &= ~(1<<PG4)
#define LEFT_MOTOR_E_H() 	PORTG |= (1<<PG4)

#define ERR_SAT				500

#define DUTY_MAX			1950

#define REF_STEP			2

//SWITCH
#define SW_1 				(PINA & (1<<PA2))
#define SW_2 				(PINA & (1<<PA3))
#define SW_3 				(PINA & (1<<PA4))
#define SW_4 				(PINA & (1<<PA5))
#define SW_5				(PINA & (1<<PA6))
#define SW_6				(PINA & (1<<PA7))
//////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// Variable //////////////////////////////////////////////////
int i;		// for interrupt loop
int j;		// for main loop

unsigned int cnt_esd;

//SONAR, PSD
volatile unsigned int detect_dist_sonar = 40;
volatile unsigned int detect_dist_psd = 60;			// centimeter
volatile unsigned char dir_detect = 0;

volatile unsigned int sensor_left[10];
volatile unsigned int sensor_right[10];
volatile unsigned int sensor_mid[10];
volatile unsigned int sensor_raw[3];
volatile unsigned int sensor_filter[3][50];
volatile unsigned int sensor_temp[3];

volatile unsigned int sensor_i2c;
unsigned char sensor_detect[3];

unsigned char sensor_idx_min;

unsigned int trigger_cnt = 0;

unsigned int sensor_mcp[2];

//IR
unsigned int pot_data[10];
unsigned int pot_filt;

unsigned int ir_data[4][10];
unsigned int ir_filt[4];

unsigned char ir_flag[4];

unsigned char ir_chk_flag;
unsigned char ir_chk_flag_mem[20];
unsigned char ir_chk_flag_white;

unsigned char ir_black;

unsigned char ir_cnt;

// ENCODER
unsigned long cnt_now1;
unsigned long cnt_now2;
unsigned long cnt_old1;
unsigned long cnt_old2;

// MOTOR
volatile unsigned char motor_onoff = 0;

unsigned int int_cnt;
unsigned char mode_motion;
unsigned char mode_motion_old;
int spd_motion;

unsigned char attack_disable;
unsigned int cnt_attack_disable;

long pulse_str_dst;
long pulse_str_now;

long pulse_rot_dst;
long pulse_rot_now;

// PID
volatile long ref_def;

volatile double duty1;
volatile double duty2;	//OC출력

volatile long ref1;
volatile long spd1;
volatile long spd1_old;
volatile long error1;
volatile long errorsum1;
volatile long error1_old;

volatile long ref2;
volatile long spd2;
volatile long spd2_old;
volatile long error2;
volatile long errorsum2;
volatile long error2_old;

volatile double DC1_P = 25.0;
volatile double DC1_I = 20.0;
volatile double DC1_D = 20.0;

volatile double DC2_P = 25.0;
volatile double DC2_I = 20.0;
volatile double DC2_D = 20.0;

//Bluetooth
char data_rx = 0;

//TCNT
unsigned char tcnt_PID;
unsigned char tcnt_sonar;
unsigned char tcnt;

//SWITCH
unsigned int flagLCD;
//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////// Function ///////////////////////////////////////////////////
unsigned int calculate_pulse_cm(double x);
unsigned int calculate_pulse_theta(double degree);

unsigned int GetMinDataIndex(uint8_t arraySize, volatile uint16_t* arrayData);
double LPF(double alpha, double yPre, double xNow);
unsigned int MAF(uint8_t arraySize, volatile uint16_t* arrayData, uint16_t newData);
unsigned int MDF(uint8_t arraySize, volatile uint16_t* arrayData, uint16_t newData);

unsigned int GetADC(unsigned char channel);

void Get_Sensor(void);
void Check_Sensor(void);

void Get_IR(void);
void Check_IR(void);

void Make(void);
void Select_Motion(unsigned char sel_motion, double value);
void Motion(int spd_def);

long get_speed(unsigned char n);
void Motor_Control(unsigned char motor_onoff);
void Motor_PID(long sp1, long sp2);

void getchars(void);
void uart1_putch(char ch);
void putchars(void);
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int calculate_pulse_cm(double x)   //cm
{
	unsigned int pulse;

	pulse = PPR / PI / WHEEL_DIA * x;

	return pulse;
}

unsigned int calculate_pulse_theta(double degree)
{
	unsigned int pulse;
	double radian;

	radian = (degree * 0.8) * 2.0 * PI / 180.0;
	pulse = PPR * WHEEL_DIS * radian / (2.0 * PI * WHEEL_DIA);

	return pulse;
}

unsigned int GetMinDataIndex(uint8_t arraySize, volatile uint16_t* arrayData)
{
	uint16_t num;
	uint16_t temp;
	
	num = 0;
	temp = arrayData[0];
	for(i = 1; i < arraySize; i++)
	{
		if(temp > arrayData[i])
		{
			num = i;
			temp = arrayData[i];
		}
	}
	
	return num;
}

// Low Pass Filter
double LPF(double alpha, double yPre, double xNow)
{
	return yPre + alpha * (xNow - yPre);
}

// Moving Average Filter
unsigned int MAF(uint8_t arraySize, volatile uint16_t* arrayData, uint16_t newData)
{
	uint32_t avgData = 0;	
 
	// Rearrange Data(d[n] -> d[n+1])
	for(i = arraySize - 1; i > 0; i--)
	{
		arrayData[i] = arrayData[i - 1];
		avgData += arrayData[i];
	}
	
	// insert new data
	arrayData[0] = newData;
	avgData += arrayData[0];
 
	// calculate average
	avgData = avgData / arraySize;
 
	return (uint16_t)avgData;
}

// Median Filter
unsigned int MDF(uint8_t arraySize, volatile uint16_t* arrayData, uint16_t newData)
{
	uint16_t sort[20];
	uint8_t n, m = 0;
	uint16_t temp = 0;
	uint16_t resData = 0;

	// Rearrange Data(d[n] -> d[n+1])
	for(i = arraySize - 1; i > 0; i--)
	{
		arrayData[i] = arrayData[i - 1];
	}
	// insert new data
	arrayData[0] = newData;
	
	for(i = 0; i < arraySize; i++)
	{
		sort[i] = arrayData[i];
	}
	
	for(n = 0; n < arraySize-1; n++) 
	{
		for(m = n+1; m < arraySize; m++) 
		{
			if(sort[n] > sort[m])
			{
				temp = sort[n];
				sort[n] = sort[m];
				sort[m] = temp;
			}
		}
	}
	resData = sort[arraySize / 2];

	return resData;	
}

unsigned int GetADC(unsigned char channel)
{
	unsigned int adc = 0;
	
	ADMUX = 0x00 | channel;				//channel select.
	ADCSRA |= (1<<ADSC);				//adc start.
	while((ADCSRA & 0x10) == 0);
	ADCSRA &= ~0x10;
	adc = ADC;

	return adc;
}

//////////////////////////////////////////////////////////////////////////////////////////
void Get_Sensor(void)
{
	trigger_cnt++;
	if(trigger_cnt == 15)
	{	
		trigger_cnt = 0;
		
		sensor_i2c = GetSonarI2C();
		TriggerSonarI2C();
	}
	
	sensor_raw[0] = GetADC(CH_LEFT_SONAR);
	if(sensor_raw[0] < 47)
	{
		sensor_filter[0][0] = 220;
	}
	else
	{
		sensor_filter[0][0] = pgm_read_byte(&psd[MAF(1, sensor_mid, sensor_raw[0])]);
	}
//	sensor_filter[0][0] = 0.0008929 * sensor_raw[0] * sensor_raw[0] - 0.7847 * sensor_raw[0] + 187.4; 
//	sensor_filter[0][0] = -0.000003933 * sensor_raw[0] * sensor_raw[0] * sensor_raw[0] + 0.004295 * sensor_raw[0] * sensor_raw[0]
//							-1.618 * sensor_raw[0] + 243.2;
	
	sensor_raw[1] = GetADC(CH_MID_PSD);
	if(sensor_raw[1] < 47)
	{
		sensor_filter[1][0] = 220;
	}
	else
	{
		sensor_filter[1][0] = pgm_read_byte(&psd[MAF(1, sensor_mid, sensor_raw[1])]);
	}
//	sensor_filter[1][0] = 0.0008929 * sensor_raw[1] * sensor_raw[1] - 0.7847 * sensor_raw[1] + 187.4; 
//	sensor_filter[1][0] = -0.000003933 * sensor_raw[1] * sensor_raw[1] * sensor_raw[1] + 0.004295 * sensor_raw[1] * sensor_raw[1]
//							-1.618 * sensor_raw[1] + 243.2;
								
	sensor_raw[2] = GetADC(CH_RIGHT_SONAR);
	if(sensor_raw[2] < 47)
	{
		sensor_filter[2][0] = 220;
	}
	else
	{
		sensor_filter[2][0] = pgm_read_byte(&psd[MAF(1, sensor_mid, sensor_raw[2])]);
	}
//	sensor_filter[2][0] = 0.0008929 * sensor_raw[2] * sensor_raw[2] - 0.7847 * sensor_raw[2] + 187.4; 
//	sensor_filter[2][0] = -0.000003933 * sensor_raw[2] * sensor_raw[2] * sensor_raw[2] + 0.004295 * sensor_raw[2] * sensor_raw[2]
//							-1.618 * sensor_raw[2] + 243.2;
}

void Check_Sensor(void)
{
	// dir_detect value => none = 0, left = 1, mid = 2, right = 3
	if(sensor_filter[0][0] < detect_dist_psd)
	{
		sensor_detect[0] = 1;
	}
	else
	{
		sensor_detect[0] = 0;
	}
	
	if(sensor_filter[1][0] < detect_dist_psd)
	{
		sensor_detect[1] = 1;
	}
	else
	{
		sensor_detect[1] = 0;
	}
	
	if(sensor_filter[2][0] < detect_dist_psd)
	{
		sensor_detect[2] = 1;
	}
	else
	{
		sensor_detect[2] = 0;
	}
	
	sensor_temp[0] = sensor_filter[0][0];
	sensor_temp[1] = sensor_filter[1][0];
	sensor_temp[2] = sensor_filter[2][0];
	
	sensor_idx_min = GetMinDataIndex(3, sensor_temp);
	dir_detect = sensor_idx_min + 1;
	
	// 센서 3개 모두 감지영역에 있지 않은 경우
	if(sensor_detect[0] == 0 && sensor_detect[1] == 0 && sensor_detect[2] == 0)
	{
		dir_detect = 0;	
	}
	
	if(data_rx == '4')
	{
		uart1_putch(dir_detect);	
	}
}
//////////////////////////////////////////////////////////////////////////////////////////
void Get_IR(void)
{
//	pot = GetADC(CH_POT);
//	ir_data[0][0] = GetADC(CH_IR_FL);
//	ir_data[1][0] = GetADC(CH_IR_FR);
//	ir_data[2][0] = GetADC(CH_IR_BL);
//	ir_data[3][0] = GetADC(CH_IR_BR);

	pot_filt = MAF(10, pot_data, GetADC(CH_POT));
	ir_filt[0] = MAF(1, ir_data[0], GetADC(CH_IR_FL));
	ir_filt[1] = MAF(1, ir_data[1], GetADC(CH_IR_FR));
	ir_filt[2] = MAF(1, ir_data[2], GetADC(CH_IR_BL));
	ir_filt[3] = MAF(1, ir_data[3], GetADC(CH_IR_BR));

	#ifdef _IR_MAIN_
	if(ir_filt[0] > pot_filt)
	{
		ir_flag[0] = 1;
	}
	else 
	{
		ir_flag[0] = 0;
	}
	
	if(ir_filt[1] > pot_filt)
	{
		ir_flag[1] = 1;
	}
	else 
	{
		ir_flag[1] = 0;
	}

	if(ir_filt[2] > pot_filt)
	{
		ir_flag[2] = 1;
	}
	else 
	{
		ir_flag[2] = 0;
	}

	if(ir_filt[3] > pot_filt)
	{
		ir_flag[3] = 1;
	}
	else 
	{
		ir_flag[3] = 0;
	}
	#else
	if(ir_filt[0] < pot_filt)
	{
		ir_flag[0] = 1;
	}
	else 
	{
		ir_flag[0] = 0;
	}
	
	if(ir_filt[1] < pot_filt)
	{
		ir_flag[1] = 1;
	}
	else 
	{
		ir_flag[1] = 0;
	}

	if(ir_filt[2] < pot_filt)
	{
		ir_flag[2] = 1;
	}
	else 
	{
		ir_flag[2] = 0;
	}

	if(ir_filt[3] < pot_filt)
	{
		ir_flag[3] = 1;
	}
	else 
	{
		ir_flag[3] = 0;
	}
	#endif
}

void Check_IR(void)
{
	if(ir_flag[0] == 1 && ir_flag[1] == 0 && ir_flag[2] == 0 && ir_flag[3] == 0)		// only FL
	{
		ir_chk_flag = IR_DETECT_FL;
	}
	else if(ir_flag[0] == 0 && ir_flag[1] == 1 && ir_flag[2] == 0 && ir_flag[3] == 0)	// only FR
	{
		ir_chk_flag = IR_DETECT_FR;
	}
	else if(ir_flag[0] == 0 && ir_flag[1] == 0 && ir_flag[2] == 1 && ir_flag[3] == 0)	// only BL
	{
		ir_chk_flag = IR_DETECT_BL;
	}
	else if(ir_flag[0] == 0 && ir_flag[1] == 0 && ir_flag[2] == 0 && ir_flag[3] == 1)	// only BR
	{
		ir_chk_flag = IR_DETECT_BR;
	}
	else if(ir_flag[0] == 1 && ir_flag[1] == 1 && ir_flag[2] == 0 && ir_flag[3] == 0)	// FL & FR
	{
		ir_chk_flag = IR_DETECT_FLFR;
	}
	else if(ir_flag[0] == 0 && ir_flag[1] == 0 && ir_flag[2] == 1 && ir_flag[3] == 1)	// BL & BR
	{
		ir_chk_flag = IR_DETECT_BLBR;
	}
	else if(ir_flag[0] == 1 && ir_flag[1] == 0 && ir_flag[2] == 1 && ir_flag[3] == 0)	// FL & BL
	{
		ir_chk_flag = IR_DETECT_FLBL;
	}
	else if(ir_flag[0] == 0 && ir_flag[1] == 1 && ir_flag[2] == 0 && ir_flag[3] == 1)	// FR & BR
	{
		ir_chk_flag = IR_DETECT_FRBR;
	}
	else if(ir_flag[0] == 0 && ir_flag[1] == 0 && ir_flag[2] == 0 && ir_flag[3] == 0)
	{
		ir_chk_flag = IR_DETECT_NO;	
	}
	else if(ir_flag[0] == 1 && ir_flag[1] == 1 && ir_flag[2] == 1 && ir_flag[3] == 0)
	{
		ir_chk_flag = IR_DETECT_FLFRBL;
	}
	else if(ir_flag[0] == 1 && ir_flag[1] == 1 && ir_flag[2] == 0 && ir_flag[3] == 1)
	{
		ir_chk_flag = IR_DETECT_FLFRBR;
	}
	else if(ir_flag[0] == 0 && ir_flag[1] == 1 && ir_flag[2] == 1 && ir_flag[3] == 1)
	{
		ir_chk_flag = IR_DETECT_FRBLBR;
	}
	else if(ir_flag[0] == 1 && ir_flag[1] == 0 && ir_flag[2] == 1 && ir_flag[3] == 1)
	{
		ir_chk_flag = IR_DETECT_FLBLBR;	
	}
	else
	{
		ir_chk_flag = IR_DETECT_UNKN;	
	}
			
	// 배열 재배치(d[n] -> d[n+1])
	for(i = 20 - 1; i > 0; i--)
	{
		ir_chk_flag_mem[i] = ir_chk_flag_mem[i - 1];
	}
	ir_chk_flag_mem[0] = ir_chk_flag;
}
//////////////////////////////////////////////////////////////////////////////////////////
void Make(void)
{
	#ifdef _SENSOR_DETECT_
	// 전방 센서가 하나라도 감지 됬을때
	if(motor_onoff == 0xFF)
	{
		///////////////////////////////////////////////////////////////////////////////////
		if(sensor_detect[0] == 1  && sensor_detect[1] == 0 && sensor_detect[2] == 0 && (attack_disable == 0))
		{
			spd_motion = DETECT_SPEED;
			Select_Motion(MOTION_ROTATECCW, DETECT_ANGLE1);		// motion, degree
		}
		else if(sensor_detect[0] == 0  && sensor_detect[1] == 1 && sensor_detect[2] == 0 && (attack_disable == 0))
		{
			spd_motion = ATTACK_SPEED;
			Select_Motion(MOTION_ATTACK, 10);
		}
		else if(sensor_detect[0] == 0  && sensor_detect[1] == 0 && sensor_detect[2] == 1 && (attack_disable == 0))
		{
			spd_motion = DETECT_SPEED;
			Select_Motion(MOTION_ROTATECW, DETECT_ANGLE1);		// motion, degree
		}
		///////////////////////////////////////////////////////////////////////////////////
		else if(sensor_detect[0] == 1  && sensor_detect[1] == 1 && sensor_detect[2] == 0 && (attack_disable == 0))
		{
///			spd_motion = 15;
//			Select_Motion(MOTION_ROTATECCW, DETECT_ANGLE2);		// motion, degree

			spd_motion = ATTACK_SPEED;
			Select_Motion(MOTION_ATTACK, 50);
		}
		else if(sensor_detect[0] == 0  && sensor_detect[1] == 1 && sensor_detect[2] == 1 && (attack_disable == 0))
		{
//			spd_motion = 15;
//			Select_Motion(MOTION_ROTATECW, DETECT_ANGLE2);		// motion, degree

			spd_motion = ATTACK_SPEED;
			Select_Motion(MOTION_ATTACK, 50);
		}
		else if(sensor_detect[0] == 1 && sensor_detect[1] == 1 && sensor_detect[2] == 1 && (attack_disable == 0))
		{
			spd_motion = ATTACK_SPEED;
			Select_Motion(MOTION_ATTACK, 10);
		}
		///////////////////////////////////////////////////////////////////////////////////
		// 후방 센서 감지
		if(motor_onoff == 0xFF && sensor_i2c < detect_dist_sonar && attack_disable == 0)
		{
			spd_motion = DETECT_SPEED;
			Select_Motion(MOTION_ROTATECW, 180);		// motion, degree	
		}
		
		// Attack
		if(sensor_filter[1][0] < detect_dist_psd && attack_disable == 0)
		{
			spd_motion = ATTACK_SPEED;
			Select_Motion(MOTION_ATTACK, 10);
		}
	
		// Attack Disable
		if(sensor_filter[1][0] < detect_dist_psd)
		{
			if((ir_flag[0] == 1) || (ir_flag[1] == 1))
			{
				cnt_attack_disable = 0;
				attack_disable = 1;
			}
		}
		
		// Attack Disable Count
		if(attack_disable == 1)
		{
			cnt_attack_disable++;
		}
		if(cnt_attack_disable > 400)
		{
			sensor_filter[1][0] = 220;
				
			cnt_attack_disable = 0;
			attack_disable = 0;
		}
	}
	#endif
	#ifdef _NORMAL_
	// 센서 감지x, 라인 감지x 일때 동작
	if(motor_onoff == 0xFF && ir_chk_flag == IR_DETECT_NO && mode_motion == MOTION_STOP)
//	if(motor_onoff == 0xFF && ir_chk_flag == IR_DETECT_NO && mode_motion == MOTION_STOP)
//	if(motor_onoff == 0xFF && mode_motion == MOTION_STOP)
	{
		spd_motion = ref_def;
		if(ir_black == 0)
		{
			ir_black = 1;	
		}
		else if(ir_black == 1)
		{
			switch(ir_chk_flag_white)
			{
				case IR_DETECT_FL:
					Select_Motion(MOTION_ROTATECW, 90.0);		// motion, degree
				break;
				case IR_DETECT_FR:
					Select_Motion(MOTION_ROTATECCW, 90.0);
				break;
				case IR_DETECT_BL:
					Select_Motion(MOTION_ROTATECCW, 90.0);
				break;
				case IR_DETECT_BR:
					Select_Motion(MOTION_ROTATECW, 90.0);
				break;
				case IR_DETECT_FLFR:
					Select_Motion(MOTION_ROTATECW, 120.0);
				break;
				case IR_DETECT_BLBR:
					Select_Motion(MOTION_ROTATECCW, 120.0);
				break;
				case IR_DETECT_FLBL:
					Select_Motion(MOTION_ROTATECW, 90.0);
				break;
				case IR_DETECT_FRBR:
					Select_Motion(MOTION_ROTATECCW, 90.0);
				break;
				default:
					Select_Motion(MOTION_ROTATECCW, 120.0);
				break;
			}
			ir_black = 2;
		}
		else if(ir_black == 2)
		{
//			spd_motion = ref_def;
//			Select_Motion(MOTION_CURVE_FR, 25);		// motion, distance
			Select_Motion(MOTION_STRAIGHT, 50);		
//			Select_Motion(MOTION_STOP, 50);	
			
			ir_black = 3;
		}
		else if(ir_black == 3)
		{
//			spd_motion = ref_def;
//			Select_Motion(MOTION_CURVE_FL, 25);		// motion, distance
			Select_Motion(MOTION_STRAIGHT, 50);
//			Select_Motion(MOTION_STOP, 50);	
			
			ir_black = 2;
		}
	}
	#endif	
	#ifdef _IR_DETECT_
	// 라인 감지 됬을때
	if(motor_onoff == 0xFF && ir_chk_flag != IR_DETECT_NO)
	{
		//if(ir_chk_flag == IR_DETECT_FL && ir_chk_flag_mem[1] == IR_DETECT_NO)
		if(ir_chk_flag == IR_DETECT_FL)
		{
			ir_black = 0;
			ir_chk_flag_white = IR_DETECT_FL;
			
			spd_motion = ESCAPE_SPEED;
			Select_Motion(MOTION_REVERSE, ESCAPE_DIST);
//			Select_Motion(MOTION_CURVE_BR, ESCAPE_DIST);
//			Select_Motion(MOTION_ESCAPE_REV, 0);
		}
		else if(ir_chk_flag == IR_DETECT_FR)
		{
			ir_black = 0;
			ir_chk_flag_white = IR_DETECT_FR;	
			
			spd_motion = ESCAPE_SPEED;
			Select_Motion(MOTION_REVERSE, ESCAPE_DIST);	
//			Select_Motion(MOTION_CURVE_BL, ESCAPE_DIST);
//			Select_Motion(MOTION_ESCAPE_REV, 0);
		}
		else if(ir_chk_flag == IR_DETECT_FLFR)
		{
			ir_black = 0;
			ir_chk_flag_white = IR_DETECT_FLFR;
			
			spd_motion = ESCAPE_SPEED;
			Select_Motion(MOTION_REVERSE, ESCAPE_DIST);
//			Select_Motion(MOTION_ESCAPE_REV, 0);	
		}
		else if(ir_chk_flag == IR_DETECT_BL)
		{
			ir_black = 0;
			ir_chk_flag_white = IR_DETECT_BL;	
			
			spd_motion = ESCAPE_SPEED;
			Select_Motion(MOTION_STRAIGHT, ESCAPE_DIST);
//			Select_Motion(MOTION_CURVE_FR, ESCAPE_DIST);	
//			Select_Motion(MOTION_ESCAPE_STR, 0);	
		}
		else if(ir_chk_flag == IR_DETECT_BR )
		{
			ir_black = 0;
			ir_chk_flag_white = IR_DETECT_BR;	
		
			spd_motion = ESCAPE_SPEED;
			Select_Motion(MOTION_STRAIGHT, ESCAPE_DIST);	
//			Select_Motion(MOTION_CURVE_FL, ESCAPE_DIST);
//			Select_Motion(MOTION_ESCAPE_STR, 0);
		}
		else if(ir_chk_flag == IR_DETECT_BLBR)
		{
			ir_black = 0;
			ir_chk_flag_white = IR_DETECT_BLBR;	
		
			spd_motion = ESCAPE_SPEED;
			Select_Motion(MOTION_STRAIGHT, ESCAPE_DIST);
//			Select_Motion(MOTION_ESCAPE_STR, 0);	
		}
		/////////////////////////////////////////////////////////
		else if(ir_chk_flag == IR_DETECT_FLBL)
		{
			ir_black = 0;
			ir_chk_flag_white = IR_DETECT_FLBL;
			
			spd_motion = ref_def;
//			Select_Motion(MOTION_CURVE_FR, ESCAPE_DIST);
			Select_Motion(MOTION_ROTATECW, 90);
		}
		else if(ir_chk_flag == IR_DETECT_FRBR)
		{
			ir_black = 0;
			ir_chk_flag_white = IR_DETECT_FRBR;
			
			spd_motion = ref_def;
//			Select_Motion(MOTION_CURVE_FL, ESCAPE_DIST);
			Select_Motion(MOTION_ROTATECCW, 90);
		}
		/////////////////////////////////////////////////////////
		else if(ir_chk_flag == IR_DETECT_FLFRBL)
		{
			ir_black = 0;
			ir_chk_flag_white = IR_DETECT_FLFRBL;
			
			spd_motion = ref_def;
			//Select_Motion(MOTION_CURVE_BR, ESCAPE_DIST);
			Select_Motion(MOTION_ROTATECW, 135);			
		}
		else if(ir_chk_flag == IR_DETECT_FLFRBR)
		{
			ir_black = 0;
			ir_chk_flag_white = IR_DETECT_FLFRBR;
			
			spd_motion = ref_def;
			//Select_Motion(MOTION_CURVE_BL, ESCAPE_DIST);
			Select_Motion(MOTION_ROTATECCW, 135);
		}
		else if(ir_chk_flag == IR_DETECT_FRBLBR)
		{
			ir_black = 0;
			ir_chk_flag_white = IR_DETECT_FRBLBR;
			
			spd_motion = ref_def;
			//Select_Motion(MOTION_CURVE_FL, ESCAPE_DIST);
			Select_Motion(MOTION_ROTATECCW, 45);
		}
		else if(ir_chk_flag == IR_DETECT_FLBLBR)
		{
			ir_black = 0;
			ir_chk_flag_white = IR_DETECT_FLBLBR;
			
			spd_motion = ref_def;
			//Select_Motion(MOTION_CURVE_FR, ESCAPE_DIST);		
			Select_Motion(MOTION_ROTATECW, 45);	
		}
		else
		{
			//for(i = 0; i < 20; i++)
			//{
			//	if(ir_chk_flag_mem[i] != IR_DETECT_UNKN)
			//	{
			//		ir_chk_flag = ir_chk_flag_mem[i];
			//	}
			//}
		}
	}
	#endif
}

// value is centimeter or degree
void Select_Motion(unsigned char sel_motion, double value)
{
	if((sel_motion == MOTION_STRAIGHT) || (sel_motion == MOTION_REVERSE) || 
		(sel_motion == MOTION_CURVE_FR) || (sel_motion == MOTION_CURVE_FL) || 
		(sel_motion == MOTION_CURVE_BR) || (sel_motion == MOTION_CURVE_BL))
	{
		pulse_str_dst = calculate_pulse_cm(value);
		mode_motion = sel_motion;
	}
	else if((sel_motion == MOTION_ROTATECW) || (sel_motion == MOTION_ROTATECCW))	// Roatate Motion
	{
		pulse_rot_dst = calculate_pulse_theta(value);
		mode_motion = sel_motion;	
	}
	else if((sel_motion == MOTION_ESCAPE_STR) || (sel_motion == MOTION_ESCAPE_REV))
	{
		int_cnt = 0;
		mode_motion = sel_motion;
	}
	else
	{
		mode_motion = sel_motion;
	}
}

void Motion(int spd_def)
{
	if(mode_motion == MOTION_STOP)
	{
		pulse_str_now = 0;
		pulse_rot_now = 0;
		
		ref1 = 0;
		ref2 = 0;
	}
	else if(mode_motion == MOTION_STRAIGHT)
	{
		if(pulse_str_dst >= pulse_str_now)
		{
			pulse_str_now += +(spd1 + spd2)/2;
			
			ref1 = +spd_def;
			ref2 = +spd_def;
		}
		else
		{
			mode_motion = MOTION_STOP;
			pulse_str_now = 0;
		}
	}
	else if(mode_motion == MOTION_REVERSE)	
	{
		if(pulse_str_dst >= pulse_str_now)
		{
			pulse_str_now += -(spd1 + spd2)/2;
			
			ref1 = -spd_def;
			ref2 = -spd_def;
		}
		else
		{
			mode_motion = MOTION_STOP;
			pulse_str_now = 0;
		}		
	}
	else if(mode_motion == MOTION_CURVE_FR)	
	{
		if(pulse_str_dst >= pulse_str_now)
		{
			pulse_str_now += spd1;
			
			ref1 = +spd_def;
			ref2 = +CURVE_RATIO * spd_def;
		}
		else 
		{
			mode_motion = MOTION_STOP;
			pulse_str_now = 0;
		}
	}
	else if(mode_motion == MOTION_CURVE_FL)	
	{
		if(pulse_str_dst >= pulse_str_now)
		{
			pulse_str_now += spd2;
			
			ref1 = +CURVE_RATIO * spd_def;
			ref2 = +spd_def;
		}
		else 
		{
			mode_motion = MOTION_STOP;
			pulse_str_now = 0;
		}
	}
	else if(mode_motion == MOTION_CURVE_BR)	
	{
		if(pulse_str_dst >= pulse_str_now)
		{
			pulse_str_now += -spd1;
			
			ref1 = -spd_def;
			ref2 = -CURVE_RATIO * spd_def;
		}
		else 
		{
			mode_motion = MOTION_STOP;
			pulse_str_now = 0;
		}
	}
	else if(mode_motion == MOTION_CURVE_BL)	
	{
		if(pulse_str_dst >= pulse_str_now)
		{
			pulse_str_now += (-spd2);
						
			ref1 = -CURVE_RATIO * spd_def;
			ref2 = -spd_def;
		}
		else 
		{
			mode_motion = MOTION_STOP;
			pulse_str_now = 0;
		}
	}
	else if(mode_motion == MOTION_LEFT)
	{
		if(pulse_str_dst >= pulse_str_now)
		{
			pulse_str_now += (spd1);
						
			ref1 = spd_def;
			ref2 = 0;
		}
		else 
		{
			mode_motion = MOTION_STOP;
			pulse_str_now = 0;
		}			
	}
	else if(mode_motion == MOTION_RIGHT)
	{
		if(pulse_str_dst >= pulse_str_now)
		{
			pulse_str_now += (spd2);
						
			ref1 = 0;
			ref2 = spd_def;
		}
		else 
		{
			mode_motion = MOTION_STOP;
			pulse_str_now = 0;
		}		
	}
	else if(mode_motion == MOTION_ROTATECW)	
	{
		if(pulse_rot_dst >= pulse_rot_now)
		{
			pulse_rot_now += (spd1 - spd2)/2;
			
			ref1 = +spd_def;
			ref2 = -spd_def;
		}
		else 
		{
			mode_motion = MOTION_STOP;
			pulse_rot_now = 0;
		}
	}
	else if(mode_motion == MOTION_ROTATECCW)	
	{	
		if(pulse_rot_dst >= pulse_rot_now)
		{
			pulse_rot_now += (-spd1 + spd2)/2 ;
			
			ref1 = -spd_def;
			ref2 = +spd_def;
		}
		else 
		{
			mode_motion = MOTION_STOP;
			pulse_rot_now = 0;
		}
	}
	else if(mode_motion == MOTION_ESCAPE_STR)
	{
		if(int_cnt < 160)
		{
			ref1 = ESCAPE_SPEED;
			ref2 = ESCAPE_SPEED;
		}
		else if(int_cnt >= 160) 
		{
			ref1 = 0;
			ref2 = 0;
			if(int_cnt >= 200)
			{
				int_cnt = 0;
			}
		}
		int_cnt++;
		
		
		/*
		if(int_cnt >= INT_CNT_ESCAPE)
		{
			mode_motion = MOTION_STOP;
		}
		else
		{
			int_cnt++;
		}
		*/
	}
	else if(mode_motion == MOTION_ESCAPE_REV)
	{
		if(int_cnt < 160)
		{
			ref1 = -ESCAPE_SPEED;
			ref2 = -ESCAPE_SPEED;
		}
		else if(int_cnt >= 160) 
		{
			ref1 = 0;
			ref2 = 0;
			if(int_cnt >= 200)
			{
				int_cnt = 0;
			}
		}
		int_cnt++;
		/*
		if(int_cnt >= INT_CNT_ESCAPE)
		{
			int_cnt = 0;
			mode_motion = MOTION_STOP;
		}
		else
		{
			int_cnt++;
		}
		*/
	}
	else if(mode_motion == MOTION_ATTACK)
	{
//		ref1 = 0;
//		ref2 = 0;
		ref1 = ATTACK_SPEED;
		ref2 = ATTACK_SPEED;		
		/*
		if(int_cnt == ATTACK_SPEED)
		{
			ref1 = 0;
			ref2 = 0;	
		}
		else if(int_cnt == ATTACK_SPEED + 5)
		{
			int_cnt = 0;
		}
		else if(ref1 < ATTACK_SPEED && ref2 < ATTACK_SPEED)
		{
			ref1 += 2;
			ref2 += 2;				
		}
		*/
		/*
		if(int_cnt < 180)
		{
			ref1 = ATTACK_SPEED;
			ref2 = ATTACK_SPEED;
		}
		else if(int_cnt >= 180) 
		{
			ref1 = 0;
			ref2 = 0;
			if(int_cnt >= 200)
			{
				int_cnt = 0;
			}
		}
		int_cnt++;
		*/
	}
}
//////////////////////////////////////////////////////////////////////////////////////////
long get_speed(unsigned char n)
{
	long sp = 0;
	
	if(n==1)
	{
		cnt_old1 = cnt_now1;
		cnt_now1 = LS7366_LEFT_GetCount();
		sp = cnt_now1 - cnt_old1;
	}
	else if(n==2)
	{
		cnt_old2 = cnt_now2;
		cnt_now2 = LS7366_RIGHT_GetCount();
		sp = cnt_now2 - cnt_old2;
	}
	
	return sp;
}

void Motor_Control(unsigned char motor_onoff)
{
	if(motor_onoff == 0x00)
	{
		RIGHT_MOTOR_E_L();
		LEFT_MOTOR_E_L();

		errorsum1 = 0;
		errorsum2 = 0;
	}
	else if(motor_onoff == 0xFF)
	{	
		RIGHT_MOTOR_E_H();
		LEFT_MOTOR_E_H();

		Motor_PID(spd1, spd2);
		/*
		// Feedback Control
		if((mode_motion != MOTION_ESCAPE_STR) && (mode_motion != MOTION_ESCAPE_REV))
		{
			Motor_PID(spd1, spd2);
		}
		// Open Loop Duty Control
		else if(mode_motion == MOTION_ESCAPE_REV)
		{
			duty1 = DUTY_ESCAPE;
			duty2 = DUTY_ESCAPE;
			
			LEFT_MOTOR_B();
			RIGHT_MOTOR_B();
			
			OCR1B = (unsigned int)(duty1);
			OCR1C = (unsigned int)(duty2);
		}
		// Open Loop Duty Control
		else if(mode_motion == MOTION_ESCAPE_STR)
		{
			duty1 = DUTY_ESCAPE;
			duty2 = DUTY_ESCAPE;
			
			LEFT_MOTOR_F();
			RIGHT_MOTOR_F();
			
			OCR1B = (unsigned int)(duty1);
			OCR1C = (unsigned int)(duty2);
		}
		*/
	}
}

void Motor_PID(long sp1, long sp2)		// sp1 = left, sp2 = right
{
	error1_old = error1;
	error2_old = error2;
	
	error1 = (long)(ref1 - sp1);
	error2 = (long)(ref2 - sp2);
	
	errorsum1 += error1;
	errorsum2 += error2;
	
	//wind-up PID
	if(errorsum1 > ERR_SAT) 		errorsum1 = ERR_SAT;
	else if(errorsum1 < -ERR_SAT) 	errorsum1 = -ERR_SAT;
	if(errorsum2 > ERR_SAT) 		errorsum2 = ERR_SAT;
	else if(errorsum2 < -ERR_SAT) 	errorsum2 = -ERR_SAT;
	
//	duty1 = LPF(0.5, duty1, DC1_P * error1 + DC1_I * errorsum1 + DC1_D * (error1 - error1_old));
//	duty2 = LPF(0.5, duty2, DC2_P * error2 + DC2_I * errorsum2 + DC2_D * (error2 - error2_old));
	
	if(mode_motion != MOTION_ATTACK)
	{
		duty1 = LPF(0.4, duty1, DC1_P * error1 + DC1_I * errorsum1 + DC1_D * (error1 - error1_old));
		duty2 = LPF(0.4, duty2, DC2_P * error2 + DC2_I * errorsum2 + DC2_D * (error2 - error2_old));
	}
	else
	{
		duty1 = LPF(0.8, duty1, DC1_P * error1 + DC1_I * errorsum1 + DC1_D * (error1 - error1_old));
		duty2 = LPF(0.8, duty2, DC2_P * error2 + DC2_I * errorsum2 + DC2_D * (error2 - error2_old));
	}
	
	if(duty1 < 0)	LEFT_MOTOR_B();
	else 			LEFT_MOTOR_F();
	
	if(duty2 < 0) 	RIGHT_MOTOR_B();
	else 			RIGHT_MOTOR_F();
	
	if(duty1 < 0) duty1 = -duty1;
	if(duty2 < 0) duty2 = -duty2;
	
	if(duty1 > DUTY_MAX) duty1 = DUTY_MAX;
	if(duty2 > DUTY_MAX) duty2 = DUTY_MAX;
	
	OCR1B = (unsigned int)(duty1);
	OCR1C = (unsigned int)(duty2);
}
//////////////////////////////////////////////////////////////////////////////////////////
void getchars(void)
{
	if((UCSR1A & 0x80) != 0)	//수신
	{
		UCSR1A &= ~0x80;
		data_rx = UDR1;
	}
}

void uart1_putch(char ch)
{
	while(!(UCSR1A & 0x20));
	UDR1 = ch;
}

void putchars(void)
{
	if(data_rx == '1')
	{
		uart1_putch(spd1);
		uart1_putch(spd2);
	}
	else if(data_rx == '2')
	{
		uart1_putch(ir_black);
	}
	else if(data_rx == '3')
	{
		uart1_putch(mode_motion);
	}
	else if(data_rx == '4')
	{
		uart1_putch(dir_detect);
	}
	else if(data_rx == '5')
	{
		uart1_putch(tcnt);
	}
	else if(data_rx == '6')
	{
		uart1_putch(OCR1B >> 8);
		uart1_putch(OCR1B);
	}
	else if(data_rx == '7')
	{
		uart1_putch(OCR1C >> 8);
		uart1_putch(OCR1C);
	}
	else if(data_rx=='N')
	{
		motor_onoff = ~motor_onoff;
		
		data_rx = 0;
	}
	else if(data_rx=='+')
	{
		ref_def += REF_STEP;
		uart1_putch('0' + ref_def/10);
		
		data_rx = 0;
	}
	else if(data_rx=='-')
	{
		ref_def-= REF_STEP;
		uart1_putch('0' + ref_def/10);
				
		data_rx = 0;
	}
	else if(data_rx=='P')
	{
		DC1_P += 0.5;
		DC2_P += 0.5;
		
		data_rx = 0;
	}
	else if(data_rx=='p')
	{
		DC1_P -= 0.5;
		DC2_P -= 0.5;
		
		data_rx = 0;
	}
	else if(data_rx=='I')
	{
		DC1_I += 0.5;
		DC2_I += 0.5;
		
		data_rx = 0;
	}
	else if(data_rx=='i')
	{
		DC1_I -= 0.5;
		DC2_I -= 0.5;

		data_rx = 0;
	}
	else if(data_rx=='D')
	{
		DC1_D += 0.5;
		DC2_D += 0.5;
		
		data_rx = 0;
	}
	else if(data_rx=='d')
	{
		DC1_D -= 0.5;
		DC2_D -= 0.5;

		data_rx = 0;
	}
}
//////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER2_COMP_vect)
{
	if(cnt_esd > 150)
	{
		cnt_esd = 0;
		
		init_LS7366_LEFT();
		init_LS7366_LEFT();
		init_LS7366_RIGHT();
		init_LS7366_RIGHT();
	}
	else
	{
		cnt_esd++;	
	}
	////////////////////////////////////////
	// Sonar
	Get_Sensor();
	Check_Sensor();
	////////////////////////////////////////
	// IRed
	Get_IR();
	Check_IR();
	////////////////////////////////////////
	////////////////////////////////////////
	// Motion
	Make();
	Motion(spd_motion);
	// Encoder
	spd1_old = spd1;
	spd2_old = spd2;
	
	spd1 = get_speed(1);			// Left
	spd2 = -get_speed(2);			// Right
	
	if(spd1 > 200 || spd1 < - 200)
	{
		spd1 = spd1_old;
	}
	if(spd2 > 200 || spd2 < - 200)
	{
		spd2 = spd2_old;
	}
	// Control
	Motor_Control(motor_onoff);
	////////////////////////////////////////
	////////////////////////////////////////
	// USART
	getchars();
	putchars();
	////////////////////////////////////////
	tcnt = TCNT2;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
void Button(void)
{
	if(SW_1 != 0)
	{
		while(SW_1 != 0);
		flagLCD++;
		
		if(flagLCD==4)
		{
			flagLCD = 0;
		}
		init_LCD();
		clear_LCD();
	}
	if(SW_2 != 0)
	{
		while(SW_2 != 0);
		
		ref_def += REF_STEP;
	}
	if(SW_3 != 0)
	{
		while(SW_3 != 0);
		
		ref_def -= REF_STEP;
	}
	if(SW_4 != 0)	//right
	{
		while(SW_4 != 0);
		
		if(motor_onoff == 0x00)
		{
			_delay_ms(50);
			
			spd_motion = ref_def;
			Select_Motion(MOTION_ROTATECW, 180);

			ir_black = 2;			
			motor_onoff = 0xFF;	
		}
		else
		{
			ref1 = 0;
			ref2 = 0;
			
			spd_motion = 0;
			pulse_str_dst = 0;
			pulse_str_now = 0;
			pulse_rot_dst = 0;
			pulse_rot_now = 0;
			mode_motion = MOTION_STOP;
			
			motor_onoff = 0x00;		
		}
	}
	if(SW_5 != 0)	//straight
	{
		while(SW_5 != 0);
		
		if(motor_onoff == 0x00)
		{
			_delay_ms(50);
			
			spd_motion = ref_def;			
//			Select_Motion(MOTION_STRAIGHT, 20);

			ir_black = 2;
			motor_onoff = 0xFF;	
		}
		else
		{			
			motor_onoff = 0x00;	
		
			duty1 = 0;
			duty2 = 0;
				
			ref1 = 0;
			ref2 = 0;
			
			spd_motion = 0;
			pulse_str_dst = 0;
			pulse_str_now = 0;
			pulse_rot_dst = 0;
			pulse_rot_now = 0;
			mode_motion = MOTION_STOP;
		}
	}
	if(SW_6 != 0)	//left
	{
		while(SW_6 != 0);
		
		if(motor_onoff == 0x00)
		{
			_delay_ms(50);
			
			spd_motion = ref_def;
			Select_Motion(MOTION_ROTATECCW, 90);
		
			ir_black = 2;	
			motor_onoff = 0xFF;	
		}
		else
		{
			ref1 = 0;
			ref2 = 0;
			
			spd_motion = 0;
			pulse_str_dst = 0;
			pulse_str_now = 0;
			pulse_rot_dst = 0;
			pulse_rot_now = 0;
			mode_motion = MOTION_STOP;
			
			motor_onoff = 0x00;		
		}
	}
}

void LCD_Motor(void)
{
	write_snum1000(0, 0, ref1);
	write_snum1000(6, 0, spd1);
	write_num1000(12, 0, OCR1B);
	
	write_snum1000(0, 1, ref2);
	write_snum1000(6, 1, spd2);
	write_num1000(12, 1, OCR1C);
	
	write_num100(0, 2, DC1_P * 10);
	write_num100(4, 2, DC1_I * 10);
	write_num100(8, 2, DC1_D * 10);
	
	write_num100(0, 3, DC2_P * 10);
	write_num100(4, 3, DC2_I * 10);
	write_num100(8, 3, DC2_D * 10);
	
	write_snum1000(11, 2, errorsum1);
	write_snum1000(11, 3, errorsum2);
	
	write_snum100(16, 2, ref_def);
	
	write_num10(17, 3, ir_black);
}

void LCD_IR(void)
{
	write_num1000(0, 0, ir_filt[0]);
	write_num1000(0, 1, ir_filt[1]);
	write_num1000(0, 2, ir_filt[2]);
	write_num1000(0, 3, ir_filt[3]);
	
	write_num1(5, 0, ir_flag[0]);
	write_num1(5, 1, ir_flag[1]);
	write_num1(5, 2, ir_flag[2]);
	write_num1(5, 3, ir_flag[3]);
	
	write_num100(7, 0, pot_filt);	
	
	write_num10(7, 1, ir_chk_flag);
	write_num10(7, 2, mode_motion);
}

void LCD_Sonar(void)
{
	write_num1000(0, 0, sensor_raw[0]);
	write_num100(5, 0, sensor_filter[0][0]);
	write_num1(9, 0, sensor_detect[0]);
	
	write_num1000(0, 1, sensor_raw[1]);
	write_num100(5, 1, sensor_filter[1][0]);
	write_num1(9, 1, sensor_detect[1]);
	
	write_num1000(0, 2, sensor_raw[2]);
	write_num100(5, 2, sensor_filter[2][0]);
	write_num1(9, 2, sensor_detect[2]);
	
	write_word(19, 2, data_rx);
	
	write_string(0, 3, "Min:");
	write_num1(4, 3, dir_detect);
	
	write_num100(11, 0, sensor_i2c);
	write_num1000(11, 2, sensor_mcp[0]);
	write_num1000(11, 3, sensor_mcp[1]);
}

void LCD_Encoder(void)
{
	write_snum1000(0, 0, cnt_now1);
	write_snum1000(6, 0, spd1);
	
	write_snum1000(0, 1, cnt_now2);
	write_snum1000(6, 1, spd2);
	
	write_snum1000(0, 2, pulse_str_now);
	write_snum1000(6, 2, pulse_rot_now);
		
	write_snum1000(0, 3, pulse_str_dst);
	write_snum1000(6, 3, pulse_rot_dst);
	
	write_num1(14, 3, attack_disable);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(void)
{
	_delay_ms(1000);
	
	// GPIO, Timer
	Init_PORT();
	Init_TIMER();
	
	// ADC
	Init_ADC();
	init_I2C();
	
	// SPI, Encoder
	Init_SPI();
	_delay_ms(1);
	init_LS7366_LEFT();
	init_LS7366_LEFT();
	init_LS7366_RIGHT();
	init_LS7366_RIGHT();
	
	// USART
	Init_USART();
	
	// Interrupt
	sei();
	
	// LCD
	init_LCD();
	
    while(1)
    {		
		Button();
		
		write_num1(19, 0, flagLCD);
		write_num100(17, 1, tcnt);
		
		if(flagLCD == 0)
		{
			LCD_Motor();
		}
		else if(flagLCD == 1)
		{
			LCD_Encoder();
		}
		else if(flagLCD == 2)
		{
			LCD_Sonar();
		}
		else if(flagLCD == 3)
		{
			LCD_IR();
		}
    }
}

/*
 * INIT.h
 *
 * Created: 2015-10-17 오후 9:34:23
 *  Author: Administrator
 */ 


#ifndef INIT_H_
#define INIT_H_

void Init_PORT();
void Init_TIMER();
void Init_ADC();
void Init_USART(void);

void Init_PORT()
{
	DDRA |= 0x00;	// SW 1~6 => PA2~7(in)
	DDRB |= 0xF7;	// SPI => PB1, PB2, PB3(in), PB4, PB5 | Motor PWM => PB6, PB7 
	DDRC |= 0xFF;	// CLCD
	DDRD |= 0xF0;	// I2C => PD0, PD1(in/out) | BT => PD2(in), PD3 | Motor Dir => PD4, PD5, PD6, PD7
	DDRE |= 0xF0;	// Sonar Trigger => PE4, PE5 | MCP3202 CS => PE6, PE7
	DDRF |= 0x00;	// ADC
	DDRG |= 0x1F;	// Sonar BW => PG0, PG1, PG2 | Motor Enable => PG3, PG4

	PORTG |= 0x07;	// Sonar BW High
}

void Init_TIMER()
{
	// Timer1 DC Motor PWM
	TCCR1A = 0b00101010;			// FAST PWM(ICR)MODE, OCR1B,OCR1C(oc출력모드: 비교시 Low)
	TCCR1B = 0b00011010;			// prescaler 8 => 0.5us
	TCNT1 = 0;
	OCR1B = 0;						// Motor 1_duty
	OCR1C = 0;						// Motor 2_duty
	ICR1 = 2000;					// TOP값,주기1ms, 주파수 1khz
	
	// Timer2 Main Control Loop
	TCCR2=0b00001101;				//분주비 1024, 1count=64us
	TCNT2=0;
	OCR2=78;						// 64 * 255 = 16.32msec
	
	// Timer3 SonarAnalog Trigger
//	TCCR3A=0b10101010;				// FAST PWM(ICR)MODE, OCR3A, OCR3B, OCR3C(oc출력모드: 비교시 Low)
//	TCCR3B=0b00011101;				// prescaler 1024 => 64us
//	TCNT3=0;
//	OCR3A=2;						// 64 * 4 = 256usec, PE3
//	OCR3B=2;						// 64 * 4 = 256usec, PE4
//	OCR3C=2;						// 64 * 4 = 256usec, PE5
//	ICR3=1000;						// 64 * 1000 = 64msec					
	
	// Timer Interrupt Mask
	TIMSK=0x80;						// Compare match timer2 enable
}

void Init_ADC()
{
	ADMUX = 0x00;		//오른쪽 정렬
	ADCSRA= 0x87;		//1000 1111 E 분주비128
}

void Init_USART(void)
{
	DDRD |= 0x08;
	DDRD &= ~0x04;
	
	// 1Mbps 	 -> 0
	// 500000bps -> 1
	// 250000bps -> 3
	// 115200bps -> 8
	UBRR1H = 0;
	UBRR1L = 8;	
	
	UCSR1A = 0x00;
	UCSR1B = 0x18;		//수신, 송신허용
	UCSR1C = 0x06;		//비동기, stop비트1개, 전송크기8비트
}

#endif /* INIT_H_ */
/*
 * LS7366.h
 *
 * Created: 2015-08-23 오전 1:41:24
 *  Author: kjy
 */ 

#define LS7366_LEFT_SS_L()		PORTB &= ~0x10	//CLEAR_SS	(select slave)
#define LS7366_LEFT_SS_H()		PORTB |= 0x10	//SET_SS	:SS0인슬래브와통신
#define LS7366_RIGHT_SS_L()		PORTB &= ~0x20
#define LS7366_RIGHT_SS_H()		PORTB |= 0x20

#define MCP3202_1_SS_L()		PORTE &= ~0x40	// PE6 
#define MCP3202_1_SS_H()		PORTE |= 0x40	// PE6
#define MCP3202_2_SS_L()		PORTE &= ~0x80	// PE7
#define MCP3202_2_SS_H()		PORTE |= 0x80	// PE7

#define MCP3202_CH0				2
#define MCP3202_CH1				3

unsigned char shift_data_io(unsigned char c);

void init_LS7366_LEFT(void);
void init_LS7366_RIGHT(void);

unsigned long LS7366_LEFT_GetCount(void);
unsigned long LS7366_RIGHT_GetCount(void);

long get_speed(unsigned char n);

void Init_SPI()
{
	LS7366_LEFT_SS_H();
	LS7366_RIGHT_SS_H();
	MCP3202_1_SS_H();
	MCP3202_2_SS_H();
	
	SPCR = 0x50;		//setup SPI(SPI Control Register) 0b0101 0000 _SPI허용_마스터 분주비4
	SPSR = 0x00;		//setup SPI SPI인터럽트플래그 충돌플래그기록
}
unsigned char shift_data_io(unsigned char c)	//SPI 데이터 or 명령 전달
{
	SPDR = c;									//SPI Data register
	while(!(SPSR & (1<<SPIF)));					//0x80 SPI FLAG HIGH 수신 완료까지 대기(SPSR=SPI통신상태 레지스터)
	return (SPDR);
}

void init_LS7366_LEFT(void)
{
	LS7366_LEFT_SS_L();		//Set_MDR0
	_delay_us(1);			//SS low 후 대기
	shift_data_io(0x88);	//SELECT_MDR0 | WR_REG
	shift_data_io(0x23);	//x4 Count Mode
	_delay_us(1);
	LS7366_LEFT_SS_H();
	_delay_us(1);
	
	LS7366_LEFT_SS_L();		//Set_MDR1
	_delay_us(1);
	shift_data_io(0x90);	//SELECT_MDR1 | WR_REG
	shift_data_io(0x10);	//4 Bytes_Counter_Mode
	_delay_us(1);
	LS7366_LEFT_SS_H();
	_delay_us(1);
	
	LS7366_LEFT_SS_L();		//Clear(0) CNTR Register
	_delay_us(1);
	shift_data_io(0x20);	//SELECT_CNTR | CLR_REG
	_delay_us(1);
	LS7366_LEFT_SS_H();
	_delay_us(1);
	
	LS7366_LEFT_SS_L();		//Select_STR -> CLR
	_delay_us(1);
	shift_data_io(0x30);	//SELECT_STR | CLR_REG
	_delay_us(1);
	LS7366_LEFT_SS_H();
	_delay_us(1);
}

void init_LS7366_RIGHT(void)
{
	LS7366_RIGHT_SS_L();		//Set_MDR0 // ss 0으로 준비완료
	_delay_us(1);
	shift_data_io(0x88);	//SELECT_MDR0 | WR_REG
	shift_data_io(0x23);	//x4 Count Mode
	_delay_us(1);
	LS7366_RIGHT_SS_H();
	_delay_us(1);
	
	LS7366_RIGHT_SS_L();		//Set_MDR1
	_delay_us(1);
	shift_data_io(0x90);	//SELECT_MDR1 | WR_REG
	shift_data_io(0x10);	//4 Bytes_Counter_Mode
	_delay_us(1);
	LS7366_RIGHT_SS_H();
	_delay_us(1);
	
	LS7366_RIGHT_SS_L();		//
	_delay_us(1);
	shift_data_io(0x20);	//SELECT_CNTR | CLR_REG Clear(0) CNTR Register
	_delay_us(1);
	LS7366_RIGHT_SS_H();
	_delay_us(1);
	
	LS7366_RIGHT_SS_L();		//Select_STR -> CLR
	_delay_us(1);
	shift_data_io(0x30);	//SELECT_STR | CLR_REG
	_delay_us(1);
	LS7366_RIGHT_SS_H();
	_delay_us(1);
}

unsigned long LS7366_LEFT_GetCount()	// 엔코더값
{
	unsigned long cnt = 0;		//4byte

	LS7366_LEFT_SS_L();
	_delay_us(1);
	shift_data_io(0x60);
	cnt = ((unsigned long)shift_data_io(0x00) << 24);	//0x00 엔코더값읽기
	cnt |= ((unsigned long)shift_data_io(0x00) << 16);
	cnt |= ((unsigned long)shift_data_io(0x00) << 8);
	cnt |= ((unsigned long)shift_data_io(0x00));
	_delay_us(1);
	LS7366_LEFT_SS_H();
	_delay_us(1);
	
	return cnt;
}

unsigned long LS7366_RIGHT_GetCount()
{
	unsigned long cnt = 0;

	LS7366_RIGHT_SS_L();
	_delay_us(1);
	shift_data_io(0x60);
	cnt = ((unsigned long)shift_data_io(0x00) << 24);
	cnt |= ((unsigned long)shift_data_io(0x00) << 16);
	cnt |= ((unsigned long)shift_data_io(0x00) << 8);
	cnt |= ((unsigned long)shift_data_io(0x00));
	_delay_us(1);
	LS7366_RIGHT_SS_H();
	_delay_us(1);
	
	return cnt;
}

unsigned int MCP3202_GetADC(unsigned char config, unsigned char msbF)
{
	unsigned char data[3];
	unsigned int mcp3202_rx_data = 0;
	
	data[0] = 0x01;							// start signal
	data[1] = (config << 6) | (msbF << 5);	// config and MSB First select
	data[2] = 0x00;							// don't care
	
	shift_data_io(data[0]);
	mcp3202_rx_data = (unsigned int)(0x0F & shift_data_io(data[1])) << 8;
	mcp3202_rx_data |= (unsigned int)(shift_data_io(data[2]));
	
	return mcp3202_rx_data;
}
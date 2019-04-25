#define TWI_FREQ	80000UL

volatile unsigned char buffer[12];

volatile int gx = 0, gy = 0, gz = 0, ax = 0, ay = 0, az = 0;
volatile long x_aTmp1, y_aTmp1, z_aTmp1;
volatile float x_aTmp2, y_aTmp2, z_aTmp2, x_aResult, y_aResult, z_aResult;
volatile float x_gTmp1, y_gTmp1, x_gResult, y_gResult;

float kp = 12.0f, ki = 1.0f;
volatile float xTmp1, yTmp1, xTmp2, yTmp2, xIntTmp1, yIntTmp1;
volatile float xFilterAngle = 0.0f, yFilterAngle = 0.0f;
volatile int pitch = 0, roll = 0;

void init_I2C(void);
void TriggerSonarI2C(void);
unsigned int GetSonarI2C(void);

void init_MPU6050(void);
unsigned char MPU6050_read(unsigned char addr);
void MPU6050_write(unsigned char addr, char data);
void getRawData(void);
void getAcclDegree(void);		// Radian
void getGyroDegree(void);		// Angular Velocity
void compFilter(void);

void init_I2C(void)
{
	TWSR = 0x00;
	TWBR = (F_CPU / TWI_FREQ - 16) / 2;
}

void TriggerSonarI2C(void)
{
///////////////////////////////////////////////////////////////////////
	// start signal
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
	
	// address, write operation
	TWDR = 0xE0; 
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));

	// take range reading command
	TWDR = 0x51; 	
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));

	// stop signal
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

unsigned int GetSonarI2C(void)
{
	// min delay 65msec
	unsigned char H, L;
	unsigned int res;	
///////////////////////////////////////////////////////////////////////
	// start signal
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));			

	// address, read Operation
	TWDR = 0xE0 | 0x01; 
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
	while (!(TWCR & (1 << TWINT)));

	// read data
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
	while (!(TWCR & (1 << TWINT)));
	H = TWDR;

	// read data
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
	L = TWDR;

	// stop signal
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);

	res = H << 8 | L;

	return res;
}

void init_MPU6050(void)
{
	MPU6050_write(0x6B, 0x00);
	MPU6050_write(0x6C, 0x00);	
}

unsigned char MPU6050_read(unsigned char addr)
{
 	unsigned char dat;
  
  	// TWSTA 
 	TWCR = 0xA4;
 	while(((TWCR & 0x80) == 0x00 || ((TWSR & 0xF8) != 0x08)));
  
  	// TWINT Chip Addr Write
 	TWDR = 0xD0;
 	TWCR = 0x84;
 	while(((TWCR & 0x80) == 0x00 || ((TWSR & 0xF8) != 0x18)));
  
  	// TWINT Regs Write
 	TWDR = addr;
 	TWCR = 0x84;
 	while(((TWCR & 0x80) == 0x00 || ((TWSR & 0xF8) != 0x28)));

	// TWSTA
 	TWCR = 0xA4;
 	while(((TWCR & 0x80) == 0x00 || ((TWSR & 0xF8) != 0x10)));

	// TWINT Chip Addr Read
 	TWDR = 0xD1;
 	TWCR = 0x84;
 	while(((TWCR & 0x80) == 0x00 || ((TWSR & 0xF8) != 0x40)));

	// TWINT Data Read
 	TWCR = 0x84;
 	while(((TWCR & 0x80) == 0x00 || ((TWSR & 0xF8) != 0x58)));
  
 	dat = TWDR;

	// TWSTO
 	TWCR = 0x94;
  
	return dat;
}

void MPU6050_write(unsigned char addr, char data)
{
	// TW STA
 	TWCR = 0xA4;
 	while(((TWCR & 0x80) == 0x00 || ((TWSR & 0xF8) != 0x08)));
  
	// TW Addr Write
 	TWDR = 0xD0;
	TWCR = 0x84;
 	while(((TWCR & 0x80) == 0x00 || ((TWSR & 0xF8) != 0x18)));
  
	// TW Regs Write
 	TWDR = addr;
 	TWCR = 0x84;
 	while(((TWCR & 0x80) == 0x00 || ((TWSR & 0xF8) != 0x28)));
//------------------------------------------------------------- 

	// TW Regs Write
 	TWDR = data;
 	TWCR = 0x84;
 	while(((TWCR & 0x80) == 0x00 || ((TWSR & 0xF8) != 0x28)));
	
	// TW STO
 	TWCR = 0x94;
 	_delay_us(50);
}

void getRawData(void)
{
    buffer[0] = MPU6050_read(0x3B);
    buffer[1] = MPU6050_read(0x3C);
    buffer[2] = MPU6050_read(0x3D);
    buffer[3] = MPU6050_read(0x3E);
    buffer[4] = MPU6050_read(0x3F);
    buffer[5] = MPU6050_read(0x40);

    buffer[6] = MPU6050_read(0x43);
    buffer[7] = MPU6050_read(0x44);
    buffer[8] = MPU6050_read(0x45);
    buffer[9] = MPU6050_read(0x46);
    buffer[10] = MPU6050_read(0x47);
    buffer[11] = MPU6050_read(0x48);
     
    ax = (int)buffer[0] << 8 | (int)buffer[1];
    ay = (int)buffer[2] << 8 | (int)buffer[3];
    az = (int)buffer[4] << 8 | (int)buffer[5];

    gx = (int)buffer[6] << 8 | (int)buffer[7];
    gy = (int)buffer[8] << 8 | (int)buffer[9];
    gz = (int)buffer[10] << 8 | (int)buffer[11];
}

void getAcclDegree(void)		// Radian
{
    x_aTmp1 = ((long)ay * (long)ay) + ((long)az * (long)az);
    y_aTmp1 = ((long)ax * (long)ax) + ((long)az * (long)az);
    z_aTmp1 = ((long)ay * (long)ay) + ((long)az * (long)az);
     
    x_aTmp2 = sqrt((float)x_aTmp1);
    y_aTmp2 = sqrt((float)y_aTmp1);
    z_aTmp2 = sqrt((float)z_aTmp1);
     
    x_aResult = atan((float)ax / x_aTmp2);
    y_aResult = atan((float)ay / y_aTmp2);
    z_aResult = atan(z_aTmp2 / (float)az);
}

void getGyroDegree(void)
{
    x_gTmp1 = (float)gx / 65536;
    y_gTmp1 = (float)gy / 65536;
     
    x_gTmp1 = x_gTmp1 * 1.8;
    y_gTmp1 = y_gTmp1 * 1.8;
     
    x_gResult = x_gTmp1;
    y_gResult = y_gTmp1;
}

void compFilter(void)
{
    xTmp1 = (-y_aResult) + (float)xFilterAngle;
    xIntTmp1 = (float)xIntTmp1 + (xTmp1 * 0.01);
    xTmp2 = (-kp * xTmp1) + (-ki * (float)xIntTmp1) + x_gResult;
    xFilterAngle += (xTmp2 * 0.01);

    pitch = (int)(xFilterAngle * 180 / M_PI);
     
    yTmp1 = (-x_aResult) + (float)yFilterAngle;
    yIntTmp1 = (float)yIntTmp1 + (yTmp1 * 0.01);
    yTmp2 = (-kp * yTmp1) + (-ki * (float)yIntTmp1) + y_gResult;
    yFilterAngle += (yTmp2 * 0.01);

    roll = (int)(yFilterAngle * 180 / M_PI);
}
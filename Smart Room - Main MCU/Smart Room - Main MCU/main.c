/*
 * Smart Room - Main MCU.c
 *
 * Created: 6/18/2016 1:20:14 AM
 * Author : Yousef
 */ 

//16 MHz

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include <stdbool.h> 
#include <avr/sleep.h>

#define START				TWCR = (1 << TWINT)|(1 << TWSTA)|(1 << TWEN)|(1 << TWIE)
#define Prepare_DATA		TWCR = (1 << TWINT)|(1 << TWEN)|(1 << TWIE)
#define STOP				TWCR = (1 << TWINT)|(1 << TWSTO)|(1 << TWEN)|(1 << TWIE)
#define Command				receive_buffer_uart[0]
#define WindowData			receive_buffer_uart[1]
#define Bed1Data			receive_buffer_uart[2]
#define Bed2Data			receive_buffer_uart[3]
#define LightData			receive_buffer_uart[4]
#define CLOSED				0x00
#define OPEN				0x01
#define portB				(*((volatile access_one_bit *) 0x25))
#define portD				(*((volatile access_one_bit *) 0x2B))

typedef struct 
{
	unsigned int bit0 : 1;
	unsigned int bit1 : 1;
	unsigned int bit2 : 1;
	unsigned int bit3 : 1;
	unsigned int bit4 : 1;
	unsigned int bit5 : 1;
	unsigned int bit6 : 1;
	unsigned int bit7 : 1;
} access_one_bit;

union data_from_i2c
{
	unsigned int pulseCount;
	unsigned char data[2];
}windowMotor, bed1Motor, bed2Motor, doorMotor;
enum{lower, upper};

//DATA_1
unsigned char receive_buffer_uart[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char receive_buffer_i2c[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

bool flag1 = 1;
bool flagOfRequest = 0;
unsigned char counterRequest = 0;
bool flagOfADC = 0;
bool internal_flagOfADC = 1;
bool flagOfSleep = 0;
unsigned char i1;
unsigned char i2;

char counterADC = 0;
unsigned char channel = 0;

unsigned char transmit_buffer_i2c[8];
unsigned char temp;

const char TWI_adress = 0x03;

unsigned char current_state = CLOSED;

//functions
void window_bed1_bed2_motors_AND_light(void);
void motorsFeedBack(void);

int main(void)
{
	//I2C => 100KHz - Master
	TWBR = 0x12; //bit rate selector
	TWSR |= (1<<TWPS0);
	TWCR = 0x00;
	//TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTA) | (1<<TWIE);
	
	//UART Setup => 
	UCSR0B |= (1<<RXCIE0)|(1<<RXEN0);
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
	UBRR0H=0x00; 
	UBRR0L=0x10;	
		
	//Timer 1 Setup to overflow every 100 ms
	TCCR1A = 0x02;
	TCCR1B = 0x1C;
	ICR1 = 6249;
	TIMSK1 = 0x01;
	
	//ADC
	ADMUX = (0 << REFS1)| (0 << REFS0)| (0 << ADLAR)| (0 << MUX3)| (0 << MUX2)| (0 << MUX1)| (0 << MUX0);
	DIDR0 = (0<<ADC5D) | (0<<ADC4D) | (1<<ADC3D) | (1<<ADC2D) | (1<<ADC1D) | (1<<ADC0D);
	ADCSRA = (1<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
	//ADCSRB=(1<<ADTS2) | (1<<ADTS1) | (0<<ADTS0);
	
	//Analog Comparator
	ACSR |= (1<<ACIE) | (1<<ACIS0) | (1<<ACIS1) | (1 << ACBG);
	ADCSRB &= ~(1<<ACME);
	DIDR1 = 0x03;
	
	//Ext. Interrupts
	EICRA = (1<<ISC11)|(1<<ISC01)|(1<<ISC10)|(1<<ISC00);
	EIMSK = (1<<INT0)|(1<<INT1);
	
	//DATA_2
	DDRB |= 0x3F;
	DDRD |= 0x30;
	DDRD &= ~0x40;
	
	//sleep mode
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	 
	sei(); //global interrupt enabled
	
	//debugging
	//Command = 0x02;
	//WindowData = 0x01;
	//Bed1Data = 0x01;
	//windowMotor.pulseCount = 0xFF66;
	
	while (1) 
    {
		if(flagOfSleep)
		{
			sleep_mode();
			flagOfSleep = 0;
		}
		if(~PIND & 0x40) flagOfRequest = 1;
		else flagOfRequest = 0;
		window_bed1_bed2_motors_AND_light();
		motorsFeedBack();
	}
}


ISR(TWI_vect)
{
	switch(TWSR & 0xF8)
	{
		//Transmit
		case 0x08:
			if(internal_flagOfADC && !flagOfRequest) 
			{
				TWDR = (TWI_adress << 1);
				i1 = 0;
			}
			else if(flagOfRequest)
			{
				TWDR = (TWI_adress << 1) | 0x01;
				i2 = 0;
			}
			Prepare_DATA;
			break;
		case 0x18:
			TWDR = transmit_buffer_i2c[i1++];
			Prepare_DATA;
			break;
		case 0x28:
			if(i1 == 8)
				STOP;
			else
			{
				TWDR = transmit_buffer_i2c[i1++];
				Prepare_DATA;
			}	
			break;
		//receive	
		case 0x40:
			Prepare_DATA;
			TWCR |= (1 << TWEA);
			break;
		case 0x50:
			if(i2 != 6)
			{
				receive_buffer_i2c[i2++] = TWDR;	
				Prepare_DATA;
				TWCR |= (1 << TWEA);
			}
			else
			{
				receive_buffer_i2c[i2++] = TWDR;
				Prepare_DATA;
				TWCR &= ~(1 << TWEA);
			}
			break;
		case 0x58:
			receive_buffer_i2c[i2] = TWDR;
			STOP;
			break;
	}
}

ISR(TIMER1_OVF_vect)
{
	counterADC++;
	if(counterADC == 20)
	{
		if(flagOfADC && internal_flagOfADC)
		{
			internal_flagOfADC = 0;
			ADMUX = channel;
			ADCSRA |= (1<<ADSC);
		}
	    counterADC = 0;
	}
	if(flagOfRequest)
	{
		counterRequest++;
		if(counterRequest == 3)
		{
			counterRequest = 0;
			START;
		}
	}
}

ISR(ANALOG_COMP_vect)
{
	flagOfSleep = 1;
}

ISR(USART_RX_vect)
{
	if(flag1)
	{
		Command = UDR0;
		flag1 = 0;
	}
	else
	{
		switch(Command)
		{
			case 0x01:
			WindowData = UDR0;
			transmit_buffer_i2c[4] = WindowData;
			flag1 = 1;
			break;
			case 0x02:
			Bed1Data = UDR0;
			transmit_buffer_i2c[5] = Bed1Data;
			flag1 = 1;
			break;
			case 0x03:
			Bed2Data = UDR0;
			transmit_buffer_i2c[6] = Bed2Data;
			flag1 = 1;
			break;
			case 0x04:
			LightData = UDR0;
			flag1 = 1;
			break;
		}
	}
}


ISR(ADC_vect)
{
	transmit_buffer_i2c[channel] = ADCL;
	temp = ADCH;
	channel++;
	if(channel == 4) 
	{
		channel = 0;
		internal_flagOfADC = 1;
		if(!flagOfRequest) START;
	}
	else
	{
		ADMUX = channel;
		ADCSRA |= (1<<ADSC);
	}
}

ISR(INT0_vect)
{
	if(current_state == CLOSED)
	{
		portD.bit4 = 1;
		portD.bit5 = 0;
		transmit_buffer_i2c[7] = 0x01;
		current_state = OPEN;
	}
	else if(current_state == OPEN)
	{
		portD.bit4 = 0;
		portD.bit5 = 1;
		transmit_buffer_i2c[7] = 0x02;
		current_state = CLOSED;
	}
}

ISR(INT1_vect)
{
	if(current_state == CLOSED)
	{
		portD.bit4 = 1;
		portD.bit5 = 0;
		transmit_buffer_i2c[7] = 0x01;
		current_state = OPEN;
	}
	else if(current_state == OPEN)
	{
		portD.bit4 = 0;
		portD.bit5 = 1;
		transmit_buffer_i2c[7] = 0x02;
		current_state = CLOSED;
	}
}

void window_bed1_bed2_motors_AND_light()
{
	switch (Command)
	{
		case 0x01:
		portB.bit0 = WindowData;
		portB.bit1 = WindowData >> 1;
		break;
		case 0x02:
		portB.bit2 = Bed1Data;
		portB.bit3 = Bed1Data >> 1;
		break;
		case 0x03:
		portB.bit4 = Bed2Data;
		portB.bit5 = Bed2Data >> 1;
		break;
		case 0x04:
		flagOfADC = LightData;
		break;
	}

}

void motorsFeedBack()
{
	windowMotor.data[lower] = receive_buffer_i2c[0];
	windowMotor.data[upper] = receive_buffer_i2c[1];
	bed1Motor.data[lower] = receive_buffer_i2c[2];
	bed1Motor.data[upper] = receive_buffer_i2c[3];
	bed2Motor.data[lower] = receive_buffer_i2c[4];
	bed2Motor.data[upper] = receive_buffer_i2c[5];
	doorMotor.data[lower] = receive_buffer_i2c[6];
	doorMotor.data[upper] = receive_buffer_i2c[7];
	if((windowMotor.pulseCount == 10) || (windowMotor.pulseCount == 0))
	{
		WindowData = 0x03;
	}
	if((bed1Motor.pulseCount == 10) || (bed1Motor.pulseCount == 0))
	{
		Bed1Data = 0x03;
	}
	if((bed2Motor.pulseCount == 10) || (bed2Motor.pulseCount == 0))
	{
		Bed2Data = 0x03;
	}
	if((doorMotor.pulseCount == 10) || (doorMotor.pulseCount == 0))
	{
		portD.bit4 = 1;
		portD.bit5 = 1;
	}
}
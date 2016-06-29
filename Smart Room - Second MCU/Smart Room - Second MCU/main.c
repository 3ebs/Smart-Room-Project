/*
 * Smart Room - Second MCU.c
 *
 * Created: 6/18/2016 1:24:24 AM
 * Author : Yousef
 */ 

#include <avr/io.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <avr/sleep.h>

#define ACK							(TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWEN)|(1<<TWIE))
#define NACK						(TWCR = ((1<<TWINT)|(1<<TWEN)|(1<<TWIE))&(~(1<<TWEA)))

//DATA_1
const char address = 0x03;
unsigned char transmit_buffer_i2c[8];
unsigned char receive_buffer_i2c[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char i1;
unsigned char i2;
bool flagOfSleep = 1;
bool flag1 = true;
union data_to_i2c
{
	unsigned int pulseCount;
	unsigned char data[2];
}windowMotor, bed1Motor, bed2Motor, doorMotor;
enum{lower, upper};

//functions
void light();
void motors();
void request_i2c();
void release_i2c();

int main(void)
{
	//USART
	/*UCSR0B |= (1 << TXCIE0);
	UCSR0B |= 0x08;
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
	UBRR0H=0x00;
	UBRR0L=0x33;*/
	
	//Analog Comparator
	ACSR |= (1<<ACIE) | (1<<ACIS0) | (1<<ACIS1) | (1 << ACBG);
	ADCSRB &= ~(1<<ACME);
	DIDR1 = 0x03;
	
	//Timer 1
	TCCR1A |= (1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);
	TCCR1B |= (1<<WGM13)|(1<<WGM12)|(1<<CS11);
	ICR1 = 32786;
	OCR1A = 0;
	OCR1B = 0;
	//TIMSK1 = 0x01;
	
	//Timer 0
	TCCR0A |= (1<<COM0B1)|(1<<WGM00)|(1<<WGM01);
	TCCR0B |= (1<<CS00)|(1<<CS01);
	OCR0B = 0;
	
	//Timer 2
	TCCR2A |= (1<<COM2A1)|(1<<WGM20)|(1<<WGM21);
	TCCR2B |= (1<<CS22);
	OCR2A = 0;
	
	//Ext. Interrupts
	EICRA = (1<<ISC11)|(1<<ISC01)|(1<<ISC10)|(1<<ISC00);
	EIMSK = (1<<INT0)|(1<<INT1);
	PCICR |= (1<<PCIE2)|(1<<PCIE0);
	PCMSK2 |= (1<<PCINT20);
	PCMSK0 |= (1<<PCINT0);
	
	//I2C
	TWCR |= (1 << TWEN) | (1 << TWIE) | (1 << TWEA) | (1 << TWINT);
	TWCR &= ~(1 << TWSTA) & ~(1 << TWSTO) & ~(1 << TWWC);
	TWAR = (address << 1);
	
	//DATA_2
	DDRB |= 0x0E;
	DDRD |= 0x21;
	PORTD |= 0x01;
	
	//sleep mode
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	
	//global interrupt
	sei();
	
	//debugging
	
    while (1) 
    {
		if(flagOfSleep)
		{
			sleep_mode();
			flagOfSleep = 0;
		}
		light();
    }
}

ISR(ANALOG_COMP_vect)
{
	flagOfSleep = 1;
}

/*ISR(TIMER1_OVF_vect)
{
	counter++;
	if(counter == 10)
	{
		counter = 0;
		SPDR = data_received;
	}
}*/

ISR(TWI_vect)
{
	switch(TWSR & 0xF8)
	{
		//receive
		case 0x60:
			i1 = 0;
			ACK;
			break;
		case 0x80:
			receive_buffer_i2c[i1++] = TWDR;
			ACK;
			break;
		case 0xA0:
			ACK;
			break;
		//transmit
		case 0xA8:
			i2 = 0;
			TWDR = transmit_buffer_i2c[i2++];
			ACK;
			break;
		case 0xB8:
			if(flag1)
			{
				motors();
				flag1 = false;
			}
			TWDR = transmit_buffer_i2c[i2++];	
			if(i2 == 8) NACK;
			else ACK;
		case 0xC0:
			flag1 = true;
			ACK;
			break;			
	}	
}

ISR(INT0_vect)
{
	if(receive_buffer_i2c[4] == 0x01) windowMotor.pulseCount++;
	else if(receive_buffer_i2c[4] == 0x02) windowMotor.pulseCount--;
	else if(receive_buffer_i2c[4] == 0x03 || receive_buffer_i2c[4] == 0x00) release_i2c();
	if(windowMotor.pulseCount > 5 && receive_buffer_i2c[4] == 0x01) request_i2c();
	if(windowMotor.pulseCount < 5 && receive_buffer_i2c[4] == 0x02) request_i2c();
}

ISR(INT1_vect)
{
	if(receive_buffer_i2c[5] == 0x01) bed1Motor.pulseCount++;
	else if(receive_buffer_i2c[5] == 0x02) bed1Motor.pulseCount--;
	else if(receive_buffer_i2c[5] == 0x03 || receive_buffer_i2c[5] == 0x00) release_i2c();
	if(bed1Motor.pulseCount > 5 && receive_buffer_i2c[5] == 0x01) request_i2c();
	if(bed1Motor.pulseCount < 5 && receive_buffer_i2c[5] == 0x02) request_i2c();
}

ISR(PCINT0_vect)
{
	if(receive_buffer_i2c[7] == 0x01) doorMotor.pulseCount++;
	else if(receive_buffer_i2c[7] == 0x02) doorMotor.pulseCount--;
	else if(receive_buffer_i2c[7] == 0x03 || receive_buffer_i2c[7] == 0x00) release_i2c();
	if(doorMotor.pulseCount > 10 && receive_buffer_i2c[7] == 0x01) request_i2c();
	if(doorMotor.pulseCount < 10 && receive_buffer_i2c[7] == 0x02) request_i2c();
}

ISR(PCINT2_vect)
{
	if(receive_buffer_i2c[6] == 0x01) bed2Motor.pulseCount++;
	else if(receive_buffer_i2c[6] == 0x02) bed2Motor.pulseCount--;
	else if(receive_buffer_i2c[6] == 0x03 || receive_buffer_i2c[6] == 0x00) release_i2c();
	if(bed2Motor.pulseCount > 10 && receive_buffer_i2c[6] == 0x01) request_i2c();
	if(bed2Motor.pulseCount < 10 && receive_buffer_i2c[6] == 0x02) request_i2c();
}

void light()
{
	OCR1A = receive_buffer_i2c[0];
	OCR1B = receive_buffer_i2c[1];
	OCR2A = receive_buffer_i2c[2];
	OCR0B = receive_buffer_i2c[3];
}

void motors()
{
	transmit_buffer_i2c[0] = windowMotor.data[lower];
	transmit_buffer_i2c[1] = windowMotor.data[upper];
	transmit_buffer_i2c[2] = bed1Motor.data[lower];
	transmit_buffer_i2c[3] = bed1Motor.data[upper];
	bed2Motor.pulseCount = bed2Motor.pulseCount >> 2;
	transmit_buffer_i2c[4] = bed2Motor.data[lower];
	transmit_buffer_i2c[5] = bed2Motor.data[upper];
	doorMotor.pulseCount = doorMotor.pulseCount >> 2;
	transmit_buffer_i2c[6] = doorMotor.data[lower];
	transmit_buffer_i2c[7] = doorMotor.data[upper];
}

void request_i2c()
{
	PORTD &= ~0x01; 
}

void release_i2c()
{
	PORTD |= 0x01;
}
/**
 * \main.c
 *
 * \brief main routine for Fastlane code
 *			Version 1.0
 *			Edited 2/25/2014
 *			Revised 3 March 2014
 *			Zachary Diggins
 */

// This include file allows data to be stored in program space.  The
// ATmega168 has 16k of program space compared to 1k of RAM, so large
// pieces of static data should be stored in program space.
#include <avr/pgmspace.h>

#include <stdio.h>


#include <asf.h>

#define F_CPU 8000000UL 
#define BAUD 9600
#define FOSC 8000000
#define MYUBRR FOSC/16/BAUD-1


#include <util/delay.h>

/////////////////////////
//Configuration Variables

//Mode: 1 = line following 3 state, 2 = playback, 3 = line following PID
const int mode = 1;



//////////////////////////////
//Supporting functions





uint16_t ReadADC(uint8_t ADCchannel)
{
	//select ADC channel with safety mask
	ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);
	//single conversion mode
	ADCSRA |= (1<<ADSC);
	// wait until ADC conversion is complete
	while( ADCSRA & (1<<ADSC) );
	
	return (ADC);
	//return (ADC/4*+200);
}

void setMotors(uint8_t leftSpeed, uint8_t rightSpeed, uint8_t leftDirection, uint8_t rightDirection)
{

	OCR0A = leftSpeed;
	OCR0B = rightSpeed;
	
	//1 is forward
	//anything else is reverse
	
	if (leftDirection == 1)
	{
		//Set up Motor Direction Pin
		DDRD |= (1 << DDD7);  //Left motor direction
	}
	else 
	{
		//Set up Motor Direction Pin
		DDRD &= ~(1 << DDD7);  //Left motor direction
	}
	
	if (rightDirection == 1)
	{
		DDRB |= (1 << DDB0);  //Right motor direction
	}
	else
	{
		DDRB &= ~(1 << DDB0);  //Right motor direction
	}
	
}

//Sets up PWM
void motor_init()
{
	//Set up PWM
	DDRD |= (1 << DDD6);
	DDRD |= (1 << DDD5);
	// PD6 is now an output
	OCR0A = 0;
	OCR0B = 0;
	// start motors at 0% duty cycle
	TCCR0A |= (1 << COM0A1);
	// set none-inverting mode
	TCCR0A |= (1 << COM0B1);
	// set none-inverting mode
	TCCR0A |= (1 << WGM01) | (1 << WGM00);
	// set fast PWM Mode
	TCCR0B |= (1 << CS01);
	// set prescaler to 8 and starts PWM
	
	//Set up Motor Direction Pin
	DDRD |= (1 << DDD7);  //Left motor direction 
	DDRB |= (1 << DDB0);  //Right motor direction
}

void adc_init()
{
	//Set up ADC
	// Select Vref=AVcc
	ADMUX |= (1<<REFS0);
	//set prescaller to 128 and enable ADC
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);
}

void uart_init (unsigned int ubrr)
{
	//Set baud rate
	UBRR0H = (unsigned char) (ubrr>>8);
	UBRR0L = (unsigned char) ubrr;
	
	//Enable receiver and transmitter
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	
	//Set frame format
	UCSR0C = (0<<USBS0)|(3<<UCSZ00);
	
}

void uart_putchar(char c, FILE *stream) {
	if (c == '\n') {
		uart_putchar('\r', stream);
	}
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c;
}

char uart_getchar(FILE *stream) {
	loop_until_bit_is_set(UCSR0A, RXC0);
	return UDR0;
}

void USART_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) )
	;
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

unsigned char USART_Receive( void )
{
	/* Wait for data to be received */
	while ( !(UCSR0A & (1<<RXC0)) )
	;
	/* Get and return received data from buffer */
	return UDR0;
}

////////////////////////////////
//Global Variables

FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
FILE uart_input = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);

uint16_t sensor0 = 0;
uint16_t sensor1 = 0;
uint16_t sensor2 = 0;
uint16_t sensor3 = 0;
uint16_t sensor4 = 0;
uint16_t sensor5 = 0;

uint16_t position = 3800;
uint16_t sum = 0;

void initialize()
{
	
	uart_init(MYUBRR);
	stdout = &uart_output;
	stdin  = &uart_input;
	
	board_init();
	motor_init();
	adc_init();
}

uint16_t read_line()
{
	sensor0 = ReadADC(5);
	sensor1 = ReadADC(4);
	sensor2 = ReadADC(3);
	sensor3 = ReadADC(2);
	sensor4 = ReadADC(1);
	sensor5 = ReadADC(0);
	
	unsigned int sensor_values[6];
	sensor_values[0] = ReadADC(5);
	sensor_values[1] = ReadADC(4);
	sensor_values[2] = ReadADC(3);
	sensor_values[3] = ReadADC(2);
	sensor_values[4] = ReadADC(1);
	sensor_values[5] = ReadADC(0);
	
	unsigned char i, on_line = 0;
	unsigned long avg; // this is for the weighted total, which is long
	// before division
	unsigned int sum; // this is for the denominator which is <= 64000
	static int last_value=0; // assume initially that the line is left.

	avg = 0;
	sum = 0;
	
	//FIX
	
	for(i=1 ; i<=6 ;i++) {
		int value = sensor_values[i];

		// keep track of whether we see the line at all
		if(value > 200) {
			on_line = 1;
		}
		
		// only average in values that are above a noise threshold
		if(value > 50) {
			avg += (long)(value) * (i * 1000);
			sum += value;
		}
	}

	if(!on_line)
	{
		// If it last read to the left of center, return 0.
		if(last_value < (6-1)*1000/2)
		return 0;
		
		// If it last read to the right of center, return the max.
		else
		return (6-1)*1000;

	}

	last_value = avg/sum;

	return last_value;
}


////////////////////////////////
//Start of main routine
int main (void)
{

	unsigned int last_proportional=0;
	long integral=0;
	
	char str[400]; //buffer for printing to Arduino Due
	
	initialize();
	
	while(1)
	{
		
		
		
		//Print sensor values mode - incorporate into control structure later
		char fromPC = USART_Receive();
		
		if (fromPC == 'm')
		{
			setMotors(100,100,1,1);
		}
		if (fromPC == 's')
		{
			
			//puts("Sensor readings: \n");
		

			sensor0 = ReadADC(5);
			//_delay_ms(1);
			sensor1 = ReadADC(4);
			//_delay_ms(1);
			sensor2 = ReadADC(3);
			//_delay_ms(1);
			sensor3 = ReadADC(2);
			//_delay_ms(1);
			sensor4 = ReadADC(1);
			//_delay_ms(1);
			sensor5 = ReadADC(0);
		
			unsigned int position = read_line();
		
			sprintf(str, "%d %d %d %d %d %d %d", sensor0, sensor1, sensor2, sensor3, sensor4, sensor5, position);
		
			puts(str);
		
		}
		
		if (fromPC == '1' || fromPC == '2' || fromPC == '3' 
					|| fromPC == '4')
		{
			while(1)
			{
				// Get the position of the line.  Note that we *must* provide
				// the "sensors" argument to read_line() here, even though we
				// are not interested in the individual sensor readings.
				unsigned int position = read_line();
				
				
				//sprintf(str, "Position %d", position);
				
				//puts(str);
				
				// The "proportional" term should be 0 when we are on the line.
				int subtractBy = 2600;
				if (fromPC == '4')
					subtractBy = 3800;
				int proportional = ((int)position) - subtractBy;
				//sprintf(str, "Proportional %d", proportional);
				
				//puts(str);

				// Compute the derivative (change) and integral (sum) of the
				// position.
				int derivative = proportional - last_proportional;
				integral += proportional;

				// Remember the last position.
				last_proportional = proportional;
				
				// Compute the difference between the two motor power settings,
				// m1 - m2.  If this is a positive number the robot will turn
				// to the right.  If it is a negative number, the robot will
				// turn to the left, and the magnitude of the number determines
				// the sharpness of the turn.
				int kpDivider = 40;
				if (fromPC == '3')
					kpDivider = 20;
				
				int power_difference = proportional/kpDivider + derivative*3/2 + integral/10000;
				
				//sprintf(str, "Power difference: %d", power_difference);
				
				//puts(str);
				
				// Compute the actual motor settings.  We never set either motor
				// to a negative value.
				const int max = 100;
				if (fromPC== '2')
					max = 150;
				if(power_difference > max)
				power_difference = max;
				if(power_difference < -max)
				power_difference = -max;

				if(power_difference < 0)
				setMotors(max,max+power_difference,1,1);
				else
				setMotors(max-power_difference, max, 1,1);
				
				
				sum = sensor0 + sensor1 + sensor2 + sensor3 + sensor4 + sensor5;
				
				_delay_ms(1);		
			}
		}		
	}
	
	
	
		
		
		

}

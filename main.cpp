#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define BLINK_DELAY	( 250 )
#define RAPID_DELAY	( 25 )
#define PWM_DELAY	( 1000 - BLINK_DELAY )
#define PWM_TOP		( 79 )

#define DUTY_CYCLE( P )	( unsigned char )( ( ( P / 100.0f ) * PWM_TOP ) - 1 )

volatile unsigned long g_tachCount = 0;

/**
 * Configure pins for input, output, pull-ups etc.
 */
void configurePins( void )
{
	// Configure all pins as inputs
	DDRB = 0;
	DDRC = 0;
	DDRD = 0;

	// Configure all pins without pull-ups
	PORTB = 0;
	PORTC = 0;
	PORTD = 0;

	DDRB |= _BV( DDB5 );	// PB5 (Arduino Uno Pin 13) => OUTPUT (LED)
	DDRD |= _BV( DDD3 );	// PD3 (Arduino Uno Pin 3)  => OUTPUT (PWM)

	PORTB |= _BV( DDD2 );   // PB2 (Arduino Uni Pin 2 ) => Pull-up Resistor

	// Set up INT0 to trigger on rising edge
	EICRA |= _BV( ISC00 ) | _BV( ISC01 );
	EIMSK |= _BV( INT0 );

	sei();
}

/**
 * Provide a variable delay.
 */
void variableDelay( unsigned int delay )
{
	for( unsigned int i = 0; i < delay; i++ )
	{
		_delay_ms( 1 );
	}
}

/**
 * Blink the LED on PB5.
 */
void blinkLed( unsigned int length )
{
	// LED on
	PORTB |= _BV( PORTB5 );		// PB5 => HIGH (LED ON)

	// Short delay
	variableDelay( length );

	// LED off
	PORTB &= ~_BV( PORTB5 );	// PB5 => LOW (LED OFF)
}

void toggleLed(void)
{
	PORTB ^= _BV( PORTB5 );
}

/**
 * Rapid blink the LED on PB5.
 */
void multiBlink( const unsigned char count, unsigned int onLength, unsigned int offLength )
{
	for( unsigned char i = 0; i < count; i++ )
	{
		// LED on
		PORTB |= _BV( PORTB5 );		// PB5 => HIGH (LED ON)
		variableDelay( onLength );

		// LED off
		PORTB &= ~_BV( PORTB5 );	// PB5 => LOW (LED OFF)
		variableDelay( offLength );
	}
}

/*
 * Start the PWM signal on PB3 at the defined duty cycle.
 */
int startPWM( const float dutyCycleAsPercentage )
{
	unsigned char dutyCycle = 0;

	// Validate the dutyCycle (0-100)
	if( dutyCycleAsPercentage > 100) return -1;

	// Set up the registers
	//
	// Fast PWM w/ TOP (WGM20 | WGM21 | WGM22)
 	// Non-Inverted    (COM2A1 | COM2B1)
	// Prescaler of 8  (CS21)
	// Clock: 16MHz
	//
	// Output B frequency: 16MHz / 8 / ( 79 + 1 ) = 25KHz
	// OCR2B = ( ( dutyCycleAsPercentage / 100 ) * PWM_TOP ) - 1
	dutyCycle = DUTY_CYCLE(  dutyCycleAsPercentage );

	TCCR2A = _BV( COM2A1 ) | _BV( COM2B1 ) | _BV( WGM21 ) | _BV( WGM20 );
	TCCR2B = _BV( CS21 ) | _BV( WGM22 );
	OCR2A = PWM_TOP;
	OCR2B = dutyCycle;

	return dutyCycle;
}

/**
 * Set the PWM duty cycle.
 */
int setPWMDutyCycle( const float dutyCycleAsPercentage )
{
	unsigned char dutyCycle = 0;

	// Validate the dutyCycle (0-100)
	if( dutyCycleAsPercentage > 100) return -1;

	// OCR2B = ( ( dutyCycleAsPercentage / 100 ) * PWM_TOP ) - 1
	dutyCycle = DUTY_CYCLE(  dutyCycleAsPercentage );

	OCR2B = dutyCycle;

	return dutyCycle;
}

/**
 * Ramp up and back down an attached PWM fan.
 */
void benchmarkPWMFan( const unsigned char min, const unsigned char max, const unsigned char increment )
{
	setPWMDutyCycle( 0 );

	// Give the fan a chance to actually stop
	variableDelay( 10 * PWM_DELAY );

	for( unsigned char dutyCycle = min; dutyCycle <= max; dutyCycle += increment )
	{
		setPWMDutyCycle( dutyCycle );
		//blinkLed( BLINK_DELAY );
		variableDelay( PWM_DELAY );
	}

	// Hang at the top for a bit
	variableDelay( 10 * PWM_DELAY );

	for( unsigned char dutyCycle = max - increment; dutyCycle >= min; dutyCycle -= increment )
	{
		setPWMDutyCycle( dutyCycle );
		//blinkLed( BLINK_DELAY );
		variableDelay( PWM_DELAY );
	}

	setPWMDutyCycle( 0 );
}

/**
 * The main program loop.
 */
int main( void )
{
	configurePins();

	if( startPWM( 0 ) >= 0 )
	{
		while( true )
		{
			//multiBlink( 10, 10 * RAPID_DELAY, 10 * RAPID_DELAY );
			benchmarkPWMFan( 10, 100, 2 );
			//multiBlink( 100, RAPID_DELAY, RAPID_DELAY );
		}
	}

	return 0;
}

ISR(INT0_vect)
{
	g_tachCount++;
	toggleLed();
}

#include <avr/io.h>
#include <util/delay.h>

#define BLINK_DELAY	( 250 )
#define RAPID_DELAY	( 25 )
#define PWM_DELAY	( 5000 - BLINK_DELAY )
#define PWM_TOP		( 20 )

/**
 * Configure pins for input, output, pull-ups etc.
 */
void configurePins( void )
{
	DDRB |= _BV( DDB5 );	// PB5 (Arduino Uno Pin 13) => OUTPUT (LED)
	DDRD |= _BV( DDB3 );	// PD3 (Arduino Uno Pin 3) => OUTPUT (PWM)
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
int startPWM( const unsigned char dutyCycleAsPercentage )
{
	unsigned char dutyCycle = 0;

	// Validate the dutyCycle (0-100)
	if( dutyCycleAsPercentage > 100) return -1;

	// Set up the registers
	//
	// Phase-correct PWM
 	// Non-Inverted
	// Prescaler: 8
	// Clock: 16Mhz
	//
	// Output A frequency: 16MHz / 8 / 20 / 2 / 2 = 12.5kHz
	// Output A duty cycle: 50%
	// Output B frequency: 16MHz / 8 / 20 / 2 = 25kHz
	// Output B duty cycle: OCR2B / 20 = dutyCycleAsPercentage%
	//
	// OCR2B = ( dutyCycleAsPercentage / 100 ) * 20
	dutyCycle = ( unsigned char )( ( ( float ) dutyCycleAsPercentage / 100.0f ) * PWM_TOP );

	TCCR2A = _BV( COM2A0 ) | _BV( COM2B1 ) | _BV( WGM20 );
	TCCR2B = _BV( WGM22 ) | _BV( CS21 );
	OCR2A = PWM_TOP;
	OCR2B = dutyCycle;

	return dutyCycle;
}

/**
 * Set the PWM duty cycle.
 */
int setPWMDutyCycle( const unsigned char dutyCycleAsPercentage )
{
	unsigned char dutyCycle = 0;

	// Validate the dutyCycle (0-100)
	if( dutyCycleAsPercentage > 100) return -1;

	// OCR2B = ( dutyCycleAsPercentage / 100 ) * 20
	dutyCycle = ( unsigned char )( ( ( float ) dutyCycleAsPercentage / 100.0f ) * PWM_TOP );

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
	variableDelay( PWM_DELAY );

	for( unsigned char dutyCycle = min; dutyCycle <= max; dutyCycle += increment )
	{
		setPWMDutyCycle( dutyCycle );
		blinkLed( BLINK_DELAY );
		variableDelay( PWM_DELAY );
	}

	for( unsigned char dutyCycle = max - increment; dutyCycle >= min; dutyCycle -= increment )
	{
		setPWMDutyCycle( dutyCycle );
		blinkLed( BLINK_DELAY );
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
			multiBlink( 10, 10 * RAPID_DELAY, 10 * RAPID_DELAY );
			benchmarkPWMFan( 20, 100, 20 );
			multiBlink( 100, RAPID_DELAY, RAPID_DELAY );
		}
	}

	return 0;
}

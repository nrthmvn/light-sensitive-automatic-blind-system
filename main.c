#define F_CPU 16000000UL // Clock Speed - 16MHz

// USART Definitions
#define BAUD 9600 // Baud rate for USART communication
#define UBRR_VALUE ((F_CPU / (16UL * BAUD)) - 1) // Calculate UBRR value for baud rate setting
#define USART_TX_BUFFER_SIZE 128 // Size of the USART transmission buffer

// AVR Headers
#include <avr/io.h> // Standard AVR IO definitions
#include <avr/interrupt.h> // AVR interrupt definitions

// Standard Library Headers
#include <stdbool.h> // Boolean definitions
#include <stdlib.h> // For functions like itoa()

// USART Global Variables
volatile char usart_tx_buffer[USART_TX_BUFFER_SIZE]; // Transmission buffer for USART
volatile uint8_t usart_tx_head = 0; // Head index for the transmission buffer
volatile uint8_t usart_tx_tail = 0; // Tail index for the transmission buffer

// Other Global Variables
volatile uint8_t mode = 0; // Current operating mode (0: Data, 1: Automatic, 2: Manual)
volatile uint8_t interrupt_count = 0; // Counter for timer interrupts
volatile uint16_t adc_sample_level = 0; // Latest ADC sample value
volatile uint16_t potentiometer_level = 0; // Latest potentiometer sample value (not used in this code)
volatile bool new_adc_sample = false; // Flag indicating a new ADC sample is available
volatile bool new_potentiometer_sample = false; // Flag for new potentiometer sample (not used)
volatile bool power_state = false; // System power state (on/off)

// Function to initialize GPIO pins
void init_gpio(void)
{
	DDRB = 0x3F; // Pins 0 to 5 set as outputs; Pins 6 and 7 not used
	DDRD = 0xF0; // Pins 4 to 7 set as outputs; Pins 0 to 3 as inputs
	PORTD = (1 << PORTD4) // Turn on standby light (7-segment display decimal point)
	| (1 << PORTD3) | (1 << PORTD2); // Enable pull-up resistors on Pins 2 and 3 (buttons)
	EICRA = (1 << ISC11) // Falling edge on INT1 generates an interrupt request
	| (1 << ISC01); // Falling edge on INT0 generates an interrupt request
	EIMSK = (1 << INT1) | (1 << INT0); // Enable external interrupts from INT1 and INT0
}

// Function to initialize Timer0
void init_timer0(void)
{
	TCNT0 = 0x00; // Reset Timer0 counter to 0
	TCCR0B = (1 << CS02) | (1 << CS00); // Set prescaler to 1024
	TIMSK0 = (1 << TOV0); // Enable Timer0 Overflow interrupt
}

// Function to initialize Timer1 for PWM
void init_timer1(void)
{
	TCNT1 = 0x0000; // Reset Timer1 counter to 0
	TCCR1A = (1 << COM1A1) // Clear OC1A on compare match, set OC1A at BOTTOM (non-inverting mode)
	| (1 << WGM11); // Set Fast PWM mode using ICR1 as TOP (Mode 14)
	TCCR1B = (1 << WGM13) | (1 << WGM12) // Set Fast PWM mode (Mode 14)
	| (1 << CS11); // Set prescaler to 8
	ICR1 = 39999; // Set TOP value for 50 Hz PWM frequency
	OCR1A = 1000; // Set initial compare value for servo position
}

// Function to initialize ADC
void init_adc(void)
{
	DDRC = 0x00; // All PORTC pins set as inputs (ADC inputs)
	ADMUX = (1 << REFS0); // Use AVcc as reference voltage, select ADC0 as input channel
	ADCSRA = (1 << ADEN)  // Enable ADC
	| (1 << ADIE) // Enable ADC conversion complete interrupt
	| (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescaler to 128 (for 125 kHz ADC clock)
}

// Function to initialize USART
void init_usart(void)
{
	UBRR0H = (UBRR_VALUE >> 8); // Set high byte of UBRR (baud rate register)
	UBRR0L = UBRR_VALUE; // Set low byte of UBRR

	UCSR0B = (1 << TXEN0) // Enable transmitter
	| (1 << RXEN0) // Enable receiver
	| (1 << RXCIE0); // Enable USART Receive Complete interrupt

	// Set frame format: 8 data bits, no parity, 1 stop bit
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Function to transmit a single character via USART
void usart_transmit_char(char data) {
	uint8_t next_head = (usart_tx_head + 1) % USART_TX_BUFFER_SIZE; // Calculate next head position

	while (next_head == usart_tx_tail); // Wait if buffer is full

	// Add data to buffer
	usart_tx_buffer[usart_tx_head] = data;
	usart_tx_head = next_head;

	// Enable Data Register Empty interrupt to start transmission
	UCSR0B |= (1 << UDRIE0);
}

// Function to transmit a string via USART
void usart_transmit_string(const char *str) {
	while (*str) {
		usart_transmit_char(*str++); // Transmit each character in the string
	}
}

// Function to display '0' on seven-segment display
void seven_segment_zero(void)
{
	PORTB &= ~(0b01111100); // Clear segments connected to PORTB
	PORTB |= 0b00011100; // Set segments to display '0' on PORTB
	PORTD &= ~(0b11110000); // Clear segments connected to PORTD
	PORTD |= 0b11100000; // Set segments to display '0' on PORTD
}

// Function to display '1' on seven-segment display
void seven_segment_one(void)
{
	PORTB &= ~(0b01111100); // Clear segments connected to PORTB
	PORTB |= 0b00000100; // Set segments to display '1' on PORTB
	PORTD &= ~(0b11110000); // Clear segments connected to PORTD
	PORTD |= 0b00100000; // Set segments to display '1' on PORTD
}

// Function to display '2' on seven-segment display
void seven_segment_two(void)
{
	PORTB &= ~(0b01111100); // Clear segments connected to PORTB
	PORTB |= 0b00101100; // Set segments to display '2' on PORTB
	PORTD &= ~(0b11110000); // Clear segments connected to PORTD
	PORTD |= 0b11000000; // Set segments to display '2' on PORTD
}

// Interrupt Service Routine for External Interrupt INT0 (Power Button Toggle)
ISR(INT0_vect) // Power Button Toggle
{
	if (power_state) {
		power_state = false; // Turn off the system
		PORTB &= ~(0b00111101); // Turn off outputs connected to PORTB
		PORTD &= ~(0b11110000); // Turn off outputs connected to PORTD
		PORTD |= (1 << PORTD4); // Turn on standby light (e.g., decimal point on 7-segment display)
	}
	else
	{
		power_state = true; // Turn on the system
		PORTB |= (1 << PORTB0); // Possibly turn on an indicator LED connected to PORTB0
		PORTD &= ~(1 << PORTD4); // Turn off standby light
		switch (mode)
		{
			case 0:
			seven_segment_zero(); // Display '0' for Data Mode
			break;
			case 1:
			seven_segment_one(); // Display '1' for Automatic Mode
			break;
			case 2:
			seven_segment_two(); // Display '2' for Manual Mode
			break;
			default:
			break;
		}
	}
}

// Interrupt Service Routine for External Interrupt INT1 (Mode Button)
ISR(INT1_vect)
{
	if (power_state)
	{
		mode++; // Increment mode
		if (mode > 2) mode = 0; // Wrap around to 0 if mode exceeds 2
		if (mode < 2)
		ADMUX &= ~(1 << MUX0); // Select ADC0 for modes 0 and 1 (LDR input)
		else
		ADMUX |= (1 << MUX0); // Select ADC1 for mode 2 (potentiometer input)
		switch (mode)
		{
			case 0:
			seven_segment_zero(); // Display '0' on seven-segment display
			usart_transmit_string("Mode changed to 0 (Data Mode)\n"); // Send mode change message
			break;
			case 1:
			seven_segment_one(); // Display '1' on seven-segment display
			usart_transmit_string("Mode changed to 1 (Automatic Mode)\n");
			break;
			case 2:
			seven_segment_two(); // Display '2' on seven-segment display
			usart_transmit_string("Mode changed to 2 (Manual Mode)\n");
			break;
			default:
			break;
		}
	}
}

// Interrupt Service Routine for Timer0 Overflow
ISR(TIMER0_OVF_vect)
{
	interrupt_count++; // Increment interrupt count
	ADCSRA |= (1 << ADSC); // Start ADC Conversion
}

// Interrupt Service Routine for ADC Conversion Complete
ISR(ADC_vect)
{
	adc_sample_level = ADC; // Read latest ADC sample
	new_adc_sample = true; // Set flag indicating a new ADC sample is available
}

// Interrupt Service Routine for USART Data Register Empty
ISR(USART_UDRE_vect) {
	if (usart_tx_head == usart_tx_tail) {
		// Buffer is empty, disable Data Register Empty interrupt
		UCSR0B &= ~(1 << UDRIE0);
		} else {
		// Send next byte from the buffer
		UDR0 = usart_tx_buffer[usart_tx_tail];
		usart_tx_tail = (usart_tx_tail + 1) % USART_TX_BUFFER_SIZE;
	}
}

// Interrupt Service Routine for USART Receive Complete
ISR(USART_RX_vect)
{
	char received_char = UDR0; // Read the received data

	if (power_state) // Only process if the system is powered on
	{
		switch (received_char)
		{
			case '0':
			mode = 0; // Set mode to 0 (Data Mode)
			seven_segment_zero(); // Update display
			usart_transmit_string("Mode changed to 0 (Data Mode)\n");
			break;

			case '1':
			mode = 1; // Set mode to 1 (Automatic Mode)
			seven_segment_one(); // Update display
			usart_transmit_string("Mode changed to 1 (Automatic Mode)\n");
			break;

			case '2':
			mode = 2; // Set mode to 2 (Manual Mode)
			seven_segment_two(); // Update display
			usart_transmit_string("Mode changed to 2 (Manual Mode)\n");
			break;

			default:
			usart_transmit_string("Invalid mode selected. Enter 0, 1, or 2.\n"); // Invalid input
			break;
		}
		
		// Update ADMUX if necessary
		if (mode < 2)
		ADMUX &= ~(1 << MUX0); // Select ADC0 for modes 0 and 1
		else
		ADMUX |= (1 << MUX0); // Select ADC1 for mode 2
	}
}

int main(void)
{
	init_gpio(); // Initialize GPIO pins
	init_timer0(); // Initialize Timer0
	init_timer1(); // Initialize Timer1
	init_adc(); // Initialize ADC
	init_usart(); // Initialize USART
	sei(); // Enable global interrupts
	
	char ldr_level_string[16]; // Buffer to store ADC value as string
	
	while (1)
	{
		if (power_state & new_adc_sample)
		{
			new_adc_sample = false; // Reset new ADC sample flag
			uint8_t adc_scaled = (adc_sample_level >> 2); // Scale ADC value to 8 bits
			uint16_t scaled_contribution = (15 * adc_scaled) + (5 * (adc_scaled >> 3)); // Calculate scaled value
			switch (mode)
			{
				case 0: // Data Mode
				if (interrupt_count > 20) // Approximately every 320 ms
				{
					itoa(adc_sample_level, ldr_level_string, 10); // Convert ADC value to string
					usart_transmit_string("Light Level: ");
					usart_transmit_string(ldr_level_string);
					usart_transmit_string("\n");
					interrupt_count = 0; // Reset interrupt count
				}
				break;
				
				case 1: // Automatic Mode
				OCR1A = 1000 + scaled_contribution; // Update OCR1A for servo position
				break;
				
				case 2: // Manual Mode
				OCR1A = 1000 + scaled_contribution; // Update OCR1A for servo position
				break;
				
				default:
				break;
			}
		}
	}
	
	return 0;
}
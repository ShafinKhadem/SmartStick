/*
 * SmartStick.cpp
 *
 * Created: 8/2/2019 3:47:12 AM
 * Author : ShafinKhadem
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 1000000UL
// below line is needed only for giving variable delays, but it decreases range of max delay to 6.5 second.
#define __DELAY_BACKWARD_COMPATIBLE__
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define D4 eS_PORTB4
#define D5 eS_PORTB5
#define D6 eS_PORTB6
#define D7 eS_PORTB7
#define RS eS_PORTC6
#define EN eS_PORTC7
#include "lcd.h"

#define reset(m, p) m &= ~(1<<(p))
#define set(m, p) m |= (1<<(p))
#define test(m, p) (((m)>>(p))&1)
#define toggle(m, p) m ^= (1<<(p))

class Port {
	public:
	volatile uint8_t *ddr, *port, *pin;
	
	Port(char name) {
		if(name=='A') ddr = &DDRA, port = &PORTA, pin = &PINA;
		else if(name=='B') ddr = &DDRB, port = &PORTB, pin = &PINB;
		else if(name=='C') ddr = &DDRC, port = &PORTC, pin = &PINC;
		else if(name=='D') ddr = &DDRD, port = &PORTD, pin = &PIND;
	}
	
	Port(const Port &rs) : ddr(rs.ddr), port(rs.port), pin(rs.pin) {}
	
	void output(int p) { set(*ddr, p); }
	void input(int p) { reset(*ddr, p); }
	void high(int p) { set(*port, p); }
	void low(int p) { reset(*port, p); }
	void flip(int p) { toggle(*port, p); }
	bool read(int p) { return test(*pin, p); }
	
	void pulse_us(int p, double t) {
		high(p), _delay_us(t), low(p);
	}
	
} pa('A'), pb('B'), pc('C'), pd('D');


class Buzzer
{
	Port port;
	int pin;

	public:
	Buzzer(Port _port, int _pin) : port(_port), pin(_pin) { port.output(pin); }

	void tone(double duration, double frequency)
	{
		double wavelength = (1/frequency)*1000, half_period;
		long cycles = duration/wavelength;
		half_period = wavelength/2;
		
		for (long i = 0; i < cycles; i++) {
			_delay_ms(half_period);
			port.high(pin);
			_delay_ms(half_period);
			port.low(pin);
		}
	}

	void beep(double duration, double onduration, double offduration) {
		long cycles = duration / (onduration + offduration);
		for (long i = 0; i < cycles; i++) {
			port.high(pin);
			_delay_ms(onduration);
			port.low(pin);
			_delay_ms(offduration);
		}
	}
	
} buzzer(pa, 0);


int V_REF = 5, V_MIN = 0;
double analogRead(int ADC_pin) {
	ADMUX = ADC_pin;	// REFS1 REFS0 ADLAR MUX4 MUX3 MUX2 MUX1 MUX0
	set(ADCSRA, ADSC);
	while(test(ADCSRA, ADSC));
	int digital = ADC;
	double analog = digital*(V_REF-V_MIN)/1024.0;
	return analog;
}

char s[100];

#define LDR_threshold 2.5

ISR(ADC_vect) {
	int digital = ADC;
	double analog = digital*(V_REF-V_MIN)/1024.0;
	dtostrf(analog, 4, 2, s);
	puts(s);
	if(analog > LDR_threshold) buzzer.beep(3000, 3000, 0);
}


ISR(INT0_vect) {
	ADMUX = 7;	// REFS1 REFS0 ADLAR MUX4 MUX3 MUX2 MUX1 MUX0
}


void UART_init(long USART_BAUDRATE)
{
	UCSRA = 2;	// double speed mode
	UCSRB = 0b10011000; // RXCIE TXCIE UDRIE RXEN TXEN UCSZ2 RXB8 TXB8
	UCSRC = (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1); /* Use 8-bit character sizes */
	int UBRR = F_CPU / (USART_BAUDRATE << (3 + !test(UCSRA, U2X))) - 1;
	UBRRL = UBRR, UBRRH = UBRR >> 8;
}

int UART_TxChar(char ch, FILE *stream)
{
	while (!test(UCSRA, UDRE)); /*Wait for empty transmit buffer*/
	UDR = ch ;
}

volatile int si = 0;

ISR(USART_RXC_vect) {
	s[si++] = UDR;
	if (test(UCSRA, FE) or test(UCSRA, DOR)) s[si-1] = '~';
	if(s[si-1]=='#') {
		s[si-1] = '\0';
		if (!strcmp(s, "on")) pa.high(0);
		else if(!strcmp(s, "off")) pa.low(0);
		si = 0;
	}
}


volatile int timer1_ovf = 0;

ISR(TIMER1_OVF_vect){
	timer1_ovf++;
}

void timer1_reset() {
	reset(TCCR1B, CS11);
	TCNT1 = 0;
	timer1_ovf = 0;
}

void timer1_init() {
	timer1_reset();
	set(TCCR1B, CS11);
}

class Sonar {
	Port triggerPort;
	int triggerPin;

	public:
	Sonar(Port &_triggerPort, int _triggerPin) : triggerPort(_triggerPort), triggerPin(_triggerPin) { triggerPort.output(triggerPin); }
	
	double getDistance_cm() {
		triggerPort.pulse_us(triggerPin, 15);
		
		/* Calculate width of Echo by Input Capture (ICP) */
		
		set(TCCR1B, ICES1);
		while ((TIFR & (1 << ICF1)) == 0);	/* Wait for rising edge */
		timer1_init();
		reset(TCCR1B, ICES1);
		TIFR = 1<<ICF1;	/* Clear ICP flag (Input Capture flag) */
		while ((TIFR & (1 << ICF1)) == 0);	/* Wait for falling edge */
		long pulse = (ICR1 + (long(timer1_ovf) << 16))*8 / (F_CPU/1000000UL);	// Prescaler was 8
		
		timer1_reset();
		return pulse/58.3;
	}
} sonar(pd, 5);

volatile long timer0_ovf = 0, freq = 1;

ISR(TIMER0_OVF_vect){
	timer0_ovf++;
	if (freq and !(timer0_ovf % freq)) {
		timer0_ovf = 0;
		pa.high(0);
		_delay_ms(150);
		pa.low(0);
	}
}

void timer0_reset() {
	reset(TCCR0, CS02), reset(TCCR0, CS00);
	TCNT0 = 0;
	timer0_ovf = 0;
}

void timer0_init() {
	timer0_reset();
	set(TCCR0, CS02), set(TCCR0, CS00);
}



int main(void)
{
	/* Replace with your application code */
	DDRA = 1, DDRB = 0xFF, DDRC = 0xFF, DDRD = 1<<5;
	set(TIMSK, TOIE1), set(TIMSK, TOIE0);
	UART_init(9600);
	ADCSRA = 0b10101011;	// ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0
	GICR = 1 << INT0;
	set(SFIOR, ADTS1);
	pd.high(2);
	set(MCUCR, ISC01), reset(MCUCR, ISC00);
	sei();
	stdout = fdevopen(UART_TxChar, NULL);
	Lcd4_Init();	// even if lcd is not connected, just having wires of lcd is enough, if this line is not written nothing else works. Don't know why.
	
	while(1)
	{
		// floating point with printf types don't work with default compilation flags, you can use vfprintf -lprintf_flt
		// see stdio.h, stdlib.h, ffs() & avr/sleep.h from https://www.microchip.com/webdoc/AVRLibcReferenceManual
		double cm = sonar.getDistance_cm();
		dtostrf(cm, 7, 2, s);
		puts(s);
		if (cm < 90) freq = ceil(cm / 15), set(TCCR0, CS02), set(TCCR0, CS00);
		else timer0_reset();
		_delay_ms(1000);
	}
}

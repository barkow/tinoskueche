#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/boot.h>

#define TICKSPERSECOND 61
#define POWERDOWNTIMEOUT 10*TICKSPERSECOND

volatile static uint16_t ticks = 0;

volatile int16_t encoderPosition = 0; // Absolute Encoder Position

void USART_Init(void){
	#define BAUD 57600
	#include <util/setbaud.h>
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
	#if USE_2X
		UCSR0A |= (1 << U2X0);
	#else
		UCSR0A &= ~(1 << U2X0);
	#endif
	UCSR0B = (1 << TXEN0);
	UCSR0C = (1 << USBS0)|(3 << UCSZ00);
}

void USART_Transmit( unsigned char data ) {
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

static inline void readEncoder(){
	//Siehe: http://www.dse-faq.elektronik-kompendium.de/dse-faq.htm#F.29
	int8_t encoderTable[4][4] = {{0,1,-1,0},{-1,0,0,1},{1,0,0,-1},{0,-1,1,0}};
	uint8_t encoderNewValue;
	static uint8_t encoderLastValue;

	encoderNewValue = (PIND >> PIND2) & 0x03;
	encoderPosition += encoderTable[encoderLastValue][encoderNewValue];
	encoderLastValue = encoderNewValue;
}

ISR(TIMER0_OVF_vect){
#ifdef DEBUG
	static uint16_t cnt = 0;
#endif
	ticks++;
#ifdef DEBUG
	cnt++;
	if (cnt >= 1*TICKSPERSECOND){
		cnt = 0;
		PORTB ^= 1 << PINB5;
	}
#endif
}

ISR(PCINT2_vect){
	//Die Routine ist absichtlich leer. Der Interrupt wird nur benötigt, um aus dem Sleep aufzuwachen
}

void SPI_MasterInit(void) {
	/* Set MOSI and SCK output, all others input */
	DDRB |= (1 << PINB3) | (1 << PINB5);
	/* Enable SPI, Master, set clock rate fck/2 */
	SPCR = (1 << SPE) | (1 << MSTR) | (0 << SPR0);
	SPSR = (1 << SPI2X);
}

void SPI_MasterTransmit(char cData){
	/* Start transmission */
	SPDR = cData;
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)));
}

void Max7219Send(uint8_t address, uint8_t data){
	//CS low
	PORTB &= ~(1 << PINB2);
#ifndef DEBUG
	SPI_MasterTransmit(address & 0x0f);
	SPI_MasterTransmit(data);
#endif
#ifdef DEBUG
	USART_Transmit(address & 0x0f);
	USART_Transmit(data);
#endif
	//CS high
	PORTB |= (1 << PINB2);
}

void Max7219EnableShutdown(){
	Max7219Send(0x0c, 0);
}

void Max7219DisableShutdown(){
	Max7219Send(0x0c, 1);
}

void Max7219Init(void){
	//CS initialisieren
	DDRB |= (1 << PINB2);

#ifndef DEBUG
	SPI_MasterInit();
#endif
	//BCD Decode Mode
	Max7219Send(0x09, 0xff);
	//Maximum Intensity
	Max7219Send(0x0a, 0x0f);
	Max7219DisableShutdown();
}

void Max7219Display(uint16_t value){
	uint8_t digit;
	for (int i = 0; i < 5; i++){
		digit = value % 10;
		Max7219Send(i + 1, digit);
		value = value / 10;
		if (value == 0){
			break;
		}
	}
}

void powerDown(void){
	//LEDs ausschalten
	Max7219EnableShutdown();

	//Bevor Sleep aktiviert wird, Flankeninterrupt einschalten
	//Wird ansonsten ja nicht benötigt und erzeugt nur unnötig Prozessorlast
	//Pin Change Interrupt auf D2 aktivieren
	PCMSK2 |= (1 << PCINT18);
	PCICR |= (1 << PCIE2);

	//Sleep aktivieren
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_mode();

	//Hier springt der Prozessor nach dem Aufwachen wieder hin
	//Benötigte Interrupts für Sleep wieder deaktivieren
	//Pin Change Interrupt auf D2 deaktivieren
	PCMSK2 &= ~(1 << PCINT18);
	PCICR &= ~(1 << PCIE2);

	//LEDs wieder einschalten
	Max7219DisableShutdown();
}

int main(void){
	static int8_t encoderPositionOld = 0;
	static uint16_t lastUserAction = 0;

	//Timer initialisieren
	TCCR0B = 0x5 << CS00;			//divide by 64 --> 16MHz / 256 (8bit overflow) / 1024 ~ 61 interrupts pro s
	TIMSK0 = 1 << TOIE0;			//enable timer interrupt

#ifdef DEBUG
	//UART initialisieren
	USART_Init();
#endif

	//7-Segment Anzeige initialisieren
	Max7219Init();

	//Encoder Eingänge initialisieren
	//Pin D2 und D3 als Input mit Pullup
	DDRD &= ~((1 << PIND2) || (1 << PIND3));
	PORTD |= ((1 << PIND2) || (1 << PIND3));

#ifdef DEBUG
	USART_Transmit('A');
	USART_Transmit('B');
	USART_Transmit('C');
	//LED
	DDRB |= 1 << PINB5;
	PORTB &= ~(1 << PINB5);
#endif

	//Interrupts aktivieren
	sei();

	while(1){
		readEncoder();
		//Prüfen, ob sich Encoderposition verändert hat
		if (encoderPositionOld != encoderPosition){
			encoderPositionOld = encoderPosition;
			//Zeitpunkt des letzten User Inputs festhalten
			lastUserAction = ticks;

			//Neuen Wert ausgeben auf 7-Segment Anzeige ausgeben
			Max7219Display(encoderPosition);
		}

		//Prüfen, ob Powerdown Zeit erreicht wurde
		if ((ticks - lastUserAction) > POWERDOWNTIMEOUT){
			powerDown();
			//Nach dem Aufwachen die Useraktivität zurücksetzen
			lastUserAction = ticks;
		}
	}
}

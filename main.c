#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define PHASE_A	(PINC & 1<<PINC0)	// PINC.0
#define PHASE_B (PINC & 1<<PINC1)	// PINC.1

#define POWERDOWNTIMEOUT 100

volatile int8_t	enc_delta;		// -128 ... 127
volatile uint16_t ticks;

void readEncoder();

ISR(TIMER0_OVF_vect){
	readEncoder();
	ticks++;
	PORTB ^= 1 << PINB5;
}

//Siehe: https://www.mikrocontroller.net/topic/drehgeber-auslesen
inline void readEncoder(){
	static char enc_last = 0x01;
	char i = 0;

	if( PHASE_A ){
		i = 1;
	}

	if( PHASE_B ){
		i ^= 3;				// convert gray to binary
	}

	i -= enc_last;			// difference new - last

	if( i & 1 ){				// bit 0 = value (1)
		enc_last += i;			// store new as next last
		enc_delta += (i & 2) - 1;		// bit 1 = direction (+/-)
	}
}

void powerDown(){
	//TODO: LEDs ausschalten

	//TODO: Bevor Sleep aktiviert wird, Flankeninterrupt einschalten
	//Wird ansonsten ja nicht benötigt und erzeugt nur unnötig Prozessorlast
	//Wenn Flankeninterrupt nicht geht, Level Interrupt einschalten. Dabei den aktuellen Wert an den Encodereingängen berücksichtigen

	//Sleep aktivieren
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_mode();

	//Hier springt der Prozessor nach dem Aufwachen wieder hin
	//TODO: Benötigte Interrupts für Sleep wieder deaktivieren
}

int main(void){
	static int8_t encoderPositionOld = 0;
	static uint16_t lastUserAction = 0;
	//Timer initialisieren
	TCCR0B = 0x5 << CS00;			//divide by 8 * 256
	TIMSK0 = 1 << TOIE0;			//enable timer interrupt

	//TODO: SPI initialisieren

	//TODO: 7-Segment Anzeige initialisieren

	//TODO: Wakeup initialisieren: Bei Flanke auf Encoder Eingang aufwachen
	//Geht Flanke überhaupt oder kann nur ein Level Interrupt den Controller aufwecken

	//Interrupts aktivieren
	sei();
	DDRB |= 1 << PINB5;
	PORTB &= ~(1 << PINB5);

	while(1){
		//PORTB = enc_delta;
		//Prüfen, ob sich Encoderposition verändert har
		if (encoderPositionOld != enc_delta){
			//Zeitpunkt des letzten User Inputs festhalten
			lastUserAction = ticks;

			//TODO: Neuen Wert ausgeben per SPI ausgeben

		}

		//Prüfen, ob Powerdown Zeit erreich wurde
		if ((ticks - lastUserAction) > POWERDOWNTIMEOUT){
			//powerDown();
		}
	}
}

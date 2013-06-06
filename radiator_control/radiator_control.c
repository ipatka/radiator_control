/*
 *  Isaac Patka
 *  CLUE Summer 2013
 *  Radiator Labs
 */

#define F_CPU 8000000UL
#include "avr/io.h"
#include "util/delay.h"
#include <avr/wdt.h>

//PORTC
#define LED_PIN 0b100000
#define ON_SWITCH 0b10000
#define RESET_SWITCH 0b1000

//PORTB
#define ENCLOSURE_TEMP 0b10000000
#define ENCLOSURE_TEMP_bit 7
#define SURFACE_TEMP 0b10
#define SURFACE_TEMP_bit 1
#define ROOM_TEMP 0b1
#define ROOM_TEMP_bit 0


//PORTD
#define AIRFLOW_A 0b100000
#define AIRFLOW_B 0b1000000
#define AIRFLOW_C 0b10000000

//SPI - PORTB
#define MOSI 0b1000;//PB3
#define MOSI_bit 3
#define MISO 0b10000;//PB4
#define MISO_bit 4
#define SS 0b100;//PB2
#define SS_bit 2
#define SCK 0b100000;//PB5
#define SCK_bit 5

#define RESET 255


/*              *
 *  Prototypes  *
 *              */

//Set up UART preferences
void UART_INIT();

//Send some data
void UART_Transmit(unsigned char data);

//Receive some data
unsigned char UART_Receive();

//Initialize the ports
void Init_Ports();


//Debugging led
void blink_led();

unsigned char get_desired_temp();

void run_radiator_until_desired_temp(unsigned char desired_temp);

void Reset_avr();

unsigned char get_current_temp(unsigned char t_select);

unsigned char get_current_pressure(unsigned char airflow_select);

unsigned char SPI_read();

void Init_timer();

void set_bit_value(volatile uint8_t * byte, unsigned char bit, unsigned char value);

/*          *
 *  Global  *
 *Variables *
 *          */
 
 int transmit_timer_count = 0;

unsigned char report_temp = 0;
unsigned char report_airflow = 0;

unsigned char transmit_freq = 0;



int main(void)
{

    
    UART_INIT();
    Init_Ports();
    MCUSR = 0;
    wdt_disable();
    

    
    unsigned char received_byte = 0;
    unsigned char instruction_byte;
    unsigned char i = 0;
    
    unsigned char desired_temp;
    
    while (1)
    {
        //labview will send desired temperature, report_temp and report_airflow
        
        desired_temp = UART_Receive();
        report_temp = UART_Receive();   //if 1, report temp
        report_airflow = UART_Receive();    //if 1, report airflow
        transmit_freq = UART_Receive();     //how long between data points
        
        Init_timer();
        
        instruction_byte = UART_Receive();  //pause while there is nothing in the receive buffer and transmit every 5 seconds based on timer interrupts
        
       
         if (instruction_byte == RESET)
         {   
         blink_led();
         blink_led();
         cli();  //DISABLE INTERRUPTS
         }
         
        report_temp = UART_Receive();   //if 1, report temp
        report_airflow = UART_Receive();    //if 1, report airflow
        
        Init_timer();
        
        instruction_byte = UART_Receive();  //pause while there is nothing in the receive buffer and transmit every 5 seconds based on timer interrupts
      
    
    }
 

    return 0;
}

void UART_INIT()
{
    unsigned int UBBR_value;
    unsigned int baud = 19200;
    
    UCSR0A &= ~(_BV(U2X0));
    
    UBBR_value =    25;// ( ( 8000000 ) / (16 * baud)) - 1;    //unsigned int so it will round down to 25 @ baud rate of 19200. According to the data sheet (page 192) the error is 0.2%
    UBRR0H = (unsigned char) (UBBR_value >> 8);
    UBRR0L = (unsigned char) UBBR_value;
    
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);     //8-bit data length
    
    //No parity set at this time
    
    UCSR0C &= ~(1 << USBS0);  //Sets 1 stop bit
    
    UCSR0B = (1 << RXEN0)  | (1 << TXEN0);   //Enable the receiver and transmitter
    
    
}

void UART_Transmit(unsigned char data)
{
    //Pause until transmitter is ready
    while ( !(UCSR0A & (1 << UDRE0)) );
    
    //Transmit the data
    UDR0 = data;
    
}

unsigned char UART_Receive()
{
    //Pause until RXC is nonzero
    while ( !(UCSR0A & (1 << RXC0)) );
    
    //Get data
    return UDR0;
    
}


void Init_Ports()
{
    
    
    DDRC |= (1 << PC5) | (1 << PC4);
    DDRC &= ~(1 << PC3);
    /*
     *PC5 is LED_PIN
     *PC4 is ON_SWITCH
     *PC3 is RESES_SWITCH
     */
     
    DDRB &= ~((1 << PB7) | (1 << PB1) | (1 << PB0));
    /*
     *PB7 is ENCLOSURE_TEMP
     *PB1 is SURFACE_TEMP
     *PB0 is ROOM_TEMP
     */
     
    DDRD &= ~((1 << PD7) | (1 << PD6) | (1 << PD5);
    /*
     *PD5 is AIRFLOW_A
     *PD6 is AIRFLOW_B
     *PD7 is AIRFLOW_C
     */
 

}
   


void blink_led()
{
  PORTC |= LED_PIN;
  _delay_ms(125);
  PORTC &= ~(LED_PIN);
  _delay_ms(125);
}


unsigned char get_desired_temp()
{
    return UART_Receive();
}

void run_radiator_until_desired_temp(unsigned char desired_temp)
{
    unsigned char temp_enclosure;
    unsigned char temp_surface;
    unsigned char temp_room;
    
    unsigned char airflow_a;
    unsigned char airflow_b;
    unsigned char airflow_c;

    PORTC |= ON_SWITCH; //turn on radiator

    Init_timer();
    while (get_current_temp(ENCLOSURE_TEMP) < desired_temp);    //report temp until desired_temp is reached
    cli();  //disable timer interrupts
    
    
    PORTC &= ~ON_SWITCH;    //turn off radiator
}

unsigned char get_current_temp(unsigned char temp_select)
{

    int32_t raw_temp;
    raw_temp = SPI_read(temp_select);
    raw_temp >>= 18;    //get rid of internal temp/ fault bits
       
    double farenheit = raw_temp;
    farenheit *= 0.45;
    farenheit += 32;
    
    return farenheit;
    
}

unsigned char get_current_pressure(unsigned char airflow_select)
{

}

void SPI_INIT()
{
    //MISO input
    DDRB &= ~(1 << PB4);
    //SCK, SS output
    DDRB |= (1 << PB3) | (1 << PB5) | (1 << PB2);
    
    /*SPCR
     *SPIE 0 - so no SPI interrupt is executed upon transfer completion
     *SPE  1 - to enable SPI
     *DORD 0 - MSB first
     *MSTR 1 - Master select
     *CPOL 0 - SCK low when idle
     *CPHA 1 - read on trailing edge
     */ 
   
    
    SPCR |= (1 << SPE) | (1 << CPHA) | (1 << MSTR);
    SPCR &= ~((1 << SPIE ) | (1 << DORD) | (1 << CPOL));
    
    //initialize chip selects to high state, pulled low to read
    set_bit_value(PORTB,ENCLOSURE_TEMP_bit, 1);
    set_bit_value(PORTB,SURFACE_TEMP_bit, 1);
    set_bit_value(PORTB,ROOM_TEMP_bit, 1);
}

uint32_t SPI_read(unsigned char chip_select);
{
  int i;
  uint32_t d = 0;
  
  set_bit_value(PORTB, SCK_bit, 0);
  _delay_ms(1);
  set_bit_value(PORTB,chip_select, 0);
  _delay_ms(1);
  
  for (i = 31; i >= 0; i--)
  {
    set_bit_value(PORTB, SCK_bit, 0);
    _delay_ms(1);
    d <<= 1;
    
    if ((PORTB >> MISO_bit) & 0b1)
    d |= 1;  
  
    set_bit_value(PORTB, SCK_bit,1);
    _delay_ms(1);
  }
  
  set_bit_value(PORTB,chip_select, 1);
  
  return d;
}

void Init_timer()
{
    TCCR1B |= (1 << WGM12); //configure timer 1 for CTC mode
    
    TIMSK1 |= (1 << OCIE1A);  //Enable CTC interrupt
    
    sei();  //enable Global interrupts 
    
    OCR1A = 31249;          //Set CTC compare value to 1Hz at 1MHz AVR clock, with prescaler of 256
    
    TCCR1B |= (1 << CS12);  //Start timer @ Fcpu/256
}

ISR(TIMER1_COMPA_vect)  //Execute this upon interrupt
{
    transmit_timer_count++;
    if (transmit_timer_count >=transmit_freq)
    {
        if (report_temp == 1)
        {
        UART_Transmit(get_current_temp(ENCLOSURE_TEMP_bit));
        UART_Transmit(get_current_temp(SURFACE_TEMP_bit));
        UART_Transmit(get_current_temp(ROOM_TEMP_bit));
   
        blink_led();
        }
        
        if (report_airflow == 1)
        {
        UART_Transmit(get_current_pressure(AIRFLOW_A));
        UART_Transmit(get_current_pressure(AIRLFLOW_B));
        UART_Transmit(get_current_pressure(AIRFLOW_C));
        }
        transmit_timer_count = 0;
    }
    
}

void Reset_avr()
{
    wdt_enable(WDTO_30MS);
    while(1);
      

}

void set_bit_value(volatile uint8_t * byte, unsigned char bit, unsigned char value)
{
   
    if (value == 1)
        *byte |= 1 << bit;
                        
        else if (value == 0)
            *byte &= ~(1 << bit);
                        
}

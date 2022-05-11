/*
 * Test2.crm
 *
 * Created: 2022-05-01 오후 17:04:32
 * Author : Flanon
 */
/////
//sys
#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU / 16 / BAUD - 1

//sonar
#define PULSE_WIDTH 10//sonar trig pulse width(us)
#define SAMPLING_INTERVAL 100//sonar sampling interval(ms)
#define SPEED_OF_SOUND 340.29//speed of sound(m/s)

//led
#define RED 0x01
#define YELLOW 0x02
#define GREEN 0x04

//pinout
#define SONAR_TRIG_PIN PA0
#define SONAR_ECHO_PIN INT1


//lib
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//ext lib
#include "millis.h"
#include "lcd.h"

//Sonar
int tof = 0;//time of flight(10us)
uint8_t isrecv = 0;//수신완료 여부
uint8_t ishigh = 0;//Echo 핀 상태

//util
char buffer[10];
char clearbuffer[16] = "                ";

//light
int first = 45;
int second = 5;
int third = 15;
unsigned long prevmillis = 0;
int display_time = 500;


//UART----------------------

//initialize USART ubrr, rx, tx, character size
void USART_Init(unsigned int ubrr)
{
    UBRR0H = (unsigned char)(ubrr >> 8);//set baud rate
    UBRR0L = (unsigned char)ubrr;//set baud rate
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);//receive and transmitter enable
    UCSR0C = (3 << UCSZ0);//8-bit character size
}

//function to write character to output
void USART_TX(unsigned char data)
{
    while (!(UCSR0A & (1 << UDRE0)));//if the data register is full, wait
    UDR0 = data;//write data to UDR0 register
}

//function to output string
void USART_TX_String(const char *text)
{
    while (*text != 0)//until the end of the string is reached
        USART_TX(*text++);//print each character in a string
}

//Sonar----------------------

void Sonar_Init()
{
    //외부 인터럽트 초기화
    // Source : INT1(PD1)
    EICRA = (1 << ISC11) | (1 << ISC10);//초기상태에서는 rising edge 검사가 이루어져야 함
    EIMSK = 1 << SONAR_ECHO_PIN;//INT1 EXT interrupt enable
    EIFR = 1 << SONAR_ECHO_PIN;//강제로 INTF1 EXT interrupt flag set

    //timer0 초기화(10us단위로 tovf 인터럽트 형성 필요)
    //interrupt cycle: 20/(F_CPU/prescaler) = 0.00001s -> 10us
    TCCR0 = 1 << CS01; // 1/8 prescaler
    TCNT0 = 0xFF - 20;
    TIMSK |= 1 << TOIE0; // Timer 0 overflow interrupt enable
}

ISR(TIMER0_OVF_vect)
{
    //cli();//clear golbal interrrupt flag
    TCNT0 = 0xFF - 20;
    tof++;
    //sei();//set golbal interrrupt flag
}

ISR(INT1_vect)//EXT intterpupt, rising 혹은 falling edge 검사
{
    //cli();//clear golbal interrrupt flag
	//정확도를 높이기 위해서는 ISR호출 시 timer관리가 최대한 빠르게 이루어져야 함
    if (!ishigh) {//rising edge가 검출되고 파형이 L상태인 경우
		//timer를 추적하기 위해 TOVF interrupt enable 및 timer 초기화
        TIMSK |= 1 << TOIE0; // Timer 0 overflow interrupt enable
        TCNT0 = 0xFF - 20;//타이머 초기화
        tof = 0;//tof 초기화
		
        EICRA = 1 << ISC11; //falling edge 검사로 전환
        ishigh = 1;//ISR이 수행된 뒤 파형은 H 상태임
    } else if (ishigh) {//falling edge가 검출되고 파형이 H상태인 경우
        //불필요한 ISR수행을 막기 위해 TOVF interrupt disable
        TIMSK &= ~(1 << TOIE0); // Timer 0 overflow interrupt disable
		isrecv = 1;//tof 계산 완료

        EICRA = (1 << ISC11) | (1 << ISC10);//rising edge 검사로 전환
        ishigh = 0;//ISR이 수행된 뒤 파형은 L 상태임
    }
    //sei();//set golbal interrrupt flag
}

void Sonar_Get_Tof()
{
    //초음파 전송
    PORTA |= 1 << SONAR_TRIG_PIN;
    _delay_us(PULSE_WIDTH);
    PORTA &= ~(1 << SONAR_TRIG_PIN);
	
	//초음파 수신대기
    while (!isrecv) {}//EXT falling edge interrupt가 감지될때까지 대기
    isrecv = 0;// tof가 계산된 경우 다음 tof 계산을 위해 isrecv 초기화
}

float Sonar_Get_Speed()//return speed in cm/s
{
    //정확도를 높이기 위해서는 두 sampling사이에 최소한의 작업만이 이루어져야 함
    Sonar_Get_Tof ();
    int t1 = tof;
    _delay_ms (SAMPLING_INTERVAL);
    Sonar_Get_Tof ();
    int t2 = tof;

    USART_TX_String("D1:");
    itoa((int)(t1 * 0.17), buffer, 10);
    USART_TX_String(buffer);
    USART_TX_String("cm\r\n");

    USART_TX_String("D2:");
    itoa((int)(t2 * 0.17), buffer, 10);
    USART_TX_String(buffer);
    USART_TX_String("cm\r\n");
	
	//예외처리 Timeout
	if(t1 > 3800 || t2 > 3800)	return 0;
	
	// 예외처리 (차 뒤로 갈 때, 차 속도가 150km이상일 때)
	if (t1 - t2 < 0 || t1 - t2 > 2451)	return 0;

    return ((SPEED_OF_SOUND / 2.0 / 1000.0) * (t1 - t2)) / (SAMPLING_INTERVAL / 1000.0);
    //((음속 / 음파 이동 횟수 / 단위보정(1m->100cm, 10us) * 시간차) / (샘플링 간격 / 단위보정(1s->1ms))
}

// LCD
void Speed_LCD_Alart(int spd){// spd 값에 따라 속도와 과속유무 LCD에 띄움
	
	// 첫번째 줄 clear
	LCD_setcursor(0,0);
	LCD_wString(clearbuffer);
	// 문자열 출력
	LCD_setcursor(0,0);
	LCD_wString("Speed : ");
	
	// spd 를 저장할 a 문자열
	char a[5];
	itoa(spd, a, 10);
	LCD_setcursor(0,8);
	LCD_wString(a);	// 속도 출력
	LCD_setcursor(0,12);
	LCD_wString("cm/s");	// cm/s 출력
	
	if(spd>75){	// 과속
		prevmillis = millis();
	}
	
	if((millis() - prevmillis) < display_time){	//500ms동안 2번째 줄에 경고 문자열 출력
		LCD_setcursor(1,0);
		LCD_wString("Too fast!!");
	}
	else{	//500ms 이후 2번째 줄 clear
		LCD_setcursor(1,0);
		LCD_wString(clearbuffer);
	}
}

// light system
void Main_Traffic_light(){	// 자동차 기준 신호등
	//나중에 보행자 신호등 같이 물릴거임
	int period = first + second + third;
	int total = millis() / 1000 % period;
	
	
	if(total > first + second)	PORTF = RED;
	else if(total > first)	PORTF = YELLOW;
	else	PORTF = GREEN;
}


int main(void)
{
	//initializing
    USART_Init(MYUBRR);
    init_millis(F_CPU);
    Sonar_Init();
	
    sei();//golbal interrrupt enable

    DDRA |= 1 << SONAR_TRIG_PIN;
	DDRB = 0xFF;	// LCD data
	DDRC = 0xFF;	// LCD control
	DDRF = 0xFF;	// traffic light
	
	LCD_Init();	// use port b and c

    /* Replace with your application code */
    while (1) {
		
		// LCD를 통합한 코드
		//속도를 읽어와 인쇄
		int spd = Sonar_Get_Speed();//Sonar_Get_Tof()함수 수행시간 약 500us ~ 3ms로 측정됨
		itoa(spd, buffer, 10);
		Speed_LCD_Alart(spd);
		
		// USART
		/*
		USART_TX_String("Speed:");
		USART_TX_String(buffer);
		USART_TX_String("cm/s\r\n");

		if (spd > 75) {//100cm/s의 속도를 초과하는 경우
			USART_TX_String("Speed limit has reached!!!\r\n");
		}
		USART_TX_String("\r\n");
		*/
		
		//
		// millis() 에 따라 led 점멸
		// use portF
		Main_Traffic_light();
    }
}

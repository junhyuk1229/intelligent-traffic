/*
 * Test2.crm
 *
 * Created: 2022-05-01 오후 17:04:32
 * Author : Flanon
 */



//test (all in cm)
#define XDISTANCE 10
#define YDISTANCE 10
#define PI 3.14
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//sys
#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU / 16 / BAUD - 1

//sonar
#define PULSE_WIDTH 10//sonar trig pulse width(us)
#define SAMPLING_INTERVAL 100//sonar sampling interval(ms)
#define SPEED_OF_SOUND 340.29//speed of sound(m/s)

//pinout
#define SONAR_TRIG_PIN PA0
#define SONAR_ECHO_PIN INT1
#define HUMAN_COUNT_PIN INT2

#define VEHICLE_RED_LED PF0
#define VEHICLE_YELLOW_LED PF1
#define VEHICLE_GREEN_LED PF2

#define PEDESTRIAN_RED_LED PF3
#define PEDESTRIAN_GREEN_LED PF4

#define BUZZER_PIN PF5
#define WARNING_LED PF6

#define SERVO_PIN PE5

//lib
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>


//test
#include <math.h>
////////////////////////////////////////////////////////////////////////////////////////////////

//ext lib
#include "millis.h"
#include "lcd.h"

//Sonar
int tof = 0;//time of flight(10us)
bool isRecv = false;//수신완료 여부
bool isHigh = false;//Echo 핀 상태

//overspeed
#define OVERSPEED_LIMIT 75
unsigned long overspeedDisplayStart = 0;
#define OVERSPEED_DISPLAY_TIME 500

//light
int carRedTime = 10000;
#define CAR_YELLOW_TIME 5000
#define MIN_GREEN_TIME 9000	// 자동차 파란불 최소 시간 제한
#define MAX_GREEN_TIME 120000	// 자동차 파란불 최대 시간 제한
int fluidGreenTimeValue = 10000;	// 유동적으로 바꿀 파란불 시간 (second)

//count
double	adjustTime = 10.0 * 1000.0;	// 10 second (임시)
int humanCount = 0;	// 사람 수
int carCount = 0;	// 자동차 수
int hours = 0;		// 시간
int prevDayTime[24];		// 다음에 사용될 시간이 저장되어있는 array
unsigned long countPrevmillis = 0;

//warning
bool carWarningFlag = false;	// 차량 경고 플래그
unsigned long carPrevmillis = 0;	// 차량 경고용
unsigned long humanPrevmillis = 0;	// 보행자 경고용

//servo
#define PWM_FREQ 50//define the PWM frequency input to the servo motor(50Hz)
int servoPos = 0;//servo positioin(in dgree)
int prevPos = 0;

//util
char buffer[10];
#define LCD_CLEAR_BUFFER "                "
unsigned long showPrevmillis = 0;

#define DEBOUNCE_TIME 30
unsigned long prevTrgtime = 0;

//pedetr fluid
int redAlphaTime = 0;
bool buttonFlag = false;
bool greenCondition = false;

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
    //port 설정
    DDRA |= 1 << SONAR_TRIG_PIN;

    //외부 인터럽트 초기화
    // Source : INT1(PD1)
    EICRA |= (1 << ISC11) | (1 << ISC10);//초기상태에서는 rising edge 검사가 이루어져야 함
    EIMSK |= 1 << SONAR_ECHO_PIN;//INT1 EXT interrupt enable
    EIFR |= 1 << SONAR_ECHO_PIN;//강제로 INTF1 EXT interrupt flag set

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
    if (!isHigh) {//rising edge가 검출되고 파형이 L상태인 경우
        //timer를 추적하기 위해 TOVF interrupt enable 및 timer 초기화
        TIMSK |= 1 << TOIE0; // Timer 0 overflow interrupt enable
        TCNT0 = 0xFF - 20;//타이머 초기화
        tof = 0;//tof 초기화

        EICRA &= ~(1 << ISC10); //falling edge 검사로 전환
        isHigh = true;//ISR이 수행된 뒤 파형은 H 상태임
    } else if (isHigh) {//falling edge가 검출되고 파형이 H상태인 경우
        //불필요한 ISR수행을 막기 위해 TOVF interrupt disable
        TIMSK &= ~(1 << TOIE0); // Timer 0 overflow interrupt disable
        isRecv = true;//tof 계산 완료

        EICRA |= (1 << ISC11) | (1 << ISC10);//rising edge 검사로 전환
        isHigh = false;//ISR이 수행된 뒤 파형은 L 상태임
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
    while (!isRecv) {}//EXT falling edge interrupt가 감지될때까지 대기
    isRecv = false;// tof가 계산된 경우 다음 tof 계산을 위해 isrecv 초기화
}

float Sonar_Get_Speed()//return speed in cm/s
{
    //정확도를 높이기 위해서는 두 sampling사이에 최소한의 작업만이 이루어져야 함
    Sonar_Get_Tof ();
    int t1 = tof;
    _delay_ms (SAMPLING_INTERVAL);
    Sonar_Get_Tof ();
    int t2 = tof;

    /*
    USART_TX_String("D1:");
    itoa((int)(t1 * 0.17), buffer, 10);
    USART_TX_String(buffer);
    USART_TX_String("cm\r\n");

    USART_TX_String("D2:");
    itoa((int)(t2 * 0.17), buffer, 10);
    USART_TX_String(buffer);
    USART_TX_String("cm\r\n");
    */


	
    //test
    if (t2 < 3800)	{
        double radAngle = atan(YDISTANCE / (XDISTANCE - (0.34 * t2) / 2));
	int degAngle = round(radAngle * 180 / PI);
        if (degAngle < 0)	{
	    degAngle = 180 + degAngle;
		}
	servoPos = degAngle;
	}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
	

    //예외처리 Timeout
    if (t1 > 3800 || t2 > 3800)	return 0;

    // 예외처리 (차 뒤로 갈 때, 차 속도가 150km이상일 때)
    if (t1 - t2 < 0 || t1 - t2 > 2451)	return 0;

    return ((SPEED_OF_SOUND / 2.0 / 1000.0) * (t1 - t2)) / (SAMPLING_INTERVAL / 1000.0);
    //((음속 / 음파 이동 횟수 / 단위보정(1m->100cm, 10us) * 시간차) / (샘플링 간격 / 단위보정(1s->1ms))
}

//Analog comparator & Ext Interrupt----------------------

void Interrupt_Init()
{
    //comp
    SFIOR &= ~(1 << ACME);//analog comp multiplexer disable
    //ADCSRA &= ~(1 << ADEN);//disable ADC
    ACSR = (1 << ACIS1) | (1 << ACIS0);//Comparator Interrupt on Rising Output Edge
    ACSR |= (1 << ACIE);// Analog Comparator Interrupt Enable

    /*
    //Ext int
    EIMSK |= (1 << VEHICLE_COUNT_PIN);
    // falling edge
    // use INT3
    EICRA |= (1 << ISC31);
    */

    //Ext int
    EIMSK |= (1 << HUMAN_COUNT_PIN);
    // falling edge
    // use INT2
    EICRA |= (1 << ISC21);

    for (int i = 0; i < 24; i++)	{
        prevDayTime[i] = MIN_GREEN_TIME;
    }
}

//comp
ISR(ANALOG_COMP_vect)
{
    if (carWarningFlag)
        carPrevmillis = millis();	// 차량 경고
    if (millis() - prevTrgtime > DEBOUNCE_TIME) {//노이즈 검사
        prevTrgtime = millis();
        carCount++;
    }
}

//ext
ISR(INT2_vect)
{
    if (!carWarningFlag)
        humanPrevmillis = millis();	// 보행자 경고
    if (millis() - prevTrgtime > DEBOUNCE_TIME) {//디바운스 검사
        prevTrgtime = millis();
        humanCount++;
    }
}

//노이즈 문제를 줄이기 위해 comp 입력 및 ext interrupt입력에 병렬로 커패시터 연결할 필요가 있음

//Fluid_Traffic----------------------

// 교통량에 따라 신호를 유동적으로 변경
void Fluid_Traffic_Light_Adjust()
{
    // adjustTime 주기로 한 번씩 trigger
    if (millis() - countPrevmillis >= adjustTime) {
        countPrevmillis = millis();

        // 밑에 알고리즘 구성
        fluidGreenTimeValue = fluidGreenTimeValue + carCount - humanCount;

        if (fluidGreenTimeValue < MIN_GREEN_TIME)	fluidGreenTimeValue = MIN_GREEN_TIME;
        if (fluidGreenTimeValue > MAX_GREEN_TIME)	fluidGreenTimeValue = MAX_GREEN_TIME;

        // 변수 초기화
        carCount = 0, humanCount = 0;

        // 결과 시간 저장 + 다음 시간 불러오기
        prevDayTime[hours] = fluidGreenTimeValue;
        hours = (hours + 1) % 24;
        fluidGreenTimeValue = prevDayTime[hours];
    }
}

//LCD Alart----------------------

void Speed_LCD_Alart(int spd) // spd 값에 따라 속도와 과속유무 LCD에 띄움
{

    // 첫번째 줄 clear
    LCD_setcursor(0, 0);
    LCD_wString(LCD_CLEAR_BUFFER);
    // 문자열 출력
    LCD_setcursor(0, 0);
    LCD_wString("Speed : ");

    // spd 를 저장할 a 문자열
    char a[5];
    itoa(spd, a, 10);
    LCD_setcursor(0, 8);
    LCD_wString(a);	// 속도 출력
    LCD_setcursor(0, 12);
    LCD_wString("cm/s");	// cm/s 출력

    if (spd > OVERSPEED_LIMIT) {	// 과속
        overspeedDisplayStart = millis();
    }

    if ((millis() - overspeedDisplayStart) < OVERSPEED_DISPLAY_TIME) {	//500ms동안 2번째 줄에 경고 문자열 출력
        LCD_setcursor(1, 0);
        LCD_wString("Too fast!!");
    } else {	//500ms 이후 2번째 줄 clear
        LCD_setcursor(1, 0);
        LCD_wString(LCD_CLEAR_BUFFER);
    }
}

//light system----------------------

void Traffic_Light_Cycle() 	// 자동차 기준 신호등
{
    //나중에 보행자 신호등 같이 물릴거임
    //모든 시간은 ms 기준
    int totalCycleTime = fluidGreenTimeValue + CAR_YELLOW_TIME + carRedTime + redAlphaTime;
    int currTime = millis() % totalCycleTime;

    if (currTime > fluidGreenTimeValue + CAR_YELLOW_TIME)	{//차량 적색
        PORTF |= 1 << VEHICLE_RED_LED;
        PORTF &= ~(1 << VEHICLE_YELLOW_LED | 1 << VEHICLE_GREEN_LED);
        carWarningFlag = true;
        //보행자 청색
        PORTF |= 1 << PEDESTRIAN_GREEN_LED;
        PORTF &= ~(1 << PEDESTRIAN_RED_LED);
		//servo 위치 변경
    } else if (currTime > fluidGreenTimeValue)	{//차량 황색
        PORTF |= 1 << VEHICLE_YELLOW_LED;
        PORTF &= ~(1 << VEHICLE_GREEN_LED | 1 << VEHICLE_RED_LED);
		//보행자 적색
		PORTF |= 1 << PEDESTRIAN_RED_LED;
		PORTF &= ~(1 << PEDESTRIAN_GREEN_LED);
    } else	{//차량 청색
        PORTF |= 1 << VEHICLE_GREEN_LED;
        PORTF &= ~(1 << VEHICLE_RED_LED | 1 << VEHICLE_YELLOW_LED);
        carWarningFlag = false;
        //보행자 적색
        PORTF |= 1 << PEDESTRIAN_RED_LED;
        PORTF &= ~(1 << PEDESTRIAN_GREEN_LED);
		//위치 변경
    }
}

//Servo----------------------

//PWM발생 함수, PWM 주파수 및 duty비 설정
//16Bit Timer/Counter3를 사용(Timer/Counter1은 millis()함수에서 사용하고, Timer/Counter2은 ICRn이 없기 때문에 주파수 설정에 제약이 있어 사용 불가)
//WGM(Waveform Generation Mode): 14
//Fast PWM, TOP:ICR3, Update OCR3C:BOTTOM, TOV3 Flag set: TOP
void PWM(unsigned long freq, float duty)
{
    TCCR3B &= 0b11111000;//stop timer for update

    unsigned long period = (F_CPU / 8 / freq);//calculate the waveform period

    ICR3 = period;//assign calculateed period
    TCNT3 = period - 1;//initialize TCNT3 value to prevent jitter
    OCR3C = period * duty / 100UL;//calculate duty ratio

    TCCR3A |= (1 << COM3C1) | (1 << WGM31); //use OC3C(PE5) as output, non-inverting, Fast PWM mode
    TCCR3B |= (1 << WGM33) | (1 << WGM32) | (1 << CS31); //fast PWM mode, prescaler division ratio 8
    DDRE |= 1 << SERVO_PIN; //declare PWM output
}

//Util & debug----------------------

// 사람과 자동차 count, 유동적으로 바뀐 시간을 보여주는 함수
void Print_Overview()
{
    // 1초마다 한 번씩 출력
    if (millis() - showPrevmillis >= 1000) {
        showPrevmillis = millis();
        USART_TX_String("People count : ");
        itoa(humanCount, buffer, 10);
        USART_TX_String(buffer);
        USART_TX_String("\r\n");
        USART_TX_String("Car count : ");
        itoa(carCount, buffer, 10);
        USART_TX_String(buffer);
        USART_TX_String("\r\n");
        USART_TX_String("fluid time value : ");
        itoa(fluidGreenTimeValue, buffer, 10);
        USART_TX_String(buffer);
        USART_TX_String("\r\n\r\n");
    }
}

//map function, perform range mapping
float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//main----------------------

int main(void)
{
    //initializing
    USART_Init(MYUBRR);
    init_millis(F_CPU);
    Sonar_Init();
    Interrupt_Init();

    sei();//golbal interrrupt enable

    //GPIO
    DDRB = 0xFF;	// LCD data
    DDRC = 0xFF;	// LCD control
    DDRF = 0xFF;	// traffic light

    LCD_Init();	// use port b and c


    /* Replace with your application code */
    while (1) {

        // LCD를 통합한 코드
        //속도를 읽어와 인쇄
        int spd = Sonar_Get_Speed();//Sonar_Get_Tof()함수 수행시간 약 500us ~ 3ms로 측정됨
        Speed_LCD_Alart(spd);

        // USART
        /*
        USART_TX_String("Speed:");
        USART_TX_String(buffer);
        USART_TX_String("cm/s\r\n");

        if (spd > OVERSPEED_LIMIT) {//100cm/s의 속도를 초과하는 경우
        	USART_TX_String("Speed limit has reached!!!\r\n");
        }
        USART_TX_String("\r\n");
        */

        //
        // millis() 에 따라 led 점멸
        // use portF
        Traffic_Light_Cycle();
        Print_Overview();
        Fluid_Traffic_Light_Adjust();

        // 시작 3초 이후 신호 위반 감지했다면 2초동안 차량 경고
        if (millis() > 3000 && millis() - carPrevmillis < 2000) {
            // 차량 경고
            PORTF |= 1 << BUZZER_PIN;
        }
        // 시작 3초 이후 신호 위반 감지했다면 3초동안 보행자 경고
        else if (millis() > 3000 && millis() - humanPrevmillis < 3000) {
            // 보행자 경고
            PORTF |= 1 << BUZZER_PIN;
            if (millis() / 500 % 2)
                PORTF |= 1 << WARNING_LED;
            else
                PORTF &= ~(1 << WARNING_LED);
        } else {
            //경고
            PORTF &= ~(1 << BUZZER_PIN);
            PORTF &= ~(1 << WARNING_LED);
        }
        if (servoPos != prevPos) {
            PWM(PWM_FREQ, map(servoPos, 0, 180, 2.5, 12.5));//value mapping and set PWM output
            prevPos = servoPos;
        }
    }
}

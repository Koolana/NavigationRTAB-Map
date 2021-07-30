// Тест пройденного расстояния по прямой
// Черно-голубое шасси Black-Blue (BB)
// PPR = 1450



// Управление в терминале (115200)
// f - задать расстояние в сантиметрах
// s - задать скорость от 0 до 255


/*   Использование выводов Arduino MEGA
===PWM==============================================================
Моторы
5 ---> правый мотор PWM1 (+ земля)
4 ---> правый мотор вперед=1, назад=0
6 ---> левый мотор PWM2
7 ---> левый мотор вперед=1, назад=0

===DIGITAL==========================================================
Энкодеры
2 ---> A-пин энкодера М1-правого мотора
3 ---> A-пин энкодера М2-левого мотора 


===ANALOG==========================================================
нет
*/

//#include <Metro.h>
//#include <PID_v1.h>
#include <math.h>
#include <uTimerLib.h>

// Число импульсов энкодера на оборот колеса
#define PPR 1450
// Диаметр колеса в мм
#define WD 134

const float WK = 3.14159 * WD / PPR / 10;  // постоянная колеса в см

// MegaADK DIGITAL PINS USABLE FOR INTERRUPTS 2, 3, 18, 19, 20, 21
//                                                 I2C pins 20, 21

//Motor control variables
const int MotorRpwm = 5;    // Right motor PWM Speed Control pin
const int MotorLpwm = 6;    // Left motor PWM Speed Control pin
const int MotorRdir = 4;    // Right motor Direction Control pin
const int MotorLdir = 7;    // Left motor Direction Control pin

//Encoder variables
const byte encoderRpinA = 2;    //A pin -> the interrupt pin ПРАВЫЙ
const byte encoderLpinA = 3;    //A pin -> the interrupt pin ЛЕВЫЙ 
bool DirectionR = LOW;
bool DirectionL = LOW;

double wheelSpeedR = 0;  // Скорость правого колеса с энкодера
double wheelSpeedL = 0;  // Скорость левого колеса с энкодера

unsigned long wheelImpR; // число импульсов с энкодера правого колеса 
unsigned long wheelImpL; // число импульсов с энкодера левого колеса 

byte encoderRPinALast;
byte encoderLPinALast;

long LastTimeR = 0;
long LastTimeL = 0;
bool CountIntervalR = LOW;
bool CountIntervalL = LOW;

byte SetSpeedR = 0;   //Wish Speed of right motor
byte SetSpeedL = 0;   //Wish Speed of left motor

bool goDistR = false;     // Признак того, что задана длина пути, в конце которого будет остановка
bool goDistL = false;     
long  pulseCountR = 0;    // Число импульсов правого энкодера осталось идти
long  pulseCountL = 0;    // Число импульсов левого энкодера осталось идти

// Timer variables
const long Interval=100000;                                // 0.1 second - Timer interrupt interval

// ---------------- Счетчики времени ----------------
// Период вывода в терминал
bool t1 = false;        // true - флаг время прошло
bool t1s = false;       // true - флаг начать счет времени
unsigned long t1m;      // значение времени в начале счёта
const unsigned long t1d = 1000;  // уставка времени в мс  

// Период опроса клавиатуры
bool t2 = false;        // true - флаг время прошло
bool t2s = false;       // true - флаг начать счет времени
unsigned long t2m;      // значение времени в начале счёта
const unsigned long t2d = 300;  // уставка времени в мс  

// Задержка реверса правого колеса
bool t3 = false;        // true - флаг время прошло
bool t3s = false;       // true - флаг начать счет времени
unsigned long t3m;      // значение времени в начале счёта
const unsigned long t3d = 40;  // уставка времени в мс  
// Задержка реверса левого колеса
bool t4 = false;        // true - флаг время прошло
bool t4s = false;       // true - флаг начать счет времени
unsigned long t4m;      // значение времени в начале счёта
const unsigned long t4d = 40;  // уставка времени в мс  

// Задержка пуска движения
bool t5 = false;        // true - флаг время прошло
bool t5s = false;       // true - флаг начать счет времени
unsigned long t5m;      // значение времени в начале счёта
const unsigned long t5d = 5000;  // уставка времени в мс  

// ================================= SETUP ==================================================
void setup() {

  pinMode(13, OUTPUT);
  
  MotorsInit();
  EncoderInit();//Initialize encoder
  
  Serial.begin(115200);//Initialize the serial port
  while (!Serial) {  }     ; // wait for serial port to connect. Needed for native USB port only

  delay(1000); 
}

// ================================= ГЛАВНЫЙ ЦИКЛ ===========================================
void loop() {

    keyboard();

    Check_times();  // проверить все выдержки времени

    Print();

}//loop

// ================================= Procedures =============================================

// Опрос клавиатуры
void keyboard(){
  if (t2){      // проверить окончание выдержки времени
  
    String inString = "";
    
    bool setDistance = false;  // признак строки, задающей длину пути
    bool settingSpeed = false; // признак строки, задающей скорость
    
    while (Serial.available() > 0) {  // чтение строки из последовательного порта
      int inChar = Serial.read(); 
      if (inChar == 'f') {setDistance = true;}      // f - задать длину пути
      if (inChar == 's') {settingSpeed = true;}     // s - задать скорость от 0 до 255
      
      if (isDigit(inChar)) {  inString += (char)inChar;   } // convert the incoming byte to a char and add it to the string:
    }
    if ((inString.length() > 0) and settingSpeed){delay(5000); settingSpeed = false; SetSpeedR = inString.toInt(); SetSpeedL = SetSpeedR; analogWrite (MotorRpwm, SetSpeedR); analogWrite (MotorLpwm, SetSpeedL);}
    if ((inString.length() > 0) and setDistance ){
      setDistance = false; goDistR = true; goDistL = true;
       
      pulseCountR = (int)((float)inString.toInt()/WK/1);
      pulseCountL = (int)((float)inString.toInt()/WK/1);
    }
  }
  t2start();  // начать счёт времени, если уже начат, будет продолжаться старый
} 


// Обработка прерывания правого энкодера, вызывается при каждом импульсе
void DecodeSpeedR()  
{
  wheelImpR ++;
  if (goDistR) {
    pulseCountR = pulseCountR - 1;
    if (pulseCountR < 1) {goDistR = false; t3start(); analogWrite(MotorRpwm, 255); digitalWrite(MotorRdir, HIGH);} // reverse
  }
  if (t3){SetSpeedR = 0; analogWrite(MotorRpwm, SetSpeedR); digitalWrite(MotorRdir, LOW); t3=false;}
}
// Обработка прерывания левого энкодера, вызывается при каждом импульсе
void DecodeSpeedL()  
{
  wheelImpL ++;
  if (goDistL) {
    pulseCountL = pulseCountL - 1;
    if (pulseCountL < 1) {goDistL = false; t4start(); analogWrite(MotorLpwm, 255); digitalWrite(MotorLdir, HIGH);} // reverse
  }
  if (t4){SetSpeedL = 0; analogWrite(MotorLpwm, SetSpeedL); digitalWrite(MotorLdir, LOW); t4=false;}
}


void Timer_finish()
{
    wheelSpeedR = wheelImpR; // * Interval * 428 / 663 / 1000000;   // мм в сек; 215 мм в обороте, 20 импульсов за оборот
    wheelSpeedL = wheelImpL; // * Interval * 428 / 663 / 1000000;   // мм в сек; 215 мм в обороте, 20 импульсов за оборот
    wheelImpR = 0;
    wheelImpL = 0;
       
    digitalWrite(13, digitalRead(13) ^ 1);    
}

// -------------------------------- Initialisation ----------------------------------------- 
 void MotorsInit() //Initialize motors variables
{
  pinMode(MotorRdir, OUTPUT);
  pinMode(MotorRdir, OUTPUT);
  pinMode(MotorRpwm, OUTPUT);
  pinMode(MotorLpwm, OUTPUT);
  
  digitalWrite (MotorRdir, LOW);
  digitalWrite (MotorLdir, LOW);
  analogWrite (MotorRpwm, 0);
  analogWrite (MotorLpwm, 0);
}

 void EncoderInit() //Initialize encoder interruption
{
  DirectionR = true;//default -> Forward  
  DirectionL = true;//default -> Forward  
  
  pinMode(encoderRpinA,INPUT);  // Right weel
  pinMode(encoderLpinA,INPUT);  // Left weel
 
  // Привязка прерывания по импульсу энкодера
  attachInterrupt(digitalPinToInterrupt(encoderRpinA), DecodeSpeedR, RISING); // вызов процедуры по прерыванию. Параметры: номер прерывания (не ножки), имя процедуры, состояние сигнала
  attachInterrupt(digitalPinToInterrupt(encoderLpinA), DecodeSpeedL, RISING);

  // Настройка таймера
TimerLib.setInterval_us(Timer_finish, Interval);
  
  wheelImpR = 0;
  wheelImpL = 0;
}


//==================================== Счетчики времени =====================================

void t1start(){         // начало счёта времени
  if (!t1s) {
    t1 = false;         // сбросить флаг завершения
    t1s = true;         // признак активного счёта
    t1m = millis ();    // запомнить начальное время
    }
}
void t2start(){         // начало счёта времени
  if (!t2s) {
    t2 = false;         // сбросить флаг завершения
    t2s = true;         // признак активного счёта
    t2m = millis ();    // запомнить начальное время
    }
}
void t3start(){         // начало счёта времени
  if (!t3s) {
    t3 = false;         // сбросить флаг завершения
    t3s = true;         // признак активного счёта
    t3m = millis ();    // запомнить начальное время
    }
}
void t4start(){         // начало счёта времени
  if (!t4s) {
    t4 = false;         // сбросить флаг завершения
    t4s = true;         // признак активного счёта
    t4m = millis ();    // запомнить начальное время
    }
}

void Check_times() {    // ------------------- Проверка выдержек времени ----------------------

if (t1s) {             // есть задание на выдержку времени
  if (t1m + t1d < millis ()) {
    t1 = true;      // время вышло
    t1s = false;    // счет закончить
    }
  }

if (t2s) {             // есть задание на выдержку времени
  if (t2m + t2d < millis ()) {
    t2 = true;      // время вышло
    t2s = false;    // счет закончить
    }
  }

if (t3s) {             // есть задание на выдержку времени
  if (t3m + t3d < millis ()) {
    t3 = true;      // время вышло
    t3s = false;    // счет закончить
    }
  }
if (t4s) {             // есть задание на выдержку времени
  if (t4m + t4d < millis ()) {
    t4 = true;      // время вышло
    t4s = false;    // счет закончить
    }
  }
if (t5s) {             // есть задание на выдержку времени
  if (t5m + t5d < millis ()) {
    t5 = true;      // время вышло
    t5s = false;    // счет закончить
    }
  }
  
}//Check_times

void Print(){   // --------------------- Печать на экране --------------------------------
    
  if (t1){      // проверить окончание выдержки времени
    Serial.println("f - задать расстояние в сантиметрах");
    Serial.println("s - задать скорость от 0 до 255");
    Serial.print ("L:");   Serial.print (SetSpeedL); Serial.print (" > "); Serial.print (wheelSpeedL); 
    Serial.print ("\tR:"); Serial.print (SetSpeedR); Serial.print (" > "); Serial.print (wheelSpeedR); 
    Serial.print ("\tIpmLast: "); Serial.print (pulseCountL); Serial.print (" "); Serial.print (pulseCountR); 
    Serial.println(); 
    }
  
  t1start();  // начать счёт времени, если уже начат, будет продолжаться старый
}

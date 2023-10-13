#define TIME_SLICE_S 4
#define TIME_SLICE (TIME_SLICE_S * 1000)
#define M 2

#define epsilon 20 // 20 us error
#define SLICE_LEN (TIME_SLICE >> M)
#define SLICE00 (SLICE_LEN*0)
#define SLICE01 (SLICE_LEN*1)
#define SLICE10 (SLICE_LEN*2)
#define SLICE11 (SLICE_LEN*3)

#define LED0 2
#define LED1 3
#define LED2 4
#define LED3 5
#define REC0 6
#define OUT0 7
#define OUT1 8
#define OUT2 9
#define OUT3 10

#include <Chrono.h>
Chrono myChrono;


int DATA = 0; // currently received data
int POS_EDGE = 0;
int FLASH_COUNTER = 0;
int flash0, flash1, flash2;
int RAW_DATA = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Hello world!");
  
  pinMode(REC0, INPUT);
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  pinMode(OUT0, OUTPUT);
  pinMode(OUT1, OUTPUT);
  pinMode(OUT2, OUTPUT);
  pinMode(OUT3, OUTPUT);


  digitalWrite(OUT0,LOW);
  digitalWrite(OUT1,LOW);
  digitalWrite(OUT2,LOW);
  digitalWrite(OUT3,LOW);
  
  digitalWrite(LED0,LOW);
  digitalWrite(LED1,LOW);
  digitalWrite(LED2,LOW);
  digitalWrite(LED3,LOW);
}

void loop() {
  delay(400);
  if(analogRead(A7) >= (540) and POS_EDGE == 0) {
//  if(digitalRead(REC0) and POS_EDGE == 0) {
    POS_EDGE = 1;
    Serial.print("#: ");
    Serial.print(FLASH_COUNTER);
    Serial.print(" >> ");

    if (FLASH_COUNTER == 0) {
        Serial.println(" 0");
        myChrono.start();
        flash0 = myChrono.elapsed(); // start
        digitalWrite(LED0,HIGH);
    } else if (FLASH_COUNTER == 1) {
        Serial.println(" 1");
        flash1 = myChrono.elapsed();
        digitalWrite(LED1,HIGH);
    } else if (FLASH_COUNTER == 2) {
        Serial.println(" 2");
        flash2 = myChrono.elapsed();
        digitalWrite(LED2,HIGH);
    }
    FLASH_COUNTER++;    
  }
  else if (!digitalRead(REC0) and POS_EDGE){
    POS_EDGE = 0;
    Serial.println("OK");
    
    
    if (FLASH_COUNTER == 3) {
      FLASH_COUNTER = 0;
      Serial.println("    > RESET");
      Serial.print("Init: "); 
      Serial.print(flash0);
      Serial.print(" | ");
      Serial.print(flash1);
      Serial.print(" | ");
      Serial.println(flash2);
      Serial.print("0->1: ");
      Serial.print(flash1-flash0);
      Serial.print(" | 1->2: ");
      Serial.println(flash2-flash1);

      digitalWrite(LED0,LOW);
      digitalWrite(LED1,LOW);
      digitalWrite(LED2,LOW);
      flash0, flash1, flash2 = 0;
      myChrono.restart();

      RAW_DATA = flash1 - flash0;
      digitalWrite(OUT0,LOW);
      digitalWrite(OUT1,LOW);
      digitalWrite(OUT2,LOW);
      digitalWrite(OUT3,LOW);
      
      
      if(SLICE00 <= RAW_DATA and RAW_DATA < SLICE01+300) {
        Serial.println("<<0>>");  
        digitalWrite(OUT0,HIGH);
      }
      else if(SLICE01 <= RAW_DATA and RAW_DATA < SLICE10+300) {
        Serial.println("<<1>>");
        digitalWrite(OUT1,HIGH);
      }
      else if(SLICE10 <= RAW_DATA and RAW_DATA < SLICE11+300) {
        Serial.println("<<2>>");
        digitalWrite(OUT2,HIGH);
      }
      else {
        Serial.println("<<3>>");
        digitalWrite(OUT3,HIGH);
      }
    }
  }
}

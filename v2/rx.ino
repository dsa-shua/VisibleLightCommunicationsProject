#define ON_TIME 100 // 50ms
#define CD_TIME (ON_TIME << 1) 
#define HEAD_BUFFER 100 // (ON_TIME << 1)
#define TAIL_BUFFER 100 //(ON_TIME << 1)
#define M (3) // 2^m 
#define N (1 << M) // = 8

#define TX_IN 12
#define HEAD_LED 2
#define DATA_LED 3

#define BIT0 10
#define BIT1 9
#define BIT2 8
#define BIT3 7

#include <Chrono.h>
Chrono timeReg;
//int busyReg = 0;
int timeDiff, dataFlash = 0;
enum states {WAIT, HEAD, DATA, DONE};
enum states statusReg = WAIT;

int result, result_raw = 0;

void setup() {
  Serial.begin(9600);
  pinMode(TX_IN,INPUT);
  pinMode(HEAD_LED,OUTPUT);
  pinMode(DATA_LED,OUTPUT);
  
  pinMode(BIT0,OUTPUT);
  pinMode(BIT1,OUTPUT);
  pinMode(BIT2,OUTPUT);
  pinMode(BIT3,OUTPUT);

}

void loop() {
  if(digitalRead(TX_IN) and statusReg == WAIT){
    timeReg.restart(); // start counting again...
//    headFlash = timeReg.elapsed();
    statusReg = HEAD; // head flash just came in
    digitalWrite(HEAD_LED, HIGH);
    delay(CD_TIME);
  } else if (digitalRead(TX_IN) and statusReg == HEAD) {
    dataFlash = timeReg.elapsed();
    statusReg = DATA;
    digitalWrite(DATA_LED, HIGH);
    delay(CD_TIME);
  } else if (statusReg == DATA) {
    statusReg = DONE;
    digitalWrite(HEAD_LED, LOW);
    digitalWrite(DATA_LED, LOW);
    delay(CD_TIME);
  } else if (statusReg == DONE) {
    Serial.print("timeDiff (head to data): ");
    Serial.println(dataFlash);

    result_raw = dataFlash-HEAD_BUFFER-ON_TIME;
    Serial.print("removing head: ");
    Serial.println(result_raw);

    result = result_raw / ON_TIME;
    Serial.print("resulting value: ");
    Serial.println(result);
    
    digitalWrite(BIT0, result & 0x1);
    digitalWrite(BIT1, result & 0x2);
    digitalWrite(BIT2, result & 0x4);
    digitalWrite(BIT3, result & 0x8);
    
    delay(800);
    statusReg = WAIT;
  }

}

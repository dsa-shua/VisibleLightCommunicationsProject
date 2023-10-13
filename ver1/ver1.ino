#define TIME_SLICE_S 4 // 1 sec
#define TIME_SLICE (TIME_SLICE_S * 1000)
#define M 2                               // 2^(m)
#define SLICE_LEN (TIME_SLICE >> (M))
#define SLICE_00 (SLICE_LEN*0)
#define SLICE_01 (SLICE_LEN*1)
#define SLICE_10 (SLICE_LEN*2)
#define SLICE_11 (SLICE_LEN*3)
#define SELF_DELAY 500                      // ms
#define NEXT_DELAY 5000                   // ms
#define LASER 12                          // DigitalPin 12

#define LED0 8
#define LED1 9
#define LED2 10
#define LED3 11

#define TEST_LED 7

#define LED_READERS 1

#include <Chrono.h>

Chrono myChrono;
int time_start, total_start;
int latency_0, latency_1, latency_2, total_latency;

int DATA = 0; // 0 : 00, 1: 01, 2: 10, 3: 11


/* Print info about configuration */ 
void _config(void){
  Serial.println("========= CONFIG ========");
  Serial.print("TIME_SLICE: "); Serial.println(TIME_SLICE);
  Serial.print("M:          "); Serial.println(M);
  Serial.print("SLICE_LEN: ");  Serial.println(SLICE_LEN);
  Serial.print("00 Time:    "); Serial.println(SLICE_LEN*0);
  Serial.print("01 Time:    "); Serial.println(SLICE_LEN*1);
  Serial.print("10 Time:    "); Serial.println(SLICE_LEN*2);
  Serial.print("11 Time:    "); Serial.println(SLICE_LEN*3); 
  Serial.println("=========================");
}

void _transfer(int data){

#if LED_READERS
  if (data == 0) digitalWrite(LED0, HIGH);  
  else if (data == 1) digitalWrite(LED1, HIGH);
  else if (data == 2) digitalWrite(LED2, HIGH);
  else digitalWrite(LED3, HIGH);
#endif
   
  total_start = myChrono.elapsed();
  time_start = myChrono.elapsed();

  // Initial Flash
  digitalWrite(LASER, HIGH);
  delay(SELF_DELAY); 
  digitalWrite(LASER,LOW);
  delay(SELF_DELAY);

  latency_0 = myChrono.elapsed() - time_start; // init flash
//  time_start = myChrono.elapsed();

  // Second Flash
  if (data == 0){
    digitalWrite(LASER,HIGH);
    
    delay(SELF_DELAY);

    latency_1 = myChrono.elapsed() - time_start; // second flash
    digitalWrite(LASER, LOW);
    delay(TIME_SLICE-SLICE_00);
  }
  else if (data == 1){
    delay(SLICE_01);
    digitalWrite(LASER,HIGH);
    
    delay(SELF_DELAY);

    latency_1 = myChrono.elapsed() - time_start; // second flash
    digitalWrite(LASER, LOW);
    delay(TIME_SLICE-SLICE_01);
    
  }
  else if (data == 2) {
    delay(SLICE_10);
    digitalWrite(LASER, HIGH);

    delay(SELF_DELAY);

    latency_1 = myChrono.elapsed() - time_start; // second flash
    digitalWrite(LASER, LOW);
    delay(TIME_SLICE-SLICE_10);
    
  } else {
    delay(SLICE_11);
    digitalWrite(LASER,HIGH);

    delay(SELF_DELAY);

    latency_1 = myChrono.elapsed() - time_start; // second flash
    digitalWrite(LASER, LOW);
    delay(1);
  }

//  latency_1 = myChrono.elapsed() - time_start;
//  time_start = myChrono.elapsed();

  // Tail flash
  delay(250);
  digitalWrite(LASER, HIGH);
  delay(SELF_DELAY);
  digitalWrite(LASER, LOW);

  latency_2 = myChrono.elapsed() - time_start;
  total_latency = myChrono.elapsed() - total_start;

  Serial.print("Total Elapsed: ");
  Serial.print(total_latency);
  Serial.print("  | 1: ");
  Serial.print(latency_0);
  Serial.print("  | 2: ");
  Serial.print(latency_1);
  Serial.print("  | 3: ");
  Serial.println(latency_2);

  if (data == 0) digitalWrite(LED0, LOW);  
  else if (data == 1) digitalWrite(LED1, LOW);
  else if (data == 2) digitalWrite(LED2, LOW);
  else digitalWrite(LED3, LOW);
}


void setup() {
  Serial.begin(9600);
  _config();
  pinMode(LASER, OUTPUT);

  digitalWrite(LASER, LOW);



#if LED_READERS
  pinMode(LED0, OUTPUT); digitalWrite(LED0, LOW);
  pinMode(LED1, OUTPUT); digitalWrite(LED1, LOW);
  pinMode(LED2, OUTPUT); digitalWrite(LED2, LOW);
  pinMode(LED3, OUTPUT); digitalWrite(LED3, LOW);
#endif

//  pinMode(TEST_LED, OUTPUT); digitalWrite(TEST_LED,HIGH);


}

void loop() {
  Serial.println(0);
  delay(NEXT_DELAY);
  _transfer(0);

  Serial.println(1);
  delay(NEXT_DELAY);
   _transfer(1);

  Serial.println(2);
  delay(NEXT_DELAY);
   _transfer(2);

  Serial.println(3);
  delay(NEXT_DELAY);
   _transfer(3);
}

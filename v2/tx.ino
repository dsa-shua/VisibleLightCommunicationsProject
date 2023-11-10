#define ON_TIME 5 // 50ms
#define CD_TIME 10 // (ON_TIME << 1) 
#define HEAD_BUFFER 10 // (ON_TIME << 1)
#define TAIL_BUFFER 10 //(ON_TIME << 1)
#define M (8) // 2^8, 8-bits 
#define N (1 << M) // 8 bits

#define BIT3 11
#define BIT2 10
#define BIT1 9
#define BIT0 8

#define LASER 12

#include <Chrono.h>
Chrono timeReg;
int headStart, headEnd, posEdge, negEdge, tailTime = 0;

void test(){
  Serial.println("===========TEST===========");
  Serial.print("Len: ");
  Serial.println(N);
  for(byte i = 0; i < N; i++){
    digitalWrite(BIT3, (i & 0x8));
    digitalWrite(BIT2, (i & 0x4));
    digitalWrite(BIT1, (i & 0x2));
    digitalWrite(BIT0, (i & 0x1));
    timeReg.restart();
    headStart = timeReg.elapsed();
    digitalWrite(LASER,HIGH);
    delay(ON_TIME);
    digitalWrite(LASER,LOW);
    delay(HEAD_BUFFER);
    headEnd = timeReg.elapsed();
    delay((i*ON_TIME)+1);
    posEdge = timeReg.elapsed(); // a
    digitalWrite(LASER,HIGH);
    delay(ON_TIME);
    digitalWrite(LASER,LOW);
    negEdge = timeReg.elapsed();
    delay(CD_TIME);
    
    delay((N-i)*ON_TIME + 1);
    tailTime = timeReg.elapsed();

    Serial.print("[");
    Serial.print(i);
    Serial.println("]");
    
    Serial.print("  headStart:");
    Serial.println(headStart);
    
    Serial.print("  headEnd:");
    Serial.println(headEnd);
    
    
    Serial.print("  posEdge:");
    Serial.println(posEdge);

    Serial.print("  negEdge:");
    Serial.println(negEdge);

    Serial.print("  tailTime:");
    Serial.println(tailTime);

    delay(200);
    
  }

  Serial.println("=========================");
  
}


void setup() {
  Serial.begin(9600);
  pinMode(BIT3, OUTPUT);
  pinMode(BIT2, OUTPUT);
  pinMode(BIT1, OUTPUT);
  pinMode(BIT0, OUTPUT);
  pinMode(LASER, OUTPUT);
  digitalWrite(LASER, LOW);

  test();
}

void loop() {

}


// VPPM PARAMETERS
#define ON_TIME 5                                                 // 50ms
#define CD_TIME 10                                                // (ON_TIME << 1) 
#define HEAD_BUFFER 10                                            // (ON_TIME << 1)
#define TAIL_BUFFER 10                                            //(ON_TIME << 1)
#define M (8)                                                     // # of bits at a time 
#define N 0xFF

// PHYSICAL PINOUT
#define TX_IN 13
#define HEAD_LED 2
#define DATA_LED 3

#define BIT0 11
#define BIT1 10
#define BIT2 9
#define BIT3 8

#define ACK_PIN 6

// DEVICE INFORMATION
#define MASTER 0
#define DEV1 1
#define DEV2 2
#define SELF_ID DEV1                                                // ID NUMBER OF THE CURRENT DEVICE

// INSTRUCTION SET 
typedef unsigned short INSTRUCTION;
#define DT_MASK 0xFF00
#define RS_MASK 0x00C0
#define RD_MASK 0x0030
#define OP_MASK 0x000E
#define OP_SHFT 1
#define RD_SHFT 4
#define RS_SHFT 6
#define DT_SHFT 8

#define opINV 0b000                                                 // do not use
#define opREAD 0b001                                                // reg read         --> [REG number]  [SELF ID] [DEV ID]    [001]
#define opACK 0b010                                                 // SEND ACK         --> [-0-]         [SELF ID] [MASTER ID] [010]
#define opACK_RELAY 0b011                                           // ACK is relayed   --> [#ID FROM]    [SELF ID] [MASTER ID] [011]
#define opLED_OFF 0b100                                             // LED OFF          --> [#LED NUMBER] [SELF ID] [DEV ID]    [100]
#define opCUSTOM 0b101                                              // Custom
#define opLED_ON 0b110                                              // LED ON           --> [#LED NUMBER] [SELF ID] [DEV ID]    [110] 
#define opCHAR 0b111                                                // send "character" --> [ASCII CODE]  [SELF ID] [DEV ID]    [111]

// Instruction is divided into two parts
#define I0_MASK 0x00FF                                              // Mask for lower bits
#define I1_MASK 0xFF00                                              // Mask for higher bits
#define I_SHIFT 8                                                   // Shift value for higher bits


// Time and State Related Values
#include <Chrono.h>
Chrono timeReg, VPPM_TIME_REG;                                      // timer registers
int VPPM_TIME_0, VPPM_TIME_1 = 0, PROTOCOL_TIME;                    // Timer registers init
enum states {WAIT0, HEAD0, DATA0, DONE0, WAIT1, HEAD1, DATA1};      // Single phase FSM status
enum states statusReg = WAIT0;                                      // Starting State (Phase)

// Instruction Buffers
INSTRUCTION I0, I1;                                               // Lower instructions, upper instructions
INSTRUCTION I_BUFFER = 0;                                         // Full instruction holder
byte OP, RD, RS, DT = 0;                                          // Instruction breakdown holder

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

  pinMode(ACK_PIN, OUTPUT);

}

// Set LED to match binary data
void DEV_LED_ON(byte data){
    digitalWrite(BIT0, data & 0x1);
    digitalWrite(BIT1, data & 0x2);
    digitalWrite(BIT2, data & 0x4);
    digitalWrite(BIT3, data & 0x8);
}


// Turn off LED set by data
void DEV_LED_OFF(byte data) {
    switch(data){
      case 0: digitalWrite(BIT0, LOW); break;
      case 1: digitalWrite(BIT1, LOW); break;
      case 2: digitalWrite(BIT2, LOW); break;
      case 3: digitalWrite(BIT3, LOW); break;
      default: Serial.println("LED OFF out of range...");
    }
}

void RX_Process(INSTRUCTION instruction){
  Serial.println(">>> Processing Received Instruction...");
  OP = (I_BUFFER & OP_MASK) >> OP_SHFT;
  RD = (I_BUFFER & RD_MASK) >> RD_SHFT;
  RS = (I_BUFFER & RS_MASK) >> RS_SHFT;
  DT = (I_BUFFER & DT_MASK) >> DT_SHFT;

  Serial.print("  OP: ");
  switch(OP){
    case opINV: Serial.println("opINV"); break;
    case opREAD: Serial.println("opREAD"); break;
    case opACK: Serial.println("opACK"); break;
    case opACK_RELAY: Serial.println("opACK_RELAY"); break;
    case opLED_OFF: Serial.println("opLED_OFF"); break;
    case opCUSTOM: Serial.println("opCUSTOM"); break;
    case opLED_ON: Serial.println("opLED_ON"); break;
    case opCHAR: Serial.println("opCHAR"); break;
    default: Serial.println("unknown opcode");
  }

  Serial.print("  RS: ");
  switch(RS) {
    case MASTER: Serial.println("MASTER"); break;
    case DEV1: Serial.println("DEV1"); break;
    case DEV2: Serial.println("DEV2"); break;
    default: Serial.println("unknown source");
  }

  Serial.print("  RD: ");
  switch(RD) {
    case MASTER: Serial.println("MASTER"); break;
    case DEV1: Serial.println("DEV1"); break;
    case DEV2: Serial.println("DEV2"); break;
    default: Serial.println("unknown destination");
  }

  Serial.print("  DT: "); Serial.println(DT);
  Serial.println("");

#if (SELF_ID == MASTER)
  // Reserved for MASTER device
#endif

#if (SELF_ID == DEV1)
  if(RD == MASTER && RD == DEV1){
  switch(OP){
    case opLED_ON: {DEV_LED_ON(DT); break;}
    case opLED_OFF: {DEV_LED_OFF(DT); break;}
    default: Serial.println("<unknown or yet to be established>");
  }
    
  }
#endif
  Serial.println("Processing done...");
  digitalWrite(ACK_PIN,HIGH);
  delay(30);
  digitalWrite(ACK_PIN,LOW);
}


void loop() {
 if(digitalRead(TX_IN) and statusReg == WAIT0){
    Serial.println("RX Start...");
    timeReg.restart(); // start counting again...
    VPPM_TIME_REG.restart();
    statusReg = HEAD0; // head flash just came in
    digitalWrite(HEAD_LED, HIGH);
    delay(CD_TIME);  
  } 
  
  else if (digitalRead(TX_IN) and statusReg == HEAD0) {
    VPPM_TIME_0 = timeReg.elapsed();
    statusReg = DATA0;
    digitalWrite(DATA_LED, HIGH);
    delay(CD_TIME);  
  } 
  
  else if (statusReg == DATA0) {
    statusReg = DONE0;
    digitalWrite(HEAD_LED, LOW);
    digitalWrite(DATA_LED, LOW);
    delay(CD_TIME);
  } 
  
  else if (statusReg == DONE0) {
    Serial.print("  [P0] D_Pulse_Time: ");
    Serial.println(VPPM_TIME_0);

    result_raw = VPPM_TIME_0 - HEAD_BUFFER - ON_TIME;
    I0 = result_raw / ON_TIME;
    Serial.print("  [P0] Obtained: ");
    Serial.println(I0);
    
    digitalWrite(BIT0, I0 & 0x1);
    digitalWrite(BIT1, I0 & 0x2);
    digitalWrite(BIT2, I0 & 0x4);
    digitalWrite(BIT3, I0 & 0x8);
    
    delay(20);
    statusReg = WAIT1;
  }

  else if(digitalRead(TX_IN) and statusReg == WAIT1){
    timeReg.restart(); // start counting again...
    statusReg = HEAD1; // head flash just came in
    digitalWrite(HEAD_LED, HIGH);
    delay(CD_TIME);  
  } 
  
  else if (digitalRead(TX_IN) and statusReg == HEAD1) {
    VPPM_TIME_0 = timeReg.elapsed();
    statusReg = DATA1;
    digitalWrite(DATA_LED, HIGH);
    delay(CD_TIME);  
  } 
  
  else if (statusReg == DATA1) {
    statusReg = WAIT0;
    digitalWrite(HEAD_LED, LOW);
    digitalWrite(DATA_LED, LOW);
    delay(CD_TIME);

    Serial.print("  [P2] D_Pulse_Time: ");
    Serial.println(VPPM_TIME_0);

    result_raw = VPPM_TIME_0 - HEAD_BUFFER - ON_TIME;
    I1 = result_raw / ON_TIME;
    Serial.print("  [P2] Obtained: ");
    Serial.println(I1);
    
    digitalWrite(BIT0, I1 & 0x1);
    digitalWrite(BIT1, I1 & 0x2);
    digitalWrite(BIT2, I1 & 0x4);
    digitalWrite(BIT3, I1 & 0x8);

    VPPM_TIME_1 = VPPM_TIME_REG.elapsed();
    Serial.print("RX Took: "); Serial.println(VPPM_TIME_1);

    
    // Start Processing
    I_BUFFER = (I1 << I_SHIFT) | I0;
    Serial.print("Received Instruction: ");
    Serial.println(I_BUFFER);

    Serial.println(""); Serial.println("");
    
    // Actual Processing
//    delay(); // remove this
    RX_Process(I_BUFFER);
  }   
}

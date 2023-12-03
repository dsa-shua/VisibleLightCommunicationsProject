/*
    ESP32 RX (DEV1)


*/


// VPPM PARAMETERS
#define ON_TIME 5                                                 // 50ms
#define CD_TIME 10                                                // (ON_TIME << 1) 
#define HEAD_BUFFER 10                                            // (ON_TIME << 1)
#define TAIL_BUFFER 10                                            //(ON_TIME << 1)
#define M (8)                                                     // # of bits at a time 
#define N 0xFF

// PHYSICAL PINOUT -done
#define TX_IN 13
#define ACK_PIN 25
#define HEAD_LED 2
#define DATA_LED 4

#define BIT0 12
#define BIT1 14
#define BIT2 27
#define BIT3 26

#define DOG 33

// DEVICE INFORMATION
#define MASTER 0
#define DEV1 1
#define DEV2 2
#define SELF_ID DEV2                                                // ID NUMBER OF THE CURRENT DEVICE

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
Chrono timeReg, VPPM_TIME_REG;// timer registers
int tailTime, headEnd, protocolStart,lowerInstEnd, upperInstEnd; 
int VPPM_TIME_0, VPPM_TIME_1 = 0, PROTOCOL_TIME;                    // Timer registers init
enum states {WAIT0, HEAD0, DATA0, DONE0, WAIT1, HEAD1, DATA1};      // Single phase FSM status
enum states statusReg = WAIT0;                                      // Starting State (Phase)

// Instruction Buffers
INSTRUCTION I0, I1;                                               // Lower instructions, upper instructions
INSTRUCTION I_BUFFER = 0;                                         // Full instruction holder
byte OP, RD, RS, DT = 0;                                          // Instruction breakdown holder

int result, result_raw = 0;
bool dog_toggle = false;
bool RECEIVE = false;
void IRAM_ATTR isr() {
//  doProcess();
    RECEIVE = true;
}

#include <esp_task_wdt.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

INSTRUCTION TX_Encode (byte data, byte source, byte destination, byte opcode){
    INSTRUCTION t_data = data << DT_SHFT;
    INSTRUCTION t_source = source << RS_SHFT;
    INSTRUCTION t_destination = destination << RD_SHFT;
    INSTRUCTION t_opcode = opcode << OP_SHFT;
    return (t_data | t_source | t_destination | t_opcode); // Return the encoded instruction in 16-bits
}


// TODO: be able to handle two 8-bit split of the full 16-bit instruction...
// Decodes the instruction received from TX, returns values of RD, RS, DT, and OP.
void RX_Decode(INSTRUCTION instruction, byte *RD, byte *RS, byte *DT, byte *OP){

    // Extract Values
    unsigned data = (instruction & DT_MASK) >> DT_SHFT;
    unsigned source = (instruction & RS_MASK) >> RS_SHFT;
    unsigned destination = (instruction & RD_MASK) >> RD_SHFT;
    unsigned opcode = (instruction & OP_MASK) >> OP_SHFT;

    // "Return" values to outside
    *OP = opcode;
    *DT = data;
    *RS = source;
    *RD = destination;
}

// Actually transmit 8-bits of the instruction (either upper or lower halves)
void TX_Transmit(byte i){
  // Change this to match instruction set
  digitalWrite(BIT3, (i & 0x8));
  digitalWrite(BIT2, (i & 0x4));
  digitalWrite(BIT1, (i & 0x2));
  digitalWrite(BIT0, (i & 0x1));

  // Reset timer
  timeReg.restart();
  
  // First Pulse
  Serial.println("  First pulse");
  digitalWrite(TX_IN,HIGH);
  delay(ON_TIME);
  digitalWrite(TX_IN,LOW);
  delay(HEAD_BUFFER);
  
  headEnd = timeReg.elapsed();
  Serial.print("  Head:"); Serial.println(headEnd);
  

  // Wait For Second Pulse Position
  delay((i*ON_TIME)+1);

  // At right time, send Second Pulse
  Serial.println("  Second pulse");
  digitalWrite(TX_IN,HIGH);
  delay(ON_TIME);
  digitalWrite(TX_IN,LOW);

  tailTime = timeReg.elapsed();
  Serial.print("  Tail:"); Serial.println(tailTime);

  // TODO: check if this is necessary
  delay(CD_TIME);

  // Allow enough time for RX to process before sending another in burst
  delay(200);
}


// Wrapper function for transmitting information.
void TX_Commit(byte RD, byte RS, byte DT, byte OP){
  INSTRUCTION instruction = TX_Encode(DT, RS, RD, OP);    // Encode the instruction (16-bits)

  // Divide instruction into two parts
  byte inst0 = instruction & I0_MASK;                     // Inst part 1 (8-bits)
  byte inst1 = (instruction & I1_MASK) >> I_SHIFT;        // Inst part 2 (8-bits)

  // For debugging: show the full instruction and its partitions
  Serial.print("Encoded instruction:");Serial.println(instruction);
  Serial.print("Lower Instruction:"); Serial.println(inst0);
  Serial.print("Upper Instruction:"); Serial.println(inst1);


  // Actual transmittion of the instruction into two 8-bit parts
  VPPM_TIME_REG.restart(); // reset the VPPM time register
  protocolStart = VPPM_TIME_REG.elapsed();
  
  TX_Transmit(inst0);   // Transmit lower 8 bits
  lowerInstEnd = VPPM_TIME_REG.elapsed();
  Serial.println("  Lower part done.");
  Serial.print("    Lower part took: "); Serial.print(lowerInstEnd-protocolStart); Serial.println(" us.");

  delay(50); // Test this
  
  TX_Transmit(inst1); // transmit higher 8 bits
  upperInstEnd = VPPM_TIME_REG.elapsed();
  Serial.println("  Upper part done.");
  Serial.print("    Full TX took: ");Serial.print(upperInstEnd-protocolStart); Serial.println(" us");
  Serial.println("VPPM Protocol Done");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup() {
//  disableInterrupts();
  Serial.begin(115200);
  pinMode(TX_IN,INPUT);
//  attachInterrupt(TX_IN, isr, RISING);

  pinMode(DOG, OUTPUT);
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

#if (SELF_ID == DEV2)
  if (RD == MASTER && RD == DEV2)
    switch(OP){
    case opLED_ON: {DEV_LED_ON(DT); break;}
    case opLED_OFF: {DEV_LED_OFF(DT); break;}
    default: Serial.println("<unknown or yet to be established>");
  } else{

    TX_Commit(RD, DEV2, DT, OP);
  }
#endif

#
  Serial.println("Processing done...");
  digitalWrite(ACK_PIN, HIGH);
  delay(20);
  digitalWrite(ACK_PIN, LOW);
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
    RX_Process(I_BUFFER);
  }
  else{
    delay(1); digitalWrite(DOG, dog_toggle); dog_toggle = !dog_toggle;
  }
}

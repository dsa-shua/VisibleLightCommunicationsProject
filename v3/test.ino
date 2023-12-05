/*

Asynchronous VPPM Project
- Device:   ESP32 WROOM DA MODULE ( C-Type )
- ID:       MASTER (v3.1)


*/



// VPPM PARAMETERS
#define ON_TIME 5 // 50ms
#define CD_TIME 10 // (ON_TIME << 1) 
#define HEAD_BUFFER 10 // (ON_TIME << 1)
#define TAIL_BUFFER 10 //(ON_TIME << 1)
#define M (8) // 2^8, 8-bits 
#define N 0xFF 

// DEVICE INFORMATION
#define MASTER 0
#define DEV1 1
#define DEV2 2
#define SELF_ID MASTER              // ID NUMBER OF THE CURRENT DEVICE


// PHYSICAL PINOUT
#define BIT3 14
#define BIT2 27
#define BIT1 26
#define BIT0 25
#define TX_PIN 12
#define TX_RELAY 13
#define RX_PIN 7    // pin 13 is Serial TX for Arduino Uno
#define LED 32                      // watchdog LED
bool LED_STAT = 0;                  // switching 
bool ACK_REC = 0;                   // ACK received


// only master receives ACK, others do not
#if(SELF_ID == MASTER)
#define ACK_PIN 33                  // Interrupt pin for master
#define ACK_PINX 35                 // interrupt from relay
//#define SEND_PIN 34                 // send single
#endif

/*
Device 1 is the main target
it receives INST from master and sends ACK to master.
no need to receive ACK!
*/
#if(SELF_ID == DEV1)
#define RX_PIN 33     // pin to send ack
#define TX_PIN 1      // pin to receive inst
#endif


// This should be ARDUINO UNO for simplicity
#if (SELF_ID == DEV2)
#define RX_PIN0 -1 // pin towards master
#define RX_PIN1 -1

#endif


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

#define opINV 0b000                 // do not use
#define opREAD 0b001                // reg read         --> [REG number]  [SELF ID] [DEV ID]    [001]
#define opACK 0b010                 // SEND ACK         --> [-0-]         [SELF ID] [MASTER ID] [010]
#define opACK_RELAY 0b011           // ACK is relayed   --> [#ID FROM]    [SELF ID] [MASTER ID] [011]
#define opLED_OFF 0b100             // LED OFF          --> [#LED NUMBER] [SELF ID] [DEV ID]    [100]
#define opCUSTOM 0b101              // Custom
#define opLED_ON 0b110              // LED ON           --> [#LED NUMBER] [SELF ID] [DEV ID]    [110] 
#define opCHAR 0b111                // send "character" --> [ASCII CODE]  [SELF ID] [DEV ID]    [111]

// Instruction is divided into two parts
#define I0_MASK 0x00FF              // Mask for lower bits
#define I1_MASK 0xFF00              // Mask for higher bits
#define I_SHIFT 8                   // Shift value for higher bits

/*
// TODO: Finish ISA first then do this.
// Interrupts Control
#include <TimerTwo.h>
volatile bool TIMEOUT = false;
volatile bool ACK = false;
*/

// Timers and Registers
#include <Chrono.h>
Chrono fulltime, timeReg, VPPM_TIME_REG;
int headStart, headEnd, posEdge, negEdge, tailTime = 0;     // timeReg
int protocolStart, lowerInstEnd, upperInstEnd = 0;          // VPPM_TIME_REG
char incomingByte = 0;                                      // Serial Read Char Buffer

// ACK Buffer for Re-sending
byte RD_BUFF = 0;
byte RS_BUFF = 0;
byte DT_BUFF = 0;
byte OP_BUFF = 0;


// Print Some Information about the current configuration
void printConfig(void){
  Serial.println("");
  Serial.println("Config:");
  Serial.print("M: "); Serial.println(M);
  Serial.print("N: "); Serial.println(N);
  Serial.print("ON TIME: "); Serial.println(ON_TIME);
  Serial.print("CD TIME: "); Serial.println(CD_TIME);
  Serial.print("Max VPPM Time:");
  Serial.println(N * ON_TIME);
}


/* ACK HANDLER AFTER FINISHING ISA PART

// Interrupt Service Routine for RX
void RX_ISR(){
  // Check if RX received ACK.

  // TODO: Re-implement RX but only checking ACK signals
}


// After X amount of time, if !ACK --> TIMEOUT
void TIMEOUT_ISR() {
  TIMEOUT = true;  
}
*/

int ACK_COUNTER = 0;

void IRAM_ATTR isr(){
//  Serial.println("Hi there!");
  ACK_REC = true;
  
}

unsigned ACTIVE_TX = 0;

void setup() {

  // Set Serial Monitor
  Serial.begin(115200);
  
  pinMode(ACK_PIN, INPUT);
  attachInterrupt(ACK_PIN, isr, RISING);
  
  pinMode(LED, OUTPUT);
  // Reset Device LEDs
  pinMode(BIT3, OUTPUT);
  pinMode(BIT2, OUTPUT);
  pinMode(BIT1, OUTPUT);
  pinMode(BIT0, OUTPUT);

  
  pinMode(TX_PIN, OUTPUT); digitalWrite(TX_PIN, LOW); 
  pinMode(TX_RELAY, OUTPUT); digitalWrite(TX_RELAY,LOW);
  ACTIVE_TX = TX_PIN;
}

// Encodes the DT, RS, RD, and OP into an instruction. [return: 16-bits]
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
  digitalWrite(ACTIVE_TX,HIGH);
  delay(ON_TIME);
  digitalWrite(ACTIVE_TX,LOW);
  delay(HEAD_BUFFER);
  
  headEnd = timeReg.elapsed();
  Serial.print("  Head:"); Serial.println(headEnd);
  

  // Wait For Second Pulse Position
  delay((i*ON_TIME)+1);

  // At right time, send Second Pulse
  Serial.println("  Second pulse");
  digitalWrite(ACTIVE_TX,HIGH);
  delay(ON_TIME);
  digitalWrite(ACTIVE_TX,LOW);

  tailTime = timeReg.elapsed();
  Serial.print("  Tail:"); Serial.println(tailTime);

  // TODO: check if this is necessary
  delay(CD_TIME);

  // Allow enough time for RX to process before sending another in burst
  delay(200);
}

int ALL_TIME = 0;

// Wrapper function for transmitting information.
void TX_Commit(byte RD, byte RS, byte DT, byte OP){
  INSTRUCTION instruction = TX_Encode(DT, RS, RD, OP);    // Encode the instruction (16-bits)

  // Buffer for ACK
  RD_BUFF = RD;
  RS_BUFF = RS;
  DT_BUFF = DT;
  OP_BUFF = OP;

  if (RD == DEV1) ACTIVE_TX = TX_PIN;
  else if (RD == DEV2) ACTIVE_TX = TX_RELAY;
  
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
  ALL_TIME = ALL_TIME + upperInstEnd - protocolStart;
  Serial.println("  Upper part done.");
  Serial.print("    Full TX took: ");Serial.print(upperInstEnd-protocolStart); Serial.println(" us");
  Serial.println("VPPM Protocol Done");
}

unsigned fulltime_length = 0;

void do_process() {
// ONLY MASTER CAN SEND INSTRUCTIONS, ONLY SLAVES CAN SEND ACK
#if (SELF_ID == MASTER)
  // Just the interface...
  if (Serial.available()){
    incomingByte = (char)Serial.read();
    if (incomingByte) {
      Serial.print((char)incomingByte);
    }
    Serial.println(""); 

    fulltime.restart();
    switch(incomingByte){  
      case '0': {TX_Commit(DEV1, MASTER, 0, opLED_ON); break;}
      case '1': {TX_Commit(DEV1, MASTER, 1, opLED_ON); break;}
      case '2': {TX_Commit(DEV1, MASTER, 2, opLED_ON); break;}
      case '3': {TX_Commit(DEV1, MASTER, 3, opLED_ON); break;}
      case '4': {TX_Commit(DEV1, MASTER, 4, opLED_ON); break;}
      case '5': {TX_Commit(DEV1, MASTER, 5, opLED_ON); break;}
      case '6': {TX_Commit(DEV1, MASTER, 6, opLED_ON); break;}
      case '7': {TX_Commit(DEV1, MASTER, 7, opLED_ON); break;}
      case '8': {TX_Commit(DEV1, MASTER, 8, opLED_ON); break;}
      case '9': {TX_Commit(DEV1, MASTER, 9, opLED_ON); break;}
      case 'a': {TX_Commit(DEV1, MASTER, 10, opLED_ON); break;}
      case 'b': {TX_Commit(DEV1, MASTER, 11, opLED_ON); break;}
      case 'c': {TX_Commit(DEV1, MASTER, 12, opLED_ON); break;}
      case 'd': {TX_Commit(DEV1, MASTER, 13, opLED_ON); break;}
      case 'e': {TX_Commit(DEV1, MASTER, 14, opLED_ON); break;}
      case 'f': {TX_Commit(DEV1, MASTER, 15, opLED_ON); break;}
      case 'g': {TX_Commit(DEV2, MASTER, 15, opLED_ON); break;}
      case 'h': {TX_Commit(DEV2, MASTER, 0, opLED_ON); break;}

      case 'q': {TX_Commit(DEV1, MASTER, 2, opLED_OFF); break;}        // Turn off BIT2 of DEV1 (under testing)
      
      case 'z': {TX_Commit(DEV1, MASTER, 255, opLED_ON); break;}      // LED open given 255 value for DT
      case 'p': {printConfig(); break;}                               // Print Configurations (no pulses)
      case 'x': {TX_Transmit(255); break;}                            // Raw send 255 using VPPM (1 phase only)

      default: ; // Do nothing for NULL string


      // Wait for ACK signal
      
      ACK_COUNTER = 0;
      while(!ACK_REC && ACK_COUNTER < 3000){
        ACK_COUNTER++;
        delay(1);
      }
      ACK_COUNTER = 0;

      if(!ACK_REC) {
        Serial.println("Ack never came :(");
        ACK_COUNTER = 0;
        Serial.println("   Ack was not received! Relaying to S2.");

        // Relay
        detachInterrupt(ACK_PIN);
        TX_Commit(DEV1, MASTER, DT_BUFF, OP_BUFF); // move laser
        attachInterrupt(ACK_PIN, isr, RISING);
      }
      else Serial.println("Ack was received");


      Serial.println("TX DONE!");
      ACK_REC = false;
//      fulltime_length = fulltime.elapsed();
      Serial.print("Everything came under: ");
      Serial.print(ALL_TIME);
      Serial.println(" ms..");
      ALL_TIME = 0;
    }
  }
  else { // Code to keep the watchdog think we are not on infinite loop LOL
   delay(5);
   if(LED_STAT) digitalWrite(LED, HIGH);
   else digitalWrite(LED, LOW);
   LED_STAT = !LED_STAT;
  }
#endif

#if (SELF_ID == DEV1)
#endif
}

void loop(){ // keep watchdog from resetting the board.(ESP32)
  do_process();
}

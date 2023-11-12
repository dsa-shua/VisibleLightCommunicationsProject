
// VPPM PARAMETERS
#define ON_TIME 5 // 50ms
#define CD_TIME 10 // (ON_TIME << 1) 
#define HEAD_BUFFER 10 // (ON_TIME << 1)
#define TAIL_BUFFER 10 //(ON_TIME << 1)
#define M (8) // 2^8, 8-bits 
#define N 0xFF 

// PHYSICAL PINOUT
#define BIT3 11
#define BIT2 10
#define BIT1 9
#define BIT0 8
#define TX_PIN 12
#define RX_PIN 7                    // pin 13 is Serial TX for Arduino Uno

// DEVICE INFORMATION
#define MASTER 0
#define DEV1 1
#define DEV2 2
#define SELF_ID MASTER              // ID NUMBER OF THE CURRENT DEVICE

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
Chrono timeReg, VPPM_TIME_REG;
int headStart, headEnd, posEdge, negEdge, tailTime = 0;     // timeReg
int protocolStart, lowerInstEnd, upperInstEnd = 0;          // VPPM_TIME_REG
char incomingByte = 0;                                      // Serial Read Char Buffer


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

void setup() {

  // Set Serial Monitor
  Serial.begin(9600);

  // Reset Device LEDs
  pinMode(BIT3, OUTPUT);
  pinMode(BIT2, OUTPUT);
  pinMode(BIT1, OUTPUT);
  pinMode(BIT0, OUTPUT);

  // Set and initialize TX and RX Pins
  pinMode(TX_PIN, OUTPUT); digitalWrite(TX_PIN, LOW); // PIN 12
  pinMode(RX_PIN, INPUT);                             // PIN 7


  // TODO: After ISA is done, do ACK handling using HW and Timer Interrupts

/*
  // Attach the (hardware) interrupt to the RX pin
  attachInterrupt(digitalPinToInterrupt(RX_PIN), RX_ISR, CHANGE);
  Timer2.attachInterrupt(TIMEOUT_ISR);
  Timer2.initialize(1000000); // Set to interrupt after 1 second
  
  // Disable Interrupts for now
  noInterrupts();
 */

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
  digitalWrite(TX_PIN,HIGH);
  delay(ON_TIME);
  digitalWrite(TX_PIN,LOW);
  delay(HEAD_BUFFER);
  
  headEnd = timeReg.elapsed();
  Serial.print("  Head:"); Serial.println(headEnd);
  

  // Wait For Second Pulse Position
  delay((i*ON_TIME)+1);

  // At right time, send Second Pulse
  Serial.println("  Second pulse");
  digitalWrite(TX_PIN,HIGH);
  delay(ON_TIME);
  digitalWrite(TX_PIN,LOW);

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

  TX_Transmit(inst1); // transmit higher 8 bits
  upperInstEnd = VPPM_TIME_REG.elapsed();
  Serial.println("  Upper part done.");
  Serial.print("    Full TX took: ");Serial.print(upperInstEnd-protocolStart); Serial.println(" us");
  Serial.println("VPPM Protocol Done");
}

void loop() {
// ONLY MASTER CAN SEND INSTRUCTIONS, ONLY SLAVES CAN SEND ACK
#if (SELF_ID == MASTER)
  // Just the interface...
  if (Serial.available()){
    incomingByte = (char)Serial.read();
    if (incomingByte) {
      Serial.print((char)incomingByte);
    }
    Serial.println(""); 
    
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
      
      case 'z': {TX_Commit(DEV1, MASTER, 255, opLED_ON); break;}      // LED open given 255 value for DT
      case 'p': {printConfig(); break;}                               // Print Configurations (no pulses)
      case 'x': {TX_Transmit(255); break;}                            // Raw send 255 using VPPM (1 phase only)

//      case '0': {TX_Transmit(0); break;}
//      case '1': {TX_Transmit(1); break;}
//      case '2': {TX_Transmit(2); break;}
//      case '3': {TX_Transmit(3); break;}
//      case '4': {TX_Transmit(4); break;}
//      case '5': {TX_Transmit(5); break;}
//      case '6': {TX_Transmit(6); break;}
//      case '7': {TX_Transmit(7); break;}
//      case '8': {TX_Transmit(8); break;}
//      case '9': {TX_Transmit(9); break;}
//      case 'a': {TX_Transmit(10); break;}
//      case 'b': {TX_Transmit(11); break;}
//      case 'c': {TX_Transmit(12); break;}
//      case 'd': {TX_Transmit(13); break;}
//      case 'e': {TX_Transmit(14); break;}
//      case 'f': {TX_Transmit(15); break;}
//      case 'p': {printConfig(); break;}
//      case 'x': {TX_Transmit(255); break;}

      default: ; // Do nothing for NULL string
    }
  }
#endif

#if (SELF_ID == DEV1)

#endif
}

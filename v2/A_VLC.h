#include <stdio.h>
#include <stdlib.h>

// Instruction Set Parameters
#define M 4                         // 2^(M) ie. 16 bits
#define N (2 << M)                  // 16 bits
typedef unsigned short INSTRUCTION; // instruction format
typedef unsigned char BYTE;         // byte format



/*
    Instruction Set Design
    0b [DATA] [SOURCE] [DESTINATION] [OPCODE] [UNUSED]
|       15-8     7-6         5-4        3-1      0

*/

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


// REGISTER FILE
BYTE *RF = 0;


// Physical Implementation Parameters
#define DEV_COUNT 3                 // 3 slave devices
#define MASTER 0                    // master's ID
#define DEV1 1                      // slave 1 ID
#define DEV2 2                      // slave 2 ID
#define DEV3 3                      // slave 3 ID

#define TX_PIN 12                   // Laser pin number
#define RX_PIN 12                   // photodiode / photoresistor pin number
#define LED3 11                     // LED3 pin number
#define LED2 10                     // LED2 pin number
#define LED1 9                      // LED1 pin number
#define LED0 8                      // LED0 pin number
#define BTN1 7                      // BTN1 PIN
#define BTN0 6                      // BTN0 PIN


#define VPPM_0 7                    // VPPM first pulse LED pin
#define VPPM_1 6                    // VPPM data pulse pin



// Transmitter Side

// Returns the instruction to be sent 
INSTRUCTION TX_Encode (BYTE data, BYTE source, BYTE destination, BYTE opcode){
    INSTRUCTION t_data = data << DT_SHFT;
    INSTRUCTION t_source = source << RS_SHFT;
    INSTRUCTION t_destination = destination << RD_SHFT;
    INSTRUCTION t_opcode = opcode << OP_SHFT;

#ifndef DEBUG
    printf("DEBUG:\n");
    printf("    DT: %u\n", data);
    printf("    RS: %u\n", source);
    printf("    RD: %u\n", destination);
    printf("    OP: %u\n\n", opcode);

    printf("    S_DT: %u\n", t_data);
    printf("    S_RS: %u\n", t_source);
    printf("    S_RD: %u\n", t_destination);
    printf("    S_OP: %u\n", t_opcode);

    printf("INST: %u\n",(t_data | t_source | t_destination | t_opcode));

#endif


    return (t_data | t_source | t_destination | t_opcode);
}


// Deccodes the instruction received from TX
void RX_Decode(INSTRUCTION instruction, BYTE *RD, BYTE *RS, BYTE *DT, BYTE *OP){

    unsigned data = (instruction & DT_MASK) >> DT_SHFT;
    unsigned source = (instruction & RS_MASK) >> RS_SHFT;
    unsigned destination = (instruction & RD_MASK) >> RD_SHFT;
    unsigned opcode = (instruction & OP_MASK) >> OP_SHFT;


#ifndef DEBUG
    printf("Received:\n");
    printf("    DT: %u\n", data);
    printf("    RS: %u\n", source);
    printf("    RD: %u\n", destination);
    printf("    OP: %u\n", opcode);
#endif

    // "Return" values to outside
    *OP = opcode;
    *DT = data;
    *RS = source;
    *RD = destination;
}
 

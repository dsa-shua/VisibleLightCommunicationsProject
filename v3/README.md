V3 VLC
- 16 bits in 2 seconds
- VPPM TX done in 2 phases (8-bits per phase) 

Current Implementation
- has instruction set defined
- goes to 2 seconds at 16 bits (confirmed and tested as of Nov 12, 2023)

Instruction Set:

```

opINV 0b000                 // do not use
opREAD 0b001                // reg read         --> [REG number]  [SELF ID] [DEV ID]    [001]
opACK 0b010                 // SEND ACK         --> [-0-]         [SELF ID] [MASTER ID] [010]
opACK_RELAY 0b011           // ACK is relayed   --> [#ID FROM]    [SELF ID] [MASTER ID] [011]
opLED_OFF 0b100             // LED OFF          --> [#LED NUMBER] [SELF ID] [DEV ID]    [100]
opCUSTOM 0b101              // Custom
opLED_ON 0b110              // LED ON           --> [#LED NUMBER] [SELF ID] [DEV ID]    [110] 
opCHAR 0b111                // send "character" --> [ASCII CODE]  [SELF ID] [DEV ID]    [111]

```


<b>Schematic:</b>
<br><br>
<img src="VLC_V3_TX_RX_Schematic.png"/>

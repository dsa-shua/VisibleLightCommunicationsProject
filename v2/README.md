V2 VLC
- improved algorithm using 2 flashes instead of 3
- improved stability by pre-defining on times and cooldowns instead of having a time slice and dividing it according to number of bits

Current Implementation
- has header and tail buffers 
- goes to 1.3 seconds at 8 bits (confirmed and tested as of Nov 10, 2023)
- 10 ms on time
- 10 ms cooldowns
- 5ms on time


<b>Schematic:</b>
<br><br>
<img src="VLC_V2_TX_RX_Schematic.png"/>

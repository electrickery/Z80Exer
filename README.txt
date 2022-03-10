README.txt file for Z80exer 0.3

Changed for 0.3:
- Fixed hardware wiring issue with grounding. 
- Fixed D0 pin number in Z80pins.h. Affects B and W command
- Added this nice README.txt including wiring list.
- Adding a RAM rest checking the databus. Not very reliable with the 
  test machine, a TRS-80 Model I, as the refresh signals (CAS, MUX) are 
  derived from the clock, not the refresh signal. 
- Added Intel-HEX dump

Changed for 0.2:
- fixed the reliability issue while reading memory by adding a delay of
  10 micro second allowing some settle time of the MREQ and RD signal.
  Applied this to memory read, memory fetch, i/o read, refresh.


Wiring the Arduino Mega 2560 R3 to the Z80 socket:

Z80 CPU    Arduino
fn no      no port name
A0 30      22  A   A0
A1 31      23  A   A1
A2 32      24  A   A2
A3 33      25  A   A3
A4 34      26  A   A4
A5 35      27  A   A5
A6 36      28  A   A6
A7 37      29  A   A7

Z80 CPU    Arduino
fn  no     no port name
A8  38     37  C   A8
A9  39     36  C   A9
A10 40     35  C   AA
A11  1     34  C   AB
A12  2     33  C   AC
A13  3     32  C   AD
A14  4     31  C   AE
A15  5     30  C   AF

Z80 CPU    Arduino
fn  no     no port name
D0  14     49  L   D0
D1  15     48  L   D1
D2  12     47  L   D2
D3   8     46  L   D3
D4   7     45  L   D4
D5   9     44  L   D5
D6  10     43  L   D6
D7  13     42  L   D7

Z80 CPU      Arduino
fn    no     no  no port name
WR    22     62  A8  K   WR
RD    21     63  A9  K   RD
MREQ  19     64 A10  K   MQ
IORQ  20     65 A11  K   IQ
M1    27     66 A12  K   M1
RFSH  28     67 A13  K   RF
HALT  18     68 A14  K   HT 
BUSAK 23     69 A15  K   BK

Z80 CPU      Arduino
fn    no     no  no port name
INT   16     57  A3  F   IN
NMI   17     58  A4  F   NM
WAIT  24     59  A5  F   WT
BUSRQ 25     60  A6  F   BQ
RESET 26     61  A7  F   RT

Z80 CPU   Arduino
fn   no   no  name
GND  29   -   GND

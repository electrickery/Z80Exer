/*
Z80emu - Z80 pin exorsizer
*/

// Modified by David Mutimer for the Z80 & TRS-80 model 1 shield

#include "Z80pins.h"
#include "TRS80data.h"

#define READ_DELAY            10
#define WRITE_DELAY           10
#define SERIALBUFSIZE         50
char serialBuffer[SERIALBUFSIZE];
byte setBufPointer = 0;

#define LED 13

uint16_t repeatRate = 1 < 9;
#define RECORDSIZE 16
#define DATARECORDTYPE 0

#define DUMPPAGE 0x0100
unsigned int lastEndAddress = 0;

unsigned int addressOffset = 0;
bool refreshMode = 1;
unsigned int refreshAddress = 0;
const char* VERSION  = "v0.7";

// core routines

void setup() {
  Serial.begin(9600);
  Serial.print("Z80 & TRS-80 Model 1 exerciser ");
  Serial.println(VERSION);
  
  pinMode(LED, OUTPUT);
  pinMode(Z80INT,   INPUT);
  pinMode(Z80NMI,   INPUT);
  pinMode(Z80WAIT,  INPUT);
  pinMode(Z80BUSRQ, INPUT);
  pinMode(Z80RESET, INPUT); 
  pinMode(EXPDETECT, INPUT); 
  digitalWrite(Z80INT,   HIGH);
  digitalWrite(Z80NMI,   HIGH);
  digitalWrite(Z80WAIT,  HIGH);
  digitalWrite(Z80BUSRQ, HIGH);
  digitalWrite(Z80RESET, HIGH);
  digitalWrite(EXPDETECT,HIGH);

  pinMode(EXPMRD, OUTPUT); 
  pinMode(EXPMWR, OUTPUT); 
  pinMode(EXPRAS, OUTPUT); 
  pinMode(EXPMUX, OUTPUT); 
  pinMode(EXPCAS, OUTPUT); 
  pinMode(EXPIN, OUTPUT); 
  pinMode(EXPOUT, OUTPUT); 
  digitalWrite(EXPMRD ,HIGH);
  digitalWrite(EXPMWR ,HIGH);
  digitalWrite(EXPRAS ,HIGH);
  digitalWrite(EXPMUX ,HIGH);
  digitalWrite(EXPCAS ,HIGH);
  digitalWrite(EXPIN ,HIGH);
  digitalWrite(EXPOUT ,HIGH);
  digitalWrite(EXPIAK ,HIGH);

  DDRA  = 0XFF; // address LSB output
  DDRC  = 0XFF; // address MSB output
  PORTK = 0xFF; // control out bits high
  DDRK  = 0XFF; // control out bits output
  DDRL  = 0x00; // data bus input
  PORTL = 0xFF; // data bus pull ups

  
//  onlineReadMode();

  delay(250);  
  if (digitalRead(EXPDETECT)) {
    Serial.println("Using CPU interface, please unplug the expansion header cable");
  }
  else {
    Serial.println("Using expansion interface, please unplug the CPU header cable");
  }
  delay(500);  
}


void loop() {
  commandCollector();
  if (refreshMode) {
    delay(20);
    refreshRow();
  }
}  
 
void clearSerialBuffer() {
  byte i;
  for (i = 0; i < SERIALBUFSIZE; i++) {
    serialBuffer[i] = 0;
  }
}

void commandCollector() {
  if (Serial.available() > 0) {
    int inByte = Serial.read();
    switch(inByte) {
    case '.':
//    case '\r':
    case '\n':
      commandInterpreter();
      clearSerialBuffer();
      setBufPointer = 0;
      break;
    case '\r':
      break;  // ignore carriage return
    default:
      serialBuffer[setBufPointer] = inByte;
      setBufPointer++;
      if (setBufPointer >= SERIALBUFSIZE) {
        Serial.println("Serial buffer overflow. Cleanup.");
        clearSerialBuffer();
        setBufPointer = 0;
      }
    }
  }
}

void commandInterpreter() {
  byte bufByte = serialBuffer[0];
  
  switch(bufByte) {
    case 'A':
    case 'a':
      setAddress();
      break;
    case 'B':
    case 'b':
      blinkPin();
      break;
    case 'C':
    case 'c':
      calcChecksum();
      break;
    case 'D':  // dump memory
    case 'd':
      dumpMemory();
      break;
    case 'H':  // help
    case 'h':
    case '?':  // help
//      Serial.println("F?:");
      usage();
      break; 
    case 'I':
    case 'i':
      generateDataRecords();
      generateEndRecord();
      break;
    case 'M':  // memory address read/write
    case 'm':
      readWriteMemory();
      break; 
    case 'P':  // I/O port operations
    case 'p':
      inputOutputPort();
      break; 
    case 'Q':  // repeatRate
    case 'q':
      setRepeatRate();
      break; 
    case 'R':
    case 'r':
      if (serialBuffer[1] == '+') {
        refreshMode = 1;
      } else if (serialBuffer[1] == '-') {
        refreshMode = 0;
      }
      if (refreshMode) {
        Serial.println("refresh on");
      }
      else {
        Serial.println("refresh off");
      }
      

      break;
    case 'S':
    case 's':
      setValue(); // fill memory range with a value
      break;
    case 'T':  // TRS-80 tests
    case 't':
      TRS80Test();
      break;
    case 'U':
    case 'u':
      testRAM();
      break;
    case 'V':
    case 'v':
      viewPorts();
      break;
    case 'W':
    case 'w':
      writePin();
      break;
    case 'X':  // test ports
    case 'x':
      portTest(serialBuffer[1]);
      break;
    default:
      Serial.print(bufByte);
      Serial.print(" ");
      Serial.println("unsupported");
      return;
  }
}

int getNibble(unsigned char myChar) {
  int nibble = myChar;
  if (nibble > 'F') nibble -= ' ';  // lower to upper case
  nibble -= '0';
  if (nibble > 9) nibble -= 7; // offset 9+1 - A
  return nibble;
}

void dumpMemory() {
  unsigned long startAddress;
  unsigned long endAddress;
  bool repeatMode = 0;
  if (setBufPointer == 1 ) {
    startAddress = lastEndAddress;
    endAddress   = startAddress + DUMPPAGE;
    lastEndAddress = endAddress;
  } else if (setBufPointer == 2 && serialBuffer[1] == '+') {
    startAddress = lastEndAddress - DUMPPAGE;
    endAddress   = startAddress + DUMPPAGE;
    lastEndAddress = endAddress;
    repeatMode = 1;
  } else if (setBufPointer == 5) {
    startAddress  = getNibble(serialBuffer[1]) * (1 << 12);
    startAddress += getNibble(serialBuffer[2]) * (1 << 8);
    startAddress += getNibble(serialBuffer[3]) * (1 << 4);
    startAddress += getNibble(serialBuffer[4]);
    endAddress   = startAddress + DUMPPAGE;
    lastEndAddress = endAddress;
  } else if (setBufPointer == 10) {
    startAddress  = getNibble(serialBuffer[1]) * (1 << 12);
    startAddress += getNibble(serialBuffer[2]) * (1 << 8);
    startAddress += getNibble(serialBuffer[3]) * (1 << 4);
    startAddress += getNibble(serialBuffer[4]);
    endAddress  = getNibble(serialBuffer[6]) * (1 << 12);
    endAddress += getNibble(serialBuffer[7]) * (1 << 8);
    endAddress += getNibble(serialBuffer[8]) * (1 << 4);
    endAddress += getNibble(serialBuffer[9]);
    lastEndAddress = endAddress;
    endAddress++;
  } else {
    Serial.println("unsupported"); 
  }
  unsigned char asChars[17];
  unsigned char *asCharsP = &asChars[0];
  unsigned char positionOnLine;
  asChars[16] = 0;
  do {
    printWord(startAddress);
    Serial.print("-");
    printWord(endAddress-1);
    Serial.println();
    unsigned long i;
    unsigned int data;
    dataBusReadMode();
    for (i = startAddress; i < endAddress; i++) {
      positionOnLine = i & 0x0F;
      if (positionOnLine == 0) {
        printWord(i);   // Address at start of line
        Serial.print(": ");
      }
      data = readByte(i);
      printByte(data);   // actual value in hex
      // fill an array with the ASCII part of the line
      asChars[positionOnLine] = (data >= ' ' && data <= '~') ? data : '.';
      if ((i & 0x03) == 0x03) Serial.print(" ");
      if ((i & 0x0F) == 0x0F) {
        Serial.print (" ");
        printString(asCharsP); // print the ASCII part
        Serial.println("");
      }
    }
    Serial.println();
    delay(repeatRate);
    if (Serial.available() > 0) {
      clearSerialBuffer();
      setBufPointer = 0;
      repeatMode = 0;
      return;
    }
  } while (repeatMode);
}

unsigned int readByte(unsigned int address) {
  unsigned int data = 0;
  unsigned int addressLSB = address & 0xFF;
  unsigned int addressMSB = address >> 8;
  dataBusReadMode();
  PORTL = 0xFF; // enable pull ups
  PORTA = addressLSB;
  PORTC = addressMSB;
  if (digitalRead(EXPDETECT)) {
    digitalWrite(Z80MREQ, LOW);
    digitalWrite(Z80RD,   LOW);
    delayMicroseconds(READ_DELAY);
    data = PINL;
    digitalWrite(Z80RD,   HIGH);
    digitalWrite(Z80MREQ, HIGH); 
  }
  else {
    digitalWrite(EXPRAS, LOW);
    digitalWrite(EXPMRD, LOW);
    digitalWrite(EXPMUX, LOW);
    digitalWrite(EXPCAS, LOW);
    delayMicroseconds(READ_DELAY);
    data = PINL;
    digitalWrite(EXPMRD, HIGH);
    digitalWrite(EXPRAS, HIGH);
    digitalWrite(EXPMUX, HIGH);
    digitalWrite(EXPCAS, HIGH);
  }
  refreshRow();
  return data; 
}

unsigned int fetchByte(unsigned int address) {
  unsigned int data = 0;
  unsigned int addressLSB = address & 0xFF;
  unsigned int addressMSB = address >> 8;
  dataBusReadMode();
  PORTL = 0xFF; // enable pull ups
  PORTA = addressLSB;
  PORTC = addressMSB;
  if (digitalRead(EXPDETECT)) {
    digitalWrite(Z80M1  , LOW);
    digitalWrite(Z80MREQ, LOW);
    digitalWrite(Z80RD,   LOW);
    delayMicroseconds(READ_DELAY);
    data = PINL;
    digitalWrite(Z80RD,   HIGH);
    digitalWrite(Z80MREQ, HIGH); 
    digitalWrite(Z80M1  , HIGH);
  }
  else {
   Serial.print ("*");
    digitalWrite(EXPMRD, LOW);
    digitalWrite(EXPRAS, LOW);
    digitalWrite(EXPMUX, LOW);
    digitalWrite(EXPCAS, LOW);
    delayMicroseconds(READ_DELAY);
    data = PINL;
    digitalWrite(EXPMRD, HIGH);
    digitalWrite(EXPRAS, HIGH);
    digitalWrite(EXPMUX, HIGH);
    digitalWrite(EXPCAS, HIGH);
  }
  refreshRow();
  return data; 
}

void refreshRow() {
  PORTA = refreshAddress & 0x7F;
  if (digitalRead(EXPDETECT)) {
    digitalWrite(Z80RFSH, LOW);
    digitalWrite(Z80MREQ, LOW);
    delayMicroseconds(READ_DELAY);
    digitalWrite(Z80MREQ, HIGH);  
    digitalWrite(Z80RFSH, HIGH);
  }
  else {
    digitalWrite(EXPRAS, LOW);
    delayMicroseconds(READ_DELAY);
    digitalWrite(EXPRAS, HIGH);
  }
  refreshAddress++;
}

unsigned int inputByte(unsigned int address) {
  unsigned int data = 0;
  unsigned int addressLSB = address & 0xFF;
  unsigned int addressMSB = address >> 8;
  dataBusReadMode();
  PORTL = 0xFF; // enable pull ups
  PORTA = addressLSB;
  PORTC = addressMSB;
  if (digitalRead(EXPDETECT)) {
    digitalWrite(Z80IORQ, LOW);
    digitalWrite(Z80RD,   LOW);
    delayMicroseconds(READ_DELAY);
    data = PINL;
    digitalWrite(Z80RD,   HIGH);
    digitalWrite(Z80IORQ, HIGH); 
  }
  else {
    digitalWrite(EXPIN, LOW);
    delayMicroseconds(READ_DELAY);
    data = PINL;
    digitalWrite(EXPIN, HIGH); 
  }
  refreshRow();
  return data; 
}


void writeByte(unsigned int address, unsigned int value) {
  unsigned int addressLSB = address & 0xFF;
  unsigned int addressMSB = address >> 8;
  dataBusWriteMode();
  PORTA = addressLSB;
  PORTC = addressMSB;
  PORTL = value;
  if (digitalRead(EXPDETECT)) {
    digitalWrite(Z80MREQ, LOW);
    digitalWrite(Z80WR,   LOW);
    delayMicroseconds(WRITE_DELAY);
    digitalWrite(Z80WR,   HIGH);
    digitalWrite(Z80MREQ, HIGH);
  }
  else {
    digitalWrite(EXPRAS, LOW);
    digitalWrite(EXPMUX, LOW);
    digitalWrite(EXPCAS, LOW);
    digitalWrite(EXPMWR,   LOW);
    delayMicroseconds(WRITE_DELAY);
    digitalWrite(EXPMWR, HIGH);
    digitalWrite(EXPRAS, HIGH);
    digitalWrite(EXPMUX, HIGH);
    digitalWrite(EXPCAS, HIGH);
  }
  dataBusReadMode();
  refreshRow();
}

void outputByte(unsigned int address, unsigned int value) {
  unsigned int addressLSB = address & 0xFF;
  unsigned int addressMSB = address >> 8;
  dataBusWriteMode();
  PORTA = addressLSB;
  PORTC = addressMSB;
  PORTL = value;
  if (digitalRead(EXPDETECT)) {
    digitalWrite(Z80IORQ, LOW);
    digitalWrite(Z80WR,   LOW);
    delayMicroseconds(WRITE_DELAY);
    digitalWrite(Z80WR,   HIGH);
    digitalWrite(Z80IORQ, HIGH);
  }
  else {
    digitalWrite(EXPOUT,   LOW);
    delayMicroseconds(WRITE_DELAY);
    digitalWrite(EXPOUT, HIGH);
  }
  dataBusReadMode();
  refreshRow();
}

void printByte(unsigned char data) {
  unsigned char dataMSN = data >> 4;
  unsigned char dataLSN = data & 0x0F;
  Serial.print(dataMSN, HEX);
  Serial.print(dataLSN, HEX);
}

void printWord(unsigned int data) {
  printByte(data >> 8);
  printByte(data & 0xFF);
}

void dataBusReadMode() {
  DDRL  = 0x00;  // read mode
}

void dataBusWriteMode() {
  DDRL  = 0xFF;  // write mode
}

void usage() {
  Serial.print("-- Z80 exerciser ");
  Serial.print(VERSION);
  Serial.println(" command set --");
  Serial.println("Aaaaa            - set address bus to value aaaa");
  Serial.println("Bpp or B#ss      - blink pin p (in hex) or symbol: A0-AF,D0-D7,RD,WR.MQ,IQ,M1,RF,HT,BK");
  Serial.println("                 - ( Model 1 expansion interface - MR, MW, IN, OT, RS, MX, CS and IA )");
  Serial.println("Cssss-eeee       - Calculate checksum from ssss to eeee");
  Serial.println("D[ssss[-eeee]|+] - Dump memory from ssss to eeee (default 256 bytes)");
  Serial.println("H                - This help text");
  Serial.println("Issss-eeee       - Generate hex intel data records");
  Serial.println("MRaaaa[+]        - Read memory address aaaa, optionally repeating");
  Serial.println("MWaaaa vv[+]     - Write vv to address aaaa, optionally repeating");
  Serial.println("PRaa[+]          - Read port address [aa]aa, optionally repeating");
  Serial.println("PWaa:vv[+]       - Write vv to address [aa]aa, optionally repeating");
  Serial.println("R[+|-]           - Refresh on/off");
  Serial.println("Qn               - Repeat rate; 1, 2, 4, 8, 16, ..., 32678 ms (n=0-9,A-F)");
  Serial.println("Sssss-eeee:vv    - fill a memory range with a value");
  Serial.println("TC               - Calculate TRS-80 ROM checksums");
  Serial.println("TD               - Exercise TRS-80 display memory (No lower case mod)");
  Serial.println("TL               - Exercise TRS-80 display memory (With lower case mod)");
  Serial.println("TK               - Reads TRS-80 keyboard matrix");
  Serial.println("Ussss-eeee       - test RAM range (walking 1s)");
  Serial.println("V                - view data bus, pins INT, NMI, WAIT, BUSRQ, RESET");
  Serial.println("Wpp v or W#ss v  - Write pin (in hex) or symbol: A0-AF,D0-D7,RD,WR.MQ,IQ,M1,RF,HT,BK; values 0, 1");
  Serial.println("                 - ( Model 1 expansion interface - MR, MW, IN, OT, RS, MX, CS and IA )");
  Serial.println("Xp               - exercise port p");
  Serial.println("?                - This help text"); 
}

void portTest(byte port) {
  byte i = 0;
  switch(port) {
    case 'A':
    case 'a':
      Serial.println("Testing PORTA");
      PORTA = 0xFF;
      DDRA  = 0xFF;
      while (1) {
        PORTA = i++;
        i &= 0xFF;
        delay(repeatRate);
        if (stopIt()) {
          return;
        }
      }
    break; 
    case 'C':
    case 'c':
      Serial.println("Testing PORTC");
      PORTC = 0xFF;
      DDRC  = 0xFF;
      while (1) {
        PORTC = i++;
        i &= 0xFF;
        delay(repeatRate);
        if (stopIt()) {
          return;
        }
      }
    break;
    case 'D':
    case 'd':
      Serial.println("Testing PORTD");
      PORTD = 0xFF;
      DDRD  = 0xFF;
      while(1) {
        PORTD = i++;
        i &= 0xFF; 
        delay(repeatRate);
        if (stopIt()) {
          return;
        }
      }
    break;
    case 'L':
    case 'l':    
      Serial.println("Testing PORTL");
      PORTL = 0xFF;
      DDRL  = 0xFF;
      while(1) {
        PORTL = i++; 
        i &= 0xFF;
        delay(repeatRate);
        if (stopIt()) {
          return;
        }
      }
    break;
    case 'H':
    case 'h':
      Serial.println("Testing PORTH");
      PORTH = 0xFF;
      DDRH  = 0xFF;
      while(1) {
        PORTH = i++;
        i &= 0xFF; 
        delay(repeatRate);
        if (stopIt()) {
          return;
        }
      }
    break;
    case 'K':
    case 'k':
      Serial.println("Testing PORTK");
      PORTK = 0xFF;
      DDRK  = 0xFF;
      while(1) {
        PORTK = i++;
        i &= 0xFF; 
        delay(repeatRate);
        if (stopIt()) {
          return;
        }
      }
    break;
    default:
    Serial.println("Unknown PORT");
    return;
   }
}

int getPinMode(uint8_t pin)
{
  if (pin >= NUM_DIGITAL_PINS) return (-1);

  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *reg = portModeRegister(port);
  if (*reg & bit) return (OUTPUT);

  volatile uint8_t *out = portOutputRegister(port);
  return ((*out & bit) ? INPUT_PULLUP : INPUT);
}

void blinkPin() {
  int pin;
  int mode;
  int value;
   
  if (serialBuffer[1] == '#') {
    pin = getPinBySymbol();    
  } else {
    pin  = getNibble(serialBuffer[1]) * (1 << 4);
    pin += getNibble(serialBuffer[2]);
  }
  mode = getPinMode(pin);
  value = digitalRead(pin);
  Serial.print("Pin: ");
  Serial.println(pin, DEC);  
  pinMode(pin, OUTPUT);
  while(1) {
    digitalWrite(pin, !digitalRead(pin));
    delay(repeatRate);
    if (Serial.available() > 0) {
        clearSerialBuffer();
        setBufPointer = 0;
        pinMode(pin,mode);
        if(mode==OUTPUT)
          digitalWrite(pin, value);
        return;
    }
  }
}

void calcChecksum() {
  unsigned int startAddress;
  unsigned int endAddress;
  unsigned char value;
  // Cssss eeee
  startAddress  = getNibble(serialBuffer[1]) * (1 << 12);
  startAddress += getNibble(serialBuffer[2]) * (1 << 8);
  startAddress += getNibble(serialBuffer[3]) * (1 << 4);
  startAddress += getNibble(serialBuffer[4]);
  endAddress  = getNibble(serialBuffer[6]) * (1 << 12);
  endAddress += getNibble(serialBuffer[7]) * (1 << 8);
  endAddress += getNibble(serialBuffer[8]) * (1 << 4);
  endAddress += getNibble(serialBuffer[9]);

  Serial.print("Checksum block ");
  Serial.print(startAddress, HEX);
  Serial.print("h - ");
  Serial.print(endAddress, HEX);
  Serial.print("h : ");
  Serial.print(blockChecksum(startAddress,endAddress), HEX);
  Serial.println("h");
  dataBusReadMode();
}

void setValue() {
  unsigned int startAddress;
  unsigned int endAddress;
  unsigned char value;
  // Sssss eeee vv
  startAddress  = getNibble(serialBuffer[1]) * (1 << 12);
  startAddress += getNibble(serialBuffer[2]) * (1 << 8);
  startAddress += getNibble(serialBuffer[3]) * (1 << 4);
  startAddress += getNibble(serialBuffer[4]);
  endAddress  = getNibble(serialBuffer[6]) * (1 << 12);
  endAddress += getNibble(serialBuffer[7]) * (1 << 8);
  endAddress += getNibble(serialBuffer[8]) * (1 << 4);
  endAddress += getNibble(serialBuffer[9]);
  value  = getNibble(serialBuffer[11]) * (1 << 4);
  value += getNibble(serialBuffer[12]);
  Serial.print("Writing ");
  Serial.print(value, HEX);
  Serial.print(" to range ");
  Serial.print(startAddress, HEX);
  Serial.print(" - ");
  Serial.println(endAddress, HEX);
  dataBusWriteMode();
  unsigned long i;
  for (i = startAddress; i <= endAddress; i++) {
    writeByte(i, value);
  }
  dataBusReadMode();
  Serial.println("set RAM done.");
}

void viewPorts() {
  Serial.println("  Data   IN NM WT BK RT");
  while(1) {
    printBin(PINL);
    Serial.print("  ");
    Serial.print(digitalRead(Z80INT),   BIN);
    Serial.print("  ");
    Serial.print(digitalRead(Z80NMI),   BIN);
    Serial.print("  ");
    Serial.print(digitalRead(Z80WAIT),  BIN);
    Serial.print("  ");
    Serial.print(digitalRead(Z80BUSRQ), BIN);
    Serial.print("  ");
    Serial.print(digitalRead(Z80RESET), BIN);
    Serial.println();
    delay(repeatRate);
    if (Serial.available() > 0) {
        clearSerialBuffer();
        setBufPointer = 0;
        return;
    }
  } 
}

void writePin() {
  unsigned char pin;
  bool value;
  Serial.print("Setting pin ");
  if (serialBuffer[1] == '#') {
    pin = getPinBySymbol();    
    value = (serialBuffer[5] == 0x30);
    Serial.print(serialBuffer[2]);
    Serial.print(serialBuffer[3]);
    Serial.print(" (");
    Serial.print(pin, DEC);
    Serial.print(") to ");
    Serial.println(serialBuffer[5]);
  } else {
    pin  = getNibble(serialBuffer[1]) * (1 << 4);
    pin += getNibble(serialBuffer[2]);
    value = (serialBuffer[4] == 0x30);
    Serial.print(serialBuffer[1]);
    Serial.print(serialBuffer[2]);
    Serial.print(" (");
    Serial.print(pin, DEC);
    Serial.print(") to ");
    Serial.println(serialBuffer[4]);    
  }
//Serial.print(pin, HEX);
//Serial.print("  ");
//Serial.println(serialBuffer[1], HEX);
  digitalWrite(pin, value ? 0 : 1);
}

unsigned char getPinBySymbol() {
  // B[0-F][0-F], B#A[0-F], B#D[0-7], B#RW, B#RD, B#MQ, B#IQ, B#MI, B#RF, B#HT, B#BK
  // B#MRD B#MWR B#IN B#OUT B#RAS B#MUX B#CAS B#IAK
  if (serialBuffer[2] == 'A') {
    if(serialBuffer[3] <= '7') {
      return serialBuffer[3] - '0' + Z80A0;
    } else {
      int i = serialBuffer[3] - '8';
      i = Z80A8 - i;
      return (i <= 28) ? i + 7 : i;
    }
  } else if (serialBuffer[2] == 'D') {
    int i = serialBuffer[3] - '0';
    return Z80D0 - i;
  } else if (serialBuffer[2] == 'W' && serialBuffer[3] == 'R') {
    return Z80WR;
  } else if (serialBuffer[2] == 'R' && serialBuffer[3] == 'D') {
    return Z80RD;
  } else if (serialBuffer[2] == 'M' && serialBuffer[3] == 'Q') {
    return Z80MREQ;
  } else if (serialBuffer[2] == 'I' && serialBuffer[3] == 'Q') {
    return Z80IORQ;
  } else if (serialBuffer[2] == 'M' && serialBuffer[3] == '1') {
    return Z80M1;
  } else if (serialBuffer[2] == 'R' && serialBuffer[3] == 'F') {
    return Z80RFSH;
  } else if (serialBuffer[2] == 'H' && serialBuffer[3] == 'T') {
    return Z80HALT;
  } else if (serialBuffer[2] == 'B' && serialBuffer[3] == 'K') {
    return Z80BUSAK;
  } else if (serialBuffer[2] == 'M' && serialBuffer[3] == 'R') {
    return EXPMRD;
  } else if (serialBuffer[2] == 'M' && serialBuffer[3] == 'W') {
    return EXPMWR;
  } else if (serialBuffer[2] == 'I' && serialBuffer[3] == 'N') {
    return EXPIN;
  } else if (serialBuffer[2] == 'O' && serialBuffer[3] == 'T') {
    return EXPOUT;
  } else if (serialBuffer[2] == 'R' && serialBuffer[3] == 'S') {
    return EXPRAS;
  } else if (serialBuffer[2] == 'M' && serialBuffer[3] == 'X') {
    return EXPMUX;
  } else if (serialBuffer[2] == 'C' && serialBuffer[3] == 'S') {
    return EXPCAS;
  } else if (serialBuffer[2] == 'I' && serialBuffer[3] == 'A') {
    return EXPIAK;
  } else {
    Serial.println("unknown symbol");
  }
  return 0;
}

void printBin(unsigned char value) {
  Serial.print((value & 0b10000000) ? "1" : "0");
  Serial.print((value & 0b01000000) ? "1" : "0");
  Serial.print((value & 0b00100000) ? "1" : "0");
  Serial.print((value & 0b00010000) ? "1" : "0");
  Serial.print((value & 0b00001000) ? "1" : "0");
  Serial.print((value & 0b00000100) ? "1" : "0");
  Serial.print((value & 0b00000010) ? "1" : "0");
  Serial.print((value & 0b00000001) ? "1" : "0");  
}

void printString(unsigned char *asCharP) {
  unsigned char i = 0;
  while(asCharP[i] != 0) {
    Serial.write(asCharP[i]); 
//    Serial.print(asCharP[i], HEX);
    i++;
  }
}

void readWriteMemory() {
  // MRaaaa[+], MWaaaa vv[+]
  uint16_t address;
  bool repeatMode = 0;
  address  = getNibble(serialBuffer[2]) * (1 << 12);
  address += getNibble(serialBuffer[3]) * (1 << 8);
  address += getNibble(serialBuffer[4]) * (1 << 4);
  address += getNibble(serialBuffer[5]);
  if (serialBuffer[1] == 'R' || serialBuffer[1] == 'r') {
    if (setBufPointer == 7 && serialBuffer[6] == '+') {
      repeatMode = 1;
    }
    dataBusReadMode();
    do {
      Serial.print("MRD ");
      Serial.print(address, HEX);
      Serial.print(": ");
      Serial.println(readByte(address), HEX);
      if (stopIt()) {
        return;
      }
      delay(repeatRate);
    } while (repeatMode);
  } else if (serialBuffer[1] == 'W' || serialBuffer[1] == 'w') {
    uint8_t data;
    data  = getNibble(serialBuffer[7]) * (1 << 4);
    data += getNibble(serialBuffer[8]);
    if (setBufPointer == 10 && serialBuffer[9] == '+') {
      repeatMode = 1;
    }
    dataBusWriteMode();
    do {
      Serial.print("MWR ");
      Serial.print(address, HEX);
      Serial.print(": ");
      Serial.println(data, HEX);
      writeByte(address, data);
      if (stopIt()) {
        return;
      }
      delay(repeatRate);  
    } while (repeatMode);
    dataBusReadMode();
  } else {
    Serial.print("not supported");
    return; 
  }
}

void inputOutputPort() {
  // PRaa+, PRaaaa[+], PWaa vv[+], PWaaaa vv[+]
  uint16_t address;
  bool repeatMode = 0;
  if (serialBuffer[1] == 'R' || serialBuffer[1] == 'r') {
    if (setBufPointer == 5 || serialBuffer[4] == '+') {
      repeatMode = 1;
    }
    address += getNibble(serialBuffer[2]) * (1 << 4);
    address += getNibble(serialBuffer[3]);
    dataBusReadMode();
    do {
      Serial.print("IO RD ");
      Serial.print(address, HEX);
      Serial.print(": ");
      Serial.println(inputByte(address), HEX);
      if (stopIt()) {
        return;
      }
      delay(repeatRate);  
    } while (repeatMode);
  } else if (serialBuffer[1] == 'W' || serialBuffer[1] == 'w') {
    if (setBufPointer == 8 || serialBuffer[7] == '+') {
      repeatMode = 1;
    }      
    uint8_t data;
    address += getNibble(serialBuffer[2]) * (1 << 4);
    address += getNibble(serialBuffer[3]);
    data  = getNibble(serialBuffer[5]) * (1 << 4);
    data += getNibble(serialBuffer[6]);
    dataBusWriteMode();
    do {
      outputByte(address, data);
      Serial.print("IO WR ");
      Serial.print(address, HEX);
      Serial.print(": ");
      Serial.println(data, HEX);
      if (stopIt()) {
        return;
      }
      delay(repeatRate);  
    } while (repeatMode);
    dataBusReadMode();
  } else {
    Serial.print("not supported");
    return; 
  }
}

bool stopIt() {
  if (Serial.available() > 0) {
    clearSerialBuffer();
    setBufPointer = 0;
    return true;
  }
  return false;
}

void setRepeatRate() {
  if (setBufPointer == 2) {
    byte value = serialBuffer[1] - '0';
    value = (value > 9) ? (value - 7) : value;
    value = (value > 15) ? (value - 32) : value; // handle lower case
    Serial.println(value);
    repeatRate = 1 << value;
    Serial.print("Repeat rate set to ");
    Serial.print(repeatRate, DEC);
    Serial.println(" ms");
    
  } else {
    Serial.println("not supported");
  }
}

byte testRAM() {
  unsigned int startAddress;
  unsigned int endAddress;
  unsigned char value;
  // Ussss eeee
  startAddress  = getNibble(serialBuffer[1]) * (1 << 12);
  startAddress += getNibble(serialBuffer[2]) * (1 << 8);
  startAddress += getNibble(serialBuffer[3]) * (1 << 4);
  startAddress += getNibble(serialBuffer[4]);
  endAddress  = getNibble(serialBuffer[6]) * (1 << 12);
  endAddress += getNibble(serialBuffer[7]) * (1 << 8);
  endAddress += getNibble(serialBuffer[8]) * (1 << 4);
  endAddress += getNibble(serialBuffer[9]);
  Serial.print("Testing RAM range ");
  Serial.print(startAddress, HEX);
  Serial.print(" - ");
  Serial.println(endAddress, HEX);
  dataBusReadMode();
  unsigned long i;
  byte result;
  for (i = startAddress; i <= endAddress; i++) {
    result = memTestDataBus(i);
    if (result) {
      Serial.print("Failed with pattern ");
      printBin(result);
      Serial.print(" at ");
      Serial.println(i, HEX);
    }
    if (stopIt()) break;
  }
  dataBusReadMode();
  Serial.println("RAM test done.");
}

void setAddress() {
  // Aaaaa
  uint16_t address;
  bool repeatMode = 0;
  address  = getNibble(serialBuffer[1]) * (1 << 12);
  address += getNibble(serialBuffer[2]) * (1 << 8);
  address += getNibble(serialBuffer[3]) * (1 << 4);
  address += getNibble(serialBuffer[4]);

  unsigned int addressLSB = address & 0xFF;
  unsigned int addressMSB = address >> 8;
  PORTA = addressLSB;
  PORTC = addressMSB;
  Serial.print("A:");
  Serial.println(address,HEX);
}

/*
  Note the RAM test is not very relable if the Arduino write and read cycle is 
  not syncronised with the machine refresh circuit. This will happen if it relies
  on the processor clock directly. It should work when the REFRESH signal is used
  (and enabled).
*/
/**********************************************************************
 *
 * Function:    memTestDataBus()
 *
 * Description: Test the data bus wiring in a memory region by
 *              performing a walking 1's test at a fixed address
 *              within that region.  The address (and hence the
 *              memory region) is selected by the caller.
 *
 * Notes:       
 *
 * Returns:     0 if the test succeeds.  
 *              A non-zero result is the first pattern that failed.
 *
 * http://www.barrgroup.com/Embedded-Systems/How-To/Memory-Test-Suite-C
 *
 **********************************************************************/
byte memTestDataBus(uint16_t address)
{
    byte pattern;


    /*
     * Perform a walking 1's test at the given address.
     */
    for (pattern = 1; pattern != 0; pattern <<= 1)
    {
        /*
         * Write the test pattern.
         */
        writeByte(address, pattern);

        /*
         * Read it back (immediately is okay for this test).
         */
        if (readByte(address) != pattern) 
        {
            return (pattern);
        }
    }
    return (0);

}   /* memTestDataBus() */

void generateDataRecords() {
  unsigned int startAddress;
  unsigned int endAddress;
  startAddress  = getNibble(serialBuffer[1]) * (1 << 12);
  startAddress += getNibble(serialBuffer[2]) * (1 << 8);
  startAddress += getNibble(serialBuffer[3]) * (1 << 4);
  startAddress += getNibble(serialBuffer[4]);
  endAddress  = getNibble(serialBuffer[6]) * (1 << 12);
  endAddress += getNibble(serialBuffer[7]) * (1 << 8);
  endAddress += getNibble(serialBuffer[8]) * (1 << 4);
  endAddress += getNibble(serialBuffer[9]);
  printWord(startAddress);
  Serial.print("-");
  printWord(endAddress);
  Serial.println();

  unsigned long i;
  unsigned int j;
  unsigned char addressMSB, addressLSB, data;
  unsigned char sumCheckCount = 0;

  dataBusReadMode();  
  for (i = startAddress; i < endAddress; i += RECORDSIZE) {
    sumCheckCount = 0;
    Serial.print(":");
    printByte(RECORDSIZE);  
    sumCheckCount -= RECORDSIZE;
    addressMSB = i >> 8;
    addressLSB = i & 0xFF;
    printByte(addressMSB);
    printByte(addressLSB);
    sumCheckCount -= addressMSB;
    sumCheckCount -= addressLSB;
    printByte(DATARECORDTYPE);
    sumCheckCount -= DATARECORDTYPE;
    for (j = 0; j < RECORDSIZE; j++) {
      data = readByte(i + j);
      printByte(data);
      sumCheckCount -= data;
    }
    printByte(sumCheckCount);
    Serial.println();
  }
}

void generateEndRecord() {
  Serial.println(":00000001FF");
}

void TRS80Test() {  
  if (serialBuffer[1] == 'D' || serialBuffer[1] == 'd') { // Test TRS-80 Display 
    TRS80DisplayTest();
    return;
  } else if (serialBuffer[1] == 'L' || serialBuffer[1] == 'l') { // Test TRS-80 Keyboard
    TRS80LCDisplayTest();
    return;
  } else if (serialBuffer[1] == 'K' || serialBuffer[1] == 'k') { // Test TRS-80 Keyboard
    TRS80KeyboardTest();
    return;
  } if (serialBuffer[1] == 'C' || serialBuffer[1] == 'c') { // TRS-80 ROM cheksum
    TRS80ROMChecksum();
    return;   Serial.println("Not supported");
  }
}

unsigned int blockChecksum(unsigned long startAddress, unsigned long endAddress)
{
  unsigned long checksum = 0;
  for (unsigned long i=startAddress; i<=endAddress; i++) {
    checksum += readByte(i);  
  }
  return checksum;
}

void TRS80ROMChecksum() {
  unsigned int checksum1 = 0;
  unsigned int checksum2 = 0;
  unsigned int checksum3 = 0;
  int  match = 0;
  const char *rs[10]= {"Unknown or faulty ROM set",
                      "TRS-80 v1.0",
                      "TRS-80 v1.1",
                      "TRS-80 v1.2", 
                      "TRS-80 v1.31", 
                      "TRS-80 v1.32",
                      "HT-10Z v2.2",
                      "LNW-80",
                      "Dick Smith System 80",
                      "Dick Smith System 80 (Patched)"};

  checksum1 = blockChecksum(0x0000, 0x0fff);
  checksum2 = blockChecksum(0x1000, 0x1fff);
  checksum3 = blockChecksum(0x2000, 0x2fff);
  
  Serial.print("ROM 1 0000h - 0fffh : ");
  Serial.println(checksum1, HEX);
  Serial.print("ROM 2 1000h - 1fffh : ");
  Serial.println(checksum2, HEX);
  Serial.print("ROM 3 2000h - 2fffh : ");
  Serial.println(checksum3, HEX);
  switch (checksum1) {
    case 0xAE5D:
      if (checksum2 == 0xDA84 && checksum3 == 4002) {
        match = 1;
      }
      break;
    case 0xAE60:
      if (checksum2 == 0xDA45 && checksum3 == 0x3E3E) {
        match = 2;
      }
      if (checksum2 == 0xDA45 && checksum3 == 0x40BA) {
        match = 3;
      }
      break;
    case 0xB078:
      if (checksum2 == 0xDA45 && checksum3 == 0x4006) {
        match = 4;
      }
      break;
    case 0xAED7:
      if (checksum2 == 0xDA45 && checksum3 == 0x4006) {
        match = 5;
      }
      break;
    case 0xC437:
      if (checksum2 == 0xDA30 && checksum3 == 0x40BA) {
        match = 6;
      }
      break;
    case 0xAB79:
      if ((checksum2 == 0xDA45 || checksum2 == 0xDA56) && checksum3 == 0x40BA) {
        match = 7;
      }  
      break;
    case 0xA94F:
      if (checksum2 == 0xDA67 && checksum3 == 0x40BA) {
        match = 8;
      }
      break;
    case 0xA74E:
      if (checksum2 == 0xDA67 && checksum3 == 0x40BA) {
        match = 9;
      }
      break;
  }
  if (match != 0)
    Serial.print("ROM set identified as ");
  Serial.println(rs[match]);
}

void TRS80DisplayTest() {
    int i, j, address;
    byte data = 0x00;
    byte temp;
    byte readData;

    outputByte(0xff,0);
    for (j = 0; j < 256; j++) {
      for (i = 0; i < T_DISP_SIZE; i++) {
        address = i + T_DISP_START;
        writeByte(address, data);
        temp = data;        
        temp &= 0xBF;
        readData = readByte(address);
        readData &= 0xBF;
        if ( temp != readData) {
          Serial.print("Read error at location: ");
          Serial.print(address, HEX);
          Serial.print("  expected ");
          Serial.print(temp, HEX);
          Serial.print("  got ");
          Serial.println(readData, HEX);
        }
        data = ++data; // auto modulo 256
      }
      if (stopIt()) break;
      data++;
    }
    Serial.println("Done.");
}

void TRS80LCDisplayTest() {
    int i, j, address;
    byte data = 0x00;
    byte readData;

    outputByte(0xff,0);
    for (j = 0; j < 256; j++) {
      for (i = 0; i < T_DISP_SIZE; i++) {
        address = i + T_DISP_START;
        writeByte(address, data);
        readData = readByte(address);
        if ( data != readData) {
          Serial.print("Read error at location: ");
          Serial.print(address, HEX);
          Serial.print("  expected ");
          Serial.print(data, HEX);
          Serial.print("  got ");
          Serial.println(readData, HEX);
        }
        data = ++data; // auto modulo 256
      }
      if (stopIt()) break;
      data++;
    }
    Serial.println("Done.");
}

void TRS80KeyboardTest() {
  Serial.println("Press TRS-80 keys...");
  Serial.println("0 1 2 3 4 5 6 7");
  byte r1, r2, r3, r4, r5, r6, r7, r8;
  while(1) {
    r1 = readByte(T_KEYB_ROW1);
    r2 = readByte(T_KEYB_ROW2);
    r3 = readByte(T_KEYB_ROW3);
    r4 = readByte(T_KEYB_ROW4);
    r5 = readByte(T_KEYB_ROW5);
    r6 = readByte(T_KEYB_ROW6);
    r7 = readByte(T_KEYB_ROW7);
    r8 = readByte(T_KEYB_ROW8);
    if (r1 + r2 + r3 + r4 + r5 + r6 + r7 + r8 != 0) {
      Serial.print(readByte(T_KEYB_ROW1), HEX);
      Serial.print(" ");
      Serial.print(readByte(T_KEYB_ROW2), HEX);
      Serial.print(" ");
      Serial.print(readByte(T_KEYB_ROW3), HEX);
      Serial.print(" ");
      Serial.print(readByte(T_KEYB_ROW4), HEX);
      Serial.print(" ");
      Serial.print(readByte(T_KEYB_ROW5), HEX);
      Serial.print(" ");
      Serial.print(readByte(T_KEYB_ROW6), HEX);
      Serial.print(" ");
      Serial.print(readByte(T_KEYB_ROW7), HEX);
      Serial.print(" ");
      Serial.println(readByte(T_KEYB_ROW8), HEX);
    }
    delay(500);
    if (stopIt()) break;
 
  }
  Serial.println("Done.");
}

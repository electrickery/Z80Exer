/*
Z80emu - Z80 pin exorsizer
*/

#include "Z80pins.h"

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
bool refreshMode = 0;

// core routines

void setup() {
  Serial.begin(9600);
  Serial.println("Z80exer v0.1");
  
  pinMode(LED, OUTPUT);
  
  pinMode(Z80INT,   INPUT);
  pinMode(Z80NMI,   INPUT);
  pinMode(Z80WAIT,  INPUT);
  pinMode(Z80BUSRQ, INPUT);
  pinMode(Z80RESET, INPUT); 
  digitalWrite(Z80INT,   HIGH);
  digitalWrite(Z80NMI,   HIGH);
  digitalWrite(Z80WAIT,  HIGH);
  digitalWrite(Z80BUSRQ, HIGH);
  digitalWrite(Z80RESET, HIGH);
  
  DDRA  = 0XFF; // address LSB output
  DDRC  = 0XFF; // address MSB output
  PORTK = 0xFF; // control out bits high
  DDRK  = 0XFF; // control out bits output
  DDRL  = 0x00; // data bus input
  PORTL = 0xFF; // data bus pull ups
  
//  onlineReadMode();
  delay(1000);  
}


void loop() {
  byte refreshAddress = 0;
  commandCollector();
  if (refreshMode) {
    refreshByte(refreshAddress);
    refreshAddress = 0x7F & ++refreshAddress;   
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
    case 'B':
    case 'b':
      blinkPin();
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
        Serial.println("refresh on");
      } else {
        refreshMode = 0;
        Serial.println("refresh off");
      }
      break;
    case 'S':
    case 's':
      setValue(); // fill memory range with a value
      break;
    case 'T':  // test ports
    case 't':
      portTest(serialBuffer[1]);
      break;
    case 'V':
    case 'v':
      viewPorts();
      break;
    case 'W':
    case 'w':
      writePin();
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
  unsigned int startAddress;
  unsigned int endAddress;
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
    printWord(endAddress - 1);
    Serial.println();
    unsigned int i, data;
    dataBusReadMode();
    for (i = startAddress; i < endAddress; i++) {
      positionOnLine = i & 0x0F;
      if (positionOnLine == 0) {
        printWord(i);
        Serial.print(": ");
      }
      data = readByte(i);
      printByte(data); 
      asChars[positionOnLine] = (data >= ' ' && data <= '~') ? data : '.';
      if ((i & 0x03) == 0x03) Serial.print(" ");
      if ((i & 0x0F) == 0x0F) {
        Serial.print (" ");
        printString(asCharsP);
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
  PORTA = addressLSB;
  PORTC = addressMSB;
  digitalWrite(Z80MREQ, LOW);
  digitalWrite(Z80RD,   LOW);
  data = PINL;
  digitalWrite(Z80RD,   HIGH);
  digitalWrite(Z80MREQ, HIGH); 
  return data; 
}

unsigned int fetchByte(unsigned int address) {
  unsigned int data = 0;
  unsigned int addressLSB = address & 0xFF;
  unsigned int addressMSB = address >> 8;
  dataBusReadMode();
  PORTA = addressLSB;
  PORTC = addressMSB;
  digitalWrite(Z80M1  , LOW);
  digitalWrite(Z80MREQ, LOW);
  digitalWrite(Z80RD,   LOW);
  data = PINL;
  digitalWrite(Z80RD,   HIGH);
  digitalWrite(Z80MREQ, HIGH); 
  digitalWrite(Z80M1  , HIGH);
  return data; 
}

void refreshByte(unsigned int address) {
//  unsigned int data = 0;
  unsigned int addressLSB = address & 0xFF;
  dataBusReadMode();
  PORTA = addressLSB;
  digitalWrite(Z80RFSH, LOW);
  digitalWrite(Z80MREQ, LOW);
//  delay(1);
  digitalWrite(Z80MREQ, HIGH);  
  digitalWrite(Z80RFSH, HIGH);
}

unsigned int inputByte(unsigned int address) {
  unsigned int data = 0;
  unsigned int addressLSB = address & 0xFF;
  unsigned int addressMSB = address >> 8;
  dataBusReadMode();
  PORTA = addressLSB;
  PORTC = addressMSB;
  digitalWrite(Z80IORQ, LOW);
  digitalWrite(Z80RD,   LOW);
  data = PINL;
  digitalWrite(Z80RD,   HIGH);
  digitalWrite(Z80IORQ, HIGH); 
  return data; 
}


void writeByte(unsigned int address, unsigned int value) {
  unsigned int addressLSB = address & 0xFF;
  unsigned int addressMSB = address >> 8;
  dataBusWriteMode();
  PORTA = addressLSB;
  PORTC = addressMSB;
  PORTL = value;
  digitalWrite(Z80MREQ, LOW);
  digitalWrite(Z80WR,   LOW);
//  delay(1);
  digitalWrite(Z80WR,   HIGH);
  digitalWrite(Z80MREQ, HIGH);
  dataBusReadMode();
}

void outputByte(unsigned int address, unsigned int value) {
  unsigned int addressLSB = address & 0xFF;
  unsigned int addressMSB = address >> 8;
  dataBusWriteMode();
  PORTA = addressLSB;
  PORTC = addressMSB;
  PORTL = value;
  digitalWrite(Z80IORQ, LOW);
  digitalWrite(Z80WR,   LOW);
//  delay(1);
  digitalWrite(Z80WR,   HIGH);
  digitalWrite(Z80IORQ, HIGH);
  dataBusReadMode();
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
  Serial.println("-- Z80 exerciser 0.1 command set --");
  Serial.println("Bpp or B#ss    - blink pin p (in hex) or symbol: A0-AF,D0-D7,RD,WR.MQ,IQ,M1,RF,HT,BK");
  Serial.println("D[ssss[-eeee]|+] - Dump memory from ssss to eeee (default 256 bytes)");
  Serial.println("H              - This help text");
  Serial.println("MRaaaa[+]      - Read memory address aaaa, optionally repeating");
  Serial.println("MWaaaa vv[+]   - Write vv to address aaaa, optionally repeating");
  Serial.println("PRaa[+]        - Read port address [aa]aa, optionally repeating");
  Serial.println("PWaa:vv[+]     - Write vv to address [aa]aa, optionally repeating");
  Serial.println("R[+|-]         - Refresh on/off");
  Serial.println("Qn             - Repeat rate; 1, 2, 4, 8, 16, ..., 32678 ms (n=0-9,A-F)");
  Serial.println("Sssss-eeee:vv  - fill a memory range with a value");
  Serial.println("Tp             - exercise port p");
  Serial.println("V              - view data bus, pins INT, NMI, WAIT, BUSRQ, RESET");
  Serial.println("Wpp v or W#ss v - Write pin (in hex) or symbol: A0-AF,D0-D7,RD,WR.MQ,IQ,M1,RF,HT,BK; values 0, 1");
  Serial.println("?              - This help text"); 
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

void blinkPin() {
  int pin;
  if (serialBuffer[1] == '#') {
    pin = getPinBySymbol();    
  } else {
    pin  = getNibble(serialBuffer[1]) * (1 << 4);
    pin += getNibble(serialBuffer[2]);
  }
  Serial.print("Pin: ");
  Serial.println(pin, DEC);  
  pinMode(pin, OUTPUT);
  while(1) {
    digitalWrite(pin, !digitalRead(pin));
    delay(repeatRate);
    if (Serial.available() > 0) {
        clearSerialBuffer();
        setBufPointer = 0;
        return;
    }
  }
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
  unsigned int i;
  for (i = startAddress; i <= endAddress; i++) {
    writeByte(i, value);
  }
  dataBusReadMode();
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
      Serial.print("IRD ");
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
      Serial.print("IWR ");
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
    repeatRate = 1 << value;
    Serial.print("Repeat rate set to ");
    Serial.print(repeatRate, DEC);
    Serial.println(" ms");
    
  } else {
    Serial.println("not supported");
  }
}

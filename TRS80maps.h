

#define ROM          0x0000
#define ROMSIZE      0x2FFF

#define KEYBOARD     0x3800
#define KEYBOARDSIZE 0x0100

#define VIDEO        0x3C00
#define VIDEOSIZE    0x1000

#define FDDRVSEL     0x37E1
#define FDCCMDSTAT   0x37EC
#define FDCTRACK     0x37ED
#define FDCSECTOR    0x37EE
#define FDCDATA      0x37EF


/*
- Test video memory with patterns
- Show character map
- Scan keyboard
- Select disk drive
- Select track
- Select density
- Seek sector

*/

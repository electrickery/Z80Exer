## Z80Exer - an Arduino based Z80 board exercizer

A long time ago (well 2014), I developed the original Z80Exersizer, to be able to check Z80 based boards from the Z80 CPU
socket. It worked good enough and I announced this on the VCF-forum (then called vintagecomputer.org). Eventually I added 
some specific features to test a TRS-80 Model 1, but mostly the project was halted.

At one time David Mutimer became interested and added the option to control the Model 1 from the expansion bus. Another 
feature was the much improved testing of the dynamic RAM.

Originally I used an experiment PCB between the Arduino Mega 2560, but with the TRS80 bus added this would get too complex, 
so in 2020 I created a shield, and included a connector for the TRS-80 connector to make it more versatile. The code was 
somewhat more optimized. The board is alas still not completely debugged.

![Z80 exer shields; prototype and first PBC version](shields.jpg)

Now the versions I still had in .tgz files are uploaded here in github, and the work of David Mutimer is in a separate 
branch DMutimer. It should be a clean merge, but I do not use the TRS-80 bus mode.

Z80exer command set:

		-- Z80 exerciser v0.8                                          
 		 -- debugging command set --
		Aaaaa            - set address bus to value aaaa
		Bpp or B#ss      - blink pin p (in hex) or symbol: A0-AF,D0-D7,RD,WR.MQ,IQ,M1,RF,HT,BK
		MRaaaa[+]        - Read memory address aaaa, optionally repeating
		MWaaaa vv[+]     - Write vv to address aaaa, optionally repeating
		PRaa[+]          - Read port address aa, optionally repeating
		PWaa:vv[+]       - Write vv to address aa, optionally repeating
		Tp               - exercise Arduino port p: ACDHKL
		V                - view data bus, pins INT, NMI, WAIT, BUSRQ, RESET
		Wpp v or W#ss v  - Write pin (in hex) or symbol: A0-AF,D0-D7,RD,WR.MQ,IQ,M1,RF,HT,BK; values 0, 1
		Xt               - Tri-state all Arduino pins. 0: off, 1: on.
 		 -- operational command set --
		D[ssss[-eeee]|+] - Dump memory from ssss to eeee (default 256 bytes)
		Issss-eeee       - Generate hex intel data records
		O                - Input Port map
		Sssss-eeee:vv    - fill memory range with a value
		Ussss-eeee       - test RAM range (walking 1s)
 		 -- misc. command set --
		E                - Toggle echo
		H                - This help text
		R[+|-]           - Refresh on/off
		Qn               - Repeat rate; 1, 2, 4, 8, 16, ..., 32678 ms (n=0-9,A-F)
		?                - This help text



The main page is at [Github](https://electrickery.nl/digaud/arduino/Z80exer/)

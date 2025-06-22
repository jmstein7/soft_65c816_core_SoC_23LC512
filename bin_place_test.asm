                .ORG    $BFFE
                .word  $C000           
; XMODEM/CRC Receiver for the 65C02
;
; By Daryl Rictor & Ross Archer  Aug 2002
;
; 21st century code for 20th century CPUs (tm?)
; 
; A simple file transfer program to allow upload from a console device
; to the SBC utilizing the x-modem/CRC transfer protocol.  Requires just
; under 1k of either RAM or ROM, 132 bytes of RAM for the receive buffer,
; and 8 bytes of zero page RAM for variable storage.
;
;**************************************************************************
; This implementation of XMODEM/CRC does NOT conform strictly to the 
; XMODEM protocol standard in that it (1) does not accurately time character
; reception or (2) fall back to the Checksum mode.

; (1) For timing, it uses a crude timing loop to provide approximate
; delays.  These have been calibrated against a 1MHz CPU clock.  I have
; found that CPU clock speed of up to 5MHz also work but may not in
; every case.  Windows HyperTerminal worked quite well at both speeds!
;
; (2) Most modern terminal programs support XMODEM/CRC which can detect a
; wider range of transmission errors so the fallback to the simple checksum
; calculation was not implemented to save space.
;**************************************************************************
;
; Files uploaded via XMODEM-CRC must be
; in .o64 format -- the first two bytes are the load address in
; little-endian format:  
;  FIRST BLOCK
;     offset(0) = lo(load start address),
;     offset(1) = hi(load start address)
;     offset(2) = data byte (0)
;     offset(n) = data byte (n-2)
;
; Subsequent blocks
;     offset(n) = data byte (n)
;
; The TASS assembler and most Commodore 64-based tools generate this
; data format automatically and you can transfer their .obj/.o64 output
; file directly.  
;   
; The only time you need to do anything special is if you have 
; a raw memory image file (say you want to load a data
; table into memory). For XMODEM you'll have to 
; "insert" the start address bytes to the front of the file.
; Otherwise, XMODEM would have no idea where to start putting
; the data.

;-------------------------- The Code ----------------------------
;
; zero page variables (adjust these to suit your needs)
;
;
crc		=	$38		; CRC lo byte  (two byte variable)
crch		=	$39		; CRC hi byte  
ptr		=	$3a		; data pointer (two byte variable)
ptrh		=	$3b		;   "    "
blkno		=	$3c		; block number 
retry		=	$3d		; retry counter 
retry2		=	$3e		; 2nd counter
bflag		=	$3f		; block flag 
;
;Monitor Variables
;
IN          = $0200          ;*Input buffer
XAML        = $E4            ;*Index pointers, really ZP $24 and up
XAMH        = $E5
STL         = $E6
STH         = $E7
L           = $E8
H           = $E9
YSAV        = $EA
MODE        = $EB
MSGL      = $EC
MSGH      = $ED
;
; non-zero page variables and buffers
;
;
Rbuff		=	$0300      	; temp 132 byte receive buffer 
					;(place anywhere, page aligned)
;
;
;  tables and constants
;
;
; The crclo & crchi labels are used to point to a lookup table to calculate
; the CRC for the 128 byte data blocks.  There are two implementations of these
; tables.  One is to use the tables included (defined towards the end of this
; file) and the other is to build them at run-time.  If building at run-time,
; then these two labels will need to be un-commented and declared in RAM.
;
;crclo		=	$7D00      	; Two 256-byte tables for quick lookup
;crchi		= 	$7E00      	; (should be page-aligned for speed)
;
;
;
; XMODEM Control Character Constants
SOH		=	$01		; start block
EOT		=	$04		; end of text marker
ACK		=	$06		; good block acknowledged
NAK		=	$15		; bad block acknowledged
CAN		=	$18		; cancel (not standard, not supported)
CR		=	$0d		; carriage return
LF		=	$0a		; line feed
ESC		=	$1b		; ESC to exit

;
;^^^^^^^^^^^^^^^^^^^^^^ Start of Program ^^^^^^^^^^^^^^^^^^^^^^
;
; Xmodem/CRC upload routine
; By Daryl Rictor, July 31, 2002
;
; v0.3  tested good minus CRC
; v0.4  CRC fixed!!! init to $0000 rather than $FFFF as stated   
; v0.5  added CRC tables vs. generation at run time
; v 1.0 recode for use with SBC2
; v 1.1 added block 1 masking (block 257 would be corrupted)

		*= 	$C000		; Start of program (adjust to your needs)
        .setting "HandleLongBranch", true
; TITLE "SWEET16 INTERPRETER"
REG	= $2		; R0
CALLV	= REG+20	; R10
STACK	= REG+24	; R12
STATUS	= REG+28	; HIGH Byte of R14
IP		= REG+30	; R15

REGA = IP+2
REGX = REGA+1
REGY = REGX+1
REGP = REGY+1

SWEET16C2:
	JSR PUTSTATE
	PLA
	STA IP		;(IP) uses 6502 Return Address
	PLA 		;6502 RtnAdd = Actual-1
	STA IP+1
	JMP NEXTOP

BROPS:
	; I use Wozniak's pseudo-ops
	.word  RTN		;0	Return to 65C02 code
	.word  BR		;1	Branch to Sweet16 location
	.word  BNC		;2	Branch if no carry
	.word  BC		;3	Branch if carry
	.word  BP		;4	Branch if last register >=0
	.word  BM		;5	Branch if last register <0
	.word  BZ		;6	Branch if last register =0
	.word  BNZ		;7	Branch if last register <>0
	.word  BM1		;8	Branch if last register =(-1)
	.word  BNM1	;9	Branch if last register <>(-1)
	.word  BK		;A	Break (hopefully to Monitor)
	.word  RS		;B	Return from Sweet16 subroutine
	.word  BS		;C	Branch Sweet16 subroutine
	.word  CALL	;D	Call ML routine offset from R11
	.word  ADJ0	;E	Adjust Accumulator by offset
	.word  ADJS	;F	Adjust Stack by offset

; Register OPS
OPS:	; I use Wozniak's pseudo-ops
	; add "I" for "@" indirect ops.
	.word  SET		;$1r	SET Rn $[lo] $[hi]
	.word  LD		;$2r	LD Rn
	.word  ST		;$3r	ST Rn
	.word  LDI		;$4r	LD @Rn
	.word  STI		;$5r	ST @Rn
	.word  LDDI	;$6r	LDD @Rn
	.word  STDI	;$7r	STD @Rn
	.word  POPI	;$8r	POP @Rn
	.word  STPI	;$9r	STP @Rn
	.word  ADD		;$Ar	ADD Rn
	.word  SUB		;$Br	SUB Rn
	.word  POPDI	;$Cr POPD @Rn
	.word  CPR		;$Dr	CPR Rn
	.word  INR		;$Er	INR Rn
	.word  DCR		;$Fr	DCR Rn

PUTSTATE:
	; Implemented as a routine so may be
	; called as a CALL op.
	; eg, when returning from Kernal call
	PHP			;Preserve Register State
	STA REGA
	STX REGX
	STY REGY
	PLA
	STA REGP
	CLD
	RTS

GETSTATE:
	; Implemented as a routine so may be
	; called as a CALL op, eg, with
	; pseudo-regs set up for Kernal call
	LDA REGP
	PHA
	LDA REGA
	LDX REGX
	LDY REGY
	PLP
	RTS

; These five Branch ops inherit their branch 
; to NEXTOP from the branch op that they jump into
; putting them here allows BRANCH to be close enough
; to NEXTOP for NEXTOP's BEQ BRANCH to work.

; BM1 and BNM1 uses the BZ and BNZ logic
; compared to #$FF rather than #0
BM1	ASL
	TAX
	LDA #$FF
	BRA BAX

BNM1	ASL
	    TAX
	    LDA #$FF
	    BRA BNAX

; Call ends by branching to step over any embedded
; data for the use of the called routine

CALL:	; Pointer to 65C02 machine code is
	; in R11. Branch via OP to support
	; embedded data
	JSR PL1
	BRA BR
PL1:	JMP (CALLV)


; Adjust Stack and R0 uses same offset logic
; As Branch, just different registers

ADJ0:	LDX #0
	BRA OFFSET

ADJS:	LDX #(STACK-REG)
	BRA OFFSET

BRANCH:
	TYA			;Only here if Y=$0x
	ASL			;A is actually the OP index
	TAX			
	INC IP		;Now increment IP, since
	BNE  PL2		;actual operand is in following byte
	INC  IP+1
PL2:	LDA  STATUS	;Holds Register*2+Carry
	    LSR			;Carry flag -> Carry, Prior Reg in A
	    JMP (BROPS,X)	;Indexed jump to Branch OPS

; Branch on condition codes in front of NEXTOP and BR so they
; can branch either way

BNC:	BCC BR	  	;Branch on Carry Clear
	    BCS NEXTOP

BC:	BCS BR		;Branch on Carry Set
	BCC NEXTOP

;The previous register number is in A, it must be
;doubled for all users of the previous register
;index, so the X register indexed by word values.

BP:	ASL
	TAX
	LDA REG+1,X	;Check sign
	BPL BR		;Branch on positive
	BMI NEXTOP

BM:	ASL
	TAX
	LDA REG+1,X	;Check sign
	BMI BR		;Branch on negative
	BPL NEXTOP

BZ:	ASL
	TAX
	LDA #0
BAX:
	CMP REG,X		;Check zero
	BNE NEXTOP
	CMP REG+1,X
	BEQ BR
	BNE NEXTOP

BNZ:	ASL
	TAX
	LDA #0
BNAX:
	CMP REG,X		;Check zero
	BNE BR
	CMP REG+1,X
	BEQ NEXTOP
	BNE BR

; Branch to Subroutine falls through to BR
; BR falls through to NEXTOP

BS:	LDA IP
	STA (STACK)
	INC STACK
	BNE PL3
	INC STACK+1
PL3:	LDA IP+1
	    STA (STACK)
	    INC STACK
	    BNE BR
	    INC STACK+1
BR:
	LDX #(IP-REG)	; Index to IP
OFFSET:
	LDY #0		;16bit sign extension
	LDA  (IP)		;Branch Offset
	BPL  PL4
	DEY
PL4:	CLC
	ADC  REG,X	  	;ADD TO IP
	STA  REG,X
	TYA
	ADC  REG+1,X
	STA  REG+1,X
NEXTOP:
	INC IP
	BNE  PL5		; ++IP
	INC IP+1
NEXTOP1:
PL5:	LDA (IP)		; if([(++IP)]&&F0h)
	TAY
	AND #$F0
	BEQ BRANCH
	LSR
	LSR
	LSR
	TAX
	TYA
	AND #$0F		; *2 = Reg if OP, BROP if BROP
	ASL
	STA STATUS		; This is the register index operand
				    ; Carry clear for each main OP
	JMP (OPS-2,X)	; Minimum X=2, since X=0 => BRANCH

; These two don't have any ops branching into
; them to share their finishing semantics.

POPI:
	TAX
	LDA REG,X		; LD R0,(--Rn)
	BNE PL6
	DEC REG+1,X
PL6:	DEC REG,X
	    LDA (REG,X)
	    STA REG
	    STZ REG+1
	    BRA NEXTOP

DCR:	
	TAX			; Rn<-Rn-1
	LDA REG,X
	BNE PL7
	DEC REG+1,X
PL7:	DEC REG,X
	    BRA NEXTOP	

; These four have operations branching in to
; share their finishing semantics

ADD:	STZ STATUS
	TAX			; ADD R0,Rn
	CLC
	LDA REG
	ADC REG,X
	STA REG
	LDA REG+1
	ADC REG+1,X
	STA REG+1
ADD1:	BCC NEXTOP
	    INC STATUS
	    BRA NEXTOP

LD:	TAY
	LDX #0
LD1:	LDA REG,Y		; LD R0,Rn
	    STA REG,X
	    LDA REG+1,Y
	    STA REG+1,X
	    BRA NEXTOP

STPI:				; LD (--Rn),R0
	TAX
	LDA REG,X
	BNE PL8
	DEC REG+1,X
PL8:	DEC REG,X
	LDA REG
	STA (REG,X)
STPI1:
	STZ STATUS		; Branch conditions reflects R0
	BRA NEXTOP

LDI:				; LD R0,(Rn++) # Bytey
	TAX
LDI1:
	STZ REG+1
	LDA (REG,X)
	STA REG
LDI2:
	STZ STATUS 	; R0 is actual target
INRX:
	INC REG,X		; The Rn++ part
	BNE PL9
	INC REG+1,X
PL9:	BRA NEXTOP


; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; ~~ * NEXTOP must be no more than ~~~
; ~~ 127 bytes from here ~~~~~~~~~~~~~
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; ~~ These must be within 127 of the op they ~~
; ~~ piggbyback their finish from, can be ~~~~~
; ~~ further than 127 from NEXTOP ~~~~~~~~~~~~~
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

SUB:	LDX #0		; SUB R0,Rn
CPR:	TAY			; CPR opcode = R13 index
	STX STATUS		; Carry will be incremented in
	SEC
	LDA REG
	SBC REG,Y
	STA REG,X
	LDA REG+1
	SBC REG+1,Y
	STA REG+1,X
	BRA ADD1		; Share ADD exit

ST:	TAX
	LDY #0
	BRA LD1

POPDI:			; LDD R0,(--Rn)
	TAX
	LDA REG,X
	BNE PL10
	DEC REG+1,X
PL10:	DEC REG,X
	    LDA (REG,X)
	    PHA
	    LDA REG,X
	    BNE PL11
	    DEC REG+1,X
PL11:	DEC REG,X
	    LDA (REG,X)
	    STA REG
	    PLA
	    STA REG+1
	    BRA STPI1		; Share STP @Rn exit

STI:		; 
	TAX
	LDA REG		; LD (Rn++), R0	# Byte
	STA (REG,X)
	BRA LDI2

LDDI:				; LDD R0,(Rn++)
	TAX
	LDA (REG,X)
	STA REG
	INC REG,X
	BNE PL12
	INC REG+1,X
PL12:	LDA (REG,X)
	    STA REG+1
	    BRA LDI2

STDI:				; LDD (Rn++),R0
	TAX
	LDA REG
	STA (REG,X)
	INC REG,X
	BNE PL13
	INC REG+1,X
PL13:	LDA REG+1
	    STA (REG,X)
	    BRA INRX

INR:	TAX
	    BRA INRX

; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
; SET, RS, BK and CALL end in jumps ~~
; not bound by -125/+128 BRA limits ~~
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

SET:				; LDD Rn,#OP1:OP2
	TAX
	LDY #1
	LDA (IP),Y
	STA REG,X
	INY
	LDA (IP),Y
	STA REG+1,X
	TYA
	SEC
	ADC IP
	STA IP
	BCC PL14
	INC IP+1
PL14:	JMP NEXTOP1

RS:				;Pop the IP from the stack
	SEC
	LDA STACK
	SBC #2
	STA STACK
	BCS PL15
	DEC STACK+1
PL15:	LDY #1
	    LDA (STACK),Y
	    STA IP+1
	    LDA (STACK)
	    STA IP
	    JMP NEXTOP

BK:	BRK

RTN:	JSR GETSTATE
	    JMP  (IP)	;Go Back to 65C02 code

SWEETVM = SWEET16C2
; END OF SWEET16
;
XModem		jsr	PrintMsg	; send prompt and info
		lda	#$01
		sta	blkno		; set block # to 1
		sta	bflag		; set flag to get address from block 1
StartCrc	lda	#"C"		; "C" start with CRC mode
		jsr	Put_Chr		; send it
		lda	#$FF	
		sta	retry2		; set loop counter for ~3 sec delay
		lda	#$00
            sta	crc
		sta	crch		; init CRC value	
		jsr	GetByte		; wait for input
            bcs	GotByte		; byte received, process it
		bcc	StartCrc	; resend "C"

StartBlk	lda	#$FF		; 
		sta	retry2		; set loop counter for ~3 sec delay
		lda	#$00		;
		sta	crc		;
		sta	crch		; init CRC value	
		jsr	GetByte		; get first byte of block
		bcc	StartBlk	; timed out, keep waiting...
GotByte		cmp	#ESC		; quitting?
                bne	GotByte1	; no
;		lda	#$FE		; Error code in "A" of desired
                rts			; YES - do BRK or change to RTS if desired
GotByte1        cmp	#SOH		; start of block?
		beq	BegBlk		; yes
		cmp	#EOT		;
		bne	BadCrc		; Not SOH or EOT, so flush buffer & send NAK	
		jmp	Done		; EOT - all done!
BegBlk		ldx	#$00
GetBlk		lda	#$ff		; 3 sec window to receive characters
		sta 	retry2		;
GetBlk1		jsr	GetByte		; get next character
		bcc	BadCrc		; chr rcv error, flush and send NAK
GetBlk2		sta	Rbuff,x		; good char, save it in the rcv buffer
		inx			; inc buffer pointer	
		cpx	#$84		; <01> <FE> <128 bytes> <CRCH> <CRCL>
		bne	GetBlk		; get 132 characters
		ldx	#$00		;
		lda	Rbuff,x		; get block # from buffer
		cmp	blkno		; compare to expected block #	
		beq	GoodBlk1	; matched!
		jsr	Print_Err	; Unexpected block number - abort	
		jsr	Flush		; mismatched - flush buffer and then do BRK
;		lda	#$FD		; put error code in "A" if desired
		rts			; unexpected block # - fatal error - BRK or RTS
GoodBlk1	eor	#$ff		; 1's comp of block #
		inx			;
		cmp	Rbuff,x		; compare with expected 1's comp of block #
		beq	GoodBlk2 	; matched!
		jsr	Print_Err	; Unexpected block number - abort	
		jsr 	Flush		; mismatched - flush buffer and then do BRK
;		lda	#$FC		; put error code in "A" if desired
		brk			; bad 1's comp of block#	
GoodBlk2	ldy	#$02		; 
CalcCrc		lda	Rbuff,y		; calculate the CRC for the 128 bytes of data	
		jsr	UpdCrc		; could inline sub here for speed
		iny			;
		cpy	#$82		; 128 bytes
		bne	CalcCrc		;
		lda	Rbuff,y		; get hi CRC from buffer
		cmp	crch		; compare to calculated hi CRC
		bne	BadCrc		; bad crc, send NAK
		iny			;
		lda	Rbuff,y		; get lo CRC from buffer
		cmp	crc		; compare to calculated lo CRC
		beq	GoodCrc		; good CRC
BadCrc		jsr	Flush		; flush the input port
		lda	#NAK		;
		jsr	Put_Chr		; send NAK to resend block
		jmp	StartBlk	; start over, get the block again			
GoodCrc		ldx	#$02		;
		lda	blkno		; get the block number
		cmp	#$01		; 1st block?
		bne	CopyBlk		; no, copy all 128 bytes
		lda	bflag		; is it really block 1, not block 257, 513 etc.
		beq	CopyBlk		; no, copy all 128 bytes
		lda	Rbuff,x		; get target address from 1st 2 bytes of blk 1
		sta	ptr		; save lo address
		inx			;
		lda	Rbuff,x		; get hi address
		sta	ptr+1		; save it
		inx			; point to first byte of data
		dec	bflag		; set the flag so we won't get another address		
CopyBlk		ldy	#$00		; set offset to zero
CopyBlk3	lda	Rbuff,x		; get data byte from buffer
		sta	(ptr),y		; save to target
		inc	ptr		; point to next address
		bne	CopyBlk4	; did it step over page boundary?
		inc	ptr+1		; adjust high address for page crossing
CopyBlk4	inx			; point to next data byte
		cpx	#$82		; is it the last byte
		bne	CopyBlk3	; no, get the next one
IncBlk		inc	blkno		; done.  Inc the block #
		lda	#ACK		; send ACK
		jsr	Put_Chr		;
		jmp	StartBlk	; get next block
Done		lda	#ACK		; last block, send ACK and exit.
		jsr	Put_Chr		;
		jsr	Flush		; get leftover characters, if any
		jsr	Print_Good	;
		jmp RESET			;can be rts
;
;^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
;
; subroutines
;
;					;
GetByte		lda	#$00		; wait for chr input and cycle timing loop
		sta	retry		; set low value of timing loop
StartCrcLp	jsr	Get_chr		; get chr from serial port, don't wait 
		bcs	GetByte1	; got one, so exit
		dec	retry		; no character received, so dec counter
		bne	StartCrcLp	;
		dec	retry2		; dec hi byte of counter
		bne	StartCrcLp	; look for character again
		clc			; if loop times out, CLC, else SEC and return
GetByte1	rts			; with character in "A"
;
Flush		lda	#$70		; flush receive buffer
		sta	retry2		; flush until empty for ~1 sec.
Flush1		jsr	GetByte		; read the port
		bcs	Flush		; if chr recvd, wait for another
		rts			; else done
;
PrintMsg	ldx	#$00		; PRINT starting message
PrtMsg1		lda   	Msg,x		
		beq	PrtMsg2			
		jsr	Put_Chr
		inx
		bne	PrtMsg1
PrtMsg2		rts
Msg		.textz	"Begin XMODEM/CRC upload. "
		;.BYTE  	CR, LF
               	;.byte   0
;
Print_Err	ldx	#$00		; PRINT Error message
PrtErr1		lda   	ErrMsg,x
		beq	PrtErr2
		jsr	Put_Chr
		inx
		bne	PrtErr1
PrtErr2		rts
ErrMsg		.textz 	"Upload Error!"
		;.BYTE  	CR, LF
                ;.byte   0
;
Print_Good	ldx	#$00		; PRINT Good Transfer message
Prtgood1	lda   	GoodMsg,x
		beq	Prtgood2
		jsr	Put_Chr
		inx
		bne	Prtgood1
Prtgood2	rts
GoodMsg		.textz 	"Upload Successful!"
		;.BYTE  	CR, LF
                ;.byte   0
;
;
;======================================================================
;  I/O Device Specific Routines
;
;  Two routines are used to communicate with the I/O device.
;
; "Get_Chr" routine will scan the input port for a character.  It will
; return without waiting with the Carry flag CLEAR if no character is
; present or return with the Carry flag SET and the character in the "A"
; register if one was present.
;
; "Put_Chr" routine will write one byte to the output port.  Its alright
; if this routine waits for the port to be ready.  its assumed that the 
; character was send upon return from this routine.
;
; Here is an example of the routines used for a standard 6551 ACIA.
; You would call the ACIA_Init prior to running the xmodem transfer
; routine.
;
;VIA_REG     =  $8800
;VIA_DATA    =  $8801
;VIA_DDRR    =  $8802
;VIA_DDRD    =  $8803
;VIA_HS      =  $880C
;
;PSP_DATA    =  $9000
;
CPRO        =   $F6
ACIA_Data	=	$8000		; Adjust these addresses to point 
ACIA_Status	=	$8001		; to YOUR 6551!
ACIA_Command	=	$8002		;
ACIA_Control	=	$8003		;
ACIA        = $8000
ACIA_CTRL   = ACIA+3
ACIA_CMD    = ACIA+2
ACIA_SR     = ACIA+1
ACIA_DAT    = ACIA
ACIA_Init      	lda	#$10      ; 115.2k init is #%00010000
               	sta	ACIA_Control   	; control reg 
               	lda	#$C9           	; N parity/echo off/rx int off/ dtr active low
               	sta	ACIA_Command   	; command reg 
                ;lda   #$99
                ;sta   VIA_HS
                ;lda   #$FF
                ;sta   VIA_DDRR
                ;sta   VIA_DDRD
                ;sta   VIA_REG
               	rts                  	; done
;
; input chr from ACIA (no waiting)
;
Get_Chr		clc			; no chr present
               	lda	ACIA_Status     ; get Serial port status
               	and	#$08            ; mask rcvr full bit
              	beq	Get_Chr2	; if not chr, done
               	Lda	ACIA_Data       ; else get chr
	       	sec			; and set the Carry Flag
Get_Chr2    	rts			; done
;
; output to OutPut Port
;
Put_Chr	   	PHA                     ; save registers
Put_Chr1     	lda	ACIA_Status     ; serial port status
              	and	#$10            ; is tx buffer empty
               	beq	Put_Chr1        ; no, go back and test it again
               	PLA                     ; yes, get chr to send
               	sta	ACIA_Data       ; put character to Port
                jsr Delay
               	RTS                     ; done
;=========================================================================
;Main Monitor Routines/Programs
RESET       CLD             ;Clear decimal arithmetic mode.
            CLI
            SEC
            XCE
            CLC                   ;switch MPU to...
            XCE                   ;native mode
            JSR ACIA_Init        ;* Init ACIA 
         LDA #$0D
         JSR ECHO      ;* New line.
         LDA #<MSG1
         STA MSGL
         LDA #>MSG1
         STA MSGH
         JSR SHWMSG      ;* Show Welcome.
         LDA #$0D
         JSR ECHO      ;* New line.
         LDA #<MSG5
         STA MSGL
         LDA #>MSG5
         STA MSGH
         JSR SHWMSG      ;* Show Sweet16 availability.
         LDA #$0D
         JSR ECHO      ;* New line.
         LDA #<MSG7
         STA MSGL
         LDA #>MSG7
         STA MSGH
         JSR SHWMSG      ;* Show clear command.
         LDA #$0D
         JSR ECHO      ;* New line.
SOFTRESET   LDA #$9B      ;* Auto escape.
NOTCR       CMP #$88        ;"<-"? * Note this was chaged to $88 which is the back space key.
            BEQ BACKSPACE   ;Yes.
            CMP #$9B        ;ESC?
            BEQ ESCAPE      ;Yes.
            INY             ;Advance text index.
            BPL NEXTCHAR    ;Auto ESC if >127.
ESCAPE      LDA #$DC        ;"\"
            JSR ECHO        ;Output it.
GETLINE     LDA #$8D        ;CR.
            JSR ECHO        ;Output it.
            LDY #$01        ;Initiallize text index.
BACKSPACE   DEY             ;Backup text index.
            BMI GETLINE     ;Beyond start of line, reinitialize.
         LDA #$A0      ;*Space, overwrite the backspaced char.
         JSR ECHO
         LDA #$88      ;*Backspace again to get to correct pos.
         JSR ECHO
NEXTCHAR    LDA ACIA_SR     ;*See if we got an incoming char
            AND #$08        ;*Test bit 3
            BEQ NEXTCHAR    ;*Wait for character
            LDA ACIA_DAT    ;*Load char
         CMP #$60      ;*Is it Lower case
         BMI   CONVERT      ;*Nope, just convert it
         AND #$5F      ;*If lower case, convert to Upper case
CONVERT     ORA #$80        ;*Convert it to "ASCII Keyboard" Input
            STA IN,Y        ;Add to text buffer.
            JSR ECHO        ;Display character.
            CMP #$8D        ;CR?
            BNE NOTCR       ;No.
            LDY #$FF        ;Reset text index.
            LDA #$00        ;For XAM mode.
            TAX             ;0->X.
SETSTOR     ASL             ;Leaves $7B if setting STOR mode.
SETMODE     STA MODE        ;$00 = XAM, $7B = STOR, $AE = BLOK XAM.
BLSKIP      INY             ;Advance text index.
NEXTITEM    LDA IN,Y        ;Get character.
            CMP #$8D        ;CR?
            BEQ GETLINE     ;Yes, done this line.
            CMP #$AE        ;"."?
            BCC BLSKIP      ;Skip delimiter.
            BEQ SETMODE     ;Set BLOCK XAM mode.
            CMP #$BA        ;":"?
            BEQ SETSTOR     ;Yes, set STOR mode.
            CMP #$D2        ;"R"?
            BEQ RUN         ;Yes, run user program.
            CMP #$CC        ;"L", as in c[L]ear?
            BEQ CLRZPSTA    ;Yes,  clear ZP and reset Stack.
            CMP #$D8        ;"X"?
            BEQ XModem      ;Yes, run XModem transfer.
            STX L           ;$00->L.
            STX H           ; and H.
            STY YSAV        ;Save Y for comparison.
NEXTHEX     LDA IN,Y        ;Get character for hex test.
            EOR #$B0        ;Map digits to $0-9.
            CMP #$0A        ;Digit?
            BCC DIG         ;Yes.
            ADC #$88        ;Map letter "A"-"F" to $FA-FF.
            CMP #$FA        ;Hex letter?
            BCC NOTHEX      ;No, character not hex.
DIG         ASL
            ASL             ;Hex digit to MSD of A.
            ASL
            ASL
            LDX #$04        ;Shift count.
HEXSHIFT    ASL             ;Hex digit left MSB to carry.
            ROL L           ;Rotate into LSD.
            ROL H           ;Rotate into MSD's.
            DEX             ;Done 4 shifts?
            BNE HEXSHIFT    ;No, loop.
            INY             ;Advance text index.
            BNE NEXTHEX     ;Always taken. Check next character for hex.
NOTHEX      CPY YSAV        ;Check if L, H empty (no hex digits).
         BNE NOESCAPE   ;* Branch out of range, had to improvise...
            JMP ESCAPE      ;Yes, generate ESC sequence.

RUN         JSR ACTRUN      ;* JSR to the Address we want to run.
        JMP   SOFTRESET     ;* When returned from the program, reset monitor.
ACTRUN      JMP (XAML)      ;Run at current XAM index.
		JMP   SOFTRESET
NOESCAPE    BIT MODE        ;Test MODE byte.
            BVC NOTSTOR     ;B6=0 for STOR, 1 for XAM and BLOCK XAM
            LDA L           ;LSD's of hex data.
            STA (STL, X)    ;Store at current "store index".
            INC STL         ;Increment store index.
            BNE NEXTITEM    ;Get next item. (no carry).
            INC STH         ;Add carry to 'store index' high order.
TONEXTITEM  JMP NEXTITEM    ;Get next command item.
NOTSTOR     BMI XAMNEXT     ;B7=0 for XAM, 1 for BLOCK XAM.
            LDX #$02        ;Byte count.
SETADR      LDA L-1,X       ;Copy hex data to
            STA STL-1,X     ;"store index".
            STA XAML-1,X    ;And to "XAM index'.
            DEX             ;Next of 2 bytes.
            BNE SETADR      ;Loop unless X = 0.
NXTPRNT     BNE PRDATA      ;NE means no address to print.
            LDA #$8D        ;CR.
            JSR ECHO        ;Output it.
            LDA XAMH        ;'Examine index' high-order byte.
            JSR PRBYTE      ;Output it in hex format.
            LDA XAML        ;Low-order "examine index" byte.
            JSR PRBYTE      ;Output it in hex format.
            LDA #$BA        ;":".
            JSR ECHO        ;Output it.
PRDATA      LDA #$A0        ;Blank.
            JSR ECHO        ;Output it.
            LDA (XAML,X)    ;Get data byte at 'examine index".
            JSR PRBYTE      ;Output it in hex format.
XAMNEXT     STX MODE        ;0-> MODE (XAM mode).
            LDA XAML
            CMP L           ;Compare 'examine index" to hex data.
            LDA XAMH
            SBC H
            BCS TONEXTITEM  ;Not less, so no more data to output.
            INC XAML
            BNE MOD8CHK     ;Increment 'examine index".
            INC XAMH
MOD8CHK     LDA XAML        ;Check low-order 'exainine index' byte
            AND #$0F        ;For MOD 8=0 ** changed to $0F to get 16 values per row **
            BPL NXTPRNT     ;Always taken.
PRBYTE      PHA             ;Save A for LSD.
            LSR
            LSR
            LSR             ;MSD to LSD position.
            LSR
            JSR PRHEX       ;Output hex digit.
            PLA             ;Restore A.
PRHEX       AND #$0F        ;Mask LSD for hex print.
            ORA #$B0        ;Add "0".
            CMP #$BA        ;Digit?
            BCC ECHO        ;Yes, output it.
            ADC #$06        ;Add offset for letter.
ECHO        PHA             ;*Save A
            AND #$7F        ;*Change to "standard ASCII"
            STA ACIA_DAT    ;*Send it.
            jsr Delay       ; try delay
WAIT       LDA ACIA_SR     ;*Load status register for ACIA
            AND #$10        ;*Mask bit 4.
            BEQ    WAIT    ;*ACIA not done yet, wait.
            PLA             ;*Restore A
            RTS             ;*Done, over and out...

SHWMSG      LDY #$0
PRINT      LDA (MSGL),Y
         BEQ DONEX
         JSR ECHO
         INY 
         BNE PRINT
DONEX      RTS 
;clear zero page and stack routine
CLRZPSTA    CLD
        TOL     =   $00 ;low address byte
        TOH     =   $01 ;top address byte
        TBL     =   $00 ;low address byte
        TBH     =   $00 ;top address byte
        TOPNT   =   $AA
        CLC
        LDX #0
        TXS
        LDA #TOL
        STA TOPNT
        LDA #TOH
        STA TOPNT+1
        LDA #$00
        TAY             ;Initialize index pointer
CLRM1   STA (TOPNT),Y   ;Clear memory location
        INY             ;Advance index pointer
        DEX             ;Decrement counter
        BNE CLRM1       ;Not zero, continue checking
BEGIN2  LDA #TBL
        STA TOPNT
        LDA #TBH
        STA TOPNT+1
        LDA #$00
        TAY             ;Initialize index pointer
CLRM2   STA (TOPNT),Y   ;Clear memory location
        INY             ;Advance index pointer
        DEX             ;Decrement counter
        BNE CLRM2       ;Not zero, continue checking
         LDA #$0D
         JSR ECHO      ;* New line.
         LDA #<MSG6
         STA MSGL
         LDA #>MSG6
         STA MSGH
         JSR SHWMSG      ;* Show clear message.
         LDA #$0D
        JMP RESET       ;jump to RESET vector
;16-bit Interrupt Service Routines 
STKPC       =   $0B
STKPC8      =   $06
BITLOC      =   $7FF0           ; Co-Pro signature address
CO_PRO16    SEI
            phb                   ;save DB
            phd                   ;save DP
            rep #%00110000        ;select 16 bit registers
            pha                   ;save .C
            phx                   ;save .X
            phy                   ;save .Y
            LDA  STKPC,S          ;save pc location
            DEC  a                ;less one
            STA  CPRO             ;save signature byte location
            LDA  (CPRO)           ;load sig byte
            STA  CPRO             ;store sig byte
            rep #%00110000        ;16 bit registers
            ply                   ;restore .Y
            plx                   ;restore .X
            pla                   ;restore .C
            pld                   ;restore DP
            plb                   ;restore DB
            CLI
            rti                   ;resume foreground task
IRQ_ISA16   SEI
            phb                   ;save DB
            phd                   ;save DP
            rep #%00110000        ;select 16 bit registers
            pha                   ;save .C
            phx                   ;save .X
            phy                   ;save .Y
            LDA #$01
			STA $F2
			INC $F4
            rep #%00110000        ;16 bit registers
            ply                   ;restore .Y
            plx                   ;restore .X
            pla                   ;restore .C
            pld                   ;restore DP
            plb                   ;restore DB
            CLI
            rti                   ;resume foreground task
;
NMI_ISA16   SEI
            phb                   ;save DB
            phd                   ;save DP
            rep #%00110000        ;select 16 bit registers
            pha                   ;save .C
            phx                   ;save .X
            phy                   ;save .Y
            ;actual routine
            rep #%00110000        ;16 bit registers
            ply                   ;restore .Y
            plx                   ;restore .X
            pla                   ;restore .C
            pld                   ;restore DP
            plb                   ;restore DB
            CLI
            rti                   ;resume foreground task
;8-bit Interrupt Service Routines
CO_PRO      SEI
            PHA
            PHX
            PHY
            LDA  STKPC8,S          ;save pc location
            DEC  a                ;less one
            STA  CPRO           ;save signature byte location
            LDA  (CPRO)           ;load sig byte
            STA  CPRO             ;store sig byte
            PLY
            PLX
            PLA
            CLI
            rti                   ;resume foreground task
IRQ_ISA     SEI
            PHA
            PHX
            PHY
            LDA #$01
			STA $F2
			INC $F4
            PLY
            PLX
            PLA
            CLI
            RTI
;
NMI_ISA     SEI
            PHA
            PHX
            PHY
            ;actual routine
            PLY
            PLX
            PLA
            CLI
            RTI
;=========================================================================
;
;
;  CRC subroutines 
;
;
UpdCrc		eor 	crc+1 		; Quick CRC computation with lookup tables
       		tax		 	; updates the two bytes at crc & crc+1
       		lda 	crc		; with the byte send in the "A" register
       		eor 	CRCHI,X
       		sta 	crc+1
      	 	lda 	CRCLO,X
       		sta 	crc
       		rts
;
; The following tables are used to calculate the CRC for the 128 bytes
; in the xmodem data blocks.  You can use these tables if you plan to 
; store this program in ROM.  If you choose to build them at run-time, 
; then just delete them and define the two labels: crclo & crchi.
;
; low byte CRC lookup table (should be page aligned)
		*= $ED00
crclo
 .byte $00,$21,$42,$63,$84,$A5,$C6,$E7,$08,$29,$4A,$6B,$8C,$AD,$CE,$EF
 .byte $31,$10,$73,$52,$B5,$94,$F7,$D6,$39,$18,$7B,$5A,$BD,$9C,$FF,$DE
 .byte $62,$43,$20,$01,$E6,$C7,$A4,$85,$6A,$4B,$28,$09,$EE,$CF,$AC,$8D
 .byte $53,$72,$11,$30,$D7,$F6,$95,$B4,$5B,$7A,$19,$38,$DF,$FE,$9D,$BC
 .byte $C4,$E5,$86,$A7,$40,$61,$02,$23,$CC,$ED,$8E,$AF,$48,$69,$0A,$2B
 .byte $F5,$D4,$B7,$96,$71,$50,$33,$12,$FD,$DC,$BF,$9E,$79,$58,$3B,$1A
 .byte $A6,$87,$E4,$C5,$22,$03,$60,$41,$AE,$8F,$EC,$CD,$2A,$0B,$68,$49
 .byte $97,$B6,$D5,$F4,$13,$32,$51,$70,$9F,$BE,$DD,$FC,$1B,$3A,$59,$78
 .byte $88,$A9,$CA,$EB,$0C,$2D,$4E,$6F,$80,$A1,$C2,$E3,$04,$25,$46,$67
 .byte $B9,$98,$FB,$DA,$3D,$1C,$7F,$5E,$B1,$90,$F3,$D2,$35,$14,$77,$56
 .byte $EA,$CB,$A8,$89,$6E,$4F,$2C,$0D,$E2,$C3,$A0,$81,$66,$47,$24,$05
 .byte $DB,$FA,$99,$B8,$5F,$7E,$1D,$3C,$D3,$F2,$91,$B0,$57,$76,$15,$34
 .byte $4C,$6D,$0E,$2F,$C8,$E9,$8A,$AB,$44,$65,$06,$27,$C0,$E1,$82,$A3
 .byte $7D,$5C,$3F,$1E,$F9,$D8,$BB,$9A,$75,$54,$37,$16,$F1,$D0,$B3,$92
 .byte $2E,$0F,$6C,$4D,$AA,$8B,$E8,$C9,$26,$07,$64,$45,$A2,$83,$E0,$C1
 .byte $1F,$3E,$5D,$7C,$9B,$BA,$D9,$F8,$17,$36,$55,$74,$93,$B2,$D1,$F0 

; hi byte CRC lookup table (should be page aligned)
		*= $EE00
crchi
 .byte $00,$10,$20,$30,$40,$50,$60,$70,$81,$91,$A1,$B1,$C1,$D1,$E1,$F1
 .byte $12,$02,$32,$22,$52,$42,$72,$62,$93,$83,$B3,$A3,$D3,$C3,$F3,$E3
 .byte $24,$34,$04,$14,$64,$74,$44,$54,$A5,$B5,$85,$95,$E5,$F5,$C5,$D5
 .byte $36,$26,$16,$06,$76,$66,$56,$46,$B7,$A7,$97,$87,$F7,$E7,$D7,$C7
 .byte $48,$58,$68,$78,$08,$18,$28,$38,$C9,$D9,$E9,$F9,$89,$99,$A9,$B9
 .byte $5A,$4A,$7A,$6A,$1A,$0A,$3A,$2A,$DB,$CB,$FB,$EB,$9B,$8B,$BB,$AB
 .byte $6C,$7C,$4C,$5C,$2C,$3C,$0C,$1C,$ED,$FD,$CD,$DD,$AD,$BD,$8D,$9D
 .byte $7E,$6E,$5E,$4E,$3E,$2E,$1E,$0E,$FF,$EF,$DF,$CF,$BF,$AF,$9F,$8F
 .byte $91,$81,$B1,$A1,$D1,$C1,$F1,$E1,$10,$00,$30,$20,$50,$40,$70,$60
 .byte $83,$93,$A3,$B3,$C3,$D3,$E3,$F3,$02,$12,$22,$32,$42,$52,$62,$72
 .byte $B5,$A5,$95,$85,$F5,$E5,$D5,$C5,$34,$24,$14,$04,$74,$64,$54,$44
 .byte $A7,$B7,$87,$97,$E7,$F7,$C7,$D7,$26,$36,$06,$16,$66,$76,$46,$56
 .byte $D9,$C9,$F9,$E9,$99,$89,$B9,$A9,$58,$48,$78,$68,$18,$08,$38,$28
 .byte $CB,$DB,$EB,$FB,$8B,$9B,$AB,$BB,$4A,$5A,$6A,$7A,$0A,$1A,$2A,$3A
 .byte $FD,$ED,$DD,$CD,$BD,$AD,$9D,$8D,$7C,$6C,$5C,$4C,$3C,$2C,$1C,$0C
 .byte $EF,$FF,$CF,$DF,$AF,$BF,$8F,$9F,$6E,$7E,$4E,$5E,$2E,$3E,$0E,$1E 
;
; End of File
;*****************************************************
;	Delay
;
;	Delay For 65535 Cycles 
;
;*****************************************************
Delay
		phx
		phy
		ldy #$40 ; restore to #$00
AL2		ldx #$10 ; restore to #$00
AL1		dex
		bne AL1
		dey
		bne AL2
		ply
		plx
		rts
;
MSG1      .textz "Welcome to JSMON 3.0 Native16."
MSG5      .textz "SWEET16 Enabled. JSR at $C000."
MSG6      .textz "Stack and Zero Page Cleared!" 
MSG7      .textz "Press L to c[L]ear zp and Stack."  
                .ORG    $FFE4
COP_VEC16       .word   CO_PRO16      ;co-processor vector
                .ORG    $FFEA
NMI_VEC16       .word   NMI_ISA16           ;IRQ vector
RESET_VEC16     .word   RESET           ;RESET vector
IRQ_VEC16       .word   IRQ_ISA16           ;IRQ vector
                .ORG    $FFF4
COP_VEC         .word   CO_PRO          ;co-processor vector
                .ORG    $FFFA
NMI_VEC         .word   NMI_ISA           ;IRQ vector
RESET_VEC       .word   RESET           ;RESET vector
IRQ_VEC         .word   IRQ_ISA           ;IRQ vector
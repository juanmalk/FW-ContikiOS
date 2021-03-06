;
; Copyright (c) 2007, Adam Dunkels and Oliver Schmidt
; All rights reserved. 
;
; Redistribution and use in source and binary forms, with or without 
; modification, are permitted provided that the following conditions 
; are met: 
; 1. Redistributions of source code must retain the above copyright 
;    notice, this list of conditions and the following disclaimer. 
; 2. Redistributions in binary form must reproduce the above copyright 
;    notice, this list of conditions and the following disclaimer in the 
;    documentation and/or other materials provided with the distribution. 
; 3. Neither the name of the Institute nor the names of its contributors 
;    may be used to endorse or promote products derived from this software 
;    without specific prior written permission. 
;
; THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND 
; ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
; ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE 
; FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
; DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
; OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
; HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
; LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
; OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
; SUCH DAMAGE. 
;
; This file is part of the Contiki operating system.
; 
; Author: Adam Dunkels <adam@sics.se>, Oliver Schmidt <ol.sc@web.de>
;
;---------------------------------------------------------------------

	.macpack	module
	module_header	_cs8900a

	; Driver signature
	.byte	$65, $74, $68	; "eth"
	.byte	$01		; Ethernet driver API version number

	; Ethernet address
mac:	.byte	$00, $0E, $3A	; OUI of Cirrus Logic
	.byte	$11, $11, $11

	; Buffer attributes
bufaddr:.res	2		; Address
bufsize:.res	2		; Size

	; Jump table.
	jmp init
	jmp poll
	jmp send
	jmp exit

;---------------------------------------------------------------------

	.if DYN_DRV

	.zeropage
sp:	.res	2		; Stack pointer (Do not trash !)
reg:	.res	2		; Address of rxtxreg
ptr:	.res	2		; Indirect addressing pointer
len:	.res	2		; Frame length
cnt:	.res	2		; Frame length counter

	.else

	.include "zeropage.inc"
reg	:=	ptr1		; Address of rxtxreg
ptr	:=	ptr2		; Indirect addressing pointer
len	:=	ptr3		; Frame length
cnt	:=	ptr4		; Frame length counter

	.endif

;---------------------------------------------------------------------

	.rodata

fixup:	.byte	fixup02-fixup01, fixup03-fixup02, fixup04-fixup03
	.byte	fixup05-fixup04, fixup06-fixup05, fixup07-fixup06
	.byte	fixup08-fixup07, fixup09-fixup08, fixup10-fixup09
	.byte	fixup11-fixup10, fixup12-fixup11, fixup13-fixup12
	.byte	fixup14-fixup13, fixup15-fixup14, fixup16-fixup15
	.byte	fixup17-fixup16, fixup18-fixup17, fixup19-fixup18
	.byte	fixup20-fixup19, fixup21-fixup20, fixup22-fixup21
	.byte	fixup23-fixup22, fixup24-fixup23, fixup25-fixup24
	.byte	fixup26-fixup25

fixups	= * - fixup

;---------------------------------------------------------------------

; 3 most significant nibbles are fixed up at runtime
rxtxreg		:= $FFF0
txcmd		:= $FFF4
txlen		:= $FFF6
isq		:= $FFF8
packetpp	:= $FFFA
ppdata		:= $FFFC

	.data

;---------------------------------------------------------------------

init:
	; Save address of rxtxreg
	sta reg
	stx reg+1

	; Start with first fixup location
	lda #<(fixup01+1)
	ldx #>(fixup01+1)
	sta ptr
	stx ptr+1
	ldx #$FF
	ldy #$00

	; Fixup address at location
:	lda (ptr),y
	and #$0F
	eor reg			; Use XOR to support C64 RR-Net
	sta (ptr),y
	iny
	lda reg+1
	sta (ptr),y
	dey

	; Advance to next fixup location
	inx
	cpx #fixups
	bcs :+
	lda ptr
	clc
	adc fixup,x
	sta ptr
	bcc :-
	inc ptr+1
	bcs :-			; Always

	; Activate C64 RR clockport in order to operate RR-Net
	; - RR config register overlays CS8900A ISQ register
	; - No need to distinguish as ISQ access doesn't hurt
:
fixup01:lda isq+1
	ora #$01		; Set clockport bit
fixup02:sta isq+1

	; Check EISA registration number of Crystal Semiconductor
	; PACKETPP = $0000, PPDATA == $630E ?
	lda #$00
	tax
	jsr packetpp_ax
	lda #$63^$0E
fixup03:eor ppdata
fixup04:eor ppdata+1
	beq :+
	sec
	rts

	; Initiate a chip-wide reset
	; PACKETPP = $0114, PPDATA = $0040
:	lda #$14
	jsr packetpp_a1
	ldy #$40
fixup05:sty ppdata
:	jsr packetpp_a1
fixup06:ldy ppdata
	and #$40
	bne :-

	; Accept valid unicast + broadcast frames
	; PACKETPP = $0104, PPDATA = $0D05
	lda #$04
	jsr packetpp_a1
	lda #$05
	ldx #$0D
	jsr ppdata_ax

	; Set MAC address
	; PACKETPP = $0158, PPDATA = MAC[0], MAC[1]
	; PACKETPP = $015A, PPDATA = MAC[2], MAC[3]
	; PACKETPP = $015C, PPDATA = MAC[4], MAC[5]
	ldy #$58
:	tya
	jsr packetpp_a1
	lda mac-$58,y
	ldx mac-$58+1,y
	jsr ppdata_ax
	iny
	iny
	cpy #$58+6
	bcc :-

	; Turn on transmission and reception of frames
	; PACKETPP = $0112, PPDATA = $00D3
	lda #$12
	jsr packetpp_a1
	lda #$D3
	ldx #$00
	jsr ppdata_ax
	txa
	clc
	rts

;---------------------------------------------------------------------

poll:
	; Check receiver event register to see if there
	; are any valid unicast frames avaliable
	; PACKETPP = $0124, PPDATA & $0D00 ?
	lda #$24
	jsr packetpp_a1
fixup07:lda ppdata+1
	and #$0D
	beq :+

	; Process the incoming frame
	; --------------------------
	
	; Read receiver event and discard it
	; RXTXREG
fixup08:ldx rxtxreg+1
fixup09:lda rxtxreg

	; Read frame length
	; cnt = len = RXTXREG
fixup10:ldx rxtxreg+1
fixup11:lda rxtxreg
	sta len
	stx len+1
	sta cnt
	stx cnt+1

	; Adjust odd frame length
	jsr adjustcnt

	; Is bufsize < cnt ?
	lda bufsize
	cmp cnt
	lda bufsize+1
	sbc cnt+1
	bcs :++

	; Yes, skip frame
	jsr skipframe

	; No frame ready
	lda #$00
:	tax
	sec
	rts

	; Read bytes into buffer
:	jsr adjustptr
:
fixup12:lda rxtxreg
	sta (ptr),y
	iny
fixup13:lda rxtxreg+1
	sta (ptr),y
	iny
	bne :-
	inc ptr+1
	dex
	bpl :-

	; Return frame length
	lda len
	ldx len+1
	clc
	rts

;---------------------------------------------------------------------

send:
	; Save frame length
	sta cnt
	stx cnt+1

	; Transmit command
	lda #$C9
	ldx #$00
fixup14:sta txcmd
fixup15:stx txcmd+1
	lda cnt
	ldx cnt+1
fixup16:sta txlen
fixup17:stx txlen+1

	; Adjust odd frame length
	jsr adjustcnt

	; 8 retries
	ldy #$08

	; Check for avaliable buffer space
	; PACKETPP = $0138, PPDATA & $0100 ?
:	lda #$38
	jsr packetpp_a1
fixup18:lda ppdata+1
	and #$01
	bne :+

	; No space avaliable, skip a received frame
	jsr skipframe

	; And try again
	dey
	bne :-
	sec
	rts

	; Send the frame
	; --------------

	; Write bytes from buffer
:	jsr adjustptr
:	lda (ptr),y
fixup19:sta rxtxreg
	iny
	lda (ptr),y
fixup20:sta rxtxreg+1
	iny
	bne :-
	inc ptr+1
	dex
	bpl :-
	clc
	rts

;---------------------------------------------------------------------

exit:
	rts

;---------------------------------------------------------------------

packetpp_a1:
	ldx #$01
packetpp_ax:
fixup21:sta packetpp
fixup22:stx packetpp+1
	rts

;---------------------------------------------------------------------

ppdata_ax:
fixup23:sta ppdata
fixup24:stx ppdata+1
	rts

;---------------------------------------------------------------------

skipframe:
	; PACKETPP = $0102, PPDATA = PPDATA | $0040
	lda #$02
	jsr packetpp_a1
fixup25:lda ppdata
	ora #$40
fixup26:sta ppdata
	rts

;---------------------------------------------------------------------

adjustcnt:
	lsr
	bcc :+
	inc cnt
	bne :+
	inc cnt+1
:	rts

;---------------------------------------------------------------------

adjustptr:
	lda cnt
	ldx cnt+1
	eor #$FF		; Two's complement part 1
	tay
	iny			; Two's complement part 2
	sty reg
	sec
	lda bufaddr
	sbc reg
	sta ptr
	lda bufaddr+1
	sbc #$00
	sta ptr+1
	rts

;---------------------------------------------------------------------

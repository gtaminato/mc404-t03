	.file	"user.c"
	.file 1 "user.c"
	.section	.debug_abbrev,"",%progbits
.Ldebug_abbrev0:
	.section	.debug_info,"",%progbits
.Ldebug_info0:
	.section	.debug_line,"",%progbits
.Ldebug_line0:
	.text
.Ltext0:
	.align	2
	.global	fib
	.type	fib, %function
fib:
.LFB3:
	.loc 1 28 0
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
.LCFI0:
	stmfd	sp!, {r4, fp, ip, lr, pc}
.LCFI1:
	sub	fp, ip, #4
.LCFI2:
	sub	sp, sp, #8
.LCFI3:
	str	r0, [fp, #-20]
	.loc 1 29 0
	ldr	r3, [fp, #-20]
	cmp	r3, #1
	bgt	.L2
	.loc 1 30 0
	ldr	r3, [fp, #-20]
	str	r3, [fp, #-24]
	b	.L1
.L2:
	.loc 1 32 0
	ldr	r3, [fp, #-20]
	sub	r3, r3, #1
	mov	r0, r3
	bl	fib
	mov	r4, r0
	ldr	r3, [fp, #-20]
	sub	r3, r3, #2
	mov	r0, r3
	bl	fib
	mov	r3, r0
	add	r4, r4, r3
	str	r4, [fp, #-24]
	.loc 1 33 0
.L1:
	ldr	r0, [fp, #-24]
	ldmea	fp, {r4, fp, sp, pc}
.LFE3:
	.size	fib, .-fib
	.global	__modsi3
	.global	__divsi3
	.align	2
	.global	int2string
	.type	int2string, %function
int2string:
.LFB5:
	.loc 1 37 0
	@ args = 0, pretend = 0, frame = 284
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
.LCFI4:
	stmfd	sp!, {fp, ip, lr, pc}
.LCFI5:
	sub	fp, ip, #4
.LCFI6:
	sub	sp, sp, #284
.LCFI7:
	str	r0, [fp, #-16]
	str	r1, [fp, #-20]
	.loc 1 39 0
.LBB2:
	mov	r3, #0
	str	r3, [fp, #-280]
	.loc 1 40 0
	mov	r3, #0
	str	r3, [fp, #-288]
	.loc 1 41 0
	ldr	r3, [fp, #-20]
	cmp	r3, #0
	bge	.L5
	.loc 1 42 0
	sub	r0, fp, #16
	ldr	r2, [r0, #0]
	mov	r1, r2
	mov	r3, #45
	strb	r3, [r1, #0]
	add	r2, r2, #1
	str	r2, [r0, #0]
	.loc 1 43 0
	ldr	r3, [fp, #-20]
	mvn	r3, r3
	add	r3, r3, #1
	str	r3, [fp, #-20]
	.loc 1 44 0
	ldr	r3, [fp, #-288]
	add	r3, r3, #1
	str	r3, [fp, #-288]
.L5:
	mov	r0, r0	@ nop
.L6:
	.loc 1 47 0
.LBB3:
	ldr	r3, [fp, #-20]
	mov	r0, r3
	mov	r1, #10
	bl	__modsi3
	mov	r3, r0
	str	r3, [fp, #-292]
	.loc 1 48 0
	ldr	r3, [fp, #-20]
	mov	r0, r3
	mov	r1, #10
	bl	__divsi3
	mov	r3, r0
	str	r3, [fp, #-20]
	.loc 1 49 0
	ldrb	r3, [fp, #-292]
	add	r3, r3, #48
	strb	r3, [fp, #-293]
	.loc 1 50 0
	sub	r0, fp, #280
	ldr	r1, [r0, #0]
	mov	r3, #-1107296256
	mov	r3, r3, asr #22
	mov	r2, r1
	sub	ip, fp, #12
	add	r2, r2, ip
	add	r2, r2, r3
	ldrb	r3, [fp, #-293]
	strb	r3, [r2, #0]
	add	r1, r1, #1
	str	r1, [r0, #0]
	.loc 1 51 0
.LBE3:
	ldr	r3, [fp, #-20]
	cmp	r3, #0
	bne	.L6
	.loc 1 52 0
	mov	r3, #0
	str	r3, [fp, #-284]
.L10:
	ldr	r2, [fp, #-284]
	ldr	r3, [fp, #-280]
	cmp	r2, r3
	blt	.L13
	b	.L11
.L13:
	.loc 1 53 0
	sub	r0, fp, #16
	ldr	r1, [r0, #0]
	ldr	r2, [fp, #-280]
	ldr	r3, [fp, #-284]
	rsb	r3, r3, r2
	mvn	r2, #264
	sub	ip, fp, #12
	add	r3, ip, r3
	add	r3, r3, r2
	mov	r2, r1
	ldrb	r3, [r3, #0]
	strb	r3, [r2, #0]
	add	r1, r1, #1
	str	r1, [r0, #0]
	.loc 1 54 0
	ldr	r3, [fp, #-288]
	add	r3, r3, #1
	str	r3, [fp, #-288]
	.loc 1 52 0
	ldr	r3, [fp, #-284]
	add	r3, r3, #1
	str	r3, [fp, #-284]
	b	.L10
.L11:
	.loc 1 56 0
	ldr	r3, [fp, #-288]
	.loc 1 57 0
.LBE2:
	mov	r0, r3
	ldmea	fp, {fp, sp, pc}
.LFE5:
	.size	int2string, .-int2string
	.align	2
	.global	str2str
	.type	str2str, %function
str2str:
.LFB7:
	.loc 1 61 0
	@ args = 0, pretend = 0, frame = 12
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
.LCFI8:
	stmfd	sp!, {fp, ip, lr, pc}
.LCFI9:
	sub	fp, ip, #4
.LCFI10:
	sub	sp, sp, #12
.LCFI11:
	str	r0, [fp, #-16]
	str	r1, [fp, #-20]
	.loc 1 62 0
.LBB4:
	mov	r3, #0
	str	r3, [fp, #-24]
	.loc 1 63 0
.L15:
	ldr	r3, [fp, #-20]
	ldrb	r3, [r3, #0]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L17
	b	.L16
.L17:
	.loc 1 64 0
	sub	lr, fp, #16
	ldr	r1, [lr, #0]
	sub	ip, fp, #20
	ldr	r2, [ip, #0]
	mov	r0, r1
	mov	r3, r2
	ldrb	r3, [r3, #0]
	strb	r3, [r0, #0]
	add	r2, r2, #1
	str	r2, [ip, #0]
	add	r1, r1, #1
	str	r1, [lr, #0]
	.loc 1 65 0
	ldr	r3, [fp, #-24]
	add	r3, r3, #1
	str	r3, [fp, #-24]
	b	.L15
.L16:
	.loc 1 67 0
	ldr	r3, [fp, #-24]
	.loc 1 68 0
.LBE4:
	mov	r0, r3
	ldmea	fp, {fp, sp, pc}
.LFE7:
	.size	str2str, .-str2str
	.section	.rodata
	.align	2
.LC0:
	.ascii	"PID \000"
	.align	2
.LC1:
	.ascii	": Processo encerrando.\n\000"
	.text
	.align	2
	.global	adeus
	.type	adeus, %function
adeus:
.LFB9:
	.loc 1 70 0
	@ args = 0, pretend = 0, frame = 204
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
.LCFI12:
	stmfd	sp!, {fp, ip, lr, pc}
.LCFI13:
	sub	fp, ip, #4
.LCFI14:
	sub	sp, sp, #204
.LCFI15:
	.loc 1 72 0
.LBB5:
	sub	r3, fp, #212
	str	r3, [fp, #-216]
	.loc 1 73 0
	ldr	r0, [fp, #-216]
	ldr	r1, .L19
	bl	str2str
	mov	r2, r0
	ldr	r3, [fp, #-216]
	add	r3, r3, r2
	str	r3, [fp, #-216]
	.loc 1 74 0
	bl	getpid
	mov	r3, r0
	ldr	r0, [fp, #-216]
	mov	r1, r3
	bl	int2string
	mov	r2, r0
	ldr	r3, [fp, #-216]
	add	r3, r3, r2
	str	r3, [fp, #-216]
	.loc 1 75 0
	ldr	r0, [fp, #-216]
	ldr	r1, .L19+4
	bl	str2str
	mov	r2, r0
	ldr	r3, [fp, #-216]
	add	r3, r3, r2
	str	r3, [fp, #-216]
	.loc 1 76 0
	sub	r1, fp, #212
	sub	r2, fp, #212
	ldr	r3, [fp, #-216]
	rsb	r3, r2, r3
	mov	r0, #1
	mov	r2, r3
	bl	write
	.loc 1 77 0
	mov	r0, #0
	bl	_exit
.L20:
	.align	2
.L19:
	.word	.LC0
	.word	.LC1
	.loc 1 78 0
.LBE5:
.LFE9:
	.size	adeus, .-adeus
	.section	.rodata
	.align	2
.LC2:
	.ascii	": Fibonacci(\000"
	.align	2
.LC3:
	.ascii	") = \000"
	.align	2
.LC4:
	.ascii	"\n\000"
	.text
	.align	2
	.global	codigo_processo_1
	.type	codigo_processo_1, %function
codigo_processo_1:
.LFB11:
	.loc 1 80 0
	@ args = 0, pretend = 0, frame = 212
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
.LCFI16:
	stmfd	sp!, {fp, ip, lr, pc}
.LCFI17:
	sub	fp, ip, #4
.LCFI18:
	sub	sp, sp, #212
.LCFI19:
	str	r0, [fp, #-16]
	.loc 1 83 0
.LBB6:
	sub	r3, fp, #216
	str	r3, [fp, #-220]
	.loc 1 84 0
	ldr	r0, [fp, #-16]
	bl	fib
	mov	r3, r0
	str	r3, [fp, #-224]
	.loc 1 85 0
	ldr	r0, [fp, #-220]
	ldr	r1, .L22
	bl	str2str
	mov	r2, r0
	ldr	r3, [fp, #-220]
	add	r3, r3, r2
	str	r3, [fp, #-220]
	.loc 1 86 0
	bl	getpid
	mov	r3, r0
	ldr	r0, [fp, #-220]
	mov	r1, r3
	bl	int2string
	mov	r2, r0
	ldr	r3, [fp, #-220]
	add	r3, r3, r2
	str	r3, [fp, #-220]
	.loc 1 87 0
	ldr	r0, [fp, #-220]
	ldr	r1, .L22+4
	bl	str2str
	mov	r2, r0
	ldr	r3, [fp, #-220]
	add	r3, r3, r2
	str	r3, [fp, #-220]
	.loc 1 88 0
	ldr	r0, [fp, #-220]
	ldr	r1, [fp, #-16]
	bl	int2string
	mov	r2, r0
	ldr	r3, [fp, #-220]
	add	r3, r3, r2
	str	r3, [fp, #-220]
	.loc 1 89 0
	ldr	r0, [fp, #-220]
	ldr	r1, .L22+8
	bl	str2str
	mov	r2, r0
	ldr	r3, [fp, #-220]
	add	r3, r3, r2
	str	r3, [fp, #-220]
	.loc 1 90 0
	ldr	r0, [fp, #-220]
	ldr	r1, [fp, #-224]
	bl	int2string
	mov	r2, r0
	ldr	r3, [fp, #-220]
	add	r3, r3, r2
	str	r3, [fp, #-220]
	.loc 1 91 0
	ldr	r0, [fp, #-220]
	ldr	r1, .L22+12
	bl	str2str
	mov	r2, r0
	ldr	r3, [fp, #-220]
	add	r3, r3, r2
	str	r3, [fp, #-220]
	.loc 1 92 0
	sub	r1, fp, #216
	sub	r2, fp, #216
	ldr	r3, [fp, #-220]
	rsb	r3, r2, r3
	mov	r0, #1
	mov	r2, r3
	bl	write
	.loc 1 93 0
	bl	adeus
	.loc 1 94 0
	ldmea	fp, {fp, sp, pc}
.L23:
	.align	2
.L22:
	.word	.LC0
	.word	.LC2
	.word	.LC3
	.word	.LC4
.LBE6:
.LFE11:
	.size	codigo_processo_1, .-codigo_processo_1
	.section	.rodata
	.align	2
.LC5:
	.ascii	": Estou vivo!\n\000"
	.text
	.align	2
	.global	codigo_processo_2
	.type	codigo_processo_2, %function
codigo_processo_2:
.LFB13:
	.loc 1 100 0
	@ args = 0, pretend = 0, frame = 216
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
.LCFI20:
	stmfd	sp!, {fp, ip, lr, pc}
.LCFI21:
	sub	fp, ip, #4
.LCFI22:
	sub	sp, sp, #216
.LCFI23:
	str	r0, [fp, #-16]
	.loc 1 102 0
.LBB7:
	mov	r3, #0
	str	r3, [fp, #-20]
.L25:
	ldr	r3, [fp, #-20]
	cmp	r3, #9
	ble	.L28
	b	.L26
.L28:
	.loc 1 103 0
.LBB8:
	mov	r3, #0
	str	r3, [fp, #-24]
.L29:
	ldr	r2, [fp, #-24]
	ldr	r3, [fp, #-16]
	cmp	r2, r3
	blt	.L31
	b	.L30
.L31:
	ldr	r3, [fp, #-24]
	add	r3, r3, #1
	str	r3, [fp, #-24]
	b	.L29
.L30:
	.loc 1 106 0
	sub	r3, fp, #224
	str	r3, [fp, #-228]
	.loc 1 107 0
	ldr	r0, [fp, #-228]
	ldr	r1, .L33
	bl	str2str
	mov	r2, r0
	ldr	r3, [fp, #-228]
	add	r3, r3, r2
	str	r3, [fp, #-228]
	.loc 1 108 0
	bl	getpid
	mov	r3, r0
	ldr	r0, [fp, #-228]
	mov	r1, r3
	bl	int2string
	mov	r2, r0
	ldr	r3, [fp, #-228]
	add	r3, r3, r2
	str	r3, [fp, #-228]
	.loc 1 109 0
	ldr	r0, [fp, #-228]
	ldr	r1, .L33+4
	bl	str2str
	mov	r2, r0
	ldr	r3, [fp, #-228]
	add	r3, r3, r2
	str	r3, [fp, #-228]
	.loc 1 110 0
	sub	r1, fp, #224
	sub	r2, fp, #224
	ldr	r3, [fp, #-228]
	rsb	r3, r2, r3
	mov	r0, #1
	mov	r2, r3
	bl	write
	.loc 1 102 0
.LBE8:
	ldr	r3, [fp, #-20]
	add	r3, r3, #1
	str	r3, [fp, #-20]
	b	.L25
.L26:
	.loc 1 112 0
	bl	adeus
	.loc 1 113 0
	ldmea	fp, {fp, sp, pc}
.L34:
	.align	2
.L33:
	.word	.LC0
	.word	.LC5
.LBE7:
.LFE13:
	.size	codigo_processo_2, .-codigo_processo_2
	.section	.rodata
	.align	2
.LC6:
	.ascii	": Criando processos filhos.\n\000"
	.text
	.align	2
	.global	main
	.type	main, %function
main:
.LFB15:
	.loc 1 116 0
	@ args = 0, pretend = 0, frame = 208
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
.LCFI24:
	stmfd	sp!, {fp, ip, lr, pc}
.LCFI25:
	sub	fp, ip, #4
.LCFI26:
	sub	sp, sp, #208
.LCFI27:
	.loc 1 117 0
.LBB9:
	mov	r3, #0
	str	r3, [fp, #-16]
	.loc 1 120 0
	sub	r3, fp, #216
	str	r3, [fp, #-220]
	.loc 1 121 0
	ldr	r0, [fp, #-220]
	ldr	r1, .L43
	bl	str2str
	mov	r2, r0
	ldr	r3, [fp, #-220]
	add	r3, r3, r2
	str	r3, [fp, #-220]
	.loc 1 122 0
	bl	getpid
	mov	r3, r0
	ldr	r0, [fp, #-220]
	mov	r1, r3
	bl	int2string
	mov	r2, r0
	ldr	r3, [fp, #-220]
	add	r3, r3, r2
	str	r3, [fp, #-220]
	.loc 1 123 0
	ldr	r0, [fp, #-220]
	ldr	r1, .L43+4
	bl	str2str
	mov	r2, r0
	ldr	r3, [fp, #-220]
	add	r3, r3, r2
	str	r3, [fp, #-220]
	.loc 1 124 0
	sub	r1, fp, #216
	sub	r2, fp, #216
	ldr	r3, [fp, #-220]
	rsb	r3, r2, r3
	mov	r0, #1
	mov	r2, r3
	bl	write
	.loc 1 127 0
	bl	fork
	str	r0, [fp, #-16]
	.loc 1 128 0
	ldr	r3, [fp, #-16]
	cmp	r3, #0
	bne	.L36
	.loc 1 129 0
	mov	r0, #30
	bl	codigo_processo_1
.L36:
	.loc 1 132 0
	bl	fork
	str	r0, [fp, #-16]
	.loc 1 133 0
	ldr	r3, [fp, #-16]
	cmp	r3, #0
	bne	.L37
	.loc 1 134 0
	mov	r0, #999424
	add	r0, r0, #576
	bl	codigo_processo_2
.L37:
	.loc 1 136 0
	bl	fork
	str	r0, [fp, #-16]
	.loc 1 137 0
	ldr	r3, [fp, #-16]
	cmp	r3, #0
	bne	.L38
	.loc 1 138 0
	mov	r0, #31
	bl	codigo_processo_1
.L38:
	.loc 1 140 0
	bl	fork
	str	r0, [fp, #-16]
	.loc 1 141 0
	ldr	r3, [fp, #-16]
	cmp	r3, #0
	bne	.L39
	.loc 1 142 0
	mov	r0, #1998848
	add	r0, r0, #1152
	bl	codigo_processo_2
.L39:
	.loc 1 144 0
	bl	fork
	str	r0, [fp, #-16]
	.loc 1 145 0
	ldr	r3, [fp, #-16]
	cmp	r3, #0
	bne	.L40
	.loc 1 146 0
	mov	r0, #32
	bl	codigo_processo_1
.L40:
	.loc 1 148 0
	bl	fork
	str	r0, [fp, #-16]
	.loc 1 149 0
	ldr	r3, [fp, #-16]
	cmp	r3, #0
	bne	.L41
	.loc 1 150 0
	mov	r0, #1490944
	add	r0, r0, #9024
	add	r0, r0, #32
	bl	codigo_processo_2
.L41:
	.loc 1 152 0
	bl	fork
	str	r0, [fp, #-16]
	.loc 1 153 0
	ldr	r3, [fp, #-16]
	cmp	r3, #0
	bne	.L42
	.loc 1 154 0
	mov	r0, #28
	bl	codigo_processo_1
.L42:
	.loc 1 157 0
	bl	adeus
	.loc 1 158 0
.LBE9:
	mov	r0, r3
	ldmea	fp, {fp, sp, pc}
.L44:
	.align	2
.L43:
	.word	.LC0
	.word	.LC6
.LFE15:
	.size	main, .-main
	.section	.debug_frame,"",%progbits
.Lframe0:
	.4byte	.LECIE0-.LSCIE0
.LSCIE0:
	.4byte	0xffffffff
	.byte	0x1
	.ascii	"\000"
	.uleb128 0x1
	.sleb128 -4
	.byte	0xe
	.byte	0xc
	.uleb128 0xd
	.uleb128 0x0
	.align	2
.LECIE0:
.LSFDE0:
	.4byte	.LEFDE0-.LASFDE0
.LASFDE0:
	.4byte	.Lframe0
	.4byte	.LFB3
	.4byte	.LFE3-.LFB3
	.byte	0x4
	.4byte	.LCFI0-.LFB3
	.byte	0xd
	.uleb128 0xc
	.byte	0x4
	.4byte	.LCFI1-.LCFI0
	.byte	0x8e
	.uleb128 0x2
	.byte	0x8d
	.uleb128 0x3
	.byte	0x8b
	.uleb128 0x4
	.byte	0x84
	.uleb128 0x5
	.byte	0x4
	.4byte	.LCFI2-.LCFI1
	.byte	0xc
	.uleb128 0xb
	.uleb128 0x4
	.align	2
.LEFDE0:
.LSFDE2:
	.4byte	.LEFDE2-.LASFDE2
.LASFDE2:
	.4byte	.Lframe0
	.4byte	.LFB5
	.4byte	.LFE5-.LFB5
	.byte	0x4
	.4byte	.LCFI4-.LFB5
	.byte	0xd
	.uleb128 0xc
	.byte	0x4
	.4byte	.LCFI5-.LCFI4
	.byte	0x8e
	.uleb128 0x2
	.byte	0x8d
	.uleb128 0x3
	.byte	0x8b
	.uleb128 0x4
	.byte	0x4
	.4byte	.LCFI6-.LCFI5
	.byte	0xc
	.uleb128 0xb
	.uleb128 0x4
	.align	2
.LEFDE2:
.LSFDE4:
	.4byte	.LEFDE4-.LASFDE4
.LASFDE4:
	.4byte	.Lframe0
	.4byte	.LFB7
	.4byte	.LFE7-.LFB7
	.byte	0x4
	.4byte	.LCFI8-.LFB7
	.byte	0xd
	.uleb128 0xc
	.byte	0x4
	.4byte	.LCFI9-.LCFI8
	.byte	0x8e
	.uleb128 0x2
	.byte	0x8d
	.uleb128 0x3
	.byte	0x8b
	.uleb128 0x4
	.byte	0x4
	.4byte	.LCFI10-.LCFI9
	.byte	0xc
	.uleb128 0xb
	.uleb128 0x4
	.align	2
.LEFDE4:
.LSFDE6:
	.4byte	.LEFDE6-.LASFDE6
.LASFDE6:
	.4byte	.Lframe0
	.4byte	.LFB9
	.4byte	.LFE9-.LFB9
	.byte	0x4
	.4byte	.LCFI12-.LFB9
	.byte	0xd
	.uleb128 0xc
	.byte	0x4
	.4byte	.LCFI13-.LCFI12
	.byte	0x8e
	.uleb128 0x2
	.byte	0x8d
	.uleb128 0x3
	.byte	0x8b
	.uleb128 0x4
	.byte	0x4
	.4byte	.LCFI14-.LCFI13
	.byte	0xc
	.uleb128 0xb
	.uleb128 0x4
	.align	2
.LEFDE6:
.LSFDE8:
	.4byte	.LEFDE8-.LASFDE8
.LASFDE8:
	.4byte	.Lframe0
	.4byte	.LFB11
	.4byte	.LFE11-.LFB11
	.byte	0x4
	.4byte	.LCFI16-.LFB11
	.byte	0xd
	.uleb128 0xc
	.byte	0x4
	.4byte	.LCFI17-.LCFI16
	.byte	0x8e
	.uleb128 0x2
	.byte	0x8d
	.uleb128 0x3
	.byte	0x8b
	.uleb128 0x4
	.byte	0x4
	.4byte	.LCFI18-.LCFI17
	.byte	0xc
	.uleb128 0xb
	.uleb128 0x4
	.align	2
.LEFDE8:
.LSFDE10:
	.4byte	.LEFDE10-.LASFDE10
.LASFDE10:
	.4byte	.Lframe0
	.4byte	.LFB13
	.4byte	.LFE13-.LFB13
	.byte	0x4
	.4byte	.LCFI20-.LFB13
	.byte	0xd
	.uleb128 0xc
	.byte	0x4
	.4byte	.LCFI21-.LCFI20
	.byte	0x8e
	.uleb128 0x2
	.byte	0x8d
	.uleb128 0x3
	.byte	0x8b
	.uleb128 0x4
	.byte	0x4
	.4byte	.LCFI22-.LCFI21
	.byte	0xc
	.uleb128 0xb
	.uleb128 0x4
	.align	2
.LEFDE10:
.LSFDE12:
	.4byte	.LEFDE12-.LASFDE12
.LASFDE12:
	.4byte	.Lframe0
	.4byte	.LFB15
	.4byte	.LFE15-.LFB15
	.byte	0x4
	.4byte	.LCFI24-.LFB15
	.byte	0xd
	.uleb128 0xc
	.byte	0x4
	.4byte	.LCFI25-.LCFI24
	.byte	0x8e
	.uleb128 0x2
	.byte	0x8d
	.uleb128 0x3
	.byte	0x8b
	.uleb128 0x4
	.byte	0x4
	.4byte	.LCFI26-.LCFI25
	.byte	0xc
	.uleb128 0xb
	.uleb128 0x4
	.align	2
.LEFDE12:
	.text
.Letext0:
	.section	.debug_info
	.4byte	0x33f
	.2byte	0x2
	.4byte	.Ldebug_abbrev0
	.byte	0x4
	.uleb128 0x1
	.4byte	.Ldebug_line0
	.4byte	.Letext0
	.4byte	.Ltext0
	.4byte	.LC24
	.4byte	.LC25
	.4byte	.LC26
	.byte	0x1
	.uleb128 0x2
	.4byte	.LC27
	.byte	0x1
	.byte	0x13
	.4byte	0x30
	.uleb128 0x3
	.4byte	.LC7
	.byte	0x4
	.byte	0x7
	.uleb128 0x4
	.4byte	0x5f
	.byte	0x1
	.ascii	"fib\000"
	.byte	0x1
	.byte	0x1c
	.byte	0x1
	.4byte	0x5f
	.4byte	.LFB3
	.4byte	.LFE3
	.byte	0x1
	.byte	0x5b
	.uleb128 0x5
	.ascii	"n\000"
	.byte	0x1
	.byte	0x1b
	.4byte	0x5f
	.byte	0x2
	.byte	0x91
	.sleb128 -20
	.byte	0x0
	.uleb128 0x6
	.ascii	"int\000"
	.byte	0x4
	.byte	0x5
	.uleb128 0x7
	.4byte	0xfc
	.byte	0x1
	.4byte	.LC8
	.byte	0x1
	.byte	0x25
	.byte	0x1
	.4byte	0x30
	.4byte	.LFB5
	.4byte	.LFE5
	.byte	0x1
	.byte	0x5b
	.uleb128 0x5
	.ascii	"dst\000"
	.byte	0x1
	.byte	0x25
	.4byte	0xfc
	.byte	0x2
	.byte	0x91
	.sleb128 -16
	.uleb128 0x5
	.ascii	"num\000"
	.byte	0x1
	.byte	0x25
	.4byte	0x5f
	.byte	0x2
	.byte	0x91
	.sleb128 -20
	.uleb128 0x8
	.ascii	"buf\000"
	.byte	0x1
	.byte	0x26
	.4byte	0x109
	.byte	0x3
	.byte	0x91
	.sleb128 -276
	.uleb128 0x8
	.ascii	"p\000"
	.byte	0x1
	.byte	0x27
	.4byte	0x5f
	.byte	0x3
	.byte	0x91
	.sleb128 -280
	.uleb128 0x8
	.ascii	"i\000"
	.byte	0x1
	.byte	0x27
	.4byte	0x5f
	.byte	0x3
	.byte	0x91
	.sleb128 -284
	.uleb128 0x9
	.4byte	.LC9
	.byte	0x1
	.byte	0x28
	.4byte	0x30
	.byte	0x3
	.byte	0x91
	.sleb128 -288
	.uleb128 0xa
	.4byte	.LBB3
	.4byte	.LBE3
	.uleb128 0x8
	.ascii	"rem\000"
	.byte	0x1
	.byte	0x2f
	.4byte	0x5f
	.byte	0x3
	.byte	0x91
	.sleb128 -292
	.uleb128 0x8
	.ascii	"c\000"
	.byte	0x1
	.byte	0x31
	.4byte	0x102
	.byte	0x3
	.byte	0x91
	.sleb128 -293
	.byte	0x0
	.byte	0x0
	.uleb128 0xb
	.byte	0x4
	.4byte	0x102
	.uleb128 0x3
	.4byte	.LC10
	.byte	0x1
	.byte	0x8
	.uleb128 0xc
	.4byte	0x119
	.4byte	0x102
	.uleb128 0xd
	.4byte	0x119
	.byte	0xff
	.byte	0x0
	.uleb128 0x3
	.4byte	.LC11
	.byte	0x4
	.byte	0x7
	.uleb128 0x7
	.4byte	0x166
	.byte	0x1
	.4byte	.LC12
	.byte	0x1
	.byte	0x3d
	.byte	0x1
	.4byte	0x30
	.4byte	.LFB7
	.4byte	.LFE7
	.byte	0x1
	.byte	0x5b
	.uleb128 0x5
	.ascii	"dst\000"
	.byte	0x1
	.byte	0x3d
	.4byte	0xfc
	.byte	0x2
	.byte	0x91
	.sleb128 -16
	.uleb128 0x5
	.ascii	"src\000"
	.byte	0x1
	.byte	0x3d
	.4byte	0xfc
	.byte	0x2
	.byte	0x91
	.sleb128 -20
	.uleb128 0x9
	.4byte	.LC9
	.byte	0x1
	.byte	0x3e
	.4byte	0x30
	.byte	0x2
	.byte	0x91
	.sleb128 -24
	.byte	0x0
	.uleb128 0xe
	.4byte	0x1bd
	.byte	0x1
	.4byte	.LC13
	.byte	0x1
	.byte	0x46
	.4byte	.LFB9
	.4byte	.LFE9
	.byte	0x1
	.byte	0x5b
	.uleb128 0x8
	.ascii	"buf\000"
	.byte	0x1
	.byte	0x47
	.4byte	0x1bd
	.byte	0x3
	.byte	0x91
	.sleb128 -212
	.uleb128 0x9
	.4byte	.LC14
	.byte	0x1
	.byte	0x48
	.4byte	0xfc
	.byte	0x3
	.byte	0x91
	.sleb128 -216
	.uleb128 0xf
	.4byte	0x1ad
	.byte	0x1
	.4byte	.LC15
	.byte	0x1
	.byte	0x4a
	.4byte	0x5f
	.byte	0x1
	.uleb128 0x10
	.byte	0x0
	.uleb128 0x11
	.byte	0x1
	.4byte	.LC18
	.byte	0x1
	.byte	0x4c
	.4byte	0x5f
	.byte	0x1
	.uleb128 0x10
	.byte	0x0
	.byte	0x0
	.uleb128 0xc
	.4byte	0x1cd
	.4byte	0x102
	.uleb128 0xd
	.4byte	0x119
	.byte	0xc7
	.byte	0x0
	.uleb128 0x12
	.4byte	0x242
	.byte	0x1
	.4byte	.LC16
	.byte	0x1
	.byte	0x50
	.byte	0x1
	.4byte	.LFB11
	.4byte	.LFE11
	.byte	0x1
	.byte	0x5b
	.uleb128 0x13
	.4byte	.LC17
	.byte	0x1
	.byte	0x50
	.4byte	0x5f
	.byte	0x2
	.byte	0x91
	.sleb128 -16
	.uleb128 0x8
	.ascii	"buf\000"
	.byte	0x1
	.byte	0x52
	.4byte	0x1bd
	.byte	0x3
	.byte	0x91
	.sleb128 -216
	.uleb128 0x9
	.4byte	.LC14
	.byte	0x1
	.byte	0x53
	.4byte	0xfc
	.byte	0x3
	.byte	0x91
	.sleb128 -220
	.uleb128 0x8
	.ascii	"res\000"
	.byte	0x1
	.byte	0x54
	.4byte	0x5f
	.byte	0x3
	.byte	0x91
	.sleb128 -224
	.uleb128 0xf
	.4byte	0x232
	.byte	0x1
	.4byte	.LC15
	.byte	0x1
	.byte	0x56
	.4byte	0x5f
	.byte	0x1
	.uleb128 0x10
	.byte	0x0
	.uleb128 0x11
	.byte	0x1
	.4byte	.LC18
	.byte	0x1
	.byte	0x5c
	.4byte	0x5f
	.byte	0x1
	.uleb128 0x10
	.byte	0x0
	.byte	0x0
	.uleb128 0x12
	.4byte	0x2ca
	.byte	0x1
	.4byte	.LC19
	.byte	0x1
	.byte	0x64
	.byte	0x1
	.4byte	.LFB13
	.4byte	.LFE13
	.byte	0x1
	.byte	0x5b
	.uleb128 0x13
	.4byte	.LC20
	.byte	0x1
	.byte	0x64
	.4byte	0x5f
	.byte	0x2
	.byte	0x91
	.sleb128 -16
	.uleb128 0x8
	.ascii	"i\000"
	.byte	0x1
	.byte	0x65
	.4byte	0x5f
	.byte	0x2
	.byte	0x91
	.sleb128 -20
	.uleb128 0x8
	.ascii	"j\000"
	.byte	0x1
	.byte	0x65
	.4byte	0x5f
	.byte	0x2
	.byte	0x91
	.sleb128 -24
	.uleb128 0xa
	.4byte	.LBB8
	.4byte	.LBE8
	.uleb128 0x8
	.ascii	"buf\000"
	.byte	0x1
	.byte	0x69
	.4byte	0x1bd
	.byte	0x3
	.byte	0x91
	.sleb128 -224
	.uleb128 0x9
	.4byte	.LC14
	.byte	0x1
	.byte	0x6a
	.4byte	0xfc
	.byte	0x3
	.byte	0x91
	.sleb128 -228
	.uleb128 0xf
	.4byte	0x2b9
	.byte	0x1
	.4byte	.LC15
	.byte	0x1
	.byte	0x6c
	.4byte	0x5f
	.byte	0x1
	.uleb128 0x10
	.byte	0x0
	.uleb128 0x11
	.byte	0x1
	.4byte	.LC18
	.byte	0x1
	.byte	0x6e
	.4byte	0x5f
	.byte	0x1
	.uleb128 0x10
	.byte	0x0
	.byte	0x0
	.byte	0x0
	.uleb128 0x14
	.byte	0x1
	.4byte	.LC21
	.byte	0x1
	.byte	0x74
	.4byte	0x5f
	.4byte	.LFB15
	.4byte	.LFE15
	.byte	0x1
	.byte	0x5b
	.uleb128 0x9
	.4byte	.LC22
	.byte	0x1
	.byte	0x75
	.4byte	0x25
	.byte	0x2
	.byte	0x91
	.sleb128 -16
	.uleb128 0x8
	.ascii	"buf\000"
	.byte	0x1
	.byte	0x77
	.4byte	0x1bd
	.byte	0x3
	.byte	0x91
	.sleb128 -216
	.uleb128 0x9
	.4byte	.LC14
	.byte	0x1
	.byte	0x78
	.4byte	0xfc
	.byte	0x3
	.byte	0x91
	.sleb128 -220
	.uleb128 0xf
	.4byte	0x31f
	.byte	0x1
	.4byte	.LC15
	.byte	0x1
	.byte	0x7a
	.4byte	0x5f
	.byte	0x1
	.uleb128 0x10
	.byte	0x0
	.uleb128 0xf
	.4byte	0x332
	.byte	0x1
	.4byte	.LC18
	.byte	0x1
	.byte	0x7c
	.4byte	0x5f
	.byte	0x1
	.uleb128 0x10
	.byte	0x0
	.uleb128 0x11
	.byte	0x1
	.4byte	.LC23
	.byte	0x1
	.byte	0x7f
	.4byte	0x5f
	.byte	0x1
	.uleb128 0x10
	.byte	0x0
	.byte	0x0
	.byte	0x0
	.section	.debug_abbrev
	.uleb128 0x1
	.uleb128 0x11
	.byte	0x1
	.uleb128 0x10
	.uleb128 0x6
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x1b
	.uleb128 0xe
	.uleb128 0x25
	.uleb128 0xe
	.uleb128 0x13
	.uleb128 0xb
	.byte	0x0
	.byte	0x0
	.uleb128 0x2
	.uleb128 0x16
	.byte	0x0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.byte	0x0
	.byte	0x0
	.uleb128 0x3
	.uleb128 0x24
	.byte	0x0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0x3e
	.uleb128 0xb
	.byte	0x0
	.byte	0x0
	.uleb128 0x4
	.uleb128 0x2e
	.byte	0x1
	.uleb128 0x1
	.uleb128 0x13
	.uleb128 0x3f
	.uleb128 0xc
	.uleb128 0x3
	.uleb128 0x8
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x27
	.uleb128 0xc
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x40
	.uleb128 0xa
	.byte	0x0
	.byte	0x0
	.uleb128 0x5
	.uleb128 0x5
	.byte	0x0
	.uleb128 0x3
	.uleb128 0x8
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x2
	.uleb128 0xa
	.byte	0x0
	.byte	0x0
	.uleb128 0x6
	.uleb128 0x24
	.byte	0x0
	.uleb128 0x3
	.uleb128 0x8
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0x3e
	.uleb128 0xb
	.byte	0x0
	.byte	0x0
	.uleb128 0x7
	.uleb128 0x2e
	.byte	0x1
	.uleb128 0x1
	.uleb128 0x13
	.uleb128 0x3f
	.uleb128 0xc
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x27
	.uleb128 0xc
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x40
	.uleb128 0xa
	.byte	0x0
	.byte	0x0
	.uleb128 0x8
	.uleb128 0x34
	.byte	0x0
	.uleb128 0x3
	.uleb128 0x8
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x2
	.uleb128 0xa
	.byte	0x0
	.byte	0x0
	.uleb128 0x9
	.uleb128 0x34
	.byte	0x0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x2
	.uleb128 0xa
	.byte	0x0
	.byte	0x0
	.uleb128 0xa
	.uleb128 0xb
	.byte	0x1
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.byte	0x0
	.byte	0x0
	.uleb128 0xb
	.uleb128 0xf
	.byte	0x0
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.byte	0x0
	.byte	0x0
	.uleb128 0xc
	.uleb128 0x1
	.byte	0x1
	.uleb128 0x1
	.uleb128 0x13
	.uleb128 0x49
	.uleb128 0x13
	.byte	0x0
	.byte	0x0
	.uleb128 0xd
	.uleb128 0x21
	.byte	0x0
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x2f
	.uleb128 0xb
	.byte	0x0
	.byte	0x0
	.uleb128 0xe
	.uleb128 0x2e
	.byte	0x1
	.uleb128 0x1
	.uleb128 0x13
	.uleb128 0x3f
	.uleb128 0xc
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x40
	.uleb128 0xa
	.byte	0x0
	.byte	0x0
	.uleb128 0xf
	.uleb128 0x2e
	.byte	0x1
	.uleb128 0x1
	.uleb128 0x13
	.uleb128 0x3f
	.uleb128 0xc
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x3c
	.uleb128 0xc
	.byte	0x0
	.byte	0x0
	.uleb128 0x10
	.uleb128 0x18
	.byte	0x0
	.byte	0x0
	.byte	0x0
	.uleb128 0x11
	.uleb128 0x2e
	.byte	0x1
	.uleb128 0x3f
	.uleb128 0xc
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x3c
	.uleb128 0xc
	.byte	0x0
	.byte	0x0
	.uleb128 0x12
	.uleb128 0x2e
	.byte	0x1
	.uleb128 0x1
	.uleb128 0x13
	.uleb128 0x3f
	.uleb128 0xc
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x27
	.uleb128 0xc
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x40
	.uleb128 0xa
	.byte	0x0
	.byte	0x0
	.uleb128 0x13
	.uleb128 0x5
	.byte	0x0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x2
	.uleb128 0xa
	.byte	0x0
	.byte	0x0
	.uleb128 0x14
	.uleb128 0x2e
	.byte	0x1
	.uleb128 0x3f
	.uleb128 0xc
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x40
	.uleb128 0xa
	.byte	0x0
	.byte	0x0
	.byte	0x0
	.section	.debug_pubnames,"",%progbits
	.4byte	0x70
	.2byte	0x2
	.4byte	.Ldebug_info0
	.4byte	0x343
	.4byte	0x37
	.ascii	"fib\000"
	.4byte	0x66
	.ascii	"int2string\000"
	.4byte	0x120
	.ascii	"str2str\000"
	.4byte	0x166
	.ascii	"adeus\000"
	.4byte	0x1cd
	.ascii	"codigo_processo_1\000"
	.4byte	0x242
	.ascii	"codigo_processo_2\000"
	.4byte	0x2ca
	.ascii	"main\000"
	.4byte	0x0
	.section	.debug_aranges,"",%progbits
	.4byte	0x1c
	.2byte	0x2
	.4byte	.Ldebug_info0
	.byte	0x4
	.byte	0x0
	.2byte	0x0
	.2byte	0x0
	.4byte	.Ltext0
	.4byte	.Letext0-.Ltext0
	.4byte	0x0
	.4byte	0x0
	.section	.debug_str,"MS",%progbits,1
.LC17:
	.ascii	"param_fib\000"
.LC13:
	.ascii	"adeus\000"
.LC12:
	.ascii	"str2str\000"
.LC26:
	.ascii	"GNU C 3.3\000"
.LC24:
	.ascii	"user.c\000"
.LC16:
	.ascii	"codigo_processo_1\000"
.LC19:
	.ascii	"codigo_processo_2\000"
.LC27:
	.ascii	"pid_t\000"
.LC11:
	.ascii	"long unsigned int\000"
.LC23:
	.ascii	"fork\000"
.LC20:
	.ascii	"param_for\000"
.LC21:
	.ascii	"main\000"
.LC10:
	.ascii	"char\000"
.LC8:
	.ascii	"int2string\000"
.LC22:
	.ascii	"num_proc\000"
.LC7:
	.ascii	"unsigned int\000"
.LC15:
	.ascii	"getpid\000"
.LC14:
	.ascii	"pbuf\000"
.LC18:
	.ascii	"write\000"
.LC25:
	.ascii	"/l/home/rafael/p/mc404/sw/trab2\000"
.LC9:
	.ascii	"num_bytes\000"
	.ident	"GCC: (GNU) 3.3"

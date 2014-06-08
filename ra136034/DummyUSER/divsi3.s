	.text
	.align	2
	.globl __divsi3
__divsi3:
	eor	ip, r0, r1
	mov	r3, #1
	mov	r2, #0
	cmp	r1, #0
	rsbmi	r1, r1, #0
	beq	Ldiv0
	cmp	r0, #0
	rsbmi	r0, r0, #0
	cmp	r0, r1
	bcc	Lgot_result
Loop1:
	cmp	r1, #268435456	@ 0x10000000
	cmpcc	r1, r0
	lslcc	r1, r1, #4
	lslcc	r3, r3, #4
	bcc	Loop1
Lbignum:
	cmp	r1, #-2147483648	@ 0x80000000
	cmpcc	r1, r0
	lslcc	r1, r1, #1
	lslcc	r3, r3, #1
	bcc	Lbignum
Loop3:
	cmp	r0, r1
	subcs	r0, r0, r1
	orrcs	r2, r2, r3
	cmp	r0, r1, lsr #1
	subcs	r0, r0, r1, lsr #1
	orrcs	r2, r2, r3, lsr #1
	cmp	r0, r1, lsr #2
	subcs	r0, r0, r1, lsr #2
	orrcs	r2, r2, r3, lsr #2
	cmp	r0, r1, lsr #3
	subcs	r0, r0, r1, lsr #3
	orrcs	r2, r2, r3, lsr #3
	cmp	r0, #0
	lsrnes	r3, r3, #4
	lsrne	r1, r1, #4
	bne	Loop3
Lgot_result:
	mov	r0, r2
	cmp	ip, #0
	rsbmi	r0, r0, #0
	mov	pc, lr
Ldiv0:
	push	{lr}		@ (str lr, [sp, #-4]!)
	bl	__div0
	mov	r0, #0
	ldm	sp!, {pc}


__div0:
	mov	pc, lr

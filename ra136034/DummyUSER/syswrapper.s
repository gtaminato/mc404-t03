@ ============================================================================ @
@   syswrapper.s
@
@   Contém a definição de funções com nomes de syscalls.
@   A implementação das funções apenas realiza a respectiva syscall e retorna o
@   resultado.
@   Esses wrappers são necessários para fazer código em C chamar syscalls.
@   Normalmente, tais wrapper são implementados pela libc. Em nosso caso,
@   não utilizaremos a libc para não complicar o projeto do sistema operacional.
@   O motivo é que a libc, para funcionar, precisa de diversas outras syscalls
@   implementadas. Essa é a razão do código de usuário nunca utilizar
@   funções como printf() (pois são funções definidas pela libc).
@ ---------------------------------------------------------------------------- @

@ ---------------------------------------------------------------------------- @
@   Wrapper C para a syscall getpid
@ ---------------------------------------------------------------------------- @
	.text
	.align	2
	.globl getpid
getpid:
	stmfd 	sp!, {r7}
	mov	r7, #20
	svc	0
	ldmfd	sp!, {r7}
	mov	pc, lr

@ ---------------------------------------------------------------------------- @
@   Wrapper C para a syscall exit
@ ---------------------------------------------------------------------------- @
	.globl _exit
	.globl exit
_exit:
exit:
	stmfd 	sp!, {r7}
	mov	r7, #1
	svc	0
	ldmfd	sp!, {r7}
	mov	pc, lr	@ should not execute

@ ---------------------------------------------------------------------------- @
@   Wrapper C para a syscall fork
@ ---------------------------------------------------------------------------- @
	.text
	.align	2
	.globl fork
fork:
	stmfd 	sp!, {r7}
	mov	r7, #2
	svc	0
	ldmfd	sp!, {r7}
	mov	pc, lr

@ ---------------------------------------------------------------------------- @
@   Wrapper C para a syscall write
@ ---------------------------------------------------------------------------- @
	.text
	.align	2
	.globl write
write:
	stmfd 	sp!, {r7}
	mov	r7, #4
	svc	0
	ldmfd	sp!, {r7}
	mov	pc, lr

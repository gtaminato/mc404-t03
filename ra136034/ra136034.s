.org 0x0
.section .iv,"a"

_start:

interrupt_vector:
    b RESET_HANDLER
.org 0x08
    b SUPERVISOR_HANDLER
.org 0x100
.text

RESET_HANDLER:
    @ Load interrupt_vector address in coprocessor 15
    ldr r0, =interrupt_vector
    mcr p15, 0, r0, c12, c0, 0

    @ Set CPSR to Supervisor mode
    msr CPSR_c, #0xD3 

    @Set UART - according to uart.pdf
    .set UART_BASE, 0x53FBC000
    .set UART_UCR1, 0x80
    .set UART_UCR2, 0x84
    .set UART_UCR3, 0x88
    .set UART_UCR4, 0x8C
    .set UART_UFCR, 0x90
    .set UART_UBIR, 0xA4
    .set UART_UBMR, 0xA8
    .set UART_USR1, 0x94
    
    @1-Enable UART
    ldr r1, =UART_BASE
    mov r0, #1
    str r0, [r1, #UART_UCR1]
    @2-Set hardware flow control, data format and enable trans and receiver
    ldr r0, =0x2127
    str r0, [r1, #UART_UCR2]
    @3-Set UCR3[RXDMUXSEL] = 1
    ldr r0, =0x0704
    str r0, [r1, #UART_UCR3]
    @4-Set CTS trigger level to 31
    ldr r0, =0x7C00
    str r0, [r1, #UART_UCR4]
    @5-Set internal clock divider = 5 (divide input uart clock by 5).
    ldr r0, =0x089E
    str r0, [r1, #UART_UFCR]
    @6 and 7-Set baud rate to 921.6Kbps based on the 20MHz reference clock.
    ldr r0, =0x08FF
    str r0, [r1, #UART_UBIR]
    ldr r0, =0x0C34
    str r0, [r1, #UART_UBMR]
    
    .set USR_STACK, 0x11000
        .set SVC_STACK, 0x10800
        .set UND_STACK, 0x07c00
        .set ABT_STACK, 0x07800
        .set FIQ_STACK, 0x07400
        .set IRQ_STACK, 0x07000

    @Configure stacks for all modes
    mov sp, #SVC_STACK
    msr CPSR_c, #0xDF  @ Enter system mode, FIQ/IRQ disabled
    mov sp, #USR_STACK
    msr CPSR_c, #0xD1  @ Enter FIQ mode, FIQ/IRQ disabled
    mov sp, #FIQ_STACK
    msr CPSR_c, #0xD2  @ Enter IRQ mode, FIQ/IRQ disabled
    mov sp, #IRQ_STACK
    msr CPSR_c, #0xD7  @ Enter abort mode, FIQ/IRQ disabled
    mov sp, #ABT_STACK
    msr CPSR_c, #0xDB  @ Enter undefined mode, FIQ/IRQ disabled
    mov sp, #UND_STACK

    @ Initialize processes
    ldr r0, =array_process
    mov r1, #1
    strb r1, [r0], #1       @ First one is enabled
    mov r1, #0
    strb r1, [r0], #1       @ The others are disabled
    strb r1, [r0], #1
    strb r1, [r0], #1
    strb r1, [r0], #1
    strb r1, [r0], #1
    strb r1, [r0], #1
    strb r1, [r0]

    @ Initialize current_process
    ldr r0, =current_process
    str r1, [r0]

    @ Back to User mode
    msr CPSR_c, #0x10
    mov pc, #0x8000         @ Jump to user code


SUPERVISOR_HANDLER:
    @ Set CPSR to Supervisor mode
    msr CPSR_c, #0xD3
    
    @ Check which Syscall command we must execute
    cmp r7, #0x4     @ write()
    beq write
    cmp r7, #0x2     @ fork()
    beq fork
    cmp r7, #0x14    @ getpid()
    beq getpid
    cmp r7, #0x1     @ exit()
    beq exit

    @ Make a Syscall Write 
    write:
        push {r4-r6}
        ldr r3, =0x53FBC094
        ldr r5, =0x53FBC040
        @make a mask in r0 to verify bit 13 of UART_USR1 is set as 1
        mov r0, #1
        mov r0, #(1<<13)
        write_loop:
            cmp r2, #0
            beq write_loop_end          @finish if all chars were transmitted
        @Wait for the transmission queue be ready
        transmitterLoop:
            ldr r4, [r3]
            and r4, r4, r0
            cmp r4, #0
            beq transmitterLoop         @try again if queue is full
        ldrb r6, [r1], #1
        strb r6, [r5]
        sub r2, r2, #1                  @decrement numbers of chars to be transmitted
        b write_loop
        write_loop_end:
    doneWriting:
        pop {r4-r6}
        b SUPERVISOR_HANDLER_EXIT

    @ Make a Syscall Fork
    fork:
        push {r1-r3}
        
        @ Try to find an available PID
        ldr r1, =array_process    @ Load address of array_process
        mov r0, #0               @ Initialize counter with 0
        fork_find_process:
            cmp r0, #8
            beq noProcessAvailable  @ None of 8 process is available
            ldrb r2, [r1, r0]       @ Load in r2 the current process
            cmp r2, #0
            addne r0, r0, #1
            bne fork_find_process   @ Check the next process
            
        @ An available PID has been found, so we must enable it
        mov r2, #1
        strb r2, [r1, r0]
        
        @ Store return address
        ldr r1, =returnArray
        str r14, [r1, r0, lsl #2]
        
        @ Load address of contexts array
        ldr r1, =p1context
        
        @ Move to right process context
        add r1, r1, r0, lsl #6
        
        @ Store CPSR
        mrs r2, SPSR
        str r2, [r1], #4
        
        @ Store Registers r0-r3
        mov r2, #0
        str r2, [r1], #4
        pop {r2}
        str r2, [r1], #4
        pop {r2}
        str r2, [r1], #4
        pop {r2}
        str r2, [r1], #4
        
        @ Store Registers r4-r12
        mov r2, r4
        str r2, [r1], #4
        mov r2, r5
        str r2, [r1], #4
        mov r2, r6
        str r2, [r1], #4
        mov r2, r7
        str r2, [r1], #4
        mov r2, r8
        str r2, [r1], #4
        mov r2, r9
        str r2, [r1], #4
        mov r2, r10
        str r2, [r1], #4
        mov r2, r11
        str r2, [r1], #4
        mov r2, r12
        str r2, [r1], #4
        
        @ Store r13
        push {r4-r8}
        ldr r2, =current_process
        ldr r2, [r2]
        
        @ Point r3 to stack of children process
        ldr r3, =0x11000
        sub r3, r3, r0, lsl #12
        
        @ Poiont r4 to stack of parent process
        ldr r4, =0x11000
        sub r4, r4, r2, lsl #12
        
        @ Go to System Mode to recover r13 and r14
        msr CPSR_c, #0xDF   
        mov r5, r13
        mov r6, r14
        
        @ Back to Supervidor Mode
        msr CPSR_c, #0xD3
        
        @ Loop to copy over stack
        CopyStack:
            cmp r4, r5
            blt doneCopyingStack
            ldr r7, [r4], #-4
            str r7, [r3], #-4
            b CopyStack
        doneCopyingStack:
            @-Adjust stack pointer
            add r3, r3, #4
            @-Save r13 and r14 on context array
            str r3, [r1], #4
            str r6, [r1]
            pop {r4-r8}

        add r0, r0, #1          @ Index starts on 0, so we must increment 1
        b SUPERVISOR_HANDLER_EXIT  

        noProcessAvailable:
            mov r0, #-1
            b SUPERVISOR_HANDLER_EXIT    

    @ Make a Syscall getpid
    getpid:
        ldr r0, =current_process     @ Load address of current process
        ldr r0, [r0]                @ Load value
        add r0, r0, #1              @ PID starts on 0
        b SUPERVISOR_HANDLER_EXIT
        
    @ Make a Syscall exit
    exit:      
        ldr r1, =array_process
        ldr r0, =current_process
        ldr r0, [r0]                @ Load value of current PID
        ldrb r1, [r1, r0]           @ Load in r1 the current process address
        mov r0, #0
        strb r0, [r1]               @ Set current PID as inactive
        b mainScheduler

    SUPERVISOR_HANDLER_EXIT:
        movs pc, lr

mainScheduler:
    ldr r0, =current_process
    ldr r1, [r0]
    ldr r0, =array_process
    mov r2, #8
      traverseArray:
        cmp r2, #0
        beq endTraversal
        cmp r1, #7
        moveq r1, #0
        addne r1, r1, #1
        ldrb r3, [r0, r1]
        cmp r3, #1
        @-if equal go to this process, changing current_process first
        beq changeProcess
        sub r2, r2, #1
        b traverseArray
    endTraversal:
        @--No more user processes to run, wait for interruption
        infiniteLoop:
            b infiniteLoop
    @--Change process and return execution to it
    changeProcess:
    ldr r0, =current_process
    str r1, [r0]
    @-Set return address on r14
    ldr r0, =returnArray
    ldr r2, [r0, r1, lsl #2]
    mov r14, r2
    @-Restore registers r14 and r13
    ldr r0, =p1context
    add r0, r0, r1, lsl #6
    add r0, r0, #60
    ldr r2, [r0], #-4
    ldr r3, [r0], #-4
    @-Change to System Mode
    msr CPSR_c, #0xDF
    mov r14, r2
    mov r13, r3
    @-Back to Supervisor
    msr CPSR_c, #0xD3
    @-Restore registers r12-r4
    ldr r2, [r0], #-4
    mov r12, r2
    ldr r2, [r0], #-4
        mov r11, r2
    ldr r2, [r0], #-4
        mov r10, r2
    ldr r2, [r0], #-4
        mov r9, r2
    ldr r2, [r0], #-4
        mov r8, r2
    ldr r2, [r0], #-4
        mov r7, r2
    ldr r2, [r0], #-4
        mov r6, r2
    ldr r2, [r0], #-4
        mov r5, r2
    ldr r2, [r0], #-4
        mov r4, r2
    @-Restore SPSR
    ldr r2, =p1context
    add r2, r2, r1, lsl #6
    ldr r3, [r2]
    msr SPSR, r3
    @-Restore registers r3-r0
    ldr r1, [r0], #-4
    mov r3, r1
    ldr r1, [r0], #-4
    mov r2, r1
    ldr r1, [r0], #-4
    ldr r0, [r0]
    @-Return execution to this process
    movs pc, lr

.ltorg

@--Interruption mode stacks
.org 0x6C00
irqStack: .space 1024
fiqStack: .space 1024
abtStack: .space 1024
undStack: .space 1024

@--User software goes in this memory range

@--User and supervisor mode stacks
.org 0x9000
p8supervisor: .space 2048
p8user: .space 2048
p7supervisor: .space 2048
p7user: .space 2048
p6supervisor: .space 2048
p6user: .space 2048
p5supervisor: .space 2048
p5user: .space 2048
p4supervisor: .space 2048
p4user: .space 2048
p3supervisor: .space 2048
p3user: .space 2048
p2supervisor: .space 2048
p2user: .space 2048
p1supervisor: .space 2048
p1user: .space 2048

@--Array to hold saved contexts
.org 0x12000
p1context: .space 512

@--Array to hold return addresses
.org 0x13000
returnArray: .space 32

@--CurrentProcess variable and array to store list of active ones
current_process: .space 4
array_process: .space 8

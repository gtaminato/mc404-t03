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
    SET_UART:
        .set UART_BASE, 0x53FBC000
        .set UART_UCR1, 0x80
        .set UART_UCR2, 0x84
        .set UART_UCR3, 0x88
        .set UART_UCR4, 0x8C
        .set UART_UFCR, 0x90
        .set UART_UBIR, 0xA4
        .set UART_UBMR, 0xA8
        .set UART_USR1, 0x94
        .set UART_UTXD, 0x40
        
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
    
    @Set GPT as made in lab07
    SET_GPT:
        .set GPT_BASE,                 0x53FA0000
        .set GPT_CR,                   0x0
        .set GPT_PR,                   0x4
        .set GPT_IR,                   0xC
        .set GPT_OCR1,                 0x10
        
        @start setting GPT stuff
        mov r0, #0x00000041
        ldr r1, =GPT_BASE
        str r0, [r1, #GPT_CR]            @enable GTP control register and set clock_src peripheral
        
        mov r0, #0
        ldr r1, =GPT_BASE
        str r0, [r1, #GPT_PR]            @set prescaler to zero
        
        mov r0, #100
        ldr r1, =GPT_BASE
        str r0, [r1, #GPT_OCR1]          @count up to 100
        
        mov r0, #1
        ldr r1, =GPT_BASE
        str r0, [r1, #GPT_IR]            @show interest in Output Compare Channel 1
        
        msr  CPSR_c,  #0x13   @ SUPERVISOR mode, IRQ/FIQ enabled
    
        .set SVC_STACK, 0x77701000
        .set UND_STACK, 0x77702000
        .set ABT_STACK, 0x77703000
        .set IRQ_STACK, 0x77704000
        .set FIQ_STACK, 0x77705000
        .set USR_STACK, 0x77706000

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
    msr CPSR_c, #0x10           @ Sets CPSR_c bits adequated to user mode
    mov pc, #0x77802000         @ Jump to DUMMYUSER


SUPERVISOR_HANDLER:
    @ Set CPSR to Supervisor mode
    msr CPSR_c, #0xD3
    
    @ Determine which Syscall command we must execute
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
        push {r4, r5}
        ldr r3, =UART_BASE
        @make a mask in r0 to verify bit 13 of UART_USR1 is set as 1
        mov r0, #(1<<13)
        write_loop:
            cmp r2, #0
            beq write_loop_end          @finish if all chars were transmitted
            @Wait for the transmission queue be ready
            check_queue:
                ldr r4, [r3, #UART_USR1]
                and r4, r4, r0
                cmp r4, #0
                beq check_queue         @try again if queue is full
        ldrb r5, [r1], #1               @take a char using char pointer in r1       
        strb r5, [r3, #UART_UTXD]       @sends it to TX_FIFO through UART_UTXD storing
        sub r2, r2, #1                  @decrement numbers of chars to be transmitted
        b write_loop
        write_loop_end:
        
        pop {r4, r5}
        b SUPERVISOR_HANDLER_EXIT

    @ Make a Syscall Fork
    fork:
        push {r1-r3}
        
        @ Try to find an available PID
        ldr r1, =array_process          @ Load address of array_process
        mov r0, #0                      @ Initialize counter with 0
        fork_find_process:
            cmp r0, #8
            beq fork_none_PID_avaiable  @ None of 8 process is available
            ldrb r2, [r1, r0]           @ Load in r2 the current process
            cmp r2, #0
            addne r0, r0, #1
            bne fork_find_process       @ Check the next process
            
        @ An available PID has been found, so we must enable it
        mov r2, #1
        strb r2, [r1, r0]
        
        @ Store return address
        ldr r1, =return_array
        str r14, [r1, r0, lsl #2]
        
        @ Load address of contexts array
        ldr r1, =contexts
        
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
        
        @ Point r4 to stack of parent process
        ldr r4, =0x11000
        sub r4, r4, r2, lsl #12
        
        @ Go to System Mode to recover r13 and r14
        msr CPSR_c, #0xDF   
        mov r5, r13
        mov r6, r14
        
        @ Back to Supervidor Mode
        msr CPSR_c, #0xD3
        
        @ Loop to copy over stack
        fork_copy_stack:
            cmp r4, r5
            blt doneCopyingStack
            ldr r7, [r4], #-4
            str r7, [r3], #-4
            b CopyStack
        fork_copy_stack_end:
            @ Adjust stack pointer
            add r3, r3, #4
            @ Save r13 and r14 on context array
            str r3, [r1], #4
            str r6, [r1]
            pop {r4-r8}

        add r0, r0, #1               @ Index starts on 0, so we must increment 1
        b SUPERVISOR_HANDLER_EXIT  

        fork_none_PID_avaiable:
            mov r0, #-1
            b SUPERVISOR_HANDLER_EXIT    

    @ Make a Syscall getpid
    getpid:
        ldr r0, =current_process     @ Load address of current process
        ldr r0, [r0]                 @ Load value
        add r0, r0, #1               @ PID starts on 0
        b SUPERVISOR_HANDLER_EXIT
        
    @ Make a Syscall exit
    exit:      
        ldr r1, =array_process
        ldr r0, =current_process
        ldr r0, [r0]                @ Load value of current PID
        ldrb r1, [r1, r0]           @ Load in r1 the current process address
        mov r0, #0
        strb r0, [r1]               @ Set current PID as inactive
        b main

    SUPERVISOR_HANDLER_EXIT:
        movs pc, lr

main:
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
        @ if equal go to this process, changing current_process first
        beq changeProcess
        sub r2, r2, #1
        b traverseArray
    endTraversal:
        @ No more user processes to run, wait for interruption
        infiniteLoop:
            b infiniteLoop
            
    @ Change process and return execution to it
    changeProcess:
        ldr r0, =current_process
        str r1, [r0]
        @ Set return address on r14
        ldr r0, =return_array
        ldr r2, [r0, r1, lsl #2]
        mov r14, r2
        @ Restore registers r14 and r13
        ldr r0, =contexts
        add r0, r0, r1, lsl #6
        add r0, r0, #60
        ldr r2, [r0], #-4
        ldr r3, [r0], #-4
        @ Change to System Mode
        msr CPSR_c, #0xDF
        mov r14, r2
        mov r13, r3
        @ Back to Supervisor
        msr CPSR_c, #0xD3
        @ Restore registers r12-r4
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
        @ Restore SPSR
        ldr r2, =contexts
        add r2, r2, r1, lsl #6
        ldr r3, [r2]
        msr SPSR, r3
        @ Restore registers r3-r0
        ldr r1, [r0], #-4
        mov r3, r1
        ldr r1, [r0], #-4
        mov r2, r1
        ldr r1, [r0], #-4
        ldr r0, [r0]
        @ Return execution to this process
        movs pc, lr

.ltorg

@ Interruption mode stacks
.org 0x77701000

SVC_STACK_SPACE: .space MODE_STACK_SZ
UND_STACK_SPACE: .space MODE_STACK_SZ
ABT_STACK_SPACE: .space MODE_STACK_SZ
IRQ_STACK_SPACE: .space MODE_STACK_SZ
FIQ_STACK_SPACE: .space MODE_STACK_SZ

@ User software goes in this memory range
.set PROCESS_STACK_SZ, 0x800  @ Size of each process to record all registers + CPSR
.set MODE_STACK_SZ, 0x3E8     @ Stack size of each user mode


@ User and supervisor mode stacks
.org 0x77705800

PID8_sup: .space PROCESS_STACK_SZ
PID8: .space PROCESS_STACK_SZ
PID7_sup: .space PROCESS_STACK_SZ
PID7: .space PROCESS_STACK_SZ
PID6_sup: .space PROCESS_STACK_SZ
PID6: .space PROCESS_STACK_SZ
PID5_sup: .space PROCESS_STACK_SZ
PID5: .space PROCESS_STACK_SZ
PID4_sup: .space PROCESS_STACK_SZ
PID4: .space PROCESS_STACK_SZ
PID3_sup: .space PROCESS_STACK_SZ
PID3: .space PROCESS_STACK_SZ
PID2_sup: .space PROCESS_STACK_SZ
PID2: .space PROCESS_STACK_SZ
PID1_sup: .space PROCESS_STACK_SZ
PID1: .space PROCESS_STACK_SZ

@ Array to hold saved contexts
.org 0x12000
contexts: .space 512


.org 0x13000
return_array:       .space 32
current_process:    .space 4
array_process:      .space 8

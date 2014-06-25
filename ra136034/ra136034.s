.org 0x0
.section .iv,"a"

_start:

interrupt_vector:
    b RESET_HANDLER
.org 0x08
    b SUPERVISOR_HANDLER
.org 0x18
    b SCHEDULE_HANDLE
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
        
    @Set GPT as made in lab07
    SET_TZIC:
        @ Constantes para os enderecos do TZIC
        .set TZIC_BASE,                0x0FFFC000
        .set TZIC_INTCTRL,             0x0
        .set TZIC_INTSEC1,             0x84 
        .set TZIC_ENSET1,              0x104
        .set TZIC_PRIOMASK,            0xC
        .set TZIC_PRIORITY9,           0x424

        @ Liga o controlador de interrupcoes
        @ R1 <= TZIC_BASE

        ldr    r1, =TZIC_BASE

        @ Configura interrupcao 39 do GPT como nao segura
        mov    r0, #(1 << 7)
        str    r0, [r1, #TZIC_INTSEC1]

        @ Habilita interrupcao 39 (GPT)
        @ reg1 bit 7 (gpt)

        mov    r0, #(1 << 7)
        str    r0, [r1, #TZIC_ENSET1]

        @ Configure interrupt39 priority as 1
        @ reg9, byte 3

        ldr r0, [r1, #TZIC_PRIORITY9]
        bic r0, r0, #0xFF000000
        mov r2, #1
        orr r0, r0, r2, lsl #24
        str r0, [r1, #TZIC_PRIORITY9]

        @ Configure PRIOMASK as 0
        eor r0, r0, r0
        str r0, [r1, #TZIC_PRIOMASK]

        @ Habilita o controlador de interrupcoes
        mov    r0, #1
        str    r0, [r1, #TZIC_INTCTRL]
              
    SET_STACKS:
        .set SVC_STACK, 0x77701000
        .set UND_STACK, 0x77702000
        .set ABT_STACK, 0x77703000
        .set IRQ_STACK, 0x77704000
        .set FIQ_STACK, 0x77705000
        .set USR_STACK, 0x7770D000
        

        @Configure stacks for all modes
        ldr sp, =SVC_STACK
        msr CPSR_c, #0xDF  @ Enter system mode, FIQ/IRQ disabled
        ldr sp, =USR_STACK
        msr CPSR_c, #0xD1  @ Enter FIQ mode, FIQ/IRQ disabled
        ldr sp, =FIQ_STACK
        msr CPSR_c, #0xD2  @ Enter IRQ mode, FIQ/IRQ disabled
        ldr sp, =IRQ_STACK
        msr CPSR_c, #0xD7  @ Enter abort mode, FIQ/IRQ disabled
        ldr sp, =ABT_STACK
        msr CPSR_c, #0xDB  @ Enter undefined mode, FIQ/IRQ disabled
        ldr sp, =UND_STACK

    @ Initialize processes
    ldr r0, =process_states
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
    
    .set ONE_CONTEXT_SZ, 68
    .set PC_DISPLACEMENT, 60

    @ Initialize current_process
    ldr r0, =current_process
    str r1, [r0]

    @ Back to User mode
    msr CPSR_c, #0x10           @ Sets CPSR_c bits adequated to user mode
    ldr pc, =0x77802000         @ Jump to DUMMYUSER
    
.ltorg


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
        push {r1-r3}                    @push them to make some calcs before store them in context vector
        
        @ Try to find an available PID
        ldr r1, =process_states          @load address of process_states
        mov r0, #0                      @ Initialize counter with 0
        check_available_PID:
            cmp r0, #8
            beq no_more_PID             @didn't find any available process
            ldrb r2, [r1, r0]           @loads in r2 the current process
            cmp r2, #0                  @check if process is available
            beq check_available_PID_end @if available stop searching
            add r0, r0, #1
            b check_available_PID       @ Check the next process
        check_available_PID_end:
            
        @mark found PID as in use
        mov r2, #1
        strb r2, [r1, r0]
        
        @store the process lr in 
        ldr r1, =process_pcs
        mov r3, #4
        mul r2, r0, r3
        str lr, [r1, r2]
        
        @load context
        ldr r1, =contexts
        
        @Calculates displacement
        mov r3, #68
        mul r2, r3              @17 register in each context, each register having 4 bytes = 68
        add r1, r2              @now r1 points to the right context
        

        
        @store registers in the context
        mov r2, #0              @prepares 0 to put in r0, that means a success return in child process
        str r2, [r1], #4        @stores r0

        @stores r1-r3
        mov r3, #3              @initializes counter
        store_register_loop_1:
            cmp r3, #0
            beq store_register_loop_1_end
            
            pop {r2}                @pops register from stack and prepares to store in register
            str r2, [r1], #4        @store register and points to next
            sub r3, r3, #1
            
            b store_register_loop_1
        store_register_loop_1_end:
        
        @store r4-r12, they are already in the registers, becase they are not caller save
        mov r2, r4              @prepares r4  to be saved
        str r2, [r1], #4        @save r4
        mov r2, r5              @prepares r5  to be saved
        str r2, [r1], #4        @save r5
        mov r2, r6              @prepares r6  to be saved       
        str r2, [r1], #4        @save r6
        mov r2, r7              @prepares r7  to be saved
        str r2, [r1], #4        @save r7
        mov r2, r8              @prepares r8  to be saved
        str r2, [r1], #4        @save r8
        mov r2, r9              @prepares r9  to be saved
        str r2, [r1], #4        @save r9
        mov r2, r10             @prepares r10 to be saved
        str r2, [r1], #4        @save r10
        mov r2, r11             @prepares r11 to be saved
        str r2, [r1], #4        @save r11
        mov r2, r12             @prepares r12 to be saved
        str r2, [r1], #4        @save r12
        
        @store r13
        push {r4-r8}
        
        @Takes stack of child process
        ldr r3, =USR_STACK              @points to stack array
        ldr r4, =PROCESS_STACK_SZ       @loads the stack size in r4
        mov r5, r0                      @mov process PID to r5
        mul r5, r4                      @calculates displacement
        sub r3, r3, r5                  @points r3 to right stack
        
        @Takes stack of parent process
        ldr r2, =current                @r2 points to current process label
        ldr r2, [r2]                    @r2 contains number of current process
        ldr r4, =USR_STACK              @points to stack array
        ldr r5, =PROCESS_STACK_SZ       @loads the stack size in r4
        mov r6, r2                      @mov process PID to r6
        mul r6, r5                      @calculates displacement
        sub r4, r4, r6                  @points r3 to right stack

        
        @needs to change to user/system mode to recover r13 and r14
        @they are not visible in supervisor mode
        msr CPSR_c, #0xDF   
        mov r5, sp      @(r13)
        mov r6, lr      @(r14)
        
        @back to supervisor mode
        msr CPSR_c, #0xD3
        
        @ Loop to copy over stack
        fork_copy_stack:
            cmp r4, r5                  @when r4 points to 
            blt fork_copy_stack_end     @stops
            ldr r7, [r4], #-4           @loads value from parent stack
            str r7, [r3], #-4           @stores value in child stack
            b fork_copy_stack
        fork_copy_stack_end:
        @correct sp
        add r3, r3, #4
        @sa r13 and r14 on context array
        str r3, [r1], #4            @save r13 (sp)
        str r6, [r1], #4            @save r14 (lr)
            
        @store the process lr in children PC 
        str lr, [r1], #4            @save PC (r15)
        
        @store CPSR
        mrs r2, SPSR
        str r2, [r1]
      
        pop {r4-r8}

        add r0, r0, #1               @return child process PID to parent process, must add 1 to correct vector index
        b SUPERVISOR_HANDLER_EXIT  

        no_more_PID:
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
        ldr r1, =process_states
        ldr r0, =current_process
        ldr r0, [r0]                @ Load value of current PID
        ldrb r1, [r1, r0]           @ Load in r1 the current process address
        mov r0, #0
        strb r0, [r1]               @ Set current PID as inactive
        b schedule_process

    SUPERVISOR_HANDLER_EXIT:
        movs pc, lr
        
.ltorg
        
SCHEDULE_HANDLE:
    b save_context

@ Save current context in array of contexts
save_context:
    push {r0, r1, r2, lr}

    ldr r0, =current_process 
    ldr r0, [r0]                @ Load process number being executed
    ldr r1, =ONE_CONTEXT_SZ
    mul r2, r0, r1              @ PID*68
    ldr r1, =contexts
    add r0, r1, r2
    
    @ Save Registers from r0 to r12
    pop {r1}
    str r1, [r0], #4            @ Save r0
    pop {r1}
    str r1, [r0], #4            @ Save r1
    pop {r1}
    str r1, [r0], #4            @ Save r2
    str r3, [r0], #4            @ Save r3
    str r4, [r0], #4            @ Save r4
    str r5, [r0], #4            @ Save r5
    str r6, [r0], #4            @ Save r6
    str r7, [r0], #4            @ Save r7
    str r8, [r0], #4            @ Save r8
    str r9, [r0], #4            @ Save r9
    str r10, [r0], #4           @ Save r10
    str r11, [r0], #4           @ Save r11
    str r12, [r0], #4           @ Save r12
    
    @ Now we need to switch to System mode
    @ to load r13 and r14
    msr CPSR_c, #0xDF
    mov r1, r13
    mov r2, r14
    
    @ Switch back to IRQ Mode
    @ to save r13 and r14
    msr CPSR_c, #0x92
    str r1, [r0], #4
    str r2, [r0], #4
    
    @ Save PC
    pop {lr}
    sub lr, lr, #4
    str lr, [r0], #4
    
    @ Save CPSR
    mrs r1, SPSR
    str r1, [r0]
  
    @ Return to Supervisor
    msr CPSR_c, #0xD3
        
schedule_process:
    ldr r0, =current_process    
    ldr r1, [r0]                @loads currrent process number in r1
    ldr r0, =process_states     @loads process_states in r0
    mov r2, #8                  @initializes counter
    schedule_process_loop:
        cmp r2, #0                      @if no process where found
        beq schedule_process_loop_end   @loops for ever
        cmp r1, #7                      @if current is the last process
        moveq r1, #0                    @back to first (round-robin)
        addne r1, r1, #1                @if not goes to next process
        ldrb r3, [r0, r1]               @loads next process state in r3
        cmp r3, #1                      @if process is live        
        beq set_next_process            @jump to next process
        sub r2, r2, #1                  @decrements counter
        b schedule_process_loop         @loop
    schedule_process_loop_end:
        b schedule_process_loop_end     @loops for ever until, interruption from GPT
            
    @ Change process and return execution to it
    set_next_process:
        ldr r0, =current_process
        str r1, [r0]                    @refresh current_process to contains the next process
        
        ldr r0, =ONE_CONTEXT_SZ
        mul r2, r0, r1                  @PID*68
        ldr r1, =contexts
        add r0, r1, r2                  @loads in r0 the new context address
        
        @changes to System Mode
        msr CPSR_c, #0xDF
        
        @loads all registers from memory
        ldr r1, [r0], #4                  @load R0 from memory
        push {r1}                         @store r0 in stack
        ldr r1, [r0], #4                  @load R1 from memory
        push {r1}                         @store r1 in stack
        ldr r2, [r0], #4                  @load R2 from memory
        ldr r3, [r0], #4                  @load R3 from memory
        ldr r4, [r0], #4                  @load R4 from memory
        ldr r5, [r0], #4                  @load R5 from memory
        ldr r6, [r0], #4                  @load R6 from memory
        ldr r7, [r0], #4                  @load R7 from memory
        ldr r8, [r0], #4                  @load R8 from memory
        ldr r9, [r0], #4                  @load R9 from memory
        ldr r10, [r0], #4                 @load R10 from memory
        ldr r11, [r0], #4                 @load R11 from memory
        ldr r12, [r0], #4                 @load R12 from memory
        ldr r13, [r0], #4                 @load SP from memory
        ldr r14, [r0], #4                 @load LR from memory
        ldr r15, [r0], #4                 @load PC from memory
        
        @change back to Supervisor
        msr CPSR_c, #0xD3
        
        @restore CPSR
        ldr r1, [r0]                      @load CPSR from memory       
        msr SPSR, r1
        pop {r1}                          @load r1
        pop {r0}                          @load r0
        
        @return execution to current PID
        movs pc, lr

.ltorg

@ User software goes in this memory range
.set PROCESS_STACK_SZ, 0x800  @ Size of each process to record all registers + CPSR

@ User and supervisor mode stacks
@ Not necessary, just to make easier the understanding
@ of stack positions in memory
.set PID8_sup, 0x77705800
.set PID8, 0x77706000
.set PID7_sup, 0x77706800
.set PID7, 0x77707000
.set PID6_sup, 0x77707800
.set PID6, 0x77708000
.set PID5_sup, 0x77708800
.set PID5, 0x77709000
.set PID4_sup, 0x77709800
.set PID4, 0x7770A000
.set PID3_sup, 0x7770A800
.set PID3, 0x7770B000
.set PID2_sup, 0x7770B800
.set PID2, 0x7770C000
.set PID1_sup, 0x7770C800
.set PID1, 0x7770D000

@ Array to hold saved contexts
contexts: .space 544            @ 17 registers * 4 bytes each register * 8 process = 544

@ Graphic representation of contexts
@ PID1: [R0][R1][R2][R3][R4][R5][R6][R7][R8][R9][R10][R11][R12][SP][LR][PC][CSPR]
@ PID2: [R0][R1][R2][R3][R4][R5][R6][R7][R8][R9][R10][R11][R12][SP][LR][PC][CSPR]
@ PID3: [R0][R1][R2][R3][R4][R5][R6][R7][R8][R9][R10][R11][R12][SP][LR][PC][CSPR]
@ PID4: [R0][R1][R2][R3][R4][R5][R6][R7][R8][R9][R10][R11][R12][SP][LR][PC][CSPR]
@ PID5: [R0][R1][R2][R3][R4][R5][R6][R7][R8][R9][R10][R11][R12][SP][LR][PC][CSPR]
@ PID6: [R0][R1][R2][R3][R4][R5][R6][R7][R8][R9][R10][R11][R12][SP][LR][PC][CSPR]
@ PID7: [R0][R1][R2][R3][R4][R5][R6][R7][R8][R9][R10][R11][R12][SP][LR][PC][CSPR]
@ PID8: [R0][R1][R2][R3][R4][R5][R6][R7][R8][R9][R10][R11][R12][SP][LR][PC][CSPR]
@ Each PID row: 17 registers * 4 bytes each register = 68 Bytes
@ PID_head_context_address = 0x7770D800 + PID*68
@ PID_PC_address = PID_head_context_address + 15*4


process_pcs:       .space 32    @stores PC from all process to be used later 32 = 4bytes from lr * 8 available
current_process:    .space 4    @label that has current process PID
process_states:      .space 8    @array of process containing one byte for each process if 0->dead, if 1->alive

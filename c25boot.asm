; *******************************************************************
; Basic loader program - Reads .LOD files from the serial port into
; data ram then copies from data ram into program ram. When the whole
; file has been copied into program ram, it branches to the start address
; and starts running the downloaded program. It also echos any received
; data back to the PC for error checking.

; Written by Steven J. Merrifield

; 08 Aug 94 - Initial release
; 23 Aug 94 - Removed delays after every uart instruction that was not
;             part of the init. sequence (now loads more quickly)
; 02 Sep 94 - Added interrupt vector table

; LED 0 turns on after initialising the UART.
; LED 1 turns on after reading the start address.
; LED 2 turns on after reading the length of code to be sent.
; LED 3 turns on after loading the program.
; *******************************************************************

vect        .set 8000h          ; start of interrupt vector table in DRAM

            .sect "VECTORS"
            b INIT              ; external reset
            b vect+2            ; int 0
            b vect+4            ; int 1
            b vect+6            ; int 2
            b vect+8            ; reserved
            b vect+10           ; reserved
            b vect+12           ; reserved
            b vect+14           ; reserved
            b vect+16           ; reserved
            b vect+18           ; reserved
            b vect+20           ; reserved
            b vect+22           ; reserved
            b vect+24           ; internal timer
            b vect+26           ; serial port rx
            b vect+28           ; serial port tx
            b vect+30           ; trap instruction address

            .text

temp0       .set 0
temp1       .set 1
temp2       .set 2
boot_addr   .set 3      ; destination addr. of boot code
byte_cnt    .set 4      ; length of code to be sent
save_acc    .set 5      ; temp for intermediate acc. access
ON          .set 6      ; data to turn LED on
OFF         .set 7      ; data to turn LED off

; IO ports
LED0        .set 0
LED1        .set 1
LED2        .set 2
LED3        .set 3
u_data      .set 4      ; UART data register
u_ctrl      .set 5      ; UART control/status register

; *******************************************************************
;           Execution starts here
; *******************************************************************
INIT        dint     ; disable interrupts
            rovm     ; disable overflow
            ssxm     ; allow extended signed no's.
            cnfd     ; configure block B0 as data memory
            ldpk 4   ; start data memory at 0200h

; Setup LED data
            zac
            sacl OFF
            lalk 01
            sacl ON

; Reset all LEDs
            out OFF,LED0
            out OFF,LED1
            out OFF,LED2
            out OFF,LED3

; *******************************************************************
;           Assume worst-case UART initialisation
; *******************************************************************
            zac
            sacl temp0
            out temp0,5     ; set sync mode operation
            call U_DELAY

            out temp0,5     ; load 1st dummy sync char
            call U_DELAY

            out temp0,5     ; load 2nd dummy sync char
            call U_DELAY

            lack 40h        ; internal reset
            sacl temp0
            out temp0,5
            call U_DELAY

; UART is now idling and waiting for configuration data

            lack 04Eh   ; N,8,1 x16
            sacl temp0
            out temp0,5
            call U_DELAY

            lack 05     ; enable Tx & Rx
            sacl temp0
            out temp0,5
            call U_DELAY

; Light LED0 after UART initialisation
            out ON,LED0

; *******************************************************************
; Get destination addr. of boot code & store it in dma(boot_addr)
; *******************************************************************
label1      in temp0,u_ctrl   ;
            bit temp0,14      ;  wait until we rx a char
            bbz label1        ;

            in temp0,u_data     ; read high byte of dest. addr.
            call SENDBACK
            lac temp0,8         ; shl 8
            sacl save_acc

label2      in temp0,u_ctrl  ;
            bit temp0,14     ;  wait for a char
            bbz label2       ;

            in temp0,u_data     ; read low byte of dest. addr.
            call SENDBACK
            lac temp0
            andk 0FFh           ; mask out top 8 bits
            sacl temp0
            lac save_acc
            or temp0
            sacl boot_addr

; Light LED1 after setting up boot adddress
            out ON,LED1

; *******************************************************************
;           Get length of code & store it in dma(byte_cnt)
; *******************************************************************
label3      in temp0,u_ctrl ;
            bit temp0,14    ; wait for a char
            bbz label3      ;

            in temp0,u_data     ; high byte of count
            call SENDBACK
            lac temp0,8
            sacl save_acc

label4      in temp0,u_ctrl
            bit temp0,14        ; wait for a char
            bbz label4

            in temp0,u_data     ; low byte of count
            call SENDBACK
            lac temp0
            andk 0FFh ; mask out top 8 bits
            sacl temp0
            lac save_acc
            or temp0
            addk 01h ; add 1 so byte_cnt agrees with Lurch's protocol
            sacl byte_cnt

; Light LED2 after setting up byte count
            out ON,LED2

; *******************************************************************
; Get code and store it in a temp memory location in data ram then transfer
; from that temp location to program ram and decrement byte_cnt. Check if
; byte_cnt = 0, if not then get next piece of code.
; *******************************************************************
            lac boot_addr   ; store boot_addr in temp mem loc so it can
            sacl temp1      ; be incremented for tblw

loop1       in temp0,u_ctrl
            bit temp0,14        ; wait for a char
            bbz loop1

            in temp0,u_data     ; high byte of data
            call SENDBACK
            lac temp0,8
            sacl save_acc

label5      in temp0,u_ctrl
            bit temp0,14        ; wait for a char
            bbz label5

            in temp0,u_data     ; low byte of data
            call SENDBACK
            lac temp0
            andk 0FFh           ; mask out top 8 bits
            sacl temp0
            lac save_acc
            or temp0
            sacl temp0  ; write data to temp mem. addr. for tlbw

            lac temp1       ; temp1 contains addr to write to in pm
            tblw temp0      ; transfer from dma(temp0) to pma(ACC)
            addk 1          ; increment ACC for next access by tblw
            sacl temp1      ; save new index for pma
            lac byte_cnt
            subk 1
            sacl byte_cnt
            bnz loop1

; Light LED3 after loading code into pm
            out ON,LED3

; jump to dest. addr and start running program
            lac boot_addr
            bacc

; *******************************************************************
; Echo the received byte back to the PC for error checking. The PC end
; compares the sent byte with the echoed byte, and if they are not the
; same it terminates with an "echo test error".
; *******************************************************************
SENDBACK    sacl save_acc
CHECK       in temp2,u_ctrl     ; wait until TxRDY
            bit temp2,15
            bbz CHECK

            out temp0,u_data    ; echo data back to PC
            lac save_acc
            ret

; *******************************************************************
; We need a delay of at least 33 CPU clock cycles (at 40MHz) after each
; UART access during initialisation to allow for the recovery time.
; *******************************************************************
U_DELAY     sacl save_acc       ; PUSH accumulator
            lalk 7
wait        nop                 ; 1 clock cycle
            subk 1              ; 1 clock cycle
            bnz wait            ; 3 clock cycles
            lac save_acc        ; POP accumulator
            ret

            .end


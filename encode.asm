; *******************************************************************
; This program reads a character from the PC serial interface, and
; uses a lookup table to decide which tone to generate. It then
; synthesises a DTMF tone for a fixed period of time, then zeros the
; ouput of the DAC.

; Note that with a 40MHz CPU clock, the actual sampling frequency could
; not be set to exactly 8kHz. It was defined to be 7936.5Hz, and the
; values in the key_table reflect this alteration.

; Note that flags are used to determine when to process new data.
; If the flag is set to 00FFh then an interrupt has occured, and the
; program branches to the relevant service routine.

; When a transmit interrupt occurs the program branches to the transmit
; interrupt service routine and sets the tx_flag. It then gets data
; from data ram and writes it to the tx serial port register.

; Written by Steven J. Merrifield, 1994
; *******************************************************************

            .sect "VECTORS"
            b start         ; 0 - External reset
            b INT0          ; 2 - User int 0
            b INT1          ; 4 - User int 1
            b INT2          ; 6 - User int 2
            b d_int         ; 8 - Reserved
            b d_int         ; 10 - Reserved
            b d_int         ; 12 - Reserved
            b d_int         ; 14 - Reserved
            b d_int         ; 16 - Reserved
            b d_int         ; 18 - Reserved
            b d_int         ; 20 - Reserved
            b d_int         ; 22 - Reserved
            b tim_int       ; 24 - Internal timer
            b rx_int        ; 26 - Serial port rx
            b tx_int        ; 28 - Serial port tx
            b d_int         ; 30 - Trap instruction addr.

            .text

sine        .word   0000h, 0324h, 0646h, 0964h, 0c7ch, 0f8dh, 1294h
            .word   1590h, 187eh, 1b5dh, 1e2bh, 20e7h, 238eh, 2620h
            .word   289ah, 2afbh, 2d41h, 2f6ch, 3179h, 3368h, 3537h
            .word   36e5h, 3871h, 39dbh, 3b21h, 3c42h, 3d3fh, 3e15h
            .word   3ec5h, 3f4fh, 3fb1h, 3fech, 4000h, 3fech, 3fb1h
            .word   3f4fh, 3ec5h, 3e15h, 3d3fh, 3c42h, 3b21h, 39dbh
            .word   3871h, 36e5h, 3537h, 3368h, 3179h, 2f6ch, 2d41h
            .word   2afbh, 289ah, 2620h, 238eh, 20e7h, 1e2bh, 1b5dh
            .word   187eh, 1590h, 1294h, 0f8dh, 0c7ch, 0964h, 0646h
            .word   0324h, 0000h
            .word   0fcdch, 0f9bah, 0f9bah, 0f69ch, 0f384h, 0f073h
            .word   0ed6ch, 0ea70h, 0e782h, 0e4a3h, 0e1d5h, 0df19h
            .word   0dc72h, 0d9e0h, 0d766h, 0d505h, 0d2bfh, 0d094h
            .word   0ce87h, 0cc98h, 0cac9h, 0c91bh, 0c78fh, 0c625h
            .word   0c4dfh, 0c3beh, 0c2c1h, 0c1ebh, 0c13bh, 0c0b1h
            .word   0c04fh, 0c014h, 0c000h, 0c014h, 0c04fh, 0c0b1h
            .word   0c13bh, 0c1ebh, 0c2c1h, 0c3beh, 0c4dfh, 0c625h
            .word   0c78fh, 0c91bh, 0cac9h, 0cc98h, 0ce87h, 0d094h
            .word   0d2bfh, 0d505h, 0d766h, 0d9e0h, 0dc72h, 0df19h
            .word   0e1d5h, 0e4a3h, 0e782h, 0ea70h, 0ed6ch, 0f073h
            .word   0f384h, 0f69ch, 0f9bah, 0fcdch

m1          .word 07fffh

key_table   .word 0f2dh, 158ch    ; 0
            .word 0b3dh, 137fh    ; 1
            .word 0b3dh, 158ch    ; 2
            .word 0b3dh, 17d2h    ; 3
            .word 0c6bh, 137fh    ; 4
            .word 0c6bh, 158ch    ; 5
            .word 0c6bh, 17d2h    ; 6
            .word 0dbdh, 137fh    ; 7
            .word 0dbdh, 158ch    ; 8
            .word 0dbdh, 17d2h    ; 9
            .word 0b3dh, 1a56h    ; A
            .word 0c6bh, 1a56h    ; B
            .word 0dbdh, 1a56h    ; C
            .word 0f2dh, 1a56h    ; D
            .word 0f2dh, 137fh    ; *
            .word 0f2dh, 17d2h    ; #

; IO ports
u_data      .set 4
u_ctrl      .set 5


; Page 0 variables (0000h)
dxr         .set 1              ; data tx reg. address
imr         .set 4              ; interrupt mask reg.
tx_flag     .set 96             ; data has been sent flag
stat_st     .set 97             ; temp for saving status reg
accl_st     .set 98             ; temp for low half of accumulator
acch_st     .set 99             ; temp for high half of accumulator
tx_data     .set 100            ; data to be tx'ed must be stored here
init_data   .set 101            ; data for init. must be stored here
init_flag   .set 102            ; flag for secondary communications

delta1      .set 107            ; increment for first sine wave
alpha1      .set 108
sin1        .set 109            ; actual sine wave value
temp        .set 110
mask        .set 111
sin_offset  .set 112            ; pointer into sine lookup table
key_offset  .set 113            ; pointer into keypad lookup table
temp2       .set 114
temp3       .set 115
tone_len    .set 116            ; address for time one tone is sent
delta2      .set 117            ; increment for second sine wave
alpha2      .set 118
sin2        .set 119            ; actual sine wave value
last        .set 120
sec_last    .set 121

; *******************************************************************
;           Initialization
; *******************************************************************
start       ldpk 0              ; point to data page zero
            fort 0              ; set serial port to be 16 bits wide
            rtxm                ; external sync
            sfsm                ; sync required for each transfer
            cnfd                ; configure block B0 as data

            zac                 ; reset tx and init flags
            sacl tx_flag
            sacl init_flag

            larp ar1            ; counter for time one tone is sent

            eint
            lalk 020h           ; enable only tx interrupt
            sacl imr

; *******************************************************************
;           Main program
; *******************************************************************
            lalk 1223h          ; setup TA and RA (divide by 9)
            sacl init_data
            call tx_ready
            lalk 1
            sacl init_flag
wait_2nd_a  lac init_flag       ; wait until secondary comms is finished
            bnz wait_2nd_a

            lalk 468eh          ; setup TB and RB (divide by 35)
            sacl init_data
            call tx_ready
            lalk 1
            sacl init_flag
wait_2nd_b  lac init_flag       ; wait until secondary comms is finished
            bnz wait_2nd_b

            lalk 1000            ; time one time is transmitted (50ms)
            sacl tone_len

            lalk m1
            tblr mask
            lalk sine           ; save start addr. of sinewave lookup table
            sacl sin_offset

            zac
            sacl alpha1         ; start at zero in sine LUT
            sacl alpha2
            sacl delta1
            sacl delta2

            lalk key_table      ; prepare keypad lookup table
            sacl key_offset

loop        call check_key      ; see if a key has been pressed
            lac alpha1,8
            sach temp
            lac temp
            add sin_offset
            tblr sin1           ; calculate first sine wave sample
            lac alpha1
            add delta1
            and mask
            sacl alpha1

; 2nd sine wave
            lac alpha2,8
            sach temp
            lac temp
            add sin_offset
            tblr sin2           ; calculate second sine wave sample
            lac alpha2
            add delta2
            and mask
            sacl alpha2

            lac sin1            ; add the first and second together
            add sin2

            sacl tx_data        ; write the combined sum to DAC
            call tx_ready


            banz loop           ; banz has got a built in decrement
            zac                 ; when loop has run down, zero DAC output
            sacl alpha1
            sacl alpha2
            sacl delta1
            sacl delta2
            b loop

; *******************************************************************
;           Check if a key was pressed
; *******************************************************************
check_key   in temp2,u_ctrl      ; test if key pressed
            bit temp2,14
            bbz end_check

            in temp2,u_data     ; read data from uart into temp addr.
            lac temp2           ; remove top half since IN only reads
            andk 00FFh          ; an 8 bit number, so the top 8 bits
            sacl temp2          ; will be garbage!

            lac temp2,1         ; mult kbhit by 2 since LUT uses row-col
            add key_offset      ; move to correct position in LUT
            tblr delta1         ; read row value from LUT
            addk 1
            tblr delta2         ; read column value from LUT
            lar ar1, tone_len   ; reset tone length after every keypress

end_check   ret
  
; *******************************************************************
;           Test if transmit flag is set
; *******************************************************************
tx_ready    lac tx_flag         ; when flag is set, tx_flag = 0ffh
            andk 00ffh
            subk 0ffh
            bnz tx_ready        ; wait until flag is set
            sacl tx_flag        ; if flag is set, then reset it
            ret

; *******************************************************************
;           Transmit interrupt service routine
; *******************************************************************
tx_int      sst stat_st         ; push status reg
            sacl accl_st        ; push accumulator
            sach acch_st

            bit init_flag,14    ; are we sending init. data?
            bbz test_2nd
send_2nd    lac init_data       ; send actual init. data (ie divide no's)
            sacl dxr
            zac
            sacl init_flag      ; reset init flag
            b exit_tx_int

test_2nd    bit init_flag,15
            bbnz send_1st
 
primary     lalk 0ffh           ; set transmit data flag
            sacl tx_flag
            lac tx_data         ; get data from memory
            andk 0fffch         ; mask out bottom 2 bits
            sacl dxr            ; write data to AIC
            b exit_tx_int

send_1st    lac init_flag       ; increment init_flag
            addk 1
            sacl init_flag
            lalk 03             ; start secondary communications
            sacl dxr

exit_tx_int zals accl_st        ; pop accumulator
            addh acch_st
            lst stat_st         ; pop status reg

            eint
            ret

; *******************************************************************
;           Interrupts we're not interested in
; *******************************************************************
d_int       ret
rx_int      ret
tim_int     ret
INT0        ret
INT1        ret
INT2        ret

            .end


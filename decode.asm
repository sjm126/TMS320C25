; *******************************************************************
; DTMF decoder implemented using the Goertzel algorithm.
; Electronics IV (Honours) Project 1994
; by Steven J. Merrifield
;
; This program incorporates changes made after the original thesis
; was submitted. Where there are discrepancies between this, and the
; original code, the code presented here should take precedence.
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

; IO ports
LED0        .set 0
LED1        .set 1
LED2        .set 2
LED3        .set 3
u_data      .set 4
u_ctrl      .set 5

; Page 0 variables (0000h)
drr         .set 0      ; data rx reg. address
dxr         .set 1      ; data tx reg. address
imr         .set 4      ; interrupt mask reg.
tx_flag     .set 96     ; data has been sent flag
rx_flag     .set 97     ; data has been received flag
stat_st     .set 98     ; temp for saving status reg. during subroutine calls
accl_st     .set 99     ; temp for low half of accumulator
acch_st     .set 100    ; temp for high half of accumulator
rx_data     .set 101    ; received data is stored here
tx_data     .set 102    ; data to be transmitted must be stored here
init_data   .set 105    ; data for initialisation muse be stored here
init_flag   .set 106    ; flag for secondary communications (initialisation)
OFF         .set 107    ; data to turn LED off
ON          .set 108    ; data to turn LED on
stat_1      .set 109    ; save status register 1

nfilt       .set 8          ; No. of filters (one for each row/col freq)
u_data      .set 4
u_ctrl      .set 5

dram        .set 0200h      ; DRAM starts at addr. 0200h  (ie DP = 4)
cs1         .set dram+00
cs2         .set dram+01
cs3         .set dram+02
cs4         .set dram+03
cs5         .set dram+04
cs6         .set dram+05
cs7         .set dram+06
cs8         .set dram+07

negmax      .set dram+08

rowmx       .set dram+11
colmx       .set dram+12
rowmax      .set dram+13
colmax      .set dram+14
count       .set dram+15
rowcol      .set dram+16
last        .set dram+19
sec_last    .set dram+20

dat11       .set dram+28
dat23       .set dram+33
dat14       .set dram+34
dat15       .set dram+36
dat17       .set dram+40
dat27       .set dram+41
dat18       .set dram+42
dat28       .set dram+43
dat29       .set dram+45
dat213      .set dram+53
dat216      .set dram+59
datin       .set dram+60
temp        .set dram+61
temp2       .set dram+62
temp3       .set dram+63
save_acc    .set dram+64
prnt        .set dram+65
test        .set dram+66

; Filter co-efficients for each row/col. frequency
; Fundamental - Real coeff N=205

tblstrt     .word 27906         ; 697 Hz
            .word 26802         ; 770 Hz
            .word 25597         ; 851 Hz
            .word 24295         ; 941 Hz
            .word 19057         ; 1209 Hz
            .word 15654         ; 1336 Hz
            .word 12945         ; 1477 Hz
            .word 09166         ; 1633 Hz
tblend      .word 08000h        ; NegMax - mask for data out

; *******************************************************************
;           Initialization
; *******************************************************************
start       ldpk 0          ; Point to data page zero
            fort 0          ; Set serial port to be 16 bits wide
            rtxm            ; External sync
            sfsm            ; Sync required for each transfer

            cnfd            ; Set block B0 to be data memory
            sovm            ; set overflow mode
            ssxm            ; set sign extention mode

            zac             ; Reset tx and rx flags
            sacl tx_flag
            sacl rx_flag
            sacl init_flag
            sacl OFF
            lalk 1
            sacl ON

            eint
            lalk 020h       ; Enable only tx interrupt to init AIC
            sacl imr

            lalk 1223h      ; setup TA and RA
            sacl init_data
            call tx_ready
            lalk 1
            sacl init_flag
wait_2nd_a  lac init_flag       ; wait until secondary comms is finished
            bnz wait_2nd_a

            lalk 468eh          ; setup TB and RB
            sacl init_data
            call tx_ready
            lalk 1
            sacl init_flag
wait_2nd_b  lac init_flag       ; wait until secondary comms is finished
            bnz wait_2nd_b

            lalk 010h           ; enable only rx int for receiveing data
            sacl imr

; ***********************************************
;       Execution starts here
; ***********************************************
            ldpk 4              ; Point to data page 4 (0200h)

            larp    ar0
            lrlk    ar0,cs1             ; Pointer to the start of ram to be
                                        ; initialised.
            lalk    tblstrt             ; Pointer to the start of init table.
            lrlk    ar1,tblend-tblstrt  ; Count of data to be moved.

next        tblr    *+,ar1
            addk    1
            banz    next,*-,ar0

            lalk 0ffh                   ; set last and second last decoded
            sacl last                   ; digits to be "invalid"
            sacl sec_last

again       zac                         ; Zero DFT loop variables
            lrlk 0,15
            lrlk 1,dat11
zero        larp 1
            sacl *+,0,0
            banz zero

; ***********************************************
;       Take data and calculate DFT loop
; ***********************************************
            lalk 205                    ; Set DFT loop variable
loop        sacl count
            lrlk ar0,cs8                ; Set up pointer to co-efficients.
            lrlk ar1,dat28              ; Set up pointer to delayed outputs.
            lrlk ar2,nfilt-1            ; Number of filters.

; read from AIC under interrupts
            ldpk 0
            call rx_ready
            lac rx_data
            ldpk 4

            sfr         ; Stop accumulator from overflowing by shifting
            sfr         ; data to the right - this effectively takes the
;            sfr        ; 16 bit value read from the serial port and converts
;            sfr        ; it to a 14 bit value as generated by the AIC.
                        ; My hardware has an amplifier which limits the gain
                        ; of the input, so only two shifts are required. When
                        ; run in the simulator, all four shifts are needed.

            sacl datin

; ***********************************************
;       Begin DFT loops
; ***********************************************
frpt        larp ar0
            lt *,ar1            ; load cos(8*C) ready for multiply
            lac datin,12        ; X(n)
            subh *-             ; X(n) - Y(n-2)
            mpy *               ; cos(8*C) * Y(n-1)
            ltd *               ; Y(n-1) -> Y(n-2)
            apac
            apac
            apac                ; X(n) + 2cos(8*C) * Y(n-1) - Y(n-2)
            sach *-,0,ar0       ;    --> Y(n-1)
            bv overflow
            mar *-,ar2          ; Decrement the co-efficient pointer.
            banz frpt,*-,ar0    ; Decrement the filter number.

            lac count           ; Repeat for length of transform
            subk 1
            bnz loop
            b check

overflow    out OFF,LED0        ; Show overflow status on LED0

; ***********************************************
;       Calculate energy at each frequency
; ***********************************************
check       lrlk 0,cs8
            lrlk 1,dat28
            lrlk 2,nfilt-1

maglp       call energy
            sach *-,1,ar0
            mar  *-,ar2
            banz maglp,*-,ar0

; ***********************************************
;       Compare energies and determine decode value
; ***********************************************
            lalk 3
            sacl rowmx
            sacl colmx

; ***********************************************
;       Find row peak
; ***********************************************
rows        lrlk 1,2
            lrlk 0,dat23
            lac dat14
            sacl rowmax
rowl        larp 0
            mar *-
            lac rowmax
            sub *
            bgez rowbr

            sar 1,rowmx
            lac *
            sacl rowmax
rowbr       mar *-,1
            banz rowl

; ***********************************************
;       Find column peak
; ***********************************************
column      lrlk 1,2
            lrlk 0,dat27
            lac dat18
            sacl colmax
coll        larp 0
            mar *-
            lac colmax
            sub *
            bgez colbr

            sar 1,colmx
            lac *
            sacl colmax
colbr       mar *-,1
            banz coll

; ***********************************************
;       Merge row / column together
; ***********************************************
            lac rowmx,4
            or  colmx
            sacl rowcol

; ***********************************************
;       Check for valid signal strength
; ***********************************************
sig_str     lac colmax
            subk 4
            blz invalid_dig

            lac rowmax
            subk 4
            blz invalid_dig
            b test_new

invalid_dig lalk 0ffh
            sacl rowcol

; ***********************************************
;       Test if the decoded digit is new
; ***********************************************
test_new    lac rowcol
            sub last
            bz samelast

            lac last
            sacl sec_last
            lac rowcol
            sacl last
            b again

samelast    lac rowcol
            sub sec_last
            bz again

            lac last
            sacl sec_last
            lac rowcol
            sacl last

; check if decoded digit == 0 (my TC prog can't read an ASCII null)
            lac rowcol
            subk 0
            bnz check_u
            addk 55h
            sacl rowcol

check_u     in temp,u_ctrl          ; wait until TxRdy
            bit temp,15
            bbz check_u
            out rowcol,u_data       ; send decoded digit to PC

            b again                 ; get next sample

; ***********************************************
; Energy calculation subroutine
; ***********************************************
energy      lac negmax,15       ; NegMax = 8000h
            add *,15,1
            sach count
            lt *-               ; -1/2 + CSn/2
            mpy count
            pac
            sach count,1        ; D2(CSn-1)/2
            lt *+
            mpy count
            pac
            sach count,1        ; D1 * D2(CSn-1)/2
            lac *-,15
            sub *,15
            abs
            sach *              ; abs(D2-D1)/2
            lt *
            mpy *
            pac                 ; ((D2-D1)/2)^2
            sub count,15        ; ((D2-D1)^2)/4 - D1*D2(CSn-1)/2
            ret

; *******************************************************************
;           Test if receive flag is set
; *******************************************************************
rx_ready    ldpk 0
            lac rx_flag
            andk 00ffh
            subk 0ffh
            bnz rx_ready
            sacl rx_flag
            ret

; *******************************************************************
;           Test if transmit flag is set
; *******************************************************************
tx_ready    ldpk 0
            lac tx_flag
            andk 00ffh
            subk 0ffh
            bnz tx_ready
            sacl tx_flag
            ret

; *******************************************************************
;           Receive interrupt service routine
; *******************************************************************
rx_int      sst stat_st         ; Push status register
            ldpk 0
            sst1 stat_1
            sacl accl_st        ; Push accumulator
            sach acch_st
            lalk 0ffh           ; Set received data flag
            sacl rx_flag
            lac drr             ; Get data from AIC
            sacl rx_data        ; Save it to memory
            zals accl_st        ; Pop accumulator
            addh acch_st
            lst1 stat_1
            lst stat_st         ; Pop status register
            eint
            ret

; *******************************************************************
;           Transmit interrupt service routine
; *******************************************************************
tx_int      sst stat_st     ; Push status reg
            ldpk 0
            sst1 stat_1
            sacl accl_st    ; Push accumulator
            sach acch_st

            bit init_flag,14
            bbz test_2nd
send_2nd    lac init_data
            sacl dxr
            zac
            sacl init_flag
            b exit_tx_int

test_2nd    bit init_flag,15
            bbnz send_1st
 
primary     lalk 0ffh       ; Set transmit data flag
            sacl tx_flag
            lac tx_data     ; Get data from memory
            andk 0fffch     ; Mask out bottom 2 bits
            sacl dxr        ; Write data to AIC
            b exit_tx_int

send_1st    rxf
            lac init_flag
            addk 1
            sacl init_flag
            lalk 03
            sacl dxr

exit_tx_int zals accl_st    ; Pop accumulator
            addh acch_st
            lst1 stat_1
            lst stat_st     ; Pop status reg

            eint
            ret

; *******************************************************************
;           Interrupts we're not interested in
; *******************************************************************
d_int       ret
tim_int     ret
int0        ret
int1        ret
int2        ret

            .end


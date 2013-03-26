/*************************************************************************
  REVISED VERSION !!! - This document contains code which was added after
  submission of the original thesis. Where there are discrepancies between
  this and the original version, the code presented here should take
  precedence.
  
  This program handles both the encoding and decoding of the DTMF codes
  to and from the TMS320C25 DSP board. It reads characters from the keyboard
  and sends them via the serial port to the C25 which generates the DTMF
  tone. It also reads back the decoded tone from the C25 and displays it on
  the screen.
*************************************************************************/

#include <dos.h>
#include <conio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "serial.h"       /* communication routines */

#define FALSE 0
#define TRUE !FALSE

#define NOERROR 0
#define BUFOVFL 1         /* buffer overflow error  */
#define RET_ERROR 99      /* all return(RET_ERROR); statements */

#define SBUFSIZ 1024      /* serial buffer size */

#define PadX 25           /* X position of keypad */
#define PadY 6            /* Y position of keypad */
#define back_color 1      /* color of background */
#define key_color 14      /* color of digits in keypad */
#define pad_color 15      /* color of key & keypad borders */

int key_count = 0,        /* number of keys pressed during encoding */
    send = FALSE,         /* flag for encoding routine */
    receive = FALSE,      /* flag for decoding routine */
    quit = FALSE,         /* flag to quit the current routine */
    EXITDOS = FALSE;      /* flag to quit the whole program */


void init_screen(void);   /* draws a fancy heading and background */
void ask_routine(void);   /* menu which prompts for the required routine */
void decode(void);        /* decoding routine */
void encode(void);        /* encoding routine */

void interrupt(*oldvects[2])();

int SError = NOERROR;
int portbase = 0;
static char ccbuf[SBUFSIZ];
unsigned int startbuf = 0;
unsigned int endbuf = 0;

/**************************************************************************
  Handle communications interrupts and put them in ccbuf
**************************************************************************/
void interrupt com_int(void)
{
  disable();
  if ((inportb(portbase + IIR) & RX_MASK) == RX_ID)
  {
    if (((endbuf + 1) & SBUFSIZ - 1) == startbuf) SError = BUFOVFL;
    ccbuf[endbuf++] = inportb(portbase + RXR);
    endbuf &= SBUFSIZ - 1;
  }
  /* Signal end of hardware interrupt */
  outportb(ICR, EOI);
  enable();
}

/*************************************************************************
  Output a character to the serial port
*************************************************************************/
int SerialOut(char x)
{
  long int timeout = 0x0000FFFFL;
  outportb(portbase + MCR,  MC_INT | DTR | RTS);
  /* Wait for Clear To Send from modem */
  while ((inportb(portbase + MSR) & CTS) == 0)
  if (!(--timeout))
    return (-1);
  timeout = 0x0000FFFFL;
  /* Wait for transmitter to clear */
  while ((inportb(portbase + LSR) & XMTRDY) == 0)
  if (!(--timeout))
    return (-1);
  disable();
  outportb(portbase + TXR, x);
  enable();
  return (0);
}

/*************************************************************************
  This routine returns the current value in the buffer
*************************************************************************/
int getccb(void)
{
  int res;
  if (endbuf == startbuf)
    return (-1);
  res = (int) ccbuf[startbuf++];
  startbuf %= SBUFSIZ;
  return (res);
}

/*************************************************************************
  Install our functions to handle communications
*************************************************************************/
void setvects(void)
{
  oldvects[0] = getvect(0x0B);
  oldvects[1] = getvect(0x0C);
  setvect(0x0B, com_int);
  setvect(0x0C, com_int);
}

/*************************************************************************
  Uninstall our vectors before exiting the program
*************************************************************************/
void resvects(void)
{
  setvect(0x0B, oldvects[0]);
  setvect(0x0C, oldvects[1]);
}

/*************************************************************************
  Turn on communications interrupts
*************************************************************************/
void i_enable(int pnum)
{
  int c;
  disable();
  c = inportb(portbase + MCR) | MC_INT;
  outportb(portbase + MCR, c);
  outportb(portbase + IER, RX_INT);
  c = inportb(IMR) & (pnum == COM1 ? IRQ4 : IRQ3);
  outportb(IMR, c);
  enable();
}

/*************************************************************************
  Turn off communications interrupts
*************************************************************************/
void i_disable(void)
{
  int c;
  disable();
  c = inportb(IMR) | ~IRQ3 | ~IRQ4;
  outportb(IMR, c);
  outportb(portbase + IER, 0);
  c = inportb(portbase + MCR) & ~MC_INT;
  outportb(portbase + MCR, c);
  enable();
}

/*************************************************************************
  Tell DSP board that we're ready to go
*************************************************************************/
void comm_on(void)
{
  int c, pnum;
  pnum = (portbase == COM1BASE ? COM1 : COM2);
  i_enable(pnum);
  c = inportb(portbase + MCR) | DTR | RTS;
  outportb(portbase + MCR, c);
}

/*************************************************************************
  Misc functions
*************************************************************************/
void comm_off(void)
{
  i_disable();
  outportb(portbase + MCR, 0);
}

void initserial(void)
{
  endbuf = startbuf = 0;
  setvects();
  comm_on();
}

void closeserial(void)
{
  comm_off();
  resvects();
}

int c_break(void)    /* Ctrl-break interrupt handler */
{
  i_disable();
  printf("\nWarning!   Ctrl-Break pressed... still online.\n");
  return(0);
}

/*************************************************************************
  Set the port number to use
*************************************************************************/
int SetPort(int Port)
{
  int Offset, far *RS232_Addr;
  switch (Port)
  { /* Sort out the base address */
    case COM1 : Offset = 0x0000; break;
    case COM2 : Offset = 0x0002; break;
    default   : return (-1);
  }
  RS232_Addr = MK_FP(0x0040, Offset);  /* Find out where the port is. */
  if (*RS232_Addr == NULL) return (-1);/* If NULL then port not used. */
  portbase = *RS232_Addr;              /* Otherwise set portbase      */
  return (0);
}

/*************************************************************************
  This routine sets the speed; will accept funny baud rates.
  Setting the speed requires that the DLAB be set on.
*************************************************************************/
int SetSpeed(int Speed)
{
  char c;
  int divisor;
  if (Speed == 0)                      /* Avoid divide by zero */
    return (-1);
  else
    divisor = (int) (115200L/Speed);
  if (portbase == 0)
    return (-1);
  disable();
  c = inportb(portbase + LCR);
  outportb(portbase + LCR, (c | 0x80)); /* Set DLAB */
  outportb(portbase + DLL, (divisor & 0x00FF));
  outportb(portbase + DLH, ((divisor >> 8) & 0x00FF));
  outportb(portbase + LCR, c);          /* Reset DLAB */
  enable();
  return (0);
}

/*************************************************************************
  Set other communications parameters
*************************************************************************/
int SetOthers(int Parity, int Bits, int StopBit)
{
  int setting;
  if (portbase == 0) return (-1);
  if (Bits < 5 || Bits > 8)	return (-1);
  if (StopBit != 1 && StopBit != 2)	return (-1);
  if (Parity != NO_PARITY && Parity != ODD_PARITY && Parity != EVEN_PARITY)
    return (-1);
  setting  = Bits-5;
  setting |= ((StopBit == 1) ? 0x00 : 0x04);
  setting |= Parity;
  disable();
  outportb(portbase + LCR, setting);
  enable();
  return (0);
}

/*************************************************************************
  Set up the port
*************************************************************************/
int SetSerial(int Port, int Speed, int Parity, int Bits, int StopBit)
{
  if (SetPort(Port)) return (-1);
  if (SetSpeed(Speed)) return (-1);
  if (SetOthers(Parity, Bits, StopBit)) return (-1);
  return (0);
}

/*************************************************************************
  End of serial handling and start of graphics/DTMF code - uses extended
  ASCII characters and these may not print correctly on paper.
*************************************************************************/
void init_screen()
{
  int i,j;
  textbackground(0);
  clrscr();
  textcolor(15);
  textbackground(back_color);
  cprintf("ษออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออป");
  cprintf("บ      Electronics IV (Honours) Project 1994    -    DTMF Encoder/Decoder      บ");
  cprintf("บ                              by  Steven J. Merrifield                        บ");
  cprintf("ศออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออออผ");
  for (i=5;i<25;i++)
  for (j=1;j<81;j++)
  {
    gotoxy(j,i);
      cprintf("ฐ");  /* ASCII 176 - may not print correctly on paper */
  }
}

/*************************************************************************
  Menu screen which waits for a response - uses extended ASCII characters
  and these may not print correctly on paper.
*************************************************************************/
void ask_routine()
{
  int x = 22, y = 10;
  char c;
  send = FALSE;
  receive = FALSE;
  EXITDOS = FALSE;
  gotoxy(x,y);   cprintf(" ษออออออออออออออออออออออออออออออออป ");
  gotoxy(x,y+1); cprintf(" บ 1. Encode (send) DTMF tones    บ ");
  gotoxy(x,y+2); cprintf(" บ 2. Decode (receive) DTMF tones บ ");
  gotoxy(x,y+3); cprintf(" บ 3. Quit to DOS                 บ ");
  gotoxy(x,y+4); cprintf(" ศออออออออออออออออออออออออออออออออผ ");
  gotoxy(x+4,y+6); cprintf(" Enter your choice (1-3) :   ");
  gotoxy(x+31,y+6);  /* put cursor at prompt postion */
  do
  {      /* There's probably a better way to do this.... */
    c = getch();
  } while ((c != 49) & (c != 50) & (c != 51));

  switch (c)
  {
    case 49: send = TRUE; break;        /*  key = '1'  */
    case 50: receive = TRUE; break;     /*  key = '2'  */
    case 51: EXITDOS = TRUE; break;     /*  key = '3'  */
  }
}

/*************************************************************************
  DTMF decoding section - uses extended ASCII characters.
*************************************************************************/
void decode()
{
  int ch, key, x=12, y=10;
  init_screen();
  gotoxy(x,y);   cprintf(" ษออออออออออออออออออออออออออออออออออออออออออออออออออออป ");
  gotoxy(x,y+1); cprintf(" บ Received number :                                  บ ");
  gotoxy(x,y+2); cprintf(" ศออออออออออออออออออออออออออออออออออออออออออออออออออออผ ");
  gotoxy(x+21,y+1);  /* put cursor at prompt postion */
  quit = FALSE;
  while ((!SError) & (!quit))
  {
    ch = getccb();    /* read char from serial port buffer */
    if (ch != -1)
    {
      switch(ch)
      {
        case 85: key = '1'; break;   /* DSP end sends 55h to represent 0 */
        case 1:  key = '2'; break;
        case 2:  key = '3'; break;
        case 3:  key = 'A'; break;
        case 16: key = '4'; break;
        case 17: key = '5'; break;
        case 18: key = '6'; break;
        case 19: key = 'B'; break;
        case 32: key = '7'; break;
        case 33: key = '8'; break;
        case 34: key = '9'; break;
        case 35: key = 'C'; break;
        case 48: key = '*'; break;
        case 49: key = '0'; break;
        case 50: key = '#'; break;
        case 51: key = 'D'; break;
      }
    textcolor(14);
    cprintf("%c",key);
    }
    if (kbhit())
      if (getch()==27) quit = TRUE;
  }
}

/*************************************************************************
  This function "flashes" a button when it is pressed
*************************************************************************/
int flash_key(char k)
{ int x,y;
  switch(*strupr(&k)) {
    case '1': x = PadX + 5;  y = PadY + 2;  break;
    case '4': x = PadX + 5;  y = PadY + 5;  break;
    case '7': x = PadX + 5;  y = PadY + 8;  break;
    case '2': x = PadX + 12; y = PadY + 2;  break;
    case '5': x = PadX + 12; y = PadY + 5;  break;
    case '8': x = PadX + 12; y = PadY + 8;  break;
    case '0': x = PadX + 12; y = PadY + 11; break;
    case '3': x = PadX + 19; y = PadY + 2;  break;
    case '6': x = PadX + 19; y = PadY + 5;  break;
    case '9': x = PadX + 19; y = PadY + 8;  break;
    case 'A': x = PadX + 26; y = PadY + 2;  break;
    case 'B': x = PadX + 26; y = PadY + 5;  break;
    case 'C': x = PadX + 26; y = PadY + 8;  break;
    case 'D': x = PadX + 26; y = PadY + 11; break;
    case '*': x = PadX + 5;  y = PadY + 11; break;
    case '#': x = PadX + 19; y = PadY + 11; break;
    default: return(1);  /* invalid key pressed */
  }
  key_count ++;          /* so we know where to put the next character */
  gotoxy(x-1,y);
  textcolor(back_color);
  textbackground(3);
  cprintf(" %c ",k);     /* flash the background color */
  delay(200);
  gotoxy(x-1,y);
  textcolor(key_color);
  textbackground(back_color);
  cprintf(" %c ",k);    /* then restore it to the way it was */
  gotoxy(PadX+16+key_count,PadY+15);
  cprintf("%c",k);      /* write the key pressed at the correct spot */
  return(0);
}

/*************************************************************************
  DTMF encoding section - uses extended ASCII characters
*************************************************************************/
void encode()
{
  int SendChar,ch;
  char k;
  init_screen();
  textcolor(pad_color);
  gotoxy(PadX,PadY);    cprintf("ษออออออออออออออออออออออออออออออป");
  gotoxy(PadX,PadY+1);  cprintf("บ  ฺฤฤฤฟ  ฺฤฤฤฟ  ฺฤฤฤฟ  ฺฤฤฤฟ  บ");
  gotoxy(PadX,PadY+2);  cprintf("บ  ณ   ณ  ณ   ณ  ณ   ณ  ณ   ณ  บ");
  gotoxy(PadX,PadY+3);  cprintf("บ  ภฤฤฤู  ภฤฤฤู  ภฤฤฤู  ภฤฤฤู  บ");
  gotoxy(PadX,PadY+4);  cprintf("บ  ฺฤฤฤฟ  ฺฤฤฤฟ  ฺฤฤฤฟ  ฺฤฤฤฟ  บ");
  gotoxy(PadX,PadY+5);  cprintf("บ  ณ   ณ  ณ   ณ  ณ   ณ  ณ   ณ  บ");
  gotoxy(PadX,PadY+6);  cprintf("บ  ภฤฤฤู  ภฤฤฤู  ภฤฤฤู  ภฤฤฤู  บ");
  gotoxy(PadX,PadY+7);  cprintf("บ  ฺฤฤฤฟ  ฺฤฤฤฟ  ฺฤฤฤฟ  ฺฤฤฤฟ  บ");
  gotoxy(PadX,PadY+8);  cprintf("บ  ณ   ณ  ณ   ณ  ณ   ณ  ณ   ณ  บ");
  gotoxy(PadX,PadY+9);  cprintf("บ  ภฤฤฤู  ภฤฤฤู  ภฤฤฤู  ภฤฤฤู  บ");
  gotoxy(PadX,PadY+10); cprintf("บ  ฺฤฤฤฟ  ฺฤฤฤฟ  ฺฤฤฤฟ  ฺฤฤฤฟ  บ");
  gotoxy(PadX,PadY+11); cprintf("บ  ณ   ณ  ณ   ณ  ณ   ณ  ณ   ณ  บ");
  gotoxy(PadX,PadY+12); cprintf("บ  ภฤฤฤู  ภฤฤฤู  ภฤฤฤู  ภฤฤฤู  บ");
  gotoxy(PadX,PadY+13); cprintf("ศออออออออออออออออออออออออออออออผ");
  textcolor(key_color);
  gotoxy(PadX+5,PadY+2);   cprintf("1"); gotoxy(PadX+12,PadY+2);  cprintf("2");
  gotoxy(PadX+19,PadY+2);  cprintf("3"); gotoxy(PadX+26,PadY+2);  cprintf("A");
  gotoxy(PadX+5,PadY+5);   cprintf("4"); gotoxy(PadX+12,PadY+5);  cprintf("5");
  gotoxy(PadX+19,PadY+5);  cprintf("6"); gotoxy(PadX+26,PadY+5);  cprintf("B");
  gotoxy(PadX+5,PadY+8);   cprintf("7"); gotoxy(PadX+12,PadY+8);  cprintf("8");
  gotoxy(PadX+19,PadY+8);  cprintf("9"); gotoxy(PadX+26,PadY+8);  cprintf("C");
  gotoxy(PadX+5,PadY+11);  cprintf("*"); gotoxy(PadX+12,PadY+11); cprintf("0");
  gotoxy(PadX+19,PadY+11); cprintf("#"); gotoxy(PadX+26,PadY+11); cprintf("D");
  gotoxy(PadX-1,PadY+15);
  textcolor(15);
  cprintf(" Number to dial :                  ");
  gotoxy(PadX+17,PadY+15);
  quit = FALSE;
  key_count = 0;
  while ((!SError) & (!quit))
  {
    ch = getccb();             /* read char from serial port buffer */
    if (ch != -1) putch(ch);   /* if buffer is not empty, then write char */
    k = getch();
    flash_key(k);
    switch(*strupr(&k))
    {
      case 27: quit = TRUE; break; /* key = 'ESC' */
      case 48: SendChar = 0x00; break;
      case 49: SendChar = 0x01; break;
      case 50: SendChar = 0x02; break;
      case 51: SendChar = 0x03; break;
      case 52: SendChar = 0x04; break;
      case 53: SendChar = 0x05; break;
      case 54: SendChar = 0x06; break;
      case 55: SendChar = 0x07; break;
      case 56: SendChar = 0x08; break;
      case 57: SendChar = 0x09; break;
      case 65: SendChar = 0x0A; break;
      case 66: SendChar = 0x0B; break;
      case 67: SendChar = 0x0C; break;
      case 68: SendChar = 0x0D; break;
      case 42: SendChar = 0x0E; break;   /*    key = '*'    */
      case 35: SendChar = 0x0F; break;   /*    key = '#'    */
    }
    if (k != 27) SerialOut(SendChar);
    delay(5);    /* get overrun errors at 'C25 end without this */
  }
}

/*************************************************************************
  Main program
**************************************************************************/
main(int argc, char **argv)
{
  int port;
  int speed;
  int parity = NO_PARITY;
  int data_bits = 8;
  int stop_bits = 1;
  if (argc < 3)
  {
    printf("DTMF encoder/decoder front end for TMS320C25 DSP board.\n");
    printf("Syntax :  %s <ComPort> <BaudRate>\n",argv[0]);
    return(99);
  }
  port = atoi(argv[1]);
  if ((port < 1) | (port > 2))   /* also covers if port == 0 (error) */
  {
    printf("Com port must be either 1 or 2\n");
    return(RET_ERROR);
  }
  if (port==1) port = COM1; else port = COM2;
  speed = atoi(argv[2]);
  if ((speed < 150) | (speed > 19200))	/* also covers speed == 0 (error) */
  {
    printf("Baud rate must be in the range 150 - 19200\n");
    return(RET_ERROR);
  }
  if (SetSerial(port, speed, parity, data_bits, stop_bits) != 0)
  {
    printf("Error setting up serial port.\n");
    return (RET_ERROR);
  }
  initserial();
  ctrlbrk(c_break);

  do
  {
    init_screen();
    ask_routine();
    if (send == TRUE) encode();
    if (receive == TRUE) decode();
  } while (EXITDOS != TRUE);

  textcolor(7);
  textbackground(0);
  clrscr();
  /* Check for errors */
  switch (SError)
  {
    case NOERROR: closeserial();
                  return (0);
    case BUFOVFL: printf("\nBuffer Overflow.\n");
                  closeserial();
                  return (RET_ERROR);
    default:      printf("\nUnknown Error, SError = %d\n", SError);
                  closeserial();
                  return (RET_ERROR);
  }
}



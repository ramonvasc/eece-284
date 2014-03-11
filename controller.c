#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <p89lpc9351.h>

#define XTAL 7373000L
#define BAUD 115200L


#define LCD_PORT P2
#define RW P1_6
#define DI P1_4
#define E P1_7
#define PWMPINl P3_0
#define PWMPINr P3_1
#define timer_mode TMOD
#define timer0 TH0

unsigned char pwm_width_left;
bit pwm_flag_left = 0;
unsigned char pwm_width_right;
bit pwm_flag_right = 0;


void initSerialPort(void)
{
	BRGCON=0x00; //Make sure the baud rate generator is off
	BRGR1=((XTAL/BAUD)-16)/0x100;
	BRGR0=((XTAL/BAUD)-16)%0x100;
	BRGCON=0x03; //Turn-on the baud rate generator
	SCON=0x52; //Serial port in mode 1, ren, txrdy, rxempty
	P1M1=0x00; //Enable pins RxD and Txd
	P1M2=0x00; //Enable pins RxD and Txd
}

void PWMPort (void)
{
	P3M1=0x00;	//set port 3 to be output
	P3M2=0x00;
}

void initLCDPort(void)
{
	P2M1=0x00;
	P2M2=0x00;
}


void Delay(unsigned int i)
{
	unsigned int j,k;
	
	for(j = 0; j < i; j++)
	{
		for(k = 0; k < 737; k++)
		{}
	}

}
void command(char i) 
{ 
 LCD_PORT = i; //put data on output Port 
 DI =0; //D/I=LOW : send instruction 
 RW =0; //R/W=LOW : Write 
 E = 1; 
 Delay(1); //enable pulse width >= 300ns 
 E = 0; //Clock enable: falling edge 
} 
/**********************************************************/ 


void write(char i) 
{ 
 LCD_PORT = i; //put data on output Port 
 DI =1; //D/I=LOW : send data 
 RW =0; //R/W=LOW : Write 
 E = 1; 
 Delay(1); //enable pulse width >= 300ns 
 E = 0; //Clock enable: falling edge 
} 
/**********************************************************/ 

void pwm_setup_left(int control_left)
{
	timer_mode = 0;
	pwm_width_left = control_left;
	IEN0_7 =1;
	IEN0_1 =1;
	TCON_4 =1; //timer control
}

void tim_left() interrupt 1
{
	if(!pwm_flag_left) //start of high level
	{
		pwm_flag_left = 1; //set flag
		PWMPINl = 1; //set pwm o/p pin
		timer0 = pwm_width_left; //load timer
		TCON_5 = 0; //clear interrupt flag
		return;
	}
	else //start low level
	{
		pwm_flag_left = 0; //clear flag
		PWMPINl = 0; //clear pwm o/p pin
		timer0 = 255 - pwm_width_left; //load timer
		TCON_5 = 0;	//clear interrupt flag
		return;
	}
}		

void pwm_setup_right(int control_right)
{
	timer_mode = 0;
	pwm_width_right = control_right;
	IEN0_7 =1;
	IEN0_1 =1;
	TCON_4 =1; //timer control
}

void tim_right() interrupt 2
{
	if(!pwm_flag_right) //start of high level
	{
		pwm_flag_right = 1; //set flag
		PWMPINr = 1; //set pwm o/p pin
		timer0 = pwm_width_right; //load timer
		TCON_5 = 0; //clear interrupt flag
		return;
	}
	else //start low level
	{
		pwm_flag_right = 0; //clear flag
		PWMPINr = 0; //clear pwm o/p pin
		timer0 = 255 - pwm_width_right; //load timer
		TCON_5 = 0;	//clear interrupt flag
		return;
	}
}			

void initLCD() 
{ 
 
 E = 0; //E = 0; 
 
 Delay(100); //Wait >15 msec after power is applied 
 command(0x30); //command 0x30 = Wake up 
 Delay(30); //must wait 5ms, busy flag not available 
 command(0x30); //command 0x30 = Wake up #2 
 Delay(10); //must wait 160us, busy flag not available 
 command(0x30); //command 0x30 = Wake up #3 
 Delay(10); //must wait 160us, busy flag not available 
 command(0x38); //Function set: 8-bit/2-line 
 command(0x10); //Set cursor 
 command(0x0c); //Display ON; Cursor ON 
 command(0x06); //Entry mode set 
}

void writeString (char* str)
{
 int i;
 for (i=0;i<strlen(str);i++)
 {
  write(str[i]);
  }
}

void Wait1S (void)
{
	_asm
	mov R2, #40
L3: mov R1, #250
L2: mov R0, #184
L1: djnz R0, L1 ; 2 machine cycles-> 2*0.27126us*184=100us
    djnz R1, L2 ; 100us*250=0.025s
    djnz R2, L3 ; 0.025s*40=1s
    _endasm;
}

void InitADC(void)
{
	// Set adc1 channel pins as input only 
	P0M1 |= (P0M1_4 | P0M1_3 | P0M1_2 | P0M1_1 | P0M1_0);
	P0M2 &= ~(P0M1_4 | P0M1_3 | P0M1_2 | P0M1_1 | P0M1_0);

	BURST1=1; //Autoscan continuos conversion mode
	ADMODB = CLK0; //ADC1 clock is 7.3728MHz/2
	ADINS  = (ADI13|ADI12|ADI11|ADI10|ADI01); // Select the five channels for conversion
	ADCON1 = (ENADC1|ADCS10); //Enable the converter and start immediately
	ADCON0 = (ENADC1|ADCS00);
	while((ADCI1&ADCON1&ADCI0&ADCON0)==0); //Wait for first conversion to complete
}

void main (void)
{
	initSerialPort();
	initLCDPort();
	initLCD();
	PWMPort();
	pwm_setup_left(100);
	tim_left();	//timer function
	pwm_setup_right(10);
	tim_right();	//timer function
	InitADC();
	command(0x80);
	command(0x01); // clear the lcd
	command(0x14); //move the cursor one block to the right
	//writeString("testando denovo");	
	//write('1');
	Delay(1000);		
	
	
	while(1)
	{
		char buffer [4];
		sprintf(buffer,"%f",((0.0386*AD1DAT3)-0.042)); //ADC values to be 
		writeString(buffer);
		command(0x02);
		Wait1S();
	}
	
}

//AD0DAT1 - Port 0.0
//AD1DAT0 - Port 0.1
//AD1DAT1 - Port 0.2
//AD1DAT2 - Port 0.3
//AD1DAT3 - Port 0.4

/********************************************************/

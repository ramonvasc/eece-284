#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <p89lpc9351.h>

#define XTAL 7373000L
#define BAUD 115200L


#define LCD_PORT P2
#define RW P1_6
#define DI P1_4
#define ENABLELCD P1_7
#define PWMPINLEFT P3_0
#define PWMPINRIGHT P3_1
#define TIMER_MODE TMOD
#define TIMER0 TH0
#define TIMER1 TH1

unsigned char PwmWidthLeft;
bit PwmFlagLeft = 0;
unsigned char PwmWidthRight;
bit PwmFlagRight = 0;
short timerCounter = 0;

//PID Variables
short thold = 3;
short blank_thold = 20;

short error = 0;
short last_error = 0;
short recent_error = 0;
short total = 0;

short leftFlag = 0;
short rightFlag = 0;
bit centered = 1;

short t = 0;
short prev_t = 0;
short base_spd = 0;

short pgain = 9;
short dgain = 0;
//short igain = 1;

//Clock Variables
short clock = 0;
short halfsecond = 0;
short fastclock = 0;
short turns = 0;
short afterTurn = 0;
//unsigned short cor_left = 0;
//unsigned short cor_right = 0;

//Middle Inductor/Turning Variables
short countWires = 0;
short start = 1;
bit leftmode = 0;
bit rightmode = 0;
	
//short thold_high = 15; 
//short timeThresh = 1;
//short thold_low = 10;
bit flag;
bit turning = 0;
unsigned short time0;
//float time1;
unsigned short endTime;
unsigned short turn_timer = 0;

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
/**********************************************************/ 

void pwmPort (void)
{
	P3M1=0x00;	//set port 3 to be output
	P3M2=0x03;
}
/**********************************************************/ 

void initLcdPort(void)
{
	P2M1=0x00; //set port 2 to be output
	P2M2=0x00;
}
/**********************************************************/ 

void Delay(unsigned short i)
{
	unsigned short j,k;
	
	for(j = 0; j < i; j++)
	{
		for(k = 0; k < 737; k++)
		{}
	}

}
/**********************************************************/ 

void commandLcd(char inputData) 
{ 
 LCD_PORT = inputData; //put data on output Port 
 DI =0; //D/I=LOW : send instruction 
 RW =0; //R/W=LOW : Write 
 ENABLELCD = 1; 
 Delay(1); //enable pulse width >= 300ns 
 ENABLELCD = 0; //Clock enable: falling edge 
} 
/**********************************************************/ 

void writeLcd(char inputData) 
{ 
 LCD_PORT = inputData; //put data on output Port 
 DI =1; //D/I=LOW : send data 
 RW =0; //R/W=LOW : Write 
 ENABLELCD = 1; 
 Delay(1); //enable pulse width >= 300ns 
 ENABLELCD = 0; //Clock enable: falling edge 
} 
/**********************************************************/ 

void pwmSetupLeft(short controlLeft)
{
	TIMER_MODE = 0; //set to counter mode
	PwmWidthLeft = controlLeft; //pwm duty cycle
	IEN0_7 =1;
	IEN0_1 =1; //timer 0 interrupt
	TCON_4 =1; //timer control to set an interrupt flag
}
/**********************************************************/ 

void pwmTimerLeft() interrupt 1
{
	if(!PwmFlagLeft) //start of high level
	{
		PwmFlagLeft = 1; //set pwm flag
		PWMPINLEFT = 1; //set pwm o/p pin
		TIMER0 = PwmWidthLeft; //load timer
		TCON_5 = 0; //clear interrupt flag		
		return;
	}
	else //start low level
	{
		PwmFlagLeft = 0; //clear pwm flag
		PWMPINLEFT = 0; //clear pwm o/p pin
		TIMER0 = 255 - PwmWidthLeft; //load timer
		TCON_5 = 0;	//clear interrupt flag
		return;
	}
}		
/**********************************************************/ 

void pwmSetupRight(int controlRight)
{
	TIMER_MODE = 0; //set to counter mode
	PwmWidthRight = controlRight; //pwm duty cycle
	IEN0_7 =1;
	IEN0_3 =1; //timer 1 interrupt
	TCON_6 =1; //timer control to set an interrupt flag
}
/**********************************************************/ 

void pwmTimerRight() interrupt 3
{
	if(!PwmFlagRight) //start of high level
	{
		PwmFlagRight = 1; //set pwm flag
		PWMPINRIGHT = 1; //set pwm o/p pin
		TIMER1 = PwmWidthRight; //load timer
		TCON_7 = 0; //clear interrupt flag
		return;
	}
	else //start low level
	{
		PwmFlagRight = 0; //clear pwm flag
		PWMPINRIGHT = 0; //clear pwm o/p pin
		TIMER1 = 255 - PwmWidthRight; //load timer
		TCON_7 = 0;	//clear interrupt flag
		return;
	}
}			
/**********************************************************/ 

void initLcd() 
{ 
 
 ENABLELCD = 0; //ENABLELCD = 0; 
 
 Delay(100); //Wait >15 msec after power is applied 
 commandLcd(0x30); //command 0x30 = Wake up 
 Delay(30); //must wait 5ms, busy flag not available 
 commandLcd(0x30); //command 0x30 = Wake up #2 
 Delay(10); //must wait 160us, busy flag not available 
 commandLcd(0x30); //command 0x30 = Wake up #3 
 Delay(10); //must wait 160us, busy flag not available 
 commandLcd(0x38); //Function set: 8-bit/2-line 
 commandLcd(0x10); //Set cursor 
 commandLcd(0x0c); //Display ON; Cursor ON 
 commandLcd(0x06); //Entry mode set 
}
/**********************************************************/ 

void writeLcdString (char* inputString)
{
	short i;
 	for (i=0;i<strlen(inputString);i++)
 	{
  		writeLcd(inputString[i]);
  	}
}
/**********************************************************/ 

void Wait1S (void)
{
	_asm
	mov R2, #1
L3: mov R1, #500
L2: mov R0, #184
L1: djnz R0, L1 ; 2 machine cycles-> 2*0.27126us*184=100us
    djnz R1, L2 ; 100us*250=0.025s
    djnz R2, L3 ; 0.025s*40=1s
    _endasm;
}
/**********************************************************/ 

void initADC(void)
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
/**********************************************************/ 

void lcdWriteADC (void) {
	
 	char lcdBuffer [4];
	//sprintf(lcdBuffer,"%.1f",((0.0386*AD1DAT3)-0.042)); //ADC 4 values to be displayed
	sprintf(lcdBuffer,"CW:%d",countWires); //ADC 4 values to be displayed
	commandLcd(0xC0);
	writeLcdString(lcdBuffer);
	sprintf(lcdBuffer,"%.1f",((0.0386*AD1DAT2)-0.042)); //ADC 3 values to be displayed
	//sprintf(lcdBuffer,"%d",AD1DAT0-AD1DAT1); //Displays the error
	//sprintf(lcdBuffer,"T:%d",turning);
	//sprintf(lcdBuffer,"M%d",AD1DAT3); //Middle Inductor Voltage
	commandLcd(0xC6); 
	writeLcdString(lcdBuffer);
		
}
/**********************************************************/ 

void lcdWriteTimer (void) {
	char lcdBuffer [4];
	sprintf(lcdBuffer,"%d",clock); //timer to be displayed
	//sprintf(lcdBuffer,"%d",clock-afterTurn); //timer to be displayed
	//sprintf(lcdBuffer,"M%d",AD1DAT3); //Middle Inductor Voltage
	commandLcd(0x02);
	writeLcdString(lcdBuffer);
}
/**********************************************************/ 

void lcdWritePWM (void) {
	char lcdBuffer [4];
		//sprintf(lcdBuffer,"L%d",AD1DAT0); //Left Inductor Voltage
		sprintf(lcdBuffer,"L%d",PwmWidthLeft); //Left pwm frequency
		commandLcd(0x86);
		writeLcdString(lcdBuffer);
		//sprintf(lcdBuffer,"R%d",AD1DAT1); //Right Inductor Voltage
		sprintf(lcdBuffer,"R%d",PwmWidthRight); //Left pwm frequency
		commandLcd(0x8A);
		writeLcdString(lcdBuffer);	
}

//AD0DAT1 - Port 0.0
//AD1DAT0 - Port 0.1
//AD1DAT1 - Port 0.2
//AD1DAT2 - Port 0.3
//AD1DAT3 - Port 0.4


/**********************************************************/ 



short checkState(short left_ind, short right_ind, short thold)
//Function checks the error state of the rover. 
{

	
	
	if(abs(left_ind-right_ind) < thold && left_ind > blank_thold && right_ind > blank_thold)
	{
		error = 0;
	}
	else if( abs(right_ind-left_ind) < thold && rightFlag == 1)
	{
		error = -255;
		//cor_right = halfsecond;
	}
	else if( abs(left_ind-right_ind) < thold && leftFlag == 1)
	{
		error = 255;
		//cor_left = halfsecond;
	}
	else if( (left_ind - right_ind) > thold)
	//Left inductor more positive, speed up right wheel
	{
		//error = 1;
		error = (left_ind-right_ind)-thold;
		leftFlag = 1;
		rightFlag = 0;
	} 
	else if( (right_ind - left_ind) > thold)
	//Right inductor more positive, speed up left wheel
	{
		//error = -1;
		error = (left_ind-right_ind)+thold;	
		rightFlag = 1;
		leftFlag = 0;
	}

	
	return error;

}


/******************************************************/

short pidController(short error)
{
	
	//Resets state timer
	if(error != last_error)
	{
		recent_error = last_error;
		prev_t = t;
		t = 1;
	}
	
	
	total = pgain*error + (short)(dgain*(error-recent_error)/(t+prev_t));
	
	
	last_error = error;
	
	if(fastclock % 1 == 0)
	{
		t++;
	}
	
	return total;
}

/******************************************************/
void drive(short total)
{

	pwmSetupLeft(abs(-base_spd+total));
	pwmTimerLeft();
	
	pwmSetupRight(base_spd+total);
	pwmTimerRight();

	if(-base_spd+total < -255)
	{
		pwmSetupLeft(255);
		pwmTimerLeft();	
			
	}
	
	if(-base_spd+total > 0)
	{
		pwmSetupLeft(0);
		pwmTimerLeft();	
			
	}
	
	if(base_spd+total > 255)
	{
		pwmSetupRight(255);
		pwmTimerRight();
	}
	
	if(base_spd+total < 0)
	{
		pwmSetupRight(0);
		pwmTimerRight();
	}


	
}



/******************************************************/


void leftTurn(short middle_ind)
{
	short tl_speed_left = 50;
	short tl_speed_right = 255;
	
	if(middle_ind >= 1 && turning == 0)
	{
		turning = 1;
		turn_timer = halfsecond;
	}

	if(turning == 1)
	{
			pwmSetupLeft(tl_speed_left);
			pwmTimerLeft();	//timer function
			pwmSetupRight(tl_speed_right);
			pwmTimerRight();	//timer function
	}
	
	if(turning == 1 && AD1DAT1 > 15 && halfsecond-turn_timer > 1)
	{
		turning = 0;
		countWires = 0;
		leftmode = 0;
		turns++;
		afterTurn = clock;
	}
}


void rightTurn(short middle_ind )
{
	short tl_speed_left = 255;
	short tl_speed_right = 50;
	
	if(middle_ind >= 1 && turning == 0)
	{
		turning = 1;
		turn_timer = halfsecond;
	}

	if(turning == 1)
	{
			pwmSetupLeft(tl_speed_left);
			pwmTimerLeft();	//timer function
			pwmSetupRight(tl_speed_right);
			pwmTimerRight();	//timer function
	}
	
	if(turning == 1 && AD1DAT0 > 15 && halfsecond-turn_timer > 1)
	{
		turning = 0;
		countWires = 0;
		rightmode = 0;
		turns++;
		afterTurn = clock;
	}
}
/***************************************************/


void middleInductor(short middle_ind)
{

		short thold_high = 60; 
		short timeThresh = 1;
		short thold_low = 50;

	

		if (middle_ind > thold_high && flag == 0 && leftmode == 0 && rightmode == 0)
		{
			flag = 1;
		}

		if (flag == 1 && middle_ind < thold_low && turning == 0 && leftmode == 0 && rightmode == 0)
		{
			flag = 0;
			countWires++;
			time0 = fastclock;
		}

		
		if( countWires == 1 && (fastclock-time0) > timeThresh)
		// Resets counter to 0 if time after reading 1 wire exceeds one second
		{
			countWires = 0;
		}
		
		if(countWires == 2 && (fastclock-time0)>timeThresh)
		{
			leftmode = 1;
			leftTurn(middle_ind);
		}

		if (countWires == 3 && (fastclock-time0)>timeThresh)
		{
			rightmode = 1;
			rightTurn(middle_ind);
		}

		if (start == 1 && countWires == 4  ) 	//Start is a variable that will be defined globally to indicate if we are starting or ending the timer
		{

			clock= 0;
			start = 0;
			countWires=0;


		}
		if (countWires == 4 && start==0) 	//Start is a variable that will be defined globally to indicate if we are starting or ending the timer

		{
			endTime = clock;
			pwmSetupLeft(255);
			pwmTimerLeft();	//timer function
			pwmSetupRight(255);
			pwmTimerRight();	//timer function
			
			start = 2;
			
		}
		
}




/*********************************************************************************************/




void main (void)
{
	
	
	
	initSerialPort();
	initLcdPort();
	initLcd();
	pwmPort();
	//pwmSetupLeft(100);
	//pwmTimerLeft();	//timer function
	//pwmSetupRight(10);
	//pwmTimerRight();	//timer function
	initADC();
	commandLcd(0x80);
	commandLcd(0x01); // clear the lcd
	commandLcd(0x14); //move the cursor one block to the right
	Delay(1000);		
	
	while(start != 2)
	{		
		
		if(timerCounter % 17 == 0)
		{
			commandLcd(0x01); // clear the lcd
		}
			
			
		//Writes info to LCD
		if(timerCounter % 34 == 0)
		{
			clock++;
		}
		 
		
		//PID Controller
		if(timerCounter % 17 == 0)
		{
			if(start == 0)
			{
				lcdWriteTimer();
			}
			lcdWritePWM();
			lcdWriteADC();	
			halfsecond++;
		}
		
		if(timerCounter % 13 == 0)
		{
			fastclock++;
		}
	
		
		if(turning == 0)
		{
			error = checkState(AD1DAT0, AD1DAT1,thold);
			total = pidController(error);
			drive(total);
		}
		   
		
		if(turns == 1)
		{
			thold = 4;
			pgain = 18;
			
			if(clock-afterTurn > 3)
			{
				pgain = 10;
				thold = 2;
			}
		}
		
		if(turns == 2)
		{
			thold = 2;
			pgain = 25;
			
			if( (clock-afterTurn) > 3)
			{
				base_spd = 50;
				thold = 2;
				pgain = 20;
			}
			
			if( (clock-afterTurn) > 7)
			{
				base_spd = 100;
				thold = 2;
				pgain = 30;
			}
			
			if( (clock-afterTurn) > 12)
			{
				base_spd = 0;
				thold = 2;
				pgain = 20;
			}
		}
		
		if(turns == 3)
		{
			base_spd = 150;
			thold = 2;
			pgain = 35;
			
			if( (clock-afterTurn) > 5)
			{
				base_spd = 0;
				thold = 3;
				pgain = 30;
			}
		}
		
		if(turns == 4)
		{	
			base_spd = 100;
			pgain = 25;
			thold = 2;
			
			if( (clock-afterTurn) > 5)
			{	
				thold = 2;
				base_spd = 0;
				pgain = 35;
			}
			
			
			if( (clock-afterTurn) > 20)
			{
				base_spd = 0;
				thold = 3;
				pgain = 20;
			}
			
		}
		
		middleInductor(AD1DAT3);

		Wait1S();
		timerCounter++;
	}
	
}

//AD0DAT1 - Port 0.0
//AD1DAT0 - Port 0.1
//AD1DAT1 - Port 0.2
//AD1DAT2 - Port 0.3
//AD1DAT3 - Port 0.4
/********************************************************/

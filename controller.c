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
int timerCounter = 0;
//int J = 0;

//PID Variables
short error = 0;
short last_error = 0;
short recent_error = 0;
short total = 0;

short leftFlag = 0;
short rightFlag = 0;
bit centered = 1;

short t = 0;
short prev_t = 0;
int base_spd = 200;

short pgain = 30;
short dgain = 10;
short igain = 4;

//Clock Variables
unsigned int clock = 0;


//Middle Inductor Variables
int countWires = 0;
bit start = 1;
	
int thold_high = 175; 
int timeThresh=3;
float thold_low = 170;
bit flag;
bit turning = 0;
float time0;
//float time1;
float endTime;
unsigned int turn_timer = 0;

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

void Delay(unsigned int i)
{
	unsigned int j,k;
	
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

void pwmSetupLeft(int controlLeft)
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
	int i;
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
	int left_ind = AD1DAT0;
	int right_ind = AD1DAT1;
	
 	char lcdBuffer [4];
	//sprintf(lcdBuffer,"%.1f",((0.0386*AD1DAT3)-0.042)); //ADC 4 values to be displayed
	sprintf(lcdBuffer,"%d",countWires); //ADC 4 values to be displayed
	commandLcd(0xC0);
	writeLcdString(lcdBuffer);
	//sprintf(lcdBuffer,"%.1f",((0.0386*AD1DAT2)-0.042)); //ADC 3 values to be displayed
	sprintf(lcdBuffer,"%d",left_ind-right_ind); //ADC 3 values to be displayed
	commandLcd(0xC6);
	writeLcdString(lcdBuffer);
		
}
/**********************************************************/ 

void lcdWriteTimer (void) {
	char lcdBuffer [4];
	sprintf(lcdBuffer,"%d",clock-turn_timer); //timer to be displayed
	commandLcd(0x02);
	writeLcdString(lcdBuffer);
}
/**********************************************************/ 

void lcdWritePWM (void) {

	int left_ind = AD1DAT0;
	int right_ind = AD1DAT1;
	int middle_ind = AD1DAT3;
	
	char lcdBuffer [4];
		sprintf(lcdBuffer,"%d",middle_ind); //Left pwm frequency
		commandLcd(0x86);
		writeLcdString(lcdBuffer);
		sprintf(lcdBuffer,"%d",turning); //Right pwm frequency
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


	if( left_ind - right_ind > thold)
	//Left inductor more positive, speed up right wheel
	{
		error = 1;
		leftFlag = 1;
		rightFlag = 0;
	} 
	else if( right_ind - left_ind > thold)
	//Right inductor more positive, speed up left wheel
	{
		error = -1;	
		rightFlag = 1;
		leftFlag = 0;
	}
`	else if( abs(left_ind - right_ind) < thold && rightFlag == 1)
	{
		error = -1;
	}
	else if( abs(left_ind - right_ind) < thold && leftFlag == 1)
	{
		error = 1;
	}
	
	if(abs(left_ind-right_ind) < 15 && left_ind > 130 && right_ind > 130)
	{
		error = 0;
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
	
	
	total = pgain*error + igain*error*t + (short)(dgain*(error-recent_error)/(t+prev_t));
	
	
	last_error = error;
	
	if(timerCounter % 1 ==0)
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

void LeftTurn()
{
	int speedRight = 25;
	int speedLeft = 210;
	
	
	if((clock-turn_timer) <= 2)
	{
		pwmSetupLeft(speedLeft);
		pwmTimerLeft();	//timer function
		pwmSetupRight(speedRight);
		pwmTimerRight();	//timer function
	}
	else
	{
		countWires=0;
		turning = 0;
	}
}


void middleInductor(int middle_ind)
{

	

		if (middle_ind >= thold_high && flag == 0)
		{
			flag = 1;
		}

		if (flag == 1 && middle_ind < thold_low)
		{
			flag = 0;
			countWires++;
			time0 = clock;
		}

		
		//if(countWires == 2 && abs(clock-time0) > timeThresh )
		if(countWires == 2)
		{
			if(turning == 0)
			{
				turn_timer = clock;
				turning = 1;
			}
			
			LeftTurn();
		}
		
		/*
		if (countWires == 3 && (time0-clock)>timeThresh)
		{
			turning = 1;
			 RightTurn();
			 countWires=0;
		}
		
		/*
		if (countWires = 4 &&  start == 1) 	//Start is a variable that will be defined globally to indicate if we are starting or ending the timer

		{

			//clock=0;
			start = 0;
			countWires=0;


		}
		if (countWires = 4 &&  Start==FALSE) 	//Start is a variable that will be defined globally to indicate if we are starting or ending the timer

		{
			//endTime = clock;
			pwmSetupLeft(0);
			pwmTimerLeft();	//timer function
			pwmSetupRight(0);
			pwmTimerRight();	//timer function
			
			
			char lcdBuffer [4];
			sprintf(lcdBuffer,"%d",endTime); //endtime to be displayed
			commandLcd(0x02);
			writeLcdString(lcdBuffer);

		 //tHIS WILL STOP THE ROBOT AND END THE TIMER
		}
		
		*/
}




/*
void RightTurn(void)
{
	int speedRight = 100;
	int speedLeft = 200;

	while(error) 
	{
		pwmSetupLeft(speedLeft);
		pwmTimerLeft();	//timer function
		pwmSetupRight(SpeedRight);
		pwmTimerRight();	//timer function
	}
	
	//Send a  Vlotage stronger PWM to the rhight wheel than to the left wheel
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
	
	while(1)
	{
		if(timerCounter % 17 == 0)
		{
			commandLcd(0x01); // clear the lcd
			}
			
			
		//Writes info to LCD
		if(timerCounter % 34 == 0)
		{
			clock++;
			lcdWriteTimer();
		}
		 
		
		//PID Controller
		if(timerCounter % 17 == 0)
		{
			lcdWritePWM();
			lcdWriteADC();	
			
		}
		
		if(timerCounter % 4 == 0)
		{
			middleInductor(AD1DAT3);
		}
		
		if(turning == 0)
		{
			error = checkState(AD1DAT0,AD1DAT1,8);
			total = pidController(error);
			drive(total);
		}
		   
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


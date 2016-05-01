/*
  ECE375 Kyle, Max, Tom
*/

#include  <hidef.h>      /* common defines and macros */
#include    <stdio.h>
#include    <mc9s12dg256.h>
#pragma LINK_INFO DERIVATIVE "mc9s12dg256b"


#define HiCnt 3       // high pulse duration in TCNT counts
#define LoCnt 500      // low signal between pulses time in TCNT Counts
#define CompareCheck 180 

#define fullSpeedForwardTime 0.0002375		//in seconds the time for 5700 clk pulses
#define fullSpeedBackwardTime 0.0001375		//in seconds the time for 3300 clk pulses
int diff = 0; 
unsigned int encoderRotationCountL = 0;
unsigned int encoderRotationCountR = 0;
     
char  HiorLo;
#pragma CODE_SEG __NEAR_SEG NON_BANKED




void interrupt 9 handler1() {
 HILOtimes1();
}

void interrupt 8 handler0() {
 HILOtimes0();
}
  


double time;
double dist;
double a = 0.0000015;
unsigned int countL=255; //set count value to 1
unsigned int countR=255; //set count value to 1
int newSpeedL =0;
int newSpeedR =0;
int NEW = 0;
int x = 0;
int y = 0;
int Ping = 0;
int iterationCount;
int i;
double t;
double wheelDistance;

unsigned int motorL = 4900;
unsigned int motorR = 4900;

void goStraight(int dis);	//forward-declared method
void turn(int dis);		//forward-declared method
void GoStraight(int);	//forward-declared method
int PingSensor(int);	//forward-declared method
void TurnLeft(int degrees, char direction);	//forward-declared method
void TurnRight(int);	//forward-declared method
void Stop (int);	//forward-declared method
void END (int);	//forward-declared method
//long PingSensor(void);	//forward-declared method
//long PingSensor(void);	//forward-declared method


static  long leftAccumulator = 0;    // total counts from Right Encoder
static  long rightAccumulator = 0;   // total counts from Left Encoder

#define ACCUM_MAX_THRESHOLD 180     // value used to know when to move value into s/w counter

void  ResetEncoder() 
{
  DDRT &=  0xF3;   //init PT2 & PT3 as input
  ICPAR = 0xFC;   //enable PT2 & PT3 as pulse accum
  TCTL4 = 0x50;   //set to capture rising edge

  leftAccumulator = 0;  
  rightAccumulator = 0;

  PACN3 = 0; // reset hw accumulators
  PACN2 = 0; 
}

void  UpdateEncoderTotals() 
{
  // increment S/W accumulators before overflow occurs
  
  if( PACN3 >= ACCUM_MAX_THRESHOLD ) 
  {
    leftAccumulator =  leftAccumulator + PACN3;
    PACN3 = 0;
  }
  if( PACN2 >= ACCUM_MAX_THRESHOLD ) 
  {
    rightAccumulator =  rightAccumulator + PACN2;
    PACN2 = 0;
  }
}

long GetLeftEncoderTotal()
{
  return( leftAccumulator + PACN3 ); 
}

long GetRightEncoderTotal()
{
  return( rightAccumulator + PACN2 ); 
}


static  long  motorSpeed;    // current motor speeds
static  long  lAdjustment;
static  long  rAdjustment;

#define STOPPED_SPEED (4500)
#define MAX_FORWARD_SPEED (5700)
#define MAX_REVERSE_SPEED (3300)

void  InitialSpeed( long speed ) 
{
  motorSpeed = speed;  
  set_servo54(motorSpeed);
  set_servo76(motorSpeed);
}

void  AdjustSpeeds( long lSpeed, long rSpeed ) 
{
  set_servo54(lSpeed);
  set_servo76(rSpeed);
}

void  StopMoving( ) 
{
  set_servo54(STOPPED_SPEED);
  set_servo76(STOPPED_SPEED);
}

void  SpeedAdjust( )
{
  long  deltaCount;
  
  UpdateEncoderTotals();
  deltaCount = GetLeftEncoderTotal() - GetRightEncoderTotal();

  if( motorSpeed > STOPPED_SPEED ) 
  {
    
    lAdjustment = motorSpeed + (4 * deltaCount);
    rAdjustment = motorSpeed - (4 * deltaCount);
    
    if( lAdjustment > MAX_FORWARD_SPEED )
      lAdjustment = MAX_FORWARD_SPEED;
    if( rAdjustment > MAX_FORWARD_SPEED )
      rAdjustment = MAX_FORWARD_SPEED;
    if( lAdjustment < STOPPED_SPEED )
      lAdjustment = STOPPED_SPEED;
    if( rAdjustment < STOPPED_SPEED )
      rAdjustment = STOPPED_SPEED;
  } 
  else 
  {
    lAdjustment = motorSpeed - (4 * deltaCount);
    rAdjustment = motorSpeed + (4 * deltaCount);
    
    if( lAdjustment < MAX_REVERSE_SPEED )
      lAdjustment = MAX_REVERSE_SPEED;
    if( rAdjustment < MAX_REVERSE_SPEED )
      rAdjustment = MAX_REVERSE_SPEED;
    if( lAdjustment > STOPPED_SPEED )
      lAdjustment = STOPPED_SPEED;
    if( rAdjustment > STOPPED_SPEED )
      rAdjustment = STOPPED_SPEED;
  }
  
  // adjust speeds if one is faster than the other (simple control loop)
  AdjustSpeeds( lAdjustment, rAdjustment ); 
}





#define CM_TRAVEL_PER_REVOLUTION  (33)
#define PULSES_PER_REVOLUTION     (90)
#define PI 3.14159265358979323846	//the constant PI
#define r 5.2375		//in centi-meters the radius of the wheel
#define ONE_ENCODER_TICK_DISTANCE 0.36564648 //in centi-meters
long selectedEncoderCount;

void goStraight(int dis){//centi-meters
	
	
	long  deltaCount;

	UpdateEncoderTotals();
	deltaCount = GetLeftEncoderTotal() - GetRightEncoderTotal();
	selectedEncoderCount = ((dis/ONE_ENCODER_TICK_DISTANCE));
	
	if(GetLeftEncoderTotal() >= selectedEncoderCount || GetRightEncoderTotal() >= selectedEncoderCount){
		StopMoving();
	} else{
		if( motorSpeed > STOPPED_SPEED ) 
		{

			lAdjustment = motorSpeed + (4 * deltaCount);
			rAdjustment = motorSpeed - (4 * deltaCount);

		if( lAdjustment > MAX_FORWARD_SPEED )
		  lAdjustment = MAX_FORWARD_SPEED;
		if( rAdjustment > MAX_FORWARD_SPEED )
		  rAdjustment = MAX_FORWARD_SPEED;
		if( lAdjustment < STOPPED_SPEED )
		  lAdjustment = STOPPED_SPEED;
		if( rAdjustment < STOPPED_SPEED )
		  rAdjustment = STOPPED_SPEED;
		} 
		else 
		{
			lAdjustment = motorSpeed - (4 * deltaCount);
			rAdjustment = motorSpeed + (4 * deltaCount);

		if( lAdjustment < MAX_REVERSE_SPEED )
		  lAdjustment = MAX_REVERSE_SPEED;
		if( rAdjustment < MAX_REVERSE_SPEED )
		  rAdjustment = MAX_REVERSE_SPEED;
		if( lAdjustment > STOPPED_SPEED )
		  lAdjustment = STOPPED_SPEED;
		if( rAdjustment > STOPPED_SPEED )
		  rAdjustment = STOPPED_SPEED;
		}

		// adjust speeds if one is faster than the other (simple control loop)
		AdjustSpeeds( lAdjustment, rAdjustment ); 
	}
  return;
}

void turn(int dis){
	selectedEncoderCount = ((dis/ONE_ENCODER_TICK_DISTANCE));
	if(GetLeftEncoderTotal() >= selectedEncoderCount || GetRightEncoderTotal() >= selectedEncoderCount){
		StopMoving();
	}
}

int HI_time0; // High time (pulse width)
int HI_time1; // High time (pulse width)

char  PingHiorLo;
char  ParkHiorLo;
#define PingHiCnt 3       // high pulse duration in TCNT counts
#define PingLoCnt 500      // low signal between pulses time in TCNT Counts
#define ParkHiCnt 3       // high pulse duration in TCNT counts
#define ParkLoCnt 500      // low signal between pulses time in TCNT Counts  
double Pingtime;
double Pingdist;
double Parktime;
double Parkdist;


void ResetUltraSoundSensor(char sensor){
	TSCR1 = 0x80;	//timer system control register
	TSCR2 = 0x0;	//timer system control register
	TIOS = 0x0;		//what input compare channel you are using / register select
	TCTL2 = 0x0;	//what will you do with the pin
	TFLG1 = 0xFF;	//clear the flag
	TIE = 0x0;		//no interrupts
	
	if(sensor == 'F'){
		HI_time1 = 0;
	}else{
		HI_time0 = 0;
	}
}




void PingCheck(){
	while(1){
		TSCR1 = 0x90;         // enable TCNT count & Fast Flag Clearing
		TSCR2 = 0x04;         // choose TCNT rate at 24MHz/16 = 1.5MHz                 //this will effect the parameters above
		TIOS |= 0x02;         // enable OC1 function
		TCTL2 = 0x0C;         // choose OC1 action to pull high
		TFLG1 = 0xFF;         // cleara all OC flags
		TC1 = TCNT+10;        // wait 10 TCNT counts for pin pull high
		while(TFLG1 & 0x02)   // wait for match to occur
		TC1 = TC1 + 0;        // this will clear the OC1 Flag
		TCTL2 = 0x04;         // set OC1 pin for toggle
		TC1 += PingHiCnt;         // Make the high last as needed
		PingHiorLo = 0;           // Clear the high-low flag to begin
		TIE = 0x02;           // Authorize OC1 interrupts
		asm("cli");
			 
		HILO1_init(); 


		HI_time1 = get_HI_time1();
		
		Pingtime = ( (double) HI_time1) * a;

		Pingdist = ( ( 331.5 + ( 0.6 * 21 ) ) * Pingtime ) * 17;
		
		
		if(Pingdist !=0){
		  return;
		}
	}
    
}




void ParkCheck(){
	while(1){
		TSCR1 = 0x90;         // enable TCNT count & Fast Flag Clearing
		TSCR2 = 0x04;         // choose TCNT rate at 24MHz/16 = 1.5MHz                 //this will effect the parameters above
		TIOS |= 0x01;         // enable OC1 function
		TCTL2 = 0x03;         // choose OC1 action to pull high
		TFLG1 = 0xFF;         // clear all OC flags
		TC0 = TCNT+10;        // wait 10 TCNT counts for pin pull high
		while(TFLG1 & 0x01)   // wait for match to occur
		TC0 = TC0 + 0;        // this will clear the OC1 Flag
		TCTL2 = 0x01;         // set OC1 pin for toggle
		TC0 += ParkHiCnt;         // Make the high last as needed
		ParkHiorLo = 0;           // Clear the high-low flag to begin
		TIE = 0x01;           // Authorize OC1 interrupts
		asm("cli");
			 
		HILO0_init(); 


		HI_time0 = get_HI_time0();
		
		Parktime = ( (double) HI_time0) * a;

		Parkdist = ( ( 331.5 + ( 0.6 * 21 ) ) * Parktime ) * 17;
		
		
		if(Parkdist !=0){
		  return;
		}
	}
    
}




void RunMotorWithLightSensors() {
	lcd_init();
	ad0_enable();


	while(1){
		int ValueLeft;
		int ColorLeft =0;
		int ValueRight;
		int ColorRight =0;

		ValueLeft = ad0conv(6);    // PAD06          //Robots left
		ValueRight = ad0conv(2);   // PAD02          //Robots right
		if(ValueLeft>200){
			ColorLeft = 1;           //black if greater than 200
		}
		if(ValueRight>200){
			ColorRight = 1;          //black if greater than 200
		}
		
		if(ColorLeft == 1 && ColorRight == 1){
			//stop
			StopMoving();
		} else if(ColorLeft == 1 && ColorRight == 0){
			//go right
			AdjustSpeeds(5200, 4500);	//LEFT - RIGHT
		} else if(ColorLeft == 0 && ColorRight == 1){
			//go left
			AdjustSpeeds(4500, 5200);	//LEFT - RIGHT
		} else{//assumed 0 : 0
			//go straight
			goStraight(30);
		}
	}
} 


















void main() {
	int i;
	
  //START PROGRAM WITH A 5 SECOND DELAY
  ms_delay(20000);

  
  
  servo54_init();
  servo76_init();
  lcd_init(); // start the LCD

  ResetEncoder();
  
  
  
  
  
  
  
  //GO STRAIGHT
  InitialSpeed(5200);

  while(1){
		goStraight(10);
		if(GetLeftEncoderTotal() >= selectedEncoderCount || GetRightEncoderTotal() >= selectedEncoderCount){
			break;
		}
		set_lcd_addr(0x00);
		write_long_lcd(GetLeftEncoderTotal());

		set_lcd_addr(0x40);
		write_long_lcd(GetRightEncoderTotal());
  }
	ResetEncoder();
	selectedEncoderCount = 0;
	
	
	
	
	
	//GO BACK
	InitialSpeed(3800);
	
	while(1){
		goStraight(10);
		if(GetLeftEncoderTotal() >= selectedEncoderCount || GetRightEncoderTotal() >= selectedEncoderCount){
			break;
		}
		set_lcd_addr(0x00);
		write_long_lcd(GetLeftEncoderTotal());
		
		set_lcd_addr(0x40);
		write_long_lcd(GetRightEncoderTotal());
    }
	ResetEncoder();
	selectedEncoderCount = 0;
	
	
	
	
	
	
	//GO RIGHT
	AdjustSpeeds(5200, 4500);	//LEFT - RIGHT
	
	while(1){
		turn(10);	//turn by distance
		if(GetLeftEncoderTotal() >= selectedEncoderCount || GetRightEncoderTotal() >= selectedEncoderCount){
			break;
		}
		set_lcd_addr(0x00);
		write_long_lcd(GetLeftEncoderTotal());
		
		set_lcd_addr(0x40);
		write_long_lcd(GetRightEncoderTotal());
    }
	ResetEncoder();
	selectedEncoderCount = 0;
	
	
	
	
	
	
	//GO LEFT
	AdjustSpeeds(4500, 5200);	//LEFT - RIGHT
	
	while(1){
		turn(10);	//turn by distance
		if(GetLeftEncoderTotal() >= selectedEncoderCount || GetRightEncoderTotal() >= selectedEncoderCount){
			break;
		}
		set_lcd_addr(0x00);
		write_long_lcd(GetLeftEncoderTotal());
		
		set_lcd_addr(0x40);
		write_long_lcd(GetRightEncoderTotal());
    }
	ResetEncoder();
	selectedEncoderCount = 0;
	
	
	
	
	
	
	
	//FAST TURN RIGHT
	AdjustSpeeds(5200, 3800);	//LEFT - RIGHT
	
	while(1){
		turn(10);	//turn by distance
		if(GetLeftEncoderTotal() >= (selectedEncoderCount) || GetRightEncoderTotal() >= (selectedEncoderCount)){
			break;
		}
		set_lcd_addr(0x00);
		write_long_lcd(GetLeftEncoderTotal());
		
		set_lcd_addr(0x40);
		write_long_lcd(GetRightEncoderTotal());
    }
	ResetEncoder();
	selectedEncoderCount = 0;
	
	
	
	
	
	
	
	//FAST TURN LEFT
	AdjustSpeeds(3800, 5200);	//LEFT - RIGHT
	
	while(1){
		turn(10);	//turn by distance
		if(GetLeftEncoderTotal() >= (selectedEncoderCount) || GetRightEncoderTotal() >= (selectedEncoderCount)){
			break;
		}
		set_lcd_addr(0x00);
		write_long_lcd(GetLeftEncoderTotal());
		
		set_lcd_addr(0x40);
		write_long_lcd(GetRightEncoderTotal());
    }
	ResetEncoder();
	selectedEncoderCount = 0;
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	//run the robot within the lines until both sensors go on the black line
	RunMotorWithLightSensors();
	
	
	
	
/* 	//LOOK AHEAD UNTIL VALUE IS LESS THAN OR EQUAL TO 20 CM & THEN BREAK OUT OF LOOP & CONTINUE OPERATIONS
	while(1){
		PingCheck();
		set_lcd_addr(0x00);
		write_long_lcd((long)Pingdist);
		if(Pingdist <= 20){
			break;
		}
	}
	ResetEncoder(); */
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
/* 	ResetUltraSoundSensor('F');
	
	
	
	//LOOK FOR EMPTY PARKING SPOTS UNTIL VALUE IS LESS THAN OR EQUAL TO 20 CM & THEN BREAK OUT OF LOOP & CONTINUE OPERATIONS
	while(1){
		ParkCheck();
		set_lcd_addr(0x0);
		write_long_lcd((long)Parkdist);
		if(Parkdist <= 20){
			break;
		}
	}
	ResetEncoder();
	
	
	
	
	ResetUltraSoundSensor('U'); */
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	asm("swi");
	
	
    
}

















































void GoStraight(NEW){        //encoder number each start at 1 ticks
      encoderRotationCountL = 0;
      encoderRotationCountR = 0;
      
      //ms_delay(10000);
      
      //set_servo54(5000);
			//set_servo76(5000);

      DDRT =  0xF3; //init PT2 & PT3 as input
      ICPAR = 0xFC; //enable PT2 & PT3 as pulse accum
      TCTL4 = 0x55; //set to capture rising edge                   ////potentially a problem 
      countR=~countR + 1; //2's comp of count value
      countL=~countL + 1; //2's comp of count value
     
      PACN3=countL; //load count into Pulse Acc reg
      PACN2=countR; //load count into Pulse Acc reg

            
	for(x=0;x<NEW;x++){

		   if(PACN3 > CompareCheck || PACN2 > CompareCheck){
			    PACN3 = PACN3-80;
			    PACN2 = PACN2-80;
		   }
		   
			 set_lcd_addr(0x00);
			 write_int_lcd(PACN3);
			 set_lcd_addr(0x40);
			 write_int_lcd(PACN2);
			 
			 
			 if(PACN3 == 0){
			    encoderRotationCountL++;
			 }
			 if(PACN2 == 0){
			    encoderRotationCountR++;
			 }
			 
			 if(encoderRotationCountL > encoderRotationCountR){    //speed up the right motor
			    if(PACN3 > PACN2){     
			       diff = 180*(encoderRotationCountL - encoderRotationCountR) + (PACN3 - 0) - (180 - PACN2);
			    } else if(PACN3 < PACN2){
			       diff = 180*(encoderRotationCountL - encoderRotationCountR) + (PACN3 - 0) + (180 - PACN2);
			    } else{
			       diff = 180*(encoderRotationCountL - encoderRotationCountR);
			    }
			    
			    //diff = (PACN3 - PACN2);    
			    
			    
				  newSpeedR = (diff*(motorR -4500)/90);    //increase speed
				  newSpeedL = motorL; 

				  set_servo54(newSpeedL);
				  set_servo76(newSpeedR);
          
          
          
			 } else if(encoderRotationCountL < encoderRotationCountR){    //speed up the left motor
		        if(PACN3 > PACN2){     
			       diff = 180*(encoderRotationCountR - encoderRotationCountL) + (PACN3 - 0) - (180 - PACN2);
			    } else if(PACN3 < PACN2){
			       diff = 180*(encoderRotationCountR - encoderRotationCountL) + (PACN3 - 0) + (180 - PACN2);
			    } else{
			       diff = 180*(encoderRotationCountR - encoderRotationCountL);
			    }
		      
		      
		      //diff = (PACN3 - PACN2);    //left - right
		      
		      
				  newSpeedL = (diff*(motorL -4500)/90);    //increase speed
				  newSpeedR = motorR;

				  set_servo54(newSpeedL);
				  set_servo76(newSpeedR);
          
          
          
			 } else{   //the encoders have the same loop count so compare within the 180 degrees
			    diff = (PACN3 - PACN2);    //left - right
  		  
  				 
          if(diff < 0){
            newSpeedL = (diff*(motorL -4500)/90);    //increase speed
            newSpeedR = motorR;

            set_servo54(newSpeedL);
            set_servo76(newSpeedR);


          }else if(diff > 0){
            newSpeedR = (diff*(motorR -4500)/90);    //increase speed
            newSpeedL = motorL; 

            set_servo54(newSpeedL);
            set_servo76(newSpeedR);


          }else {

            newSpeedL = motorL;
            newSpeedR = motorR;

            set_servo54(newSpeedL);
            set_servo76(newSpeedR);


          }
			 }      
	}     
}



int PingSensor (Ping) { 

  
  
    
    while(Ping){
      
      
    TSCR1 = 0x90;         // enable TCNT count & Fast Flag Clearing
    TSCR2 = 0x04;         // choose TCNT rate at 24MHz/16 = 1.5MHz                 //this will effect the parameters above
    TIOS |= 0x02;         // enable OC1 function
    TCTL2 = 0x0C;         // choose OC1 action to pull high
    TFLG1 = 0xFF;         // clear all OC flags
    TC1 = TCNT+10;        // wait 10 TCNT counts for pin pull high
    while(TFLG1 & 0x02)   // wait for match to occur
    TC1 = TC1 + 0;        // this will clear the OC1 Flag
    TCTL2 = 0x04;         // set OC1 pin for toggle
    TC1 += HiCnt;         // Make the high last as needed
    HiorLo = 0;           // Clear the high-low flag to begin
    TIE = 0x02;           // Authorize OC1 interrupts
    asm("cli");
         
    HILO1_init(); 


    HI_time1 = get_HI_time1();
    
    time = ( (double) HI_time1) * a;

    dist = ( ( 331.5 + ( 0.6 * 21 ) ) * time ) * 17;
    
    
    //ms_delay(Ping);
    
    if(dist !=0){
      return dist;
    
    }
    
    }
}









void turnLeft(int degrees, char direction){			//example of calling the function : turnLeft(180, 'F');	//'B' is backwards
	wheelDistance = (((((degrees*PI/180)*r)/(2*PI))*(180/PI))/180);	//distance the wheel should turn
	
	if(direction == 'F'){//go forwards
		t = fullSpeedForwardTime;
		newSpeedL = 5700;
		newSpeedR = 5700;
	} else{//go backwards
		t = fullSpeedBackwardTime;
		newSpeedL = 3300;
		newSpeedR = 3300;
	}
	iterationCount = (int)(wheelDistance/t);	//this number represents the amount of iterations must occur for the wheel to turn the amount of degrees exactly specified
	
	for(i = 0; i < iterationCount; i++){
		set_servo54(newSpeedL);
		set_servo76(newSpeedR);
	}
  //set_servo54(5200);
  //set_servo76(3950);
  //ms_delay(NEW);
}

void turnRight(NEW){
  set_servo54(3400);
  set_servo76(5700);
  ms_delay(NEW);
}


void stop (NEW){
  set_servo54(4500);
  set_servo76(4500);
  ms_delay(NEW);
}

void END (NEW){
while(NEW){
  
  set_servo54(4500);
  set_servo76(4500);
}
}
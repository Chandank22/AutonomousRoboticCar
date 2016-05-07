/*
  ECE375 Kyle, Max, Tom
*/

#include  <hidef.h>      /* common defines and macros */
#include    <stdio.h>
#include    <mc9s12dg256.h>
#pragma LINK_INFO DERIVATIVE "mc9s12dg256b"
#pragma CODE_SEG __NEAR_SEG NON_BANKED

void interrupt 9 handler1() {
 HILOtimes1();
}

void interrupt 8 handler0() {
 HILOtimes0();
}

//initialize variables
double a = 0.0000015;		//period in seconds for the 1.5 MHz clock


void goStraight(int dis);	//forward-declared method
void turn(int dis);		//forward-declared method
int PingSensor(int);	//forward-declared method
void TurnLeft(int);	//forward-declared method
void TurnRight(int);	//forward-declared method
void Stop (int);	//forward-declared method
void END (int);	//forward-declared method


//initialize variables
static  long leftAccumulator = 0;    // total counts from Right Encoder
static  long rightAccumulator = 0;   // total counts from Left Encoder

void  ResetEncoder() {
	DDRT  &= 0xF3;			//init PT2 & PT3 as input
	ICPAR = 0xFC;			//enable PT2 & PT3 as pulse accum
	TCTL4 = 0x50;			//set to capture rising edge

	leftAccumulator = 0;  
	rightAccumulator = 0;
	
	//reset accumulators
	PACN3 = 0;
	PACN2 = 0; 
}

//initialize variables
long selectedEncoderCount;

void ClearEncoders() {		//reset accumulators
	leftAccumulator = 0;  
	rightAccumulator = 0;

	PACN3 = 0;
	PACN2 = 0; 
	
	//set the counter to 0
	selectedEncoderCount = 0;
}

//initialize variables
#define ACCUM_MAX_THRESHOLD 180     // value used to know when to move value into s/w counter

void  UpdateEncoderTotals() {
	// increment S/W accumulators before overflow occurs
	if( PACN3 >= ACCUM_MAX_THRESHOLD ) {
		leftAccumulator =  leftAccumulator + PACN3;
		PACN3 = 0;
	}
	if( PACN2 >= ACCUM_MAX_THRESHOLD ) {
		rightAccumulator =  rightAccumulator + PACN2;
		PACN2 = 0;
	}
}

long GetLeftEncoderTotal() {
  return( leftAccumulator + PACN3 ); 
}

long GetRightEncoderTotal() {
  return( rightAccumulator + PACN2 ); 
}

//initialize variables
static  long  motorSpeed;    // current motor speeds
static  long  lAdjustment;
static  long  rAdjustment;
#define STOPPED_SPEED (4500)
#define MAX_FORWARD_SPEED (5700)
#define MAX_REVERSE_SPEED (3300)

void  InitialSpeed( long speed ) {
  motorSpeed = speed;  
  set_servo54(motorSpeed);
  set_servo76(motorSpeed);
}

void  AdjustSpeeds( long lSpeed, long rSpeed ) {
  set_servo54(lSpeed);
  set_servo76(rSpeed);
}

void  StopMoving( ) {
  set_servo54(STOPPED_SPEED);
  set_servo76(STOPPED_SPEED);
}

void  SpeedAdjust( ) {		//this method is a control loop that adjusts the speed and encoder counts of the robot
  long  deltaCount;
  
  UpdateEncoderTotals();
  deltaCount = GetLeftEncoderTotal() - GetRightEncoderTotal();		//this represents the difference

  if( motorSpeed > STOPPED_SPEED ) {
    
    lAdjustment = motorSpeed + (4 * deltaCount);	//adjust the speed by the addition of the current motor speed & the difference multiplied by the gain
    rAdjustment = motorSpeed - (4 * deltaCount);	//adjust the speed by the subtraction of the current motor speed & the difference multiplied by the gain
    
    if( lAdjustment > MAX_FORWARD_SPEED )
      lAdjustment = MAX_FORWARD_SPEED;
    if( rAdjustment > MAX_FORWARD_SPEED )
      rAdjustment = MAX_FORWARD_SPEED;
    if( lAdjustment < STOPPED_SPEED )
      lAdjustment = STOPPED_SPEED;
    if( rAdjustment < STOPPED_SPEED )
      rAdjustment = STOPPED_SPEED;
  } 
  else {
    lAdjustment = motorSpeed - (4 * deltaCount);	//adjust the speed by the subtraction of the current motor speed & the difference multiplied by the gain
    rAdjustment = motorSpeed + (4 * deltaCount);	//adjust the speed by the addition of the current motor speed & the difference multiplied by the gain
    
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

//initialize variables
#define ONE_ENCODER_TICK_DISTANCE 0.36564648 //in centi-meters

void goStraight(int dis){				//go straight forward with the robot and the distance in centi-meters as a parameter
	
	
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

void turn(int dis){					//turn the robot and the distance in centi-meters as a parameter
	selectedEncoderCount = ((dis/ONE_ENCODER_TICK_DISTANCE));
	if(GetLeftEncoderTotal() >= selectedEncoderCount || GetRightEncoderTotal() >= selectedEncoderCount){
		StopMoving();
	}
}

int sensorChange = 0;   // needs to be set when a new ultrasound sensor is used to prevent a race condition
void SensorSwitch() 
{
  sensorChange = 1;
}

//initialize variables
int HI_time1; // High time (pulse width)
char  pingHiorLo;
#define PingHiCnt 3       // high pulse duration in TCNT counts
#define PingLoCnt 500      // low signal between pulses time in TCNT Counts
double pingtime;
double pingdist;

void PingCheck(){
	while(1){
		TSCR1 = 0x90;         // enable TCNT count & Fast Flag Clearing
		TSCR2 = 0x04;         // choose TCNT rate at 24MHz/16 = 1.5MHz                 //this will effect the parameters above
		TIOS = 0x02;         // enable OC1 function
		TCTL2 = 0x0C;         // choose OC1 action to pull high
		TFLG1 = 0xFF;         // cleara all OC flags
		TC1 = TCNT+10;        // wait 10 TCNT counts for pin pull high
		while(TFLG1 & 0x02);   // wait for match to occur
		
		TFLG1 =  TFLG1 | 0x02;
		if( sensorChange != 0 )
		{
		  ms_delay(1);
		  sensorChange = 0;
		}
		
		//TC1 = TC1 + 0;        // this will clear the OC1 Flag
		TCTL2 = 0x04;         // set OC1 pin for toggle
		TC1 += PingHiCnt;         // Make the high last as needed
		pingHiorLo = 0;           // Clear the high-low flag to begin
		TIE = 0x02;           // Authorize OC1 interrupts
		asm("cli");
			 
		HILO1_init(); 


		HI_time1 = get_HI_time1();
		
		pingtime = ( (double) HI_time1) * a;

		pingdist = ( ( 331.5 + ( 0.6 * 21 ) ) * pingtime ) * 17;
		
		
		if(pingdist > 0){
		  return;
		}
	}
    
}

//initialize variables
int HI_time0; // High time (pulse width)
char  parkHiorLo;
#define ParkHiCnt 3       // high pulse duration in TCNT counts
#define ParkLoCnt 500      // low signal between pulses time in TCNT Counts 
double parktime;
double parkdist;


void ParkCheck(){
	while(1){
		TSCR1 = 0x90;         // enable TCNT count & Fast Flag Clearing
		TSCR2 = 0x04;         // choose TCNT rate at 24MHz/16 = 1.5MHz                 //this will effect the parameters above
		TIOS = 0x01;         // enable OC1 function
		TCTL2 = 0x03;         // choose OC1 action to pull high
		TFLG1 = 0xFF;         // clear all OC flags
		TC0 = TCNT+10;        // wait 10 TCNT counts for pin pull high
		while(TFLG1 & 0x01);   // wait for match to occur
		
		TFLG1 =  TFLG1 | 0x01;
		if( sensorChange != 0 )
		{
		  ms_delay(1);
		  sensorChange = 0;
		}
		
		//TC0 = TC0 + 0;        // this will clear the OC1 Flag
		TCTL2 = 0x01;         // set OC1 pin for toggle
		TC0 += ParkHiCnt;         // Make the high last as needed
		parkHiorLo = 0;           // Clear the high-low flag to begin
		TIE = 0x01;           // Authorize OC1 interrupts
		asm("cli");
			 
		HILO0_init(); 


		HI_time0 = get_HI_time0();
		
		parktime = ( (double) HI_time0) * a;

		parkdist = ( ( 331.5 + ( 0.6 * 21 ) ) * parktime ) * 17;
		
		
		if(parkdist > 0){
		  return;
		}
	}
    
}


//a method to set the speeds of both motors to travel forward a set distance
int MoveForward(int speed1, int speed2, int dis){
	
	AdjustSpeeds(speed1, speed2);

	  while(1){
			turn(dis);	//turn by distance
			if(GetLeftEncoderTotal() >= selectedEncoderCount || GetRightEncoderTotal() >= selectedEncoderCount){
				break;
			}
			set_lcd_addr(0x00);
			write_long_lcd(GetLeftEncoderTotal());

			set_lcd_addr(0x40);
			write_long_lcd(GetRightEncoderTotal());
	  }
		ClearEncoders();
		selectedEncoderCount = 0;
}

//a method to set the speeds of both motors to travel backward a set distance
int MoveBackward(int speed1, int speed2, int dis){
	AdjustSpeeds(speed1, speed2);

	  while(1){
			turn(dis);	//turn by distance
			if(GetLeftEncoderTotal() >= selectedEncoderCount || GetRightEncoderTotal() >= selectedEncoderCount){
				break;
			}
			set_lcd_addr(0x00);
			write_long_lcd(GetLeftEncoderTotal());

			set_lcd_addr(0x40);
			write_long_lcd(GetRightEncoderTotal());
	  }
		ClearEncoders();
		selectedEncoderCount = 0;
}

//a method that is hard-coded to backup at an already set speed that is slow for a user-specified distance
int Backup(int dist){
	InitialSpeed(3800);
	while(1){
		goStraight(dist);
		if(GetLeftEncoderTotal() >= selectedEncoderCount || GetRightEncoderTotal() >= selectedEncoderCount){
			break;
		}
		set_lcd_addr(0x00);
		write_long_lcd(GetLeftEncoderTotal());
		
		set_lcd_addr(0x40);
		write_long_lcd(GetRightEncoderTotal());
    }
	ClearEncoders();
	selectedEncoderCount = 0;
}

//initialize variables
int ValueLeft;		//left line sensor
int ValueRight;		//right line sensor
int fsm;			//this represents the past state of the 2 line sensors (LEFT --- RIGHT) : WW is 0     BW is 1     WB is 2     BB is 3
int forwardState;	//this represents the current state of the robot in terms of :   WW is 1      and      anything else is 0

void RunMotorAndAlignSensors() {
	ad0_enable();
	InitialSpeed(4800);
	fsm = 0;	//robot is reading left W and right W
	forwardState = 1;
	while(1){
		while(1){
			ValueLeft = ad0conv(6);    // PAD06          //Robots left
			ValueRight = ad0conv(2);   // PAD02          //Robots right
			set_lcd_addr(0x00);
			write_int_lcd(ValueLeft);
			set_lcd_addr(0x40);
			write_int_lcd(ValueRight);
			
			
			
			if(forwardState == 1){//is forward mode still enabled
				if(ValueLeft>250 && ValueRight>250){
					//clear the encoders
					ClearEncoders();
					forwardState = 0;//forward mode disabled
					//check the last state
					
					//break out of everything
					fsm = 3;	//robot is reading left B and right B
					break;	//stop moving
				}else if(ValueLeft>250 && ValueRight<250){
					//clear the encoders
					ClearEncoders();
					MoveBackward(3800, 3800, 8);
					//clear the encoders
					ClearEncoders();
					//add delay to prevent reading old adc values
					ms_delay(100);
					//go right & MOVE FORWARD
					AdjustSpeeds(4850, 4700);
					forwardState = 0;//forward mode disabled
					//check the last state
					
					fsm = 1;	//robot is reading left B and right W
					
				}else if(ValueLeft<250 && ValueRight>250){
					//clear the encoders
					ClearEncoders();
					MoveBackward(3800, 3800, 8);
					//clear the encoders
					ClearEncoders();
					//add delay to prevent reading old adc values
					ms_delay(100);
					//go left & MOVE FORWARD
					AdjustSpeeds(4700, 4850);
					forwardState = 0;//forward mode disabled
					//check the last state
					
					fsm = 2;	//robot is reading left W and right B
					
				}else{
					if(fsm == 3){
						//clear the encoders
						ClearEncoders();
						//stop
						StopMoving();
						fsm = 3;	//robot is reading left B and right B
						break;
					} else if(fsm == 0){
						//go straight
						AdjustSpeeds(4800, 4800);	//--------------------------right - left
						SpeedAdjust();	//adjust the speed
						fsm = 0;	//robot is reading left W and right W
						break;
					} else if(fsm == 1){
						//clear the encoders
						ClearEncoders();
						//add delay to prevent reading old adc values
						ms_delay(100);
						//go right & MOVE FORWARD
						AdjustSpeeds(4850, 4700);
						fsm = 1;	//robot is reading left B and right W
						break;
					} else if(fsm == 2){
						//clear the encoders
						ClearEncoders();
						//add delay to prevent reading old adc values
						ms_delay(100);
						//go left & MOVE FORWARD
						AdjustSpeeds(4700, 4850);
						fsm = 2;	//robot is reading left W and right B
						break;
					}
				}
			}else{//is forward mode disabled
				if(ValueLeft>250 && ValueRight>250){
					//clear the encoders
					ClearEncoders();
					forwardState = 0;//forward mode disabled
					//check the last state
					
					//break out of everything
					fsm = 3;	//robot is reading left B and right B
					break;	//stop moving
				}else if(ValueLeft>250 && ValueRight<250){
					//clear the encoders
					ClearEncoders();
					MoveBackward(3800, 3800, 8);
					//clear the encoders
					ClearEncoders();
					//add delay to prevent reading old adc values
					ms_delay(100);
					//go right & MOVE FORWARD
					AdjustSpeeds(4850, 4700);
					forwardState = 0;//forward mode disabled
					//check the last state
					
					fsm = 1;	//robot is reading left B and right W
					
				}else if(ValueLeft<250 && ValueRight>250){
					//clear the encoders
					ClearEncoders();
					MoveBackward(3800, 3800, 8);
					//clear the encoders
					ClearEncoders();
					//add delay to prevent reading old adc values
					ms_delay(100);
					//go left & MOVE FORWARD
					AdjustSpeeds(4700, 4850);
					forwardState = 0;//forward mode disabled
					//check the last state
					
					fsm = 2;	//robot is reading left W and right B
					
				}
			}
		}
		if((ValueLeft>250 && ValueRight>250) || (fsm == 3)){
			break;
		}
	}
	//stop
	StopMoving();
	//clear the encoders
	ClearEncoders();
} 

void RunBoulevard(char startPos){//Enter a start position EX : 'L'  'R'
	//check the front sensors to be on the black line OR CHECK FOR ANOTHER OBSTACLE
	MovingForwardRunAllFrontSensors();//if it stops then there is no obstacle in the way
	//backup 5 cm
	Backup(5);
	
	if(startPos == 'L'){
		//turn FAST right 90 degrees
		TurnRight(25);
		
		//SensorSwitch();	//set a delay to prevent ping sensor race condition
		//drive to the line and align the robot with the line
		RunMotorAndAlignSensors();
		
		//backup 7 cm
		Backup(7);
		
		//turn FAST left 90 degrees
		TurnLeft(24);
		
		ms_delay(100);
		
		//drive to the line & check both line sensors to be on the black line OR CHECK FOR ANOTHER OBSTACLE
		SensorSwitch();	//set a delay to prevent ping sensor race condition
		MovingForwardRunAllFrontSensors();
		
		//backup 5 cm
		Backup(5);
		
		//turn FAST left 90 degrees
		TurnLeft(28);
		
		//drive to the line and align the robot with the line
		RunMotorAndAlignSensors();
		
		//backup 7 cm
		Backup(7);
		
		//turn FAST right 90 degrees
		TurnRight(23);
		
		//drive to the line & check both line sensors to be on the black line OR CHECK FOR ANOTHER OBSTACLE
		SensorSwitch();	//set a delay to prevent ping sensor race condition
		MovingForwardRunAllFrontSensors();
		
		//backup 7 cm
		Backup(7);
		
		//drive to the line and align the robot with the line
		RunMotorAndAlignSensors();
		
		//backup 7 cm
		Backup(7);
		
		//turn FAST left 90 degrees
		TurnLeft(24);
	}else{//Right is assumed <---> 'R'
		//turn FAST left 90 degrees
		TurnLeft(25);
		
		//SensorSwitch();	//set a delay to prevent ping sensor race condition
		//drive to the line and align the robot with the line
		RunMotorAndAlignSensors();
		
		//backup 7 cm
		Backup(7);
		
		//turn FAST right 90 degrees
		TurnRight(24);
		
		ms_delay(100);
		
		//drive to the line & check both line sensors to be on the black line OR CHECK FOR ANOTHER OBSTACLE
		SensorSwitch();	//set a delay to prevent ping sensor race condition
		MovingForwardRunAllFrontSensors();
		
		//backup 5 cm
		Backup(5);
		
		//turn FAST right 90 degrees
		TurnRight(28);
		
		//drive to the line and align the robot with the line
		RunMotorAndAlignSensors();
		
		//backup 7 cm
		Backup(7);
		
		//turn FAST left 90 degrees
		TurnLeft(23);
		
		//drive to the line & check both line sensors to be on the black line OR CHECK FOR ANOTHER OBSTACLE
		SensorSwitch();	//set a delay to prevent ping sensor race condition
		MovingForwardRunAllFrontSensors();
		
		//backup 7 cm
		Backup(7);
		
		//drive to the line and align the robot with the line
		RunMotorAndAlignSensors();
		
		//backup 7 cm
		Backup(7);
		
		//turn FAST left 90 degrees
		TurnLeft(24);
	}
}

void RunPath1(){
	//adjust the robots orientation
	Adjust(5250, 4500, 1);//--------------------------right - left
	
	//drive to the line & check both line sensors to be on the black line OR CHECK FOR ANOTHER OBSTACLE
	SensorSwitch();	//set a delay to prevent ping sensor race condition
	MovingForwardRunAllFrontSensors();
	
	//adjust the robots orientation
	Adjust(3800, 4500, 2);//--------------------------right - left
	
	//backup 7 cm
	Backup(7);
	
	//adjust the robots orientation
	Adjust(4500, 3800, 1);//--------------------------right - left
	
	//drive to the line and align the robot with the line
	RunMotorAndAlignSensors();
	
	//backup 7 cm
	Backup(7);
	
	//turn FAST left 90 degrees
	TurnLeft(24);
}

void RunPath2(){
	//adjust the robots orientation
	Adjust(5250, 4500, 1);//--------------------------right - left
	
	//drive to the line & check both line sensors to be on the black line OR CHECK FOR ANOTHER OBSTACLE
	SensorSwitch();	//set a delay to prevent ping sensor race condition
	MovingForwardRunAllFrontSensors();
	
	//adjust the robots orientation
	Adjust(3800, 4500, 2);//--------------------------right - left
	
	//backup 5 cm
	Backup(5);
	
	//adjust the robots orientation
	Adjust(4500, 3800, 1);//--------------------------right - left
	
	//drive to the line and align the robot with the line
	RunMotorAndAlignSensors();
	
	//backup 15 cm
	Backup(15);
	
	//adjust the robots orientation
	Adjust(5250, 4500, 25);//--------------------------right - left
	ms_delay(100);
	//adjust the robots orientation
	Adjust(4500, 3800, 20);//--------------------------right - left
	ms_delay(100);
}

void RunPath3(){
	//drive to the line
	RunLightSensors();
	ms_delay(100);
	
	//adjust the robots orientation
	Adjust(4500, 3800, 2);//--------------------------right - left
	
	//backup 5 cm
	Backup(5);
	
	//adjust the robots orientation
	Adjust(3800, 4500, 1);//--------------------------right - left
	
	//drive to the line and align the robot with the line
	RunMotorAndAlignSensors();
	ms_delay(100);

	//adjust the robots orientation
	Adjust(3800, 4500, 22);//--------------------------right - left
	ms_delay(100);
	//adjust the robots orientation
	Adjust(4500, 5250, 22);//--------------------------right - left
	ms_delay(100);
}

//initialize variables
int counterCheck;
int parkingSpot1;
int parkingSpot2;
int parkingSpot3;
int parkingSpot4;
int rightTurns;
int leftTurns;

void RunParkingLot(){
	
	parkingSpot1 = 0;
	parkingSpot2 = 0;
	parkingSpot3 = 0;
	parkingSpot4 = 0;
	
	ad0_enable();
	
	while(1){
		rightTurns = 0;
		leftTurns = 0;
		ms_delay(1000);
		SensorSwitch();	//set a delay to prevent ping sensor race condition
		//go forward 57 cm
		MoveForward(4800, 4800, 52);
		//check 1st parking spot
		for(counterCheck = 0; counterCheck < 75; counterCheck++){
			ValueLeft = ad0conv(6);    // PAD06          //Robots left
			ValueRight = ad0conv(2);   // PAD02          //Robots right
			
			if(ValueLeft>250 && ValueRight<250){
				//go right
				Adjust(4500, 5000, 1);
				rightTurns++;
			}
			if(ValueLeft<250 && ValueRight>250){
				//go left
				Adjust(5000, 4500, 1);
			}
			
			ParkCheck();
			set_lcd_addr(0x00);
			write_long_lcd((long)parkdist);
			
			if(parkdist <= 35 && counterCheck == 74){
				//object present
				parkingSpot1 = 1;
				break;
			}
			
		}
		
		if(rightTurns%2 == 1 && rightTurns > 0){
			leftTurns = rightTurns/2 + 1;
			//go left
			Adjust(5200, 4500, leftTurns);
		} else if(rightTurns%2 == 0 && rightTurns > 0){
			leftTurns = rightTurns/2;
			//go left
			Adjust(5200, 4500, leftTurns);
		}
		
		
		
		if(parkingSpot1 == 0){//park because no object is present
			ParallelPark();
			break;
		}
		rightTurns = 0;
		leftTurns = 0;
		ms_delay(1000);
		SensorSwitch();	//set a delay to prevent ping sensor race condition
		//go forward 45 cm
		MoveForward(4800, 4800, 45);
		
		//check 1st parking spot
		for(counterCheck = 0; counterCheck < 75; counterCheck++){
			ValueLeft = ad0conv(6);    // PAD06          //Robots left
			ValueRight = ad0conv(2);   // PAD02          //Robots right

			if(ValueLeft>250 && ValueRight<250){
				//go right
				Adjust(4500, 5000, 1);
				rightTurns++;
			}
			if(ValueLeft<250 && ValueRight>250){
				//go left
				Adjust(5000, 4500, 1);
			}
			
			ParkCheck();
			set_lcd_addr(0x00);
			write_long_lcd((long)parkdist);
			
			if(parkdist <= 35 && counterCheck == 74){
				//object present
				parkingSpot2 = 1;
				break;
			}
			
		}
		
		if(rightTurns%2 == 1 && rightTurns > 0){
			leftTurns = rightTurns/2 + 1;
			//go left
			Adjust(5200, 4500, leftTurns);
		} else if(rightTurns%2 == 0 && rightTurns > 0){
			leftTurns = rightTurns/2;
			//go left
			Adjust(5200, 4500, leftTurns);
		}
		
		if(parkingSpot2 == 0){//park because no object is present
			ParallelPark();
			break;
		}
		//go left
		Adjust(5200, 4500, 1);
		
		rightTurns = 0;
		leftTurns = 0;
		ms_delay(1000);
		SensorSwitch();	//set a delay to prevent ping sensor race condition
		//go forward 45 cm
		MoveForward(4800, 4800, 45);
		//check 1st parking spot
		for(counterCheck = 0; counterCheck < 75; counterCheck++){
			ValueLeft = ad0conv(6);    // PAD06          //Robots left
			ValueRight = ad0conv(2);   // PAD02          //Robots right

			if(ValueLeft>250 && ValueRight<250){
				//go right
				Adjust(4500, 5000, 1);
				rightTurns++;
			}
			if(ValueLeft<250 && ValueRight>250){
				//go left
				Adjust(5000, 4500, 1);
			}
			
			ParkCheck();
			set_lcd_addr(0x00);
			write_long_lcd((long)parkdist);
			
			if(parkdist <= 35 && counterCheck == 74){
				//object present
				parkingSpot3 = 1;
				break;
			}
			
		}
		
		if(rightTurns%2 == 1 && rightTurns > 0){
			leftTurns = rightTurns/2 + 1;
			//go left
			Adjust(5200, 4500, leftTurns);
		} else if(rightTurns%2 == 0 && rightTurns > 0){
			leftTurns = rightTurns/2;
			//go left
			Adjust(5200, 4500, leftTurns);
		}
		
		if(parkingSpot3 == 0){//park because no object is present
			ParallelPark();
			break;
		}
		rightTurns = 0;
		leftTurns = 0;
		ms_delay(1000);
		SensorSwitch();	//set a delay to prevent ping sensor race condition
		//go forward 45 cm
		MoveForward(4800, 4800, 45);

		//check 1st parking spot
		for(counterCheck = 0; counterCheck < 75; counterCheck++){
			ValueLeft = ad0conv(6);    // PAD06          //Robots left
			ValueRight = ad0conv(2);   // PAD02          //Robots right

			if(ValueLeft>250 && ValueRight<250){
				//go right
				Adjust(4500, 5000, 1);
				rightTurns++;
			}
			if(ValueLeft<250 && ValueRight>250){
				//go left
				Adjust(5000, 4500, 1);
			}
			
			ParkCheck();
			set_lcd_addr(0x00);
			write_long_lcd((long)parkdist);
			
			if(parkdist <= 35 && counterCheck == 74){
				//object present
				parkingSpot4 = 1;
				break;
			}
		}
		
		if(rightTurns%2 == 1 && rightTurns > 0){
			leftTurns = rightTurns/2 + 1;
			//go left
			Adjust(5200, 4500, leftTurns);
		} else if(rightTurns%2 == 0 && rightTurns > 0){
			leftTurns = rightTurns/2;
			//go left
			Adjust(5200, 4500, leftTurns);
		}
		
		if(parkingSpot4 == 0){//park because no object is present
			ParallelPark();
			break;
		}
		break;
	}

	//clear the encoders
	ClearEncoders();
}

int ParallelPark(){
	//go forward 28 cm
	MoveForward(4800, 4800, 28);
	//clear the encoders
	ClearEncoders();
	//start backing into the parking spot
	MoveBackward(4300, 3800, 66);
	//clear the encoders
	ClearEncoders();
	MoveBackward(3800, 3800, 5);
	//clear the encoders
	ClearEncoders();
	//turn the robot in the parking spot to finish
	TurnRight(26);
}

//initialize variables
int toggle;
int disable;

int RunLightSensorsAdjustRobot(){
	disable = 0;
	toggle = 0;
	InitialSpeed(4800);
	ad0_enable();
	while(1){
		while(1){
			ValueLeft = ad0conv(6);    // PAD06          //Robots left
			ValueRight = ad0conv(2);   // PAD02          //Robots right
			
			SpeedAdjust();	//adjust the speed
			
			if(toggle == 1 && ValueLeft<250 && ValueRight<250){
				//go straight
				StopMoving();
				disable = 1;
				break;
			} else if(ValueLeft>250 && ValueRight<250){
				//clear the encoders
				ClearEncoders();
				//stop
				StopMoving();
				toggle = 1;
				break;
			} else if(ValueLeft<250 && ValueRight>250){
				//clear the encoders
				ClearEncoders();
				//stop
				StopMoving();
				toggle = 1;
				break;
			} else if(ValueLeft>250 && ValueRight>250){
				//stop
				StopMoving();
				toggle = 1;
				break;
			} else if(toggle == 0 && ValueLeft<250 && ValueRight<250){
				//go straight
				AdjustSpeeds(4900, 4900);	//--------------------------right - left
				SpeedAdjust();	//adjust the speed
				break;
			}
			
			
		}
		if(disable == 1){
			//stop
			StopMoving();
			break;
		}
		if(ValueLeft>250 && ValueRight<250){
			//go left
			AdjustSpeeds(4500, 5250);	//--------------------------right - left
			
		}
		if(ValueLeft<250 && ValueRight>250){
			//go right
			
			AdjustSpeeds(5250, 4500);	//--------------------------right - left
		}
		if(ValueLeft>250 && ValueRight>250){
			//stop
			StopMoving();
			break;
		}
	}
	//clear the encoders
	ClearEncoders();
}

int RunLightSensors(){
	InitialSpeed(4800);
	ad0_enable();
	while(1){
		while(1){
			ValueLeft = ad0conv(6);    // PAD06          //Robots left
			ValueRight = ad0conv(2);   // PAD02          //Robots right
			
			SpeedAdjust();	//adjust the speed
			
			if(ValueLeft>250 && ValueRight>250){
				//stop
				StopMoving();
				break;
			} else if(ValueLeft>250 && ValueRight<250){
				//clear the encoders
				ClearEncoders();
				//stop
				StopMoving();
				break;
			} else if(ValueLeft<250 && ValueRight>250){
				//clear the encoders
				ClearEncoders();
				//stop
				StopMoving();
				break;
			} else{//assumed 0 : 0
				//go straight
				AdjustSpeeds(4900, 4900);	//--------------------------right - left
				SpeedAdjust();	//adjust the speed
				break;
			}
			
			
		}
		if(ValueLeft>250 && ValueRight<250){
			//go left
			AdjustSpeeds(4500, 5250);	//--------------------------right - left
		}
		if(ValueLeft<250 && ValueRight>250){
			//go right
			AdjustSpeeds(5250, 4500);	//--------------------------right - left
		}
		if(ValueLeft>250 && ValueRight>250){
			//stop
			StopMoving();
			break;
		}
	}
	//clear the encoders
	ClearEncoders();
}

int MovingForwardRunAllFrontSensors(){
	InitialSpeed(4800);
	ad0_enable();
	while(1){
		while(1){
			PingCheck();
			set_lcd_addr(0x00);
			write_long_lcd((long)pingdist);
			if(pingdist <= 20){
				StopMoving();
				break;
			}
			
			ValueLeft = ad0conv(6);    // PAD06          //Robots left
			ValueRight = ad0conv(2);   // PAD02          //Robots right
			
			SpeedAdjust();	//adjust the speed
			
			if(ValueLeft>250 && ValueRight>250){
				//stop
				StopMoving();
				break;
			} else if(ValueLeft>250 && ValueRight<250){
				//clear the encoders
				ClearEncoders();
				//stop
				StopMoving();
				break;
			} else if(ValueLeft<250 && ValueRight>250){
				//clear the encoders
				ClearEncoders();
				//stop
				StopMoving();
				break;
			} else{//assumed 0 : 0
				//go straight
				AdjustSpeeds(4900, 4900);	//--------------------------right - left
				SpeedAdjust();	//adjust the speed
				break;
			}
			
			
		}
		if(ValueLeft>250 && ValueRight<250){
			//go left
			AdjustSpeeds(4500, 5250);	//--------------------------right - left
		}
		if(ValueLeft<250 && ValueRight>250){
			//go right
			AdjustSpeeds(5250, 4500);	//--------------------------right - left
		}
		if(ValueLeft>250 && ValueRight>250){
			//stop
			StopMoving();
			break;
		}
		if(pingdist <= 20){
			StopMoving();
			break;
		}
	}
	//clear the encoders
	ClearEncoders();
}

int MovingForwardObstacleCheck(int a){
	InitialSpeed(4800);
	while(1){
		SpeedAdjust();	//adjust the speed
		PingCheck();
		set_lcd_addr(0x00);
		write_long_lcd((long)pingdist);
		if(pingdist <= 20){
			StopMoving();
			break;
		}
	}
	//clear the encoders
	ClearEncoders();
}



int Adjust(int speedl, int speedr, int dist){
	AdjustSpeeds(speedl, speedr);	//--------------------------right - left
	while(1){
		turn(dist);	//turn by distance
		if(GetLeftEncoderTotal() >= (selectedEncoderCount) || GetRightEncoderTotal() >= (selectedEncoderCount)){
			break;
		}
		set_lcd_addr(0x00);
		write_long_lcd(GetLeftEncoderTotal());
		
		set_lcd_addr(0x40);
		write_long_lcd(GetRightEncoderTotal());
	}
	ClearEncoders();
}



void TurnLeft(int dist){
	AdjustSpeeds(5250, 3800);	//--------------------------right - left
	while(1){
		turn(dist);	//turn by distance
		if(GetLeftEncoderTotal() >= (selectedEncoderCount) || GetRightEncoderTotal() >= (selectedEncoderCount)){
			break;
		}
		set_lcd_addr(0x00);
		write_long_lcd(GetLeftEncoderTotal());
		
		set_lcd_addr(0x40);
		write_long_lcd(GetRightEncoderTotal());
	}
	ClearEncoders();
}

void TurnRight(int dist){
	AdjustSpeeds(3800, 5250);	//--------------------------right - left
	while(1){
		turn(dist);	//turn by distance
		if(GetLeftEncoderTotal() >= (selectedEncoderCount) || GetRightEncoderTotal() >= (selectedEncoderCount)){
			break;
		}
		set_lcd_addr(0x00);
		write_long_lcd(GetLeftEncoderTotal());
		
		set_lcd_addr(0x40);
		write_long_lcd(GetRightEncoderTotal());
	}
	ClearEncoders();
}





//initialize variables
int keyValue;

void main() {
	keypad_enable();
	keyValue = getkey();
	
	if(keyValue == 1){			//push the button labeled 1 on the HCS12 DRAGONBOARD
	
		//initialize variables
		int i;
		
		//disable the SEVEN-SEGMENT DISPLAY
		seg7_disable();
		
		//enable & initialize & reset all components of the robot
		servo54_init();
		servo76_init();
		lcd_init();
		ResetEncoder();
		
		
		
		//Start The Boulevard Part Of The Course
		//run the motor & ping sensor until the sensor is within 16 cm of the obstacles
		//MUST BE A STATE FOR 2 DIFFERENT CONDITIONS :
		//1 : start on left
		//2 : start on right
		
		
		//RunBoulevard('R');	//2 : start on right & run the robot through the boulevard with this method
		
		RunBoulevard('L');		//1 : start on left & run the robot through the boulevard with this method
		ms_delay(200);
		
		RunPath1();				//run the robot on the 1st path with this method
		ms_delay(200);
		
		RunPath2();				//run the robot on the 2nd path with this method
		ms_delay(200);
		
		RunPath3();				//run the robot on the 3rd path with this method
		ms_delay(200);
		
		RunParkingLot();		//park the robot with this method
	}
}
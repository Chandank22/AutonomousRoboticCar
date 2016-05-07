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
void TurnLeft(int);	//forward-declared method
void TurnRight(int);	//forward-declared method
void Stop (int);	//forward-declared method
void END (int);	//forward-declared method
//long PingSensor(void);	//forward-declared method
//long PingSensor(void);	//forward-declared method
//MovingForwardObstacleCheck(void);//forward-declared method


static  long leftAccumulator = 0;    // total counts from Right Encoder
static  long rightAccumulator = 0;   // total counts from Left Encoder

#define ACCUM_MAX_THRESHOLD 180     // value used to know when to move value into s/w counter

void  ResetEncoder() 
{
  DDRT  &= 0xF3;   //init PT2 & PT3 as input
  ICPAR = 0xFC;   //enable PT2 & PT3 as pulse accum
  TCTL4 = 0x50;   //set to capture rising edge

  leftAccumulator = 0;  
  rightAccumulator = 0;

  PACN3 = 0; // reset hw accumulators
  PACN2 = 0; 
}

void ClearEncoders(){
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

int sensorChange = 0;   // needs to be set when a new sensor is used
void SensorSwitch() 
{
  sensorChange = 1;
}

int HI_time0; // High time (pulse width)
int HI_time1; // High time (pulse width)

char  pingHiorLo;
char  parkHiorLo;
#define PingHiCnt 3       // high pulse duration in TCNT counts
#define PingLoCnt 500      // low signal between pulses time in TCNT Counts
#define ParkHiCnt 3       // high pulse duration in TCNT counts
#define ParkLoCnt 500      // low signal between pulses time in TCNT Counts  
double pingtime;
double pingdist;
double parktime;
double parkdist;

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



int ValueLeft;
int ValueRight;
int fsm;
int forwardState;

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
					//reset the counter
					selectedEncoderCount = 0;
					forwardState = 0;//forward mode disabled
					//check the last state
					
					//break out of everything
					fsm = 3;	//robot is reading left B and right B
					break;	//stop moving
				}else if(ValueLeft>250 && ValueRight<250){
					//clear the encoders
					ClearEncoders();
					//reset the counter
					selectedEncoderCount = 0;
					MoveBackward(3800, 3800, 8);
					//clear the encoders
					ClearEncoders();
					//reset the counter
					selectedEncoderCount = 0;
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
					//reset the counter
					selectedEncoderCount = 0;
					MoveBackward(3800, 3800, 8);
					//clear the encoders
					ClearEncoders();
					//reset the counter
					selectedEncoderCount = 0;
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
						//reset the counter
						selectedEncoderCount = 0;
						//stop
						StopMoving();
						fsm = 3;	//robot is reading left B and right B
						break;
					} else if(fsm == 0){
						//go straight
						AdjustSpeeds(4800, 4800);	//LEFT - RIGHT
						SpeedAdjust();	//adjust the speed
						fsm = 0;	//robot is reading left W and right W
						break;
					} else if(fsm == 1){
						//clear the encoders
						ClearEncoders();
						//reset the counter
						selectedEncoderCount = 0;
						//add delay to prevent reading old adc values
						ms_delay(100);
						//go right & MOVE FORWARD
						AdjustSpeeds(4850, 4700);
						fsm = 1;	//robot is reading left B and right W
						break;
					} else if(fsm == 2){
						//clear the encoders
						ClearEncoders();
						//reset the counter
						selectedEncoderCount = 0;
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
					//reset the counter
					selectedEncoderCount = 0;
					forwardState = 0;//forward mode disabled
					//check the last state
					
					//break out of everything
					fsm = 3;	//robot is reading left B and right B
					break;	//stop moving
				}else if(ValueLeft>250 && ValueRight<250){
					//clear the encoders
					ClearEncoders();
					//reset the counter
					selectedEncoderCount = 0;
					MoveBackward(3800, 3800, 8);
					//clear the encoders
					ClearEncoders();
					//reset the counter
					selectedEncoderCount = 0;
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
					//reset the counter
					selectedEncoderCount = 0;
					MoveBackward(3800, 3800, 8);
					//clear the encoders
					ClearEncoders();
					//reset the counter
					selectedEncoderCount = 0;
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
	//reset the counter
	selectedEncoderCount = 0;
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
	//Adjust(3800, 3800, 3);//--------------------------right - left
	//ms_delay(100);
	
	
	
	//adjust the robots orientation
	Adjust(3800, 4500, 22);//--------------------------right - left
	ms_delay(100);
	//adjust the robots orientation
	Adjust(4500, 5250, 22);//--------------------------right - left
	ms_delay(100);
	
	
	
	//RunLightSensorsAdjustRobot();
	
	
	
	
	
	

	
	ms_delay(100);
}

int counterCheck;

int parkingSpot1;
int parkingSpot2;
int parkingSpot3;
int parkingSpot4;

int whiteSpaceStateCount;
int rightTurns;
int leftTurns;

void RunParkingLot(){
	parkingSpot1 = 0;
	parkingSpot2 = 0;
	parkingSpot3 = 0;
	parkingSpot4 = 0;
	
	ad0_enable();
	
	while(1){
		whiteSpaceStateCount = 0;
		rightTurns = 0;
		leftTurns = 0;
		ms_delay(1000);
		SensorSwitch();	//set a delay to prevent ping sensor race condition
		//go forward 57 cm
		MoveForward(4800, 4800, 52);
		//adjust the robots orientation
		//Adjust(4900, 4500, 2);//--------------------------right - left
		//check 1st parking spot
		for(counterCheck = 0; counterCheck < 75; counterCheck++){
			ValueLeft = ad0conv(6);    // PAD06          //Robots left
			ValueRight = ad0conv(2);   // PAD02          //Robots right
			/*
			if(ValueLeft<250 && ValueRight<250){
				whiteSpaceStateCount++;
			}
			if(whiteSpaceStateCount == 1){
				//go right
				Adjust(5250, 4500, 1);	//LEFT - RIGHT
			}
			*/
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
		whiteSpaceStateCount = 0;
		rightTurns = 0;
		leftTurns = 0;
		ms_delay(1000);
		SensorSwitch();	//set a delay to prevent ping sensor race condition
		//go forward 45 cm
		MoveForward(4800, 4800, 45);
		//adjust the robots orientation
		
		//check 1st parking spot
		for(counterCheck = 0; counterCheck < 75; counterCheck++){
			ValueLeft = ad0conv(6);    // PAD06          //Robots left
			ValueRight = ad0conv(2);   // PAD02          //Robots right
			/*
			if(ValueLeft<250 && ValueRight<250){
				whiteSpaceStateCount++;
			}
			if(whiteSpaceStateCount == 1){
				//go right
				Adjust(5250, 4500, 1);	//LEFT - RIGHT
			}
			*/
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
		
		whiteSpaceStateCount = 0;
		rightTurns = 0;
		leftTurns = 0;
		ms_delay(1000);
		SensorSwitch();	//set a delay to prevent ping sensor race condition
		//go forward 45 cm
		MoveForward(4800, 4800, 45);
		//Adjust(4800, 4500, 2);//--------------------------right - left
		//check 1st parking spot
		for(counterCheck = 0; counterCheck < 75; counterCheck++){
			ValueLeft = ad0conv(6);    // PAD06          //Robots left
			ValueRight = ad0conv(2);   // PAD02          //Robots right
			/*
			if(ValueLeft<250 && ValueRight<250){
				whiteSpaceStateCount++;
			}
			if(whiteSpaceStateCount == 1){
				//go right
				Adjust(5250, 4500, 1);	//LEFT - RIGHT
			}
			*/
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
		whiteSpaceStateCount = 0;
		rightTurns = 0;
		leftTurns = 0;
		ms_delay(1000);
		SensorSwitch();	//set a delay to prevent ping sensor race condition
		//go forward 45 cm
		MoveForward(4800, 4800, 45);
		//adjust the robots orientation
		//Adjust(5200, 4500, 2);//--------------------------right - left
		//check 1st parking spot
		for(counterCheck = 0; counterCheck < 75; counterCheck++){
			ValueLeft = ad0conv(6);    // PAD06          //Robots left
			ValueRight = ad0conv(2);   // PAD02          //Robots right
			/*
			if(ValueLeft<250 && ValueRight<250){
				whiteSpaceStateCount++;
			}
			if(whiteSpaceStateCount == 1){
				//go right
				Adjust(5250, 4500, 1);	//LEFT - RIGHT
			}
			*/
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
	//reset the counter
	selectedEncoderCount = 0;
}

int ParallelPark(){
	//go forward 28 cm
	MoveForward(4800, 4800, 28);
	//clear the encoders
	ClearEncoders();
	//reset the counter
	selectedEncoderCount = 0;
	//start backing into the parking spot
	MoveBackward(4300, 3800, 66);
	//clear the encoders
	ClearEncoders();
	//reset the counter
	selectedEncoderCount = 0;
	MoveBackward(3800, 3800, 5);
	//clear the encoders
	ClearEncoders();
	//reset the counter
	selectedEncoderCount = 0;
	//turn the robot in the parking spot to finish
	TurnRight(26);
}


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
				//reset the counter
				selectedEncoderCount = 0;
				//stop
				StopMoving();
				toggle = 1;
				break;
			} else if(ValueLeft<250 && ValueRight>250){
				//clear the encoders
				ClearEncoders();
				//reset the counter
				selectedEncoderCount = 0;
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
				AdjustSpeeds(4900, 4900);	//LEFT - RIGHT
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
			AdjustSpeeds(4500, 5250);	//LEFT - RIGHT
			
		}
		if(ValueLeft<250 && ValueRight>250){
			//go right
			
			AdjustSpeeds(5250, 4500);	//LEFT - RIGHT
		}
		if(ValueLeft>250 && ValueRight>250){
			//stop
			StopMoving();
			break;
		}
	}
	//clear the encoders
	ClearEncoders();
	//reset the counter
	selectedEncoderCount = 0;
}









/*
int RunLightSensorsPerDisatnce(int dis){
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
				//reset the counter
				selectedEncoderCount = 0;
				//stop
				StopMoving();
				break;
			} else if(ValueLeft<250 && ValueRight>250){
				//clear the encoders
				ClearEncoders();
				//reset the counter
				selectedEncoderCount = 0;
				//stop
				StopMoving();
				break;
			} else{//assumed 0 : 0
				//go straight
				AdjustSpeeds(4900, 4900);	//LEFT - RIGHT
				SpeedAdjust();	//adjust the speed
				break;
			}
			
			
		}
		if(ValueLeft>250 && ValueRight<250){
			//go left
			AdjustSpeeds(4500, 5250);	//LEFT - RIGHT
		}
		if(ValueLeft<250 && ValueRight>250){
			//go right
			AdjustSpeeds(5250, 4500);	//LEFT - RIGHT
		}
		if(ValueLeft>250 && ValueRight>250){
			//stop
			StopMoving();
			break;
		}
	}
	//clear the encoders
	ClearEncoders();
	//reset the counter
	selectedEncoderCount = 0;
}
*/















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
				//reset the counter
				selectedEncoderCount = 0;
				//stop
				StopMoving();
				break;
			} else if(ValueLeft<250 && ValueRight>250){
				//clear the encoders
				ClearEncoders();
				//reset the counter
				selectedEncoderCount = 0;
				//stop
				StopMoving();
				break;
			} else{//assumed 0 : 0
				//go straight
				AdjustSpeeds(4900, 4900);	//LEFT - RIGHT
				SpeedAdjust();	//adjust the speed
				break;
			}
			
			
		}
		if(ValueLeft>250 && ValueRight<250){
			//go left
			AdjustSpeeds(4500, 5250);	//LEFT - RIGHT
		}
		if(ValueLeft<250 && ValueRight>250){
			//go right
			AdjustSpeeds(5250, 4500);	//LEFT - RIGHT
		}
		if(ValueLeft>250 && ValueRight>250){
			//stop
			StopMoving();
			break;
		}
	}
	//clear the encoders
	ClearEncoders();
	//reset the counter
	selectedEncoderCount = 0;
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
				//reset the counter
				selectedEncoderCount = 0;
				//stop
				StopMoving();
				break;
			} else if(ValueLeft<250 && ValueRight>250){
				//clear the encoders
				ClearEncoders();
				//reset the counter
				selectedEncoderCount = 0;
				//stop
				StopMoving();
				break;
			} else{//assumed 0 : 0
				//go straight
				AdjustSpeeds(4900, 4900);	//LEFT - RIGHT
				SpeedAdjust();	//adjust the speed
				break;
			}
			
			
		}
		if(ValueLeft>250 && ValueRight<250){
			//go left
			AdjustSpeeds(4500, 5250);	//LEFT - RIGHT
		}
		if(ValueLeft<250 && ValueRight>250){
			//go right
			AdjustSpeeds(5250, 4500);	//LEFT - RIGHT
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
	//reset the counter
	selectedEncoderCount = 0;
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
	//reset the counter
	selectedEncoderCount = 0;
}



int Adjust(int speedl, int speedr, int dist){
	AdjustSpeeds(speedl, speedr);	//LEFT - RIGHT
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
	selectedEncoderCount = 0;
}



void TurnLeft(int dist){
	AdjustSpeeds(5250, 3800);	//LEFT - RIGHT
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
	selectedEncoderCount = 0;
}

void TurnRight(int dist){
	AdjustSpeeds(3800, 5250);	//LEFT - RIGHT
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
	selectedEncoderCount = 0;
}

int keyValue;
void main() {
	keypad_enable();
	keyValue = getkey();
	
	if(keyValue == 1){
		//initialize all variables
		int i;

		//START PROGRAM WITH A 10 SECOND DELAY
		//ms_delay(10000);

		seg7_disable();
		servo54_init();
		servo76_init();
		lcd_init();

		ResetEncoder();
		
		//Start The Boulevard Part Of The Course
		//run the motor & ping sensor until the sensor is within 16 cm of the obstacles
		//MUST BE A STATE FOR 3 DIFFERENT CONDITIONS :
		//1 : start in middle
		//2 : start on left
		//3 : start on right
		
		//CREATE A CONDITIONAL THAT STARTS THE ROBOT ON THE PREFERRED SIDE OF THE STARTING LINE
		//RunBoulevard('R');
		
		
		
		
		
		RunBoulevard('L');
		ms_delay(200);
		
		RunPath1();//run 1st time for path 1
		ms_delay(200);
		
		RunPath2();//run 2nd time for path 2
		ms_delay(200);
		
		RunPath3();//run 3rd time for path 3
		ms_delay(200);
		
		RunParkingLot();
	}
}
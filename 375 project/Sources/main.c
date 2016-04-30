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
#define PI 3.14159265358979323846	//the constant PI
#define r 0.052375		//in meters the radius of the wheel
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

  


int HI_time1; // High time (pulse width)
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

unsigned int motorL = 4500;
unsigned int motorR = 4500;

void goStraight(int degrees, char direction);	//forward-declared method
void GoStraight(int);	//forward-declared method
int PingSensor(int);	//forward-declared method
void TurnLeft(int degrees, char direction);	//forward-declared method
void TurnRight(int);	//forward-declared method
void Stop (int);	//forward-declared method
void END (int);	//forward-declared method





void goStraight(int degrees, char direction){			//example of calling the function : goStraight(180, 'F');	//'B' is backwards
/* 	wheelDistance = (((((degrees*PI/180)*r)/(2*PI))*(180/PI))/180);	//distance the wheel should turn
	
	if(direction == 'F'){//go forwards
		t = fullSpeedForwardTime;
		newSpeedL = 5700;
		newSpeedR = 5700;
	} else{//go backwards
		t = fullSpeedBackwardTime;
		newSpeedL = 3300;
		newSpeedR = 3300;
	}
	
	iterationCount = ((int) wheelDistance/t);	//this number represents the amount of iterations must occur for the wheel to turn the amount of degrees exactly specified
	 */
	 newSpeedL = 5700;
		newSpeedR = 5700;
	for(i = 0; i < 100000; i++){
		set_servo54(newSpeedL);
		set_servo76(newSpeedR);
		
	}
	
	
	
	
}











void main() {
	
    //
    servo54_init();
    servo76_init();
    lcd_init(); // start the LCD
  


  while(1){
   
  
    
    //PingSensor(1);
    
   // set_lcd_addr(0x00);
   // write_int_lcd((int)dist);
    
    
    
      
    //  if(dist <28){
       
      
         
    //Stop(1000);
    //TurnLeft(1200); 
/* 	newSpeedL = 5500;
	newSpeedR = 5500;
	
	set_servo54(newSpeedL);
    set_servo76(newSpeedR); */
	
    GoStraight(100);    //encoder counts may be affected by the turn counts, need to address
	
	
	//goStraight(180, 'F');
	
	//END(1);            //terminate program
	
	
	
   // Stop(1000);
    //TurnRight(1000);
   // GoStraight(100);    //encoder counts may be affected by the turn counts, need to address
     
                      
     
   //    } else{
       
   //     GoStraight(10);
  //     }
       
       
       
  }


  
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
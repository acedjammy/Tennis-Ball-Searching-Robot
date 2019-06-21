#include "VirtualWire.h"
#include "Transmitter.h"
#include <SPI.h>
#include <Pixy.h>
#include "motion.h"

#include "debounceclass.h"

# define angleRatio   0.06024 //in deg 9/149.4
# define distanceRatio 0.167552  // in mm  3.1415926 * 60 */1124.43



//----state definition----//
const int start = 0;       // go to initial start point
const int catchBalls = 1;  // catch ball loop
const int findContainer_and_hit_it = 2;    // find door and turn to hit the boundary
const int go_backward = 3;   // using limitswitch to align the car with the door, keep distance
const int communicatewithDoor = 4; // send inf to door, wait for open.
const int go_forward = 5;
const int releaseBalls = 6;     // move onto the gate, push out all balls
const int finish = 7;            // move backward, send inf to close door.

int state = 0;

//-----object asignment----//
Pixy pixy;
motion *myMotion;
PIDnew *pixyPID;
PIDnew *Approach_Door_mark_PID;


debounce pixydebounce[3]={debounce(),debounce(),debounce()};



//-----motion variblzes----//
volatile long countL = 0;
volatile long countR = 0;
volatile unsigned long countL_ALL = 0;
volatile unsigned long countR_ALL = 0;



//-----limit switch ------//
const int PIN_Front = 18;
const int PIN_FL = 20;
const int PIN_FR = 19;

volatile int LS_FLhit = 0;
volatile int LS_FRhit = 0;
volatile int LS_Fronthit = 0;



//-----pixy varibles------//
int deadband[2] = {0,0};
int range[2] = {-30, 30};

//-----Find ball varibles----//
const int doorCounterPIN = 42; 

// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&//
int ball_total_number = 3 ;


int catched_ball_num = 0;
int sidemake_signature[4] ={4,5,6,7};
const int robot_door = 36;


//-------go to container---------//
int hit_flag = 0;

//-------go back/forward state-------//
float backdistance = 300;
float forwarddistance = 300;

//-------container gate ------//
int CD = 0; // indicate whether container should open or close

//-------Communication----------//
int send_inf_flag = 1;   // 1 is send open inf, 0 is send close inf

//-------Push ball------------//
const int stepPin = 34; 
const int dirPin = 35; 

//------IR----------//
int IR = 0;  //A0




void setup() {

Serial.begin(19200);

SetupTransmitter(12);   //transmitter initiate

pinMode(PIN_FL,INPUT_PULLUP);  // limitswitchPIN
pinMode(PIN_FR,INPUT_PULLUP);
pinMode(PIN_Front,INPUT_PULLUP); 

pinMode(2,INPUT);  // encoder PIN
pinMode(3,INPUT);


pinMode(stepPin,OUTPUT); 
pinMode(dirPin,OUTPUT);

pinMode(robot_door,OUTPUT);
pinMode(IR,INPUT);

attachInterrupt(digitalPinToInterrupt(PIN_FL), limitswitchFL, CHANGE);
attachInterrupt(digitalPinToInterrupt(PIN_FR), limitswitchFR, CHANGE);
attachInterrupt(digitalPinToInterrupt(PIN_Front),limitswitchFront , CHANGE);

attachInterrupt(digitalPinToInterrupt(2), counterL, CHANGE);
attachInterrupt(digitalPinToInterrupt(3), counterR, CHANGE);

myMotion = new motion (5, 12, 0.001, 8,7,6,&countL,5, 12, 0.001,4,5,9,&countR);
pixy.init();
pixyPID = new PIDnew(1, 0, 0.001, 5, deadband, range);  //use to find ball

Approach_Door_mark_PID = new PIDnew(1.5, 0, 0, 5, deadband, range);  // use to find the door mark

}

void loop() {


	
 // float actualVR = myMotion->getWheelSpeed('R');
 // Serial.print("actualspeed:");
 // Serial.print (actualVR);

 // float actualVL = myMotion->getWheelSpeed('L');
 
 // Serial.print("actualspeedL:");
 // Serial.println (actualVL);

 switch (state){
	 //---------------------------------------------------------------------------------------------------------------------------//
	case start:
    if (goto_initial_position() == 1){
       state = catchBalls;
	   myMotion->refreshAll();
     }
   break;
   
   //---------------------------------------------------------------------------------------------------------------------------//
   case catchBalls:
    
        static int signature_num = 0;
        static int need_goto_sidemark = 0;
		static int find_oneball_flag = 0;
   
      find_oneball_flag = catch_ball_func();//catch_ball_func();  // 1 find; 0 do not ; -1 need go to mark try again. 
	  
      if ( find_oneball_flag == 1){
       catched_ball_num++;
       
       myMotion->stop();
       pixyPID->refresh();
       
      }

    if (catched_ball_num == ball_total_number){
       state = findContainer_and_hit_it;
	   
	   myMotion->stop();
    }
    LS_FLhit = 0;
	LS_FRhit = 0;
	LS_Fronthit = 0;
   break;
   
   //---------------------------------------------------------------------------------------------------------------------------//
   case findContainer_and_hit_it:
   
   Serial.println("ININIcontainer");
   
      static int findContainer_state = 0;
	  static int hit_flag = 0;
	  Serial.print(findContainer_state);
	  if  (findContainer_state == 0)
		   findContainer_state = go_to_container();   //2 is gate signiture  // return 0,scan, return 1 approach
	  else{
		  state = go_backward;
		  myMotion->stop();
	  }
	  
	
	  
   break;
   
 //---------------------------------------------------------------------------------------------------------------------------//
   case go_backward:
    Serial.println("ININIgo_backward");
     static int go_backward_finish_flag = 0;
	 go_backward_finish_flag = go_backward_function(backdistance);
	 myMotion->move(-1,60,0);
	 Serial.println("GOBACK");
	if (go_backward_finish_flag == 1){
		state = communicatewithDoor;
		send_inf_flag = 1;
		myMotion->stop();
	}

   break;
//---------------------------------------------------------------------------------------------------------------------------//

   case communicatewithDoor:
   Serial.println("communicatewithDoor");
	
   if  (send_inf_flag == 1)
	   
   {static unsigned long  send_open_start = millis();    
   Serial.println("Send");
	   
	   SendMsg(101);

	   if(millis()- send_open_start > 3000){
	   state = go_forward;
	   }
	   
   }
   else if (send_inf_flag == 0){
		
	   static unsigned long send_close_start = millis();
	   SendMsg(100);
	   if(millis()- send_close_start > 3000){
	   state = finish;
	   }
}
     
	 
	 break;


   //---------------------------------------------------------------------------------------------------------------------------//
	case go_forward:
	  Serial.println("GOFOR"); 
     static int go_forward_finish_flag = 0; 
	 go_forward_finish_flag = go_forward_function(forwarddistance);
	 myMotion->move(1,30,0);
	 
	if (go_forward_finish_flag == 1){
		state = releaseBalls;
		myMotion->stop();
	}
	 //---------------------------------------------------------------------------------------------------------------------------//
	break;
	case releaseBalls:
	  Serial.println("Release"); 
      myMotion->stop();  
      
      digitalWrite(robot_door,HIGH); // Enables the motor to move in a particular direction
      // Makes 200 pulses for making one full cycle rotation
	  
      digitalWrite(dirPin,HIGH);   // push push
	  delay(1000);
	  for(int x = 0; x < 3800; x++) {
        digitalWrite(stepPin,HIGH); 
        delayMicroseconds(500); 
        digitalWrite(stepPin,LOW); 
        delayMicroseconds(500); 
      }
            
	  countL_ALL = 0;
	  countR_ALL = 0;
	  
      while (countL_ALL*distanceRatio<400 && countR_ALL*distanceRatio<400){
        myMotion->move(-1, 60, 0);
      }
	  
	  send_inf_flag = 0;
      state = communicatewithDoor;
	 // break;
	 
	 case finish:
	 
	 Serial.println("finish");
	 myMotion->stop();
	 
	 static int exe_finish = 1;
	 if (exe_finish == 1){
		 
	 	  delay(1000);
		  
		  digitalWrite(dirPin,LOW);
	  for(int x = 0; x < 3780; x++) {
        digitalWrite(stepPin,HIGH); 
        delayMicroseconds(500); 
        digitalWrite(stepPin,LOW); 
        delayMicroseconds(500); 
      }
	  exe_finish =0;
	 }
	 
	 break;
	 
	 
	 default:
	 break;
}
}


//---------------------------------------subfunctions----------------------------------------------//


//-----------------start state----------------------//
//-----------------start state----------------------//
//-----------------start state----------------------//

int goto_initial_position(){   
    static int  goto_initial_position_state = 0;

    // if (Serial.read() == 'C'){
    // goto_initial_position_state = 1;
  // }
  
  if (LS_FLhit == 1){
    goto_initial_position_state = 1;

    LS_FLhit = 0;

  }


  switch (goto_initial_position_state){
    case 0 :  
      static int Execute1 = 1;
      if (Execute1 == 1){
        countR_ALL = 0;
        Execute1 = 0;
      }
      myMotion -> turn('L',1,100);
      
      if (countR_ALL * angleRatio > 60){
        goto_initial_position_state = 2;

      }
     
      return 0;
    break;
    
    case 1 :
      static int Execute2 = 1;
      if (Execute2 == 1){
        countL_ALL = 0;
        Execute2 = 0;
 
      }
      myMotion -> turn('R',1,100);
      
      if (countL_ALL * angleRatio > 60){
        goto_initial_position_state = 2;
      }

      return 0;
    break;
    
    case 2 :
      static int Execute3 = 1;
      if (Execute3 == 1){
        countR_ALL = 0;
        countL_ALL = 0;
        Execute3 = 0;
      }
      myMotion -> move(1, 80, 0);
      if (countR_ALL * distanceRatio > 1000 && countL_ALL * distanceRatio > 1000){
        myMotion -> stop();
        return 1;
      } else {return 0;}
      
    break;
    
    default:
    
    break;
      
    
  }
  

}

//-----------------catchBalls state----------------------//
//-----------------catchBalls state----------------------//
//-----------------catchBalls state----------------------//

int catch_ball_func(){
  
  static float pixyParam[5] = {0,0,0,0,0}; //Xerror, Yvertical,Width,Height,Area
  static float Xerr_use = 0;
  static float Yvertical_use = 0;
  static int find_ball_state = 0;
  static float movingtime_start = 0;
  static int getballflag = 0;
  
  static int Execute_findball_case2 = 1;
  static int Execute_findball_case0 = 1;  
  
  static int count_start_flag = 0;
  
  static long back_to_center_distance = 0;
  
  int catch_oneball_finish_mark = 0;
  
  getPixyPara(pixyParam,1);  // parameter matrix, color signiture.
  
   float Xerror = pixyParam[0] - 160;
   float Yvertical = 200 - pixyParam[1];
   float Area = pixyParam[4];
   int IRreading = analogRead(IR);
     // print 
     Serial.print("Xcoor is:");
	 Serial.print(Xerror);
	 Serial.print(", Ycoor is:");
	 Serial.print(Yvertical);
	 Serial.print(", substate :");
	 Serial.print(find_ball_state);
  
   if (Xerror != - 160){
    Xerr_use = Xerror ;
   } 
   
   if (Yvertical != 200){
    Yvertical_use = Yvertical ;
   }

	switch (find_ball_state){
		
		
	case 0:  //scan for ball
    
   
	static int delayflag = 0;
	static unsigned long oldtime = 0;


      if (IRreading <400){
        find_ball_state = 3;
      }
 
      if (abs(Xerror)<40){
		  
		  if (delayflag == 0){
			  delayflag = 1;
			  oldtime = millis();
		  }

		  
		  if (((millis()- oldtime)>100)){    //rotation debounce
				
			  myMotion->stop();
			  myMotion->refreshAll(); 
			  if (Yvertical<70){
			myMotion -> stop();
             find_ball_state = 3;
              }else {find_ball_state = 1;}
			  
		  }
       
        
      
      }else if (Xerror < -40 ){
          delayflag = 0;
          myMotion->turn('L', 1, constrain(abs(Xerror)-40,5,70));
          
          if (Execute_findball_case0 == 1){
            countR_ALL = 0;
            Execute_findball_case0 = 0;
          }
	
			 
          if (countR_ALL * angleRatio > 720 ){
             myMotion -> stop();        // finished initiate all para.   
			 
             Xerr_use = 0;
             Yvertical_use = 0;
             find_ball_state = 0;
             movingtime_start = 0;
             getballflag = 0;
             Execute_findball_case2 = 1;
             Execute_findball_case0 = 1;
			 myMotion->refreshAll();
             catch_oneball_finish_mark = -1;
          }
      }
	  else {
		   delayflag = 0;
      myMotion->turn('R', 1,constrain(abs(Xerror)-40,5,70));
      }
      break;
		case 1:
		
		static int back_to_center_exe = 1;
		if (back_to_center_exe == 1){
		back_to_center_exe = 0;
		countL_ALL = 0;
		countR_ALL = 0;
		back_to_center_distance = 0;
		}
		
      find_ball_state = ApproachBall(Xerr_use,Yvertical_use,pixyParam[4]);  // return 1 or 2

      if (find_ball_state == 2){        
        Execute_findball_case2 = 1;
		}
	break;
  
	case 2 :
      count_start_flag = 1;
      myMotion->move(1, 55, 0);
	  digitalWrite(robot_door,HIGH); 
      	  //enable the counter  
       
	break;
  
	case 3 :   // if the ball is too close to robot, backward.
    myMotion->move(-1, 40, 0);
      if ((Yvertical>80)&&(Yvertical!=200)){
      find_ball_state = 1;
      }
      else {find_ball_state = 3;}
	break;
    case 4 : 
	static int move_back_to_center = 1;
	
	if (move_back_to_center == 1){
	countL_ALL = 0;
	countR_ALL = 0;
	move_back_to_center = 0;
	}
	
	myMotion->move(-1,80,0);
	
	if (countL_ALL * distanceRatio > back_to_center_distance && countR_ALL * distanceRatio > back_to_center_distance)
	{
		back_to_center_distance = 0;

		catch_oneball_finish_mark = 1;

		move_back_to_center = 1;
		
		myMotion->stop();

	}
	
  
	default:
	break;
}
    if (count_start_flag == 1){
		
    if (Getball_flag() == 1 ){
     movingtime_start = millis();
     getballflag = 1;
	 count_start_flag = 0;
   }
	}
    if (millis()-movingtime_start > 1000 && getballflag == 1){
       //finished initiate all para.
		 // find_ball_state = 4;  // go back to center
		 // back_to_center_distance = countL_ALL * distanceRatio;
		 
		 
         Xerr_use = 0;
         Yvertical_use = 0;
         find_ball_state = 0;
         movingtime_start = 0;
         getballflag = 0;
		 Execute_findball_case2 = 1;
         Execute_findball_case0 = 1;
        
		 digitalWrite(robot_door,LOW);  //close the robot door
		 
		 myMotion->refreshAll();
         pixyPID->refresh();
		 
		 catch_oneball_finish_mark = 1;

  }
   
  
  
  return catch_oneball_finish_mark;

}
int ApproachBall(float Xerror, float Yvertical, float Area){
  
  static float timeNew = 0;
  static float timeOld = 0;
  static float deltaTime = 0; 
  static int  ballinflag = 1;
  static int output = 0; 
  static float DeltaVec = 0;
  static float BaseVec = 0;
  
    timeNew=micros();
    if ((timeNew - timeOld)>10000){
    deltaTime = (timeNew - timeOld) / 1e6;  
    timeOld = timeNew;
    pixyPID->update (0, Xerror, deltaTime, output);
    
    DeltaVec = - output;

    }
     
	 
    
    BaseVec = 50 + 0.3 * Yvertical;
    myMotion->move(1, BaseVec, DeltaVec);
//    Serial.print("DeltaT:");
//	Serial.print(deltaTime);
	
    if (Yvertical < 60 && Area <250){
      ballinflag = 2;
      // initiate for next use
      pixyPID->refresh();
      timeNew = 0;
      timeOld = 0;
      deltaTime = 0;
      output = 0;
      DeltaVec = 0;
    BaseVec = 0;
	
    }else{ballinflag = 1;}    //which is find_ball_state
    
    
    return ballinflag;
  
}

int getPixyPara(float *pixyPara,int signature){
  int detected_flag = 0;
  int block_num;
  uint16_t original_blocks = pixy.getBlocks();    
  for (int i = 0; i < original_blocks; i++){
    
    if (pixy.blocks[i].signature == signature){
      block_num = i;
      detected_flag = 1;
    }
    
  }
    
  
   int detected_flag_db = pixydebounce[signature-1].refresh(detected_flag,30);
   int pixyX=0;
   int pixyY=0;
   int pixyWidth=0;
   int pixyHeight=0; 
  
  
  if ( detected_flag_db && (pixy.blocks[block_num].width*pixy.blocks[block_num].height>10) )
  {
    pixyX=pixy.blocks[block_num].x;
    pixyY=pixy.blocks[block_num].y;
    pixyWidth=pixy.blocks[block_num].width;
    pixyHeight=pixy.blocks[block_num].height;
    detected_flag = 1;
  }   
       pixyPara[0]=pixyX;      
     pixyPara[1]=pixyY;
     pixyPara[2]=pixyWidth;
     pixyPara[3]=pixyHeight;
       pixyPara[4]=pixyHeight * pixyWidth;
  
    
  return detected_flag;
}
  
int Getball_flag(){
  
  pinMode(doorCounterPIN,INPUT);
  
  static int ballin_mark = 0;
  static long timelastrecord = 0;
    
  if (digitalRead(doorCounterPIN) == HIGH ){
    ballin_mark = 1;
  }
  
  if (ballin_mark==1 && digitalRead(doorCounterPIN)== LOW && (millis() - timelastrecord)>2500){
    ballin_mark = 0; 

    timelastrecord = millis();
    
    return 1;
  } else {
    return 0;
  }
}

//-----------------findContainer_and_hit_itstate----------------------//
//-----------------findContainer_and_hit_it state----------------------//
//-----------------findContainer_and_hit_it state----------------------//

int go_to_container(){
  
  static float pixyParaDoor[5] = {0,0,0,0,0}; //Xerror, Yvertical,Width,Height,Area
  static float Xerr_use = 0;
  static float Yvertical_use = 0;
  static int go_to_door_state = 0;
  static int already_hit_door_mark = 0;
 
  getPixyPara(pixyParaDoor,2);  // parameter matrix, color signiture.
  
   float Xerror = pixyParaDoor[0] - 160;
   float Yvertical = 200 - pixyParaDoor[1];
   unsigned long Area = pixyParaDoor[4];
     // print 
     Serial.print("Xcoor is:");
	 Serial.print(Xerror);
	 Serial.print(", Ycoor is:");
	 Serial.print(Yvertical);
	 Serial.print("Area is:");
	 Serial.print(Area);
	 Serial.print(", substate :");
	 Serial.println(go_to_door_state);
  
   if (Xerror != - 160){
    Xerr_use = Xerror ;
   } 
   
   if (Yvertical != 200){
    Yvertical_use = Yvertical ;
   }

	switch (go_to_door_state){
		
		
	case 0:  //scan for gate color
    
	static int delayflag = 0;
	static unsigned long oldtime = 0;

      if (abs(Xerror)<40){
		  
		  if (delayflag == 0){
			  delayflag = 1;
			  oldtime = millis();
		  }

		  if (((millis()- oldtime)>600)){
			  myMotion->stop();
			  myMotion->refreshAll(); 
			  go_to_door_state = 1;
			  
		  }
       
      }
	  else if (Xerror < -40 ){
            delayflag = 0;
            myMotion->turn('L', 1, constrain(abs(Xerror)-40,5,60));

      }
	  else {
		   delayflag = 0;
      myMotion->turn('R', 1,constrain(abs(Xerror)-40,5,60));
      }
	  
      break;
	  
	  case 1:

      go_to_door_state = Approach_Door_mark(Xerr_use,Yvertical_use,pixyParaDoor[4]);  // return 1 or 2
	  break;
		
	  default:
	  break;
}
	  if (go_to_door_state == 2){
		  already_hit_door_mark = 1;
	  }else
	  { already_hit_door_mark = 0;}
  
  
  return already_hit_door_mark;

}

int Approach_Door_mark(float Xerror, float Yvertical, float Area){  //return hit door flag 
	
  
    static float timeNew = 0;
    static float timeOld = 0;
    static float deltaTime = 0; 

  static int output = 0; 
  static float DeltaVec = 0;
  static float BaseVec = 0;
  int hit_door_sidemark = 1;
  
    timeNew=micros();
    if ((timeNew - timeOld)>10000){
    deltaTime = (timeNew - timeOld) / 1e6;  
    timeOld = timeNew;
    Approach_Door_mark_PID->update (0, Xerror, deltaTime, output);
    
    DeltaVec = - output;

    }
	
	static int change_flag = 0;
	if (change_flag == 0){
			if (Area < 12000)
			{
				BaseVec = 60 + 0.15 * Yvertical;
			}else{change_flag = 1;}
	}
	else { 
	     BaseVec = 30;
		
	}

    myMotion->move(1, BaseVec, DeltaVec);
	
    if (LS_FLhit == 1 || LS_FRhit == 1 || LS_Fronthit ==1 ){               // front-left trigger hit
      hit_door_sidemark = 2;
      timeNew = 0;
      timeOld = 0;
      deltaTime = 0; 
      output = 0; 
      DeltaVec = 0;
      BaseVec = 0;
      LS_FLhit = 0;
	  LS_FRhit = 0;
	  LS_Fronthit = 0;
      Approach_Door_mark_PID->refresh();
    }
    else {hit_door_sidemark = 1;}    //which is the mark that the car is close enough to sidemark--> keep approaching
    
    
    return hit_door_sidemark;
  
}


//-----------------go_backward----------------------//
//-----------------go_backward----------------------//
//-----------------go_backward----------------------//

int go_backward_function(float backdistance){
	static int Execute_goback = 1;
	if (Execute_goback == 1){
		Execute_goback = 0;
		countR_ALL = 0;
		countL_ALL = 0;
	}
	Serial.println(countR_ALL * distanceRatio);
	if (countR_ALL * distanceRatio > backdistance && countR_ALL * distanceRatio > backdistance){
		return 1;
	} else {return 0;}
}

//-----------------go_forward----------------------//
//-----------------go_forward----------------------//
//-----------------go_forward----------------------//

int go_forward_function(float forwarddistance){
	static int Execute_goforward = 1;
	if (Execute_goforward == 1){
		Execute_goforward = 0;
		countR_ALL = 0;
		countL_ALL = 0;
	}
	
	if (countR_ALL * distanceRatio > forwarddistance && countR_ALL * distanceRatio > forwarddistance){
		return 1;
	} else {return 0;}
}



// ISR subfuntion
// --------------
void counterL(){
   countL++;
   countL_ALL++;
   
}

void counterR(){
   countR++;
   countR_ALL++;
   
}

void limitswitchFL(){
	LS_FLhit = 1;
	
}
void limitswitchFR(){
	LS_FRhit = 1;
	
}
void limitswitchFront(){
	LS_Fronthit = 1;
}





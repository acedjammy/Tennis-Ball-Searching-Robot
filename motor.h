
#include "Arduino.h"
#include "PIDnew.h"

class motor{
	  public:
		motor(int pin1, int pin2, int pinEn,volatile long *counter);
		float rotate(int wheelspd, int wheeldir);
		void NewPIDpara(float NewKp, float NewKi, float NewKd);
		void refreshMotor();
	  private:
		int _pin1;
		int _pin2;
		int _pinEn;
		
		volatile long *_count;
		
		float timeOld;
		float timeNew; 
		float deltaTime;
		float actual_velocity;
		int output;
		
		
		float Kp = 5;
	    float Ki = 12;
	    float Kd = 0.001;
		
		int deadband[2] = {-35, 35};
	    int range[2] = {0, 255};
		
		
		
		PIDnew *myPID;
};

/*——————————————*/
motor::motor(int pin1, int pin2, int pinEn,volatile long *counter)
		{
		  pinMode(pin1, OUTPUT);
		  pinMode(pin2, OUTPUT);
		  pinMode(pinEn, OUTPUT);
		  _pin1=pin1;
		  _pin2=pin2;
		  _pinEn=pinEn;
		  
		  _count = counter;
		  
		  timeOld=micros();
		  timeNew=0;
		  deltaTime=0;
		  actual_velocity=0;
		  output = 0;
		  
		  myPID = new PIDnew(Kp, Ki, Kd, 5, deadband, range);
		  *_count = 0;

		}


float motor::rotate(int wheelspd,int wheeldir)
{		  
          // Serial.print("countbefore:");
		  // Serial.print(*_count);

		  timeNew=micros();
		  
		  if ((timeNew - timeOld)>10000){
		  deltaTime = (timeNew - timeOld) / 1e6;  
		  timeOld = timeNew;
		  
		  actual_velocity = *_count / 1124.43 / deltaTime *60;    //in rpm
		  
	      // Serial.print("later:");
		  *_count = 0;
		  // Serial.print(*_count);
		  
		  myPID->update (abs(wheelspd), actual_velocity, deltaTime, output);
		  
		  
		  // Serial.print("vec:");
		  // Serial.print(actual_velocity);
		  
		  // Serial.print("output:");
		  // Serial.println(output);
		  
		  }
		 
		  if (wheeldir ==1){
		   digitalWrite(_pin1,HIGH);
		   digitalWrite(_pin2,LOW);
		   analogWrite(_pinEn,output); 
		  }else if (wheeldir== -1){
		   digitalWrite(_pin1,LOW);
		   digitalWrite(_pin2,HIGH);
		   analogWrite(_pinEn,output); 
		  }else{
		   digitalWrite(_pin1,LOW);
		   digitalWrite(_pin2,LOW);
		   analogWrite(_pinEn,0); 
		  }
		return actual_velocity;
}



// void motor::Stop()
// {
		  // digitalWrite(_pin1,LOW);
		  // digitalWrite(_pin2,LOW);
		  // analogWrite(_pinEn,0); 
		  // myPID->refresh();
		  // *_count = 0;
// }

void motor::NewPIDpara(float NewKp, float NewKi, float NewKd)
{
		myPID->change(NewKp, NewKi, NewKd);

}

void motor::refreshMotor()
{
	
	myPID->refresh();
}

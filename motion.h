#include "motor.h"


class motion{
  private:
    
    float leftspeed = 0; float rightspeed = 0;
    float _Kp,  _Ki,  _Kd;
    float actualVecLeft = 0; float actualVecRight = 0;

 public:
    motor *motorLeft;
    motor *motorRight;
    motion(float KpL, float KiL, float KdL, int pinLeftA, int pinLeftB, int pinLeftEn, volatile long *counterLeft,float KpR, float KiR, float KdR,int pinRightA, int pinRightB, int pinRightEn, volatile long *counterRight );

    void move(int dir, float carspeed, float deltaV) ; //deltaV >0 Vleft>Vright, deltaV<0 Vleft<Vright
 
    void turn(char center, int dirflag, float omega);
	
	void stop();
    float getWheelSpeed (char label);
    void refreshAll();

};
motion::motion(float KpL, float KiL, float KdL, int pinLeftA, int pinLeftB, int pinLeftEn, volatile long *counterLeft,float KpR, float KiR, float KdR,int pinRightA, int pinRightB, int pinRightEn, volatile long *counterRight )
  {
    motorLeft = new motor(pinLeftA, pinLeftB, pinLeftEn, counterLeft);
    motorRight = new motor(pinRightA, pinRightB, pinRightEn, counterRight);
    motorLeft->NewPIDpara(KpL,KiL,KdL);
    motorRight->NewPIDpara(KpR,KiR,KdR);
  
  }
void motion::move(int dir, float carspeed, float deltaV)   //deltaV >0 Vleft>Vright, deltaV<0 Vleft<Vright
  {
  leftspeed = carspeed + deltaV / 2;
  rightspeed =carspeed - deltaV / 2;
  actualVecLeft  = motorLeft->rotate(leftspeed , - dir);
  actualVecRight = motorRight->rotate(rightspeed ,dir);
  
  }
  
  # define L  1 
void motion::turn(char center, int dirflag, float omega)   //dirflag==1(cw) or -1(ccw) center= L or R
  {  
	if (center == 'L'){
	actualVecLeft  = motorLeft->rotate(0, 0);
	actualVecRight  = motorRight->rotate(omega * L, dirflag);
	}		
	if (center == 'R'){
	actualVecRight = motorRight->rotate(0, 0);
	actualVecLeft = motorLeft->rotate(omega * L, -dirflag);	
	}
  }
void motion::stop()
{
	actualVecLeft  = motorLeft->rotate(0, 0);
	actualVecRight = motorRight->rotate(0, 0);
	
}

float motion::getWheelSpeed (char label)
  {
    if (label =='L') {return actualVecLeft;}
      else  {return actualVecRight;}
  }

  
void motion::refreshAll()
{	 
    motorLeft->refreshMotor();
    motorRight->refreshMotor();
	
}
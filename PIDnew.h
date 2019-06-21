#include "Arduino.h"

class PIDnew{
  private:
	//they are floats
	float Kp, Ki, Kd, tmp;
	float ek, ek1, ek2, u, dt;
	//they are integers
	int deadBand[2], range[2], flags;

 public:
	PIDnew(float const &Kp, float const &Ki, float const &Kd, int const &flags = 5, int *deadBand = nullptr, int *range = nullptr)
	{
  
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    // this->dt = dt;
    this->tmp = this->Ki;
    // Serial.print("the range is ");
    // Serial.print(range[0]);
    // Serial.print(" ");
    // Serial.println(range[1]);
    if(deadBand != nullptr){

      this->deadBand[0] = deadBand[0];
      this->deadBand[1] = deadBand[1];
    }
    if(range != nullptr){

      this->range[0] = range[0];
      this->range[1] = range[1];
    }
    
    this->flags = flags;
    ek1 = ek2 = u = ek = 0;

  }

	void update(float const &setpoint, float const &input, float const &dt, int &res){
			

	ek2 = ek1;
	ek1 = ek;
	ek = setpoint - input;
	
	// Serial.print("the error:");
	// Serial.print(ek);
	// Serial.print(", ");

	u += Kp * (ek - ek1) + Ki * ek * dt + Kd / dt * (ek - 2 * ek1 + ek2);
	
	//dead band zone
	res = u;
	// Serial.print("V is ");
	// Serial.print(input);
	// Serial.print("original is ");
	// Serial.print(ek);
	// Serial.print(",  ");
	// Serial.print("res is ");
	// Serial.print(res);
	// Serial.print(", ");

	
	if(flags & 4){
	  if(res > 0) res += deadBand[1];
	  else res -= deadBand[0];
	}
	// saturation
	if(flags & 1){
	  if(res > range[1]) res = range[1];
	  if(res < range[0]) res = range[0];
	  //anti-windup
	  if(flags & 2){
		if(res == range[1]){
		  this->Ki = 0;
		}else{
		  this->Ki = this->tmp;
		}
	  }
	}	
	// Serial.print("res2 is ");
	// Serial.print(res);
	// Serial.print(", ");

  }

	void change(float const &_Kp, float const &_Ki, float const &_Kd){
		Kp = _Kp;
		Ki = _Ki;
		Kd = _Kd;
	}

  void refresh(){
    u = ek = ek1 = ek2 = 0;
  }
};

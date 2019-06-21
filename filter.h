#ifndef filter_H_
#define filter_H_
#include "Arduino.h"

class filter {
  public:
   filter (int arraylenth);
   void filterinit();
   float filteroutcome (double newdata);
   
  private:
   int len;
   int index;
   int thisReading;
   float *filterarray;
   float total;  
   float lastdata;
};
#endif

/*——————————————*/
filter::filter(int arraylenth)
{
  len=arraylenth;
  total=0;
  index=0;
  filterarray = new float[arraylenth];
  lastdata=0;
  this->filterinit();
}
void filter::filterinit()
{
   for (thisReading = 0; thisReading < len; thisReading++) 
   {
    filterarray[thisReading] = 0;
  }
}

float filter::filteroutcome (double newdata)
{
  if (abs(newdata-lastdata)<100 || lastdata==0){
  total=total-filterarray[index];
  filterarray[index]=newdata;
  total=total+filterarray[index];  
  index++;
    if (index==len){
        index=0;
      }
  lastdata=newdata;
  }

  return total/len;
}

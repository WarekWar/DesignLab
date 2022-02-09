
#include<ZumoShield.h>
#include<Wire.h>
ZumoBuzzer buzzer;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);

int XY_ACCELERATION_THRESHOLD = 16000;
const int motorSpeed = 120;
int RA_SIZE = 2;
unsigned long loop_start_time;
unsigned long contact_made_time;
int MIN_DELAY_BETWEEN_CONTACTS = 3000;  

template <typename T>
class RunningAverage
{
  public:
    RunningAverage(void);
    RunningAverage(int);
    ~RunningAverage();
    void clear();
    void addValue(T);
    T getAverage() const;
    void fillValue(T, int);
  protected:
    int _size;
    int _cnt;
    int _idx;
    T _sum;
    T * _ar;
    static T zero;
};

class Accelerometer : public ZumoIMU
{
  typedef struct acc_data_xy
  {
    unsigned long timestamp;
    int x;
    int y;
    float dir;
  } acc_data_xy;

  public:
    Accelerometer() : ra_x(RA_SIZE), ra_y(RA_SIZE) {};
    ~Accelerometer() {};
    void getLogHeader(void);
    void readAcceleration(unsigned long timestamp);
    float len_xy() const;
    float dir_xy() const;
    int x_avg(void) const;
    int y_avg(void) const;
    long ss_xy_avg(void) const;
    float dir_xy_avg(void) const;
  private:
    acc_data_xy last;
    RunningAverage<int> ra_x;
    RunningAverage<int> ra_y;
};
Accelerometer acc;


void CountDown()
{
  button.waitForButton();
  for(int i =0; i<3; i++)
  {
    delay(500);
    buzzer.playNote(NOTE_G(3), 250, 10);
    delay(500);
    buzzer.playNote(NOTE_G(4), 250, 10);
  }
  delay(1000);
  buzzer.playNote(NOTE_G(5), 600, 15);
  delay(500);
  
  contact_made_time = 0;
 
}
void setup() 
{
   // Initialize the Wire library and join the I2C bus as a master
  Wire.begin();

  // Initialize accelerometer
  acc.init();
  acc.enableDefault();

#ifdef LOG_SERIAL
  Serial.begin(9600);
  acc.getLogHeader();
#endif
  randomSeed((unsigned int) millis());
  CountDown();
  
}

void loop() 
{
  

  if(button.isPressed())
  {
    motors.setSpeeds(0, 0);
    button.waitForRelease();
    CountDown();
  }
  
  
  loop_start_time = millis();
  if(loop_start_time>contact_made_time+3000)
    XY_ACCELERATION_THRESHOLD = 2400;
  acc.readAcceleration(loop_start_time);
 // Serial.println(acc.dir_xy());
  //Serial.println(contact_detected_where());
  //Serial.println(acc.y_avg());
  //Serial.println(acc.x_avg());
  //delay(800);
 // Serial.println((long) XY_ACCELERATION_THRESHOLD * (long) XY_ACCELERATION_THRESHOLD);
 //Serial.println(check_collision());
  if(check_collision())
    {
      contact_detected();
    }
  else
  { 
  motors.setSpeeds(motorSpeed, motorSpeed);   
  }  
  
}
bool check_collision()
{
  static long threshold_squared = (long) XY_ACCELERATION_THRESHOLD * (long) XY_ACCELERATION_THRESHOLD;
   return(acc.ss_xy_avg() >  threshold_squared)&& \
    (loop_start_time - contact_made_time > MIN_DELAY_BETWEEN_CONTACTS);
   
}
void contact_detected()
{
  
  contact_made_time = loop_start_time;
  contact_response();
  XY_ACCELERATION_THRESHOLD = 16000;
  
} 
int contact_detected_where()
{
  if(acc.dir_xy()>=-110.0 && acc.dir_xy()<=-70.0)
    return 1;   //front
  else if(acc.dir_xy() > -70.0 && acc.dir_xy()< -20.0)
    return 2;    //front, right
  else if(acc.dir_xy() >= -20.0 && acc.dir_xy()<= 20.0)
    return 3;    //right
  else if(acc.dir_xy() > 20.0 && acc.dir_xy()< 70.0)
    return 4;      //back, right 
  else if(acc.dir_xy() >= 70.0 && acc.dir_xy()<= 110.0)
    return 5;      //back   
  else if(acc.dir_xy() > 110.0 && acc.dir_xy()< 160.0)
    return 6;      //back, left  
  else if(acc.dir_xy() > 160.0 || acc.dir_xy()< -160.0)
    return 7;      //left 
  else
    return 8;    //front, left
} 

void contact_response()
{
  int change = contact_detected_where();
  switch(change)
  {
    case 1: //front
      motors.setSpeeds(motorSpeed*2, motorSpeed*2);
      buzzer.playNote(NOTE_G(4), 400, 20);
      delay(1000);
      break;
    case 2:    //front, right
      if(random(0,1)==0)  //flee
      {
        motors.setSpeeds(0, motorSpeed*1.5);
        delay(200);
        buzzer.playNote(NOTE_C(4), 600, 10);
        motors.setSpeeds(motorSpeed*2,motorSpeed*2);
        delay(800);        
      }
      else  //fight!
      {
        motors.setSpeeds(motorSpeed*1.5, 0);
        buzzer.playNote(NOTE_A(4), 600, 10);
        delay(200);
        motors.setSpeeds(motorSpeed*2,motorSpeed*2);
        delay(800);   
      }
      break;
    case 3:    //right
      if(random(0,1)==0) //flee forward
      {
         motors.setSpeeds(motorSpeed*2, motorSpeed*2);
         buzzer.playNote(NOTE_G(4), 400, 20);
         delay(600);
         motors.setSpeeds(motorSpeed, 0);
         buzzer.playNote(NOTE_C_SHARP(4), 400, 10);
         delay(400);  
      }
      else  //flee backward 
      { 
         motors.setSpeeds(-motorSpeed*2, -motorSpeed*2);
         buzzer.playNote(NOTE_B(4), 400, 20);
         delay(600);
         motors.setSpeeds(0, -motorSpeed);
         buzzer.playNote(NOTE_C_SHARP(4), 400, 10);
         delay(400);
      }
      break;  
      case 4:  //back, right
        motors.setSpeeds(motorSpeed*1.5 ,0);
        buzzer.playNote(NOTE_C(4), 400, 20);
        delay(200);
        motors.setSpeeds(motorSpeed, motorSpeed);
        buzzer.playNote(NOTE_G(4), 400, 20);
        delay(800);
        break;
    case 5:  //back
        if(random(0,1)==0) //flee left
        {
           motors.setSpeeds(motorSpeed*2 ,0);
           buzzer.playNote(NOTE_C_SHARP(4), 400, 20);
           delay(200);
           motors.setSpeeds(motorSpeed*1.5, motorSpeed*1.5);
           buzzer.playNote(NOTE_G(4), 400, 20);
           delay(800);
        } 
        else  //flee right
        {
            motors.setSpeeds(0,motorSpeed*2);
           buzzer.playNote(NOTE_C_SHARP(4), 400, 20);
           delay(200);
           motors.setSpeeds(motorSpeed*1.5, motorSpeed*1.5);
           buzzer.playNote(NOTE_G(4), 400, 20);
           delay(800);
        } 
        break;
    case 6:   //back, left
        motors.setSpeeds(0, motorSpeed*1.5);
        buzzer.playNote(NOTE_C_SHARP(4), 400, 20);
        delay(200);
        motors.setSpeeds(motorSpeed, motorSpeed);
        buzzer.playNote(NOTE_G(4), 400, 20);
        delay(800);
        break;
    case 7: //left
        if(random(0,1)==0) //flee forward 
        {
          motors.setSpeeds(motorSpeed*2, motorSpeed*2);
          buzzer.playNote(NOTE_G(4), 400, 20);
          delay(600);
          motors.setSpeeds(0, motorSpeed);
          buzzer.playNote(NOTE_C_SHARP(4), 400, 10);
          delay(400);  
        }
        else  //flee backward
        {
          motors.setSpeeds(-motorSpeed*2, -motorSpeed*2);
          buzzer.playNote(NOTE_B(4), 400, 20);
          delay(600);
          motors.setSpeeds(-motorSpeed, 0);
          buzzer.playNote(NOTE_C_SHARP(4), 400, 10);
          delay(400);
        } 
      break;
    case 8: //front, left
      if(random(0,1)==0)  //flee
      {
        motors.setSpeeds(motorSpeed*1.5, 0);
        delay(600);
        buzzer.playNote(NOTE_C(4), 600, 10);
        motors.setSpeeds(motorSpeed*2, motorSpeed*2);
        delay(400);        
      }
      else  //fight!
      {
        motors.setSpeeds(0, motorSpeed*1.5);
        buzzer.playNote(NOTE_A(4), 600, 10);
        delay(600);
        motors.setSpeeds(motorSpeed*2, motorSpeed*2);
        delay(400);   
      }
      break;
  }
}


void Accelerometer::readAcceleration(unsigned long timestamp)
{
  readAcc();
  if (a.x == last.x && a.y == last.y) return;

  last.timestamp = timestamp;
  last.x = a.x;
  last.y = a.y;

  ra_x.addValue(last.x);
  ra_y.addValue(last.y);
}
long Accelerometer::ss_xy_avg(void) const
{
  long x_avg_long = static_cast<long>(x_avg());
  long y_avg_long = static_cast<long>(y_avg());
  return x_avg_long*x_avg_long + y_avg_long*y_avg_long;
}

int Accelerometer::x_avg(void) const
{
  return ra_x.getAverage();
}

int Accelerometer::y_avg(void) const
{
  return ra_y.getAverage();
}
float Accelerometer::len_xy() const
{
  return sqrt(last.x*a.x + last.y*a.y);
}

float Accelerometer::dir_xy() const
{
  return atan2(last.x, last.y) * 180.0 / M_PI;
}
float Accelerometer::dir_xy_avg(void) const
{
  return atan2(static_cast<float>(x_avg()), static_cast<float>(y_avg())) * 180.0 / M_PI;
}



template <typename T>
T RunningAverage<T>::zero = static_cast<T>(0);

template <typename T>
RunningAverage<T>::RunningAverage(int n)
{
  _size = n;
  _ar = (T*) malloc(_size * sizeof(T));
  clear();
}

template <typename T>
RunningAverage<T>::~RunningAverage()
{
  free(_ar);
}

// resets all counters
template <typename T>
void RunningAverage<T>::clear()
{
  _cnt = 0;
  _idx = 0;
  _sum = zero;
  for (int i = 0; i< _size; i++) _ar[i] = zero;  // needed to keep addValue simple
}

// adds a new value to the data-set
template <typename T>
void RunningAverage<T>::addValue(T f)
{
  _sum -= _ar[_idx];
  _ar[_idx] = f;
  _sum += _ar[_idx];
  _idx++;
  if (_idx == _size) _idx = 0;  // faster than %
  if (_cnt < _size) _cnt++;
}

// returns the average of the data-set added so far
template <typename T>
T RunningAverage<T>::getAverage() const
{
  if (_cnt == 0) return zero; // NaN ?  math.h
  return _sum / _cnt;
}

// fill the average with a value
// the param number determines how often value is added (weight)
// number should preferably be between 1 and size
template <typename T>
void RunningAverage<T>::fillValue(T value, int number)
{
  clear();
  for (int i = 0; i < number; i++)
  {
    addValue(value);
  }
}

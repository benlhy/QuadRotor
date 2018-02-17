#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>
#include <stdint.h>
#include <signal.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <curses.h>

//gcc -o week1 week_1.cpp -lwiringPi -lncurses -lm

#define frequency 25000000.0
#define CONFIG           0x1A
#define SMPLRT_DIV       0x19
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C

#define PWM_MAX 1900
#define frequency 25000000.0
#define LED0 0x6			
#define LED0_ON_L 0x6		
#define LED0_ON_H 0x7		
#define LED0_OFF_L 0x8		
#define LED0_OFF_H 0x9		
#define LED_MULTIPLYER 4

enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};
 
enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};
/*
struct Keyboard {
  char key_press;
  int heartbeat;
  int version;
  float pitch;
  float roll;
  float yaw;
  float thrust;
};*/

/// This is the new struct for MILESTONE /////

struct Keyboard {
  int keypress;
  float pitch;
  float roll;
  float yaw;
  float thrust;
  int version;
};

Keyboard* shared_memory; 

// Prototypes goes here
int setup_imu();
void setup_keyboard();
void calibrate_imu();      
void read_imu();    
void update_filter();
void trap(int signal);
//void read_keyboard(Keyboard keyboard);
void safety_check();
void init_pwm();
void init_motor(uint8_t channel);
void set_PWM( uint8_t channel, float time_on_us);
void pid_update(int desired_pitch, int desired_roll);
int limit_speed(int thepwm);
void get_update();
// MILESTONE for WEEK 4//
void get_joystick(Keyboard keyboard);

//global variables
int imu;
float x_gyro_calibration=0;
float y_gyro_calibration=0;
float z_gyro_calibration=0;
float roll_calibration=0;
float pitch_calibration=0;
float accel_z_calibration=0;
float imu_data[6]; //gyro xyz, accel xyz
long time_curr;
long time_prev;
struct timespec te;
float yaw=0;
float pitch_angle=0;
float roll_angle=0;
int pwm; // don't initialise! Used by something else
int quad_thrust = 1500; // neutral speed
int quad_base_thrust = quad_thrust;



float joy_pitch=0;
float joy_roll=0;
float joy_yaw=0;
float joy_thrust=0;


int run_program=1;


// Added variables
float roll_angle_acc = 0;
float pitch_angle_acc = 0;
float roll_angle_gyro = 0;
float pitch_angle_gyro = 0;

// Flip pitch and roll
float real_pitch_angle = 0;
float real_roll_angle = 0;

// kill variables
int last_seen = 0;
int last_version = 0;
long last_time = 0;
long curr_time= 0;
long init_time=0;

// pwm
int pitch_integral_term = 0;
int prev_error = 0;
int roll_integral_term = 0;
int prev_d_error = 0;


int pwm0 = 1100;
int pwm1 = 1100;
int pwm2 = 1100;
int pwm3 = 1100;

struct timeval myte; // struct to store the time

// small change

// struct timeval graphTimerS;
// struct timeval graphTimerE;
// float graphTime;

int main (int argc, char *argv[])
{
    

    setup_imu();
    calibrate_imu();
    //in main before while(1) loop add...
    setup_keyboard();
    signal(SIGINT, &trap); // instant interrupt!
    
    init_pwm();
    init_motor(0);
    init_motor(1);
    init_motor(2);
    init_motor(3);
    delay(1000);
    
    set_PWM(0,1000);//speed between 1000 and PWM_MAX, motor 0-3 
    set_PWM(1,1000);//speed between 1000 and PWM_MAX, motor 0-3
    set_PWM(2,1000);//speed between 1000 and PWM_MAX, motor 0-3
    set_PWM(3,1000);//speed between 1000 and PWM_MAX, motor 0-3*/
    // getimeofday(&graphTimerS,NULL);
 
    //printf("Final_Roll, Acc_Roll, Gyro_roll, Final_Pitch, Acc_Pitch, Gyro_Ptich\n");
    printf("Time \t PWM 0 \t PWM 1 \t  PWM 2 \t PWM 3 \t Current Roll \t Quad Thrust \t Joy Pitch\r\n");
    gettimeofday(&myte,NULL);
    last_time = myte.tv_sec*1000LL+myte.tv_usec/1000; // first set the time to current time
    init_time = last_time;
    while(run_program==1)
    {
      gettimeofday(&myte,NULL); // get time of day
      curr_time = myte.tv_sec*1000LL+myte.tv_usec/1000; // update current time.
      


      //Once working, comment this if loop out.
      /*if(curr_time>last_time+250){
        printf("Keyboard timeout\r\n");
        run_program=0; // Shinu
        break;
      }*/

      Keyboard keyboard=*shared_memory;

      //read_keyboard(keyboard); // this function only updates last time if it detects a heartbeat
      get_joystick(keyboard);   
      //printf("%d \r\n", keyboard.heartbeat); 
      read_imu();
      roll_angle_acc = atan2(imu_data[4],-imu_data[5])*180/M_PI - roll_calibration;
      pitch_angle_acc = atan2(imu_data[3],-imu_data[5])*180/M_PI - pitch_calibration;
      update_filter();

      
      
      safety_check();

      // This should be the only location that motor speed is varied. All other places should be zeroing the motor.

      pid_update(joy_pitch,0); // set desired pitch angle and roll angle


      ////////////// MILESTONE WEEK 4 CHECK //////////////////
      // pid_update(joy_pitch); // This is our desired pitch

      /////////////// MILESTONE WEEK4 CHECK ////////////////////
      // pid_update(joy_pitch, joy_roll); // This is our desired roll;

      
      //      GX, GY, GZ, Acc_Roll, Acc_Pitch, Final_Roll, Final_Pitch
      //printf("%f, %f, %f, %f, %f, %f, %f \r\n",imu_data[0],imu_data[1],imu_data[2],roll_angle_acc,pitch_angle_acc,roll_angle,pitch_angle); 
     //printf("%f,%f,%f,%f,%f,%f \r\n", roll_angle,roll_angle_acc,roll_angle_gyro,pitch_angle,pitch_angle_acc,pitch_angle_gyro);
     //printf("%f,%f \r\n", roll_angle_gyro,pitch_angle_gyro);
     //printf("%f,%f \r\n", roll_angle_acc,pitch_angle_acc);
     //printf("%f, %f,%f \r\n", roll_angle, roll_angle_gyro,roll_angle_acc);
     //printf("%f, %f,%f \r\n", roll_angle, imu_data[0], roll_angle_acc);
     //printf("%f, %f,%f \r\n", pitch_angle, imu_data[1],pitch_angle_acc);

     

      // gettimeofday(&graphTimerE,NULL);
      // graphTime=(graphTimerE*1000LL+graphTimerE/1000)-(graphTimerS*1000LL+graphTimerS/1000);

<<<<<<< HEAD
       printf("%f \t %d \t %d \t %d \t %d \t %f \t %d \t %f\r\n",(curr_time-init_time)/1000.0, pwm0,pwm1,pwm2,pwm3, real_roll_angle,quad_thrust,joy_pitch); // Mojojojojo
=======
       printf("%d %d %d %d %f %f\r\n",pwm0,pwm1,pwm2,pwm3, real_pitch_angle, (curr_time-init_time)/1000.0); // Mojojojojo
>>>>>>> origin/master

      ///////////////////////////////////////////////////////////////////////////
     
    }
    printf("Mojojojojo.");

      
   return 0;
  
}

void pid_update(int desired_pitch, int desired_roll){
  // PID control values
<<<<<<< HEAD
  float Kp = 7;
  float Kd = 120;
  float Ki = 0.003;
  float Kpr = 6.5;//best: 6.5;//8;
  float Kdr = 90;//best: 90;//200;
  float Kir = 0.005;//best: 0.005;//0.02;
=======
  float Kp = 8;
  float Kd = 100;
  float Ki = 0.01;
  float Kpr = 0;//8;
  float Kdr = 0;//200;
  float Kir = 0;//0.02;
>>>>>>> origin/master
  // int neutral_power=1100; // replaced with global thrust
  float pitch_error = 0;
  float roll_error = 0;
  float pitch_d_error, pitch_i_error;
  float roll_d_error,roll_i_error;

  
  // Errors
  pitch_error = desired_pitch - real_pitch_angle;
  pitch_d_error = prev_error - pitch_error;
  //printf("D_error: %f",pitch_d_error);
  pitch_integral_term = pitch_integral_term + pitch_error;
  pitch_i_error = pitch_integral_term;
  //printf("pitch: %f pitch error: %f pitch_i_error:%f \r\n",real_pitch_angle, pitch_error,pitch_i_error);
  
  ///////////////////////////// MILESTONE for WEEK 4 /////////////////////////////

  // calculate but do nothing with them at the moment.
  roll_error = desired_roll - real_roll_angle;
  roll_d_error = prev_d_error - roll_error;
  roll_integral_term = roll_integral_term + roll_error;
  roll_i_error = roll_integral_term;
  //printf("roll: %f roll error: %f roll d error: %f roll_i_error: %f \r\n",real_roll_angle,roll_error, roll_d_error, roll_i_error);
  ///////////////////////////////////////////////////////////////////////////////

  //pitch_velocity_error = desired_pitch_velocity - imu_data[1]; //imudata[1] was originally roll, but roll is pitch
  //roll_velocity_error = desired_roll_velocity - imu_data[0];
  //pitch_integral_term = pitch_integral_term+I*pitch_error; // Integrating over time

  // Ensure that integral doesn't get out of control.
  if (pitch_i_error*Ki<-50){
    pitch_i_error=-50/Ki;
  }
  else if (pitch_i_error*Ki>100){
    pitch_i_error = 100/Ki;
  }
  /////////////////////// MILESTONE for WEEK 4 ///////////////////////////
  
  if (roll_i_error*Kir<-50){
    pitch_i_error=-50/Kir;
  }
  else if (roll_i_error*Kir>100){
    pitch_i_error = 100/Kir;
  }
  
  /////////////////////////////////////////////////////////////////////


  /////////////////////////////// MILESTONE ///////////////////////////////////
  //////// Uncomment this line by line to reach each milestone ////////////////

<<<<<<< HEAD
  // pwm0 = quad_thrust + pitch_error*Kp - pitch_d_error*Kd + pitch_i_error*Ki;
  // pwm1 = quad_thrust - pitch_error*Kp + pitch_d_error*Kd - pitch_i_error*Ki;
  // pwm2 = quad_thrust + pitch_error*Kp - pitch_d_error*Kd + pitch_i_error*Ki;
  // pwm3 = quad_thrust - pitch_error*Kp + pitch_d_error*Kd - pitch_i_error*Ki;
=======
  pwm0 = quad_thrust + pitch_error*Kp - pitch_d_error*Kd + pitch_i_error*Ki;
  pwm1 = quad_thrust - pitch_error*Kp + pitch_d_error*Kd - pitch_i_error*Ki;
  pwm2 = quad_thrust + pitch_error*Kp - pitch_d_error*Kd + pitch_i_error*Ki;
  pwm3 = quad_thrust - pitch_error*Kp + pitch_d_error*Kd - pitch_i_error*Ki;
>>>>>>> origin/master
  //printf("P: %f, D: %f, I: %f",pitch_error*Kp,pitch_d_error*Kd,pitch_i_error*Ki);
  // pwm2 = quad_thrust + roll_error*Kpr - roll_d_error*Kdr + roll_i_error*Kir;
  // pwm3 = quad_thrust + roll_error*Kpr - roll_d_error*Kdr + roll_i_error*Kir;
  // pwm1 = quad_thrust - roll_error*Kpr + roll_d_error*Kdr - roll_i_error*Kir;
  // pwm0 = quad_thrust - roll_error*Kpr + roll_d_error*Kdr - roll_i_error*Kir;

  //////////////////////////// MILESTONE WEEK 4 ////////////////////////////////
  //  Might want to change the negative signs and positive signs in roll
  //
  // pwm1 = quad_thrust + pitch_error*Kp + pitch_d_error*Kd+pitch_i_error*Ki + roll_error*Kpr + roll_d_error*Kdr + roll_i_error*Kir;
  // pwm0 = quad_thrust - pitch_error*Kp - pitch_d_error*Kd-pitch_i_error*Ki + roll_error*Kpr + roll_d_error*Kdr + roll_i_error*Kir;
  // pwm3 = quad_thrust + pitch_error*Kp + pitch_d_error*Kd+pitch_i_error*Ki - roll_error*Kpr - roll_d_error*Kdr - roll_i_error*Kir;
  // pwm2 = quad_thrust - pitch_error*Kp - pitch_d_error*Kd-pitch_i_error*Ki - roll_error*Kpr - roll_d_error*Kdr - roll_i_error*Kir;
  //
  //
  //
  //
  //
  //
  //
  //////////////////////////////////////////////////////////////////////////////

  // Limit speed
  pwm0=limit_speed(pwm0);
  pwm1=limit_speed(pwm1);
  pwm2=limit_speed(pwm2);
  pwm3=limit_speed(pwm3);


  set_PWM(0,pwm0); // might need to flip the signs for P
  set_PWM(1,pwm1);
  set_PWM(2,pwm2);
  set_PWM(3,pwm3);

  prev_error = pitch_error;// update prev error
  prev_d_error = roll_error; // update prev error for roll
}

int limit_speed(int thepwm){
  if (thepwm>PWM_MAX){
    return PWM_MAX;
  }
  if (thepwm<1100){
    return 1100;
  }
  else{
    return thepwm;
  }
}
void get_joystick(Keyboard keyboard){

  if (keyboard.version != last_version){
      // if not last version, update all values!!!!!!! 
      //printf("key press:%c, keyboard heatbeat:%d keyboard version:%d \r\n",keyboard.keypress, keyboard.version);
      //printf("key press:%d keyboard version:%ld \r\n", keyboard.version);
      if (keyboard.keypress =='\"'){
        printf("X pressed! "); // do unpause
      }
      else if (keyboard.keypress == '!'){
        printf("B pressed! "); // do pause
      }
      else if(keyboard.keypress == '#' ){
        printf("Y pressed! "); // do calibration
      }


      last_version = keyboard.version; 

      //MOJOJO TIME
      joy_thrust = keyboard.thrust; // update thrust // up is 16
      joy_yaw = keyboard.yaw; // update yaw // right is high
      joy_roll = keyboard.roll; //  update roll // right is high
      joy_pitch = 20*(keyboard.pitch-128)/128; // update pitch // up is 16
      //printf("Thrust: %f, Yaw: %f, Roll,: %f, Pitch: %f",joy_thrust,joy_roll,joy_roll,joy_pitch);
<<<<<<< HEAD
      quad_thrust= quad_base_thrust+ (int)joy_thrust; // update from base thrust.
=======
      // quad_thrust= quad_base_thrust+ (int)joy_thrust; // update from base thrust.
>>>>>>> origin/master
      // printf("Quad_thrust: %d Quad pitch: %f \r\n", joy_thrust,joy_pitch);


      
      //////////////////////// MILESTONE TO CHECK ////////////////////////////////

      if (keyboard.keypress==' '){ // CHECK if this is the right syntax 
        run_program=0;
        printf("Space pressed\r\n");
        // cannot break from here :(, only can set run_program=0
      }

      /////////////////////////////////////////////////////////////////////////
      
    }

}


/*
void read_keyboard(Keyboard keyboard){
  if (keyboard.heartbeat != last_seen){ // if heartbeat is not equal to the last heartbeat, means that this is a new heartbeat

    last_seen=keyboard.heartbeat;

    if (keyboard.version != last_version){
      printf("key press:%c, keyboard heatbeat:%d keyboard version:%d \r\n",keyboard.key_press, keyboard.heartbeat, keyboard.version);
      last_version = keyboard.version; 
      
      //////////////////////// MILESTONE TO CHECK ////////////////////////////////

      if (keyboard.key_press==' '){ // CHECK if this is the right syntax to compare space
        run_program=0;
        printf("Space pressed\r\n");
        // cannot break from here :(, only can set run_program=0
      }

      /////////////////////////////////////////////////////////////////////////
      
    }

    last_time = curr_time; // update the time.
  }
    
}*/


void safety_check(){
  // CHECK convert to gs?
  int stay_on = 1;
  if (imu_data[3]>17.6 or imu_data[4]>17.6 or imu_data[5]>17.6){
    //printf("Impact!");
    //stay_on = 0; //shinu
  }
  if (imu_data[3]<2.45 and imu_data[4]<2.45 and imu_data[5]>2.45){
    //printf("Free fall!");
    //stay_on = 0; //shinu
  }
  if (roll_angle>45 or roll_angle<-45){
    //printf("Pitch fail!");
    //stay_on = 0; // shinu
  }
  if (pitch_angle>45 or pitch_angle<-45){
    //printf("Roll fail!");
    //stay_on = 0; // shinu
  }
  if (imu_data[0]>300 or imu_data[1]>300 or imu_data[2]>300){
    //printf("Spinning too fast!");
    stay_on = 0; 
  }
  //if(curr_time>last_time+250){
    //printf("Keyboard timeout");
    //run_program=0; 
  //}
  /*if (keyboard.key_press==' '){ // CHECK if this is the right syntax to compare space
    run_program=0;
    printf("Space pressed"); // Mojojojojo
  }*/
  if (stay_on == 0){
    // Cut our speed.
    set_PWM(0,1000);//speed between 1000 and PWM_MAX, motor 0-3
    set_PWM(1,1000);//speed between 1000 and PWM_MAX, motor 0-3
    set_PWM(2,1000);//speed between 1000 and PWM_MAX, motor 0-3
    set_PWM(3,1000);//speed between 1000 and PWM_MAX, motor 0-3
    run_program = 0; //Terminate program.
  
  }

}


void trap(int signal)

{
    //printf("here");
    set_PWM(0,1000);//speed between 1000 and PWM_MAX, motor 0-3
    set_PWM(1,1000);//speed between 1000 and PWM_MAX, motor 0-3
    set_PWM(2,1000);//speed between 1000 and PWM_MAX, motor 0-3
    set_PWM(3,1000);//speed between 1000 and PWM_MAX, motor 0-3

   printf("Control-C pressed!\n\r");
   run_program=0;
}
 





void setup_keyboard()
{

  int segment_id;   
  struct shmid_ds shmbuffer; 
  int segment_size; 
  const int shared_segment_size = 0x6400; 
  int smhkey=33222;
  
  /* Allocate a shared memory segment.  */ 
  segment_id = shmget (smhkey, shared_segment_size,IPC_CREAT | 0666); 
  /* Attach the shared memory segment.  */ 
  shared_memory = (Keyboard*) shmat (segment_id, 0, 0); 
  printf ("shared memory attached at address %p\n", shared_memory); 
  /* Determine the segment's size. */ 
  shmctl (segment_id, IPC_STAT, &shmbuffer); 
  segment_size  =               shmbuffer.shm_segsz; 
  printf ("segment size: %d\n", segment_size); 
  /* Write a string to the shared memory segment.  */ 
  //sprintf (shared_memory, "test!!!!."); 

}


void calibrate_imu()
{
  double myaverage1 = 0;
  double myaverage2 = 0;
  double myaverage3 = 0;
  double myaverage4 = 0;
  double myaverage5 = 0;
  double myaverage6 = 0;
  for (int i = 0; i<1000; i++){
    read_imu();
    myaverage1 = myaverage1 + imu_data[0]; // our gyro x
    myaverage2 = myaverage2 + imu_data[1]; // our gyro y
    myaverage3 = myaverage3 + imu_data[2]; // our gyro y
    myaverage4 =myaverage4 +atan2(imu_data[4],-imu_data[5]);
    myaverage5=myaverage5 + atan2(imu_data[3],-imu_data[5]);
    myaverage6 = myaverage6 + imu_data[5]; // our gyro z
    //printf("IMU values are %f",imu_data[0]);
  }
  x_gyro_calibration = myaverage1/1000.0;
  y_gyro_calibration = myaverage2/1000.0;
  z_gyro_calibration = myaverage3/1000.0;
  roll_calibration=(myaverage4/1000.0)*180/M_PI;
  pitch_calibration=(myaverage5/1000.0)*180/M_PI;
  accel_z_calibration=(myaverage6/1000.0);

  
  
  
printf("calibration complete, %f %f %f %f %f %f\n\r",x_gyro_calibration,y_gyro_calibration,z_gyro_calibration,roll_calibration,pitch_calibration,accel_z_calibration);


}

void read_imu()
{
  int address=0x3B;//todo: set address value for accel x value 
  float ax=0;
  float az=0;
  float ay=0; 
  int vh,vl;
  float holdv=0.0;
  
  //read in data
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  //convert 2 complement
  int vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  holdv=2*9.81*vw/(32768.0);          
  imu_data[3]=holdv;//  todo: convert vw from raw values to "g's"
  //printf("%d",vw);
  
  
  address=0x3D;//todo: set address value for accel y value
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  holdv=2*9.81*vw/(32768.0);           
  imu_data[4]=holdv;//Todo: convert vw from raw valeus to "g's"
  
  
  address=0x3F;//todo: set addres value for accel z value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }    
  holdv=2*9.81*vw/(32768.0);       
  imu_data[5]=holdv;//todo: convert vw from raw values to g's
  
  
  address=0x43;//todo: set addres value for gyro x value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  holdv=-500.0*vw/(32768.0);
  //printf("Our raw gyrox is %d\r\n",vw);          
  imu_data[0]=-x_gyro_calibration+holdv;////todo: convert vw from raw values to degrees/second
  
  
  address=0x45;//todo: set addres value for gyro y value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  holdv=500.0*vw/(32768.0);
  //printf("Our raw gyro y is %d\r\n",vw);           
  imu_data[1]=-y_gyro_calibration+holdv;////todo: convert vw from raw values to degrees/second
  
  address=0x47;////todo: set addres value for gyro z value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  holdv=500.0*vw/(32768.0);
  //printf("Our raw gyro z is %d\r\n",vw);              
  imu_data[2]=-z_gyro_calibration+holdv;////todo: convert vw from raw values to degrees/second
}

void update_filter()
{

  //get current time in nanoseconds
  timespec_get(&te,TIME_UTC);
  time_curr=te.tv_nsec;
  //compute time since last execution
  float imu_diff=time_curr-time_prev;           
  
  //check for rollover
  if(imu_diff<=0)
  {
    imu_diff+=1000000000;
  }
  //convert to seconds
  imu_diff=imu_diff/1000000000;
  time_prev=time_curr;
 //comp. filter for roll, pitch here: 

  float confidence = 0.01;//0.00025;

  // we already calculated the roll from the accelerometer data, so we take the global roll and combine it with that, and update the global roll
  
  roll_angle = roll_angle_acc *confidence+(1.0-confidence)*(imu_data[0]*imu_diff+roll_angle); // roll_angle is global and x
   
  
  pitch_angle = pitch_angle_acc*confidence+(1.0-confidence)*(imu_data[1]*imu_diff+pitch_angle); // pitch_angle is global and y
  roll_angle_gyro = imu_data[0]*imu_diff+roll_angle_gyro;
  pitch_angle_gyro = imu_data[1]*imu_diff+pitch_angle_gyro;

  // I'm calling pitch roll and roll pitch. Wassup!
  real_pitch_angle = roll_angle;
  real_roll_angle = pitch_angle;
  
 

}


int setup_imu()
{
  wiringPiSetup ();
  
  
  //setup imu on I2C
  imu=wiringPiI2CSetup (0x68) ; //accel/gyro address
  
  if(imu==-1)
  {
    printf("-----cant connect to I2C device %d --------\n",imu);
    return -1;
  }
  else
  {
  
    printf("connected to i2c device %d\n",imu);
    printf("imu who am i is %d \n",wiringPiI2CReadReg8(imu,0x75));
    
    uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
    uint8_t Gscale = GFS_500DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
    
    
    //init imu
    wiringPiI2CWriteReg8(imu,PWR_MGMT_1, 0x00);
    printf("                    \n\r");
    wiringPiI2CWriteReg8(imu,PWR_MGMT_1, 0x01);
    wiringPiI2CWriteReg8(imu, CONFIG, 0x00);  
    wiringPiI2CWriteReg8(imu, SMPLRT_DIV, 0x00); //0x04        
    int c=wiringPiI2CReadReg8(imu,  GYRO_CONFIG);
    wiringPiI2CWriteReg8(imu,  GYRO_CONFIG, c & ~0xE0);
    wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c & ~0x18);
    wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c | Gscale << 3);       
    c=wiringPiI2CReadReg8(imu, ACCEL_CONFIG);
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c | Ascale << 3);      
    c=wiringPiI2CReadReg8(imu, ACCEL_CONFIG2);         
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG2, c & ~0x0F); //
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG2,  c | 0x00);
  }
  return 0;
}

void init_pwm()
{

    pwm=wiringPiI2CSetup (0x40);
    if(pwm==-1)
    {
      printf("-----cant connect to I2C device %d --------\n",pwm);
     
    }
    else
    {
  
      float freq =400.0*.95;
      float prescaleval = 25000000;
      prescaleval /= 4096;
      prescaleval /= freq;
      prescaleval -= 1;
      uint8_t prescale = floor(prescaleval+0.5);
      int settings = wiringPiI2CReadReg8(pwm, 0x00) & 0x7F;
      int sleep	= settings | 0x10;
      int wake 	= settings & 0xef;
      int restart = wake | 0x80;
      wiringPiI2CWriteReg8(pwm, 0x00, sleep);
      wiringPiI2CWriteReg8(pwm, 0xfe, prescale);
      wiringPiI2CWriteReg8(pwm, 0x00, wake);
      delay(10);
      wiringPiI2CWriteReg8(pwm, 0x00, restart|0x20);
    }
}

void init_motor(uint8_t channel)
{
	int on_value=0;

	int time_on_us=900;
	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

	 time_on_us=1200;
	 off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

	 time_on_us=1000;
	 off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

}


void set_PWM( uint8_t channel, float time_on_us)
{
  if(run_program==1)
  {
    if(time_on_us>PWM_MAX)
    {
      time_on_us=PWM_MAX;
    }
    else if(time_on_us<1000)
    {
      time_on_us=1000;
    }
  	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
  	wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,off_value);
  }
}

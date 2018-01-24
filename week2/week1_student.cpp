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

struct Keyboard {
  char key_press;
  int heartbeat;
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
void read_keyboard(Keyboard keyboard);
void safety_check();

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


int run_program=1;


// Added variables
float roll_angle_acc = 0;
float pitch_angle_acc = 0;
float roll_angle_gyro = 0;
float pitch_angle_gyro = 0;
int last_seen = 0;
int last_version = 0;
long last_time = 0;
long curr_time= 0;

struct timeval myte; // struct to store the time

int main (int argc, char *argv[])
{
    

    setup_imu();
    calibrate_imu();
    //in main before while(1) loop add...
    setup_keyboard();
    signal(SIGINT, &trap); // instant interrupt!
    

 
    printf("Final_Roll, Acc_Roll, Gyro_roll, Final_Pitch, Acc_Pitch, Gyro_Ptich\n");
    gettimeofday(&myte,NULL);
    last_time = myte.tv_sec*1000LL+myte.tv_usec/1000; // first set the time to current time
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
      read_keyboard(keyboard); // this function only updates last time if it detects a heartbeat

      //printf("%d \r\n", keyboard.heartbeat); 
      read_imu();
      roll_angle_acc = atan2(imu_data[4],-imu_data[5])*180/M_PI - roll_calibration;
      pitch_angle_acc = atan2(imu_data[3],-imu_data[5])*180/M_PI - pitch_calibration;
           
      update_filter();   
      // //////////////////////MILESTONE TWO TO CHECK////////////////////////////
      
      safety_check();

      ///////////////////////////////////
      //      GX, GY, GZ, Acc_Roll, Acc_Pitch, Final_Roll, Final_Pitch
      //printf("%f, %f, %f, %f, %f, %f, %f \r\n",imu_data[0],imu_data[1],imu_data[2],roll_angle_acc,pitch_angle_acc,roll_angle,pitch_angle); 
     //printf("%f,%f,%f,%f,%f,%f \r\n", roll_angle,roll_angle_acc,roll_angle_gyro,pitch_angle,pitch_angle_acc,pitch_angle_gyro);
     //printf("%f,%f \r\n", roll_angle_gyro,pitch_angle_gyro);
     //printf("%f,%f \r\n", roll_angle_acc,pitch_angle_acc);
     //printf("%f, %f,%f \r\n", roll_angle, roll_angle_gyro,roll_angle_acc);
     //printf("%f, %f,%f \r\n", roll_angle, imu_data[0], roll_angle_acc);
     //printf("%f, %f,%f \r\n", pitch_angle, imu_data[1],pitch_angle_acc);
    }
      
     return 0;
    
   
  
}

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
    
}


void safety_check(){
  // CHECK convert to gs?
  if (imu_data[3]>17.6 or imu_data[4]>17.6 or imu_data[5]>17.6){
    printf("Impact!");
    run_program=0; //shinu
  }
  if (imu_data[3]<2.45 and imu_data[4]<2.45 and imu_data[5]>2.45){
    printf("Free fall!");
    run_program=0; //shinu
  }
  if (roll_angle>45 or roll_angle<-45){
    printf("Roll fail!");
    run_program=0; // shinu
  }
  if (pitch_angle>45 or pitch_angle<-45){
    printf("Pitch fail!");
    run_program=0; // shinu
  }
  if (imu_data[0]>300 or imu_data[1]>300 or imu_data[2]>300){
    printf("Spinning too fast!");
    run_program=0; //ksu
  }
  if(curr_time>last_time+250){
    printf("Keyboard timeout");
    run_program=0; 
  }
  /*if (keyboard.key_press==' '){ // CHECK if this is the right syntax to compare space
    run_program=0;
    printf("Space pressed"); // Mojojojojo
  }*/

}


void trap(int signal)

{
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

  float confidence = 0.2;

  // we already calculated the roll from the accelerometer data, so we take the global roll and combine it with that, and update the global roll
  
  roll_angle = roll_angle_acc *confidence+(1.0-confidence)*(imu_data[0]*imu_diff+roll_angle); // roll_angle is global and x
   
  
  pitch_angle = pitch_angle_acc*confidence+(1.0-confidence)*(imu_data[1]*imu_diff+pitch_angle); // pitch_angle is global and y
  roll_angle_gyro = imu_data[0]*imu_diff+roll_angle_gyro;
  pitch_angle_gyro = imu_data[1]*imu_diff+pitch_angle_gyro;
  
 

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
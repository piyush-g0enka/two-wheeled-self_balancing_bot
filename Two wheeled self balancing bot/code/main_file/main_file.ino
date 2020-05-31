 /*    
  * Filename:main_file      
  * Functions: setup, loop, TImer_1_ISR, timer_interrupt_setup        
  */
  

#include "I2Cdev.h" 
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif



///////////////////////  MOTOR /////////////////////////

# define RM1 6
# define RM2 7
# define LM1 4
# define LM2 5
# define LMEN 9
# define RMEN 8


int pwm =0;
/////////////////////// END   MOTOR /////////////////////////



/////////////////////// ENCODER ////////////////////////////////

// pins for the encoder inputs
#define RH_ENCODER_A 3 
#define RH_ENCODER_B 11

#define LH_ENCODER_A 2
#define LH_ENCODER_B 10

signed long rightCount = 0,leftCount = 0;
float angle_r = 0,angle_l = 0; 
float angle_r0 = 0,angle_l0 = 0;
float angle_r_dt = 0,angle_l_dt = 0; 
float rpm_r =0, rpm_l = 0;
float phi_dot_l = 0;
float phi = 0, phi_dot = 0;
float pwm_l =0;
/////////////////////// END ENCODER ////////////////////////////////


/////////////////////// MPU6050  ////////////////////////////////

MPU6050 accelgyro;
int16_t ax=0, ay=0, az=0 ,ax_l=0, ay_l=0, az_l=0;
int16_t gx=0, gy=0, gz=0 ,gx0=0, gy0=0, gz0=0, gx_h=0 , gy_h=0, gz_h=0;
float angle_0=0, angle_1=0, angle_1_l=0, d_angle=0,  t_dot =0 , t_dot_l=0 ;
float theta = 0, theta_dot = 0;
float ang_offset = 7.6;
/////////////////////// END MPU6050 ////////////////////////////////


////////////////////////// PID //////////////////////////////////////

float setpoint =0;
float Kp_angle=80, Kd_angle=5.0,  Ki_angle=0 ;
float integralSum=0;
float output_l =0;
float Kp_encoder=0, Kd_encoder=0.36,  Ki_encoder=0 ;
////////////////////////// END PID //////////////////////////////////////


void setup() {
  

   encoder_setup();
   motor_setup();
   sensor_setup();
   timer_interrupt_setup ();
   
   //Serial2.begin(9600);
   //Serial.begin(115200);

}


///////////////////////////////// START ISR ////////////////////////////////////////////
ISR(TIMER1_COMPA_vect){

 theta =  get_theta_value();
theta_dot = get_theta_dot_value();
 phi =  get_phi_value();
 phi_dot =  get_phi_dot_value();
 
 setpoint =    PID_encoder ( phi,  phi_dot, Kp_encoder,  Kd_encoder) ;
 
 pwm = PID_angle ( setpoint,  theta, theta_dot,  Kp_angle ,  Kd_angle ,  Ki_angle); 

 
 motors_actuate ( pwm );

 sei();
 }



///////////////////////////////// END ISR ////////////////////////////////////////////

void loop() {

/*
  if (Serial2.available()> 2)
  {
/*
   
   Kp_angle = Serial2.read()-48;
   Kp_angle = Kp_angle*10 + (Serial2.read()-48) ;
       Serial2.read();

    Kd_angle = Serial2.read()-48;
   Kd_angle = Kd_angle + (Serial2.read()-48)*0.1 ;
       Serial2.read();


       
   Serial2.print(Kp_angle); Serial2.print("\t"); Serial2.println(Kd_angle);   

   

    motor_offset = 1 + (Serial2.read()-48)*0.1;
   motor_offset= motor_offset + (Serial2.read()-48)*0.01 ;
       Serial2.read();
       
    Serial2.println(motor_offset);

  }
  
 //Serial.print(theta); Serial.print("\n");
 // Serial.print(theta_dot); Serial.print("\n");
   //Serial.print(phi); Serial.print("\n");
    //Serial.print(phi_dot); Serial.print("\n");
    //Serial.print(pwm); Serial.print("\n");
 delay(10);
 */
 
}













///////////////////////////////// TIMER INTERRUPT FUNCTION ///////////////////////////////////////

void timer_interrupt_setup (void)
{

  cli();//stop interrupts

//set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 19999;// = (16*10^6) / (100*8) - 1 (must be <65536)  , 19999 = 10 ms
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bits for 1024 prescaler = 8
 
  TCCR1B |= (1 << CS11) ;   
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

sei();//allow interrupts

}

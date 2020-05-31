 /*
  * Filename: PID           
  * Functions: PID_angle, PID_encoder   
 */ 


///////////////////////////////// PID FUNCTIONS ///////////////////////////////////////

/**********************************
Function name  : PID_angle
Functionality : To compute the PWM value to set the motor speed using a PID Controller
                Process variable is tilt angle
Return Value  : pwm value
Example Call  : PID_angle()
***********************************/

float PID_angle (float setpoint, float theta, float theta_dot, float Kp, float Kd, float Ki) 

{
  float theta_error;
  float output;
  
  theta_error = theta - setpoint;        ///////////////////////////// error 


  
   
  float proportional = theta_error * Kp ;   ////////////////////////// Proportional
  
  integralSum = integralSum + theta_error* 0.1 ;   ////////// Integral
  
  float integral = Ki * integralSum ;
  
  integral = constrain (integral, -255, 255);
  
  float derivative = Kd * theta_dot ;  //////////////////////////////// Derivative


 //Serial.print(proportional); Serial.print("\t"); Serial.print(derivative); Serial.print("\t"); Serial.print(integral); Serial.print("\n");
  
  output = proportional + derivative + integral;



  if (abs(theta)>45)    // if bot is at an angle > 45 degrees, turn off motors because bot is going to fall anyways
  {
    output =0;
  }


 
  output = constrain (output, -255, 255);
   
  return (output);
}




/**********************************
Function name  : PID_encoder
Functionality : To compute the angle set point using a Proportional Differential controller.
                Process variables is phi and phi_dot.
                This is the dominant controller when robot is still and is 
                used to hold the position of the robot and prevent drifting.
Return Value  : Angle setpoint
Example Call  : PID_encoder()
***********************************/
float PID_encoder (float phi, float phi_dot, float Kp, float Kd) 

{
  
  float output=0;

  float proportional = phi * Kp ;   ////////////////////////// Proportional
   
  float derivative = Kd * phi_dot ;  //////////////////////////////// Derivative


 //Serial.print(proportional); Serial.print("\t"); Serial.print(derivative); Serial.print("\t"); Serial.print(integral); Serial.print("\n");
  
  output = -1* proportional -1* derivative ;
  output = round(output *10)/10.0;

  output = constrain (output, -2, 2);   // restrain output to 2 degrees
   
  return (output);
}

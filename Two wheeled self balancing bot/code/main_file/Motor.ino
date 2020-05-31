 /*  
  * Filename: Motor     
  * Functions: motors_actuate, motor_setup 
 */


///////////////////////////////// MOTORS FUNCTIONS ///////////////////////////////////////


 /*  

 * Function Name: motors_actuate 
 * Input: pwm_value    
 * Output: Nonw    
 * Logic: We give power to the motors based on pwm values     
 * Example Call: motors_actuate (255);       

 */


 void motors_actuate (int pwm_value )
  { 
    pwm_l = lowpassfilter( pwm_value, pwm_l, 10);
    pwm_value = pwm_l;
  if (pwm_value<0)
  {
    pwm_value = pwm_value*(-1);
  constrain(pwm_value,60, 255);
  digitalWrite(LM1 , 0);
  digitalWrite(LM2 , 1);
  digitalWrite(RM1 , 0);
  digitalWrite(RM2 , 1);
  analogWrite(LMEN , pwm_value);
  analogWrite(RMEN , pwm_value);
  }
  
  else if (pwm_value>0)
  {
    
  constrain(pwm_value, 60, 255);
  digitalWrite(LM1 , 1);
  digitalWrite(LM2 , 0);
  digitalWrite(RM1 , 1);
  digitalWrite(RM2 , 0);
  analogWrite(LMEN ,pwm_value);
  analogWrite(RMEN , pwm_value);
  }

  else
  {

  analogWrite(LMEN , 0);
  analogWrite(RMEN , 0);
  }
  }



 /*  

 * Function Name: motor_setup
 * Input: None      
 * Output: None     
 * Logic: Sets up and initializes the motor pins     
 * Example Call: motor_setup()      

 */
void motor_setup(void)
{
  pinMode (LM1 , OUTPUT);
  pinMode (LM2 , OUTPUT);
  pinMode (RM1 , OUTPUT);
  pinMode (RM2 , OUTPUT);
  pinMode (LMEN , OUTPUT);
  pinMode (RMEN , OUTPUT);
  
  digitalWrite(LM1 , 1);
  digitalWrite(LM2 , 0);
  digitalWrite(RM1 , 1);
  digitalWrite(RM2 , 0);
  digitalWrite(LMEN , 0);
  digitalWrite(RMEN , 0);
}

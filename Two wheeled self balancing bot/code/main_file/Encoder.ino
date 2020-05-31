
 /*
  * Filename: Encoder          
  * Functions:rightEncoderEvent, leftEncoderEvent, encoder_setup, get_phi_value, get_phi_dot_value, get_rpm_value      
 */


///////////////////////////////// ENCODER FUNCTIONS ///////////////////////////////////////



 /*  

 * Function Name: rightEncoderEvent
 * Input: None      
 * Output: None     
 * Logic: Increments or decrements the right motor encoder count by 1      
 * Example Call: rightEncoderEvent()       

 */

void rightEncoderEvent() {
  
    if (digitalRead(RH_ENCODER_B) == LOW) {
      rightCount--;
    } else {
      rightCount++;
    }
  
  
}



 /*  

 * Function Name: leftEncoderEvent
 * Input: None      
 * Output: None     
 * Logic: Increments or decrements the left motor encoder count by 1      
 * Example Call: leftEncoderEvent()       

 */
void leftEncoderEvent() {
  
    if (digitalRead(LH_ENCODER_B) == LOW) 
    {
      leftCount--;
    } else
    {
      leftCount++;
    }
}


 /*  

 * Function Name: encoder_setup
 * Input: None      
 * Output: None    
 * Logic: Ses up the Encoder pins      
 * Example Call: encoder_setup()       

 */
void encoder_setup(void)
{
  pinMode(RH_ENCODER_A, INPUT);
  pinMode(RH_ENCODER_B, INPUT);
  pinMode(LH_ENCODER_A, INPUT);
  pinMode(LH_ENCODER_B, INPUT);
  
 attachInterrupt(1, rightEncoderEvent, RISING);
 attachInterrupt(0, leftEncoderEvent, RISING);
}


 /*  

 * Function Name: get_phi_value
 * Input: None     
 * Output: Angle travelled by the wheels in radians
 * Logic: We take the encoder counts of the motors and convert it into radians and average them    
 * Example Call: phi_value = get_phi_value()      

 */
float get_phi_value(void)
{
  angle_r = (rightCount/270.0)*6.283;
  angle_l = (leftCount/270.0)*6.283;

  float phi = (angle_r + angle_l )/2.0 ;

  return ( phi);
}


 /*  

 * Function Name: get_phi_dot_value
 * Input: None     
 * Output: Angular velocity of the wheel     
 * Logic: We take difference between the angles made by wheels in consecutive time instants and we average them together     
 * Example Call: phi_dot = get_phi_dot_value()       

 */
 
float get_phi_dot_value(void)
{
  angle_r_dt = (angle_r - angle_r0 )/ 0.01;
  angle_l_dt = (angle_l - angle_l0 )/ 0.01;
  angle_r0= angle_r;
  angle_l0= angle_l; 

  float phi_dot = (angle_r_dt + angle_l_dt) / 2.0;

  //phi_dot_l = lowpassfilter( phi_dot, phi_dot_l, 10);

  return ( phi_dot ) ;
  
}

 /*  

 * Function Name: get_rpm_value
 * Input: None     
 * Output: rotations per minute of the wheel    
 * Logic: we convert diatance travelled into rotations with radius of wheel. we oconver seconds into minutes. 
 * Example Call:  get_rpm_value()       

 */
void get_rpm_value(void)
{
  rpm_r = angle_r_dt*9.5495;
  rpm_l = angle_l_dt*9.5495;
}

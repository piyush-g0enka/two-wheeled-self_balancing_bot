
 /*   
  * Filename: MPU_6050        
  * Functions: lowpassfilter, highpassfilter, comp_filter, sesnor_setup, get_theta_value, get_theta_dot_value 
 */ 

 
///////////////////////////////// MPU6050 FUNCTIONS ///////////////////////////////////////



 /*  

 * Function Name: lowpassfilter 
 * Input: unfiltered_value,  filtered_value,  f_cut  
 * Output: filtered_output    
 * Logic: We pass the signal through a low pass filter by using a difference equation     
 * Example Call: singal = lowpassfilter(new_signal, filtered_signal, 5)      

 */

float lowpassfilter ( float unfiltered_value, float filtered_value, int16_t f_cut)
{
    float dT = 0.01; 
    float Tau = 1/(2*f_cut*3.14);
    float alpha = Tau/(Tau+dT);                
    filtered_value = (1-alpha)*unfiltered_value + alpha*filtered_value;
    return (filtered_value);
}




 /*  

 * Function Name: highpassfilter 
 * Input: unfiltered_value,  filtered_value,  f_cut  
 * Output: filtered_output    
 * Logic: We pass the signal through a high pass filter by using a difference equation     
 * Example Call: singal = highpassfilter(new_signal, filtered_signal, 5)      

 */
   
int16_t highpassfilter ( int16_t unfiltered_value0,int16_t unfiltered_value, int16_t filtered_value, int16_t f_cut)
{
  
    float dT = 0.01;
    float Tau = 1/(2*f_cut*3.14);
    float alpha = Tau/(Tau+dT);  
                  
    filtered_value = (1-alpha)*filtered_value+(1-alpha)*(unfiltered_value - unfiltered_value0);
    
    return(filtered_value);
}



 /*  

 * Function Name: comp_filter 
 * Input: unfiltered_value,  filtered_value,  f_cut  
 * Output: filtered_output    
 * Logic: We pass the signal through a complimentary filter to combine both gyro and accelerometer readings for better output      
 * Example Call: singal = comp_filter(ax, ay, az, gx, gy, gz, filtered_signal, 5)      

 */
 
float comp_filter(int16_t Ax,int16_t Ay,int16_t Az,int16_t Gx,int16_t Gy,int16_t Gz, float angle_old,int16_t f_cut)
{  
    
    float dT = 0.01;
    float Tau = 1/(2*f_cut*3.14);
    float alpha = 0.15;               //Tau/(Tau+dT);   
    
    float ax = Ax;
   
    float az = abs(Az);
   
    float gy = Gy/ 131.0;
  

    float acc= -atan(ax/az);
    float gyro= angle_old + gy*dT;

   
   
    
    float angle = (1-alpha)*(gyro) + (alpha)*(180/3.14)*(acc);



    angle = round(angle*10)/10.0;

  
   return (angle); 
}




 /*  

 * Function Name: sensor_setup
 * Input: None     
 * Output: None    
 * Logic: Initialises the sensor for the robot      
 * Example Call: sensor_setup ()       

 */
 
void sensor_setup (void)
{
      // join I2C bus (I2Cdev library doesn't do this automatically)
      #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
      #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
      #endif
      accelgyro.initialize();
  
}


 /*  

 * Function Name: get_theta_value 
 * Input: None     
 * Output: Gives value of angle in degrees    
 * Logic: We read sensor data and process the signals through complimentary filter and get the value of theta     
 * Example Call: theta = get_theta_value()      

 */
float get_theta_value (void)
{
  sei();
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  cli();
   
  gy_h= highpassfilter (gy0,gy,gy_h,5 );

  ax_l= lowpassfilter ( ax,ax_l,5 );
  az_l= lowpassfilter ( az,az_l,5 );


  gy0=gy;
  

  angle_1 = comp_filter(ax_l,ay_l,az_l,gx_h,gy_h,gz_h,angle_0,5);
 
  
 
  angle_0=angle_1;

  angle_1 = angle_1 + ang_offset ;    
 
 return (angle_1);
}



 /*  

 * Function Name: get_theta_dot_value 
 * Input: None     
 * Output: Gives value of angular velocity in degrees    
 * Logic: We read gyro sensor data and compute the angular velocity   
 * Example Call: theta_dot = get_theta_dot_value()      

 */

float get_theta_dot_value (void)
{
  t_dot = -gy/131.0;

  t_dot = round(t_dot *10)/10.0 - 0.5 ;


     
  
  return (t_dot);
}

int checkState(int sensor_left_, int sensor_right_, const int thresh, const int lasterr)
{
  //Both see black
  if( (sensor_left_>thresh) && (sensor_right_>thresh) ) error = 0; 
  //Right sees white
  if( (sensor_left_>thresh) && (sensor_right_<thresh) ) error = righterr; 
  //Left sees white
  if( (sensor_left_<thresh) && (sensor_right_>thresh) ) error = lefterr; 
  
  //Both see white: multiply the error by a factor specified by correction variable
  
  if( (sensor_left_<thresh) && (sensor_right_<thresh) )
  {
    if(lasterr<0) error = correction*lefterr;
    
    if(lasterr>=0) error = correction*righterr;
  }
  
  return error;
}

void followTape(int error)
{
  if(error != lasterr)
    {
      recerr = lasterr;
      prevt = time;
      time = 1;
    }
    
    prop = pgain*error;
    deriv = dgain*(error-recerr)/(prevt+time);
    total = prop+deriv;
    
    time = time+1;
    
    //Top left and bottom right wheels cause motion to the right
    spd_tleft = -spd+total;
    spd_bleft = -spd+total;
    
    //Top right and bottom left wheels cause motion to the left
    spd_tright = spd+total;
    spd_bright = spd+total;

    lasterr = error;
    
    //Correcting signs
    spd_tleft = -spd_tleft;
    spd_bleft = -spd_bleft;
    
}

void calcSteeringPID(void)
{
    //Proportional only
    pValue = steerSettings.Kp * steerAngleError;
    pwmDrive = (int16_t)pValue;

    errorAbs = abs(steerAngleError);
    int16_t newMax = 0;

    if (errorAbs < LOW_HIGH_DEGREES)
    {
        newMax = (errorAbs * highLowPerDeg) + steerSettings.lowPWM;
    }
    else newMax = steerSettings.highPWM;

    //add min throttle factor so no delay from motor resistance.
    if (pwmDrive < 0) pwmDrive -= steerSettings.minPWM;
    else if (pwmDrive > 0) pwmDrive += steerSettings.minPWM;

    //Serial.print(newMax); //The actual steering angle in degrees
    //Serial.print(",");

    //limit the pwm drive
    if (pwmDrive > newMax) pwmDrive = newMax;
    if (pwmDrive < -newMax) pwmDrive = -newMax;

    if (steerConfig.MotorDriveDirection) pwmDrive *= -1;    
}

//#########################################################################################

void motorDrive(void)
{
  // Used with Cytron MD30C Driver
  // Steering Motor
  // Dir + PWM Signal
  if (steerConfig.CytronDriver)
  {
    // Cytron MD30C Driver Dir + PWM Signal
    if (pwmDrive >= 0)
    {
      bitSet(PORTD, 4);  //set the correct direction
    }
    else
    {
      bitClear(PORTD, 4);
      pwmDrive = -1 * pwmDrive;
    }

    //write out the 0 to 255 value
    analogWrite(PWM1_LPWM, pwmDrive);
    pwmDisplay = pwmDrive;
  }  
}

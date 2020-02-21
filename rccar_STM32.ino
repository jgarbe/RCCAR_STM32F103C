/*
 * Created by Vasilakis Michalis // 12-12-2014 ver.2
 * Project: Control RC Car via Bluetooth with Android Smartphone
 * More information at www.ardumotive.com
 * 
 * Changed this code to fit STM32F103 by Jim Garbe, jimgarbe@gmail.com
 * Note that 8-bit values 0-255 have been changed to
 * reflect 16-bit values 0-65535
 */

 /***************************
  * On the STM32, the analog write still works at 8-bit 255,
  * But you can get the full functionm of the PWM range, 0-65535, by declaring the Pin as PWM
  *AND using pwnWrite() instead of analogWrite()
  ****************************/

  
////L293 Connection   
  const int motorA1  = PB6;  // to Pin  15 of L293
  const int motorA2  = PB7;  // to Pin  10 of L293
  const int motorB1  = PB8; // to Pin 7 of L293
  const int motorB2  = PB9;  // to Pin 2 of L293
//Leds connected to STM32F103C Pin A12
  const int lights  = PA12;
//Buzzer / Speaker to Arduino UNO Pin A8
  const int buzzer = PA8 ;   
//Bluetooth (HC-06 JY-MCU) State pin on pin A11 of STM32F103C
  const int BTState = PA11;
//Calculate Battery Level


/* Battery level will be checked on Pin PA5
 * Changed the next line for the STM32F103C because the ADC can't handle 
 * anything over 3.3v
 * I just commented it out 
 * A voltage divider, using two resistors must be calculated and used 
 * to measure the ADC input further down in the code
 * example:
 *            GND---2K resistor-----------------1K resistor ------5v
 *                                      |
 *                                      |
 *                                     3.3v
 */
 // const float maxBattery = 3.3;// Change value to your max battery voltage level! 
  
  
  
  int perVolt;                 // Percentage variable 
  float voltage = 0.0;         // Read battery voltage
  int level;
// Use it to make a delay... without delay() function!
  long previousMillis = -1000*10;// -1000*10=-10sec. to read the first value. If you use 0 then you will take the first value after 10sec.  
  long interval = 1000*10;       // interval at which to read battery voltage, change it if you want! (10*1000=10sec)
  unsigned long currentMillis;   //unsigned long currentMillis;
//Useful Variables
  int i=0;
  int j=0;
  int state;
  /*
   * If declaring the Pin as PWM, otherwise it will still be the arduino 255
   */
  int vSpeed=49150;     // Default speed, from 0 to 65535

void setup() {
    // Set pins as outputs:
    pinMode(motorA1, PWM);
    pinMode(motorA2, PWM);
    pinMode(motorB1, PWM);
    pinMode(motorB2, PWM);
    pinMode(lights, OUTPUT); 
    pinMode(BTState, INPUT);    
    // Initialize serial communication at 9600 bits per second:
    Serial.begin(9600);
}
 
void loop() {
  //Stop car when connection lost or bluetooth disconnected
     if(digitalRead(BTState)==LOW) { state='S'; }

  //Save income data to variable 'state'
    if(Serial.available() > 0){     
      state = Serial.read();   
        /****************************************************************************88
     * Debugging printing the state
     * 
     */
//  Serial.print ("the state is");
//   Serial.println(state);
   }

  //Change speed if state is equal from 0 to 4. Values must be from 0 to 65535 (PWM)
    if (state == '0'){
      vSpeed=0;}
    else if (state == '1'){ //20%
      vSpeed=13107;}
    else if (state == '2'){  //50%
      vSpeed=32767;}
    else if (state == '3'){  //75%
      vSpeed=49150;}
    else if (state == '4'){  //100%
      vSpeed=65535;}
 
  /***********************Forward****************************/
  //If state is equal with letter 'F', car will go forward!
    if (state == 'F') {
      pwmWrite(motorA1, vSpeed); pwmWrite(motorA2, 0);
        pwmWrite(motorB1, 0);      pwmWrite(motorB2, 0); 
    }
  /**********************Forward Left************************/
  //If state is equal with letter 'G', car will go forward left
    else if (state == 'G') {
      pwmWrite(motorA1, vSpeed); pwmWrite(motorA2, 0);  
        pwmWrite(motorB1, 49150);    pwmWrite(motorB2, 0); 
    }
  /**********************Forward Right************************/
  //If state is equal with letter 'I', car will go forward right
    else if (state == 'I') {
        pwmWrite(motorA1, vSpeed); pwmWrite(motorA2, 0); 
        pwmWrite(motorB1, 0);     pwmWrite(motorB2, 49150); 
    }
  /***********************Backward****************************/
  //If state is equal with letter 'B', car will go backward
    else if (state == 'B') {
      pwmWrite(motorA1, 0);   pwmWrite(motorA2, vSpeed); 
        pwmWrite(motorB1, 0);   pwmWrite(motorB2, 0); 
    }
  /**********************Backward Left************************/
  //If state is equal with letter 'H', car will go backward left
    else if (state == 'H') {
      pwmWrite(motorA1, 0);   pwmWrite(motorA2, vSpeed); 
        pwmWrite(motorB1, 49150); pwmWrite(motorB2, 0); 
    }
  /**********************Backward Right************************/
  //If state is equal with letter 'J', car will go backward right
    else if (state == 'J') {
      pwmWrite(motorA1, 0);   pwmWrite(motorA2, vSpeed); 
        pwmWrite(motorB1, 0);   pwmWrite(motorB2, 49150); 
    }
  /***************************Left*****************************/
  //If state is equal with letter 'L', wheels will turn left
    else if (state == 'L') {
      pwmWrite(motorA1, 0);   pwmWrite(motorA2, 0); 
        pwmWrite(motorB1, 49150); pwmWrite(motorB2, 0); 
    }
  /***************************Right*****************************/
  //If state is equal with letter 'R', wheels will turn right
    else if (state == 'R') {
      pwmWrite(motorA1, 0);   pwmWrite(motorA2, 0); 
        pwmWrite(motorB1, 0);   pwmWrite(motorB2, 49150);     
    }
  /************************Lights*****************************/
  //If state is equal with letter 'W', turn leds on or of off
    else if (state == 'W') {
              /****************************************************************************88
     * Debugging printing the vspeed
     * 
     */
//  Serial.print ("the vspeed is ");
//   Serial.println(state);
      if (i==0){  
         digitalWrite(lights, HIGH); 
         i=1;
      }
      else if (i==1){
         digitalWrite(lights, LOW); 
         i=0;
      }
      state='n';
    }
  /**********************Horn sound***************************/
  //If state is equal with letter 'V', play (or stop) horn sound
    else if (state == 'V'){
      if (j==0){  
         tone(buzzer, 1000);//Speaker on 
         j=1;
      }
      else if (j==1){
         noTone(buzzer);    //Speaker off 
         j=0;
      }
      state='n';  
    }
  /************************Stop*****************************/
  //If state is equal with letter 'S', stop the car
    else if (state == 'S'){
        pwmWrite(motorA1, 0);  pwmWrite(motorA2, 0); 
        pwmWrite(motorB1, 0);  pwmWrite(motorB2, 0);
    }
  /***********************Battery*****************************/
  //Read battery voltage every 10sec.
    currentMillis = millis();
    if(currentMillis - (previousMillis) > (interval)) {
       previousMillis = currentMillis; 
       //Read voltage from analog pin PA5 and make calibration:
       //Be sure to use a voltage divider to get no more than 3.3v from max battery
       voltage = (analogRead(PA5) / 4096.0)*3.3;  //TTL volts
       //Calculate percentage...
       //perVolt = (voltage*100)/ maxBattery;
       perVolt = (voltage*100)/ 3.3;
       if      (perVolt<=75)               { level=0; }
       else if (perVolt>75 && perVolt<=80) { level=1; }    //        Battery level
       else if (perVolt>80 && perVolt<=85) { level=2; }    //Min ------------------------   Max
       else if (perVolt>85 && perVolt<=90) { level=3; }    //    | 0 | 1 | 2 | 3 | 4 | 5 | >
       else if (perVolt>90 && perVolt<=95) { level=4; }    //    ------------------------
       else if (perVolt>95)                { level=5; }   
       Serial.println(level);  
//       Serial.print("The Speed level is ");  
//       Serial.print(vSpeed); 
//       Serial.println("/65535.");
    }
    
}

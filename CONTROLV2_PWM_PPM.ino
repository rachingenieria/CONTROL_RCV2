
#include <Servo.h>

#define DEBUGGER    1

#define MOTORD_AINA 4
#define MOTORD_AINB A0
#define MOTORD_PWM  5
#define AM          A7

#define MOTORI_AINA 7
#define MOTORI_AINB 11
#define MOTORI_PWM  6
#define BM          A6

#define SEL_PH      A1

//We create variables for the time width values of each PWM input signal
unsigned long counter_1, counter_2, counter_3, counter_4, current_count;

//We create 4 variables to stopre the previous value of the input signal (if LOW or HIGH)
byte last_CH1_state, last_CH2_state, last_CH3_state, last_CH4_state;

//To store the 1000us to 2000us value we create variables and store each channel
int input_YAW;      //In my case channel 4 of the receiver and pin D12 of arduino
int input_PITCH;    //In my case channel 2 of the receiver and pin D9 of arduino
int input_ROLL;     //In my case channel 1 of the receiver and pin D8 of arduino
int input_THROTTLE; //In my case channel 3 of the receiver and pin D10 of arduino

static int motord, motori;
Servo servoMotor;


unsigned long int a,b,c;
int x[30],ch1[30],ch[10],i,ch_a[10];
//specifing arrays and variables to store values 



void setup() 
{
   Serial.begin(9600);
   
   Motor_Init();

    //PPM 2,
    pinMode(2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(2), ppm_read_me, FALLING);

   //PINs 8,9,10,12
   PWM_setup();
}

void loop() 
{
   ppm_read_channels();
   //PWM_loop();
   //SetSpeeds(2*(input_THROTTLE-1500),2*(input_PITCH-1500));
   SetSpeeds(2*(ch[1]-1500),2*(ch[2]-1500));
   delay(20);
  
}


void Motor_Init(void)
{
  pinMode(MOTORD_AINA, OUTPUT);
  pinMode(MOTORD_AINB, OUTPUT);
  pinMode(MOTORI_PWM, OUTPUT);
  
  pinMode(MOTORI_AINA, OUTPUT);
  pinMode(MOTORI_AINB, OUTPUT);
  pinMode(MOTORD_PWM, OUTPUT);
  
  analogWrite(MOTORI_PWM,0);
  analogWrite(MOTORD_PWM,0);
}

void SetSpeeds(int mi,int md)
{
 motord = md/4;
 motori = mi/4 ;

 int motor_derecho = md/4;
 int motor_izquierdo = mi/4;
 
 if(motor_derecho > 0)
  {
    
     if(motor_derecho>255)
         {
            motor_derecho = 255;
         }

     digitalWrite(MOTORD_AINA,LOW);
     digitalWrite(MOTORD_AINB,HIGH);
     analogWrite(MOTORD_PWM,motor_derecho);
  }
  else if(motor_derecho < 0)
    {
      
      if(motor_derecho<-255) //LIMITE GIRO ATRAS
         {
            motor_derecho = -255;
         }
         
     digitalWrite(MOTORD_AINA,HIGH);
     digitalWrite(MOTORD_AINB,LOW);
     analogWrite(MOTORD_PWM,-motor_derecho);
  }
 
 
 
  if(motor_izquierdo > 0)
  { 
    if(motor_izquierdo>255)
    {
      motor_izquierdo = 255;
    }
     digitalWrite(MOTORI_AINA,LOW);
     digitalWrite(MOTORI_AINB,HIGH);
     analogWrite(MOTORI_PWM,motor_izquierdo);
  }
  else if (motor_izquierdo < 0)
    {
       if(motor_izquierdo<-255)
         {
            motor_izquierdo = -255;
         }
     digitalWrite(MOTORI_AINA,HIGH);
     digitalWrite(MOTORI_AINB,LOW);
     analogWrite(MOTORI_PWM,-motor_izquierdo);
  }
  
}

void PWM_setup() {
  /*
   * Port registers allow for lower-level and faster manipulation of the i/o pins of the microcontroller on an Arduino board. 
   * The chips used on the Arduino board (the ATmega8 and ATmega168) have three ports:
     -B (digital pin 8 to 13)
     -C (analog input pins)
     -D (digital pins 0 to 7)
   
  //All Arduino (Atmega) digital pins are inputs when you begin...
  */  
   
  PCICR |= (1 << PCIE0);    //enable PCMSK0 scan                                                 
  PCMSK0 |= (1 << PCINT0);  //Set pin D8 trigger an interrupt on state change. 
  PCMSK0 |= (1 << PCINT1);  //Set pin D9 trigger an interrupt on state change.                                             
  PCMSK0 |= (1 << PCINT2);  //Set pin D10 trigger an interrupt on state change.                                               
  PCMSK0 |= (1 << PCINT4);  //Set pin D12 trigger an interrupt on state change.  

}

void PWM_loop() {
  /*
   * Ok, so in the loop the only thing that we do in this example is to print
   * the received values on the Serial monitor. The PWM values are read in the ISR below.
   */ 

    
    //Add the next part of the code here...
    //Soon...
  Serial.print(" PWM ");
  Serial.print(input_YAW);Serial.print("\t");
  Serial.print(input_PITCH);Serial.print("\t");
  Serial.print(input_ROLL);Serial.print("\t");
  Serial.println(input_THROTTLE);Serial.print("\t");
 
  
}




//This is the interruption routine
//----------------------------------------------

ISR(PCINT0_vect){
//First we take the current count value in micro seconds using the micros() function
  
  current_count = micros();
  ///////////////////////////////////////Channel 1
  if(PINB & B00000001){                              //We make an AND with the pin state register, We verify if pin 8 is HIGH???
    if(last_CH1_state == 0){                         //If the last state was 0, then we have a state change...
      last_CH1_state = 1;                            //Store the current state into the last state for the next loop
      counter_1 = current_count;                     //Set counter_1 to current value.
    }
  }
  else if(last_CH1_state == 1){                      //If pin 8 is LOW and the last state was HIGH then we have a state change      
    last_CH1_state = 0;                              //Store the current state into the last state for the next loop
    input_ROLL = current_count - counter_1;   //We make the time difference. Channel 1 is current_time - timer_1.
  }



  ///////////////////////////////////////Channel 2
  if(PINB & B00000010 ){                             //pin D9 -- B00000010                                              
    if(last_CH2_state == 0){                                               
      last_CH2_state = 1;                                                   
      counter_2 = current_count;                                             
    }
  }
  else if(last_CH2_state == 1){                                           
    last_CH2_state = 0;                                                     
    input_PITCH = current_count - counter_2;                             
  }


  
  ///////////////////////////////////////Channel 3
  if(PINB & B00000100 ){                             //pin D10 - B00000100                                         
    if(last_CH3_state == 0){                                             
      last_CH3_state = 1;                                                  
      counter_3 = current_count;                                               
    }
  }
  else if(last_CH3_state == 1){                                             
    last_CH3_state = 0;                                                    
    input_THROTTLE = current_count - counter_3;                            

  }


  
  ///////////////////////////////////////Channel 4
  if(PINB & B00010000 ){                             //pin D12  -- B00010000                      
    if(last_CH4_state == 0){                                               
      last_CH4_state = 1;                                                   
      counter_4 = current_count;                                              
    }
  }
  else if(last_CH4_state == 1){                                             
    last_CH4_state = 0;                                                  
    input_YAW = current_count - counter_4;                            
  }

}


void ppm_read_channels()
{
int k=0;
for(k=0;k<8;k++)
 {
      ch_a[k] = ch[k];
 }

ppm_read_rc();
//
Serial.print(" PPM ");Serial.print("\t");
Serial.print(ch[0]);Serial.print("\t");
Serial.print(ch[1]);Serial.print("\t");
Serial.print(ch[2]);Serial.print("\t");
Serial.print(ch[3]);Serial.print("\t");
Serial.print(ch[4]);Serial.print("\t");
Serial.print(ch[5]);Serial.print("\t");
Serial.print(ch[6]);Serial.print("\t");
Serial.print(ch[7]);Serial.println("\t");


}
 
void ppm_read_me()  {
     //this code reads value from RC reciever from PPM pin (Pin 2 or 3)
     //this code gives channel values from 0-1000 values 
     //    -: ABHILASH :-    //
    a=micros(); //store time value a when pin value falling
    c=a-b;      //calculating time inbetween two peaks
    b=a;        // 
    x[i]=c;     //storing 15 value in array
    //Serial.print(c);Serial.print("\n");
    i=i+1;       
    if(i==15)
    {
      for(int j=0;j<30;j++)
      {
        ch1[j]=x[j];
      }
      i=0;
     }
}//copy store all values from temporary array another array after 15 reading  

void ppm_read_rc()
{
int i,j,k=0;
  for(k=30;k>-1;k--)
   {
      if(ch1[k]>4000){j=k;}
   }  //detecting separation space 10000us in that another array      
                  
    for(i=1;i<=8;i++)
    {
     ch[i]=(ch1[i+j]);
    }
}     //assign 6 channel values after separation space

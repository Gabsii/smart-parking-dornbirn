//used Libraries
//======================================================================================================
#include <Ultrasonic.h>   //distance sensor (Ultrasonic distance sensor) 
//#include <lmic.h>
//#include <hal/hal.h>
//#include <SPI.h>
//http://www.netzmafia.de/skripten/hardware/Arduino/Sleep/index.html
//https://www.youtube.com/watch?v=usKaGRzwIMI

//defined Pins
//======================================================================================================
#define TRIG_PIN 9      //ultrasonic sensor trigger pin is connected to arduino pin 9   
#define ECHO_PIN 10      //ultrasonic sensor echo pin is connected to arduino pin 10
#define BAUDRATE 9600   //for debugging and testing

//defined stuff
#define park_time_threshold 30  //threshold for parking seconds
#define park_out_time_threshold 15  //threshold for parking seconds // has to be lower then park_time_threshold due to logik

#define distance_threshold 50   //threshold for praking distance 

#define timer1_preload 49909    //for 1s
#define timer2_preload 99    //for 10ms
//#define timer2_softload 100    //for 1s //10ms*100 = 1s
//#define sleep_time 60         //upcoming feature

//variables
//======================================================================================================
int distance_cm;

bool park_flag=false;
bool park_status=false;
int park_time=0;

//for sleep mode // future update
//bool sleep_flag=false;  
//int timer2_softcount=0;         
//int timer2_softcount_sec=0;         

//objects
//======================================================================================================
Ultrasonic parkingsensor(TRIG_PIN, ECHO_PIN);


//Timer1 magic config :P
//for more info check the datasheet http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
//======================================================================================================
void timer1_setup(){
  TCCR1A=0x00;
  TCCR1B=0x00;

  //Normal Mode //Prescaler is set to 1024          
  TCCR1B|=(1<<CS12);
  TCCR1B|=(1<<CS10);

  //Interrupt enable
  TIMSK1=0x00;
  TIMSK1|=(1<<TOIE1);     //enable timer overflow interrupt

  TCNT1=timer1_preload;    //load value for timer to hit one second precicly   //startvalue of the counter
}


//main
//======================================================================================================
void setup() {
  timer1_setup();         //timer1 setup 
  //timer2_setup();         //timer2 setup
  Serial.begin(9600);     
}

//LOOP
//======================================================================================================
void loop() {
  parkingsensor.measure();      //reads distance to car in cm 
  distance_cm=parkingsensor.get_cm(); 
  
   if((distance_cm <= distance_threshold)&&(!park_status)){      //if the distance is under 50cm and the park status is not set then it starts checking if the car is parking or not.
    park_flag=true;   
   }
   if((distance_cm > distance_threshold)&&park_status){    //if the distance is bigger then the threshold it should check again if the car is trying to park or not
    park_flag=false;
   }

   if(park_status==park_flag){
    park_time=0;
   }
   delay(1000);   //wait a sec and print data
              Serial.print("park_threshcheck: ");   //time since park_flag is changed
              Serial.print(park_time);
              Serial.print("     ");
              Serial.print("distance: ");           //current distance to car
              Serial.print(distance_cm);
              Serial.print("     ");
              Serial.print("Millis: ");             //time since microcontroller started
              Serial.println(millis());
}


//======================================================================================================
ISR(TIMER1_OVF_vect){   //triggers every second
  
  TCNT1=timer1_preload;  //load value for timer to hit one second precicly
  
 
  if((park_time<=park_time_threshold)&&park_flag){       //start counting if the slot is taken and distnace under threshold
    if(distance_cm<distance_threshold){
      park_time++;
    }
    else{
      park_time=0;
    }
  }

  if((park_time==(park_time_threshold))&&park_flag){  //updates parkstatus       
    park_status=true;
    park_time=0;
              Serial.println("                      Jetzt ist eingeparkt!!!!!!!");
  }

  if((park_time<=park_out_time_threshold)&&(!park_flag)){       //start counting if slot is freed and distance over the threshold
    if(distance_cm>distance_threshold){
      park_time++;
    }
    else{
      park_time=0;
    }
  }
  
  if((park_time==(park_out_time_threshold))&&(!park_flag)){  //updates parkstatus     
    park_status=false;
    park_time=0;
              Serial.println("                      Jetzt ist ausgeparkt!!!!!!!");
  }
}

//======================================================================================================

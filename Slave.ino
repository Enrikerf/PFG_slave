/*
  Slaves module for control PID of Scorbot axe
 
 Autor: Enrique Arrabal
 Date of creation: 3/8/2014
 Date of last modification: 14/9/2014
 copyright (c) 2014 Enrique Arrabal
 
 GNU General Public License
 
 references:
 - http://arduino.cc/en/Tutorial/AnalogInOutSerial
 - http://playground.arduino.cc/Main/RotaryEncoders
 - http://playground.arduino.cc/Code/Timer1
 
 version history
 
 */

/*
------------------------------------------------------------------------------------------------
 0         1         2         3         4         5         6         7         8         9
 012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345
 ------------------------------------------------------------------------------------------------ 
 */
// STANDAR LIBRARYS
#include <Wire.h>
#include <TimerOne.h>
// Archivos agregados
#include "BasicStructures.h"
#include "Configuration.h"
#include "functions.h"
//-----------------------------------------------------------------------------------------------

Data_Package      myData_Package                      ;  
boolean           Receive_OorV              =  true   ;
boolean           flagProcess_Commands      =  false  ; 

void setup(){
  // Configure Input Pins
  pinMode(Pin_EncoderPhaseA, INPUT_PULLUP);      
  pinMode(Pin_EncoderPhaseB, INPUT_PULLUP);      
  pinMode(Pin_Origin    , INPUT_PULLUP);  
  //Configure Output Pins
  pinMode(Pin_Brake     , OUTPUT      );      
  pinMode(Pin_Dir       , OUTPUT      );
  // Attach External Interrupts
  attachInterrupt(0, Ev_EncoderA, CHANGE);   
  attachInterrupt(1, Ev_EncoderB, CHANGE);   
  // Inicializate and Attach Timer internal interrupt
  Timer1.initialize(long(1e6 * SampleTime)); 
  Timer1.attachInterrupt(controlFcn);        
  // Begin Wire Comunication
  Wire.begin(MyID);  
  Wire.onRequest(Request_Routine);  
  Wire.onReceive(Receive_Routine);  
  // Enable brake of Motor by default
  digitalWrite(Pin_Brake,HIGH);
  digitalWrite(Pin_Dir,LOW);
  digitalWrite(PinPWM,HIGH);
}

void loop(){
  if(flagProcess_Commands){
    Process_Commands(int(myData_Package.Order.fValue), myData_Package.Valor.fValue);
    flagProcess_Commands = false;  
  };
  delay(NewOrderDelay);
}

/*
RUTINAS DE INTERRUPCION
 */

// Control Routine
void controlFcn() {
  V_Proceso.fValue   =   RES * EncoderValue;
  dV_Proceso         =   (V_Proceso.fValue - _V_Proceso)/SampleTime ;
  if (Enable){
    Control();
  };
  if(Home){  
    Homing();
  };
  atHome            =   (digitalRead(Pin_Origin) == Hometrue);
  _V_Proceso        =   V_Proceso.fValue;   
}

// I2C Routines
void Request_Routine(){
  Wire.write(V_Proceso.bArray,4);
};

void Receive_Routine(int nDat){
  static int i;
  if(Receive_OorV){
    for(i = 0;i <nDat;i++) 
      myData_Package.Order.bArray[i] = Wire.read();        
    Receive_OorV = false;
  }  
  else{      
    for(i = 0;i <nDat;i++) 
      myData_Package.Valor.bArray[i] = Wire.read();
    Receive_OorV = true;
    flagProcess_Commands = true; 
  }
};








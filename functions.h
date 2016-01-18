
#ifndef functions_h
#define functions_h
#include<math.h>
#include"Arduino.h"
#include"BasicStructures.h"
#include"Configuration.h"

Orders whichOrder(int Order);


void Control(){
  // variables internas
  static float   iTerm, derror,_error;
  //--------------------------------------------------------------------  
  error            =   SetPoint   -   V_Proceso.fValue;
  iTerm            +=  I * error ;                     
  iTerm            =   constrain (iTerm, -255, 255);
  derror           =   (error - _error);
  V_Manipulada     =   P * error  +   iTerm    +    D * derror;  
  V_ManipuladaSat  =   constrain (V_Manipulada, -255, 255);    
  // Control triestado    
  if(z){  
    if (error > z ){
      V_ManipuladaSat   =   map (V_ManipuladaSat, 0,  255,  DeadZone,  255);
    }
    else if (error < -z ){
      V_ManipuladaSat   =   map (V_ManipuladaSat, 0, -255, -DeadZone, -255);
    }
    else
      V_Manipulada = 0;
  };
  if(DeadZone){
    if (error > 0 ){
      V_ManipuladaSat  =   map (V_ManipuladaSat, 0,  255,  DeadZone,  255);
    }
    else{
      V_ManipuladaSat  =   map (V_ManipuladaSat, 0, -255, -DeadZone, -255);
    }
  };
  //---------------------------------------------------------------------- 
  analogWrite(PinPWM, abs(V_ManipuladaSat));  
  digitalWrite(Pin_Dir, (V_ManipuladaSat <= 0)); 
  _error = error;
}

// 

void Homing(){
  static int NgetHome   = 0;
  static int Velocity   = PWMsearchHome;
  static int StartUp    = 0;
  static int PassHome   = 0;
  static boolean DIR    = true,ChangeDIR = false,StayAtHome = false;
  static float    derror,_error,Kwindup;
  //--------------------------------------------------------------------  
  error            =   Velocity - abs(dV_Proceso);  
  iTerm            +=  (2 * error);                             
  derror           =   (error - _error);
  V_Manipulada     =   2 * error +   iTerm ;    
  V_ManipuladaSat     =   constrain (V_Manipulada, -255, 255);  
  // si baja la velocidad, ya habia arrancado y todavia no ha enNgetHomerado home ninguna vez se cambia de direccion
  if((abs(dV_Proceso) <= VMecObstacle) && StartUp > StartUpFactor && Velocity == PWMsearchHome){
    if(ChangeDIR == true){
      PassHome += PassHomeFactor;
    }
    else
      DIR = !DIR;
  };
  // si cambia el estado del microinterruptor se cambia la direccion y se cuenta las veces que ha cambiado
  if(StayAtHome != atHome){
    StayAtHome = atHome;    
    ChangeDIR = true;
    NgetHome++;

  }
  // si ha enNgetHomerado home por segunda vez es que ya lo ha enNgetHomerado bien
  else if(NgetHome > 2){
    // pull up the brake pin to stop
    digitalWrite(Pin_Brake,HIGH);
    digitalWrite(Pin_Dir,LOW);
    digitalWrite(PinPWM,HIGH);  
    // Set default values
    EncoderValue    = 0;
    SetPoint        = 0;
    DIR             = true;
    ChangeDIR       = false;
    NgetHome        = 0;
    Velocity        = PWMsearchHome;
    StartUp         = 0;
    PassHome        = 0;
    Home            = false;

  }
  // Si ha enNgetHomerado home una o ninguna vez
  else
  {
    if(ChangeDIR){ // si se tiene que cambiar la direccion porque se ha enNgetHomerado una vez
      if(PassHome > PassHomeFactor ){ // se comprueba que se ha pasado de largo home para volver con precision
        DIR         = !DIR;
        ChangeDIR   = false;
        Velocity    = PWMsetHome;
        StartUp     = 0;
      }
      else
        PassHome++;
    };
    analogWrite(PinPWM, abs(V_ManipuladaSat));  
    digitalWrite(Pin_Dir, DIR); 
    StartUp++;
  } 
  _error = error;
}


// Subfuncion que NgetHomeiene el Process_Commands de Orderes enviada por I2C

void Process_Commands(int Order, float Valor){
  // Variables internas
  static boolean flag2 = false;
  Orders myOrder = whichOrder(Order);
  //----------------------------
  switch(myOrder){
  case _Enable:
    if(Valor == 1.0){
      Enable = true;
      digitalWrite(Pin_Brake,LOW); 
      digitalWrite(Pin_Dir,HIGH);
    }
    else{
      Enable = false;
      digitalWrite(Pin_Brake,HIGH);
      digitalWrite(Pin_Dir,LOW);
      digitalWrite(PinPWM,HIGH);
    }
    break;
  case _NewSampleTime:
    SampleTime       = Valor;
    Timer1.setPeriod(long(1e6 * SampleTime));
    break;  
  case _NewP:
    P         = Valor;
    break;
  case _NewI:
    I         = Valor*SampleTime;
    break;
  case _NewD:
    D        = Valor/SampleTime;
    break;
  case _Newz:
    z       = Valor;
    break;
  case _SetPosAct:
    V_Proceso.fValue = Valor;
    _V_Proceso       = V_Proceso.fValue;
    SetPoint         = V_Proceso.fValue;
    EncoderValue     = V_Proceso.fValue/RES;   
    break;
  case _NewSetPoint:
    SetPoint         = Valor;
    break; 
  case _NewEncoderSlots:
    encoderSlots      = Valor;    
    break; 
  case _NewMotorGearRatio:
    motorGearRatio   = Valor;
    break; 
  case _NewCountingRatio:
    countingRatio    = Valor;
    break; 
  case _NewJointGearRatio:
    jointGearRatio   = Valor;
    break;  
  case _NewDeadZone:
    DeadZone         =Valor;
    break; 
  case _Homing:      
    if(Valor == 1.0){
      V_Proceso.fValue = 0;
      _V_Proceso       = 0;
      SetPoint         = 0;
      EncoderValue     = 0; 
      Home =true;
      digitalWrite(Pin_Brake,LOW);
    }
    else{
      Home = false;
      digitalWrite(Pin_Brake,HIGH);
      digitalWrite(Pin_Dir,LOW);
      digitalWrite(PinPWM,HIGH);
    }
    break;
  }
  // Si se ha cambiado algun parametro que afecta a la resolucion se recalcula
  if(Order == 8 || Order == 9||Order == 10 ||Order == 11){
    PPR 		= encoderSlots * countingRatio * motorGearRatio * jointGearRatio; 
    RES 		= 360.0 / PPR;
  };
}


Orders whichOrder(int Order){
  static Orders mOrder;
  switch(Order){
  case 0:
    mOrder = _Enable;
    break;  
  case 1:
    mOrder =_NewSampleTime;  
    break;
  case 2:
    mOrder = _NewP;             
    break;
  case 3:
    mOrder =_NewI;              
    break;
  case 4:
    mOrder =_NewD;     
    break;
  case 5:
    mOrder =_Newz      ;   
    break;
  case 6:
    mOrder =_SetPosAct  ;
    break;
  case 7:
    mOrder =_NewSetPoint;
    break;
  case 8:
    mOrder =_NewEncoderSlots ;  
    break;
  case 9:
    mOrder =_NewMotorGearRatio ;
    break;
  case 10:
    mOrder =_NewCountingRatio; 
  case 11:
    mOrder =_NewJointGearRatio; 
    break;
    break;
  case 12:
    mOrder =_NewDeadZone;       
    break;
  case 13:
    mOrder = _Homing;   
    break;
  }
  return mOrder;
}
// Funciones de las interrupciones de lectura de encoders

void Ev_EncoderA () {
  if (digitalRead(Pin_EncoderPhaseA) == digitalRead(Pin_EncoderPhaseB)) 
    EncoderValue++;
  else
    EncoderValue--;
}

void Ev_EncoderB () {
  if (digitalRead(Pin_EncoderPhaseA) != digitalRead(Pin_EncoderPhaseB))
    EncoderValue++;
  else
    EncoderValue--;
}


// NO SE USA

// subfuncion que determina la zona muerta
boolean SetDeadZone(){
  static short int NgetHome = 0;  

  if(dV_Proceso){
    //DeadZone--;
    analogWrite(PinPWM, DeadZone);  
    digitalWrite(Pin_Dir, LOW);
    NgetHome++;
  }
  else if(NgetHome >= 25){        
    error = 0;
    dV_Proceso = 0;
    NgetHome = 0;
    return false;
  }
  else{     
    DeadZone++;
    analogWrite(PinPWM, DeadZone);  
    digitalWrite(Pin_Dir, HIGH);
    return true;
  }
}
#endif













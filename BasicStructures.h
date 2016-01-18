#ifndef BasicStructures_h
#define BasicStructures_h
#include"Arduino.h"
//----------------------------------------------------------------------------------------------
// ESTRUCCTURAS DE DATOS
//----------------------------------------------------------------------------------------------
union ufloat{
  byte bArray[4];
  float fValue;
};
struct Data_Package{
  ufloat   Order;
  ufloat   Valor;
};

enum Orders{
  _Enable            =0, 
  _NewSampleTime     ,
  _NewP              ,
  _NewI              ,
  _NewD              ,
  _Newz              ,
  _SetPosAct         ,
  _NewSetPoint       ,
  _NewEncoderSlots   ,
  _NewMotorGearRatio ,
  _NewCountingRatio  ,
  _NewJointGearRatio ,
  _NewDeadZone       ,
  _Homing            
};
#endif


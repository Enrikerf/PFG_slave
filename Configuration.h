#ifndef Configuration_h
#define Configuration_h
#include"Arduino.h"
#include"BasicStructures.h"
//-----------------------------------------------------------------------------------------------
const int                    MyID                 = 2    ;                     // Identificador como esclavo
float			     SampleTime 	  = 20e-3 ; 
float                        NewOrderDelay        = 100   ;
//----------------------------------------------------------------------------------------------
// Parametros del motor
int 		             encoderSlots 	  = 20	  ;      		// ScorbotA1A2 = 20      || EMG30 = 
float 		             motorGearRatio 	  = 127.1 ;  		        // Scorbot A1A2 = 127.1  || EMG30 = 30:1
float 		             jointGearRatio	  = 4	  ;   			// Scorbot A1 = 5        || Scorbot A2 = 4 || EMG30 =
int 		             countingRatio 	  = 4	  ;      		// Scorbot A1 A2 = 4     || EMG30 = 
float 		             DeadZone	          = 0	  ;       		// Scorbot A1 =      0   ||Scorbot A2      ||EMG30 =
float 		             PPR 		  = encoderSlots * countingRatio * motorGearRatio * jointGearRatio; 	
float 	                     RES 		  = 360.0 / PPR;		// resolution
//----------------------------------------------------------------------------------------------
// Variable de las rutinas del encoder
volatile long 		     EncoderValue	  = 0     ; 
//----------------------------------------------------------------------------------------------
// Parametros del PID
boolean                      Enable  =  false             ;                     // Control Activado/Desactivado
float 	                     SetPoint                     ;                     // Posicion de referencia
ufloat     	             V_Proceso                    ;
float                        dV_Proceso =0,_V_Proceso=0   ;                     // Variable del proceso
float 	                     V_Manipulada,V_ManipuladaSat,iTerm,Kupwind;        // Variable manipulada
float 	                     error                        ;                     // error [grados]
float                        P                    = 39.3536    ;
float                        I                    = 9.0628*SampleTime;
float                        D                    = 0/SampleTime;
float                        z                    = 0     ;
//----------------------------------------------------------------------------------------------
// motor current info
boolean 		     atHome			  ;     		// 
boolean                      Hometrue             = false ;                     // if pullup pin set 0 or 1 to microswitch
float 			     motorSpeed		          ;    			// 
float 			     motorPosition		  ; 			// 
float 			     current			  ;       		// 
float 			     nominalSpeed		  ;  			// 
//----------------------------------------------------------------------------------------------
// homing parameters
boolean                      Home                 = false;
float                        VMecObstacle         = 0     ; // A1 =5 A2 = 3
int                          StartUpFactor        = 20    ; // factor of time for start up the articulation A1 = 20    A2=
int                          PassHomeFactor       = 30    ; // factor of time for spend long home position  A1 = 30    A2=
float                        PWMsearchHome        = 150    ; //                                              A1 = 150   A2=
float                        PWMsetHome           = 50     ; //                                              A1 = 50    A2=
float                        Qmax                 = 310   ; // Grados mximos y minimo de la articulacion    A1 = 310   A2=
float                        Qmin                 = 0     ;                                               //A1 = 0     A2=


//---------------------------------------------------------------------------------------------
// PINES
//---------------------------------------------------------------------------------------------
// INPUTS 
#define Pin_EncoderPhaseA     2                             // external interrupt #0 for encoder phase A signal
#define Pin_EncoderPhaseB     3                             // external interrupt #1 for encoder phase B signal
#define Pin_Origin            4                             // encoder origin signal
// OUTPUTS
#define Pin_Brake             8                             // brake output for motor driver
#define PinPWM                11                            // PWM output f
#define Pin_Dir               13
//------------------------------------------------------------------------------------------------------------

#endif


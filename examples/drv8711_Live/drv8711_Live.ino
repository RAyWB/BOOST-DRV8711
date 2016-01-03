//######################################################################################
//   Example sketch demonstrating the DRV8711 library for use with BOOST-DRV8711 Module 
//######################################################################################

#include <drv8711.h>
#include <SPI.h>

#define RESETpin 7
#define X_ssPin 3
#define Y_ssPin 4
#define Z_ssPin 5
#define EnableInPin 6

// Current Settings
const int X_Moving_Torque = 186 ;
const int X_Stopped_Torque = 93 ;
const int Y_Moving_Torque = 186 ;
const int Y_Stopped_Torque = 93 ;
const int Z_Moving_Torque = 186 ;
const int Z_Stopped_Torque = 93 ;

// Initialise an instance of the drv8711 object for each driver
drv8711 Axis[3] = { drv8711(X_ssPin), drv8711(Y_ssPin), drv8711(Z_ssPin)}; //parameter is CSS Pin for Driver

// variables for decoded parameters
const float ISENSE = 0.05;  // Value of current sense resistors in ohms
int ISGAIN = 0;      // Currently Selected ISGAIN Value
float Amps = 0.00;   // For Calculated Peak Current 
String DECMOD = ""; // For Text Desc of Decay Mode
float TBLANK = 0.0;  // For Blanking Time in uS
String ABT = "";  // For ABT State
float TDECAY = 0.0;  // For Decay Time in uS
float TOFF = 0.0;  // For PWM Off time in uS
int currentAxis = 0;

int readDelay = 2000 ;
long int LastRead = 0;
boolean Locked = true ;
boolean LastEnableIn = HIGH ;
boolean EnableVal = HIGH ;

//##########################################################################
void setup ()
//##########################################################################
{
  pinMode(EnableInPin, INPUT) ;
  
  // Reset the driver before initialisation
  pinMode (RESETpin, OUTPUT) ;
  digitalWrite (RESETpin, HIGH) ;
  delay (10) ;
  digitalWrite (RESETpin, LOW) ;
  delay (1) ;
  
  // Start Serial
  Serial.begin (115200) ;

  Locked=true;
}

//##########################################################################
void loop ()
//##########################################################################
{
   //if Locked initialise drivers
   if (Locked) {
     setRegisters();
     Locked = false;
   }
  
   // Change TORQUE value when EnableInPin changes
   EnableVal = digitalRead(EnableInPin);
   
   if (EnableVal != LastEnableIn) {
     if (EnableVal == LOW) {
       Axis[0].G_TORQUE_REG.TORQUE = X_Moving_Torque ;  
       Axis[1].G_TORQUE_REG.TORQUE = Y_Moving_Torque ;  
       Axis[2].G_TORQUE_REG.TORQUE = Z_Moving_Torque ;  
     } else {
       Axis[0].G_TORQUE_REG.TORQUE = X_Stopped_Torque ;  
       Axis[1].G_TORQUE_REG.TORQUE = Y_Stopped_Torque ;  
       Axis[2].G_TORQUE_REG.TORQUE = Z_Stopped_Torque ;  
     }
     Axis[0].WriteTORQUERegister() ;
     Axis[1].WriteTORQUERegister() ;
     Axis[2].WriteTORQUERegister() ;
     LastEnableIn = EnableVal ;
   }
   
   // Periodically check status register, and print any flagged errors
   if (millis() > LastRead + readDelay) {
      for (int i=0; i<3; i++){
       Axis[i].get_status();
      
       if (Axis[i].G_STATUS_REG.UVLO) {
        Serial.println("ERROR: Axis:" + String(i) + " UnderVoltage Lockout");
        Locked = true;
       } else {
          if (Axis[i].G_STATUS_REG.STDLAT) Serial.println("ERROR: Axis:" + String(i) + " Latched Stall Detect");
          if (Axis[i].G_STATUS_REG.BPDF) Serial.println("ERROR: Axis:" + String(i) + " Channel B Predriver Fault");
          if (Axis[i].G_STATUS_REG.APDF) Serial.println("ERROR: Axis:" + String(i) + " Channel A Predriver Fault");
          if (Axis[i].G_STATUS_REG.BOCP) Serial.println("ERROR: Axis:" + String(i) + " Channel B Over Current");
          if (Axis[i].G_STATUS_REG.AOCP) Serial.println("ERROR: Axis:" + String(i) + " Channel A Over Current");
          if (Axis[i].G_STATUS_REG.OTS) Serial.println("ERROR: Axis:" + String(i) + " Over Temperature");
       }
      
       Axis[i].clear_status();
      }
   LastRead = millis();
   }
  
  
  // if '?' received over serial, display settings for all axis. 
  if (Serial.available()) {
    if ((char)Serial.read() == '?') {
      for (int i=0; i<3; i++){
        currentAxis = i;
        Axis[i].ReadAllRegisters();
        displaySettings();
      }
    }
  }
}

//##########################################################################
void setRegisters()
//##########################################################################
{
  // Load Library Defaults
  for(int i=0; i<3; i++){
    Axis[i].set_defaults();
  }
  
  // Make specific register settings
  Axis[0].G_TORQUE_REG.TORQUE = X_Stopped_Torque ;// Set TORQUE value 0..255 ( Peak Amps = 2.75 * TORQUE / (256 * IGAIN * RSENSE))
  Axis[0].G_CTRL_REG.MODE = STEPS_32 ;           // Microstepping mode to 1/32
  Axis[0].G_DECAY_REG.DECMOD = DECMOD_MIXAUTO ;  // Decay Mode to Mixed Auto
  Axis[0].G_BLANK_REG.TBLANK = 115 ;             // TBLANK to 3.3 uS
  Axis[0].G_BLANK_REG.ABT = ON ;                 // ABT ON
  Axis[0].G_DECAY_REG.TDECAY = 16 ;              // TDECAY to 8 uS
  Axis[0].G_OFF_REG.TOFF = 32 ;                  // TOFF to 16 uS
  Axis[0].G_CTRL_REG.ENBL = ON ;                 // Enable Motor
  
  Axis[1].G_TORQUE_REG.TORQUE = Y_Stopped_Torque ;// Set TORQUE value 0..255 ( Peak Amps = 2.75 * TORQUE / (256 * IGAIN * RSENSE))
  Axis[1].G_CTRL_REG.MODE = STEPS_32 ;           // Microstepping mode to 1/32
  Axis[1].G_DECAY_REG.DECMOD = DECMOD_MIXAUTO ;  // Decay Mode to Mixed Auto
  Axis[1].G_BLANK_REG.TBLANK = 115 ;             // TBLANK to 3.3 uS
  Axis[1].G_BLANK_REG.ABT = ON ;                 // ABT ON
  Axis[1].G_DECAY_REG.TDECAY = 16 ;              // TDECAY to 8 uS
  Axis[1].G_OFF_REG.TOFF = 32 ;                  // TOFF to 16 uS
  Axis[1].G_CTRL_REG.ENBL = ON ;                 // Enable Motor

  Axis[2].G_TORQUE_REG.TORQUE = Z_Stopped_Torque ;// Set TORQUE value 0..255 ( Peak Amps = 2.75 * TORQUE / (256 * IGAIN * RSENSE))
  Axis[2].G_CTRL_REG.MODE = STEPS_32 ;           // Microstepping mode to 1/16
  Axis[2].G_DECAY_REG.DECMOD = DECMOD_MIXAUTO ;  // Decay Mode to Mixed Auto
  Axis[2].G_BLANK_REG.TBLANK = 115 ;             // TBLANK to 3.3 uS
  Axis[2].G_BLANK_REG.ABT = ON ;                 // ABT ON
  Axis[2].G_DECAY_REG.TDECAY = 16 ;              // TDECAY to 8 uS
  Axis[2].G_OFF_REG.TOFF = 32 ;                  // TOFF to 16 uS
  Axis[2].G_CTRL_REG.ENBL = ON ;                 // Enable Motor
  
  for (int i=0; i<3; i++){
    Axis[i].WriteAllRegisters() ;                  // Write the changes to Drivers
  }
}

//##########################################################################
void displaySettings ()
//##########################################################################
{
  // Get ISGAIN as a Number
  switch (Axis[currentAxis].G_CTRL_REG.ISGAIN) {
    case ISGAIN_5:
      ISGAIN = 5;
      break;
    case ISGAIN_10:
      ISGAIN = 10;
      break;
    case ISGAIN_20:
      ISGAIN = 20;
      break;
    case ISGAIN_40:
      ISGAIN = 40;
      break;
  }
  //calculate human friendly values
  Amps = 2.75 * Axis[currentAxis].G_TORQUE_REG.TORQUE / ( 256 * ISENSE * ISGAIN ) ;
  TBLANK = Axis[currentAxis].G_BLANK_REG.TBLANK * 0.02 + 1 ;
  if (Axis[currentAxis].G_BLANK_REG.ABT) { ABT = "ON"; } else { ABT = "OFF"; }
  TDECAY = Axis[currentAxis].G_DECAY_REG.TDECAY * 0.5 ;
  TOFF = Axis[currentAxis].G_OFF_REG.TOFF * 0.5 ;

  //output summary
  Serial.println ("############################################################################################") ;
  Serial.println (" DRV8711 Settings Summary") ;
  Serial.println (" Axis: " + String( currentAxis ) + " (0:X, 1:Y, 2:Z)");
  Serial.println (" Peak Current: " + String( Amps ) + "A    ( TORQUE: " + String(Axis[currentAxis].G_TORQUE_REG.TORQUE) +" , ISGAIN: " + String(ISGAIN) + " )");
  Serial.println (" MicroStep Mode: 1/" + String(pow(2,Axis[currentAxis].G_CTRL_REG.MODE),0));
  Serial.println (" Decay Mode: " + String(Axis[currentAxis].G_DECAY_REG.DECMOD) + " (0:SLOW, 1:SLOW/MIXED, 2:FAST, 3:MIXED , 4:SLOW/AUTO, 5:AUTOMIXED)");
  Serial.println (" TBLANK: " + String(TBLANK,2) + "uS");
  Serial.println (" ABT: " +  ABT ) ;
  Serial.println (" TDECAY: " + String(TDECAY,1) + "uS");
  Serial.println (" TOFF: " + String(TOFF,1) + "uS");
  Serial.print (" Motor ");
  if (Axis[currentAxis].G_CTRL_REG.ENBL) { Serial.println ("Enabled"); } else { Serial.println ("Disabled");}
  Serial.println ("############################################################################################") ;
}

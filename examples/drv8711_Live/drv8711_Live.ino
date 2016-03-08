//######################################################################################
//   Example sketch demonstrating the DRV8711 library for use with BOOST-DRV8711 Module 
//######################################################################################

#include <drv8711.h>
#include <SPI.h>

#define FaultPin 2  // connected to LED to indicate Fault 
#define SleepPin 9  // Drv8711 nSleep
#define ResetPin 7  // Drv8711 Reset
#define X_ssPin 3   // SPI ss Pins
#define Y_ssPin 4
#define Z_ssPin 5
#define EnableInPin 6 // Input from Grbl - Stepper Enable/Disable

const int nAxis = 3;

// Initialise an instance of the drv8711 object for each driver
drv8711 Axis[nAxis] = { drv8711(X_ssPin), drv8711(Y_ssPin), drv8711(Z_ssPin)}; //parameter is CSS Pin for Driver

// Current Settings
const int Moving_Torque[nAxis] = {186,186,186} ;
//const int Stopped_Torque[nAxis] = {93,93,93} ;

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

int readDelay = 1000 ;
long int LastRead = 0;
boolean LastEnableIn = LOW ;
boolean EnableVal = HIGH ;

//##########################################################################
void setup ()
//##########################################################################
{
  pinMode(FaultPin, OUTPUT) ;
  pinMode(SleepPin, OUTPUT) ;
  pinMode(ResetPin, OUTPUT) ;
  pinMode(X_ssPin, OUTPUT) ;
  pinMode(Y_ssPin, OUTPUT) ;
  pinMode(Z_ssPin, OUTPUT) ;
  pinMode(EnableInPin, INPUT) ;
  
  digitalWrite(FaultPin, HIGH);
  digitalWrite(SleepPin, HIGH);
  
  // Start Serial
  Serial.begin (115200) ;
  Serial.println();
 
  setRegisters();
  
}

//##########################################################################
void loop ()
//##########################################################################
{
   
   // Check for error flags & if found disable motor
   for(int i=0; i<nAxis; i++){
      if (Axis[i].ErrorFlag) {
        //Serial.println("Comms Error Axis:" + String(i));
        digitalWrite(FaultPin, HIGH);
        Axis[i].clear_error();
        Axis[i].disable();
        if (!Axis[i].ErrorFlag) Serial.println("Disabled Motor Axis:" + String(i) + ", Reset Required");   
        }
   }

   // Change TORQUE value when EnableInPin changes
   EnableVal = digitalRead(EnableInPin);
   if (EnableVal != LastEnableIn) {
     for (int i=0; i<nAxis; i++){
       if (EnableVal == LOW) {
         //Axis[i].G_TORQUE_REG.TORQUE = Moving_Torque[i] ;
         Axis[i].enable();  
       } else {
         //Axis[i].G_TORQUE_REG.TORQUE = Stopped_Torque[i] ;
         Axis[i].disable();  
       }
       //Axis[i].WriteTORQUERegister() ;
     }
     LastEnableIn = EnableVal ;
   }
   
  // Periodically check status registers
  if (millis() > LastRead + readDelay) checkStatus();
  
  // Check for Serial Input
  if (Serial.available()) {
    if ((char)Serial.read() == '?') { // if '?' received, display settings for all axis.
      for (int i=0; i<nAxis; i++){
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
  Serial.println("Initialising Drivers");
  
  // Load Library Defaults
  for(int i=0; i<nAxis; i++){
    Axis[i].clear_error();
    Axis[i].set_defaults();
  }
  
  // Make specific register settings
  Axis[0].G_TORQUE_REG.TORQUE = Moving_Torque[0] ;// Set TORQUE value 0..255 ( Peak Amps = 2.75 * TORQUE / (256 * IGAIN * RSENSE))
  Axis[0].G_CTRL_REG.MODE = STEPS_32 ;           // Microstepping mode to 1/32
  Axis[0].G_DECAY_REG.DECMOD = DECMOD_MIXAUTO ;  // Decay Mode to Mixed Auto
  Axis[0].G_BLANK_REG.TBLANK = 129 ;             // TBLANK to 2.6 uS
  Axis[0].G_BLANK_REG.ABT = ON ;                 // ABT ON
  Axis[0].G_DECAY_REG.TDECAY = 7 ;              // TDECAY to 4 uS
  Axis[0].G_OFF_REG.TOFF = 31 ;                  // TOFF to 16 uS
  
  Axis[1].G_TORQUE_REG.TORQUE = Moving_Torque[1] ;// Set TORQUE value 0..255 ( Peak Amps = 2.75 * TORQUE / (256 * IGAIN * RSENSE))
  Axis[1].G_CTRL_REG.MODE = STEPS_32 ;           // Microstepping mode to 1/32
  Axis[1].G_DECAY_REG.DECMOD = DECMOD_MIXAUTO ;  // Decay Mode to Mixed Auto
  Axis[1].G_BLANK_REG.TBLANK = 129 ;             // TBLANK to 2.6 uS
  Axis[1].G_BLANK_REG.ABT = ON ;                 // ABT ON
  Axis[1].G_DECAY_REG.TDECAY = 7 ;              // TDECAY to 4 uS
  Axis[1].G_OFF_REG.TOFF = 31 ;                  // TOFF to 16 uS
  
  Axis[2].G_TORQUE_REG.TORQUE = Moving_Torque[2] ;// Set TORQUE value 0..255 ( Peak Amps = 2.75 * TORQUE / (256 * IGAIN * RSENSE))
  Axis[2].G_CTRL_REG.MODE = STEPS_32 ;           // Microstepping mode to 1/16
  Axis[2].G_DECAY_REG.DECMOD = DECMOD_MIXAUTO ;  // Decay Mode to Mixed Auto
  Axis[2].G_BLANK_REG.TBLANK = 129 ;             // TBLANK to 2.6 uS
  Axis[2].G_BLANK_REG.ABT = ON ;                 // ABT ON
  Axis[2].G_DECAY_REG.TDECAY = 7 ;              // TDECAY to 4 uS
  Axis[2].G_OFF_REG.TOFF = 31 ;                  // TOFF to 16 uS
  
  // toggle Reset Pin
  digitalWrite (ResetPin, HIGH) ;
  delay (10) ;
  digitalWrite (ResetPin, LOW) ;
  delay (1) ;
  
  // Write Registers 
  Serial.println("##############################################");
  for (int i=0; i<nAxis; i++){
    Serial.println("######### Writing Registers Axis:" + String(i) + " ###########");
    Axis[i].WriteAllRegisters() ;                  // Write the changes to Drivers  
  }
  Serial.println("##############################################");
  
  // Enabling Motors only after all the other settings have been successfully written
  for (int i=0; i<nAxis; i++){
    if(!Axis[i].ErrorFlag){
      Axis[i].enable();
      Serial.println("Axis:"+ String(i) + " Enabled");
      digitalWrite(FaultPin, LOW);
    }
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
  TBLANK = ( Axis[currentAxis].G_BLANK_REG.TBLANK + 1 ) * 0.02 ;
  if (Axis[currentAxis].G_BLANK_REG.ABT) { ABT = "ON"; } else { ABT = "OFF"; }
  TDECAY = ( Axis[currentAxis].G_DECAY_REG.TDECAY + 1 ) * 0.5 ;
  TOFF = ( Axis[currentAxis].G_OFF_REG.TOFF + 1 ) * 0.5 ;

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

//##########################################################################
void checkStatus()
//##########################################################################
{
      for (int i=0; i<nAxis; i++){
       // only check status if motor enabled
       if (Axis[i].G_CTRL_REG.ENBL == ON) {
          Axis[i].get_status();
      
          if (Axis[i].G_STATUS_REG.UVLO) {
            Serial.println("ERROR: Axis:" + String(i) + " UnderVoltage Lockout");
            Axis[i].ErrorFlag = true; 
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
      }
   LastRead = millis();
}



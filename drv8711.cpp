#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "drv8711.h"

drv8711::drv8711 (int pin)
{
	pinMode(pin, OUTPUT);
	_pin = pin ;
	SavedStart = OFF ;
	
 }

void drv8711::init () 
{
    set_defaults();

    WriteAllRegisters();
  
 }

void drv8711::set_defaults ()
{
    
	// CTRL Register
	G_CTRL_REG.Address 	= 0x00;
	G_CTRL_REG.DTIME 	= DTIME_850; 	//Dead Time in ns
	G_CTRL_REG.ISGAIN   = ISGAIN_20;	//ISENSE amp gain 
	G_CTRL_REG.EXSTALL 	= OFF;			//External Stall Detect
	G_CTRL_REG.MODE     = STEPS_32;		//Microstepping Mode
	G_CTRL_REG.RSTEP 	= OFF;			//Step 
	G_CTRL_REG.RDIR 	= OFF;			//Reverse Direction Input
	G_CTRL_REG.ENBL 	= OFF;			//Enable Motor
	
	// TORQUE Register
	G_TORQUE_REG.Address = 0x01;
	G_TORQUE_REG.SIMPLTH = SIMPLTH_100;	//Back EMF Sample Threshold uS
	G_TORQUE_REG.TORQUE  = 128;			//Torque Value 0..255
        
	// OFF Register
	G_OFF_REG.Address 	= 0x02;
	G_OFF_REG.PWMMODE 	= OFF;			//PWM Direct Mode - OFF for Indexed Mode 
	G_OFF_REG.TOFF 		= 48;			//Off Time 0..255 * 500ns 

	// BLANK Register
	G_BLANK_REG.Address = 0x03;			
	G_BLANK_REG.ABT 	= ON;			//Enable Adaptive Blanking Time
	G_BLANK_REG.TBLANK 	= 100;			//Blanking Time 0..255 * 20ns + 1uS

	// DECAY Register.
	G_DECAY_REG.Address = 0x04;
	G_DECAY_REG.DECMOD  = DECMOD_MIXAUTO;	//Decay Mode
	G_DECAY_REG.TDECAY 	= 16;				//Decay Time 0..255 * 500ns

	// STALL Register
	G_STALL_REG.Address = 0x05;
	G_STALL_REG.VDIV 	= VDIV_4;		//Back EMF Div factor
	G_STALL_REG.SDCNT 	= SDCNT_8;		//Stall Step Count 
	G_STALL_REG.SDTHR 	= 0 ;			//Stall Detection Threshold 0..255

	// DRIVE Register
	G_DRIVE_REG.Address = 0x06;
	G_DRIVE_REG.IDRIVEP = IDRIVEP_50;	//High Side Gate Current mA (Source)
	G_DRIVE_REG.IDRIVEN = IDRIVEN_100;	//Low Side Gate Current mA (Sink)
	G_DRIVE_REG.TDRIVEP = TDRIVEP_500;	//High Side Gate Drive Time ns 
	G_DRIVE_REG.TDRIVEN = TDRIVEN_500;	//Low Side Gate Drive Time ns 
	G_DRIVE_REG.OCPDEG  = OCPDEG_2;		//OCP Deglitch time uS  
	G_DRIVE_REG.OCPTH   = OCPTH_500;	//OCP Threshold mV

	// STATUS Register
	G_STATUS_REG.Address = 0x07;
	G_STATUS_REG.STDLAT  = OFF;			//Latched Stall Detect
	G_STATUS_REG.STD     = OFF;  		//Stall Detected 
	G_STATUS_REG.UVLO    = OFF;  		//UnderVoltage Lockout  
	G_STATUS_REG.BPDF    = OFF; 	 	//Channel B Predriver Fault 
	G_STATUS_REG.APDF    = OFF;			//Channel A Predriver Fault 
	G_STATUS_REG.BOCP    = OFF;			//Channel B OverCurrent 
	G_STATUS_REG.AOCP    = OFF;			//Channel A OverCurrent 
	G_STATUS_REG.OTS     = OFF;			//Over Temperature 
	
}
 
void drv8711::enable ()
{
	// Set Enable
	G_CTRL_REG.ENBL 	= ON;  //enable motor
	WriteCTRLRegister();
}

void drv8711::disable ()
{
	// Set Disable Bit
	G_CTRL_REG.ENBL 	= OFF;  //disable motor
	WriteCTRLRegister();

}

void drv8711::get_status ()
{
	ReadSTATUSRegister();
}

void drv8711::clear_status ()
{
	G_STATUS_REG.STDLAT  = OFF;
	G_STATUS_REG.STD     = OFF;
	G_STATUS_REG.UVLO    = OFF;
	G_STATUS_REG.BPDF    = OFF;
	G_STATUS_REG.APDF    = OFF;
	G_STATUS_REG.BOCP    = OFF;
	G_STATUS_REG.AOCP    = OFF;
	G_STATUS_REG.OTS     = OFF;

	WriteSTATUSRegister();
}

void drv8711::ReadCTRLRegister()
{
    unsigned char dataHi = 0x00;
    const unsigned char dataLo = 0x00;
    unsigned int readData = 0x00;

    // Read CTRL Register
    dataHi = REGREAD | (G_CTRL_REG.Address << 4);
    readData = SPI_ReadWrite(dataHi, dataLo);
    G_CTRL_REG.DTIME        = ((readData >> 10) & 0x0003);
    G_CTRL_REG.ISGAIN       = ((readData >> 8) & 0x0003);
    G_CTRL_REG.EXSTALL      = ((readData >> 7) & 0x0001);
    G_CTRL_REG.MODE         = ((readData >> 3) & 0x000F);
    G_CTRL_REG.RSTEP        = ((readData >> 2) & 0x0001);
    G_CTRL_REG.RDIR         = ((readData >> 1) & 0x0001);
    G_CTRL_REG.ENBL         = ((readData >> 0) & 0x0001);

}

void drv8711::ReadTORQUERegister()
{
    unsigned char dataHi = 0x00;
    const unsigned char dataLo = 0x00;
    unsigned int readData = 0x00;

	// Read TORQUE Register
    dataHi = REGREAD | (G_TORQUE_REG.Address << 4);
    readData = SPI_ReadWrite(dataHi, dataLo);
    G_TORQUE_REG.SIMPLTH    = ((readData >> 8) & 0x0007);
    G_TORQUE_REG.TORQUE     = ((readData >> 0) & 0x00FF);
	
}

void drv8711::ReadOFFRegister()
{
    unsigned char dataHi = 0x00;
    const unsigned char dataLo = 0x00;
    unsigned int readData = 0x00;

    // Read OFF Register
    dataHi = REGREAD | (G_OFF_REG.Address << 4);
    readData = SPI_ReadWrite(dataHi, dataLo);
    G_OFF_REG.PWMMODE       = ((readData >> 8) & 0x0001);
    G_OFF_REG.TOFF          = ((readData >> 0) & 0x00FF);
	
}

void drv8711::ReadBLANKRegister()
{
    unsigned char dataHi = 0x00;
    const unsigned char dataLo = 0x00;
    unsigned int readData = 0x00;

    // Read BLANK Register
    dataHi = REGREAD | (G_BLANK_REG.Address << 4);
    readData = SPI_ReadWrite(dataHi, dataLo);
    G_BLANK_REG.ABT         = ((readData >> 8) & 0x0001);
    G_BLANK_REG.TBLANK      = ((readData >> 0) & 0x00FF);
}

void drv8711::ReadDECAYRegister()
{
    unsigned char dataHi = 0x00;
    const unsigned char dataLo = 0x00;
    unsigned int readData = 0x00;

    // Read DECAY Register
    dataHi = REGREAD | (G_DECAY_REG.Address << 4);
    readData = SPI_ReadWrite(dataHi, dataLo);
    G_DECAY_REG.DECMOD      = ((readData >> 8) & 0x0007);
    G_DECAY_REG.TDECAY      = ((readData >> 0) & 0x00FF);
}

void drv8711::ReadSTALLRegister()
{
    unsigned char dataHi = 0x00;
    const unsigned char dataLo = 0x00;
    unsigned int readData = 0x00;

    // Read STALL Register
    dataHi = REGREAD | (G_STALL_REG.Address << 4);
    readData = SPI_ReadWrite(dataHi, dataLo);
    G_STALL_REG.VDIV        = ((readData >> 10) & 0x0003);
    G_STALL_REG.SDCNT       = ((readData >> 8) & 0x0003);
    G_STALL_REG.SDTHR       = ((readData >> 0) & 0x00FF);
}

void drv8711::ReadDRIVERegister()
{
    unsigned char dataHi = 0x00;
    const unsigned char dataLo = 0x00;
    unsigned int readData = 0x00;

    // Read DRIVE Register
    dataHi = REGREAD | (G_DRIVE_REG.Address << 4);
    readData = SPI_ReadWrite(dataHi, dataLo);
    G_DRIVE_REG.IDRIVEP     = ((readData >> 10) & 0x0003);
    G_DRIVE_REG.IDRIVEN     = ((readData >> 8) & 0x0003);
    G_DRIVE_REG.TDRIVEP     = ((readData >> 6) & 0x0003);
    G_DRIVE_REG.TDRIVEN     = ((readData >> 4) & 0x0003);
    G_DRIVE_REG.OCPDEG      = ((readData >> 2) & 0x0003);
    G_DRIVE_REG.OCPTH       = ((readData >> 0) & 0x0003);
}

void drv8711::ReadSTATUSRegister()
{
    unsigned char dataHi = 0x00;
    const unsigned char dataLo = 0x00;
    unsigned int readData = 0x00;

    // Read STATUS Register
    dataHi = REGREAD | (G_STATUS_REG.Address << 4);
    readData = SPI_ReadWrite(dataHi, dataLo);
    G_STATUS_REG.STDLAT     = ((readData >> 7) & 0x0001);
    G_STATUS_REG.STD        = ((readData >> 6) & 0x0001);
    G_STATUS_REG.UVLO       = ((readData >> 5) & 0x0001);
    G_STATUS_REG.BPDF       = ((readData >> 4) & 0x0001);
    G_STATUS_REG.APDF       = ((readData >> 3) & 0x0001);
    G_STATUS_REG.BOCP       = ((readData >> 2) & 0x0001);
    G_STATUS_REG.AOCP       = ((readData >> 1) & 0x0001);
    G_STATUS_REG.OTS        = ((readData >> 0) & 0x0001);
}

void drv8711::ReadAllRegisters()
{
	ReadCTRLRegister();
	ReadTORQUERegister();	
	ReadOFFRegister();
	ReadBLANKRegister();
	ReadDECAYRegister();
	ReadSTALLRegister();
	ReadDRIVERegister();
	ReadSTATUSRegister();
}

void drv8711::WriteCTRLRegister()
{
    unsigned char dataHi = 0x00;
    unsigned char dataLo = 0x00;

    // Write CTRL Register
    dataHi = REGWRITE | (G_CTRL_REG.Address << 4) | (G_CTRL_REG.DTIME << 2) | (G_CTRL_REG.ISGAIN);
    dataLo = (G_CTRL_REG.EXSTALL << 7) | (G_CTRL_REG.MODE << 3) | (G_CTRL_REG.RSTEP << 2) | (G_CTRL_REG.RDIR << 1) | (G_CTRL_REG.ENBL);
    SPI_ReadWrite(dataHi, dataLo);
}
void drv8711::WriteTORQUERegister()
{
    unsigned char dataHi = 0x00;
    unsigned char dataLo = 0x00;

    // Write TORQUE Register
    dataHi = REGWRITE | (G_TORQUE_REG.Address << 4) | (G_TORQUE_REG.SIMPLTH);
    dataLo = G_TORQUE_REG.TORQUE;
    SPI_ReadWrite(dataHi, dataLo);
}	
void drv8711::WriteOFFRegister()
{
    unsigned char dataHi = 0x00;
    unsigned char dataLo = 0x00;

    // Write OFF Register
    dataHi = REGWRITE | (G_OFF_REG.Address << 4) | (G_OFF_REG.PWMMODE);
    dataLo = G_OFF_REG.TOFF;
    SPI_ReadWrite(dataHi, dataLo);
}
void drv8711::WriteBLANKRegister()
{
    unsigned char dataHi = 0x00;
    unsigned char dataLo = 0x00;

    // Write BLANK Register
    dataHi = REGWRITE | (G_BLANK_REG.Address << 4) | (G_BLANK_REG.ABT);
    dataLo = G_BLANK_REG.TBLANK;
    SPI_ReadWrite(dataHi, dataLo);
}
void drv8711::WriteDECAYRegister()
{
    unsigned char dataHi = 0x00;
    unsigned char dataLo = 0x00;

    // Write DECAY Register
    dataHi = REGWRITE | (G_DECAY_REG.Address << 4) | (G_DECAY_REG.DECMOD);
    dataLo = G_DECAY_REG.TDECAY;
    SPI_ReadWrite(dataHi, dataLo);
}
void drv8711::WriteSTALLRegister()
{
    unsigned char dataHi = 0x00;
    unsigned char dataLo = 0x00;

    // Write STALL Register
    dataHi = REGWRITE | (G_STALL_REG.Address << 4) | (G_STALL_REG.VDIV << 2) | (G_STALL_REG.SDCNT);
    dataLo = G_STALL_REG.SDTHR;
    SPI_ReadWrite(dataHi, dataLo);
}
void drv8711::WriteDRIVERegister()
{
    unsigned char dataHi = 0x00;
    unsigned char dataLo = 0x00;

    // Write DRIVE Register
    dataHi = REGWRITE | (G_DRIVE_REG.Address << 4) | (G_DRIVE_REG.IDRIVEP << 2) | (G_DRIVE_REG.IDRIVEN);
    dataLo = (G_DRIVE_REG.TDRIVEP << 6) | (G_DRIVE_REG.TDRIVEN << 4) | (G_DRIVE_REG.OCPDEG << 2) | (G_DRIVE_REG.OCPTH);
    SPI_ReadWrite(dataHi, dataLo);
}
void drv8711::WriteSTATUSRegister()
{
    unsigned char dataHi = 0x00;
    unsigned char dataLo = 0x00;

    // Write STATUS Register
    dataHi = REGWRITE | (G_STATUS_REG.Address << 4);
    dataLo = (G_STATUS_REG.STDLAT << 7) | (G_STATUS_REG.STD << 6) | (G_STATUS_REG.UVLO << 5) | (G_STATUS_REG.BPDF << 4) | (G_STATUS_REG.APDF << 3) | (G_STATUS_REG.BOCP << 2) | (G_STATUS_REG.AOCP << 1) | (G_STATUS_REG.OTS);
    SPI_ReadWrite(dataHi, dataLo);
}

void drv8711::WriteAllRegisters()
{
	WriteCTRLRegister();
	WriteTORQUERegister();	
	WriteOFFRegister();
	WriteBLANKRegister();
	WriteDECAYRegister();
	WriteSTALLRegister();
	WriteDRIVERegister();
	WriteSTATUSRegister();
}

unsigned int drv8711::SPI_ReadWrite(unsigned char dataHi, unsigned char dataLo)
{
	unsigned int readData = 0;
    SPI.begin(); 
    SPI.setClockDivider(SPI_CLOCK_DIV8);
	digitalWrite(_pin, HIGH);
	delayMicroseconds (1) ;
	readData |= (SPI.transfer(dataHi) << 8);
	readData |= SPI.transfer(dataLo);

	digitalWrite(_pin, LOW);

	return readData;
}
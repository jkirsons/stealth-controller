#include "Arduino.h"
#include <Wire.h>
#include <SimpleFOC.h>
#include <Math.h>
#include "drivers/drv8316/drv8316.h"

// DRV8316
#define DRV_MISO   21
#define DRV_MOSI   22
#define DRV_SCLK   19
#define DRV_SS     5

#define VREF        DAC1

// MA702
#define ENC_MISO   15
#define ENC_MOSI   20
#define ENC_SCLK   13
#define ENC_SS     4

// VSPI - for DRV8613
SPIClass * vspi = NULL;
// HSPI - for Encoder
SPIClass * hspi = NULL;

// magnetic sensor instance - SPI
MagneticSensorSPIConfig_s MA702_SPI = {
  .spi_mode = SPI_MODE0,
  .clock_speed = 1000000,
  .bit_resolution = 14,
  .angle_register = 0x0000,
  .data_start_bit = 15, 
  .command_rw_bit = 0,
  .command_parity_bit = 0
};

MagneticSensorSPI sensor = MagneticSensorSPI(MA702_SPI, ENC_SS );

BLDCMotor motor = BLDCMotor(11);
DRV8316Driver6PWM driver = DRV8316Driver6PWM(12,14,27,26,33,32,5,false/*,8,39*/);
//DRV8316Driver3PWM driver = DRV8316Driver3PWM(2,1,0,11,false);

Commander command = Commander(Serial);
void doCommander(char* cmd) { command.motor(&motor, cmd); }

unsigned long pidDelay = 0;
bool pids_set = false;

void printDRV8316Status() {
	driver.clearFault();
	delayMicroseconds(100); // ensure 400ns delay

	DRV8316Status status = driver.getStatus();
	Serial.println("DRV8316 Status:");
	Serial.print("Fault: ");
	Serial.println(status.isFault());
	Serial.print("Buck Error: ");
	Serial.print(status.isBuckError());
	Serial.print("  Undervoltage: ");
	Serial.print(status.isBuckUnderVoltage());
	Serial.print("  OverCurrent: ");
	Serial.println(status.isBuckOverCurrent());
	Serial.print("Charge Pump UnderVoltage: ");
	Serial.println(status.isChargePumpUnderVoltage());
	Serial.print("OTP Error: ");
	Serial.println(status.isOneTimeProgrammingError());
	Serial.print("OverCurrent: ");
	Serial.print(status.isOverCurrent());
	Serial.print("  Ah: ");
	Serial.print(status.isOverCurrent_Ah());
	Serial.print("  Al: ");
	Serial.print(status.isOverCurrent_Al());
	Serial.print("  Bh: ");
	Serial.print(status.isOverCurrent_Bh());
	Serial.print("  Bl: ");
	Serial.print(status.isOverCurrent_Bl());
	Serial.print("  Ch: ");
	Serial.print(status.isOverCurrent_Ch());
	Serial.print("  Cl: ");
	Serial.println(status.isOverCurrent_Cl());
	Serial.print("OverTemperature: ");
	Serial.print(status.isOverTemperature());
	Serial.print("  Shutdown: ");
	Serial.print(status.isOverTemperatureShutdown());
	Serial.print("  Warning: ");
	Serial.println(status.isOverTemperatureWarning());
	Serial.print("OverVoltage: ");
	Serial.println(status.isOverVoltage());
	Serial.print("PowerOnReset: ");
	Serial.println(status.isPowerOnReset());
	Serial.print("SPI Error: ");
	Serial.print(status.isSPIError());
	Serial.print("  Address: ");
	Serial.print(status.isSPIAddressError());
	Serial.print("  Clock: ");
	Serial.print(status.isSPIClockFramingError());
	Serial.print("  Parity: ");
	Serial.println(status.isSPIParityError());

	if (status.isFault())
		driver.clearFault();
	delayMicroseconds(1); // ensure 400ns delay
	DRV8316_PWMMode val = driver.getPWMMode();
	Serial.print("PWM Mode: ");
	Serial.println(val);
	delayMicroseconds(1); // ensure 400ns delay
	bool lock = driver.isRegistersLocked();
	Serial.print("Lock: ");
	Serial.println(lock);

	Serial.println("---");
	Serial.print("OvervoltageProtection: ");
	Serial.println(driver.isOvervoltageProtection());
	Serial.print("SPIFaultReporting: ");
	Serial.println(driver.isSPIFaultReporting());
	Serial.print("DriverOffEnabled: ");
	Serial.println(driver.isDriverOffEnabled());
	Serial.print("BuckPowerSequencingEnabled: ");
	Serial.println(driver.isBuckPowerSequencingEnabled());
	Serial.println("---");
}


void setup() {
	Serial.begin(115200);
	while (!Serial);
	delay(1);
	Serial.println("Initializing...");
	motor.useMonitoring(Serial);

	//setup VRef / ILim
	pinMode(VREF, OUTPUT);
	dacWrite(VREF, (uint8_t)(1.66/2.98*255.0));

	//setup driver off pin
	pinMode(8, OUTPUT);
	digitalWrite(8, 0);

	//setup fault pin
	pinMode(39, INPUT_PULLUP);

	driver.pwm_frequency = 190000;
	driver.voltage_power_supply = 12;

	hspi = new SPIClass(HSPI);
	hspi->begin(DRV_SCLK, DRV_MISO, DRV_MOSI, DRV_SS); //SCLK, MISO, MOSI, SS
	Serial.println("Driver Init...");
	driver.init(hspi);
	delayMicroseconds(1);
	driver.setBuckVoltage(DRV8316_BuckVoltage::VB_4V);
	delayMicroseconds(1);
	//driver.setPWMMode(DRV8316_PWMMode::PWM6_CurrentLimit_Mode);
	Serial.println("Buck Voltage Set...");

	vspi = new SPIClass(VSPI);
	vspi->begin(ENC_SCLK, ENC_MISO, ENC_MOSI, ENC_SS); //SCLK, MISO, MOSI, SS
	sensor.init(vspi);

	Serial.println("Driver Init complete...");

	motor.linkSensor(&sensor);
	motor.linkDriver(&driver);
	motor.controller = MotionControlType::angle;
	motor.foc_modulation = FOCModulationType::SinePWM;
	motor.motion_downsample = 5;
	motor.voltage_limit = 1;
	motor.voltage_sensor_align = 1;	
	motor.velocity_limit = 30;
  	motor.LPF_velocity.Tf = 0.005;	
	motor.PID_velocity.output_ramp = 500;
	
	// velocity PI controller parameters
	motor.PID_velocity.P = 0.0;
	motor.PID_velocity.I = 0.0;
	motor.PID_velocity.D = 0.0;	

	delay(100);
	printDRV8316Status();

	motor.target = motor.shaftAngle();	
	
	motor.init();
	motor.initFOC(); 

	motor.target = motor.shaftAngle();	

	Serial.println("Motor Init complete...");

	command.add('M', doCommander, "Commander Command");  
  	command.verbose = VerboseMode::user_friendly;
  	command.decimal_places = 4;
	
	pidDelay = millis() + 10;
}


void loop() {
	motor.loopFOC();
	motor.move();
	//motor.monitor();
	// user communication
  	command.run();

  // Smooth start the PIDs
  if(!pids_set && (millis() > pidDelay)) {
    motor.PID_velocity.P = 0.1;
    motor.PID_velocity.I = 10.0;
    motor.PID_velocity.D = 0.0;
    pids_set = true;
  }	
}


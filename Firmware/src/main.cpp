#include "Arduino.h"
#include <Wire.h>
#include <SimpleFOC.h>
#include "drivers/drv8316/drv8316.h"
#include "can/CANDriver.h"
#include "can/CANCommander.h"

// DRV8316
#define DRV_MISO   21
#define DRV_MOSI   22
#define DRV_SCLK   19
#define DRV_SS     5

#define DRV_VREF	DAC1
#define DRV_OFF		8
#define DRV_FAULT	39

// MA702
#define ENC_MISO   15
#define ENC_MOSI   20
#define ENC_SCLK   13
#define ENC_SS     4

// CAN
#define CAN_TX		7
#define CAN_RX 		34

SPIClass * drv_spi = NULL;
SPIClass * enc_spi = NULL;

// magnetic sensor instance - SPI
MagneticSensorSPIConfig_s MA702_SPI = {
  .spi_mode = SPI_MODE0,
  .clock_speed = 20000000,
  .bit_resolution = 14,
  .angle_register = 0x0000,
  .data_start_bit = 15, 
  .command_rw_bit = 0,
  .command_parity_bit = 0
};

MagneticSensorSPI sensor = MagneticSensorSPI(MA702_SPI, ENC_SS );

BLDCMotor motor = BLDCMotor(11);
DRV8316Driver6PWM driver = DRV8316Driver6PWM(12,14,27,26,33,32,5,false/*,8,39*/);

CANDriver can = CANDriver(CAN_TX, CAN_RX);
CANCommander command = CANCommander(can);
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

	command.add('M', doCommander, (char*)"motor");
	motor.useMonitoring(Serial);

	//setup VRef / ILim
	pinMode(DRV_VREF, OUTPUT);
	dacWrite(DRV_VREF, (uint8_t)(1.66/2.98*255.0));

	//setup driver off pin
	pinMode(DRV_OFF, OUTPUT);
	digitalWrite(DRV_OFF, 0);

	//setup fault pin
	pinMode(DRV_FAULT, INPUT_PULLUP);

	driver.pwm_frequency = 50000;
	driver.voltage_power_supply = 12;

	enc_spi = new SPIClass(HSPI);
	enc_spi->begin(DRV_SCLK, DRV_MISO, DRV_MOSI, DRV_SS);
	Serial.println("Driver Init...");
	driver.init(enc_spi);
	delayMicroseconds(1);
	driver.setBuckVoltage(DRV8316_BuckVoltage::VB_5V);
	delayMicroseconds(1);
	Serial.println("Buck Voltage Set...");
	//driver.setPWMMode(DRV8316_PWMMode::PWM6_CurrentLimit_Mode);

	drv_spi = new SPIClass(VSPI);
	drv_spi->begin(ENC_SCLK, ENC_MISO, ENC_MOSI, ENC_SS);
	sensor.init(drv_spi);

	Serial.println("Driver Init complete...");

	motor.linkSensor(&sensor);
	motor.linkDriver(&driver);
	motor.controller = MotionControlType::angle;
	motor.foc_modulation = FOCModulationType::SinePWM;
	//motor.motion_downsample = 10;
	motor.voltage_limit = 1.0;
	motor.voltage_sensor_align = 1.2;	
	motor.velocity_limit = 50;
	motor.LPF_velocity.Tf = 0.01;	
	motor.PID_velocity.output_ramp = 300;
	
	// velocity PI controller parameters
	motor.PID_velocity.P = 0.0;
	motor.PID_velocity.I = 0.0;
	motor.PID_velocity.D = 0.0;	

	delay(100);
	printDRV8316Status();

	motor.monitor_downsample = 0; // disable monitor at first
	motor.target = motor.shaftAngle();	
	motor.init();
	motor.initFOC(); 
	motor.target = motor.shaftAngle();	

	Serial.println("Motor Init complete...");

	pidDelay = millis() + 10;
}

int count = 0;
unsigned long lastTime = 0;

void loop() {
	motor.loopFOC();
	motor.move();
	motor.monitor();

	command.runWithCAN();
	
	// loop frequency counter
	count++;
	if(count > 100000)
	{
		unsigned long timeDiff = millis() - lastTime;
		if(timeDiff > 0) {
			float fps = 100000.0f / ((float)timeDiff / 1000.0f);
			printf("Main Loop Rate: %.6f\n", fps);
		}
		lastTime = millis();
		count = 0;
	}

	// Smooth start the PIDs
	if(!pids_set && (millis() > pidDelay)) {
		motor.PID_velocity.P = 0.08;
		motor.PID_velocity.I = 2.0;
		motor.PID_velocity.D = 0.0;
		pids_set = true;
	}	
}
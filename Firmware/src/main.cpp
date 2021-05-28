#include "Arduino.h"
#include <Wire.h>
#include <SimpleFOC.h>
#include <SimpleFOCCAN.h>
#include <StealthController.h>

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

MagneticSensorSPI sensor = MagneticSensorSPI(MA702_SPI, ENC_SS);

BLDCMotor motor = BLDCMotor(7);
DRV8316Driver6PWM driver = DRV8316Driver6PWM(DRV_A_H, DRV_A_L, DRV_B_H, DRV_B_L, DRV_C_H, DRV_C_L, DRV_SS, false/*, DRV_OFF, DRV_FAULT*/);

StealthController stealth = StealthController(driver);

CANDriver can = CANDriver(CAN_TX, CAN_RX);
CANCommander canCommand = CANCommander(can);

Commander command = Commander(Serial);

void doCommander(char* cmd) { command.motor(&motor, cmd); }
void doCommanderCAN(char* cmd) { canCommand.motor(&motor, cmd); }

void doCustom(char* cmd) { 
	switch(cmd[0]) {
		case 'S':
			driver.setSlew((DRV8316_Slew)atoi(&cmd[1])); 
			break;
	}
}

void setup() {	
	Serial.begin(115200);
	delay(100);
	Serial.println("Initializing...");

	command.add('M', doCommander, (char*)"motor");
	command.add('Z', doCustom, (char*)"Custom");
	canCommand.add('M', doCommanderCAN, (char*)"motor");
	
	motor.useMonitoring(Serial);

	stealth.setup();

	driver.pwm_frequency = 50000;
	driver.voltage_power_supply = 12;
	driver.init(stealth.enc_spi);
	motor.linkDriver(&driver);

	sensor.init(stealth.drv_spi);
	motor.linkSensor(&sensor);

	Serial.println("Driver Init complete...");

	motor.controller = MotionControlType::angle;
	motor.foc_modulation = FOCModulationType::SinePWM;
	//motor.motion_downsample = 10;
	motor.voltage_limit = 4.0;
	motor.voltage_sensor_align = 4.0;	
	motor.velocity_limit = 50;
	motor.LPF_velocity.Tf = 0.02;	
	motor.PID_velocity.output_ramp = 500;
	
	// velocity PI controller parameters
	motor.PID_velocity.P = 0.0;
	motor.PID_velocity.I = 0.0;
	motor.PID_velocity.D = 0.0;	

	delay(100);
	stealth.printStatus();

	motor.monitor_downsample = 0; // disable monitor at first
	motor.target = motor.shaftAngle();	
	motor.init();
	motor.initFOC(); 
	motor.target = motor.shaftAngle();	

	Serial.println("Motor Init complete...");
}

void loop() {
	motor.loopFOC();
	motor.move();
	motor.monitor();

	canCommand.runWithCAN();
	command.run();
	
	stealth.loopStep(motor, 0.15, 2.0, 0.0, true);
}
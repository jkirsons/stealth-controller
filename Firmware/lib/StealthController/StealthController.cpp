#include <StealthController.h>

StealthController::StealthController(DRV8316Driver &drv) {
  driver = &drv;
}

void StealthController::setup() {
  //setup VRef / ILim
	pinMode(DRV_VREF, OUTPUT);
	dacWrite(DRV_VREF, (uint8_t)(1.66/2.98*255.0));

	//setup driver off pin
	pinMode(DRV_OFF, OUTPUT);
	digitalWrite(DRV_OFF, 0);

	//setup fault pin
	pinMode(DRV_FAULT, INPUT_PULLUP);

  enc_spi = new SPIClass(HSPI);
	enc_spi->begin(DRV_SCLK, DRV_MISO, DRV_MOSI, DRV_SS);
	delayMicroseconds(1);
	driver->setBuckVoltage(DRV8316_BuckVoltage::VB_5V);
	delayMicroseconds(1);
	//driver.setPWMMode(DRV8316_PWMMode::PWM6_CurrentLimit_Mode);

	drv_spi = new SPIClass(VSPI);
	drv_spi->begin(ENC_SCLK, ENC_MISO, ENC_MOSI, ENC_SS);

}

void StealthController::loopStep(BLDCMotor &motor, float P, float I, float D, bool printLoopRate) {
	// loop frequency counter
	if(printLoopRate) {
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
  }

	// Smooth start the PIDs
  if(pidDelay == 0)
    pidDelay = millis() + 100;
    
	if(!pids_set && (millis() > pidDelay)) {
		motor.PID_velocity.P = P;
		motor.PID_velocity.I = I;
		motor.PID_velocity.D = D;
		pids_set = true;
	}	
}

void StealthController::printStatus() {
	driver->clearFault();
	delayMicroseconds(100); // ensure 400ns delay

	DRV8316Status status = driver->getStatus();
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
		driver->clearFault();
	delayMicroseconds(1); // ensure 400ns delay
	DRV8316_PWMMode val = driver->getPWMMode();
	Serial.print("PWM Mode: ");
	Serial.println(val);
	delayMicroseconds(1); // ensure 400ns delay
	bool lock = driver->isRegistersLocked();
	Serial.print("Lock: ");
	Serial.println(lock);

	Serial.println("---");
	Serial.print("OvervoltageProtection: ");
	Serial.println(driver->isOvervoltageProtection());
	Serial.print("SPIFaultReporting: ");
	Serial.println(driver->isSPIFaultReporting());
	Serial.print("DriverOffEnabled: ");
	Serial.println(driver->isDriverOffEnabled());
	Serial.print("BuckPowerSequencingEnabled: ");
	Serial.println(driver->isBuckPowerSequencingEnabled());
	Serial.println("---");
}
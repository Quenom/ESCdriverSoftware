#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/sc60228/MagneticSensorSC60228.h"

#include "drv8323.h"

#define UARTTX 0
#define UARTRX 1

#define SPI0_CLK 2
#define SPI0_TX 3
#define SPI0_RX 4
#define SPI0_CS 5

#define SPI1_CLK 14
#define SPI1_TX 15
#define SPI1_RX 12
#define DRV_A_CS 13
#define DRV_B_CS 22

#define ENCODER_A_CS 27
#define ENCODER_B_CS 26

#define A_Z_L 6
#define A_Z_H 7
#define A_Y_L 8
#define A_Y_H 9
#define A_X_L 10
#define A_X_H 11

#define B_Z_L 16
#define B_Z_H 17
#define B_Y_L 18
#define B_Y_H 19
#define B_X_L 20
#define B_X_H 21

#define VBattADC 28

// vbus 1V = 370.37 mV at ADC  →  ratio = 1 / 0.37037 = 2.7
#define VDIV_RATIO 2.8f
// RP2040 ADC: 12-bit (0–4095), 3.3V reference
#define ADC_REF 3.3f
#define ADC_RES 4095.0f

float readVbus() {
	uint16_t raw = analogRead(VBattADC);
	float vADC = (raw / ADC_RES) * ADC_REF;
	return vADC * VDIV_RATIO;
}

// ----- DRV8323 gate drivers (SPI1) ---------------------------
DRV8323 drvA(DRV_A_CS, SPI1);
DRV8323 drvB(DRV_B_CS, SPI1);

DRV8323Config drvCfg() {
	DRV8323Config cfg;

	// ── PWM ───────────────────────────────────────────────────
	cfg.pwmMode = PWM_6x;

	// ── Gate drive ────────────────────────────────────────────
	cfg.idriveP_HS = IDRIVEP_60MA;
	cfg.idriveN_HS = IDRIVEN_120MA;
	cfg.idriveP_LS = IDRIVEP_60MA;
	cfg.idriveN_LS = IDRIVEN_120MA;
	cfg.tdrive = TDRIVE_1000NS;
	cfg.deadTime = DEAD_100NS;

	// ── Overcurrent protection ────────────────────────────────
	cfg.ocpMode = OCP_REPORT_ONLY;
	cfg.ocpRetry = OCP_RETRY_4MS;
	cfg.ocpDeglitch = OCP_DEG_4US;
	cfg.vdsLevel = VDS_0V26;
	cfg.cbcMode = false;

	// ── Current sense amp ─────────────────────────────────────
	cfg.csaGain = CSA_GAIN_20;
	cfg.csaEnabled = false;
	cfg.csaCalA = false;
	cfg.csaCalB = false;
	cfg.csaCalC = false;
	cfg.csaFet = true;
	cfg.vrefDiv = true;
	cfg.senLvl = SEN_LVL_1V0;

	// ── Misc ──────────────────────────────────────────────────
	cfg.disableGateFaultReport = false;
	cfg.disableChargePumpUVLO = false;
	cfg.reportOTW = true;

	return cfg;
}

// (SC60228 on SPI1) ----------------------------
SPISettings encSPISettings(10000000, SC60228_BITORDER, SPI_MODE1);

MagneticSensorSC60228 encoderA(ENCODER_A_CS, encSPISettings);
MagneticSensorSC60228 encoderB(ENCODER_B_CS, encSPISettings);

// ----- Motors & drivers --------------------------------------
BLDCMotor motorA = BLDCMotor(7);
BLDCMotor motorB = BLDCMotor(7);

BLDCDriver6PWM driverA(A_X_H, A_X_L, A_Y_H, A_Y_L, A_Z_H, A_Z_L);
BLDCDriver6PWM driverB(B_X_H, B_X_L, B_Y_H, B_Y_L, B_Z_H, B_Z_L);

// ----- Commander ---------------------------------------------
Commander command = Commander(Serial); // TODO:CHANGE ALL SERIALS TO MATCH
void onMotorA(char* cmd) { command.motor(&motorA, cmd); }
void onMotorB(char* cmd) { command.motor(&motorB, cmd); }

// ADC

// ============================================================
void initMotor(BLDCMotor& motor, BLDCDriver6PWM& driver, MagneticSensorSC60228& encoder, float supplyVoltage,
		DRV8323& drv, SPIClassRP2040& spi = SPI1) {
	// encoder.init(reinterpret_cast<SPIClass*>(&spi));
	// motor.linkSensor(&encoder);

	driver.voltage_power_supply = supplyVoltage;
	driver.voltage_limit = supplyVoltage / 2;
	driverA.dead_zone = 0.05f;
	if (!driver.init()) {
		Serial.println("Driver init failed!");
		return;
	}
	motor.linkDriver(&driver);
	drv.begin(drvCfg());

	motor.controller = MotionControlType::velocity_openloop;
	motor.voltage_limit = 0.2f;
	motor.velocity_limit = 10.0f;

	motor.useMonitoring(Serial);
	if (!motor.init()) {
		Serial.println("Motor init failed!");
		return;
	}
	_delay(1000);
}

// ============================================================
void setup() {
	// Serial1.setTX(UARTTX);
	// Serial1.setRX(UARTRX);
	// Serial1.begin(115200);
	Serial.begin(115200);
	while (!Serial)
		;
	delay(100);
	SimpleFOCDebug::enable(&Serial);
	analogReadResolution(12); // use full 12-bit range on RP2040

	float vbus = readVbus();
	Serial.print("Vbus = ");
	Serial.print(vbus, 2);
	Serial.println(" V");

	// SPI1 – encoders (GP12/14/15)
	SPI1.setRX(SPI1_RX);
	SPI1.setTX(SPI1_TX);
	SPI1.setSCK(SPI1_CLK);
	SPI1.begin();

	encoderA.init(reinterpret_cast<SPIClass*>(&SPI1));
	encoderB.init(reinterpret_cast<SPIClass*>(&SPI1));

	SimpleFOCDebug::enable(&Serial);

	initMotor(motorA, driverA, encoderA, vbus, drvA, SPI);
	//  initMotor(motorB, driverB, encoderB, vbus,drvb, SPI);

	command.add('L', onMotorA, "motorA");
	command.add('R', onMotorB, "motorB");
}

// ============================================================
void loop() {

	encoderA.update();
	encoderB.update();

	motorA.loopFOC();
	//  motorB.loopFOC();

	motorA.move();
	//  motorB.move();

	motorA.monitor();
	//  motorB.monitor();

	command.run();

	if (drvA.hasFault()) {
		drvA.printStatus(Serial);
		drvA.clearFaults();
	}
}
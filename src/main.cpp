#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <cstdint>
#include "RP2040Support.h"
#include "SerialUSB.h"
#include "common/base_classes/CurrentSense.h"
#include "common/base_classes/Sensor.h"
#include "encoders/sc60228/MagneticSensorSC60228.h"

#include "drv8323.h"

#include "ADC124S051.h"

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
	cfg.idriveP_HS = IDRIVEP_570MA;
	cfg.idriveN_HS = IDRIVEN_1140MA;
	cfg.idriveP_LS = IDRIVEP_570MA;
	cfg.idriveN_LS = IDRIVEN_1140MA;
	cfg.tdrive = TDRIVE_1000NS;
	cfg.deadTime = DEAD_100NS;

	// ── Overcurrent protection ────────────────────────────────
	cfg.ocpMode = OCP_REPORT_ONLY;
	cfg.ocpRetry = OCP_RETRY_4MS;
	cfg.ocpDeglitch = OCP_DEG_8US;
	cfg.vdsLevel = VDS_0V45;
	cfg.cbcMode = false;

	// ── Current sense amp ─────────────────────────────────────
	cfg.csaGain = CSA_GAIN_40;
	cfg.csaEnabled = true;
	cfg.csaCalA = true;
	cfg.csaCalB = true;
	cfg.csaCalC = true;
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
SPISettings encSPISettings(8000000, SC60228_BITORDER, SPI_MODE1);

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
ADC124S051 adcek(SPI, SPI0_CS, 8 * 1000000);
volatile uint16_t adc_buf[4];
static spin_lock_t* adc_lock;
volatile bool adc_ready = false;
#define ADC_TO_AMPS (3.3f / 4095.0f / 0.04f)
typedef enum { CH_IB_B = 0, CH_IC_B = 1, CH_IB_A = 2, CH_IC_A = 3 } adc_channel_t;

float adc_offset[4] = {0.0f};

void calibrate_current_sensors(void) {
	const uint16_t samples = 1000;

	uint32_t sum[4] = {0};

	for (uint16_t i = 0; i < samples; i++) {
		uint32_t save = spin_lock_blocking(adc_lock);

		sum[0] += adc_buf[0];
		sum[1] += adc_buf[1];
		sum[2] += adc_buf[2];
		sum[3] += adc_buf[3];

		spin_unlock(adc_lock, save);

		delayMicroseconds(100);
	}

	for (uint8_t ch = 0; ch < 4; ch++) {
		adc_offset[ch] = (float)sum[ch] / (float)samples;
	}
}

static inline float counts_to_amps_ch(uint8_t ch) { return ((float)adc_buf[ch] - adc_offset[ch]) * ADC_TO_AMPS; }

static float ic_filt = 0, ib_filt = 0;
const float alpha = 0.1f; // tune this
const float MAX_AMPS = 5.0f;

PhaseCurrent_s readCurrentSenseA() {
	if (!adc_lock) {
		return {0, 0, 0};
	}
	uint32_t save = spin_lock_blocking(adc_lock);
	float ic = counts_to_amps_ch(CH_IC_A);
	float ib = counts_to_amps_ch(CH_IB_A);
	spin_unlock(adc_lock, save);

	if (fabsf(ic) < MAX_AMPS)
		ic_filt = alpha * ic + (1.0f - alpha) * ic_filt;
	if (fabsf(ib) < MAX_AMPS)
		ib_filt = alpha * ib + (1.0f - alpha) * ib_filt;

	PhaseCurrent_s c;
	c.b = ib_filt;
	c.c = ic_filt;
	c.a = 0;
	return c;
}
/*
PhaseCurrent_s readCurrentSenseB() {
	uint32_t save = spin_lock_blocking(adc_lock);
	PhaseCurrent_s c;
	uint8_t r = adc_read_idx;
	c.a = counts_to_amps_ch(r, 3);
	c.b = counts_to_amps_ch(r, 2);
	c.c = 0;
	spin_unlock(adc_lock, save);
	return c;
}
*/
// GenericCurrentSense currentSenseB(readCurrentSenseB);
GenericCurrentSense currentSenseA(readCurrentSenseA);

// ============================================================
void initMotor(BLDCMotor& motor, BLDCDriver6PWM& driver, MagneticSensorSC60228& encoder, float supplyVoltage,
		DRV8323& drv, GenericCurrentSense& cs, SPIClassRP2040& spi = SPI1) {
	encoder.init(reinterpret_cast<SPIClass*>(&spi));
	encoder.min_elapsed_time = 0.001;
	motor.linkSensor(&encoder);
	motor.voltage_sensor_align = 1.5f;
	driver.voltage_power_supply = supplyVoltage;
	driver.voltage_limit = supplyVoltage / 2;
	// driver.pwm_frequency = 10000;
	driverA.dead_zone = 0.05f;
	if (!driver.init()) {
		Serial.println("Driver init failed!");
		return;
	}
	motor.linkDriver(&driver);
	drv.begin(drvCfg());
	delay(100);
	drv.disableCsaCalibration();
	delay(500);
	calibrate_current_sensors();
	delay(5000);

	// cs.init();
	// motor.linkCurrentSense(&cs);
	motor.zero_electric_angle = 5.95f;
	motor.sensor_direction = Direction::CCW;

	motor.torque_controller = TorqueControlType::voltage;
	motor.controller = MotionControlType::torque;
	// motor.voltage_limit = 3.0f;
	// motor.KV_rating = 360;
	// motor.phase_resistance = 0.25;
	// motor.phase_inductance = 0.001; // 0,0001

	// float bandwidth = _2PI * 150.0;
	// motor.tuneCurrentController(200);
	
	motor.PID_velocity.P = 0.01f;
	motor.PID_velocity.I = 0.5f;
	motor.PID_velocity.D = 0.0f;
	motor.PID_velocity.output_ramp = 500.0f;
	motor.LPF_velocity.Tf = 0.001f;
	
	/*
		motor.current_limit = 1.0f;

			motor.PID_current_q.P = 0.0f;
			motor.PID_current_q.I = 0.0f;
			motor.PID_current_d.P = 0.0f;
			motor.PID_current_d.I = 0.0f;
			motor.PID_current_q.limit = 1.0f;
			motor.PID_current_d.limit = 1.0f;
		*/
	motor.useMonitoring(Serial);
	motor.monitor_downsample = 100;
	// motor.monitor_variables =_MON_VOLT_Q | _MON_VOLT_D;
	// motor.monitor_start_char = 'L';
	// motor.monitor_end_char = 'L';
	// command.verbose = VerboseMode::machine_readable;
	if (!motor.init()) {
		Serial.println("Motor init failed!");
		return;
	}
	if (!motor.initFOC()) {
		Serial.println("FOC init failed!");
		return;
	}
	delay(1000);
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

	SimpleFOCDebug::enable(&Serial);
	while (!adc_ready)
		;
	initMotor(motorA, driverA, encoderA, vbus, drvA, currentSenseA, SPI1);
	//  initMotor(motorB, driverB, encoderB, vbus,drvb, SPI);

	command.add('L', onMotorA, "motorA");
	command.add('R', onMotorB, "motorB");
}

void setup1() {
	adc_lock = spin_lock_init(spin_lock_claim_unused(true));
	SPI.setRX(SPI0_RX);
	SPI.setTX(SPI0_TX);
	SPI.setSCK(SPI0_CLK);
	SPI.begin();

	adcek.begin();
	adc_ready = true;
}

void loop1() {
	uint16_t tmp[4];
	adcek.readAll(tmp);
	uint32_t save = spin_lock_blocking(adc_lock);
	for (int i = 0; i < 4; i++)
		adc_buf[i] = tmp[i];
	spin_unlock(adc_lock, save);
}

// ============================================================
void loop() {
	motorA.loopFOC();
	//   motorB.loopFOC();

	motorA.move();
	//   motorB.move();

	motorA.monitor();

	//  motorB.monitor();
	command.run();
	if (drvA.hasFault()) {
		drvA.printStatus(Serial);
		drvA.clearFaults();
	}
}
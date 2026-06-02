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

#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "hardware/adc.h"

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

#define VBATT_ADC 2
#define VBATT_GPIO 28
#define VDIV_RATIO 2.8f
#define ADC_REF 3.3f
#define ADC_RES 4095.0f
#define ADC_SAMPLE_RATE 1000

volatile uint16_t adcRaw = 0;
int dmaChan = -1;
dma_channel_config dmaCfg;

void setupDMA() {
	dmaChan = dma_claim_unused_channel(true);
	dmaCfg = dma_channel_get_default_config(dmaChan);

	channel_config_set_transfer_data_size(&dmaCfg, DMA_SIZE_16);
	channel_config_set_read_increment(&dmaCfg, false);
	channel_config_set_write_increment(&dmaCfg, false);
	channel_config_set_dreq(&dmaCfg, DREQ_ADC);

	dma_channel_configure(dmaChan, &dmaCfg, &adcRaw, &adc_hw->fifo,
			0xFFFFFFFF, // essentially infinite transfers
			false);
}

void setupADC() {
	adc_init();
	adc_gpio_init(VBATT_GPIO);
	adc_select_input(VBATT_ADC);

	adc_fifo_setup(true, true, 1, false, false);

	// Set sample rate via clock divider
	// ADC clock is 48MHz, each conversion takes 96 cycles minimum
	// so max reliable sample rate is 500kHz
	adc_set_clkdiv(48000000.0f / ADC_SAMPLE_RATE);

	setupDMA();
	dma_channel_start(dmaChan);
	adc_run(true); // start continuous conversions
}

float readVbus() {
	uint16_t raw = adcRaw & 0x0FFF; // mask to 12-bit, DMA writes raw FIFO word
	float voltage = (raw * ADC_REF) / ADC_RES;
	return voltage * VDIV_RATIO;
}

// ----- DRV8323 gate drivers (SPI1) ---------------------------
DRV8323 drvA(DRV_A_CS, SPI1);
DRV8323 drvB(DRV_B_CS, SPI1);

DRV8323Config drvCfg() {
	DRV8323Config cfg;

	// ── PWM ───────────────────────────────────────────────────
	cfg.pwmMode = PWM_3x;

	// ── Gate drive ────────────────────────────────────────────
	cfg.idriveP_HS = IDRIVEP_260MA;
	cfg.idriveN_HS = IDRIVEN_280MA;
	cfg.idriveP_LS = IDRIVEP_260MA;
	cfg.idriveN_LS = IDRIVEN_280MA;
	cfg.tdrive = TDRIVE_1000NS;
	cfg.deadTime = DEAD_800NS;

	// ── Overcurrent protection ────────────────────────────────
	cfg.ocpMode = OCP_RETRY;
	cfg.ocpRetry = OCP_RETRY_4MS;
	cfg.ocpDeglitch = OCP_DEG_8US;
	cfg.vdsLevel = VDS_0V75;
	cfg.cbcMode = false;

	// ── Current sense amp ─────────────────────────────────────
	cfg.csaGain = CSA_GAIN_20;
	cfg.csaEnabled = true;
	cfg.csaCalA = true;
	cfg.csaCalB = true;
	cfg.csaCalC = true;
	cfg.csaFet = false;
	cfg.vrefDiv = true;
	cfg.senLvl = SEN_LVL_1V0;

	// ── Misc ──────────────────────────────────────────────────
	cfg.disableGateFaultReport = false;
	cfg.disableChargePumpUVLO = false;
	cfg.reportOTW = true;

	return cfg;
}

// (SC60228 on SPI1) ----------------------------
SPISettings encSPISettings(1000000, SC60228_BITORDER, SPI_MODE1);

MagneticSensorSC60228 encoderA(ENCODER_A_CS);
MagneticSensorSC60228 encoderB(ENCODER_B_CS);

// ----- Motors & drivers --------------------------------------
BLDCMotor motorA = BLDCMotor(7);
BLDCMotor motorB = BLDCMotor(7);

BLDCDriver6PWM driverA(A_X_H, A_X_L, A_Y_H, A_Y_L, A_Z_H, A_Z_L);
BLDCDriver6PWM driverB(B_X_H, B_X_L, B_Y_H, B_Y_L, B_Z_H, B_Z_L);

// ----- Commander ---------------------------------------------
Commander command = Commander(Serial); // TODO:CHANGE this
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
const float MAX_AMPS = 5.0f;

PhaseCurrent_s readCurrentSenseA() {
	while (!adc_lock) {}
	uint32_t save = spin_lock_blocking(adc_lock);
	float ic = counts_to_amps_ch(CH_IC_A);
	float ib = counts_to_amps_ch(CH_IB_A);
	spin_unlock(adc_lock, save);

	/*
	if (fabsf(ic) < MAX_AMPS)
		ic_filt = alpha * ic + (1.0f - alpha) * ic_filt;
	if (fabsf(ib) < MAX_AMPS)
		ib_filt = alpha * ib + (1.0f - alpha) * ib_filt;
	*/
	PhaseCurrent_s c;
	c.c = ic;
	c.b = ib;
	c.a = 0;
	// Serial.printf("C:%f , B:%f\n", c.c, c.b);
	return c;
}

PhaseCurrent_s readCurrentSenseB() {
	while (!adc_lock) {}
	uint32_t save = spin_lock_blocking(adc_lock);
	float ic = counts_to_amps_ch(CH_IC_B);
	float ib = counts_to_amps_ch(CH_IB_B);
	spin_unlock(adc_lock, save);

	/*
	if (fabsf(ic) < MAX_AMPS)
		ic_filt = alpha * ic + (1.0f - alpha) * ic_filt;
	if (fabsf(ib) < MAX_AMPS)
		ib_filt = alpha * ib + (1.0f - alpha) * ib_filt;
	*/
	PhaseCurrent_s c;
	c.c = ic;
	c.b = ib;
	c.a = 0;
	// Serial.printf("C:%f , B:%f\n", c.c, c.b);
	return c;
}

volatile bool sample_now = false;
uint sliceA;
uint sliceB;
volatile bool pwm_ready = false;
/*PhaseCurrent_s readCurrentSenseB() {
	uint32_t save = spin_lock_blocking(adc_lock);
	PhaseCurrent_s c;
	uint8_t r = adc_read_idx;
	c.a = counts_to_amps_ch(r, 3);
	c.b = counts_to_amps_ch(r, 2);
	c.c = 0;
	spin_unlock(adc_lock, save);
	return c;
}*/

// GenericCurrentSense currentSenseB(readCurrentSenseB);
GenericCurrentSense currentSenseA(readCurrentSenseA);
GenericCurrentSense currentSenseB(readCurrentSenseB);

// ============================================================
void initMotor(BLDCMotor& motor, BLDCDriver6PWM& driver, MagneticSensorSC60228& encoder, float supplyVoltage,
		DRV8323& drv, SPIClassRP2040& spi = SPI1) {
	encoder.init(reinterpret_cast<SPIClass*>(&spi));
	encoderA.min_elapsed_time = 0.005f;
	motor.linkSensor(&encoder);
	motor.voltage_sensor_align = 1.5f;
	driver.voltage_power_supply = supplyVoltage;
	driver.voltage_limit = supplyVoltage;
	driver.pwm_frequency = 5000;
	driver.dead_zone = 0.01f;
	if (!driver.init()) {
		Serial.println("Driver init failed!");
		return;
	}
	motor.linkDriver(&driver);
	drv.begin(drvCfg());
	delay(200);
	drv.disableCsaCalibration();
	delay(200);
	calibrate_current_sensors();
	delay(200);
	Serial.print("ADC offsets: ");
	for (int i = 0; i < 4; i++) {
		Serial.print(adc_offset[i]);
		Serial.print(" ");
	}
	Serial.println();
	pwm_ready = 1;

	currentSenseA.init();
	motor.linkCurrentSense(&currentSenseA);
	// motor.zero_electric_angle = 3.1f;
	// motor.sensor_direction = Direction::CCW;
	// motor.foc_modulation = FOCModulationType::SinePWM;
	motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
	motor.modulation_centered = false;
	motor.torque_controller = TorqueControlType::foc_current;
	motor.controller = MotionControlType::velocity;
	motor.voltage_limit = supplyVoltage / 1.5f;
	motor.KV_rating = 360;
	motor.phase_resistance = 0.85;
	motor.phase_inductance = 0.000116; // 0,0001
	motor.current_limit = 1.0f;

	// motor.tuneCurrentController(200);
	// MOT:Tunned PI params for BW [Hz]: 200.00
	// MOT:Pq: 0.15
	// MOT:Iq: 1068.14
	// MOT:Pd: 0.15
	// MOT:Id: 1068.14

	float max_velocity = 200.0;							// rad/s
	float motor_frequency_hz = max_velocity / (2 * PI); // ~16 Hz
	float filter_cutoff_hz = motor_frequency_hz * 5;	// ~80 Hz
	/*
	// Tf = 1 / (2 * PI * f_cutoff)
	motor.LPF_velocity.Tf = 1.0 / (2.0 * PI * filter_cutoff_hz);


	motor.PID_velocity.P = 0.4f;
	motor.PID_velocity.I = 0.2f;
	motor.PID_velocity.D = 0.0f;
	motor.PID_velocity.output_ramp = 20.0f;
	motor.LPF_velocity.Tf = 0.05f;
	*/
	motor.LPF_velocity.Tf = 1.0 / (2.0 * PI * filter_cutoff_hz);

	motor.PID_velocity.P = 0.15f;
	motor.PID_velocity.I = 1.0f;
	motor.PID_velocity.D = 0.0f;
	motor.PID_velocity.output_ramp = 20.0f;
	motor.LPF_velocity.Tf = 0;

	//
	motor.PID_current_q.P = 0.4f;
	motor.PID_current_q.I = 0.005f;
	motor.PID_current_q.limit = 2.0f;
	motor.PID_current_d.P = 0.4f;
	motor.PID_current_d.I = 0.005f;
	motor.PID_current_d.limit = 2.0f;
	motor.LPF_current_d.Tf = 0;
	motor.LPF_current_q.Tf = 0;

	motor.PID_current_q.output_ramp = 1000.0f;
	motor.PID_current_d.output_ramp = 1000.0f;

	motor.useMonitoring(Serial);
	motor.monitor_downsample = 100;
	// motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VOLT_D | _MON_CURR_Q | _MON_CURR_D | _MON_VEL |
	// _MON_ANGLE;
	motor.monitor_start_char = 'L';
	motor.monitor_end_char = 'L';
	motor.monitor_variables = _MON_TARGET | _MON_VEL;
	command.verbose = VerboseMode::machine_readable;
	if (!motor.init()) {
		Serial.println("Motor init failed!");
		return;
	}

	// motor.characteriseMotor(1.0f);

	if (!motor.initFOC()) {
		Serial.println("FOC init failed!");
		return;
	}
	delay(1000);
}

void initMotor2(BLDCMotor& motor, BLDCDriver6PWM& driver, MagneticSensorSC60228& encoder, float supplyVoltage,
		DRV8323& drv, SPIClassRP2040& spi = SPI1) {
	encoder.init(reinterpret_cast<SPIClass*>(&spi));
	encoder.min_elapsed_time = 0.005f;
	motor.linkSensor(&encoder);
	motor.voltage_sensor_align = 1.5f;
	driver.voltage_power_supply = supplyVoltage;
	driver.voltage_limit = supplyVoltage;
	driver.pwm_frequency = 5000;
	// driver.dead_zone = 0.01f;
	if (!driver.init()) {
		Serial.println("Driver init failed!");
		return;
	}
	motor.linkDriver(&driver);
	drv.begin(drvCfg());
	delay(200);
	drv.disableCsaCalibration();
	delay(200);
	calibrate_current_sensors();
	delay(200);
	Serial.print("ADC offsets: ");
	for (int i = 0; i < 4; i++) {
		Serial.print(adc_offset[i]);
		Serial.print(" ");
	}
	Serial.println();
	pwm_ready = 1;

	// currentSenseA.init();
	//  motor.linkCurrentSense(&currentSenseA);
	//   motor.zero_electric_angle = 3.1f;
	//   motor.sensor_direction = Direction::CCW;
	motor.foc_modulation = FOCModulationType::SinePWM;
	// motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
	// motor.modulation_centered = false;
	motor.torque_controller = TorqueControlType::estimated_current;
	motor.controller = MotionControlType::velocity;
	motor.voltage_limit = supplyVoltage / 2.0f;
	motor.KV_rating = 360;
	motor.phase_resistance = 0.85;
	motor.phase_inductance = 0.000116; // 0,0001
	motor.current_limit = 1.0f;

	// motor.tuneCurrentController(200);
	// MOT:Tunned PI params for BW [Hz]: 200.00
	// MOT:Pq: 0.15
	// MOT:Iq: 1068.14
	// MOT:Pd: 0.15
	// MOT:Id: 1068.14

	float max_velocity = 200.0;							// rad/s
	float motor_frequency_hz = max_velocity / (2 * PI); // ~16 Hz
	float filter_cutoff_hz = motor_frequency_hz * 5;	// ~80 Hz
	/*
	// Tf = 1 / (2 * PI * f_cutoff)
	motor.LPF_velocity.Tf = 1.0 / (2.0 * PI * filter_cutoff_hz);


	motor.PID_velocity.P = 0.4f;
	motor.PID_velocity.I = 0.2f;
	motor.PID_velocity.D = 0.0f;
	motor.PID_velocity.output_ramp = 20.0f;
	motor.LPF_velocity.Tf = 0.05f;
	*/
	motor.LPF_velocity.Tf = 1.0 / (2.0 * PI * filter_cutoff_hz);

	motor.PID_velocity.P = 0.15f;
	motor.PID_velocity.I = 1.0f;
	motor.PID_velocity.D = 0.0f;
	motor.PID_velocity.output_ramp = 20.0f;
	motor.LPF_velocity.Tf = 0;

	//
	motor.PID_current_q.P = 0.4f;
	motor.PID_current_q.I = 0.005f;
	motor.PID_current_q.limit = 2.0f;
	motor.PID_current_d.P = 0.4f;
	motor.PID_current_d.I = 0.005f;
	motor.PID_current_d.limit = 2.0f;
	motor.LPF_current_d.Tf = 0;
	motor.LPF_current_q.Tf = 0;

	motor.PID_current_q.output_ramp = 1000.0f;
	motor.PID_current_d.output_ramp = 1000.0f;

	motor.useMonitoring(Serial);
	motor.monitor_downsample = 100;
	// motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VOLT_D | _MON_CURR_Q | _MON_CURR_D | _MON_VEL |
	// _MON_ANGLE;
	motor.monitor_start_char = 'R';
	motor.monitor_end_char = 'R';
	motor.monitor_variables = _MON_TARGET | _MON_VEL;
	command.verbose = VerboseMode::machine_readable;
	if (!motor.init()) {
		Serial.println("Motor init failed!");
		return;
	}

	// motor.characteriseMotor(1.0f);

	if (!motor.initFOC()) {
		Serial.println("FOC init failed!");
		return;
	}
	delay(1000);
}
// ============================================================
void setup() {
	Serial1.setTX(UARTTX);
	Serial1.setRX(UARTRX);
	Serial1.begin(115200);
	Serial.begin(115200);
	while (!Serial)
		;
	delay(100);
	setupADC();
	while (adcRaw == 0) {
		tight_loop_contents();
	}
	float vbus = readVbus();
	Serial.print("Vbus = ");
	Serial.print(vbus, 2);
	Serial.println(" V");

	// SPI1 – encoders (GP12/14/15)
	SPI1.setRX(SPI1_RX);
	SPI1.setTX(SPI1_TX);
	SPI1.setSCK(SPI1_CLK);
	pinMode(DRV_A_CS, OUTPUT);
	digitalWrite(DRV_A_CS, HIGH);
	pinMode(DRV_B_CS, OUTPUT);
	digitalWrite(DRV_B_CS, HIGH);
	SPI1.begin();
	delay(1000);

	SimpleFOCDebug::enable(&Serial);
	while (!adc_ready)
		;
	delay(100);

	initMotor(motorA, driverA, encoderA, vbus, drvA, SPI1);
	// initMotor2(motorB, driverB, encoderB, vbus, drvB, SPI1);
	command.add('L', onMotorA, "motorA");
	//command.add('R', onMotorB, "motorB");
}

void pwm_isr() {
	pwm_clear_irq(sliceA);
	busy_wait_us(90);
	sample_now = true;
}

void setup1() {
	adc_lock = spin_lock_init(spin_lock_claim_unused(true));
	SPI.setRX(SPI0_RX);
	SPI.setTX(SPI0_TX);
	SPI.setSCK(SPI0_CLK);
	SPI.begin();
	adcek.begin();
	adc_ready = true;

	while (!pwm_ready) {
		uint16_t tmp[4];
		adcek.readAll(tmp);
		uint32_t save = spin_lock_blocking(adc_lock);
		for (int i = 0; i < 4; i++) {
			adc_buf[i] = tmp[i];
		};
		spin_unlock(adc_lock, save);
	}
	sliceA = pwm_gpio_to_slice_num(6);
	pwm_set_irq_enabled(sliceA, true);
	irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_isr);
	irq_set_enabled(PWM_IRQ_WRAP, true);
}

void loop1() {
	if (!sample_now)
		return;
	sample_now = false;
	uint16_t tmp[4];
	adcek.readAll(tmp);
	uint32_t save = spin_lock_blocking(adc_lock);
	for (int i = 0; i < 4; i++)
		adc_buf[i] = tmp[i];
	spin_unlock(adc_lock, save);
	// Serial.printf("ch1:%d,ch2:%d\n", adc_buf[0], adc_buf[1]);
}

uint8_t loopmove = 0;
// ============================================================
bool motorEnabledA = true;
bool motorEnabledB = true;

void loop() {
	bool shouldEnableA = fabsf(motorA.target) >= 0.1f || fabsf(motorA.shaft_velocity) >= 1.0f;
	if (shouldEnableA) {
		if (!motorEnabledA) {
			motorA.enable();
			motorEnabledA = true;
		}
	} else {
		if (motorEnabledA) {
			motorA.disable();
			motorEnabledA = false;
		}
	}
	/*
	bool shouldEnableB = fabsf(motorB.target) >= 0.1f || fabsf(motorB.shaft_velocity) >= 1.0f;
	if (shouldEnableB) {
		if (!motorEnabledB) {
			motorB.enable();
			motorEnabledB = true;
		}
	} else {
		if (motorEnabledB) {
			motorB.disable();
			motorEnabledB = false;
		}
	}*/

	motorA.move();
	motorA.loopFOC();
	// motorB.move();
	// motorB.loopFOC();

	motorA.monitor();

	// motorB.monitor();

	command.run();
	if (drvA.hasFault()) {
		Serial.println("=== DRV A FAULT ===");
		drvA.printStatus(Serial);
		drvA.clearFaults();
	}
	if (drvB.hasFault()) {
		Serial.println("=== DRV B FAULT ===");
		drvB.printStatus(Serial);
		drvB.clearFaults();
	}
	static uint32_t lastVbusUpdate = 0;
	uint32_t now = millis();
	if (now - lastVbusUpdate >= 100) {
		float supplyVoltage = readVbus();
		driverA.voltage_power_supply = supplyVoltage;
		driverA.voltage_limit = supplyVoltage;
		motorA.voltage_limit = supplyVoltage / 1.5f;
		driverB.voltage_power_supply = supplyVoltage;
		driverB.voltage_limit = supplyVoltage;
		motorB.voltage_limit = supplyVoltage / 1.5f;
		lastVbusUpdate = now;
	}
}
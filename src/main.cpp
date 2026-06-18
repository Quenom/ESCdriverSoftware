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

	dma_channel_configure(dmaChan, &dmaCfg, &adcRaw, &adc_hw->fifo, 0xFFFFFFFF, false);
}

void setupADC() {
	adc_init();
	adc_gpio_init(VBATT_GPIO);
	adc_select_input(VBATT_ADC);

	adc_fifo_setup(true, true, 1, false, false);

	adc_set_clkdiv(48000000.0f / ADC_SAMPLE_RATE);

	setupDMA();
	dma_channel_start(dmaChan);
	adc_run(true);
}

float readVbus() {
	uint16_t raw = adcRaw & 0x0FFF;
	float voltage = (raw * ADC_REF) / ADC_RES;
	return voltage * VDIV_RATIO;
}

DRV8323 drvA(DRV_A_CS, SPI1);
DRV8323 drvB(DRV_B_CS, SPI1);

DRV8323Config drvCfg() {
	DRV8323Config cfg;

	// ── PWM
	cfg.pwmMode = PWM_6x;

	// ── Gate drive
	cfg.idriveP_HS = IDRIVEP_260MA;
	cfg.idriveN_HS = IDRIVEN_280MA;
	cfg.idriveP_LS = IDRIVEP_260MA;
	cfg.idriveN_LS = IDRIVEN_280MA;
	cfg.tdrive = TDRIVE_1000NS;
	cfg.deadTime = DEAD_800NS;

	// ── Overcurrent protection
	cfg.ocpMode = OCP_LATCH;
	cfg.ocpRetry = OCP_RETRY_4MS;
	cfg.ocpDeglitch = OCP_DEG_8US;
	cfg.vdsLevel = VDS_0V75;
	cfg.cbcMode = false;

	// ── Current sense amp
	cfg.csaGain = CSA_GAIN_20;
	cfg.csaEnabled = true;
	cfg.csaCalA = true;
	cfg.csaCalB = true;
	cfg.csaCalC = true;
	cfg.csaFet = false;
	cfg.vrefDiv = true;
	cfg.senLvl = SEN_LVL_1V0;

	// ── Misc
	cfg.disableGateFaultReport = false;
	cfg.disableChargePumpUVLO = false;
	cfg.reportOTW = true;

	return cfg;
}

SPISettings encSPISettings(1000000, SC60228_BITORDER, SPI_MODE1);

MagneticSensorSC60228 encoderA(ENCODER_A_CS);
MagneticSensorSC60228 encoderB(ENCODER_B_CS);

// ----- Motors & drivers --------------------------------------
BLDCMotor motorA = BLDCMotor(7);
BLDCMotor motorB = BLDCMotor(7);

BLDCDriver6PWM driverA(A_X_H, A_X_L, A_Y_H, A_Y_L, A_Z_H, A_Z_L);
BLDCDriver6PWM driverB(B_X_H, B_X_L, B_Y_H, B_Y_L, B_Z_H, B_Z_L);

// ----- Commander ---------------------------------------------
Commander command = Commander(Serial1); // TODO:CHANGE this
void onMotorA(char* cmd) { command.motor(&motorA, cmd); }
void onMotorB(char* cmd) { command.motor(&motorB, cmd); }

// ADC
ADC124S051 adcek(spi0, SPI0_CS, 8 * 1000000);
volatile uint16_t adc_buf[4];
static spin_lock_t* adc_lock;
volatile bool adc_ready = false;
#define ADC_TO_AMPS (3.3f / 4095.0f / 0.02f) // 3.3Vref / 12bit / 1mohm shunt*20x gain
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

PhaseCurrent_s readCurrentSenseA() {
	while (!adc_lock) {}
	uint32_t save = spin_lock_blocking(adc_lock);
	float ic = counts_to_amps_ch(CH_IC_A);
	float ib = counts_to_amps_ch(CH_IB_A);
	spin_unlock(adc_lock, save);
	PhaseCurrent_s c;
	c.c = ic;
	c.b = ib;
	c.a = 0;
	return c;
}

PhaseCurrent_s readCurrentSenseB() {
	while (!adc_lock) {}
	uint32_t save = spin_lock_blocking(adc_lock);
	float ic = counts_to_amps_ch(CH_IC_B);
	float ib = counts_to_amps_ch(CH_IB_B);
	spin_unlock(adc_lock, save);
	PhaseCurrent_s c;
	c.c = ic;
	c.b = ib;
	c.a = 0;
	return c;
}

volatile bool sample_now = false;
uint sliceA;
uint sliceB;
volatile bool pwm_ready = false;

GenericCurrentSense currentSenseA(readCurrentSenseA);
GenericCurrentSense currentSenseB(readCurrentSenseB);

// ============================================================
void initMotor(float supplyVoltage) {
	encoderA.init(reinterpret_cast<SPIClass*>(&SPI1));
	encoderA.min_elapsed_time = 0.005f;
	encoderB.init(reinterpret_cast<SPIClass*>(&SPI1));
	encoderB.min_elapsed_time = 0.005f;

	motorA.linkSensor(&encoderA);
	motorA.voltage_sensor_align = 1.5f;
	motorB.linkSensor(&encoderB);
	motorB.voltage_sensor_align = 1.5f;

	driverA.voltage_power_supply = supplyVoltage;
	driverA.voltage_limit = supplyVoltage;
	driverA.pwm_frequency = 10000;
	driverA.dead_zone = 0.02f;
	driverB.voltage_power_supply = supplyVoltage;
	driverB.voltage_limit = supplyVoltage;
	driverB.pwm_frequency = 10000;
	driverB.dead_zone = 0.02f;
	if (!driverA.init()) {
		Serial.println("DriverA init failed!");
		return;
	}
	if (!driverB.init()) {
		Serial.println("DriverB init failed!");
		return;
	}
	motorA.linkDriver(&driverA);
	motorB.linkDriver(&driverB);

	drvA.begin(drvCfg());
	delay(200);
	drvA.disableCsaCalibration();
	delay(200);

	drvB.begin(drvCfg());
	delay(200);
	drvB.disableCsaCalibration();
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
	currentSenseB.init();

	motorA.linkCurrentSense(&currentSenseA);
	motorA.foc_modulation = FOCModulationType::SpaceVectorPWM;
	motorA.modulation_centered = false;
	motorA.torque_controller = TorqueControlType::foc_current;
	motorA.controller = MotionControlType::velocity;
	motorA.velocity_limit = 45;
	motorA.voltage_limit = supplyVoltage / 1.5;
	motorA.KV_rating = 360;
	motorA.phase_resistance = 0.49;
	motorA.phase_inductance = 0.000116; // 0,0001
	motorA.current_limit = 5.0f;

	motorA.PID_velocity.P = 0.6f;
	motorA.PID_velocity.I = 0.2f;
	motorA.PID_velocity.D = 0.0f;
	motorA.PID_velocity.output_ramp = 100.0f;
	motorA.LPF_velocity.Tf = 0.05f;

	motorA.PID_current_q.P = 0.15f;
	motorA.PID_current_d.P = 0.15f;
	motorA.PID_current_q.I = 0.005f;
	motorA.PID_current_d.I = 0.005f;

	motorA.PID_current_q.limit = 10.0f;
	motorA.PID_current_d.limit = 10.0f;
	motorA.LPF_current_d.Tf = 0.0005f;
	motorA.LPF_current_q.Tf = 0.0005f;

	motorA.PID_current_q.output_ramp = 1000.0f;
	motorA.PID_current_d.output_ramp = 1000.0f;

	motorB.linkCurrentSense(&currentSenseB);
	motorB.foc_modulation = FOCModulationType::SpaceVectorPWM;
	motorB.modulation_centered = false;
	motorB.torque_controller = TorqueControlType::foc_current;
	motorB.controller = MotionControlType::velocity;
	motorB.velocity_limit = 45;
	motorB.voltage_limit = supplyVoltage / 1.5f;
	motorB.KV_rating = 360;
	motorB.phase_resistance = 0.49;
	motorB.phase_inductance = 0.000116; // 0,0001
	motorB.current_limit = 5.0f;

	motorB.PID_velocity.P = 0.6f;
	motorB.PID_velocity.I = 0.2f;
	motorB.PID_velocity.D = 0.0f;
	motorB.PID_velocity.output_ramp = 100.0f;
	motorB.LPF_velocity.Tf = 0.05f;

	motorB.PID_current_q.P = 0.15f;
	motorB.PID_current_d.P = 0.15f;
	motorB.PID_current_q.I = 0.005f;
	motorB.PID_current_d.I = 0.005f;

	motorB.PID_current_q.limit = 10.0f;
	motorB.PID_current_d.limit = 10.0f;
	motorB.LPF_current_d.Tf = 0.0005f;
	motorB.LPF_current_q.Tf = 0.0005f;

	motorB.PID_current_q.output_ramp = 1000.0f;
	motorB.PID_current_d.output_ramp = 1000.0f;

	motorA.useMonitoring(Serial1);
	motorA.monitor_downsample = 10;
	motorA.monitor_start_char = 'L';
	motorA.monitor_end_char = 'L';
	motorA.monitor_variables = _MON_TARGET | _MON_VEL;

	motorB.useMonitoring(Serial1);
	motorB.monitor_downsample = 10;
	motorB.monitor_start_char = 'R';
	motorB.monitor_end_char = 'R';
	motorB.monitor_variables = _MON_TARGET | _MON_VEL;

	command.verbose = VerboseMode::machine_readable;
	if (!motorA.init()) {
		Serial.println("Motor-A init failed!");
		return;
	}
	if (!motorB.init()) {
		Serial.println("Motor-B init failed!");
		return;
	}

	if (!motorA.initFOC()) {
		Serial.println("FOC-A init failed!");
		return;
	}
	if (!motorB.initFOC()) {
		Serial.println("FOC-B init failed!");
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
	while (!Serial1)
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

	initMotor(vbus);
	command.add('L', onMotorA, "motorA");
	command.add('R', onMotorB, "motorB");
}

void pwm_isr() {
	pwm_clear_irq(sliceA);
	busy_wait_us(44);
	sample_now = true;
}

void setup1() {
	adc_lock = spin_lock_init(spin_lock_claim_unused(true));
	gpio_set_function(SPI0_RX, GPIO_FUNC_SPI);
	gpio_set_function(SPI0_CLK, GPIO_FUNC_SPI);
	gpio_set_function(SPI0_TX, GPIO_FUNC_SPI);
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
	}

	motorA.move();
	motorA.loopFOC();
	motorB.move();
	motorB.loopFOC();

	motorA.monitor();
	motorB.monitor();

	command.run();
	if (drvA.hasFault()) {
		Serial.println("=== DRV A FAULT ===");
		drvA.printStatus(Serial);
		// drvA.clearFaults();
	}
	if (drvB.hasFault()) {
		Serial.println("=== DRV B FAULT ===");
		drvB.printStatus(Serial);
		// drvB.clearFaults();
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
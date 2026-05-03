#pragma once
#include <Arduino.h>
#include <SPI.h>

// RP2040 earlephilhower core uses SPIClassRP2040
using DrvSPIClass = SPIClassRP2040;

// ═══════════════════════════════════════════════════════════════
//  DRV8323 – User-friendly configuration
//  Edit the enums and DRV8323Config struct below to configure.
//  No need to touch raw register values.
// ═══════════════════════════════════════════════════════════════

// ── PWM mode ──────────────────────────────────────────────────
enum DrvPwmMode {
	PWM_6x = 0, // 6 independent INH/INL inputs
	PWM_3x = 1, // 3 INH inputs, DRV handles low-side internally
	PWM_1x = 2, // 1 input (direction + brake)
};

// ── OCP (overcurrent) behaviour ───────────────────────────────
enum DrvOcpMode {
	OCP_LATCH = 0,		 // latch fault – requires CLR_FLT to recover
	OCP_RETRY = 1,		 // auto retry after TRETRY delay
	OCP_REPORT_ONLY = 2, // assert nFAULT but keep driving
	OCP_DISABLED = 3,	 // no overcurrent protection
};

// ── OCP retry time (when OCP_RETRY selected) ──────────────────
enum DrvOcpRetry {
	OCP_RETRY_4MS = 0,
	OCP_RETRY_50US = 1,
};

// ── OCP deglitch time ─────────────────────────────────────────
enum DrvOcpDeglitch {
	OCP_DEG_1US = 0,
	OCP_DEG_2US = 1,
	OCP_DEG_4US = 2,
	OCP_DEG_8US = 3,
};

// ── VDS overcurrent threshold ─────────────────────────────────
enum DrvVdsLevel {
	VDS_0V06 = 0, // 0.06 V – very sensitive
	VDS_0V13 = 1,
	VDS_0V20 = 2,
	VDS_0V26 = 3,
	VDS_0V31 = 4,
	VDS_0V45 = 5,
	VDS_0V53 = 6,
	VDS_0V60 = 7,
	VDS_0V68 = 8,
	VDS_0V75 = 9,
	VDS_0V94 = 10,
	VDS_1V13 = 11,
	VDS_1V30 = 12,
	VDS_1V50 = 13,
	VDS_1V70 = 14,
	VDS_1V88 = 15,
};

// ── Dead time ─────────────────────────────────────────────────
enum DrvDeadTime {
	DEAD_100NS = 0,
	DEAD_200NS = 1,
	DEAD_400NS = 2,
	DEAD_800NS = 3,
};

// ── Peak gate drive time ──────────────────────────────────────
enum DrvTdrive {
	TDRIVE_500NS = 0,
	TDRIVE_1000NS = 1,
	TDRIVE_2000NS = 2,
	TDRIVE_4000NS = 3,
};

// ── Gate drive source current (IDRIVEP) ───────────────────────
// Higher = faster turn-on, more ringing. Start low for testing.
enum DrvIdriveP {
	IDRIVEP_10MA = 0,
	IDRIVEP_30MA = 1,
	IDRIVEP_60MA = 2,
	IDRIVEP_80MA = 3,
	IDRIVEP_120MA = 4,
	IDRIVEP_140MA = 5,
	IDRIVEP_170MA = 6,
	IDRIVEP_190MA = 7,
	IDRIVEP_260MA = 8,
	IDRIVEP_330MA = 9,
	IDRIVEP_370MA = 10,
	IDRIVEP_440MA = 11,
	IDRIVEP_570MA = 12,
	IDRIVEP_680MA = 13,
	IDRIVEP_820MA = 14,
	IDRIVEP_1000MA = 15,
};

// ── Gate drive sink current (IDRIVEN) ─────────────────────────
enum DrvIdriveN {
	IDRIVEN_20MA = 0,
	IDRIVEN_60MA = 1,
	IDRIVEN_120MA = 2,
	IDRIVEN_160MA = 3,
	IDRIVEN_240MA = 4,
	IDRIVEN_280MA = 5,
	IDRIVEN_340MA = 6,
	IDRIVEN_380MA = 7,
	IDRIVEN_520MA = 8,
	IDRIVEN_660MA = 9,
	IDRIVEN_740MA = 10,
	IDRIVEN_880MA = 11,
	IDRIVEN_1140MA = 12,
	IDRIVEN_1360MA = 13,
	IDRIVEN_1640MA = 14,
	IDRIVEN_2000MA = 15,
};

// ── CSA gain ──────────────────────────────────────────────────
// gain = 3.3V / (I_max * R_shunt)  e.g. 20A * 10mΩ → 16.5 → use 20x
enum DrvCsaGain {
	CSA_GAIN_5 = 0,
	CSA_GAIN_10 = 1,
	CSA_GAIN_20 = 2,
	CSA_GAIN_40 = 3,
};

// ── CSA sense level threshold ─────────────────────────────────
enum DrvSenLvl {
	SEN_LVL_0V25 = 0,
	SEN_LVL_0V5 = 1,
	SEN_LVL_0V75 = 2,
	SEN_LVL_1V0 = 3,
};

// ═══════════════════════════════════════════════════════════════
//  Configuration struct – fill this in, pass to DRV8323::begin()
// ═══════════════════════════════════════════════════════════════
struct DRV8323Config {
	// ── PWM ───────────────────────────────────────────────────
	DrvPwmMode pwmMode = PWM_3x;

	// ── Gate drive ────────────────────────────────────────────
	DrvIdriveP idriveP_HS = IDRIVEP_60MA;  // HS source
	DrvIdriveN idriveN_HS = IDRIVEN_120MA; // HS sink
	DrvIdriveP idriveP_LS = IDRIVEP_60MA;  // LS source
	DrvIdriveN idriveN_LS = IDRIVEN_120MA; // LS sink
	DrvTdrive tdrive = TDRIVE_2000NS;
	DrvDeadTime deadTime = DEAD_100NS;

	// ── Overcurrent protection ────────────────────────────────
	DrvOcpMode ocpMode = OCP_REPORT_ONLY;
	DrvOcpRetry ocpRetry = OCP_RETRY_4MS;
	DrvOcpDeglitch ocpDeglitch = OCP_DEG_4US;
	DrvVdsLevel vdsLevel = VDS_0V45;
	bool cbcMode = true; // retry OCP on each PWM cycle

	// ── Current sense amp ─────────────────────────────────────
	DrvCsaGain csaGain = CSA_GAIN_20;
	bool csaEnabled = false; // set true when current sensing ready
	bool csaCalA = false;	 // calibrate phase A offset
	bool csaCalB = false;
	bool csaCalC = false;
	bool csaFet = true;				// true = low-side FET shunt reference
	bool vrefDiv = true;			// true = VREF/2 reference
	DrvSenLvl senLvl = SEN_LVL_1V0; // sense level threshold

	// ── Misc ──────────────────────────────────────────────────
	bool disableGateFaultReport = true; // DIS_GDF
	bool disableChargePumpUVLO = false; // DIS_CPUV
	bool reportOTW = false;				// OTW_REP
};

// ═══════════════════════════════════════════════════════════════
//  DRV8323 driver class
// ═══════════════════════════════════════════════════════════════
class DRV8323 {
public:
	DRV8323(uint8_t csPin, DrvSPIClass& spi = SPI1)
			: _cs(csPin), _spi(spi), _spiSettings(1000000, MSBFIRST, SPI_MODE1) {}

	void begin(const DRV8323Config& cfg = DRV8323Config()) {
		_cfg = cfg;
		pinMode(_cs, OUTPUT);
		digitalWrite(_cs, HIGH);
		delay(10);
		applyConfig();
		delay(5);
		verify(Serial);
	}

	void applyConfig() {
		write(0x02, buildDrvCtrl());
		delay(1);
		write(0x03, buildGateHS());
		delay(1);
		write(0x04, buildGateLS());
		delay(1);
		write(0x05, buildOcpCtrl());
		delay(1);
		write(0x06, buildCsaCtrl());
		delay(1);
	}

	// Returns true if all registers match expected values.
	// Note: LOCK bits in GateDriveHS (0x03) are write-only and masked out.
	bool verify(Stream& serial) {
		bool ok = true;

		struct {
			uint8_t addr;
			uint16_t expected;
			uint16_t mask;
			const char* name;
		} regs[] = {
				{0x02, buildDrvCtrl(), 0x7FF, "DriverControl"},
				{0x03, buildGateHS(), 0x0FF, "GateDriveHS"}, // mask out LOCK bits [11:8]
				{0x04, buildGateLS(), 0x7FF, "GateDriveLS"},
				{0x05, buildOcpCtrl(), 0x7FF, "OCPControl"},
				{0x06, buildCsaCtrl(), 0x7FF, "CSAControl"},
		};

		for (auto& r : regs) {
			uint16_t actual = read(r.addr) & r.mask;
			uint16_t expected = r.expected & r.mask;
			if (actual != expected) {
				serial.print("  MISMATCH ");
				serial.print(r.name);
				serial.print("  expected=0x");
				serial.print(expected, HEX);
				serial.print("  got=0x");
				serial.println(actual, HEX);
				ok = false;
			}
		}

		if (ok)
			serial.println("  DRV8323 config OK");
		return ok;
	}

	void clearFaults() {
		uint16_t drvCtrl = read(0x02);
		write(0x02, drvCtrl | 1); // set CLR_FLT
		delay(1);
		write(0x02, drvCtrl & ~1); // clear CLR_FLT
	}

	// Returns true if any real fault bits are set.
	bool hasFault() { return (read(0x00) != 0) || (read(0x01) != 0); }

	void printStatus(Stream& serial) {
		uint16_t fault = read(0x00);
		uint16_t vgs = read(0x01);
		serial.print("  FAULT=0x");
		serial.print(fault, HEX);
		serial.print("  VGS=0x");
		serial.print(vgs, HEX);
		if (!fault && !vgs) {
			serial.println("  OK");
		} else {
			serial.println();
			if (fault & (1 << 10))
				serial.println("    FAULT: OTW  – overtemp warning");
			if (fault & (1 << 9))
				serial.println("    FAULT: OTSD – overtemp shutdown");
			if (fault & (1 << 8))
				serial.println("    FAULT: UVLO – undervoltage");
			if (fault & (1 << 7))
				serial.println("    FAULT: GDF  – gate drive fault");
			if (fault & (1 << 6))
				serial.println("    FAULT: VDS  – overcurrent VDS");
			if (vgs & (1 << 10))
				serial.println("    VGS:   GDUV – gate drive UV");
			if (vgs & (1 << 9))
				serial.println("    VGS:   VGS_HA");
			if (vgs & (1 << 8))
				serial.println("    VGS:   VGS_LA");
			if (vgs & (1 << 7))
				serial.println("    VGS:   VGS_HB");
			if (vgs & (1 << 6))
				serial.println("    VGS:   VGS_LB");
			if (vgs & (1 << 5))
				serial.println("    VGS:   VGS_HC");
			if (vgs & (1 << 4))
				serial.println("    VGS:   VGS_LC");
		}
	}

	uint16_t read(uint8_t addr) { return transfer(0x8000 | ((addr & 0x7) << 11)) & 0x7FF; }
	void write(uint8_t addr, uint16_t v) { transfer(((addr & 0x7) << 11) | (v & 0x7FF)); }

private:
	uint8_t _cs;
	DrvSPIClass& _spi;
	SPISettings _spiSettings;
	DRV8323Config _cfg;

	// ── Register builders ─────────────────────────────────────
	uint16_t buildDrvCtrl() {
		return (_cfg.disableChargePumpUVLO ? 1 : 0) << 9 | (_cfg.disableGateFaultReport ? 1 : 0) << 8
				| (_cfg.reportOTW ? 1 : 0) << 7 | (uint16_t)_cfg.pwmMode << 5;
	}

	uint16_t buildGateHS() {
		return 3 << 8 // LOCK = 0b0011 to unlock writes
				| (uint16_t)_cfg.idriveP_HS << 4 | (uint16_t)_cfg.idriveN_HS;
	}

	uint16_t buildGateLS() {
		return (_cfg.cbcMode ? 1 : 0) << 10 | (uint16_t)_cfg.tdrive << 8 | (uint16_t)_cfg.idriveP_LS << 4
				| (uint16_t)_cfg.idriveN_LS;
	}

	uint16_t buildOcpCtrl() {
		return (uint16_t)_cfg.ocpRetry << 10 | (uint16_t)_cfg.deadTime << 8 | (uint16_t)_cfg.ocpMode << 6
				| (uint16_t)_cfg.ocpDeglitch << 4 | (uint16_t)_cfg.vdsLevel;
	}

	uint16_t buildCsaCtrl() {
		return (_cfg.csaFet ? 1 : 0) << 10 | (_cfg.vrefDiv ? 1 : 0) << 9 | (uint16_t)_cfg.csaGain << 6
				| (_cfg.csaEnabled ? 0 : 1) << 5 // DIS_SEN is inverted
				| (_cfg.csaCalA ? 1 : 0) << 4 | (_cfg.csaCalB ? 1 : 0) << 3 | (_cfg.csaCalC ? 1 : 0) << 2
				| (uint16_t)_cfg.senLvl;
	}

	uint16_t transfer(uint16_t tx) {
		uint8_t txBuf[2] = {(uint8_t)(tx >> 8), (uint8_t)(tx & 0xFF)};
		uint8_t rxBuf[2] = {0, 0};
		_spi.beginTransaction(_spiSettings);
		digitalWrite(_cs, LOW);
		_spi.transfer(txBuf, rxBuf, 2);
		digitalWrite(_cs, HIGH);
		_spi.endTransaction();
		return ((uint16_t)rxBuf[0] << 8) | rxBuf[1];
	}
};
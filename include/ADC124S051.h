
#pragma once

#include <Arduino.h>
#include <SPI.h>
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#define ADC124S051_MAX_VALUE 4095
#define ADC124S051_MAX_CHANNEL 4

class ADC124S051 {
public:
	ADC124S051(spi_inst_t* spi, uint8_t csPin, uint32_t speed = 8000000UL)
			: _spi(spi), _cs(csPin), _speed(speed), _lastChannel(255), _count(0) {}

	void begin() {
		// init SPI peripheral
		spi_init(_spi, _speed);
		spi_set_format(_spi,
				16,			// 16 bits per transfer
				SPI_CPOL_1, // MODE3: CPOL=1, CPHA=1
				SPI_CPHA_1, SPI_MSB_FIRST);

		// CS as fast GPIO
		gpio_init(_cs);
		gpio_set_dir(_cs, GPIO_OUT);
		gpio_put(_cs, 1);
	}

	// single channel read
	// note: pipelined - result is for previously selected channel
	uint16_t read(uint8_t ch) {
		if (ch >= ADC124S051_MAX_CHANNEL)
			return 0;

		if (ch != _lastChannel) {
			_lastChannel = ch;
			_transfer(chAddr(ch)); // dummy to set channel
		}

		_count++;
		return _transfer(chAddr(ch));
	}

	// read all 4 channels as fast as possible
	// uses pipelined channel select - total: 5 transfers, 4 results
	void readAll(uint16_t (&out)[ADC124S051_MAX_CHANNEL]) {
		// precompute all control words
		uint16_t cmd[ADC124S051_MAX_CHANNEL + 1];
		for (uint8_t i = 0; i <= ADC124S051_MAX_CHANNEL; i++)
			cmd[i] = chAddr(i % ADC124S051_MAX_CHANNEL);

		uint16_t raw[ADC124S051_MAX_CHANNEL + 1];

		// CS low once, do all transfers back to back
		// CS must pulse between conversions per datasheet (min 10ns)
		// at 8MHz each transfer is 2us, CS pulse overhead is minimal
		for (uint8_t i = 0; i <= ADC124S051_MAX_CHANNEL; i++) {
			gpio_put(_cs, 0);
			spi_write16_read16_blocking(_spi, &cmd[i], &raw[i], 1);
			gpio_put(_cs, 1);
			// no delay needed - gpio_put takes ~2 cycles = >10ns at 133MHz
		}

		// first result is garbage (pipeline warmup), skip it
		for (uint8_t i = 0; i < ADC124S051_MAX_CHANNEL; i++)
			out[i] = raw[i + 1] & 0x0FFF;

		_count += ADC124S051_MAX_CHANNEL;
		_lastChannel = 255;
	}

	// burst read: fill a buffer with repeated reads of one channel
	// fastest possible - minimal overhead per sample
	void readBurst(uint8_t ch, uint16_t* buf, uint16_t n) {
		if (ch >= ADC124S051_MAX_CHANNEL)
			return;

		uint16_t cmd = chAddr(ch);
		uint16_t dummy;

		// pipeline prime
		gpio_put(_cs, 0);
		spi_write16_read16_blocking(_spi, &cmd, &dummy, 1);
		gpio_put(_cs, 1);

		for (uint16_t i = 0; i < n; i++) {
			gpio_put(_cs, 0);
			spi_write16_read16_blocking(_spi, &cmd, &buf[i], 1);
			gpio_put(_cs, 1);
			buf[i] &= 0x0FFF;
		}

		_count += n;
		_lastChannel = ch;
	}

	void setSPIspeed(uint32_t speed) {
		_speed = speed;
		spi_set_baudrate(_spi, speed);
	}

	uint32_t getSPIspeed() const { return spi_get_baudrate(_spi); }
	uint16_t maxValue() const { return ADC124S051_MAX_VALUE; }
	uint8_t maxChannel() const { return ADC124S051_MAX_CHANNEL; }
	uint8_t lastChannel() const { return _lastChannel; }
	uint32_t count() const { return _count; }

private:
	spi_inst_t* _spi;
	uint8_t _cs;
	uint32_t _speed;
	uint8_t _lastChannel;
	uint32_t _count;

	static constexpr uint16_t chAddr(uint8_t ch) { return (uint16_t)ch << 11; }

	inline uint16_t _transfer(uint16_t cmd) {
		uint16_t result;
		gpio_put(_cs, 0);
		spi_write16_read16_blocking(_spi, &cmd, &result, 1);
		gpio_put(_cs, 1);
		return result & 0x0FFF;
	}
};
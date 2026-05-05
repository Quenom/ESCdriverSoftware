#pragma once
//
//  FILE: ADC124S051.h
//  PURPOSE: RP2040 library for ADC124S051 - 12-bit, 4-channel ADC (SPI)
//  DEVICE: ADC124S051 (TI) - 12-bit, 4-channel, 200kSPS–1MSPS
//  SPI:    MODE3, MSBFIRST, CS active LOW
//
//  Channel addressing bits [D4:D3] in 16-bit transfer:
//    CH0 -> 0x0000
//    CH1 -> 0x0800
//    CH2 -> 0x1000
//    CH3 -> 0x1800
//
//  NOTE: ADC124S051 returns data for channel N-1 while you address channel N.
//        A dummy read is required on channel switch (or on first read).
//        readAll() handles this correctly: 1 dummy + 4 real reads.

#include <Arduino.h>
#include <SPI.h>

#define ADC124S051_MAX_VALUE    4095
#define ADC124S051_MAX_CHANNEL  4
#define ADC124S051_SPI_MODE     SPI_MODE3
#define ADC124S051_DEFAULT_SPEED 1000000UL  // 1 MHz, safe default (max 8 MHz at 3.3V)

class ADC124S051
{
public:
	ADC124S051(SPIClassRP2040 &spi, uint8_t csPin, uint32_t speed = ADC124S051_DEFAULT_SPEED)
		: _spi(&spi), _cs(csPin), _speed(speed),
		_lastChannel(255), _count(0)
	{
		_settings = SPISettings(_speed, MSBFIRST, ADC124S051_SPI_MODE);
	}

	void begin()
	{
		pinMode(_cs, OUTPUT);
		digitalWrite(_cs, HIGH);
	}

	// Single channel read. Returns 12-bit value 0..4095.
	// Issues a dummy read automatically on channel change.
	uint16_t read(uint8_t ch)
	{
		if (ch >= ADC124S051_MAX_CHANNEL) return 0;

		if (ch != _lastChannel)
		{
		_lastChannel = ch;
		transfer16(chAddr(ch));  // dummy - flushes previous channel
		}

		_count++;
		return transfer16(chAddr(ch));
	}

	// Read all 4 channels: 1 dummy (ch0 select) + 4 real reads.
	// Results written to out[0..3]. Returns false if out is null.
	void readAll(uint16_t (&out)[ADC124S051_MAX_CHANNEL])
	{
		// Dummy read to select CH0 (result is stale, discard)
		transfer16(chAddr(0));
		_lastChannel = 0;

		// Each transfer addresses the next channel while returning current channel data.
		for (uint8_t ch = 0; ch < ADC124S051_MAX_CHANNEL; ch++)
		{
		uint8_t nextCh = (ch + 1) % ADC124S051_MAX_CHANNEL;
			out[ch] = transfer16(chAddr(nextCh)) & 0x0FFF;
		_count++;
		}

		_lastChannel = 255;  // force dummy on next single read() call
	}

	uint16_t maxValue()    const { return ADC124S051_MAX_VALUE; }
	uint8_t  maxChannel()  const { return ADC124S051_MAX_CHANNEL; }
	uint8_t  lastChannel() const { return _lastChannel; }
	uint32_t count()       const { return _count; }

	void setSPIspeed(uint32_t speed)
	{
		_speed = speed;
		_settings = SPISettings(_speed, MSBFIRST, ADC124S051_SPI_MODE);
	}

	uint32_t getSPIspeed() const { return _speed; }

	private:

	SPIClassRP2040 *_spi;
	SPISettings     _settings;
	uint8_t         _cs;
	uint32_t        _speed;
	uint8_t         _lastChannel;
	uint32_t        _count;

	static constexpr uint16_t chAddr(uint8_t ch)
	{
		return (uint16_t)ch << 11;  // 0x0000, 0x0800, 0x1000, 0x1800
	}

	uint16_t transfer16(uint16_t addr)
	{
		_spi->beginTransaction(_settings);
		digitalWrite(_cs, LOW);
		uint16_t data = _spi->transfer16(addr);
		digitalWrite(_cs, HIGH);
		_spi->endTransaction();
		return data & 0x0FFF;  // mask to 12 bits
	}
};

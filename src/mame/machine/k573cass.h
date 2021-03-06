// license:MAME
// copyright-holders:smf
/*
 * Konami 573 Security Cassette
 *
 */

#pragma once

#ifndef __K573CASS_H__
#define __K573CASS_H__

#include "cpu/psx/siodev.h"
#include "machine/adc083x.h"
#include "machine/ds2401.h"
#include "machine/x76f041.h"
#include "machine/x76f100.h"
#include "machine/zs01.h"

class konami573_cassette_interface
{
public:
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d0);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d1) = 0;
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d2) = 0;
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d3) = 0;
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d4);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d5);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d6);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d7);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_zs01_sda);
	virtual DECLARE_READ_LINE_MEMBER(read_line_ds2401);
	virtual DECLARE_READ_LINE_MEMBER(read_line_secflash_sda) = 0;
	virtual DECLARE_READ_LINE_MEMBER(read_line_dsr);
	virtual DECLARE_READ_LINE_MEMBER(read_line_adc083x_do);
	virtual DECLARE_READ_LINE_MEMBER(read_line_adc083x_sars);

	virtual ~konami573_cassette_interface() {}
};


extern const device_type KONAMI573_CASSETTE_X;

class konami573_cassette_x_device: public device_t,
	public konami573_cassette_interface,
	public device_slot_card_interface
{
public:
	konami573_cassette_x_device(const machine_config &mconfig, const char *tag, device_t *owner, UINT32 clock);
	konami573_cassette_x_device(const machine_config &mconfig, device_type type, const char *name, const char *tag, device_t *owner, UINT32 clock,const char *shortname, const char *source);

	virtual DECLARE_READ_LINE_MEMBER(read_line_secflash_sda);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d0);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d1);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d2);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d3);

protected:
	virtual void device_start();
	virtual machine_config_constructor device_mconfig_additions() const;

private:
	required_device<x76f041_device> m_x76f041;
};


extern const device_type KONAMI573_CASSETTE_XI;

class konami573_cassette_xi_device: public konami573_cassette_x_device
{
public:
	konami573_cassette_xi_device(const machine_config &mconfig, const char *tag, device_t *owner, UINT32 clock);

	virtual DECLARE_READ_LINE_MEMBER(read_line_ds2401);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d4);

	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d0);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d1);
	virtual DECLARE_READ_LINE_MEMBER(read_line_adc083x_do);
	virtual DECLARE_READ_LINE_MEMBER(read_line_adc083x_sars);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d5);

protected:
	virtual machine_config_constructor device_mconfig_additions() const;

private:
	required_device<ds2401_device> m_ds2401;
	required_device<adc0838_device> m_adc0838;
};


extern const device_type KONAMI573_CASSETTE_Y;


#define MCFG_KONAMI573_CASSETTE_Y_D0_HANDLER(_devcb) \
	devcb = &konami573_cassette_y_device::set_d0_handler(*device, DEVCB2_##_devcb);

#define MCFG_KONAMI573_CASSETTE_Y_D1_HANDLER(_devcb) \
	devcb = &konami573_cassette_y_device::set_d1_handler(*device, DEVCB2_##_devcb);

#define MCFG_KONAMI573_CASSETTE_Y_D2_HANDLER(_devcb) \
	devcb = &konami573_cassette_y_device::set_d2_handler(*device, DEVCB2_##_devcb);

#define MCFG_KONAMI573_CASSETTE_Y_D3_HANDLER(_devcb) \
	devcb = &konami573_cassette_y_device::set_d3_handler(*device, DEVCB2_##_devcb);

#define MCFG_KONAMI573_CASSETTE_Y_D4_HANDLER(_devcb) \
	devcb = &konami573_cassette_y_device::set_d4_handler(*device, DEVCB2_##_devcb);

#define MCFG_KONAMI573_CASSETTE_Y_D5_HANDLER(_devcb) \
	devcb = &konami573_cassette_y_device::set_d5_handler(*device, DEVCB2_##_devcb);

#define MCFG_KONAMI573_CASSETTE_Y_D6_HANDLER(_devcb) \
	devcb = &konami573_cassette_y_device::set_d6_handler(*device, DEVCB2_##_devcb);

#define MCFG_KONAMI573_CASSETTE_Y_D7_HANDLER(_devcb) \
	devcb = &konami573_cassette_y_device::set_d7_handler(*device, DEVCB2_##_devcb);

class konami573_cassette_y_device: public device_t,
	public konami573_cassette_interface,
	public device_slot_card_interface
{
public:
	konami573_cassette_y_device(const machine_config &mconfig, const char *tag, device_t *owner, UINT32 clock);
	konami573_cassette_y_device(const machine_config &mconfig, device_type type, const char *name, const char *tag, device_t *owner, UINT32 clock,const char *shortname, const char *source);

	// static configuration helpers
	template<class _Object> static devcb2_base &set_d0_handler(device_t &device, _Object object) { return downcast<konami573_cassette_y_device &>(device).m_d0_handler.set_callback(object); }
	template<class _Object> static devcb2_base &set_d1_handler(device_t &device, _Object object) { return downcast<konami573_cassette_y_device &>(device).m_d1_handler.set_callback(object); }
	template<class _Object> static devcb2_base &set_d2_handler(device_t &device, _Object object) { return downcast<konami573_cassette_y_device &>(device).m_d2_handler.set_callback(object); }
	template<class _Object> static devcb2_base &set_d3_handler(device_t &device, _Object object) { return downcast<konami573_cassette_y_device &>(device).m_d3_handler.set_callback(object); }
	template<class _Object> static devcb2_base &set_d4_handler(device_t &device, _Object object) { return downcast<konami573_cassette_y_device &>(device).m_d4_handler.set_callback(object); }
	template<class _Object> static devcb2_base &set_d5_handler(device_t &device, _Object object) { return downcast<konami573_cassette_y_device &>(device).m_d5_handler.set_callback(object); }
	template<class _Object> static devcb2_base &set_d6_handler(device_t &device, _Object object) { return downcast<konami573_cassette_y_device &>(device).m_d6_handler.set_callback(object); }
	template<class _Object> static devcb2_base &set_d7_handler(device_t &device, _Object object) { return downcast<konami573_cassette_y_device &>(device).m_d7_handler.set_callback(object); }

	virtual DECLARE_READ_LINE_MEMBER(read_line_secflash_sda);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d0);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d1);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d2);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d3);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d4);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d5);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d6);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d7);

protected:
	virtual void device_start();
	virtual machine_config_constructor device_mconfig_additions() const;

private:
	required_device<x76f100_device> m_x76f100;
	devcb2_write_line m_d0_handler;
	devcb2_write_line m_d1_handler;
	devcb2_write_line m_d2_handler;
	devcb2_write_line m_d3_handler;
	devcb2_write_line m_d4_handler;
	devcb2_write_line m_d5_handler;
	devcb2_write_line m_d6_handler;
	devcb2_write_line m_d7_handler;
};


extern const device_type KONAMI573_CASSETTE_YI;

class konami573_cassette_yi_device: public konami573_cassette_y_device
{
public:
	konami573_cassette_yi_device(const machine_config &mconfig, const char *tag, device_t *owner, UINT32 clock);

	virtual DECLARE_READ_LINE_MEMBER(read_line_ds2401);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d4);

protected:
	virtual machine_config_constructor device_mconfig_additions() const;

private:
	required_device<ds2401_device> m_ds2401;
};


extern const device_type KONAMI573_CASSETTE_ZI;

class konami573_cassette_zi_device: public device_t,
	public konami573_cassette_interface,
	public device_slot_card_interface
{
public:
	konami573_cassette_zi_device(const machine_config &mconfig, const char *tag, device_t *owner, UINT32 clock);

	virtual DECLARE_READ_LINE_MEMBER(read_line_ds2401);
	virtual DECLARE_READ_LINE_MEMBER(read_line_secflash_sda);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d4);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d1);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d2);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_d3);
	virtual DECLARE_WRITE_LINE_MEMBER(write_line_zs01_sda);

protected:
	virtual void device_start();
	virtual machine_config_constructor device_mconfig_additions() const;

private:
	required_device<zs01_device> m_zs01;
	required_device<ds2401_device> m_ds2401;
};


extern const device_type KONAMI573_CASSETTE_SLOT_SERIAL;

class konami573_cassette_slot_serial_device : public psxsiodev_device
{
public:
	konami573_cassette_slot_serial_device(const machine_config &mconfig, const char *tag, device_t *owner, UINT32 clock);

	void _data_out( int data, int mask );

protected:
	virtual void device_start();

private:
	virtual void data_in( int data, int mask );
};


extern const device_type KONAMI573_CASSETTE_SLOT;

class konami573_cassette_slot_device : public device_t,
	public device_slot_interface
{
public:
	konami573_cassette_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, UINT32 clock);

	DECLARE_WRITE_LINE_MEMBER(write_line_d0);
	DECLARE_WRITE_LINE_MEMBER(write_line_d1);
	DECLARE_WRITE_LINE_MEMBER(write_line_d2);
	DECLARE_WRITE_LINE_MEMBER(write_line_d3);
	DECLARE_WRITE_LINE_MEMBER(write_line_d4);
	DECLARE_WRITE_LINE_MEMBER(write_line_d5);
	DECLARE_WRITE_LINE_MEMBER(write_line_d6);
	DECLARE_WRITE_LINE_MEMBER(write_line_d7);
	DECLARE_WRITE_LINE_MEMBER(write_line_zs01_sda);
	DECLARE_READ_LINE_MEMBER(read_line_ds2401);
	DECLARE_READ_LINE_MEMBER(read_line_secflash_sda);
	DECLARE_READ_LINE_MEMBER(read_line_adc083x_do);
	DECLARE_READ_LINE_MEMBER(read_line_adc083x_sars);

protected:
	virtual void device_start();

private:
	required_device<konami573_cassette_slot_serial_device> m_serial;
	konami573_cassette_interface *m_cassette;
};


#endif

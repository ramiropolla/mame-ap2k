/***********************************************************************************************************

 Saturn Battery RAM cart emulation

 ***********************************************************************************************************/


#include "emu.h"
#include "machine/sat_bram.h"


//-------------------------------------------------
//  constructor
//-------------------------------------------------

const device_type SATURN_BRAM_4MB = &device_creator<saturn_bram4mb_device>;
const device_type SATURN_BRAM_8MB = &device_creator<saturn_bram8mb_device>;
const device_type SATURN_BRAM_16MB = &device_creator<saturn_bram16mb_device>;
const device_type SATURN_BRAM_32MB = &device_creator<saturn_bram32mb_device>;


saturn_bram_device::saturn_bram_device(const machine_config &mconfig, device_type type, const char *name, const char *tag, device_t *owner, UINT32 clock, UINT32 size)
					: device_t(mconfig, type, name, tag, owner, clock),
						device_sat_cart_interface( mconfig, *this ),
						device_nvram_interface(mconfig, *this),
						m_size(size)
{
}

saturn_bram4mb_device::saturn_bram4mb_device(const machine_config &mconfig, const char *tag, device_t *owner, UINT32 clock)
					: saturn_bram_device(mconfig, SATURN_BRAM_4MB, "Saturn Battery RAM 4Mbit Cart", tag, owner, clock, 0x80000)
{
	m_cart_type = 0x21;
}

saturn_bram8mb_device::saturn_bram8mb_device(const machine_config &mconfig, const char *tag, device_t *owner, UINT32 clock)
					: saturn_bram_device(mconfig, SATURN_BRAM_8MB, "Saturn Battery RAM 8Mbit Cart", tag, owner, clock, 0x100000)
{
	m_cart_type = 0x22;
}

saturn_bram16mb_device::saturn_bram16mb_device(const machine_config &mconfig, const char *tag, device_t *owner, UINT32 clock)
					: saturn_bram_device(mconfig, SATURN_BRAM_16MB, "Saturn Battery RAM 16Mbit Cart", tag, owner, clock, 0x200000)
{
	m_cart_type = 0x23;
}

saturn_bram32mb_device::saturn_bram32mb_device(const machine_config &mconfig, const char *tag, device_t *owner, UINT32 clock)
					: saturn_bram_device(mconfig, SATURN_BRAM_32MB, "Saturn Battery RAM 32Mbit Cart", tag, owner, clock, 0x400000)
{
	m_cart_type = 0x24;
}


//-------------------------------------------------
//  start/reset
//-------------------------------------------------

void saturn_bram_device::device_start()
{
	// TODO: only allocate the real amount of RAM
	m_ext_bram = auto_alloc_array_clear(machine(), UINT8, 0x400000);
	m_ext_bram_size = 0x400000;
	save_pointer(NAME(m_ext_bram), 0x400000);
}

void saturn_bram_device::device_reset()
{
}


/*-------------------------------------------------
 IO handlers
 -------------------------------------------------*/

// Battery RAM: single chip

READ32_MEMBER(saturn_bram_device::read_ext_bram)
{
	if (offset < m_size/2)
		return (m_ext_bram[offset * 2] << 16) | m_ext_bram[offset * 2 + 1];
	else
	{
		mame_printf_error("Battery RAM read beyond its boundary! offs: %X\n", offset);
		return 0xffffffff;
	}
}

WRITE32_MEMBER(saturn_bram_device::write_ext_bram)
{
	if (offset < m_size/2)
	{
		if (ACCESSING_BITS_16_23)
			m_ext_bram[offset * 2 + 0] = (data & 0x00ff0000) >> 16;
		if (ACCESSING_BITS_0_7)
			m_ext_bram[offset * 2 + 1] = (data & 0x000000ff) >> 0;
	}
	else
		mame_printf_error("Battery RAM write beyond its boundary! offs: %X data: %X\n", offset, data);
}

#if 0
READ8_MEMBER(saturn_bram_device::read_ext_bram)
{
	if (offset < m_size)
		return m_ext_bram[offset];
	else
	{
		mame_printf_error("Battery RAM read beyond its boundary! offs: %X\n", offset);
		return 0xff;
	}
}

WRITE8_MEMBER(saturn_bram_device::write_ext_bram)
{
	if (offset < m_size)
	{
		m_ext_bram[offset] = data;
	}
	else
		mame_printf_error("Battery RAM write beyond its boundary! offs: %X data: %X\n", offset, data);
}
#endif

/*
 *   Data East Pinball Dot Matrix Display
 *
 *    Type 1: 128x16
 *    Z80 @ 4MHz
 */

#include "decodmd1.h"
#include "rendlay.h"

const device_type DECODMD1 = &device_creator<decodmd_type1_device>;

READ8_MEMBER( decodmd_type1_device::latch_r )
{
	return 0;
}

WRITE8_MEMBER( decodmd_type1_device::data_w )
{
	m_latch = data;
}

READ8_MEMBER( decodmd_type1_device::busy_r )
{
	return m_status;
}


WRITE8_MEMBER( decodmd_type1_device::ctrl_w )
{
	if(!(m_ctrl & 0x01) && (data & 0x01))
	{
		m_command = m_latch;
		m_busy = 1;
		m_cpu->set_input_line(INPUT_LINE_IRQ0,ASSERT_LINE);
	}
	if((m_ctrl & 0x02) && !(data & 0x02))
	{
		m_rombank1->set_entry(0);
		m_bank = 0;
		m_busy = 0;
		m_rowselect = 0;
		m_blank = 0;
		m_frameswap = false;
		m_cpu->set_input_line(INPUT_LINE_RESET,PULSE_LINE);
	}
	m_ctrl = data;
}

READ8_MEMBER( decodmd_type1_device::ctrl_r )
{
	return m_ctrl;
}

READ8_MEMBER( decodmd_type1_device::status_r )
{
	return (m_busy & 0x01) | ((m_ctrl) << 1);
}

WRITE8_MEMBER( decodmd_type1_device::status_w )
{
	m_status = data;
}

// Z80 I/O ports not fully decoded.
// if bit 7 = 0, then when bit 2 is 0 selects COCLK, and when bit 2 is 1 selects CLATCH
READ8_MEMBER( decodmd_type1_device::dmd_port_r )
{
	if((offset & 0x84) == 0x80)
	{
		// IDAT (read only)
		m_busy = 0;
		m_ctrl &= ~0x01;
		m_cpu->set_input_line(INPUT_LINE_IRQ0,CLEAR_LINE);
		return m_latch;
	}
	return 0xff;
}

WRITE8_MEMBER( decodmd_type1_device::dmd_port_w )
{
	UINT8 bit;

	switch(offset & 0x84)
	{
	case 0x00:  // COCLK
		bit = (data >> ((offset & 0x03)*2));  // selects pair of bits depending on port used (0x00-0x03)
		m_pxdata1 = (m_pxdata1 >> 1) | ((bit & 0x01) ? 0x80000000 : 0x00000000);
		m_pxdata2 = (m_pxdata2 >> 1) | ((bit & 0x02) ? 0x80000000 : 0x00000000);
		break;
	case 0x04:  // CLATCH
		m_pxdata1_latched = m_pxdata1;
		m_pxdata2_latched = m_pxdata2;
		if(m_blank)
			output_data();
		break;
	case 0x80:  // IDAT (ignored)
		break;
	case 0x84:
		bit = data & 0x01;
		switch(offset & 0xdc)
		{
		case 0x84:  // Bank bit 0
			m_bank = (m_bank & ~0x01) | (~bit & 0x01);
			m_rombank1->set_entry(m_bank);
			break;
		case 0x8c:  // Bank bit 1
			m_bank = (m_bank & ~0x02) | ((~bit & 0x01) << 1);
			m_rombank1->set_entry(m_bank);
			break;
		case 0x94:  // Bank bit 2
			m_bank = (m_bank & ~0x04) | ((~bit & 0x01) << 2);
			m_rombank1->set_entry(m_bank);
			break;
		case 0x9c:  // Blanking
			m_blank = bit;
			if(bit)
				output_data();
			break;
		case 0xc4:  // Status
			m_status = bit;
			break;
		case 0xcc:  // Row data
			m_rowdata = bit;
			break;
		case 0xd4:  // Row clock
			if(~bit & m_rowclock)  // on negative edge
				m_rowselect = (m_rowselect << 1) | m_rowdata;
			m_rowclock = bit;
			break;
		case 0xdc:  // Test
			m_busy_set = bit;
			//check_busy();
			break;
		}
		break;
	}
}

void decodmd_type1_device::output_data()
{
	UINT8 ptr = 0;
	UINT32 row = m_rowselect;

	if(row == 0)
		m_frameswap = !m_frameswap;

	if(!m_frameswap)
		ptr = 0x80;

	while(row != 0)
	{
		if(row & 0x01)
		{
			m_pixels[ptr] = m_pxdata2_latched;
			m_pixels[ptr+1] = m_pxdata1_latched;
		}
		ptr += 2;
		row >>= 1;
	}
}

void decodmd_type1_device::check_busy()
{
	if(m_busy_clr)
	{
		m_busy = 0;
		m_cpu->set_input_line(INPUT_LINE_IRQ0,CLEAR_LINE);
	}
	else if(!m_busy_set)
	{
		m_busy = 1;
		m_cpu->set_input_line(INPUT_LINE_IRQ0,ASSERT_LINE);
	}
	else
	{
		if(!m_busy_clk)
		{
			m_busy = 1;
			m_cpu->set_input_line(INPUT_LINE_IRQ0,ASSERT_LINE);
		}
	}

}

TIMER_DEVICE_CALLBACK_MEMBER(decodmd_type1_device::dmd_nmi)
{
	m_cpu->set_input_line(INPUT_LINE_NMI, PULSE_LINE);
}

static ADDRESS_MAP_START( decodmd1_map, AS_PROGRAM, 8, decodmd_type1_device )
	AM_RANGE(0x0000, 0x3fff) AM_ROMBANK("dmdbank2") // last 16k of ROM
	AM_RANGE(0x4000, 0x7fff) AM_ROMBANK("dmdbank1") //AM_WRITE(status_w)
	AM_RANGE(0x8000, 0x9fff) AM_RAMBANK("dmdram")
ADDRESS_MAP_END

static ADDRESS_MAP_START( decodmd1_io_map, AS_IO, 8, decodmd_type1_device )
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE(0x00, 0xff) AM_READWRITE(dmd_port_r, dmd_port_w)
ADDRESS_MAP_END

static MACHINE_CONFIG_FRAGMENT( decodmd1 )
	/* basic machine hardware */
	MCFG_CPU_ADD("dmdcpu", Z80, XTAL_4MHz)
	MCFG_CPU_PROGRAM_MAP(decodmd1_map)
	MCFG_CPU_IO_MAP(decodmd1_io_map)

	MCFG_QUANTUM_TIME(attotime::from_hz(60))

	MCFG_TIMER_DRIVER_ADD_PERIODIC("nmi_timer",decodmd_type1_device,dmd_nmi,attotime::from_hz(2000))  // seems a lot

	MCFG_DEFAULT_LAYOUT(layout_lcd)

	MCFG_SCREEN_ADD("dmd",LCD)
	MCFG_SCREEN_SIZE(128, 16)
	MCFG_SCREEN_VISIBLE_AREA(0, 128-1, 0, 16-1)
	MCFG_SCREEN_UPDATE_DRIVER(decodmd_type1_device,screen_update)
	MCFG_SCREEN_REFRESH_RATE(60)

	MCFG_RAM_ADD(RAM_TAG)
	MCFG_RAM_DEFAULT_SIZE("8K")

MACHINE_CONFIG_END

machine_config_constructor decodmd_type1_device::device_mconfig_additions() const
{
	return MACHINE_CONFIG_NAME( decodmd1 );
}

decodmd_type1_device::decodmd_type1_device(const machine_config &mconfig, const char *tag, device_t *owner, UINT32 clock)
	: device_t(mconfig, DECODMD1, "Data East Pinball Dot Matrix Display Type 1", tag, owner, clock, "decodmd1", __FILE__),
	  m_cpu(*this,"dmdcpu"),
	  m_rombank1(*this,"dmdbank1"),
	  m_rombank2(*this,"dmdbank2"),
	  m_ram(*this,RAM_TAG)
{}

void decodmd_type1_device::device_start()
{
	save_pointer(m_pixels,"DMD Video data",0x100);
}

void decodmd_type1_device::device_reset()
{
	UINT8* ROM;
	UINT8* RAM = m_ram->pointer();
	m_rom = memregion(m_romregion);

	memset(RAM,0,0x2000);

	ROM = m_rom->base();
	m_rombank1->configure_entries(0, 8, &ROM[0x0000], 0x4000);
	m_rombank2->configure_entry(0, &ROM[0x1c000]);
	m_rombank1->set_entry(0);
	m_rombank2->set_entry(0);
	m_bank = 0;
	m_busy = 0;
	m_rowselect = 0;
	m_blank = 0;
	m_frameswap = false;
}

void decodmd_type1_device::device_config_complete()
{
	// inherit a copy of the static data
	const decodmd_type1_intf *intf = reinterpret_cast<const decodmd_type1_intf *>(static_config());
	if (intf != NULL)
		*static_cast<decodmd_type1_intf *>(this) = *intf;

	// or initialize to defaults if none provided
	else
	{
		m_romregion = NULL;
	}
}

UINT32 decodmd_type1_device::screen_update( screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect )
{
	UINT8 ptr = 0;
	UINT8 x,y,dot;
	UINT32 data1,data2;

	if(m_frameswap)
		ptr = 0x80;

	for(y=0;y<16;y++)  // scanline
	{
		for(x=0;x<128;x+=64)
		{
			data1 = m_pixels[ptr];
			data2 = m_pixels[ptr+1];
			for(dot=0;dot<64;dot+=2)
			{
				bitmap.pix32(y,x+dot) = (data1 & 0x01) ? MAKE_RGB(0xff,0xaa,0x00) : RGB_BLACK;
				bitmap.pix32(y,x+dot+1) = (data2 & 0x01) ? MAKE_RGB(0xff,0xaa,0x00) : RGB_BLACK;
				data1 >>= 1;
				data2 >>= 1;
			}
			ptr+=2;
		}
	}

	return 0;
}
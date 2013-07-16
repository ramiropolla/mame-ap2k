/*
 * video/apollo.c
 *
 *  Created on: April 25, 2013
 *      Author: Hans Ostermeyer
 *
 *  Released for general non-commercial use under the MAME license
 *  Visit http://mamedev.org for licensing and usage restrictions.
 *
 *  see also:
 *  - Domain Series 3000/Series 4000 Hardware Architecture Handbook (Order No. 007861 Rev. 02)
 *  - http://www.bitsavers.org/pdf/apollo/002398-04_Domain_Engineering_Handbook_Rev4_Jan87.pdf (page 12-16 ...)
 *  - http://www.bitsavers.org/pdf/brooktree/Brooktree_1991.pdf (page 305 ...)
 *
 */

#define VERBOSE 0

#include "includes/apollo.h"
#include "drivlgcy.h"
#include "scrlegcy.h"

#include "apollo.lh"
#include "apollo_15i.lh"

/***************************************************************************
 TYPE DEFINITIONS
 ***************************************************************************/

// monochrome 1280x1024
#define SCREEN_DEVICE_ID_19I 9

// monochrome 1024x800
#define SCREEN_DEVICE_ID_15I 11

// 4 plane color 1024x800
#define SCREEN_DEVICE_ID_C4P 8

// 8 plane color 1024x800
#define SCREEN_DEVICE_ID_C8P 10

#define VIDEO_SCREEN_TAG "screen"

// status register
#define SR_BLANK        0x80
#define SR_V_BLANK      0x40
#define SR_H_SYNC       0x20
#define SR_DONE         0x20    // 4- and 8-plane color
#define SR_R_M_W        0x10
#define SR_ALT          0x08
#define SR_V_SYNC       0x04
#define SR_SYNC         0x04    // 4- and 8-plane color
#define SR_H_CK         0x02
#define SR_V_DATA       0x01
#define SR_V_FLAG       0x01    // 4-plane color
#define SR_LUT_OK       0x01    // 8-plane color
// control register 0
#define CR0_MODE(a)     ((a) >> 5)
#define CR0_MODE_0      0
#define CR0_MODE_1      1
#define CR0_MODE_VECTOR 2
#define CR0_MODE_3      3
#define CR0_MODE_BLT    4
#define CR0_MODE_NORMAL 7
#define CR0_SHIFT(a)    ((a) & 0x1f)

// control register 1
#define CR1_INV         0x80
#define CR1_AD_BIT      0x80    // 4- and 8-plane color
#define CR1_DADDR_16    0x40
#define CR1_DV_CK       0x40    // 4- and 8-plane color
#define CR1_DH_CK       0x20
#define CR1_ROP_EN      0x10
#define CR1_RESET       0x08
#define CR1_DP_CK       0x04
#define CR1_SYNC_EN     0x02
#define CR1_DISP_EN     0x01

// control register 2
#define CR2_S_DATA(a)   ((a) >> 6)
#define CR2_CONST_ACCESS 0x00
#define CR2_PIXEL_ACCESS 0x01
#define CR2_SHIFT_ACCESS 0x02
#define CR2_PLANE_ACCESS 0x03
#define CR2_S_PLANE(a)   (((a) >> 4) & 0x03)
#define CR2_D_PLANE(a)   ((a) & 0x0f)
#define CR2B_S_PLANE(a)  ((a) & 0x07)
#define CR2A_D_PLANE(a)  (a)

// Lookup table control register
#define LUT_AD_CS       0x80
#define LUT_CPAL_CS     0x40
#define LUT_FIFO_CS     0x20
#define LUT_FIFO_RST    0x10
#define LUT_ST_LUK      0x08
#define LUT_R_W         0x04
#define LUT_C1          0x02
#define LUT_C0          0x01
#define LUT_C1_C0(a)    ((a)& (LUT_C1|LUT_C0))

#define LUT_FIFO_SIZE   1024

//**************************************************************************
//  class apollo_graphics
//**************************************************************************

class apollo_graphics /*: public device_t*/
{
public:
	apollo_graphics()
	{
	}

	void device_start(running_machine &m_machine);
	void device_reset();
	void device_reset_mono19i();

	// monochrome control
	READ8_DEVICE_HANDLER( apollo_mcr_r );
	WRITE8_DEVICE_HANDLER( apollo_mcr_w );

	// monochrome and color memory
	READ16_DEVICE_HANDLER( apollo_mem_r );
	WRITE16_DEVICE_HANDLER( apollo_mem_w );

	// color control
	READ8_DEVICE_HANDLER( apollo_ccr_r );
	WRITE8_DEVICE_HANDLER( apollo_ccr_w );

	UINT32 screen_update(bitmap_rgb32 &bitmap, const rectangle &cliprect);
	void vblank_state_changed(device_t *device, screen_device &screen, bool vblank_state);

	int is_mono() { return m_n_planes == 1; }

private:
	class lut_fifo;
	class bt458;

	running_machine &machine() const
	{
		assert(m_machine != NULL);
		return *m_machine;
	}

	const char *cr_text(offs_t offset, UINT8 data, UINT8 rw);

	void increment_h_clock();
	void increment_v_clock();
	void increment_p_clock();

	void log_cr1(const char * text, device_t *device);
	void set_cr1(device_t *device, UINT8 data);
	void set_cr3a(device_t *device, UINT8 data);
	void set_cr3b(device_t *device, UINT8 data);
	void set_lut_cr(device_t *device, UINT8 data);

	UINT32 set_msb0(UINT32 value, UINT8 data)
	{
		return (value & 0xffffff00) | data;
	}
	UINT32 set_lsb0(UINT32 value, UINT8 data)
	{
		return (value & 0xffff00ff) | (data << 8);
	}
	UINT32 set_msb1(UINT32 value, UINT8 data)
	{
		return (value & 0xff00ffff) | (data << 16);
	}
	UINT32 set_lsb1(UINT32 value, UINT8 data)
	{
		return (value & 0x00ffffff) | (data << 24);
	}
	UINT8 get_msb1(UINT32 value)
	{
		return (value >> 16) & 0xff;
	}
	UINT8 get_lsb1(UINT32 value)
	{
		return (value >> 24) & 0xff;
	}

	void set_status_rmw();
	UINT16 rop(UINT16 dest_data, UINT16 src_data, UINT8 plane);
	void set_source_data(UINT32 offset);
	UINT32 get_source_data(UINT8 plane);
	void blt(UINT32 dest_addr, UINT16 mem_mask);

	UINT8 get_pixel(UINT32 offset, UINT16 mask);
	UINT8 c4p_read_adc(UINT8 data);
	UINT8 c8p_read_adc(UINT8 data);

	void screen_update1(bitmap_rgb32 &bitmap, const rectangle &cliprect);

	UINT16 m_n_planes;
	UINT16 m_width;
	UINT16 m_height;
	UINT16 m_buffer_width;
	UINT16 m_buffer_height;

	UINT8 m_sr;
	UINT8 m_device_id;
	UINT16 m_write_enable_register;
	UINT32 m_rop_register;
	UINT16 m_diag_mem_request;
	UINT8 m_cr0;
	UINT8 m_cr1;
	UINT8 m_cr2;
	UINT8 m_cr2b;
	UINT8 m_cr2_s_data;
	UINT8 m_cr2_s_plane;
	UINT8 m_cr2_d_plane;
	UINT8 m_cr3a;
	UINT8 m_cr3b;
	UINT8 m_ad_result;
	UINT8 m_ad_pending;

	UINT8 m_lut_control;
	UINT8 m_lut_data;

	UINT8 m_update_flag;
	UINT8 m_update_pending;

	UINT8 m_blt_cycle_count;
	UINT32 m_image_offset;
	UINT32 m_guard_latch[8];

	int m_h_clock;
	int m_v_clock;
	int m_p_clock;
	int m_data_clock;

	UINT16 *m_image_memory;
	int m_image_plane_size;
	int m_image_memory_size;

	UINT32 m_color_lookup_table[16];

	lut_fifo *m_lut_fifo;
	bt458 *m_bt458;

	running_machine *m_machine;
};

	//**************************************************************************
	// class LUT Fifo
	//**************************************************************************

class apollo_graphics::lut_fifo
{
public:
	lut_fifo()
	{
		reset();
	}

	void reset()
	{
		m_size = LUT_FIFO_SIZE;
		m_get_index = 0;
		m_put_index = 0;
	}

	void put(const UINT8 data)
	{
		if (!is_full())
		{
			m_data[m_put_index] = data;
			m_put_index = (m_put_index + 1) % m_size;
		}
	}

	UINT8 get()
	{
		UINT8 data = is_empty() ? 0xff : m_data[m_get_index];
		m_get_index = (m_get_index + 1) % m_size;
		return data;
	}

	int is_empty()
	{
		return m_get_index == m_put_index;
	}

	int is_full()
	{
		return ((m_put_index + 1) % m_size) == m_get_index;
	}

private:
	UINT16 m_size;
	UINT16 m_get_index;
	UINT16 m_put_index;
	UINT8 m_data[LUT_FIFO_SIZE];
};

//**************************************************************************
//  class Brooktree Bt458
//**************************************************************************

class apollo_graphics::bt458
{
public:
	bt458(running_machine &running_machine);
	void start();
	void reset();
	UINT8 read(UINT8 c10);
	void write(UINT8 data, UINT8 c10);
	UINT32 get_rgb(UINT8 index);

private:
	running_machine &machine() const
	{
		assert(m_machine != NULL);
		return *m_machine;
	}

	UINT8 m_color_counter;
	UINT8 m_red;
	UINT8 m_green;

	UINT8 m_address_register;
	UINT32 m_color_palette_RAM[256];
	UINT32 m_overlay_color[4];
	UINT8 m_read_mask_register;
	UINT8 m_blink_mask_register;
	UINT8 m_command_register;
	UINT8 m_control_test_register;

	running_machine *m_machine;
};

apollo_graphics::bt458::bt458(running_machine &running_machine)
{
	m_machine = &running_machine;
}

void apollo_graphics::bt458::start()
{
	MLOG1(("start apollo_graphics::bt458"));
}

void apollo_graphics::bt458::reset()
{
	MLOG1(("reset apollo_graphics::bt458"));

	m_color_counter = 0;
	m_red = 0;
	m_green = 0;

	m_address_register = 0;
	memset(m_color_palette_RAM, 0, sizeof(m_color_palette_RAM));
	memset(m_overlay_color, 0, sizeof(m_overlay_color));
	m_read_mask_register = 0;
	m_blink_mask_register = 0;
	m_command_register = 0;
	m_control_test_register = 0;
}

void apollo_graphics::bt458::write(UINT8 data, UINT8 c10)
{
	MLOG1(("writing Bt458 data=%02x C1,C0=%d", data, c10));
	switch (c10)
	{
	case 0: // address register
		m_address_register = data;
		m_color_counter = 0;
		MLOG1(("bt458::write 0: addr=%02x", data));
		break;
	case 1: // color palette RAM
		switch (m_color_counter)
		{
		case 0:
			m_red = data;
			m_color_counter++;
			break;
		case 1:
			m_green = data;
			m_color_counter++;
			break;
		case 2:
			m_color_palette_RAM[m_address_register] = (m_red << 16) | (m_green << 8) | data;
			m_address_register++;
			m_color_counter = 0;
			break;
		}
		break;
	case 2: // registers
		switch (m_address_register)
		{
		case 0x04:
			m_read_mask_register = data;
			MLOG1(("bt458::write: writing Bt458 m_read_mask_register=%02x", data))
			break;
		case 0x05:
			m_blink_mask_register = data;
			MLOG1(("bt458::write: writing Bt458 m_blink_mask_register=%02x", data))
			break;
		case 0x06:
			m_command_register = data;
			MLOG1(("bt458::write: writing Bt458 m_command_register=%02x", data))
			break;
		case 0x07:
			m_control_test_register = data;
			break;
		default:
			MLOG1(("bt458::write: writing unexpected Bt458 data=%02x C1,C0=%d at %02x", data, c10, m_address_register))
			break;
		}
		break;
	case 3: // overlay color
		switch (m_address_register)
		{
		case 0x00:
		case 0x01:
		case 0x02:
		case 0x03:
			m_overlay_color[m_address_register] = data;
			MLOG1(("bt458::write: writing Bt458 m_overlay_color[%d]=%02x",m_address_register, data));
			break;
		default:
			MLOG1(("bt458::write: writing unexpected Bt458 data=%02x C1,C0=%d at %02x", data, c10, m_address_register));
			break;
		}
		break;
	default:
		MLOG1(("bt458::write: writing unexpected Bt458 data=%02x C1,C0=%d", data, c10))
		;
		break;
	}
}

UINT8 apollo_graphics::bt458::read(UINT8 c10)
{
	UINT8 data = 0xff;

	switch (c10)
	{
	case 0: // address register
		data = m_address_register;
		break;
	case 1: // color palette RAM
		switch (m_color_counter)
		{
		case 0: // red
			data =  (m_color_palette_RAM[m_address_register] >> 16) & 0xff;
			m_color_counter++;
			break;
		case 1: // Green
			data =  (m_color_palette_RAM[m_address_register] >> 8) & 0xff;
			m_color_counter++;
			break;
		case 2: // blue
			data =  m_color_palette_RAM[m_address_register] & 0xff;
			m_address_register++;
			m_color_counter = 0;
			break;
		}
		break;
	case 2: // registers
		switch (m_address_register)
		{
		case 0x04:
			data = m_read_mask_register;
			MLOG1(("bt458::read: reading Bt458 m_read_mask_register=%02x", data))
			break;
		case 0x05:
			data = m_blink_mask_register;
			MLOG1(("bt458::read: reading Bt458 m_blink_mask_register=%02x", data))
			break;
		case 0x06:
			data = m_command_register;
			MLOG1(("bt458::read: reading Bt458 m_command_register=%02x", data))
			break;
		case 0x07:
			{
				UINT32 rgb = m_color_palette_RAM[0];
				switch (m_control_test_register & 0x0f)
				{
				case 0x01: data = 0x01 | ((rgb >> 16) & 0xf0); break;
				case 0x09: data = 0x09 | ((rgb >> 12) & 0xf0); break;
				case 0x02: data = 0x02 | ((rgb >> 8) & 0xf0); break;
				case 0x0a: data = 0x0a | ((rgb >> 4) & 0xf0); break;
				case 0x04: data = 0x04 | ((rgb >> 0) & 0xf0); break;
				case 0x0c: data = 0x0c | ((rgb << 4) & 0xf0); break;
				default: data = 0xff; break;
				}
			}
			break;
		default:
			MLOG1(("bt458::read: reading unexpected Bt458 data=%02x C1,C0=%d at %02x", data, c10, m_address_register))
			break;
		}
		break;
	default:
		MLOG1(("bt458::read: reading unexpected Bt458 data=%02x C1,C0=%d at %02x", data, c10, m_address_register))
		break;
	}

//  MLOG1(("reading Bt458 data=%02x cs=%d", m_data, c10));
	return data;
}

UINT32 apollo_graphics::bt458::get_rgb(UINT8 index)
{
	return m_color_palette_RAM[index];
}

/*****************************************************************************
 INLINE FUNCTIONS
 *****************************************************************************/

INLINE apollo_graphics *get_safe_token(device_t *device)
{
	assert(device != NULL);
	assert(device->type() == APOLLO_GRAPHICS || device->type() == APOLLO_MONO19I );
	return (apollo_graphics *) downcast<apollo_graphics_15i *> (device)->token();
}

void apollo_graphics::device_start(running_machine &running_machine)
{
	m_n_planes = 0;
	m_width = 0;
	m_height = 0;
	m_buffer_width = 0;
	m_buffer_height = 0;

	m_sr = 0;
	m_device_id = 0;
	m_write_enable_register = 0;
	m_rop_register = 0;
	m_diag_mem_request = 0;
	m_cr0 = 0;
	m_cr1 = 0;
	m_cr2 = 0;
	m_cr2b = 0;
	m_cr2_s_data = 0;
	m_cr2_s_plane = 0x00;
	m_cr2_d_plane = 0x0e;
	m_cr3a = 0;
	m_cr3b = 0;
	m_ad_result = 0;
	m_ad_pending = 0;

	m_lut_control = 0;
	m_lut_data = 0;

	m_update_flag = 0;
	m_update_pending = 0;

	m_blt_cycle_count = 0;
	m_image_offset = 0;
	memset(m_guard_latch, 0, sizeof(m_guard_latch));

	m_h_clock = 0;
	m_v_clock = 0;
	m_p_clock = 0;
	m_data_clock = 0;

	m_image_memory = 0;
	m_image_plane_size = 0;
	m_image_memory_size = 0;

	memset(m_color_lookup_table, 0, sizeof(m_color_lookup_table));

	m_lut_fifo = NULL;
	m_bt458 = NULL;

	m_machine = &running_machine;
}

void apollo_graphics::device_reset()
{
	if (m_n_planes == 0)
	{
		if (apollo_config(APOLLO_CONF_MONO_19I))
		{
			// monochrome 1280x1024
			m_n_planes = 1;
			m_device_id = SCREEN_DEVICE_ID_19I;
			m_width = 1280;
			m_height = 1024;
			m_buffer_width = 2048;
			m_buffer_height = 1024;
		}
		else if (apollo_config(APOLLO_CONF_MONO_15I))
		{
			// monochrome 1024x800
			m_n_planes = 1;
			m_device_id = SCREEN_DEVICE_ID_15I;
			m_width = 1024;
			m_height = 800;
			m_buffer_width = 1024;
			m_buffer_height = 1024;
		}
		else if (apollo_config(APOLLO_CONF_4_PLANES))
		{
			// 4-planes color 1024x800
			m_n_planes = 4;
			m_device_id = SCREEN_DEVICE_ID_C4P;
			m_width = 1024;
			m_height = 800;
			m_buffer_width = 1024;
			m_buffer_height = 1024;
		}
		else
		{
			// 8-planes color 1024x800
			m_n_planes = 8;
			m_device_id = SCREEN_DEVICE_ID_C8P;
			m_width = 1024;
			m_height = 800;
			m_buffer_width = 1024;
			m_buffer_height = 1024;

			m_lut_fifo = new lut_fifo();

			m_bt458 = new bt458(*m_machine);
			m_bt458->start();
			m_bt458->reset();
		}
	}

	if (m_image_memory == NULL)
	{
		/* allocate the memory image */
		m_image_plane_size = m_buffer_height * m_buffer_width / 16;
		m_image_memory_size = m_image_plane_size * m_n_planes;
		m_image_memory
				= auto_alloc_array(machine(), UINT16, m_image_memory_size);
		assert(m_image_memory != NULL);

		MLOG1(("device reset apollo graphics: buffer=%p size=%0x", m_image_memory, m_image_memory_size));
	}

	memset(m_color_lookup_table, 0, sizeof(m_color_lookup_table));
	memset(m_image_memory, 0, m_image_memory_size * 2);

	//  register_vblank_callback(this);
}

void apollo_graphics::device_reset_mono19i()
{
	if (m_n_planes == 0)
	{
		// monochrome 1280x1024
		m_n_planes = 1;
		m_device_id = SCREEN_DEVICE_ID_19I;
		m_width = 1280;
		m_height = 1024;
		m_buffer_width = 2048;
		m_buffer_height = 1024;
	}

	device_reset();
}

/***************************************************************************
 Monochrome Controller Registers at 0x5d800 - 0x5dc07
 ***************************************************************************/

const char *apollo_graphics::cr_text(offs_t offset, UINT8 data, UINT8 rw)
{
	static const char *cr0[8] =
	{ "cr0 mode=0 CPU dest BLT", "cr0 mode=1 Alternating BLT",
			"cr0 mode=2 Vector mode", "cr0 mode=3 CPU Source BLT",
			"cr0 mode=4 Double access BLT ", "cr0 mode=5 ???",
			"cr0 mode=6 ???", "cr0 mode=7 Normal" };

	static const char *cr2[4] =
			{ "cr2 Constant access", "cr2 Pixel access", "cr2 ???",
					"cr2 Plane access" };

	static const char *cr2b[4] =
	{ "cr2b Constant access", "cr2b Pixel access", "cr2b ???",
			"cr2b Plane access" };

	switch (offset & 0x407)
	{
	case 0:
		return rw ? "sr" : "we";
	case 1:
		return rw ? "id" : "we";
	case 2:
	case 3:
		return "rop0";
	case 4:
	case 5:
		return m_n_planes == 8 ? "rop1" : "mem refresh";
	case 6:
	case 7:
		return m_n_planes == 8 ? "mem refresh" : "???";
	case 0x400:
		return cr0[data >> 5];
	case 0x401:
		return m_n_planes == 8 ? "LUT data" : "red";
	case 0x402:
		return "cr1";
	case 0x403:
		return m_n_planes == 8 ? "LUT ctrl" : "green";
	case 0x404:
		return m_n_planes == 8 ? "cr2a" : cr2[data >> 6];
	case 0x405:
		return m_n_planes == 8 ? cr2b[data >> 6] : "blue";
	case 0x406:
		return "cr3";
	case 0x407:
		return m_n_planes == 8 ? "cr3b" : "a/d";
	default:
		return "???";
	}
}

void apollo_graphics::log_cr1(const char * text, device_t *device)
{
	DLOG2(("%s: cr0=%02x cr1=%02x sr=%02x pixel_clock=%3d/%3d bl=%d vb=%d vs=%d hs=%d hc=%d vck=%d hck=%d pck=%d vd=%d",
					text,
					m_cr0,
					m_cr1,
					m_sr,
					m_p_clock,
					m_data_clock,
					m_sr & SR_BLANK ? 1 : 0,
					m_sr & SR_V_BLANK ? 1 : 0,
					m_sr & SR_V_SYNC ? 1 : 0,
					m_sr & SR_H_SYNC ? 1 : 0,
					m_sr & SR_H_CK ? 1 : 0,
					m_cr1 & CR1_DV_CK ? 1 : 0,
					m_cr1 & CR1_DH_CK ? 1 : 0,
					m_cr1 & CR1_DP_CK ? 1 : 0,
					m_sr & SR_V_DATA ? 1 : 0));
}

void apollo_graphics::increment_h_clock()
{
	MLOG1(("increment_h_clock: sr=%02x m_h_clock=%d", m_sr, m_h_clock));

	if (m_device_id == SCREEN_DEVICE_ID_19I)
	{
		// DISP7A.DEX Test 5
		// Note: 108 = 80 + 28 = 1280/16 + 448/16
		switch (m_h_clock %= 108)
		{
		case 8: // Not blanking
			m_sr |= SR_BLANK;
			break;
		case 88: // blanking
			m_sr &= ~SR_BLANK;
			break;
		case 93: // HSync active
			m_sr &= ~SR_H_SYNC;
			// DISP7A.DEX.1 Test 6
			increment_v_clock();
			break;
		case 104: // HSync inactive
			m_sr |= SR_H_SYNC;
			break;
		}
	}
	else if (m_n_planes == 1)
	{
		switch (m_h_clock %= 84)
		{
		case 1: // HSync inactive
			m_sr |= SR_H_SYNC;
			break;
		case 8: // Not blanking
			m_sr |= SR_BLANK;
			break;
		case 72: // blanking
			m_sr &= ~SR_BLANK;
			break;
		case 77: // HSync active
			m_sr &= ~SR_H_SYNC;
			// DISP7D.DEX.1 Test 6
			increment_v_clock();
			break;
		}
	}
	else if (m_n_planes == 4)
	{
		switch (m_h_clock %= 84)
		{
		case 8: // Not blanking
			m_sr |= SR_BLANK;
			if (m_sr & SR_V_BLANK)
			{
				m_sr |= SR_V_FLAG;
			}
			break;
		case 73: // blanking
			m_sr &= ~SR_BLANK;
			if (m_sr & SR_V_BLANK)
			{
				m_sr &= ~SR_V_FLAG;
			}
			break;
		}
	}
	else // m_n_planes == 8
	{
		switch (m_h_clock %= 84)
		{
		case 9: // Not blanking
			m_sr |= SR_BLANK;
			break;
		case 73: // blanking
			m_sr &= ~SR_BLANK;
			break;
		}
	}
	m_h_clock++;
}

void apollo_graphics::increment_v_clock()
{
	MLOG1(("increment_v_clock: sr=%02x m_v_clock=%d", m_sr, m_v_clock));

	if (m_device_id == SCREEN_DEVICE_ID_19I)
	{
		switch (m_v_clock %= 1066)
		{
		case 1023: // blanking
			m_sr &= ~(SR_V_BLANK | SR_BLANK);
			break;
		case 1028: // VSync active
			m_sr &= ~SR_V_SYNC;
			break;
		case 1032: // VSync inactive
			m_sr |= SR_V_SYNC;
			break;
		case 1065: // not blanking
			m_sr |= (SR_V_BLANK | SR_BLANK);
			break;
		}
	}
	else if (m_n_planes == 1)
	{
		switch (m_v_clock %= 842)
		{
		case 799:
			m_sr &= ~SR_V_BLANK;
			break;
		case 804:
			m_sr &= ~SR_SYNC;
			break;
		case 808:
			m_sr |= SR_SYNC;
			break;
		case 841:
			m_sr |= SR_V_BLANK;
			break;
		}
	}
	else if (m_n_planes == 4)
	{
		// DISP7B.DEX Test 6 and Test 20
		switch (m_v_clock %= 842)
		{
		case 799:
			m_sr &= ~SR_V_BLANK;
			m_sr &= ~SR_V_FLAG;
			break;
		case 803:
			m_sr &= ~SR_SYNC;
			break;
		case 807:
			m_sr |= SR_SYNC;
			break;
		case 836:
			m_sr |= SR_V_FLAG;
			break;
		case 841:
			m_sr |= SR_V_BLANK;
			break;
		}
	}
	else if (m_n_planes == 8)
	{
		// DISP7C.DEX Test 50
		switch (m_v_clock %= 842)
		{
		case 800:
			m_sr &= ~SR_V_BLANK;
			break;
		case 804:
			m_sr &= ~SR_SYNC;
			break;
		case 808:
			m_sr |= SR_SYNC;
			break;
		case 0:
			m_sr |= SR_V_BLANK;
			break;
		}
	}
	m_v_clock++;
	m_p_clock = 0;
	m_data_clock = 0;
}

void apollo_graphics::increment_p_clock()
{
	if (m_n_planes == 1)
	{
		if ((m_cr1 & CR1_DISP_EN) == 0)
		{
			m_sr &= ~SR_V_DATA;
		}
		else
		{
			int pixel_offset = (m_device_id == SCREEN_DEVICE_ID_19I) ? 10 : 8;

			if (m_p_clock > pixel_offset)
			{
				// FIXME: ok for DEX Test 5 6 17 19 - nok for 20
				int pixel_addr = m_v_clock * m_width + m_data_clock;

				// FIXME: ok for DEX Test 5 6 17 20 - nok for 19
				// Note: for dn3500_19i DEX Test 17 18 20 will fail !!!!
				// Note: must NOT reset m_data_clock in increment_p_clock !
//              int pixel_addr = m_data_clock * 32;

				UINT16 pixel = m_image_memory[pixel_addr / 16] & (0x8000 >> (pixel_addr % 16));

				pixel = (pixel ? 1 : 0) ^ ((m_cr1 & CR1_INV) ? 0 : 1);

				m_sr = pixel ? (m_sr | SR_V_DATA) : (m_sr & ~SR_V_DATA);

				m_data_clock++;
			}

			m_p_clock++;

			// DEX Test 4: Pixel Counter Test
			if ((m_p_clock % 8) == 0)
			{
				m_sr ^= SR_H_CK;
			}
		}
	}
	else if (m_n_planes == 4)
	{
		if ((m_p_clock % 8) == 0 && m_p_clock > 0)
		{
			m_sr ^= SR_H_CK;
		}
		m_p_clock++;
	}
	else // m_n_planes == 8
	{
		if ((m_p_clock % 8) == 1 && m_p_clock > 1)
		{
			m_sr ^= SR_H_CK;
		}
		m_p_clock++;
	}
}

void apollo_graphics::set_cr1(device_t *device, UINT8 data)
{
	UINT8 diffs = m_cr1 ^ data;
	m_cr1 = data;

	UINT8 dp_clock = (diffs & CR1_DP_CK) && (m_cr1 & CR1_DP_CK) == 0;
	UINT8 dh_clock = (diffs & CR1_DH_CK) && (m_cr1 & CR1_DH_CK) == 0;
	UINT8 dv_clock = m_n_planes == 1 ? 0 : ((diffs & CR1_DV_CK) && (m_cr1 & CR1_DV_CK) == 0);

	if ((m_cr1 & CR1_RESET) == 0)
	{
		if (diffs & CR1_RESET)
		{
			MLOG1(("!!! set_cr1: CR1_RESET"));

			m_blt_cycle_count = 0;
			m_sr &= ~SR_ALT;
			m_image_offset = 0;
			memset(m_guard_latch, 0, sizeof(m_guard_latch));

			m_h_clock = 0;
			m_v_clock = 0;
			m_p_clock = 0;
			m_data_clock = 0;

			if (m_device_id == SCREEN_DEVICE_ID_19I)
			{
				m_sr = SR_H_CK | SR_V_BLANK | SR_H_SYNC | SR_V_SYNC;
			}
			else if (m_n_planes == 1)
			{
				m_sr = SR_V_BLANK | SR_V_SYNC;
			}
			else
			{
				m_sr = SR_H_CK | SR_V_BLANK | SR_SYNC | SR_DONE;
			}
		}
		log_cr1("CR1_RESET", device);
	}
	else
	{
		if ((diffs & CR1_RESET) && (m_cr1 & CR1_RESET) != 0)
		{
			log_cr1("CR1_RESET", device);
		}

		if (dh_clock)
		{
			increment_h_clock();
			log_cr1("CR1_DH_CK", device);
		}

		if (dv_clock)
		{
			increment_v_clock();
			log_cr1("CR1_DV_CK", device);
		}

		if (dp_clock)
		{
			increment_p_clock();
			log_cr1("CR1_DP_CK", device);
		}

		if ((m_sr & SR_V_BLANK) == 0)
		{
			m_sr &= ~SR_BLANK;
		}

		if (diffs & CR1_DISP_EN)
		{
			// update screen
			m_update_flag = 1;
		}
	}
}

void apollo_graphics::set_cr3a(device_t *device, UINT8 data)
{
	m_cr3a = data;
	if ((data & 0x80) == 0)
	{
		int shift = (data & 0x0f) >> 1;
		UINT8 bit_mask = 1 << shift;
		if (data & 0x01)
		{
			set_cr1(device, m_cr1 | bit_mask);
		}
		else
		{
			set_cr1(device, m_cr1 & ~bit_mask);
		}
	}
}

void apollo_graphics::set_cr3b(device_t *device, UINT8 data)
{
	m_cr3b = data;
	if ((data & 0x80) == 0)
	{
		int shift = (data & 0x0f) >> 1;
		UINT8 bit_mask = 1 << shift;
		if (data & 0x01)
		{
			set_lut_cr(device, m_lut_control | bit_mask);
		}
		else
		{
			set_lut_cr(device, m_lut_control & ~bit_mask);
		}
	}
}

void apollo_graphics::set_lut_cr(device_t *device, UINT8 data)
{
	UINT8 diffs = m_lut_control ^ data;
	m_lut_control = data;

	if ((diffs & LUT_CPAL_CS) && (data & LUT_CPAL_CS) != 0)
	{
		DLOG1(("writing Color Graphics Controller: LUT_CPAL_CS Disabled"));
		while (!m_lut_fifo->is_empty())
		{
			m_bt458->write(m_lut_fifo->get(), LUT_C1_C0(m_lut_control));
		}
	}

	if ((diffs & LUT_FIFO_RST) && (data & LUT_FIFO_RST) == 0)
	{
		DLOG1(("writing Color Graphics Controller: LUT_FIFO_RST Active"));
		m_lut_fifo->reset();
		m_sr |= SR_LUT_OK;
	}

	if ((diffs & LUT_FIFO_CS) && (data & LUT_FIFO_CS) == 0)
	{
		DLOG1(("writing Color Graphics Controller: LUT_FIFO_CS Enabled"));
	}

	if ((diffs & LUT_ST_LUK) && (data & LUT_ST_LUK) == 0)
	{
		DLOG1(("writing Color Graphics Controller: LUT_ST_LUK Active"));
		m_sr &= ~SR_LUT_OK;
	}
}

READ8_DEVICE_HANDLER( apollo_graphics::apollo_mcr_r )
{
	UINT8 data;
	switch (offset & 0x407)
	{
	case 0:
		data = m_sr;
		if (m_ad_pending)
		{
			m_ad_pending = 0;
			m_sr &= ~SR_DONE;
		}
		break;
	case 1:
		data = m_n_planes == 1 ? m_device_id : 0xff;
		break;
	case 0x400:
		data = m_cr0;
		break;
	case 0x402:
		data = m_cr1;
		break;
	case 0x404:
		data = m_cr2;
		break;
	case 0x406:
		data = m_cr3a;
		break;
	default:
		data = 0xff;
		break;
	}

	// omit excessive logging
	static UINT8 status0 = 0xff;
	if ((offset != 1) && (offset != 0 || data != status0))
	{
		if (offset == 0)
			status0 = data;
		DLOG1(("reading Graphics Controller at offset %03x = %02x (%s)", offset, data, cr_text(offset, data, 1)));
	}

	return data;
}

WRITE8_DEVICE_HANDLER( apollo_graphics::apollo_mcr_w )
{
	DLOG1(("writing Graphics Controller at offset %03x = %02x (%s)", offset, data, cr_text(offset, data, 0)));
	switch (offset & 0x407)
	{
	case 0:
		m_write_enable_register = set_lsb0(m_write_enable_register, data);
		break;
	case 1:
		m_write_enable_register = set_msb0(m_write_enable_register, data);
		// FIXME: seems to be necessary for dex
		m_blt_cycle_count = 0;
		m_sr &= ~SR_ALT;
		break;
	case 2:
		m_rop_register = set_lsb0(m_rop_register, data);
		break;
	case 3:
		m_rop_register = set_msb0(m_rop_register, data);
		set_status_rmw();
		break;
	case 4:
	case 5:
		// trigger memory refresh in diagnostic mode
		m_diag_mem_request = data;
		break;
	case 0x400:
		m_cr0 = data;
		break;
	case 0x402:
		set_cr1(device, data);
		break;
	case 0x404:
		m_cr2 = data;
		m_cr2_s_data = CR2_S_DATA(data);
		m_cr2_s_plane = 0x00;
		m_cr2_d_plane = 0x0e;
		// for DISP7B.DEX Test 16
		m_sr |= SR_R_M_W;
		break;
	case 0x406:
		set_cr3a(device, data);
		break;
	case 0x407: // A/D Channel Register
		m_ad_pending = 1;
		m_sr |= SR_DONE;
		break;
	}
}

void apollo_graphics::set_status_rmw()
{
	UINT8 plane, d_plane_bit;
	UINT32 rop_reg;

	m_sr &= ~SR_R_M_W;
	rop_reg = m_rop_register;
	d_plane_bit = 0x01;
	for (plane = 0; plane < m_n_planes; plane++)
	{
		if ((m_cr2_d_plane & d_plane_bit) == 0)
		{
			switch (rop_reg & 0x0f)
			{
			case 0: // zero
			case 3: // Source
			case 0x0c: // ~Source
			case 0x0f: // one
				break;
			default:
				m_sr |= SR_R_M_W;
				break;
			}
		}
		rop_reg >>= 4;
		d_plane_bit <<= 1;
	}
}

UINT16 apollo_graphics::rop(UINT16 dest_data, UINT16 src_data, UINT8 plane)
{
	UINT16 src_data1 = src_data;
	if (m_cr1 & CR1_ROP_EN)
	{
		switch ((m_rop_register >> (plane * 4)) & 0x0f)
		{
		case 0: // zero
			src_data = 0;
			break;
		case 1: // Source AND Destination
			src_data = src_data & dest_data;
			break;
		case 2: // Source AND ~Destination
			src_data = src_data & (~dest_data);
			break;
		case 3: // Source
			break;
		case 4: // ~Source AND Destination
			src_data = (~src_data) & dest_data;
			break;
		case 5: // Destination
			src_data = dest_data;
			break;
		case 6: // Source XOR Destination
			src_data = src_data ^ dest_data;
			break;
		case 7: // Source OR Destination
			src_data = src_data | dest_data;
			break;
		case 8: // Source NOR Destination
			src_data = ~(src_data | dest_data);
			break;
		case 9: // Source XNOR Destination
			src_data = ~(src_data ^ dest_data);
			break;
		case 0x0a: // ~Destination
			src_data = ~dest_data;
			break;
		case 0x0b: // Source OR ~Destination
			src_data = src_data | (~dest_data);
			break;
		case 0x0c: // ~Source
			src_data = ~src_data;
			break;
		case 0x0d: // ~Source OR Destination
			src_data = (~src_data) | dest_data;
			break;
		case 0x0e: // Source NAND Destination
			src_data = ~(src_data & dest_data);
			break;
		case 0x0f: // One
			src_data = 0xffff;
			break;
		}
	}
	MLOG2(("rop: cr0=%02x cr1=%02x cr2=%02x rop_register=%08x dest=%04x src=%04x plane=%d s_plane=%d ->%04x", m_cr0, m_cr1, m_cr2, m_rop_register,dest_data, src_data1, plane, m_cr2_s_plane, src_data ))
	return src_data;
}

void apollo_graphics::set_source_data(UINT32 offset)
{
	if (m_n_planes == 1 || (m_cr1 & CR1_AD_BIT))
	{
		offset += m_image_plane_size * m_cr2_s_plane;
		m_guard_latch[m_cr2_s_plane] <<= 16;
		m_guard_latch[m_cr2_s_plane] |= m_image_memory[offset];
	}
	else
	{
		UINT8 plane;
		for (plane = 0; plane < m_n_planes; plane++)
		{
			m_guard_latch[plane] <<= 16;
			m_guard_latch[plane] |= m_image_memory[offset];
			offset += m_image_plane_size;
		}
	}
}

UINT32 apollo_graphics::get_source_data(UINT8 plane)
{
	UINT32 src_data;

	if (m_n_planes == 1 || (m_cr1 & CR1_AD_BIT))
	{
		src_data = m_guard_latch[m_cr2_s_plane];
	}
	else
	{
		src_data = m_guard_latch[plane];
	}

	switch (m_cr2_s_data)
	{
	case CR2_CONST_ACCESS: // 0x00
		// set source to all ones (used for vectors)
		src_data = 0xffff;
		break;
	case CR2_PIXEL_ACCESS: // 0x01
		// replicate 4 LSB of data bus
		src_data = src_data & (1 << plane) ? 0xffff : 0;
		break;
	case CR2_SHIFT_ACCESS: // 0x02
		// replicate LSB of shifter
		src_data = src_data & 1 ? 0xffff : 0;
		break;
	case CR2_PLANE_ACCESS: // 0x03
		// use source data unchanged (normal use)
		if (CR0_SHIFT(m_cr0) >= 16)
		{
			src_data = (src_data << 16) | (src_data >> 16);
		}
		src_data >>= (CR0_SHIFT(m_cr0) & 0x0f);
		break;
	}
	return src_data;
}

void apollo_graphics::blt(UINT32 dest_addr, UINT16 mem_mask)
{
	UINT16 src_data, dest_data;
	UINT8 d_plane_bit;
	UINT8 plane;

	d_plane_bit = 0x01;
	for (plane = 0; plane < m_n_planes; plane++)
	{
		if ((m_cr2_d_plane & d_plane_bit) == 0)
		{
			dest_data = m_image_memory[dest_addr];
			src_data = get_source_data(plane);
			src_data = rop(dest_data, src_data, plane);
			src_data &= ~(m_write_enable_register | ~mem_mask);
			dest_data &= (m_write_enable_register | ~mem_mask);
			m_image_memory[dest_addr] = dest_data | src_data;
		}
		dest_addr += m_image_plane_size;
		d_plane_bit <<= 1;
	}
}

/***************************************************************************
 Monochrome graphics memory space at FA0000 - FDFFFF
 Color graphics memory space at A0000 - BFFFF
 ***************************************************************************/

READ16_DEVICE_HANDLER( apollo_graphics::apollo_mem_r )
{
	UINT16 data;
	UINT32 src_addr;

	if (offset >= m_image_memory_size)
	{
		// 128 kB display buffer of 15" screen seems to be shadowed from $fa0000 to $fc0000
		DLOG1(("reading Graphics Memory at invalid offset %05x", offset));
		offset %= m_image_memory_size;
	}

	src_addr = offset + m_image_plane_size * m_cr2_s_plane;

	switch (CR0_MODE(m_cr0))
	{
	case CR0_MODE_VECTOR: // vector or fill mode
	case CR0_MODE_3: // CPU source BLT: read internal data bus
		data = m_guard_latch[m_cr2_s_plane];
		break;

	default:
		set_source_data(offset);
		data = m_image_memory[src_addr];
		break;
	}

	// omit excessive logging
	if ((offset & (m_image_plane_size - 1)) < 8)
	{
		DLOG1(("reading Graphics Memory with mode %d: src_addr %05x = %04x & %04x", CR0_MODE(m_cr0), src_addr, data, mem_mask));
	}
	else if ((offset & (m_image_plane_size - 1)) == 8)
	{
		DLOG1(("..."));
	}
	return data;
}

WRITE16_DEVICE_HANDLER( apollo_graphics::apollo_mem_w )
{
	UINT32 dest_addr;
	UINT32 src_addr;

	if (offset >= m_image_memory_size)
	{
		// 128 kB display buffer of 15" screen seems to be shadowed from $fa0000 to $fc0000
		DLOG1(("writing Graphics Memory at invalid offset %05x = %04x & %04x ", offset, data, mem_mask));
		offset %= m_image_memory_size;
	}

	// omit excessive logging
	if (offset < 24)
	{
		DLOG1(("writing Graphics Memory with mode %d: offset=%04x data=%04x mask=%04x", CR0_MODE(m_cr0), offset, data, mem_mask));
	}
	else if (offset == 24)
	{
		DLOG1(("..."));
	}

	switch (CR0_MODE(m_cr0))
	{
	case CR0_MODE_0:
		// CPU destination BLT
		// 1. bus write to provide display memory address
		// 2. bus read to get data
		src_addr = offset + m_image_plane_size * m_cr2_s_plane;
		m_guard_latch[m_cr2_s_plane] <<= 16;
		m_guard_latch[m_cr2_s_plane] |= m_image_memory[src_addr];
		break;

	case CR0_MODE_1:
		// Alternating BLT
		// alternating bus writes provide src/dest address
		// second write provides Write-enables
		if (++m_blt_cycle_count == 1)
		{
			m_sr |= SR_ALT;
			set_source_data(offset);
		}
		else
		{
			m_blt_cycle_count = 0;
			m_sr &= ~SR_ALT;
			m_write_enable_register = data;
			blt(offset, mem_mask);
		}
		break;

	case CR0_MODE_VECTOR:
		// Vector or fill mode
		// write provides Write-enables and address
		m_write_enable_register = data;
		blt(offset, mem_mask);
		break;

	case CR0_MODE_3:
		// CPU source BLT
		// 1. bus write to provide src data
		// 2. bus write to provide Write-enables and address
		if (++m_blt_cycle_count == 1)
		{
			m_sr |= SR_ALT;

			// strange: must fix byte access for /systest/grtest on sr10.2
			if (mem_mask == 0xff00)
			{
				data >>= 8;
				mem_mask >>= 8;
			}

			m_guard_latch[m_cr2_s_plane] <<= 16;
			m_guard_latch[m_cr2_s_plane] |= (data & mem_mask);
		}
		else
		{
			m_blt_cycle_count = 0;
			m_sr &= ~SR_ALT;

			m_write_enable_register = data;
			blt(offset, mem_mask);
		}
		break;

	case CR0_MODE_BLT:
		// Double access BLT
		// bus write to provide src addr on address lines
		// dest addr on data lines (16-bit WORD Offset)

		set_source_data(offset);
		dest_addr = (data & mem_mask);
		if (m_device_id == SCREEN_DEVICE_ID_19I && (m_cr1 & CR1_DADDR_16))
		{
			dest_addr += 0x10000;
		}
		blt(dest_addr, 0xffff);
		break;

	case CR0_MODE_NORMAL:
		m_guard_latch[m_cr2_s_plane] <<= 16;
		m_guard_latch[m_cr2_s_plane] |= (data & mem_mask);
		blt(offset, mem_mask);
		break;

	default:
		DLOG(("writing Graphics Memory - unexpected cr0 mode %d", CR0_MODE(m_cr0)))
		;
		break;
	}
	m_update_flag = 1;
}

/***************************************************************************
 Color Screen
 ***************************************************************************/

READ8_DEVICE_HANDLER( apollo_graphics::apollo_ccr_r )
{
	UINT8 data;

	if (m_n_planes == 4)
	{
		switch (offset & 0x407)
		{
		case 1:
			data = m_n_planes == 4 ? m_device_id : 0xff;
			break;
		case 0x407:
			data = m_ad_result;
			break;
		default:
			return apollo_mcr_r(device, space, offset, mem_mask);
		}
	}
	else if (m_n_planes == 8)
	{
		switch (offset & 0x407)
		{
		case 1:
			data = m_n_planes == 8 ? m_device_id : 0xff;
			break;
		case 4:
			data = get_lsb1(m_rop_register);
			break;
		case 5:
			data = get_msb1(m_rop_register);
			break;

		case 0x401:
			// LUT data register
			if ((m_lut_control & LUT_FIFO_CS) == 0)
			{
				data = m_lut_fifo->get();
			}
			else if ((m_lut_control & LUT_R_W) == 0)
			{
				DLOG1(("apollo_graphics::apollo_ccr_r: reading LUT data register with unexpected RW = 0 in LUT Control register"));
				data = m_lut_data;
			}
			else if ((m_lut_control & LUT_AD_CS) == 0)
			{
				data = m_ad_result;
			}
			else if ((m_lut_control & LUT_CPAL_CS) == 0)
			{
				data = m_bt458->read(LUT_C1_C0(m_lut_control));
			}
			else
			{
				DLOG1(("apollo_graphics::apollo_ccr_r: reading LUT data register with unexpected CS in LUT Control register"));
				data = m_lut_data;
			}
			break;
		case 0x403:
			// LUT control register
			data = m_lut_control;
			break;
		case 0x404:
			// cr2a
			data = m_cr2;
			break;
		case 0x405:
			// cr2b
			data = m_cr2b;
			break;
		case 0x407:
			// cr3b
			data = m_cr3b;
			break;
		default:
			return apollo_mcr_r(device, space, offset, mem_mask);
		}
	}
	else
	{
		data = 0xff;
	}

	// omit excessive logging
	static UINT8 status1 = 0xff;
	if ((offset != 1) && (offset != 0 || data != status1))
	{
		if (offset == 0)
			status1 = data;
		DLOG1(("reading Color Graphics Controller at offset %03x = %02x (%s)", offset, data, cr_text(offset, data, 1)));
	}

	return data;
}

UINT8 apollo_graphics::get_pixel(UINT32 offset, UINT16 mask)
{
	UINT8 data = 0;
	UINT16 *source_ptr = m_image_memory + offset;

	if (m_n_planes == 4)
	{
		UINT16 data0 = source_ptr[0];
		UINT16 data1 = source_ptr[m_image_plane_size];
		UINT16 data2 = source_ptr[m_image_plane_size * 2];
		UINT16 data3 = source_ptr[m_image_plane_size * 3];

		data = (data0 & mask) ? 1 : 0;
		data |= (data1 & mask) ? 2 : 0;
		data |= (data2 & mask) ? 4 : 0;
		data |= (data3 & mask) ? 8 : 0;
	}
	else if (m_n_planes == 8)
	{
		UINT16 data0 = source_ptr[0];
		UINT16 data1 = source_ptr[m_image_plane_size];
		UINT16 data2 = source_ptr[m_image_plane_size * 2];
		UINT16 data3 = source_ptr[m_image_plane_size * 3];
		UINT16 data4 = source_ptr[m_image_plane_size * 4];
		UINT16 data5 = source_ptr[m_image_plane_size * 5];
		UINT16 data6 = source_ptr[m_image_plane_size * 6];
		UINT16 data7 = source_ptr[m_image_plane_size * 7];

		data = (data0 & mask) ? 1 : 0;
		data |= (data1 & mask) ? 2 : 0;
		data |= (data2 & mask) ? 4 : 0;
		data |= (data3 & mask) ? 8 : 0;
		data |= (data4 & mask) ? 0x10 : 0;
		data |= (data5 & mask) ? 0x20 : 0;
		data |= (data6 & mask) ? 0x40 : 0;
		data |= (data7 & mask) ? 0x80 : 0;
	}

	return data;
}

// read the 4-plane ADC value for data

UINT8 apollo_graphics::c4p_read_adc(UINT8 data)
{
	UINT8 value = 0;

	if ((data & 0x0c) == 0x04)
	{
		UINT8 red, green, blue;
		UINT8 pixel = get_pixel((m_v_clock * m_buffer_width / 16) + m_h_clock,
				0x8000);
		UINT32 rgb = m_color_lookup_table[pixel];

		if ((m_sr & SR_BLANK) != 0)
		{
			// not blanking
			red = 30 + ((rgb >> 16) & 0xff) / 4;
			green = 60 + ((rgb >> 8) & 0xff) / 4;
			blue = 30 + (rgb & 0xff) / 4;
		}
		else if (m_h_clock > 2)
		{
			// blanking
			red = 20;
			green = 50;
			blue = 20;
		}
		else
		{
			// sync
			red = 20;
			green = 10;
			blue = 20;
		}

		switch (data & 3)
		{
		case 0: // Red
			value = red;
			break;
		case 1: // Green
			value = green;
			break;
		case 2: // Blue
			value = blue;
			break;
		default: // unused
			value = 0;
			break;
		}
	}
	return value;
}

// read the 8-plane ADC value for data

UINT8 apollo_graphics::c8p_read_adc(UINT8 data)
{
	UINT8 value = 0;

	if ((data & 0x0c) == 0x04)
	{
		UINT8 red, green, blue;
		UINT8 pixel = get_pixel((m_v_clock * m_buffer_width / 16) + m_h_clock, 0x8000);
		UINT32 rgb = m_bt458->get_rgb(pixel);

		if ((m_sr & SR_BLANK) != 0)
		{
			// not blanking
			red = 10 + ((rgb >> 16) & 0xff) / 2;
			green = 70 + ((rgb >> 8) & 0xff) / 2;
			blue = 10 + (rgb & 0xff) / 2;
		}
		else if (m_h_clock < 20)
		{
			// blanking
			red = 5;
			green = 60;
			blue = 5;
		}
		else
		{
			// sync
			red = 5;
			green = 5;
			blue = 5;
		}

		switch (data & 3)
		{
		case 0: // Red
			value = red;
			break;
		case 1: // Green
			value = green;
			break;
		case 2: // Blue
			value = blue;
			break;
		default: // unused
			value = 0;
			break;
		}
	}
	return value;
}

WRITE8_DEVICE_HANDLER( apollo_graphics::apollo_ccr_w )
{
	static const UINT8 rgb_value[16] =
	{ 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb,
			0xcc, 0xdd, 0xee, 0xff };

	if (m_n_planes == 4)
	{
		switch (offset & 0x407)
		{
		case 0x401:
			// red lookup table register
			m_color_lookup_table[data >> 4] &= 0xff00ffff;
			m_color_lookup_table[data >> 4] |= rgb_value[data & 0x0f] << 16;
			break;
		case 0x403:
			// green lookup table register
			m_color_lookup_table[data >> 4] &= 0xffff00ff;
			m_color_lookup_table[data >> 4] |= rgb_value[data & 0x0f] << 8;
			break;
		case 0x404:
			// cr2
			m_cr2 = data;
			m_cr2_s_data = CR2_S_DATA(data);
			m_cr2_s_plane = CR2_S_PLANE(data);
			m_cr2_d_plane = CR2_D_PLANE(data);
			// for  DISP7B.DEX Test 16
			m_sr |= SR_R_M_W;
			break;
		case 0x405:
			// blue lookup table register
			m_color_lookup_table[data >> 4] &= 0xffffff00;
			m_color_lookup_table[data >> 4] |= rgb_value[data & 0x0f];
			break;
		case 0x407:
			// A/D channel register
			m_ad_result = c4p_read_adc(data);
			m_ad_pending = 1;
			m_sr |= SR_DONE;
			break;
		default:
			apollo_mcr_w(device, space, offset, data, mem_mask);
			return;
		}
	}
	else if (m_n_planes == 8)
	{
		switch (offset & 0x407)
		{
		case 2:
			m_rop_register = set_lsb0(m_rop_register, data);
			break;
		case 3:
			m_rop_register = set_msb0(m_rop_register, data);
			set_status_rmw();
			break;
		case 4:
			m_rop_register = set_lsb1(m_rop_register, data);
			break;
		case 5:
			m_rop_register = set_msb1(m_rop_register, data);
			set_status_rmw();
			break;
		case 6:
		case 7:
			// trigger memory refresh in diagnostic mode
			m_diag_mem_request = data;
			break;
		case 0x401:
			// LUT data register
			m_lut_data = data;
			if ((m_lut_control & LUT_R_W) == 1)
			{
				DLOG1(("apollo_graphics::apollo_ccr_w: writing LUT data register with RW = 1 in LUT Control register"));
			}
			else if ((m_lut_control & LUT_AD_CS) == 0)
			{
				m_ad_result = c8p_read_adc(data);
				m_ad_pending = 1;
				m_sr |= SR_DONE;
			}
			else if ((m_lut_control & LUT_CPAL_CS) == 0)
			{
				m_bt458->write(data, LUT_C1_C0(m_lut_control));
			}
			else if ((m_lut_control & LUT_FIFO_CS) == 0)
			{
				m_lut_fifo->put(data);
			}
			else
			{
				DLOG1(("apollo_graphics::apollo_ccr_w: writing LUT data register with unexpected CS in LUT Control register"));
			}
			break;
		case 0x403:
			// LUT control register
			set_lut_cr(device, data);
			break;
		case 0x404:
			// cr2a
			m_cr2 = data;
			m_cr2_d_plane = CR2A_D_PLANE(data);
			m_sr |= SR_R_M_W;
			break;
		case 0x405:
			// cr2b
			m_cr2b = data;
			m_cr2_s_data = CR2_S_DATA(data);
			m_cr2_s_plane = CR2B_S_PLANE(data);
			break;
		case 0x407:
			// cr3b
			set_cr3b(device, data);
			break;
		default:
			apollo_mcr_w(device, space, offset, data, mem_mask);
			return;
		}
	}

	DLOG1(("writing Color Graphics Controller at offset %03x = %02x (%s)", offset, data, cr_text(offset, data, 0)));
}

READ16_DEVICE_HANDLER( apollo_cgm_r )
{
	apollo_graphics *apollo_graphics = get_safe_token(device);
	if (!apollo_graphics->is_mono())
	{
		return apollo_graphics->apollo_mem_r(device, space, offset, mem_mask);
	}
	else
	{
		return 0xffff;
	}
}

WRITE16_DEVICE_HANDLER( apollo_cgm_w )
{
	apollo_graphics *apollo_graphics = get_safe_token(device);
	if (!apollo_graphics->is_mono())
	{
		apollo_graphics->apollo_mem_w(device, space, offset, data, mem_mask);
	}
}

/***************************************************************************
 VIDEO HARDWARE
 ***************************************************************************/

UINT32 apollo_graphics::screen_update(bitmap_rgb32 &bitmap,
		const rectangle &cliprect)
{
	int has_changed = 0;

	if (m_update_flag && !m_update_pending)
	{
		has_changed = 1;
		m_update_flag = 0;
		m_update_pending = 1;
		screen_update1(bitmap, cliprect);
		m_update_pending = 0;
	}
	return has_changed ? 0 : UPDATE_HAS_NOT_CHANGED;
}

void apollo_graphics::screen_update1(bitmap_rgb32 &bitmap,
		const rectangle &cliprect)
{
	UINT16 *source_ptr = m_image_memory;
	int x, y;
	UINT16 data, mask;
	UINT16 inverse = (m_cr1 & CR1_INV) ? 0xffff : 0;

	MLOG1(("screen_update1: size=%0x rowpixels=%d", m_image_memory_size, bitmap.rowpixels()));

	if ((m_cr1 & CR1_DISP_EN) == 0)
	{
		// display is disabled
		for (y = 0; y < m_height; y++)
		{
			int dest = 0;
			for (x = 0; x < m_width; x += 16)
			{
				for (mask = 0x8000; mask; mask >>= 1)
				{
					bitmap.pix32(y, dest++) = 0;
				}
			}
			source_ptr += (m_buffer_width - m_width) / 16;
		}
	}
	else if (m_n_planes == 4)
	{
		for (y = 0; y < m_height; y++)
		{
			int dest = 0;
			for (x = 0; x < m_width; x += 16)
			{
				UINT16 data0 = source_ptr[0];
				UINT16 data1 = source_ptr[m_image_plane_size];
				UINT16 data2 = source_ptr[m_image_plane_size * 2];
				UINT16 data3 = source_ptr[m_image_plane_size * 3];
				source_ptr++;
				for (mask = 0x8000; mask; mask >>= 1)
				{
					data = (data0 & mask) ? 1 : 0;
					data |= (data1 & mask) ? 2 : 0;
					data |= (data2 & mask) ? 4 : 0;
					data |= (data3 & mask) ? 8 : 0;
					bitmap.pix32(y, dest++) = m_color_lookup_table[data];
				}
			}
			source_ptr += (m_buffer_width - m_width) / 16;
		}
	}
	else if (m_n_planes == 8)
	{
		for (y = 0; y < m_height; y++)
		{
			int dest = 0;
			for (x = 0; x < m_width; x += 16)
			{
				UINT16 data0 = source_ptr[0];
				UINT16 data1 = source_ptr[m_image_plane_size];
				UINT16 data2 = source_ptr[m_image_plane_size * 2];
				UINT16 data3 = source_ptr[m_image_plane_size * 3];
				UINT16 data4 = source_ptr[m_image_plane_size * 4];
				UINT16 data5 = source_ptr[m_image_plane_size * 5];
				UINT16 data6 = source_ptr[m_image_plane_size * 6];
				UINT16 data7 = source_ptr[m_image_plane_size * 7];
				source_ptr++;
				for (mask = 0x8000; mask; mask >>= 1)
				{
					data = (data0 & mask) ? 1 : 0;
					data |= (data1 & mask) ? 2 : 0;
					data |= (data2 & mask) ? 4 : 0;
					data |= (data3 & mask) ? 8 : 0;
					data |= (data4 & mask) ? 0x10 : 0;
					data |= (data5 & mask) ? 0x20 : 0;
					data |= (data6 & mask) ? 0x40 : 0;
					data |= (data7 & mask) ? 0x80 : 0;
					bitmap.pix32(y, dest++) = m_bt458->get_rgb(data);
				}
			}
			source_ptr += (m_buffer_width - m_width) / 16;
		}
	}
	else // m_n_planes == 1
	{
		for (y = 0; y < m_height; y++)
		{
			int dest = 0;
			for (x = 0; x < m_width; x += 16)
			{
				data = *source_ptr++ ^ inverse;
				for (mask = 0x8000; mask; mask >>= 1)
				{
					bitmap.pix32(y, dest++) = data & mask ? 0 : 0x00ffffff;
				}
			}
			source_ptr += (m_buffer_width - m_width) / 16;
		}
	}
}

/*-------------------------------------------------
 vblank_state_changed -
 called on each state change of the VBLANK signal
 -------------------------------------------------*/

void apollo_graphics::vblank_state_changed(device_t *device,
		screen_device &screen, bool vblank_state)
{
	if ((m_cr1 & CR1_RESET) && (m_cr1 & CR1_SYNC_EN))
	{
		if (vblank_state)
		{
			m_sr &= ~(SR_V_BLANK | SR_BLANK);
			if (m_n_planes == 1)
			{
				// faking V_DATA for disp.dex test 16
				if (m_image_memory[0])
				{
					m_sr |= SR_V_DATA;
				}
			}
			else if (m_n_planes == 4)
			{
				m_sr &= ~SR_V_FLAG;
			}
		}
		else
		{
			m_sr |= (SR_V_BLANK | SR_BLANK);
			if (m_n_planes == 1)
			{
				m_sr &= ~SR_V_DATA;
			}
			else if (m_n_planes == 4)
			{
				m_sr |= SR_V_FLAG;
			}
		}
	}
}

void vblank_state_changed(device_t *device, screen_device &screen,
		bool vblank_state)
{
	apollo_graphics *apollo_graphics = get_safe_token(device);
	apollo_graphics->vblank_state_changed(device, screen, vblank_state);
}

static void register_vblank_callback(device_t *device)
{
	DLOG1(("register_vblank_callback"));

	/* register for VBLANK callbacks */
	screen_device *screen = (screen_device *) device->machine().device(
			VIDEO_SCREEN_TAG);
	screen->register_vblank_callback(vblank_state_delegate(
			FUNC(vblank_state_changed),device) );
}

VIDEO_START( apollo_screen )
{
}

SCREEN_UPDATE_RGB32( apollo_screen )
{
	// FIXME: omit using APOLLO_SCREEN_TAG
	device_t *apollo_screen = screen.machine().device(APOLLO_SCREEN_TAG);
	apollo_graphics *apollo_graphics = get_safe_token(apollo_screen);

	return apollo_graphics->screen_update(bitmap, cliprect);
}

/***************************************************************************
 MACHINE DRIVERS
 ***************************************************************************/

MACHINE_CONFIG_FRAGMENT( apollo_graphics )
	MCFG_DEFAULT_LAYOUT( layout_apollo_15i )
	MCFG_SCREEN_ADD(VIDEO_SCREEN_TAG, RASTER)
	MCFG_VIDEO_ATTRIBUTES(VIDEO_UPDATE_AFTER_VBLANK)
	MCFG_SCREEN_REFRESH_RATE(76)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(657))
	MCFG_SCREEN_SIZE(1024, 800)
	MCFG_SCREEN_VISIBLE_AREA(0, 1023, 0, 799)
	MCFG_VIDEO_START(apollo_screen)
	MCFG_SCREEN_UPDATE_STATIC(apollo_screen)
	MACHINE_CONFIG_END

const device_type APOLLO_GRAPHICS = &device_creator<apollo_graphics_15i> ;

apollo_graphics_15i::apollo_graphics_15i(const machine_config &mconfig,
		const char *tag, device_t *owner, UINT32 clock) :
	device_t(mconfig, APOLLO_GRAPHICS, "Apollo Screen", tag, owner, clock,
			"apollo_graphics_15i", __FILE__)
{
	m_token = new apollo_graphics;
}

apollo_graphics_15i::apollo_graphics_15i(const machine_config &mconfig,
		const char *tag, device_t *owner, UINT32 clock, device_type type,
		const char *name, const char *shortname, const char *source) :
	device_t(mconfig, type, name, tag, owner, clock, shortname, source)
{
	m_token = new apollo_graphics;
}

//-------------------------------------------------
//  device_config_complete - perform any
//  operations now that the configuration is
//  complete
//-------------------------------------------------

void apollo_graphics_15i::device_config_complete()
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void apollo_graphics_15i::device_start()
{
	MLOG1(("apollo_graphics_15i::device_start"))

	apollo_graphics *apollo_graphics = get_safe_token(this);
	apollo_graphics->device_start(machine());
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void apollo_graphics_15i::device_reset()
{
	MLOG1(("apollo_graphics_15i::device_reset"));

	apollo_graphics *apollo_graphics = get_safe_token(this);
	apollo_graphics->device_reset();

	/* FIXME: register for VBLANK callbacks */
	register_vblank_callback(this);
}

READ8_DEVICE_HANDLER( apollo_ccr_r )
{
	apollo_graphics *apollo_graphics = get_safe_token(device);
	return apollo_graphics->apollo_ccr_r(device, space, offset, mem_mask);
}

WRITE8_DEVICE_HANDLER( apollo_ccr_w )
{
	apollo_graphics *apollo_graphics = get_safe_token(device);
	apollo_graphics->apollo_ccr_w(device, space, offset, data, mem_mask);
}

//-------------------------------------------------

MACHINE_CONFIG_FRAGMENT( apollo_mono19i )
	MCFG_DEFAULT_LAYOUT( layout_apollo )
	MCFG_SCREEN_ADD(VIDEO_SCREEN_TAG, RASTER)
	MCFG_VIDEO_ATTRIBUTES(VIDEO_UPDATE_AFTER_VBLANK)
	MCFG_PALETTE_LENGTH(2)
	MCFG_PALETTE_INIT_OVERRIDE(driver_device, black_and_white)
	MCFG_SCREEN_REFRESH_RATE(64)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(616))
	MCFG_SCREEN_SIZE(1280, 1024)
	MCFG_SCREEN_VISIBLE_AREA(0, 1279, 0, 1023)
	MCFG_VIDEO_START(apollo_screen)
	MCFG_SCREEN_UPDATE_STATIC(apollo_screen)
	MACHINE_CONFIG_END

const device_type APOLLO_MONO19I = &device_creator<apollo_graphics_19i> ;

apollo_graphics_19i::apollo_graphics_19i(const machine_config &mconfig,
		const char *tag, device_t *owner, UINT32 clock) :
	apollo_graphics_15i(mconfig, tag, owner, clock, APOLLO_MONO19I,
			"Apollo 19\" Monochrome Screen", "apollo_graphics_19i", __FILE__)
{
}

//-------------------------------------------------
//  device_config_complete - perform any
//  operations now that the configuration is
//  complete
//-------------------------------------------------

void apollo_graphics_19i::device_config_complete()
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void apollo_graphics_19i::device_start()
{
	MLOG1(("apollo_graphics_19i::device_start"));

	apollo_graphics_15i::device_start();
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void apollo_graphics_19i::device_reset()
{
	MLOG1(("apollo_graphics_19i::device_reset"));

	apollo_graphics *apollo_graphics = get_safe_token(this);
	apollo_graphics->device_reset_mono19i();

	/* FIXME: register for VBLANK callbacks */
	register_vblank_callback(this);
}

READ8_DEVICE_HANDLER( apollo_mcr_r )
{
	apollo_graphics *apollo_graphics = get_safe_token(device);
	return apollo_graphics->apollo_mcr_r(device, space, offset, mem_mask);
}

WRITE8_DEVICE_HANDLER( apollo_mcr_w )
{
	apollo_graphics *apollo_graphics = get_safe_token(device);
	apollo_graphics->apollo_mcr_w(device, space, offset, data, mem_mask);
}

READ16_DEVICE_HANDLER( apollo_mgm_r )
{
	apollo_graphics *apollo_graphics = get_safe_token(device);
	if (apollo_graphics->is_mono())
	{
		return apollo_graphics->apollo_mem_r(device, space, offset, mem_mask);
	}
	else
	{
		return 0xffff;
	}
}

WRITE16_DEVICE_HANDLER( apollo_mgm_w )
{
	apollo_graphics *apollo_graphics = get_safe_token(device);
	if (apollo_graphics->is_mono())
	{
		apollo_graphics->apollo_mem_w(device, space, offset, data, mem_mask);
	}
}

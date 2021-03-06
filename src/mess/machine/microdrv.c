/*********************************************************************

    microdrv.c

    MESS interface to the Sinclair Microdrive image abstraction code

*********************************************************************/

#include "emu.h"
#include "microdrv.h"

/***************************************************************************
    CONSTANTS
***************************************************************************/

#define LOG 1

#define MDV_SECTOR_COUNT            255
#define MDV_SECTOR_LENGTH           686
#define MDV_IMAGE_LENGTH            (MDV_SECTOR_COUNT * MDV_SECTOR_LENGTH)

#define MDV_PREAMBLE_LENGTH         12
#define MDV_GAP_LENGTH              120

#define MDV_OFFSET_HEADER_PREAMBLE  0
#define MDV_OFFSET_HEADER           MDV_OFFSET_HEADER_PREAMBLE + MDV_PREAMBLE_LENGTH
#define MDV_OFFSET_DATA_PREAMBLE    28
#define MDV_OFFSET_DATA             MDV_OFFSET_DATA_PREAMBLE + MDV_PREAMBLE_LENGTH
#define MDV_OFFSET_GAP              566

#define MDV_BITRATE                 120000 // invalid, from ZX microdrive

/***************************************************************************
    TYPE DEFINITIONS
***************************************************************************/

// device type definition
const device_type MICRODRIVE = &device_creator<microdrive_image_device>;

//-------------------------------------------------
//  microdrive_image_device - constructor
//-------------------------------------------------

microdrive_image_device::microdrive_image_device(const machine_config &mconfig, const char *tag, device_t *owner, UINT32 clock)
	: device_t(mconfig, MICRODRIVE, "Microdrive", tag, owner, clock, "microdrive_image", __FILE__),
		device_image_interface(mconfig, *this)
{
}

//-------------------------------------------------
//  microdrive_image_device - destructor
//-------------------------------------------------

microdrive_image_device::~microdrive_image_device()
{
}

//-------------------------------------------------
//  device_config_complete - perform any
//  operations now that the configuration is
//  complete
//-------------------------------------------------

void microdrive_image_device::device_config_complete()
{
	// inherit a copy of the static data
	const microdrive_interface *intf = reinterpret_cast<const microdrive_interface *>(static_config());
	if (intf != NULL)
		*static_cast<microdrive_interface *>(this) = *intf;

	// or initialize to defaults if none provided
	else
	{
		memset(&m_out_comms_out_cb, 0, sizeof(m_out_comms_out_cb));
		memset(&m_interface, 0, sizeof(m_interface));
		memset(&m_device_displayinfo, 0, sizeof(m_device_displayinfo));
	}

	// set brief and instance name
	update_names();
}


void microdrive_image_device::device_start()
{
	// resolve callbacks
	m_out_comms_out_func.resolve(m_out_comms_out_cb, *this);

	// allocate track buffers
	m_left = auto_alloc_array(machine(), UINT8, MDV_IMAGE_LENGTH / 2);
	m_right = auto_alloc_array(machine(), UINT8, MDV_IMAGE_LENGTH / 2);

	// allocate timers
	m_bit_timer = timer_alloc();
	m_bit_timer->adjust(attotime::zero, 0, attotime::from_hz(MDV_BITRATE));
	m_bit_timer->enable(0);

	m_clk = 0;
	m_comms_in = 0;
	m_comms_out = 0;
}

bool microdrive_image_device::call_load()
{
	if (length() != MDV_IMAGE_LENGTH)
		return IMAGE_INIT_FAIL;

	for (int i = 0; i < MDV_IMAGE_LENGTH / 2; i++)
	{
		fread(m_left, 1);
		fread(m_right, 1);
	}

	m_bit_offset = 0;
	m_byte_offset = 0;

	return IMAGE_INIT_PASS;
}

void microdrive_image_device::call_unload()
{
	memset(m_left, 0, MDV_IMAGE_LENGTH / 2);
	memset(m_right, 0, MDV_IMAGE_LENGTH / 2);
}

void microdrive_image_device::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	m_bit_offset++;

	if (m_bit_offset == 8)
	{
		m_bit_offset = 0;
		m_byte_offset++;

		if (m_byte_offset == MDV_IMAGE_LENGTH)
		{
			m_byte_offset = 0;
		}
	}
}

WRITE_LINE_MEMBER( microdrive_image_device::clk_w )
{
	if (LOG) logerror("Microdrive '%s' CLK: %u\n", tag(), state);
	if (!m_clk && state)
	{
		m_comms_out = m_comms_in;
		if (LOG) logerror("Microdrive '%s' COMMS OUT: %u\n", tag(), m_comms_out);
		m_out_comms_out_func(m_comms_out);
		m_bit_timer->enable(m_comms_out);
	}
	m_clk = state;
}

WRITE_LINE_MEMBER( microdrive_image_device::comms_in_w )
{
	if (LOG) logerror("Microdrive '%s' COMMS IN: %u\n", tag(), state);
	m_comms_in = state;
}

WRITE_LINE_MEMBER( microdrive_image_device::erase_w )
{
	if (LOG) logerror("Microdrive '%s' ERASE: %u\n", tag(), state);
	m_erase = state;
}

WRITE_LINE_MEMBER( microdrive_image_device::read_write_w )
{
	if (LOG) logerror("Microdrive '%s' READ/WRITE: %u\n", tag(), state);
	m_read_write = state;
}

WRITE_LINE_MEMBER( microdrive_image_device::data1_w )
{
	if (m_comms_out && !m_read_write)
	{
		// TODO
	}
}

WRITE_LINE_MEMBER( microdrive_image_device::data2_w )
{
	if (m_comms_out && !m_read_write)
	{
		// TODO
	}
}

READ_LINE_MEMBER( microdrive_image_device::data1_r )
{
	int data = 0;
	if (m_comms_out && m_read_write)
	{
		data = BIT(m_left[m_byte_offset], 7 - m_bit_offset);
	}
	return data;
}

READ_LINE_MEMBER( microdrive_image_device::data2_r )
{
	int data = 0;
	if (m_comms_out && m_read_write)
	{
		data = BIT(m_right[m_byte_offset], 7 - m_bit_offset);
	}
	return data;
}

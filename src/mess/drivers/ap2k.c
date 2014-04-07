/***************************************************************************

    Epson ActionPrinter 2000 dot matrix printer

    license: MAME, GPL-2.0+
    copyright-holders: Dirk Best
                       Ramiro Polla

    Skeleton driver

    - CPU type uPD7810HG
    - CPU PORTD and PORTF are connected to the Gate Array
    - processing gets stuck in a loop, and never gets to scan the
      input buttons and switches.
    - CPU disassembly doesn't seem to indicate conditional JR or RET.

***************************************************************************/

#include "emu.h"
#include "cpu/upd7810/upd7810.h"
#include "machine/eepromser.h"
#include "machine/steppers.h"
#include "machine/e05a30.h"
#include "sound/beep.h"
#include "ap2k.lh"

//#define AP2KDEBUG
#ifdef AP2KDEBUG
#define LOG(...) fprintf(stderr, __VA_ARGS__)
#else
#define LOG(...)
#endif

/***************************************************************************
    TYPE DEFINITIONS
***************************************************************************/

class ap2k_state : public driver_device
{
public:
	ap2k_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
	m_maincpu(*this, "maincpu"),
	m_eeprom(*this, "eeprom"),
	m_beep(*this, "beeper"),
	m_93c06_clk(0),
	m_93c06_cs(0),
	m_pf_pos_abs(200),
	m_pf_pos_prev(0),
	m_cr_pos_abs(200),
	m_cr_pos_prev(0)
	{ }

	required_device<cpu_device> m_maincpu;
	required_device<eeprom_serial_93cxx_device> m_eeprom;
	required_device<beep_device> m_beep;
	DECLARE_READ8_MEMBER(ap2k_porta_r);
	DECLARE_WRITE8_MEMBER(ap2k_porta_w);
	DECLARE_READ8_MEMBER(ap2k_portb_r);
	DECLARE_WRITE8_MEMBER(ap2k_portb_w);
	DECLARE_READ8_MEMBER(ap2k_portc_r);
	DECLARE_WRITE8_MEMBER(ap2k_portc_w);

	/* ADC */
	DECLARE_READ8_MEMBER(ap2k_an0_r);
	DECLARE_READ8_MEMBER(ap2k_an1_r);
	DECLARE_READ8_MEMBER(ap2k_an2_r);
	DECLARE_READ8_MEMBER(ap2k_an3_r);
	DECLARE_READ8_MEMBER(ap2k_an4_r);
	DECLARE_READ8_MEMBER(ap2k_an5_r);
	DECLARE_READ8_MEMBER(ap2k_an6_r);
	DECLARE_READ8_MEMBER(ap2k_an7_r);

	/* fake memory I/O to get past memory reset check */
	DECLARE_READ8_MEMBER(ff_r);
	DECLARE_WRITE8_MEMBER(ff_w);

	/* GATE ARRAY */
	DECLARE_WRITE8_MEMBER(ap2k_pf_stepper);
	DECLARE_WRITE8_MEMBER(ap2k_cr_stepper);

	virtual void machine_start();

private:
	int m_93c06_clk;
	int m_93c06_cs;
	int m_pf_pos_abs;
	int m_pf_pos_prev;
	int m_cr_pos_abs;
	int m_cr_pos_prev;
	UINT8 m_ff;
};


/***************************************************************************
    I/O PORTS
***************************************************************************/

/*
 * PA0  R   CN7 sensor (Home Position, HP, active low)
 * PA1  R   CN6 sensor (Paper-End, PE, active low)
 * PA2  R   CN4 sensor (Release, low = tractor)
 * PA3   W  Stepper motor voltage reference (these 3 pins make up one voltage)
 * PA4   W  Stepper motor voltage reference (these 3 pins make up one voltage)
 * PA5   W  Stepper motor voltage reference (these 3 pins make up one voltage)
 * PA6  R   Line Feed SWITCH
 * PA7  R   Form Feed SWITCH
 */
READ8_MEMBER( ap2k_state::ap2k_porta_r )
{
	UINT8 result = 0;
	UINT8 hp_sensor = m_cr_pos_abs <= 0 ? 0 : 1;

	result |= hp_sensor; /* home position */
	result |= ioport("LINEFEED")->read() << 6;
	result |= ioport("FORMFEED")->read() << 7;

	LOG("%s: ap2k_PA_r(%02x): result %02x\n", machine().describe_context(), offset, result);

	return result;
}

WRITE8_MEMBER( ap2k_state::ap2k_porta_w )
{
	LOG("%s: ap2k_PA_w(%02x): %02x: stepper vref %d\n", machine().describe_context(), offset, data, BIT(data, 3) | (BIT(data, 4)<<1) | (BIT(data, 5)<<2));
}

/*
 * PB0  R   DIP1.0 & 93C06.DO
 * PB1  RW  DIP1.1 & 93C06.DI
 * PB2  R   DIP1.2
 * PB3  R   DIP1.3
 * PB4  R   DIP1.4
 * PB5  R   DIP1.5
 * PB6  R   DIP1.6
 * PB7  R   DIP1.7
 */
READ8_MEMBER( ap2k_state::ap2k_portb_r )
{
	UINT8 result = ioport("LINEFEED")->read();

	/* if 93C06 is selected */
	if (m_93c06_cs) {
		UINT8 do_r = m_eeprom->do_read();
		result &= 0xfe;
		result |= do_r;
	}

	LOG("%s: ap2k_PB_r(%02x): result %02x\n", machine().describe_context(), offset, result);

	return result;
}

WRITE8_MEMBER( ap2k_state::ap2k_portb_w )
{
	UINT8 data_in = BIT(data, 1);

	/* if 93C06 is selected */
	if (m_93c06_cs)
		m_eeprom->di_write(data_in);

	LOG("%s: ap2k_PB_w(%02x): %02x: 93c06 data %d\n", machine().describe_context(), offset, data, data_in);
}

/*
 * PC0   W  TXD        serial i/o txd, also TAMA.25
 * PC1  R   RXD        serial i/o rxd, also E05A30.28
 * PC2   W  ????????  W  ONLINE LP  online led
 * PC3  R   ONLINE SW  online switch
 * PC4   W  93C06.SK
 * PC5   W  93C06.CS
 * PC6   W  E05A30.57 ???????? W  FIRE       drive pulse width signal
 * PC7   W  BUZZER     buzzer signal
 */
READ8_MEMBER( ap2k_state::ap2k_portc_r )
{
	UINT8 result = 0;

	LOG("%s: ap2k_PC_r(%02x)\n", machine().describe_context(), offset);

	/* result |= ioport("serial")->read() << 1; */
	result |= ioport("ONLINE")->read() << 3;
	result |= m_93c06_clk << 4;
	result |= m_93c06_cs  << 5;

	return result;
}

WRITE8_MEMBER( ap2k_state::ap2k_portc_w )
{
	/* ioport("serial")->write(BIT(data, 0)); */

	m_93c06_clk = BIT(data, 4);
	m_93c06_cs  = BIT(data, 5);

	LOG("%s: ap2k_PC_w(%02x): %02x 93c06 clk: %d cs: %d\n", machine().describe_context(), offset, data, m_93c06_clk, m_93c06_cs);

	m_eeprom->clk_write(m_93c06_clk ? ASSERT_LINE : CLEAR_LINE);
	m_eeprom->cs_write (m_93c06_cs  ? ASSERT_LINE : CLEAR_LINE);

	output_set_value("online_led", !BIT(data, 2));

	m_beep->set_state(!BIT(data, 7));
}


/***************************************************************************
    GATE ARRAY
***************************************************************************/

WRITE8_MEMBER(ap2k_state::ap2k_pf_stepper)
{
	int prev = m_pf_pos_prev;

	stepper_update(0, data);
	int pos = stepper_get_position(0);

	LOG("%s: %s(%02x); prev %d cur %d abs %d\n", machine().describe_context(), __func__, data, prev, pos, m_pf_pos_abs);

	if      (prev == 95 && !pos) m_pf_pos_abs--;
	else if (!prev && pos == 95) m_pf_pos_abs++;
	else                         m_pf_pos_abs += prev - pos;
	m_pf_pos_prev = pos;
}

WRITE8_MEMBER(ap2k_state::ap2k_cr_stepper)
{
	int prev = m_cr_pos_prev;

	stepper_update(1, data);
	int pos = stepper_get_position(1);

	LOG("%s: %s(%02x); prev %d cur %d abs %d\n", machine().describe_context(), __func__, data, prev, pos, m_cr_pos_abs);

	if      (prev == 95 && !pos) m_cr_pos_abs--;
	else if (!prev && pos == 95) m_cr_pos_abs++;
	else                         m_cr_pos_abs += prev - pos;
	m_cr_pos_prev = pos;
}


/***************************************************************************
    ADC
***************************************************************************/

static int reset_it = 1;
READ8_MEMBER(ap2k_state::ap2k_an0_r)
{
	if (reset_it) {
		/* HACK */
		m_maincpu->set_input_line(INPUT_LINE_NMI, ASSERT_LINE);
		reset_it = 0;
	}
	return 0;
	return 0xff; /* DIPSW2.1 */
}

READ8_MEMBER(ap2k_state::ap2k_an1_r)
{
	return 0xff; /* DIPSW2.2 */
}

READ8_MEMBER(ap2k_state::ap2k_an2_r)
{
	return 0xff; /* DIPSW2.3 */
}

READ8_MEMBER(ap2k_state::ap2k_an3_r)
{
	return 0xff; /* DIPSW2.4 */
}

READ8_MEMBER(ap2k_state::ap2k_an4_r)
{
	return 0xff;
}

READ8_MEMBER(ap2k_state::ap2k_an5_r)
{
	return 0xCB; /* motor voltage, 0xcb = 24V */
}

READ8_MEMBER(ap2k_state::ap2k_an6_r)
{
	return 0xff;
}

READ8_MEMBER(ap2k_state::ap2k_an7_r)
{
	return 0xff;
}

/***************************************************************************
    MACHINE EMULATION
***************************************************************************/

static const stepper_interface pf_stepper =
{
	STARPOINT_48STEP_REEL,
	16,
	24,
	0x00,
	0
};

static const stepper_interface cr_stepper =
{
	STARPOINT_48STEP_REEL,
	16,
	24,
	0x00,
	2
};

void ap2k_state::machine_start()
{
	m_beep->set_state(0);
	m_beep->set_frequency(4000); /* ? */

	stepper_config(machine(), 0, &pf_stepper);
	stepper_config(machine(), 1, &cr_stepper);
}


/***************************************************************************
    FF READ/WRITE
***************************************************************************/

READ8_MEMBER(ap2k_state::ff_r)
{
	return m_ff;
}

WRITE8_MEMBER(ap2k_state::ff_w)
{
	m_ff = data;
}


/***************************************************************************
    ADDRESS MAPS
***************************************************************************/

static ADDRESS_MAP_START( ap2k_mem, AS_PROGRAM, 8, ap2k_state )
	AM_RANGE(0x0000, 0x7fff) AM_ROM /* 32k firmware */
	AM_RANGE(0x8000, 0x9fff) AM_RAM /* 8k external RAM */
	AM_RANGE(0xa000, 0xbfff) AM_READWRITE(ff_r, ff_w) /* UNKNOWN should be 0xff */
//	AM_RANGE(0xa000, 0xbfff) AM_NOP /* not used */
	AM_RANGE(0xc000, 0xdfff) AM_MIRROR(0x1ff0) AM_DEVREADWRITE("ic3b", e05a30_device, read, write)
	AM_RANGE(0xe000, 0xfeff) AM_NOP /* not used */
	AM_RANGE(0xff00, 0xffff) AM_RAM /* internal CPU RAM */
ADDRESS_MAP_END

static ADDRESS_MAP_START( ap2k_io, AS_IO, 8, ap2k_state )
	AM_RANGE(UPD7810_PORTA, UPD7810_PORTA) AM_READWRITE(ap2k_porta_r, ap2k_porta_w)
	AM_RANGE(UPD7810_PORTB, UPD7810_PORTB) AM_READWRITE(ap2k_portb_r, ap2k_portb_w)
	AM_RANGE(UPD7810_PORTC, UPD7810_PORTC) AM_READWRITE(ap2k_portc_r, ap2k_portc_w)
ADDRESS_MAP_END


/***************************************************************************
    INPUT PORTS
***************************************************************************/

static INPUT_PORTS_START( ap2k )

	/* Buttons on printer */
	PORT_START("ONLINE")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("On Line") PORT_CODE(KEYCODE_O)
	PORT_START("FORMFEED")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Form Feed") PORT_CODE(KEYCODE_F)
	PORT_START("LINEFEED")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Line Feed") PORT_CODE(KEYCODE_L)
	PORT_START("LOAD_EJECT")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Load/Eject") PORT_CODE(KEYCODE_E)

	/* DIPSW1 */
	PORT_START("DIPSW1")

	PORT_DIPNAME(0x01, 0x01, "Character spacing")
	PORT_DIPLOCATION("DIP:1")
	PORT_DIPSETTING(0x01, "12 cpi") /* default */
	PORT_DIPSETTING(0x00, "10 cpi")

	PORT_DIPNAME(0x02, 0x00, "Shape of zero")
	PORT_DIPLOCATION("DIP:2")
	PORT_DIPSETTING(0x02, "Slashed")
	PORT_DIPSETTING(0x00, "Not slashed") /* default */

	PORT_DIPNAME(0x0c, 0x08, "Page length")
	PORT_DIPLOCATION("DIP:3,4")
	PORT_DIPSETTING(0x00, "11 inches")
	PORT_DIPSETTING(0x04, "12 inches")
	PORT_DIPSETTING(0x08, "8.5 inches") /* default */
	PORT_DIPSETTING(0x0c, "11.7 inches")

	PORT_DIPNAME(0x10, 0x10, "Character table")
	PORT_DIPLOCATION("DIP:5")
	PORT_DIPSETTING(0x10, "Graphics") /* default */
	PORT_DIPSETTING(0x00, "Italics")

	PORT_DIPNAME(0xe0, 0xe0, "International characters and PC selection")
	PORT_DIPLOCATION("DIP:6,7,8")
	PORT_DIPSETTING(0xe0, "United States") /* default */
	PORT_DIPSETTING(0x60, "France")
	PORT_DIPSETTING(0xa0, "Germany")
	PORT_DIPSETTING(0x20, "United Kingdom")
	PORT_DIPSETTING(0xc0, "Denmark")
	PORT_DIPSETTING(0x40, "Sweden")
	PORT_DIPSETTING(0x80, "Italy")
	PORT_DIPSETTING(0x00, "Spain")

	/* DIPSW2 */
	PORT_START("DIPSW2")

	PORT_DIPNAME(0x01, 0x01, "Short tear-off")
	PORT_DIPLOCATION("DIP:1")
	PORT_DIPSETTING(0x01, "Invalid") /* default */
	PORT_DIPSETTING(0x00, "Valid")

	PORT_DIPNAME(0x02, 0x00, "Cut-sheet feeder mode")
	PORT_DIPLOCATION("DIP:2")
	PORT_DIPSETTING(0x02, "ON")
	PORT_DIPSETTING(0x00, "OFF") /* default */

	PORT_DIPNAME(0x04, 0x00, "Skip-over-perforation")
	PORT_DIPLOCATION("DIP:3")
	PORT_DIPSETTING(0x04, "ON")
	PORT_DIPSETTING(0x00, "OFF") /* default */

	PORT_DIPNAME(0x08, 0x00, "Auto line feed")
	PORT_DIPLOCATION("DIP:4")
	PORT_DIPSETTING(0x08, "ON")
	PORT_DIPSETTING(0x00, "OFF") /* default */

INPUT_PORTS_END


/***************************************************************************
    MACHINE DRIVERS
***************************************************************************/

static const e05a30_interface ap2k_e05a30_intf =
{
	DEVCB_DRIVER_MEMBER(ap2k_state, ap2k_pf_stepper),
	DEVCB_DRIVER_MEMBER(ap2k_state, ap2k_cr_stepper),
};

static MACHINE_CONFIG_START( ap2k, ap2k_state )
	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", UPD7810, XTAL_14_7456MHz)
	MCFG_UPD7810_AN0(READ8(ap2k_state, ap2k_an0_r))
	MCFG_UPD7810_AN1(READ8(ap2k_state, ap2k_an1_r))
	MCFG_UPD7810_AN2(READ8(ap2k_state, ap2k_an2_r))
	MCFG_UPD7810_AN3(READ8(ap2k_state, ap2k_an3_r))
	MCFG_UPD7810_AN4(READ8(ap2k_state, ap2k_an4_r))
	MCFG_UPD7810_AN5(READ8(ap2k_state, ap2k_an5_r))
	MCFG_UPD7810_AN6(READ8(ap2k_state, ap2k_an6_r))
	MCFG_UPD7810_AN7(READ8(ap2k_state, ap2k_an7_r))
	MCFG_CPU_PROGRAM_MAP(ap2k_mem)
	MCFG_CPU_IO_MAP(ap2k_io)

	MCFG_DEFAULT_LAYOUT(layout_ap2k)

	/* audio hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_ADD("beeper", BEEP, 0)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.0)

	/* gate array */
	MCFG_E05A30_ADD("ic3b", ap2k_e05a30_intf)

	/* 256-bit eeprom */
	MCFG_EEPROM_SERIAL_93C06_ADD("eeprom")
MACHINE_CONFIG_END


/***************************************************************************
    ROM DEFINITIONS
***************************************************************************/

ROM_START( ap2k )
	ROM_REGION(0x8000, "maincpu", 0)
	ROM_LOAD("ap2k.ic3c", 0x0000, 0x8000, CRC(ee7294b7) SHA1(219ffa6ff661ce95d5772c9fc1967093718f04e9))

//	ROM_REGION(0x2000, "ff", 0)
//	ROM_LOAD("ff", 0x0000, 0x2000, CRC(b4293435) SHA1(5e2b96c19c4f5c63a5afa2de504d29fe64a4c908))

	ROM_REGION(0x20, "eeprom", 0)
	ROM_LOAD( "at93c06", 0x0000, 0x0020, NO_DUMP )
ROM_END


/***************************************************************************
    GAME DRIVERS
***************************************************************************/

/*    YEAR  NAME   PARENT  COMPAT  MACHINE  INPUT  INIT  COMPANY  FULLNAME  FLAGS */
COMP( 1987, ap2k, 0,      0,      ap2k,   ap2k, driver_device, 0,    "Epson", "ActionPrinter 2000", GAME_NOT_WORKING )

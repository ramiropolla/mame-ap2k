/***************************************************************************

    Atari Skull & Crossbones hardware

    driver by Aaron Giles

    Games supported:
        * Skull & Crossbones (1989) [5 sets]

    Known bugs:
        * none at this time

****************************************************************************

    Memory map (TBA)

***************************************************************************/


#include "emu.h"
#include "cpu/m68000/m68000.h"
#include "video/atarimo.h"
#include "includes/skullxbo.h"



/*************************************
 *
 *  Initialization & interrupts
 *
 *************************************/

void skullxbo_state::update_interrupts()
{
	m_maincpu->set_input_line(1, m_scanline_int_state ? ASSERT_LINE : CLEAR_LINE);
	m_maincpu->set_input_line(2, m_video_int_state ? ASSERT_LINE : CLEAR_LINE);
	m_maincpu->set_input_line(4, m_sound_int_state ? ASSERT_LINE : CLEAR_LINE);
}


TIMER_DEVICE_CALLBACK_MEMBER(skullxbo_state::scanline_timer)
{
	scanline_int_gen(m_maincpu);
}


void skullxbo_state::scanline_update(screen_device &screen, int scanline)
{
	UINT16 *check = &m_alpha[(scanline / 8) * 64 + 42];

	/* check for interrupts in the alpha ram */
	/* the interrupt occurs on the HBLANK of the 6th scanline following */
	if (check < &m_alpha[0x7c0] && (*check & 0x8000))
	{
		int width = screen.width();
		attotime period = screen.time_until_pos(screen.vpos() + 6, width * 0.9);
		m_scanline_timer->adjust(period);
	}

	/* update the playfield and motion objects */
	skullxbo_scanline_update(scanline);
}


WRITE16_MEMBER(skullxbo_state::skullxbo_halt_until_hblank_0_w)
{
	halt_until_hblank_0(space.device(), *machine().primary_screen);
}


MACHINE_RESET_MEMBER(skullxbo_state,skullxbo)
{
	atarigen_state::machine_reset();
	scanline_timer_reset(*machine().primary_screen, 8);
}



/*************************************
 *
 *  Who knows what this is?
 *
 *************************************/

WRITE16_MEMBER(skullxbo_state::skullxbo_mobwr_w)
{
	logerror("MOBWR[%02X] = %04X\n", offset, data);
}



/*************************************
 *
 *  Main CPU memory handlers
 *
 *************************************/

static ADDRESS_MAP_START( main_map, AS_PROGRAM, 16, skullxbo_state )
	AM_RANGE(0x000000, 0x07ffff) AM_ROM
	AM_RANGE(0xff0000, 0xff07ff) AM_WRITE(skullxbo_mobmsb_w)
	AM_RANGE(0xff0800, 0xff0bff) AM_WRITE(skullxbo_halt_until_hblank_0_w)
	AM_RANGE(0xff0c00, 0xff0fff) AM_WRITE(eeprom_enable_w)
	AM_RANGE(0xff1000, 0xff13ff) AM_WRITE(video_int_ack_w)
	AM_RANGE(0xff1400, 0xff17ff) AM_DEVWRITE8("jsa", atari_jsa_ii_device, main_command_w, 0x00ff)
	AM_RANGE(0xff1800, 0xff1bff) AM_DEVWRITE("jsa", atari_jsa_ii_device, sound_reset_w)
	AM_RANGE(0xff1c00, 0xff1c7f) AM_WRITE(skullxbo_playfieldlatch_w)
	AM_RANGE(0xff1c80, 0xff1cff) AM_WRITE(skullxbo_xscroll_w) AM_SHARE("xscroll")
	AM_RANGE(0xff1d00, 0xff1d7f) AM_WRITE(scanline_int_ack_w)
	AM_RANGE(0xff1d80, 0xff1dff) AM_WRITE(watchdog_reset16_w)
	AM_RANGE(0xff1e00, 0xff1e7f) AM_WRITE(skullxbo_playfieldlatch_w)
	AM_RANGE(0xff1e80, 0xff1eff) AM_WRITE(skullxbo_xscroll_w)
	AM_RANGE(0xff1f00, 0xff1f7f) AM_WRITE(scanline_int_ack_w)
	AM_RANGE(0xff1f80, 0xff1fff) AM_WRITE(watchdog_reset16_w)
	AM_RANGE(0xff2000, 0xff2fff) AM_RAM_WRITE(paletteram_666_w) AM_SHARE("paletteram")
	AM_RANGE(0xff4000, 0xff47ff) AM_WRITE(skullxbo_yscroll_w) AM_SHARE("yscroll")
	AM_RANGE(0xff4800, 0xff4fff) AM_WRITE(skullxbo_mobwr_w)
	AM_RANGE(0xff6000, 0xff6fff) AM_WRITE(eeprom_w) AM_SHARE("eeprom")
	AM_RANGE(0xff5000, 0xff5001) AM_DEVREAD8("jsa", atari_jsa_ii_device, main_response_r, 0x00ff)
	AM_RANGE(0xff5800, 0xff5801) AM_READ_PORT("FF5800")
	AM_RANGE(0xff5802, 0xff5803) AM_READ_PORT("FF5802")
	AM_RANGE(0xff6000, 0xff6fff) AM_READ(eeprom_r)
	AM_RANGE(0xff8000, 0xff9fff) AM_RAM_WRITE(playfield_latched_lsb_w) AM_SHARE("playfield")
	AM_RANGE(0xffa000, 0xffbfff) AM_RAM_WRITE(playfield_upper_w) AM_SHARE("playfield_up")
	AM_RANGE(0xffc000, 0xffcf7f) AM_RAM_WRITE(alpha_w) AM_SHARE("alpha")
	AM_RANGE(0xffcf80, 0xffcfff) AM_READWRITE_LEGACY(atarimo_0_slipram_r, atarimo_0_slipram_w)
	AM_RANGE(0xffd000, 0xffdfff) AM_READWRITE_LEGACY(atarimo_0_spriteram_r, atarimo_0_spriteram_w)
	AM_RANGE(0xffe000, 0xffffff) AM_RAM
ADDRESS_MAP_END



/*************************************
 *
 *  Port definitions
 *
 *************************************/

static INPUT_PORTS_START( skullxbo )
	PORT_START("FF5800")
	PORT_BIT( 0x00ff, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0100, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(1)
	PORT_BIT( 0x0200, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(1)
	PORT_BIT( 0x0400, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0800, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x1000, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_PLAYER(1)
	PORT_BIT( 0x2000, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_PLAYER(1)
	PORT_BIT( 0x4000, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_PLAYER(1)
	PORT_BIT( 0x8000, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_PLAYER(1)

	PORT_START("FF5802")
	PORT_BIT( 0x000f, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0010, IP_ACTIVE_HIGH, IPT_CUSTOM ) PORT_HBLANK("screen")
	PORT_BIT( 0x0020, IP_ACTIVE_HIGH, IPT_CUSTOM ) PORT_VBLANK("screen")
	PORT_BIT( 0x0040, IP_ACTIVE_LOW, IPT_CUSTOM ) PORT_ATARI_JSA_MAIN_TO_SOUND_READY("jsa")  /* /AUDBUSY */
	PORT_SERVICE( 0x0080, IP_ACTIVE_LOW )
	PORT_BIT( 0x0100, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(2)
	PORT_BIT( 0x0200, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(2)
	PORT_BIT( 0x0400, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x0800, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x1000, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT ) PORT_PLAYER(2)
	PORT_BIT( 0x2000, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT ) PORT_PLAYER(2)
	PORT_BIT( 0x4000, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN ) PORT_PLAYER(2)
	PORT_BIT( 0x8000, IP_ACTIVE_LOW, IPT_JOYSTICK_UP ) PORT_PLAYER(2)
INPUT_PORTS_END



/*************************************
 *
 *  Graphics definitions
 *
 *************************************/

static const gfx_layout pflayout =
{
	16,8,
	RGN_FRAC(1,2),
	4,
	{ 0, 1, 2, 3 },
	{ RGN_FRAC(1,2)+0,RGN_FRAC(1,2)+0, RGN_FRAC(1,2)+4,RGN_FRAC(1,2)+4, 0,0, 4,4,
		RGN_FRAC(1,2)+8,RGN_FRAC(1,2)+8, RGN_FRAC(1,2)+12,RGN_FRAC(1,2)+12, 8,8, 12,12 },
	{ 0*8, 2*8, 4*8, 6*8, 8*8, 10*8, 12*8, 14*8 },
	16*8
};


static const gfx_layout anlayout =
{
	16,8,
	RGN_FRAC(1,1),
	2,
	{ 0, 1 },
	{ 0,0, 2,2, 4,4, 6,6, 8,8, 10,10, 12,12, 14,14 },
	{ 0*16, 1*16, 2*16, 3*16, 4*16, 5*16, 6*16, 7*16 },
	8*16
};


static const gfx_layout molayout =
{
	16,8,
	RGN_FRAC(1,5),
	5,
	{ RGN_FRAC(4,5), RGN_FRAC(3,5), RGN_FRAC(2,5), RGN_FRAC(1,5), RGN_FRAC(0,5) },
	{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 },
	{ 0*8, 2*8, 4*8, 6*8, 8*8, 10*8, 12*8, 14*8 },
	16*8
};


static GFXDECODE_START( skullxbo )
	GFXDECODE_ENTRY( "gfx1", 0, molayout, 0x000, 32 )
	GFXDECODE_ENTRY( "gfx2", 0, pflayout, 0x200, 16 )
	GFXDECODE_ENTRY( "gfx3", 0, anlayout, 0x300, 16 )
GFXDECODE_END



/*************************************
 *
 *  Machine driver
 *
 *************************************/

static MACHINE_CONFIG_START( skullxbo, skullxbo_state )

	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", M68000, ATARI_CLOCK_14MHz/2)
	MCFG_CPU_PROGRAM_MAP(main_map)
	MCFG_DEVICE_VBLANK_INT_DRIVER("screen", atarigen_state, video_int_gen)

	MCFG_TIMER_DRIVER_ADD("scan_timer", skullxbo_state, scanline_timer)
	MCFG_MACHINE_RESET_OVERRIDE(skullxbo_state,skullxbo)
	MCFG_NVRAM_ADD_1FILL("eeprom")

	/* video hardware */
	MCFG_VIDEO_ATTRIBUTES(VIDEO_UPDATE_BEFORE_VBLANK)
	MCFG_GFXDECODE(skullxbo)
	MCFG_PALETTE_LENGTH(2048)

	MCFG_SCREEN_ADD("screen", RASTER)
	/* note: these parameters are from published specs, not derived */
	/* the board uses an SOS-2 chip to generate video signals */
	MCFG_SCREEN_RAW_PARAMS(ATARI_CLOCK_14MHz, 456*2, 0, 336*2, 262, 0, 240)
	MCFG_SCREEN_UPDATE_DRIVER(skullxbo_state, screen_update_skullxbo)

	MCFG_VIDEO_START_OVERRIDE(skullxbo_state,skullxbo)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("mono")
	
	MCFG_ATARI_JSA_II_ADD("jsa", WRITELINE(atarigen_state, sound_int_write_line))
	MCFG_ATARI_JSA_TEST_PORT("FF5802", 7)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.0)
MACHINE_CONFIG_END



/*************************************
 *
 *  ROM definition(s)
 *
 *************************************/

ROM_START( skullxbo )
	ROM_REGION( 0x80000, "maincpu", 0 ) /* 8*64k for 68000 code */
	ROM_LOAD16_BYTE( "136072-5150.228a", 0x000000, 0x010000, CRC(9546d88b) SHA1(4b02c1c8a57377e651a719a0fe2a3532594f3e71) )
	ROM_LOAD16_BYTE( "136072-5151.228c", 0x000001, 0x010000, CRC(b9ed8bd4) SHA1(784a2fc8f5901d9e462966e3f7226ce3c6db980a) )
	ROM_LOAD16_BYTE( "136072-5152.213a", 0x020000, 0x010000, CRC(c07e44fc) SHA1(0aacb77be59c398c9eb5f01108957bbb17c9e5cd) )
	ROM_LOAD16_BYTE( "136072-5153.213c", 0x020001, 0x010000, CRC(fef8297f) SHA1(f62f378a957599ea38579a29df1f9e11335e8fb3) )
	ROM_LOAD16_BYTE( "136072-1154.200a", 0x040000, 0x010000, CRC(de4101a3) SHA1(21cf656fc0695a0ef31cfa6e686069d7d4965cce) )
	ROM_LOAD16_BYTE( "136072-1155.200c", 0x040001, 0x010000, CRC(78c0f6ad) SHA1(21ce2a83cd3350cd7ff53627a7e27599b9754a12) )
	ROM_LOAD16_BYTE( "136072-1156.185a", 0x070000, 0x008000, CRC(cde16b55) SHA1(bf5059f0f73a8819551fb3ded3cb6a3123841481) )
	ROM_LOAD16_BYTE( "136072-1157.185c", 0x070001, 0x008000, CRC(31c77376) SHA1(19eb54d73edb25fda6803df896e182d05c5bad7e) )

	ROM_REGION( 0x14000, "jsa:cpu", 0 ) /* 64k for 6502 code */
	ROM_LOAD( "136072-1149.1b",  0x010000, 0x004000, CRC(8d730e7a) SHA1(b89fb9cadcf813ea5cba55f1efcdcdd2517944c7) )
	ROM_CONTINUE(                0x004000, 0x00c000 )

	ROM_REGION( 0x190000, "gfx1", 0 )
	ROM_LOAD( "136072-1102.13r", 0x000000, 0x010000, CRC(90becdfa) SHA1(aa5aaeda70e137518a9e58906daed66fa563b27e) )
	ROM_LOAD( "136072-1104.28r", 0x010000, 0x010000, CRC(33609071) SHA1(6d9671a9edbdd28c1e360017253dab5dd858dbe7) )
	ROM_LOAD( "136072-1106.41r", 0x020000, 0x010000, CRC(71962e9f) SHA1(4e6017ede5ce2fec7f6e25eadfc4070f3296ff2f) )
	ROM_LOAD( "136072-1101.13p", 0x030000, 0x010000, CRC(4d41701e) SHA1(b34b392ba00e580cb719be2c1a40cfc0d2f177e3) )
	ROM_LOAD( "136072-1103.28p", 0x040000, 0x010000, CRC(3011da3b) SHA1(e7118b111e0de9b9a2dfe5a165f2140e90c919e7) )
	ROM_LOAD( "136072-1108.53r", 0x050000, 0x010000, CRC(386c7edc) SHA1(a2f61c7f8fb822433957565f373ab5cc8e0a2ba0) )
	ROM_LOAD( "136072-1110.67r", 0x060000, 0x010000, CRC(a54d16e6) SHA1(ad10623f0a87e5a92b8e8c611c2d374a8fd7047e) )
	ROM_LOAD( "136072-1112.81r", 0x070000, 0x010000, CRC(669411f6) SHA1(a0c678a75076b466f4a27c881642c44d47c9dea0) )
	ROM_LOAD( "136072-1107.53p", 0x080000, 0x010000, CRC(caaeb57a) SHA1(e20050d10ee18f52ac36616003de241aa9951eab) )
	ROM_LOAD( "136072-1109.67p", 0x090000, 0x010000, CRC(61cb4e28) SHA1(6d0cb4409fa4c9c696abd612395d5f6ddede6779) )
	ROM_LOAD( "136072-1114.95r", 0x0a0000, 0x010000, CRC(e340d5a1) SHA1(29a23ad2b6c8302508a8b49cfbc064fe86a8b908) )
	ROM_LOAD( "136072-1116.109r", 0x0b0000, 0x010000, CRC(f25b8aca) SHA1(c8c6d0951098c32e32b87d9717cc14bb91ac2017) )
	ROM_LOAD( "136072-1118.123r", 0x0c0000, 0x010000, CRC(8cf73585) SHA1(b1f3e44f6cd434ecfe4d88463b7e2e7b48d2bf1f) )
	ROM_LOAD( "136072-1113.95p", 0x0d0000, 0x010000, CRC(899b59af) SHA1(c7344e4bf57024415463cb50c788631fbad07132) )
	ROM_LOAD( "136072-1115.109p", 0x0e0000, 0x010000, CRC(cf4fd19a) SHA1(731fc7bb1dacacc6e2e4db1e096d07d5fe3d8b19) )
	ROM_LOAD( "136072-1120.137r", 0x0f0000, 0x010000, CRC(fde7c03d) SHA1(ec336af0f3314af134fd1a64c478be06249a550a) )
	ROM_LOAD( "136072-1122.151r", 0x100000, 0x010000, CRC(6ff6a9f2) SHA1(af8fda010c15e13e58e0f235b7b5a12d5a21fc0c) )
	ROM_LOAD( "136072-1124.165r", 0x110000, 0x010000, CRC(f11909f1) SHA1(2dbd5c92e8329f5b5033b3633bd56618eb7da875) )
	ROM_LOAD( "136072-1119.137p", 0x120000, 0x010000, CRC(6f8003a1) SHA1(df8494ce56213dddd11f1365c03bb77ebf3e6eba) )
	ROM_LOAD( "136072-1121.151p", 0x130000, 0x010000, CRC(8ff0a1ec) SHA1(8df33657a035316a1e4ce7d7b33af6e51b990c48) )
	ROM_LOAD( "136072-1125.123n", 0x140000, 0x010000, CRC(3aa7c756) SHA1(c1585733cef28fdf031e335503364846cfd0384a) )
	ROM_LOAD( "136072-1126.137n", 0x150000, 0x010000, CRC(cb82c9aa) SHA1(1469d2825e6d366a8e3f74294b0a64f2a63384aa) )
	ROM_LOAD( "136072-1128.151n", 0x160000, 0x010000, CRC(dce32863) SHA1(b0476de8d54dcf163a723b1fc805da4e1ca11c27) )
	/* 170000-18ffff empty */

	ROM_REGION( 0x0a0000, "gfx2", ROMREGION_INVERT )
	ROM_LOAD( "136072-2129.180p", 0x000000, 0x010000, CRC(36b1a578) SHA1(ded9cccd1009e517662387353333f20031abddd5) )
	ROM_LOAD( "136072-2131.193p", 0x010000, 0x010000, CRC(7b7c04a1) SHA1(b57f3f35f39ecf912daf2611919f31a92005f07b) )
	ROM_LOAD( "136072-2133.208p", 0x020000, 0x010000, CRC(e03fe4d9) SHA1(d9a9174a2d2e2d83f7c32177f0dbab74f3875d2e) )
	ROM_LOAD( "136072-2135.221p", 0x030000, 0x010000, CRC(7d497110) SHA1(4d5ce6673a112486e9dff77c901d90105aa0a210) )
	ROM_LOAD( "136072-2137.235p", 0x040000, 0x010000, CRC(f91e7872) SHA1(690715a92e8ca1b1d22fff85f9ed3f1e02edca99) )
	ROM_LOAD( "136072-2130.180r", 0x050000, 0x010000, CRC(b25368cc) SHA1(110e6882399e200b3a4cdd823cc5b0565183cfeb) )
	ROM_LOAD( "136072-2132.193r", 0x060000, 0x010000, CRC(112f2d20) SHA1(3acd43cf73f1be10c17a717c8990f5c656935b3a) )
	ROM_LOAD( "136072-2134.208r", 0x070000, 0x010000, CRC(84884ed6) SHA1(6129f090a4e5b8f65895086e1731b13ee72b6079) )
	ROM_LOAD( "136072-2136.221r", 0x080000, 0x010000, CRC(bc028690) SHA1(e75a961febf1f1a6c4981301b73bab82c3d19785) )
	ROM_LOAD( "136072-2138.235r", 0x090000, 0x010000, CRC(60cec955) SHA1(d184746589130a8f10fcc6c79484578bd08757f0) )

	ROM_REGION( 0x008000, "gfx3", 0 )
	ROM_LOAD( "136072-2141.250k", 0x000000, 0x008000, CRC(60d6d6df) SHA1(a8d56092f014a0a93742c701effaec86c75772e1) )

	ROM_REGION( 0x40000, "jsa:oki1", 0 )   /* 256k for ADPCM samples */
	ROM_LOAD( "136072-1145.7k",  0x000000, 0x010000, CRC(d9475d58) SHA1(5a4a0094c83f5d0e26f0c35feb0b1f15a5f5c3f9) )
	ROM_LOAD( "136072-1146.7j",  0x010000, 0x010000, CRC(133e6aef) SHA1(e393d0b246889779029443a19e3859d45cb900db) )
	ROM_LOAD( "136072-1147.7e",  0x020000, 0x010000, CRC(ba4d556e) SHA1(af4364f2c456abc20f1742c945a3acfeb5e192c4) )
	ROM_LOAD( "136072-1148.7d",  0x030000, 0x010000, CRC(c48df49a) SHA1(c92fde9be1a1ab09453c57eefb0dcfe49e538d07) )
ROM_END


ROM_START( skullxbo4 )
	ROM_REGION( 0x80000, "maincpu", 0 ) /* 8*64k for 68000 code */
	ROM_LOAD16_BYTE( "136072-4150.228a", 0x000000, 0x010000, CRC(607fc73b) SHA1(e6ebaf1a7570df1d12becae217becdd0a60d6aca) )
	ROM_LOAD16_BYTE( "136072-4151.228c", 0x000001, 0x010000, CRC(76bbf619) SHA1(2cbd61f414684587c0e634c223c758b0a28aafc0) )
	ROM_LOAD16_BYTE( "136072-4152.213a", 0x020000, 0x010000, CRC(095206f5) SHA1(c468b908b7a6cb83a4a91e3999494531511eee2b) )
	ROM_LOAD16_BYTE( "136072-4153.213c", 0x020001, 0x010000, CRC(e44be9d3) SHA1(f1637c3512f99a43f62833319194d18c2a56e581) )
	ROM_LOAD16_BYTE( "136072-1154.200a", 0x040000, 0x010000, CRC(de4101a3) SHA1(21cf656fc0695a0ef31cfa6e686069d7d4965cce) )
	ROM_LOAD16_BYTE( "136072-1155.200c", 0x040001, 0x010000, CRC(78c0f6ad) SHA1(21ce2a83cd3350cd7ff53627a7e27599b9754a12) )
	ROM_LOAD16_BYTE( "136072-1156.185a", 0x070000, 0x008000, CRC(cde16b55) SHA1(bf5059f0f73a8819551fb3ded3cb6a3123841481) )
	ROM_LOAD16_BYTE( "136072-1157.185c", 0x070001, 0x008000, CRC(31c77376) SHA1(19eb54d73edb25fda6803df896e182d05c5bad7e) )

	ROM_REGION( 0x14000, "jsa:cpu", 0 ) /* 64k for 6502 code */
	ROM_LOAD( "136072-1149.1b",  0x010000, 0x004000, CRC(8d730e7a) SHA1(b89fb9cadcf813ea5cba55f1efcdcdd2517944c7) )
	ROM_CONTINUE(                0x004000, 0x00c000 )

	ROM_REGION( 0x190000, "gfx1", 0 )
	ROM_LOAD( "136072-1102.13r", 0x000000, 0x010000, CRC(90becdfa) SHA1(aa5aaeda70e137518a9e58906daed66fa563b27e) )
	ROM_LOAD( "136072-1104.28r", 0x010000, 0x010000, CRC(33609071) SHA1(6d9671a9edbdd28c1e360017253dab5dd858dbe7) )
	ROM_LOAD( "136072-1106.41r", 0x020000, 0x010000, CRC(71962e9f) SHA1(4e6017ede5ce2fec7f6e25eadfc4070f3296ff2f) )
	ROM_LOAD( "136072-1101.13p", 0x030000, 0x010000, CRC(4d41701e) SHA1(b34b392ba00e580cb719be2c1a40cfc0d2f177e3) )
	ROM_LOAD( "136072-1103.28p", 0x040000, 0x010000, CRC(3011da3b) SHA1(e7118b111e0de9b9a2dfe5a165f2140e90c919e7) )
	ROM_LOAD( "136072-1108.53r", 0x050000, 0x010000, CRC(386c7edc) SHA1(a2f61c7f8fb822433957565f373ab5cc8e0a2ba0) )
	ROM_LOAD( "136072-1110.67r", 0x060000, 0x010000, CRC(a54d16e6) SHA1(ad10623f0a87e5a92b8e8c611c2d374a8fd7047e) )
	ROM_LOAD( "136072-1112.81r", 0x070000, 0x010000, CRC(669411f6) SHA1(a0c678a75076b466f4a27c881642c44d47c9dea0) )
	ROM_LOAD( "136072-1107.53p", 0x080000, 0x010000, CRC(caaeb57a) SHA1(e20050d10ee18f52ac36616003de241aa9951eab) )
	ROM_LOAD( "136072-1109.67p", 0x090000, 0x010000, CRC(61cb4e28) SHA1(6d0cb4409fa4c9c696abd612395d5f6ddede6779) )
	ROM_LOAD( "136072-1114.95r", 0x0a0000, 0x010000, CRC(e340d5a1) SHA1(29a23ad2b6c8302508a8b49cfbc064fe86a8b908) )
	ROM_LOAD( "136072-1116.109r", 0x0b0000, 0x010000, CRC(f25b8aca) SHA1(c8c6d0951098c32e32b87d9717cc14bb91ac2017) )
	ROM_LOAD( "136072-1118.123r", 0x0c0000, 0x010000, CRC(8cf73585) SHA1(b1f3e44f6cd434ecfe4d88463b7e2e7b48d2bf1f) )
	ROM_LOAD( "136072-1113.95p", 0x0d0000, 0x010000, CRC(899b59af) SHA1(c7344e4bf57024415463cb50c788631fbad07132) )
	ROM_LOAD( "136072-1115.109p", 0x0e0000, 0x010000, CRC(cf4fd19a) SHA1(731fc7bb1dacacc6e2e4db1e096d07d5fe3d8b19) )
	ROM_LOAD( "136072-1120.137r", 0x0f0000, 0x010000, CRC(fde7c03d) SHA1(ec336af0f3314af134fd1a64c478be06249a550a) )
	ROM_LOAD( "136072-1122.151r", 0x100000, 0x010000, CRC(6ff6a9f2) SHA1(af8fda010c15e13e58e0f235b7b5a12d5a21fc0c) )
	ROM_LOAD( "136072-1124.165r", 0x110000, 0x010000, CRC(f11909f1) SHA1(2dbd5c92e8329f5b5033b3633bd56618eb7da875) )
	ROM_LOAD( "136072-1119.137p", 0x120000, 0x010000, CRC(6f8003a1) SHA1(df8494ce56213dddd11f1365c03bb77ebf3e6eba) )
	ROM_LOAD( "136072-1121.151p", 0x130000, 0x010000, CRC(8ff0a1ec) SHA1(8df33657a035316a1e4ce7d7b33af6e51b990c48) )
	ROM_LOAD( "136072-1125.123n", 0x140000, 0x010000, CRC(3aa7c756) SHA1(c1585733cef28fdf031e335503364846cfd0384a) )
	ROM_LOAD( "136072-1126.137n", 0x150000, 0x010000, CRC(cb82c9aa) SHA1(1469d2825e6d366a8e3f74294b0a64f2a63384aa) )
	ROM_LOAD( "136072-1128.151n", 0x160000, 0x010000, CRC(dce32863) SHA1(b0476de8d54dcf163a723b1fc805da4e1ca11c27) )
	/* 170000-18ffff empty */

	ROM_REGION( 0x0a0000, "gfx2", ROMREGION_INVERT )
	ROM_LOAD( "136072-2129.180p", 0x000000, 0x010000, CRC(36b1a578) SHA1(ded9cccd1009e517662387353333f20031abddd5) )
	ROM_LOAD( "136072-2131.193p", 0x010000, 0x010000, CRC(7b7c04a1) SHA1(b57f3f35f39ecf912daf2611919f31a92005f07b) )
	ROM_LOAD( "136072-2133.208p", 0x020000, 0x010000, CRC(e03fe4d9) SHA1(d9a9174a2d2e2d83f7c32177f0dbab74f3875d2e) )
	ROM_LOAD( "136072-2135.221p", 0x030000, 0x010000, CRC(7d497110) SHA1(4d5ce6673a112486e9dff77c901d90105aa0a210) )
	ROM_LOAD( "136072-2137.235p", 0x040000, 0x010000, CRC(f91e7872) SHA1(690715a92e8ca1b1d22fff85f9ed3f1e02edca99) )
	ROM_LOAD( "136072-2130.180r", 0x050000, 0x010000, CRC(b25368cc) SHA1(110e6882399e200b3a4cdd823cc5b0565183cfeb) )
	ROM_LOAD( "136072-2132.193r", 0x060000, 0x010000, CRC(112f2d20) SHA1(3acd43cf73f1be10c17a717c8990f5c656935b3a) )
	ROM_LOAD( "136072-2134.208r", 0x070000, 0x010000, CRC(84884ed6) SHA1(6129f090a4e5b8f65895086e1731b13ee72b6079) )
	ROM_LOAD( "136072-2136.221r", 0x080000, 0x010000, CRC(bc028690) SHA1(e75a961febf1f1a6c4981301b73bab82c3d19785) )
	ROM_LOAD( "136072-2138.235r", 0x090000, 0x010000, CRC(60cec955) SHA1(d184746589130a8f10fcc6c79484578bd08757f0) )

	ROM_REGION( 0x008000, "gfx3", 0 )
	ROM_LOAD( "136072-2141.250k", 0x000000, 0x008000, CRC(60d6d6df) SHA1(a8d56092f014a0a93742c701effaec86c75772e1) )

	ROM_REGION( 0x40000, "jsa:oki1", 0 )   /* 256k for ADPCM samples */
	ROM_LOAD( "136072-1145.7k",  0x000000, 0x010000, CRC(d9475d58) SHA1(5a4a0094c83f5d0e26f0c35feb0b1f15a5f5c3f9) )
	ROM_LOAD( "136072-1146.7j",  0x010000, 0x010000, CRC(133e6aef) SHA1(e393d0b246889779029443a19e3859d45cb900db) )
	ROM_LOAD( "136072-1147.7e",  0x020000, 0x010000, CRC(ba4d556e) SHA1(af4364f2c456abc20f1742c945a3acfeb5e192c4) )
	ROM_LOAD( "136072-1148.7d",  0x030000, 0x010000, CRC(c48df49a) SHA1(c92fde9be1a1ab09453c57eefb0dcfe49e538d07) )
ROM_END


ROM_START( skullxbo3 )
	ROM_REGION( 0x80000, "maincpu", 0 ) /* 8*64k for 68000 code */
	ROM_LOAD16_BYTE( "136072-3150.228a", 0x000000, 0x010000, CRC(47083d59) SHA1(a713231c22a3c2de09af65aa2bae17ea41f10cf0) )
	ROM_LOAD16_BYTE( "136072-3151.228c", 0x000001, 0x010000, CRC(2c03feaf) SHA1(e7ad1568e3008386f520ed3ba90aefbfc9417a64) )
	ROM_LOAD16_BYTE( "136072-3152.213a", 0x020000, 0x010000, CRC(aa0471de) SHA1(c0bf12304b9d64595f2fc40c2fb67f6ccb9d3b8f) )
	ROM_LOAD16_BYTE( "136072-3153.213c", 0x020001, 0x010000, CRC(a65386f9) SHA1(3bc7d0bc844cd1f9efa1a5f6abccfbe3ab7c2bef) )
	ROM_LOAD16_BYTE( "136072-1154.200a", 0x040000, 0x010000, CRC(de4101a3) SHA1(21cf656fc0695a0ef31cfa6e686069d7d4965cce) )
	ROM_LOAD16_BYTE( "136072-1155.200c", 0x040001, 0x010000, CRC(78c0f6ad) SHA1(21ce2a83cd3350cd7ff53627a7e27599b9754a12) )
	ROM_LOAD16_BYTE( "136072-1156.185a", 0x070000, 0x008000, CRC(cde16b55) SHA1(bf5059f0f73a8819551fb3ded3cb6a3123841481) )
	ROM_LOAD16_BYTE( "136072-1157.185c", 0x070001, 0x008000, CRC(31c77376) SHA1(19eb54d73edb25fda6803df896e182d05c5bad7e) )

	ROM_REGION( 0x14000, "jsa:cpu", 0 ) /* 64k for 6502 code */
	ROM_LOAD( "136072-1149.1b",  0x010000, 0x004000, CRC(8d730e7a) SHA1(b89fb9cadcf813ea5cba55f1efcdcdd2517944c7) )
	ROM_CONTINUE(                0x004000, 0x00c000 )

	ROM_REGION( 0x190000, "gfx1", 0 )
	ROM_LOAD( "136072-1102.13r", 0x000000, 0x010000, CRC(90becdfa) SHA1(aa5aaeda70e137518a9e58906daed66fa563b27e) )
	ROM_LOAD( "136072-1104.28r", 0x010000, 0x010000, CRC(33609071) SHA1(6d9671a9edbdd28c1e360017253dab5dd858dbe7) )
	ROM_LOAD( "136072-1106.41r", 0x020000, 0x010000, CRC(71962e9f) SHA1(4e6017ede5ce2fec7f6e25eadfc4070f3296ff2f) )
	ROM_LOAD( "136072-1101.13p", 0x030000, 0x010000, CRC(4d41701e) SHA1(b34b392ba00e580cb719be2c1a40cfc0d2f177e3) )
	ROM_LOAD( "136072-1103.28p", 0x040000, 0x010000, CRC(3011da3b) SHA1(e7118b111e0de9b9a2dfe5a165f2140e90c919e7) )
	ROM_LOAD( "136072-1108.53r", 0x050000, 0x010000, CRC(386c7edc) SHA1(a2f61c7f8fb822433957565f373ab5cc8e0a2ba0) )
	ROM_LOAD( "136072-1110.67r", 0x060000, 0x010000, CRC(a54d16e6) SHA1(ad10623f0a87e5a92b8e8c611c2d374a8fd7047e) )
	ROM_LOAD( "136072-1112.81r", 0x070000, 0x010000, CRC(669411f6) SHA1(a0c678a75076b466f4a27c881642c44d47c9dea0) )
	ROM_LOAD( "136072-1107.53p", 0x080000, 0x010000, CRC(caaeb57a) SHA1(e20050d10ee18f52ac36616003de241aa9951eab) )
	ROM_LOAD( "136072-1109.67p", 0x090000, 0x010000, CRC(61cb4e28) SHA1(6d0cb4409fa4c9c696abd612395d5f6ddede6779) )
	ROM_LOAD( "136072-1114.95r", 0x0a0000, 0x010000, CRC(e340d5a1) SHA1(29a23ad2b6c8302508a8b49cfbc064fe86a8b908) )
	ROM_LOAD( "136072-1116.109r", 0x0b0000, 0x010000, CRC(f25b8aca) SHA1(c8c6d0951098c32e32b87d9717cc14bb91ac2017) )
	ROM_LOAD( "136072-1118.123r", 0x0c0000, 0x010000, CRC(8cf73585) SHA1(b1f3e44f6cd434ecfe4d88463b7e2e7b48d2bf1f) )
	ROM_LOAD( "136072-1113.95p",  0x0d0000, 0x010000, CRC(899b59af) SHA1(c7344e4bf57024415463cb50c788631fbad07132) )
	ROM_LOAD( "136072-1115.109p", 0x0e0000, 0x010000, CRC(cf4fd19a) SHA1(731fc7bb1dacacc6e2e4db1e096d07d5fe3d8b19) )
	ROM_LOAD( "136072-1120.137r", 0x0f0000, 0x010000, CRC(fde7c03d) SHA1(ec336af0f3314af134fd1a64c478be06249a550a) )
	ROM_LOAD( "136072-1122.151r", 0x100000, 0x010000, CRC(6ff6a9f2) SHA1(af8fda010c15e13e58e0f235b7b5a12d5a21fc0c) )
	ROM_LOAD( "136072-1124.165r", 0x110000, 0x010000, CRC(f11909f1) SHA1(2dbd5c92e8329f5b5033b3633bd56618eb7da875) )
	ROM_LOAD( "136072-1119.137p", 0x120000, 0x010000, CRC(6f8003a1) SHA1(df8494ce56213dddd11f1365c03bb77ebf3e6eba) )
	ROM_LOAD( "136072-1121.151p", 0x130000, 0x010000, CRC(8ff0a1ec) SHA1(8df33657a035316a1e4ce7d7b33af6e51b990c48) )
	ROM_LOAD( "136072-1125.123n", 0x140000, 0x010000, CRC(3aa7c756) SHA1(c1585733cef28fdf031e335503364846cfd0384a) )
	ROM_LOAD( "136072-1126.137n", 0x150000, 0x010000, CRC(cb82c9aa) SHA1(1469d2825e6d366a8e3f74294b0a64f2a63384aa) )
	ROM_LOAD( "136072-1128.151n", 0x160000, 0x010000, CRC(dce32863) SHA1(b0476de8d54dcf163a723b1fc805da4e1ca11c27) )
	/* 170000-18ffff empty */

	ROM_REGION( 0x0a0000, "gfx2", ROMREGION_INVERT )
	ROM_LOAD( "136072-2129.180p", 0x000000, 0x010000, CRC(36b1a578) SHA1(ded9cccd1009e517662387353333f20031abddd5) )
	ROM_LOAD( "136072-2131.193p", 0x010000, 0x010000, CRC(7b7c04a1) SHA1(b57f3f35f39ecf912daf2611919f31a92005f07b) )
	ROM_LOAD( "136072-2133.208p", 0x020000, 0x010000, CRC(e03fe4d9) SHA1(d9a9174a2d2e2d83f7c32177f0dbab74f3875d2e) )
	ROM_LOAD( "136072-2135.221p", 0x030000, 0x010000, CRC(7d497110) SHA1(4d5ce6673a112486e9dff77c901d90105aa0a210) )
	ROM_LOAD( "136072-2137.235p", 0x040000, 0x010000, CRC(f91e7872) SHA1(690715a92e8ca1b1d22fff85f9ed3f1e02edca99) )
	ROM_LOAD( "136072-2130.180r", 0x050000, 0x010000, CRC(b25368cc) SHA1(110e6882399e200b3a4cdd823cc5b0565183cfeb) )
	ROM_LOAD( "136072-2132.193r", 0x060000, 0x010000, CRC(112f2d20) SHA1(3acd43cf73f1be10c17a717c8990f5c656935b3a) )
	ROM_LOAD( "136072-2134.208r", 0x070000, 0x010000, CRC(84884ed6) SHA1(6129f090a4e5b8f65895086e1731b13ee72b6079) )
	ROM_LOAD( "136072-2136.221r", 0x080000, 0x010000, CRC(bc028690) SHA1(e75a961febf1f1a6c4981301b73bab82c3d19785) )
	ROM_LOAD( "136072-2138.235r", 0x090000, 0x010000, CRC(60cec955) SHA1(d184746589130a8f10fcc6c79484578bd08757f0) )

	ROM_REGION( 0x008000, "gfx3", 0 )
	ROM_LOAD( "136072-2141.250k", 0x000000, 0x008000, CRC(60d6d6df) SHA1(a8d56092f014a0a93742c701effaec86c75772e1) )

	ROM_REGION( 0x40000, "jsa:oki1", 0 )   /* 256k for ADPCM samples */
	ROM_LOAD( "136072-1145.7k",  0x000000, 0x010000, CRC(d9475d58) SHA1(5a4a0094c83f5d0e26f0c35feb0b1f15a5f5c3f9) )
	ROM_LOAD( "136072-1146.7j",  0x010000, 0x010000, CRC(133e6aef) SHA1(e393d0b246889779029443a19e3859d45cb900db) )
	ROM_LOAD( "136072-1147.7e",  0x020000, 0x010000, CRC(ba4d556e) SHA1(af4364f2c456abc20f1742c945a3acfeb5e192c4) )
	ROM_LOAD( "136072-1148.7d",  0x030000, 0x010000, CRC(c48df49a) SHA1(c92fde9be1a1ab09453c57eefb0dcfe49e538d07) )
ROM_END


ROM_START( skullxbo2 )
	ROM_REGION( 0x80000, "maincpu", 0 ) /* 8*64k for 68000 code */
	ROM_LOAD16_BYTE( "136072-2150.228a", 0x000000, 0x010000, CRC(8614f9ef) SHA1(981ba6fad7aa7002c3a5aa0d4dd859e664ca0fdb) )
	ROM_LOAD16_BYTE( "136072-2151.228c", 0x000001, 0x010000, CRC(47090acb) SHA1(12c47d6112bec88aaf25d10ba2d5335b6b474fb7) )
	ROM_LOAD16_BYTE( "136072-2152.213a", 0x020000, 0x010000, CRC(02f6a944) SHA1(24c2326c8b175fd03c5cf44f091365a860dbb9c9) )
	ROM_LOAD16_BYTE( "136072-2153.213c", 0x020001, 0x010000, CRC(589ce449) SHA1(fdc3d2ba30390848d8728b4256bf06af470de6a7) )
	ROM_LOAD16_BYTE( "136072-1154.200a", 0x040000, 0x010000, CRC(de4101a3) SHA1(21cf656fc0695a0ef31cfa6e686069d7d4965cce) )
	ROM_LOAD16_BYTE( "136072-1155.200c", 0x040001, 0x010000, CRC(78c0f6ad) SHA1(21ce2a83cd3350cd7ff53627a7e27599b9754a12) )
	ROM_LOAD16_BYTE( "136072-1156.185a", 0x070000, 0x008000, CRC(cde16b55) SHA1(bf5059f0f73a8819551fb3ded3cb6a3123841481) )
	ROM_LOAD16_BYTE( "136072-1157.185c", 0x070001, 0x008000, CRC(31c77376) SHA1(19eb54d73edb25fda6803df896e182d05c5bad7e) )

	ROM_REGION( 0x14000, "jsa:cpu", 0 ) /* 64k for 6502 code */
	ROM_LOAD( "136072-1149.1b",  0x010000, 0x004000, CRC(8d730e7a) SHA1(b89fb9cadcf813ea5cba55f1efcdcdd2517944c7) )
	ROM_CONTINUE(                0x004000, 0x00c000 )

	ROM_REGION( 0x190000, "gfx1", 0 )
	ROM_LOAD( "136072-1102.13r", 0x000000, 0x010000, CRC(90becdfa) SHA1(aa5aaeda70e137518a9e58906daed66fa563b27e) )
	ROM_LOAD( "136072-1104.28r", 0x010000, 0x010000, CRC(33609071) SHA1(6d9671a9edbdd28c1e360017253dab5dd858dbe7) )
	ROM_LOAD( "136072-1106.41r", 0x020000, 0x010000, CRC(71962e9f) SHA1(4e6017ede5ce2fec7f6e25eadfc4070f3296ff2f) )
	ROM_LOAD( "136072-1101.13p", 0x030000, 0x010000, CRC(4d41701e) SHA1(b34b392ba00e580cb719be2c1a40cfc0d2f177e3) )
	ROM_LOAD( "136072-1103.28p", 0x040000, 0x010000, CRC(3011da3b) SHA1(e7118b111e0de9b9a2dfe5a165f2140e90c919e7) )
	ROM_LOAD( "136072-1108.53r", 0x050000, 0x010000, CRC(386c7edc) SHA1(a2f61c7f8fb822433957565f373ab5cc8e0a2ba0) )
	ROM_LOAD( "136072-1110.67r", 0x060000, 0x010000, CRC(a54d16e6) SHA1(ad10623f0a87e5a92b8e8c611c2d374a8fd7047e) )
	ROM_LOAD( "136072-1112.81r", 0x070000, 0x010000, CRC(669411f6) SHA1(a0c678a75076b466f4a27c881642c44d47c9dea0) )
	ROM_LOAD( "136072-1107.53p", 0x080000, 0x010000, CRC(caaeb57a) SHA1(e20050d10ee18f52ac36616003de241aa9951eab) )
	ROM_LOAD( "136072-1109.67p", 0x090000, 0x010000, CRC(61cb4e28) SHA1(6d0cb4409fa4c9c696abd612395d5f6ddede6779) )
	ROM_LOAD( "136072-1114.95r", 0x0a0000, 0x010000, CRC(e340d5a1) SHA1(29a23ad2b6c8302508a8b49cfbc064fe86a8b908) )
	ROM_LOAD( "136072-1116.109r", 0x0b0000, 0x010000, CRC(f25b8aca) SHA1(c8c6d0951098c32e32b87d9717cc14bb91ac2017) )
	ROM_LOAD( "136072-1118.123r", 0x0c0000, 0x010000, CRC(8cf73585) SHA1(b1f3e44f6cd434ecfe4d88463b7e2e7b48d2bf1f) )
	ROM_LOAD( "136072-1113.95p", 0x0d0000, 0x010000, CRC(899b59af) SHA1(c7344e4bf57024415463cb50c788631fbad07132) )
	ROM_LOAD( "136072-1115.109p", 0x0e0000, 0x010000, CRC(cf4fd19a) SHA1(731fc7bb1dacacc6e2e4db1e096d07d5fe3d8b19) )
	ROM_LOAD( "136072-1120.137r", 0x0f0000, 0x010000, CRC(fde7c03d) SHA1(ec336af0f3314af134fd1a64c478be06249a550a) )
	ROM_LOAD( "136072-1122.151r", 0x100000, 0x010000, CRC(6ff6a9f2) SHA1(af8fda010c15e13e58e0f235b7b5a12d5a21fc0c) )
	ROM_LOAD( "136072-1124.165r", 0x110000, 0x010000, CRC(f11909f1) SHA1(2dbd5c92e8329f5b5033b3633bd56618eb7da875) )
	ROM_LOAD( "136072-1119.137p", 0x120000, 0x010000, CRC(6f8003a1) SHA1(df8494ce56213dddd11f1365c03bb77ebf3e6eba) )
	ROM_LOAD( "136072-1121.151p", 0x130000, 0x010000, CRC(8ff0a1ec) SHA1(8df33657a035316a1e4ce7d7b33af6e51b990c48) )
	ROM_LOAD( "136072-1125.123n", 0x140000, 0x010000, CRC(3aa7c756) SHA1(c1585733cef28fdf031e335503364846cfd0384a) )
	ROM_LOAD( "136072-1126.137n", 0x150000, 0x010000, CRC(cb82c9aa) SHA1(1469d2825e6d366a8e3f74294b0a64f2a63384aa) )
	ROM_LOAD( "136072-1128.151n", 0x160000, 0x010000, CRC(dce32863) SHA1(b0476de8d54dcf163a723b1fc805da4e1ca11c27) )
	/* 170000-18ffff empty */

	ROM_REGION( 0x0a0000, "gfx2", ROMREGION_INVERT )
	ROM_LOAD( "136072-2129.180p", 0x000000, 0x010000, CRC(36b1a578) SHA1(ded9cccd1009e517662387353333f20031abddd5) )
	ROM_LOAD( "136072-2131.193p", 0x010000, 0x010000, CRC(7b7c04a1) SHA1(b57f3f35f39ecf912daf2611919f31a92005f07b) )
	ROM_LOAD( "136072-2133.208p", 0x020000, 0x010000, CRC(e03fe4d9) SHA1(d9a9174a2d2e2d83f7c32177f0dbab74f3875d2e) )
	ROM_LOAD( "136072-2135.221p", 0x030000, 0x010000, CRC(7d497110) SHA1(4d5ce6673a112486e9dff77c901d90105aa0a210) )
	ROM_LOAD( "136072-2137.235p", 0x040000, 0x010000, CRC(f91e7872) SHA1(690715a92e8ca1b1d22fff85f9ed3f1e02edca99) )
	ROM_LOAD( "136072-2130.180r", 0x050000, 0x010000, CRC(b25368cc) SHA1(110e6882399e200b3a4cdd823cc5b0565183cfeb) )
	ROM_LOAD( "136072-2132.193r", 0x060000, 0x010000, CRC(112f2d20) SHA1(3acd43cf73f1be10c17a717c8990f5c656935b3a) )
	ROM_LOAD( "136072-2134.208r", 0x070000, 0x010000, CRC(84884ed6) SHA1(6129f090a4e5b8f65895086e1731b13ee72b6079) )
	ROM_LOAD( "136072-2136.221r", 0x080000, 0x010000, CRC(bc028690) SHA1(e75a961febf1f1a6c4981301b73bab82c3d19785) )
	ROM_LOAD( "136072-2138.235r", 0x090000, 0x010000, CRC(60cec955) SHA1(d184746589130a8f10fcc6c79484578bd08757f0) )

	ROM_REGION( 0x008000, "gfx3", 0 )
	ROM_LOAD( "136072-2141.250k", 0x000000, 0x008000, CRC(60d6d6df) SHA1(a8d56092f014a0a93742c701effaec86c75772e1) )

	ROM_REGION( 0x40000, "jsa:oki1", 0 )   /* 256k for ADPCM samples */
	ROM_LOAD( "136072-1145.7k",  0x000000, 0x010000, CRC(d9475d58) SHA1(5a4a0094c83f5d0e26f0c35feb0b1f15a5f5c3f9) )
	ROM_LOAD( "136072-1146.7j",  0x010000, 0x010000, CRC(133e6aef) SHA1(e393d0b246889779029443a19e3859d45cb900db) )
	ROM_LOAD( "136072-1147.7e",  0x020000, 0x010000, CRC(ba4d556e) SHA1(af4364f2c456abc20f1742c945a3acfeb5e192c4) )
	ROM_LOAD( "136072-1148.7d",  0x030000, 0x010000, CRC(c48df49a) SHA1(c92fde9be1a1ab09453c57eefb0dcfe49e538d07) )
ROM_END


ROM_START( skullxbo1 )
	ROM_REGION( 0x80000, "maincpu", 0 ) /* 8*64k for 68000 code */
	ROM_LOAD16_BYTE( "136072-1150.228a", 0x000000, 0x010000, CRC(376bb0c7) SHA1(195c8411f3ea9681e9ba6661a55418c194324339) )
	ROM_LOAD16_BYTE( "136072-1151.228c", 0x000001, 0x010000, CRC(858382f7) SHA1(6e18183962c36bf2599cf04b7dc824e840a94343) )
	ROM_LOAD16_BYTE( "136072-1152.213a", 0x020000, 0x010000, CRC(dc5b2008) SHA1(03a343d3cfd7c86b778ea9d568babc72dee23e1f) )
	ROM_LOAD16_BYTE( "136072-1153.213c", 0x020001, 0x010000, CRC(e5266c7c) SHA1(c8facce7fcb9777b157e515ca62b1e5f2be8b2ab) )
	ROM_LOAD16_BYTE( "136072-1154.200a", 0x040000, 0x010000, CRC(de4101a3) SHA1(21cf656fc0695a0ef31cfa6e686069d7d4965cce) )
	ROM_LOAD16_BYTE( "136072-1155.200c", 0x040001, 0x010000, CRC(78c0f6ad) SHA1(21ce2a83cd3350cd7ff53627a7e27599b9754a12) )
	ROM_LOAD16_BYTE( "136072-1156.185a", 0x070000, 0x008000, CRC(cde16b55) SHA1(bf5059f0f73a8819551fb3ded3cb6a3123841481) )
	ROM_LOAD16_BYTE( "136072-1157.185c", 0x070001, 0x008000, CRC(31c77376) SHA1(19eb54d73edb25fda6803df896e182d05c5bad7e) )

	ROM_REGION( 0x14000, "jsa:cpu", 0 ) /* 64k for 6502 code */
	ROM_LOAD( "136072-1149.1b",  0x010000, 0x004000, CRC(8d730e7a) SHA1(b89fb9cadcf813ea5cba55f1efcdcdd2517944c7) )
	ROM_CONTINUE(                0x004000, 0x00c000 )

	ROM_REGION( 0x190000, "gfx1", 0 )
	ROM_LOAD( "136072-1102.13r", 0x000000, 0x010000, CRC(90becdfa) SHA1(aa5aaeda70e137518a9e58906daed66fa563b27e) )
	ROM_LOAD( "136072-1104.28r", 0x010000, 0x010000, CRC(33609071) SHA1(6d9671a9edbdd28c1e360017253dab5dd858dbe7) )
	ROM_LOAD( "136072-1106.41r", 0x020000, 0x010000, CRC(71962e9f) SHA1(4e6017ede5ce2fec7f6e25eadfc4070f3296ff2f) )
	ROM_LOAD( "136072-1101.13p", 0x030000, 0x010000, CRC(4d41701e) SHA1(b34b392ba00e580cb719be2c1a40cfc0d2f177e3) )
	ROM_LOAD( "136072-1103.28p", 0x040000, 0x010000, CRC(3011da3b) SHA1(e7118b111e0de9b9a2dfe5a165f2140e90c919e7) )
	ROM_LOAD( "136072-1108.53r", 0x050000, 0x010000, CRC(386c7edc) SHA1(a2f61c7f8fb822433957565f373ab5cc8e0a2ba0) )
	ROM_LOAD( "136072-1110.67r", 0x060000, 0x010000, CRC(a54d16e6) SHA1(ad10623f0a87e5a92b8e8c611c2d374a8fd7047e) )
	ROM_LOAD( "136072-1112.81r", 0x070000, 0x010000, CRC(669411f6) SHA1(a0c678a75076b466f4a27c881642c44d47c9dea0) )
	ROM_LOAD( "136072-1107.53p", 0x080000, 0x010000, CRC(caaeb57a) SHA1(e20050d10ee18f52ac36616003de241aa9951eab) )
	ROM_LOAD( "136072-1109.67p", 0x090000, 0x010000, CRC(61cb4e28) SHA1(6d0cb4409fa4c9c696abd612395d5f6ddede6779) )
	ROM_LOAD( "136072-1114.95r", 0x0a0000, 0x010000, CRC(e340d5a1) SHA1(29a23ad2b6c8302508a8b49cfbc064fe86a8b908) )
	ROM_LOAD( "136072-1116.109r", 0x0b0000, 0x010000, CRC(f25b8aca) SHA1(c8c6d0951098c32e32b87d9717cc14bb91ac2017) )
	ROM_LOAD( "136072-1118.123r", 0x0c0000, 0x010000, CRC(8cf73585) SHA1(b1f3e44f6cd434ecfe4d88463b7e2e7b48d2bf1f) )
	ROM_LOAD( "136072-1113.95p", 0x0d0000, 0x010000, CRC(899b59af) SHA1(c7344e4bf57024415463cb50c788631fbad07132) )
	ROM_LOAD( "136072-1115.109p", 0x0e0000, 0x010000, CRC(cf4fd19a) SHA1(731fc7bb1dacacc6e2e4db1e096d07d5fe3d8b19) )
	ROM_LOAD( "136072-1120.137r", 0x0f0000, 0x010000, CRC(fde7c03d) SHA1(ec336af0f3314af134fd1a64c478be06249a550a) )
	ROM_LOAD( "136072-1122.151r", 0x100000, 0x010000, CRC(6ff6a9f2) SHA1(af8fda010c15e13e58e0f235b7b5a12d5a21fc0c) )
	ROM_LOAD( "136072-1124.165r", 0x110000, 0x010000, CRC(f11909f1) SHA1(2dbd5c92e8329f5b5033b3633bd56618eb7da875) )
	ROM_LOAD( "136072-1119.137p", 0x120000, 0x010000, CRC(6f8003a1) SHA1(df8494ce56213dddd11f1365c03bb77ebf3e6eba) )
	ROM_LOAD( "136072-1121.151p", 0x130000, 0x010000, CRC(8ff0a1ec) SHA1(8df33657a035316a1e4ce7d7b33af6e51b990c48) )
	ROM_LOAD( "136072-1125.123n", 0x140000, 0x010000, CRC(3aa7c756) SHA1(c1585733cef28fdf031e335503364846cfd0384a) )
	ROM_LOAD( "136072-1126.137n", 0x150000, 0x010000, CRC(cb82c9aa) SHA1(1469d2825e6d366a8e3f74294b0a64f2a63384aa) )
	ROM_LOAD( "136072-1128.151n", 0x160000, 0x010000, CRC(dce32863) SHA1(b0476de8d54dcf163a723b1fc805da4e1ca11c27) )
	/* 170000-18ffff empty */

	ROM_REGION( 0x0a0000, "gfx2", ROMREGION_INVERT )
	ROM_LOAD( "136072-2129.180p", 0x000000, 0x010000, CRC(36b1a578) SHA1(ded9cccd1009e517662387353333f20031abddd5) )
	ROM_LOAD( "136072-2131.193p", 0x010000, 0x010000, CRC(7b7c04a1) SHA1(b57f3f35f39ecf912daf2611919f31a92005f07b) )
	ROM_LOAD( "136072-2133.208p", 0x020000, 0x010000, CRC(e03fe4d9) SHA1(d9a9174a2d2e2d83f7c32177f0dbab74f3875d2e) )
	ROM_LOAD( "136072-2135.221p", 0x030000, 0x010000, CRC(7d497110) SHA1(4d5ce6673a112486e9dff77c901d90105aa0a210) )
	ROM_LOAD( "136072-2137.235p", 0x040000, 0x010000, CRC(f91e7872) SHA1(690715a92e8ca1b1d22fff85f9ed3f1e02edca99) )
	ROM_LOAD( "136072-2130.180r", 0x050000, 0x010000, CRC(b25368cc) SHA1(110e6882399e200b3a4cdd823cc5b0565183cfeb) )
	ROM_LOAD( "136072-2132.193r", 0x060000, 0x010000, CRC(112f2d20) SHA1(3acd43cf73f1be10c17a717c8990f5c656935b3a) )
	ROM_LOAD( "136072-2134.208r", 0x070000, 0x010000, CRC(84884ed6) SHA1(6129f090a4e5b8f65895086e1731b13ee72b6079) )
	ROM_LOAD( "136072-2136.221r", 0x080000, 0x010000, CRC(bc028690) SHA1(e75a961febf1f1a6c4981301b73bab82c3d19785) )
	ROM_LOAD( "136072-2138.235r", 0x090000, 0x010000, CRC(60cec955) SHA1(d184746589130a8f10fcc6c79484578bd08757f0) )

	ROM_REGION( 0x008000, "gfx3", 0 )
	ROM_LOAD( "136072-2141.250k", 0x000000, 0x008000, CRC(60d6d6df) SHA1(a8d56092f014a0a93742c701effaec86c75772e1) )

	ROM_REGION( 0x40000, "jsa:oki1", 0 )   /* 256k for ADPCM samples */
	ROM_LOAD( "136072-1145.7k",  0x000000, 0x010000, CRC(d9475d58) SHA1(5a4a0094c83f5d0e26f0c35feb0b1f15a5f5c3f9) )
	ROM_LOAD( "136072-1146.7j",  0x010000, 0x010000, CRC(133e6aef) SHA1(e393d0b246889779029443a19e3859d45cb900db) )
	ROM_LOAD( "136072-1147.7e",  0x020000, 0x010000, CRC(ba4d556e) SHA1(af4364f2c456abc20f1742c945a3acfeb5e192c4) )
	ROM_LOAD( "136072-1148.7d",  0x030000, 0x010000, CRC(c48df49a) SHA1(c92fde9be1a1ab09453c57eefb0dcfe49e538d07) )
ROM_END



/*************************************
 *
 *  ROM decoding
 *
 *************************************/

DRIVER_INIT_MEMBER(skullxbo_state,skullxbo)
{
	memset(memregion("gfx1")->base() + 0x170000, 0, 0x20000);
}



/*************************************
 *
 *  Game driver(s)
 *
 *************************************/

GAME( 1989, skullxbo,  0,        skullxbo, skullxbo, skullxbo_state, skullxbo, ROT0, "Atari Games", "Skull & Crossbones (rev 5)", 0 )
GAME( 1989, skullxbo4, skullxbo, skullxbo, skullxbo, skullxbo_state, skullxbo, ROT0, "Atari Games", "Skull & Crossbones (rev 4)", 0 )
GAME( 1989, skullxbo3, skullxbo, skullxbo, skullxbo, skullxbo_state, skullxbo, ROT0, "Atari Games", "Skull & Crossbones (rev 3)", 0 )
GAME( 1989, skullxbo2, skullxbo, skullxbo, skullxbo, skullxbo_state, skullxbo, ROT0, "Atari Games", "Skull & Crossbones (rev 2)", 0 )
GAME( 1989, skullxbo1, skullxbo, skullxbo, skullxbo, skullxbo_state, skullxbo, ROT0, "Atari Games", "Skull & Crossbones (rev 1)", 0 )

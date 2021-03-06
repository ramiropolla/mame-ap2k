/***************************************************************************
 
 Ramtek Discrete Games List
 
 Game Name                  DATA      Board #
 --------------------------------------------
 (Deluxe) Baseball (1974)   YES
 Clean Sweep (1974)         YES       501082
 Hockey (1973)              UNKNOWN
 Horoscope (1976)           UNKNOWN
 Knockout (1974)            UNKNOWN
 Sea Battle (1976)          UNKNOWN
 Soccer (1973)              UNKNOWN
 Trivia (1976)              YES
 Volly/Volley (1973)        YES
 Wipe Out (1974)            YES
 
 ***************************************************************************/


#include "emu.h"

#include "machine/rescap.h"
#include "machine/netlist.h"
#include "netlist/devices/net_lib.h"
#include "video/fixfreq.h"
#include "astring.h"

// copied by Pong, not accurate for this driver!
// start
#define MASTER_CLOCK    7159000
#define V_TOTAL         (0x105+1)       // 262
#define H_TOTAL         (0x1C6+1)       // 454

#define HBSTART                 (H_TOTAL)
#define HBEND                   (80)
#define VBSTART                 (V_TOTAL)
#define VBEND                   (16)

#define HRES_MULT                   (1)

fixedfreq_interface fixedfreq_mode_ramtek = {
	MASTER_CLOCK,
	H_TOTAL-67,H_TOTAL-40,H_TOTAL-8,H_TOTAL,
	V_TOTAL-22,V_TOTAL-19,V_TOTAL-12,V_TOTAL,
	1,  /* non-interlaced */
	0.30
};
// end


class ramtek_state : public driver_device
{
public:
	ramtek_state(const machine_config &mconfig, device_type type, const char *tag)
	: driver_device(mconfig, type, tag),
	  m_maincpu(*this, "maincpu"),
	  m_video(*this, "fixfreq")
	{
	}
	
	// devices
	required_device<netlist_mame_device_t> m_maincpu;
	required_device<fixedfreq_device> m_video;
	
protected:
	
	// driver_device overrides
	virtual void machine_start();
	virtual void machine_reset();
	
	virtual void video_start();
	
private:
	
};


static NETLIST_START(ramtek)
	SOLVER(Solver, 48000)
//	PARAM(Solver.FREQ, 48000)
	PARAM(Solver.ACCURACY, 1e-4) // works and is sufficient

	// schematics
	//...

//	NETDEV_ANALOG_CALLBACK(sound_cb, sound, exidyttl_state, sound_cb, "")
//	NETDEV_ANALOG_CALLBACK(video_cb, videomix, fixedfreq_device, update_vid, "fixfreq")
NETLIST_END()



void ramtek_state::machine_start()
{
}

void ramtek_state::machine_reset()
{
}


void ramtek_state::video_start()
{
}

static MACHINE_CONFIG_START( ramtek, ramtek_state )

	/* basic machine hardware */
	MCFG_DEVICE_ADD("maincpu", NETLIST_CPU, NETLIST_CLOCK)
	MCFG_NETLIST_SETUP(ramtek)

	/* video hardware */
	MCFG_FIXFREQ_ADD("fixfreq", "screen", fixedfreq_mode_ramtek)
MACHINE_CONFIG_END


/***************************************************************************
 
 Game driver(s)
 
 ***************************************************************************/


ROM_START( bballrmt )
	ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )

	ROM_REGION( 0x0020, "roms", ROMREGION_ERASE00 )
	ROM_LOAD( "ramtek.a1",     0x0000, 0x0020, CRC(3d505672) SHA1(aba7d80f97f2a1584ffee4e3bde2501b267e071b) )
	ROM_LOAD( "ramtek.a3",     0x0000, 0x0020, CRC(bb2c75a0) SHA1(809d48dfdc966c3aff7498b0c866707796dc646b) )
	ROM_LOAD( "ramtek.a4",     0x0000, 0x0020, CRC(3e449889) SHA1(ddba70b0e9721b60f4846397517b59e900be9dc6) )
	ROM_LOAD( "ramtek.f8",     0x0000, 0x0020, CRC(205bf9a1) SHA1(3014d226d8afebc6a52e5adb84f1846dd1c0b01c) )
	ROM_LOAD( "ramtek.h1",     0x0000, 0x0020, CRC(5d13db19) SHA1(384822aaf55423c7c8736d86c6376f3eab44a5a3) )
	ROM_LOAD( "ramtek.h2",     0x0000, 0x0020, CRC(24d8716c) SHA1(152dae056ced91723059f2debfd0e132383a80c6) )
	ROM_LOAD( "ramtek.h6",     0x0000, 0x0020, CRC(0ead8f9f) SHA1(d84a203221e3bd669ed301643450c6f86bf0f790) )
	ROM_LOAD( "ramtek.h7",     0x0000, 0x0020, CRC(cd19c9ff) SHA1(d9867db0b1b1023d9fb6440b9d20f8417e1da6a7) )
	ROM_LOAD( "ramtek.j9",     0x0000, 0x0020, CRC(4b86eb32) SHA1(07823d59d658911ba4b96ceb87e1f8dc7b27180d) )
	ROM_LOAD( "ramtek.j10",    0x0000, 0x0020, CRC(9e825d0c) SHA1(20c18d152d0b9135cd1fefd62b8bf58d03023529) )
ROM_END


// The board number for Clean Sweep is 501082, and the letter to the right would be the revision letter (A, B, C, D). 
// Ramtek had revisions A, B, C, D, noted with a sticker that shows the letter for each specific revision. 
// Revisions A,B,C have been dumped and rom content is the same (only diff is the location of some chips on the rev B pcb)
ROM_START( cleanswp )
	ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )

	ROM_REGION( 0x0020, "roms", ROMREGION_ERASE00 )
	ROM_LOAD( "501075.c6",     0x0000, 0x0020, CRC(be02b5f5) SHA1(f1c616a73c6c2915ea3d0252543b0806704ab2e9) )	// shape of paddle
	ROM_LOAD( "501076.d7",     0x0000, 0x0020, CRC(be40b153) SHA1(07fb64ea8caee601e3e3bd6c69beea619dd0489d) )	// ball control memory 
	ROM_LOAD( "501074.k3",     0x0000, 0x0020, CRC(515a34ba) SHA1(471ca9d99851591ff11a87d18b88871edd7fd268) )	// number character generation
ROM_END


ROM_START( ramtek3 )	// maybe Hockey?
	ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )

	ROM_REGION( 0x0020, "roms", ROMREGION_ERASE00 )
	ROM_LOAD( "ramtek3.g8",     0x0000, 0x0020, CRC(f14416f8) SHA1(413f051c73f05c3c5c2ec1dd8620e03c43835e10) )
ROM_END


ROM_START( vollyrmt )
	ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )

	ROM_REGION( 0x0020, "roms", ROMREGION_ERASE00 )
	ROM_LOAD( "ramtek2.e0",     0x0000, 0x0020, CRC(205bf9a1) SHA1(3014d226d8afebc6a52e5adb84f1846dd1c0b01c) )
	// an alt dump exists for this rom labeled as ramtek5.g9 ... was it used on two different PCBs?
ROM_END


ROM_START( wipeormt )
	ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )

	ROM_REGION( 0x0020, "roms", ROMREGION_ERASE00 )
	ROM_LOAD( "ramtek6.a4",     0x0000, 0x0020, CRC(205bf9a1) SHA1(3014d226d8afebc6a52e5adb84f1846dd1c0b01c) )
	ROM_LOAD( "ramtek6.g7",     0x0000, 0x0020, CRC(215437e1) SHA1(635f98d8a890b3ddb72a2f764ef007b5f40fcd7f) )
ROM_END


GAME( 1974, bballrmt,  0, ramtek, 0, driver_device,  0, ROT0, "Ramtek", "Baseball (Ramtek) [TTL]", GAME_IS_SKELETON )
GAME( 1974, cleanswp,  0, ramtek, 0, driver_device,  0, ROT0, "Ramtek", "Clean Sweep [TTL]", GAME_IS_SKELETON )
GAME( 1973, vollyrmt,  0, ramtek, 0, driver_device,  0, ROT0, "Ramtek", "Volly (Ramtek) [TTL]", GAME_IS_SKELETON )
GAME( 1974, wipeormt,  0, ramtek, 0, driver_device,  0, ROT0, "Ramtek", "Wipeout (Ramtek) [TTL]", GAME_IS_SKELETON )

GAME( 197?, ramtek3,   0, ramtek, 0, driver_device,  0, ROT0, "Ramtek", "unknown Ramtek Game (Maybe Hockey?) [TTL]", GAME_IS_SKELETON )

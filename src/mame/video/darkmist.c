#include "emu.h"
#include "includes/darkmist.h"



/* vis. flags */

#define DISPLAY_SPR     1
#define DISPLAY_FG      2 /* 2 or 8 */
#define DISPLAY_BG      4
#define DISPLAY_TXT     16


TILE_GET_INFO_MEMBER(darkmist_state::get_bgtile_info)
{
	int code,attr,pal;

	code=memregion("user1")->base()[tile_index]; /* TTTTTTTT */
	attr=memregion("user2")->base()[tile_index]; /* -PPP--TT - FIXED BITS (0xxx00xx) */
	code+=(attr&3)<<8;
	pal=(attr>>4);

	SET_TILE_INFO_MEMBER(m_gfxdecode, 
		1,
		code,
		pal,
		0);
}

TILE_GET_INFO_MEMBER(darkmist_state::get_fgtile_info)
{
	int code,attr,pal;

	code=memregion("user3")->base()[tile_index]; /* TTTTTTTT */
	attr=memregion("user4")->base()[tile_index]; /* -PPP--TT - FIXED BITS (0xxx00xx) */
	pal=attr>>4;

	code+=(attr&3)<<8;

	code+=0x400;

	pal+=16;

	SET_TILE_INFO_MEMBER(m_gfxdecode, 
		1,
		code,
		pal,
		0);
}

TILE_GET_INFO_MEMBER(darkmist_state::get_txttile_info)
{
	UINT8 *videoram = m_videoram;
	int code,attr,pal;

	code=videoram[tile_index];
	attr=videoram[tile_index+0x400];
	pal=(attr>>1);

	code+=(attr&1)<<8;

	pal+=48;

	SET_TILE_INFO_MEMBER(m_gfxdecode, 
		0,
		code,
		pal,
		0);
}

PALETTE_INIT_MEMBER(darkmist_state, darkmist)
{
	const UINT8 *color_prom = memregion("proms")->base();
	int i;

	for (i = 0; i < 0x400; i++)
	{
		int ctabentry;

		if (color_prom[i] & 0x40)
			ctabentry = 0x100;
		else
		{
			ctabentry = (color_prom[i] & 0x3f);

			switch (i & 0x300)
			{
			case 0x000:  ctabentry = ctabentry | 0x80; break;
			case 0x100:  ctabentry = ctabentry | 0x00; break;
			case 0x200:  ctabentry = ctabentry | 0x40; break;
			case 0x300:  ctabentry = ctabentry | 0xc0; break;
			}
		}

		palette.set_pen_indirect(i, ctabentry);
	}
}


void darkmist_state::set_pens()
{
	int i;

	for (i = 0; i < 0x100; i++)
	{
		int r = pal4bit(m_generic_paletteram_8[i | 0x200] >> 0);
		int g = pal4bit(m_generic_paletteram_8[i | 0x000] >> 4);
		int b = pal4bit(m_generic_paletteram_8[i | 0x000] >> 0);

		m_palette->set_indirect_color(i, rgb_t(r, g, b));
	}

	m_palette->set_indirect_color(0x100, rgb_t::black);
}


void darkmist_state::video_start()
{
	m_bgtilemap = &machine().tilemap().create(tilemap_get_info_delegate(FUNC(darkmist_state::get_bgtile_info),this),TILEMAP_SCAN_ROWS,16,16,512,64 );
	m_fgtilemap = &machine().tilemap().create(tilemap_get_info_delegate(FUNC(darkmist_state::get_fgtile_info),this),TILEMAP_SCAN_ROWS,16,16,64,256 );
	m_txtilemap = &machine().tilemap().create(tilemap_get_info_delegate(FUNC(darkmist_state::get_txttile_info),this),TILEMAP_SCAN_ROWS,8,8,32,32 );
	m_fgtilemap->set_transparent_pen(0);
	m_txtilemap->set_transparent_pen(0);
}

UINT32 darkmist_state::screen_update_darkmist(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	UINT8 *spriteram = m_spriteram;

#define DM_GETSCROLL(n) (((m_scroll[(n)]<<1)&0xff) + ((m_scroll[(n)]&0x80)?1:0) +( ((m_scroll[(n)-1]<<4) | (m_scroll[(n)-1]<<12) )&0xff00))

	set_pens();

	m_bgtilemap->set_scrollx(0, DM_GETSCROLL(0x2));
	m_bgtilemap->set_scrolly(0, DM_GETSCROLL(0x6));
	m_fgtilemap->set_scrollx(0, DM_GETSCROLL(0xa));
	m_fgtilemap->set_scrolly(0, DM_GETSCROLL(0xe));

	bitmap.fill(m_palette->black_pen(), cliprect);

	if(m_hw & DISPLAY_BG)
		m_bgtilemap->draw(screen, bitmap, cliprect, 0,0);

	if(m_hw & DISPLAY_FG)
		m_fgtilemap->draw(screen, bitmap, cliprect, 0,0);

	if(m_hw & DISPLAY_SPR)
	{
/*
    Sprites

    76543210
0 - TTTT TTTT - tile
1 - xyBP PPP? - palette (P), flips (x,y), B - use spritebank,
                ? - unknown, according to gamecode top bit of one of coords(y/x)
2 - YYYY YYYY - y coord
3 - XXXX XXXX - x coord

*/
		int i,fx,fy,tile,palette;
		for(i=0;i<m_spriteram.bytes();i+=32)
		{
			fy=spriteram[i+1]&0x40;
			fx=spriteram[i+1]&0x80;

			tile=spriteram[i+0];

			if(spriteram[i+1]&0x20)
				tile += (*m_spritebank << 8);

			palette=((spriteram[i+1])>>1)&0xf;

			if(spriteram[i+1]&0x1)
				palette=machine().rand()&15;

			palette+=32;

			
				m_gfxdecode->gfx(2)->transpen(m_palette,
				bitmap,cliprect,
				tile,
				palette,
				fx,fy,
				spriteram[i+3],spriteram[i+2],0 );
		}
	}

	if(m_hw & DISPLAY_TXT)
	{
		m_txtilemap->mark_all_dirty();
		m_txtilemap->draw(screen, bitmap, cliprect, 0,0);
	}


	return 0;
}

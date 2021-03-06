/***************************************************************************

    Bionic Commando Video Hardware

    This board handles tile/tile and tile/sprite priority with a PROM. Its
    working is complicated and hardcoded in the driver.

    The PROM is a 256x4 chip, with address inputs wired as follows:

    A0 bg opaque
    A1 \
    A2 |  fg pen
    A3 |
    A4 /
    A5 fg has priority over sprites (bit 5 of tile attribute)
    A6 fg has not priority over bg (bits 6 & 7 of tile attribute both set)
    A7 sprite opaque

    The output selects the active layer, it can be:
    0  bg
    1  fg
    2  sprite

***************************************************************************/

#include "emu.h"
#include "includes/bionicc.h"


/***************************************************************************

  Callbacks for the TileMap code

***************************************************************************/

TILE_GET_INFO_MEMBER(bionicc_state::get_bg_tile_info)
{
	int attr = m_bgvideoram[2 * tile_index + 1];
	SET_TILE_INFO_MEMBER(m_gfxdecode, 
			1,
			(m_bgvideoram[2 * tile_index] & 0xff) + ((attr & 0x07) << 8),
			(attr & 0x18) >> 3,
			TILE_FLIPXY((attr & 0xc0) >> 6));
}

TILE_GET_INFO_MEMBER(bionicc_state::get_fg_tile_info)
{
	int attr = m_fgvideoram[2 * tile_index + 1];
	int flags;

	if ((attr & 0xc0) == 0xc0)
	{
		tileinfo.category = 1;
		tileinfo.group = 0;
		flags = 0;
	}
	else
	{
		tileinfo.category = 0;
		tileinfo.group = (attr & 0x20) >> 5;
		flags = TILE_FLIPXY((attr & 0xc0) >> 6);
	}

	SET_TILE_INFO_MEMBER(m_gfxdecode, 
			2,
			(m_fgvideoram[2 * tile_index] & 0xff) + ((attr & 0x07) << 8),
			(attr & 0x18) >> 3,
			flags);
}

TILE_GET_INFO_MEMBER(bionicc_state::get_tx_tile_info)
{
	int attr = m_txvideoram[tile_index + 0x400];
	SET_TILE_INFO_MEMBER(m_gfxdecode, 
			0,
			(m_txvideoram[tile_index] & 0xff) + ((attr & 0x00c0) << 2),
			attr & 0x3f,
			0);
}



/***************************************************************************

  Start the video hardware emulation.

***************************************************************************/

void bionicc_state::video_start()
{
	m_tx_tilemap = &machine().tilemap().create(tilemap_get_info_delegate(FUNC(bionicc_state::get_tx_tile_info),this), TILEMAP_SCAN_ROWS,  8, 8, 32, 32);
	m_fg_tilemap = &machine().tilemap().create(tilemap_get_info_delegate(FUNC(bionicc_state::get_fg_tile_info),this), TILEMAP_SCAN_ROWS, 16, 16, 64, 64);
	m_bg_tilemap = &machine().tilemap().create(tilemap_get_info_delegate(FUNC(bionicc_state::get_bg_tile_info),this), TILEMAP_SCAN_ROWS,  8, 8, 64, 64);

	m_tx_tilemap->set_transparent_pen(3);
	m_fg_tilemap->set_transmask(0, 0xffff, 0x8000); /* split type 0 is completely transparent in front half */
	m_fg_tilemap->set_transmask(1, 0xffc1, 0x803e); /* split type 1 has pens 1-5 opaque in front half */
	m_bg_tilemap->set_transparent_pen(15);
}



/***************************************************************************

  Memory handlers

***************************************************************************/

WRITE16_MEMBER(bionicc_state::bionicc_bgvideoram_w)
{
	COMBINE_DATA(&m_bgvideoram[offset]);
	m_bg_tilemap->mark_tile_dirty(offset / 2);
}

WRITE16_MEMBER(bionicc_state::bionicc_fgvideoram_w)
{
	COMBINE_DATA(&m_fgvideoram[offset]);
	m_fg_tilemap->mark_tile_dirty(offset / 2);
}

WRITE16_MEMBER(bionicc_state::bionicc_txvideoram_w)
{
	COMBINE_DATA(&m_txvideoram[offset]);
	m_tx_tilemap->mark_tile_dirty(offset & 0x3ff);
}

WRITE16_MEMBER(bionicc_state::bionicc_paletteram_w)
{
	int r, g, b, bright;
	data = COMBINE_DATA(&m_paletteram[offset]);

	bright = (data & 0x0f);

	r = ((data >> 12) & 0x0f) * 0x11;
	g = ((data >> 8 ) & 0x0f) * 0x11;
	b = ((data >> 4 ) & 0x0f) * 0x11;

	if ((bright & 0x08) == 0)
	{
		r = r * (0x07 + bright) / 0x0e;
		g = g * (0x07 + bright) / 0x0e;
		b = b * (0x07 + bright) / 0x0e;
	}

	m_palette->set_pen_color (offset, rgb_t(r, g, b));
}

WRITE16_MEMBER(bionicc_state::bionicc_scroll_w)
{
	data = COMBINE_DATA(&m_scroll[offset]);

	switch (offset)
	{
		case 0:
			m_fg_tilemap->set_scrollx(0, data);
			break;
		case 1:
			m_fg_tilemap->set_scrolly(0, data);
			break;
		case 2:
			m_bg_tilemap->set_scrollx(0, data);
			break;
		case 3:
			m_bg_tilemap->set_scrolly(0, data);
			break;
	}
}

WRITE16_MEMBER(bionicc_state::bionicc_gfxctrl_w)
{
	if (ACCESSING_BITS_8_15)
	{
		flip_screen_set(data & 0x0100);

		m_bg_tilemap->enable(data & 0x2000);    /* guess */
		m_fg_tilemap->enable(data & 0x1000);    /* guess */

		coin_counter_w(machine(), 0, data & 0x8000);
		coin_counter_w(machine(), 1, data & 0x4000);
	}
}



/***************************************************************************

  Display refresh

***************************************************************************/

void bionicc_state::draw_sprites( bitmap_ind16 &bitmap, const rectangle &cliprect )
{
	UINT16 *buffered_spriteram = m_spriteram->buffer();
	int offs;
	gfx_element *gfx = m_gfxdecode->gfx(3);

	for (offs = (m_spriteram->bytes() - 8) / 2; offs >= 0; offs -= 4)
	{
		int tile_number = buffered_spriteram[offs] & 0x7ff;
		if( tile_number != 0x7ff )
		{
			int attr = buffered_spriteram[offs + 1];
			int color = (attr & 0x3c) >> 2;
			int flipx = attr & 0x02;
			int flipy = 0;
			int sx = (INT16)buffered_spriteram[offs + 3];   /* signed */
			int sy = (INT16)buffered_spriteram[offs + 2];   /* signed */

			if (sy > 512 - 16)
				sy -= 512;

			if (flip_screen())
			{
				sx = 240 - sx;
				sy = 240 - sy;
				flipx = !flipx;
				flipy = !flipy;
			}

			gfx->transpen(m_palette,bitmap,cliprect,
				tile_number,
				color,
				flipx,flipy,
				sx,sy,15);
		}
	}
}

UINT32 bionicc_state::screen_update_bionicc(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	bitmap.fill(m_palette->black_pen(), cliprect);
	m_fg_tilemap->draw(screen, bitmap, cliprect, 1 | TILEMAP_DRAW_LAYER1, 0);   /* nothing in FRONT */
	m_bg_tilemap->draw(screen, bitmap, cliprect, 0, 0);
	m_fg_tilemap->draw(screen, bitmap, cliprect, 0 | TILEMAP_DRAW_LAYER1, 0);
	draw_sprites(bitmap, cliprect);
	m_fg_tilemap->draw(screen, bitmap, cliprect, 0 | TILEMAP_DRAW_LAYER0, 0);
	m_tx_tilemap->draw(screen, bitmap, cliprect, 0, 0);
	return 0;
}

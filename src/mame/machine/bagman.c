/***************************************************************************

  machine.c

  Functions to emulate general aspects of the machine (RAM, ROM, interrupts,
  I/O ports)

***************************************************************************/

#include "driver.h"
#include "includes/bagman.h"


/*Creation date: 98-02-18 */
/*  A few words of comment:
**
**   What's inside of this file is a PAL16R6 emulator. Maybe someday we will
**need to use it for some other game too. We will need to make it more exact
**then (some of the functionality of this chip IS NOT implemented). However I
**have bought a book about PALs and I'm able to improve it. Just LMK.
**  Jarek Burczynski
**  bujar at mame dot net
*/


/*table holds outputs of all ANDs (after AND map)*/
static UINT8 andmap[64];

/*table holds inputs (ie. not x, x, not q, q) to the AND map*/
static UINT8 columnvalue[32];

/*8 output pins (actually 6 output and 2 input/output)*/
static UINT8 outvalue[8];

/*      64 rows x 32 columns
**  1 - fuse blown: disconnected from input (equal to 1)
**  0 - fuse not blown: connected to input (ie. x, not x, q, not q accordingly)
*/
static const UINT8 fusemap[64*32]=
{
1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
1,1,1,1,1,1,0,1,1,1,1,0,1,1,0,1,1,1,1,0,1,1,0,1,1,1,0,1,1,1,1,1,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
1,1,0,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
1,1,0,1,1,1,0,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
1,1,0,1,1,1,1,0,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
1,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
1,1,0,1,1,1,1,0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
1,1,0,1,1,1,1,1,1,1,1,0,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
1,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,
1,1,0,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,
1,1,0,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,
1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
1,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,1,1,1,1,1,1,
1,1,0,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,
1,1,0,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,
1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,
1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,0,1,1,1,1,1,1,1,1,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
1,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,1,1,
1,1,0,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,
1,1,0,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,
1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,
1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0,1,1,1,1,
1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,0,1,1,1,1,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};


static void update_pal(void)
{
UINT16 rowoffs;
UINT8 row, column, val;

/*calculate all rows ANDs*/
	for (row = 0; row < 64; row++)
	{
		rowoffs = row*32;
		val = 1; /*prepare for AND */
		for (column = 0; column < 32; column++)
		{
			if ( fusemap[ rowoffs + column ] == 0 )
				val &= columnvalue[column];
		}
		andmap[row] = val;
	}

/* I/O pin #19 */
	val = 0; /*prepare for OR*/
	for (row = 1; row < 8; row++)
		val |= andmap[row];
	if (andmap[0] == 1)
	{
		columnvalue[2] = 1-val;
		columnvalue[3] = val;
		outvalue[0]    = 1-val;
	}
	else
	{
		/*pin is in INPUT configuration so it doesn't create output...*/
		columnvalue[2] = 0;
		columnvalue[3] = 1;
	}

/* O pin #18 (D1) */
	val = 0; /*prepare for OR*/
	for (row = 8; row < 16; row++)
		val |= andmap[row];
	columnvalue[6] = 1-val;
	columnvalue[7] = val;
	outvalue[1]    = 1-val;

/* O pin #17 (D2) */
	val = 0; /*prepare for OR*/
	for (row = 16; row < 24; row++)
		val |= andmap[row];
	columnvalue[10] = 1-val;
	columnvalue[11] = val;
	outvalue[2]     = 1-val;

/* O pin #16 (D3) */
	val = 0; /*prepare for OR*/
	for (row = 24; row < 32; row++)
		val |= andmap[row];
	columnvalue[14] = 1-val;
	columnvalue[15] = val;
	outvalue[3]     = 1-val;

/* O pin #15 (D4) */
	val = 0; /*prepare for OR*/
	for (row = 32; row < 40; row++)
		val |= andmap[row];
	columnvalue[18] = 1-val;
	columnvalue[19] = val;
	outvalue[4]     = 1-val;

/* O pin #14 (D5) */
	val = 0; /*prepare for OR*/
	for (row = 40; row < 48; row++)
		val |= andmap[row];
	columnvalue[22] = 1-val;
	columnvalue[23] = val;
	outvalue[5]     = 1-val;

/* O pin #13 (D6) */
	val = 0; /*prepare for OR*/
	for (row = 48; row < 56; row++)
		val |= andmap[row];
	columnvalue[26] = 1-val;
	columnvalue[27] = val;
	outvalue[6]     = 1-val;

/* I/O pin #12 */
	val = 0; /*prepare for OR*/
	for (row = 57; row < 64; row++)
		val |= andmap[row];
	if (andmap[56] == 1)
	{
		columnvalue[30] = 1-val;
		columnvalue[31] = val;
		outvalue[7]     = 1-val;
	}
	else
	{
		/*pin is in INPUT configuration so it doesn't create output...*/
		columnvalue[30] = 0;
		columnvalue[31] = 1;
	}

}


WRITE8_HANDLER( bagman_pal16r6_w )
{
UINT8 line;

	line = offset*4;
	columnvalue[line  ] = data&1;
	columnvalue[line+1] = 1-(data&1);
}

MACHINE_RESET( bagman )
{
	const address_space *space = cpu_get_address_space(machine->cpu[0], ADDRESS_SPACE_PROGRAM);
	bagman_pal16r6_w(space,0,1);	/*pin 2*/
	bagman_pal16r6_w(space,1,1);	/*pin 3*/
	bagman_pal16r6_w(space,2,1);	/*pin 4*/
	bagman_pal16r6_w(space,3,1);	/*pin 5*/
	bagman_pal16r6_w(space,4,1);	/*pin 6*/
	bagman_pal16r6_w(space,5,1);	/*pin 7*/
	bagman_pal16r6_w(space,6,1);	/*pin 8*/
	bagman_pal16r6_w(space,7,1);	/*pin 9*/
	update_pal();
}

READ8_HANDLER( bagman_pal16r6_r )
{
	update_pal();
	return	(outvalue[6]) + (outvalue[5]<<1) + (outvalue[4]<<2) +
		(outvalue[3]<<3) + (outvalue[2]<<4) + (outvalue[1]<<5);

/* Bagman schematics show that this is right mapping order of PAL outputs to bits.
** This is the PAL 16R6 shown almost in the middle of the schematics.
** The /RD4 line goes low (active) whenever CPU reads from memory address a000.
*/
}

/***************************************************************************

    Tatsumi TX-1/Buggy Boy sound hardware

***************************************************************************/
#include "driver.h"
#include "includes/tx1.h"
#include "video/resnet.h"
#include "sound/custom.h"
#include "streams.h"

static sound_stream *stream;
static int freq_to_step;

/*************************************
 *
 *  TX-1
 *
 *************************************/


/*************************************
 *
 *  Buggy Boy
 *
 *************************************/

#define BUGGYBOY_PIT_CLOCK		(BUGGYBOY_ZCLK / 8)
#define BUGGYBOY_NOISE_CLOCK	(BUGGYBOY_PIT_CLOCK / 4)

#define BUGGYBOY_R1		47000.0
#define BUGGYBOY_R2		22000.0
#define BUGGYBOY_R3		10000.0
#define BUGGYBOY_R4		5600.0
#define BUGGYBOY_SHUNT	250.0

#define BUGGYBOY_R1S	(1.0/(1.0/BUGGYBOY_R1 + 1.0/BUGGYBOY_SHUNT))
#define BUGGYBOY_R2S	(1.0/(1.0/BUGGYBOY_R2 + 1.0/BUGGYBOY_SHUNT))
#define BUGGYBOY_R3S	(1.0/(1.0/BUGGYBOY_R3 + 1.0/BUGGYBOY_SHUNT))
#define BUGGYBOY_R4S	(1.0/(1.0/BUGGYBOY_R4 + 1.0/BUGGYBOY_SHUNT))

static int noise_lfsra;
static int noise_lfsrb;
static int noise_lfsrc;
static int noise_lfsrd;
static int noise_counter;
static int step0;
static int step1;
static UINT8 ym_outputa;
static UINT8 ym_outputb;
static UINT16 buggyboy_eng_voltages[16];

static const double bb_engine_gains[16] =
{
	-1.0/(1.0/(BUGGYBOY_R1  + BUGGYBOY_R2  + BUGGYBOY_R3  + BUGGYBOY_R4 ) + 1.0/100E3)/100E3,
	-1.0/(1.0/(BUGGYBOY_R1  + BUGGYBOY_R2  + BUGGYBOY_R3  + BUGGYBOY_R4S) + 1.0/100E3)/100E3,
	-1.0/(1.0/(BUGGYBOY_R1  + BUGGYBOY_R2  + BUGGYBOY_R3S + BUGGYBOY_R4 ) + 1.0/100E3)/100E3,
	-1.0/(1.0/(BUGGYBOY_R1  + BUGGYBOY_R2  + BUGGYBOY_R3S + BUGGYBOY_R4S) + 1.0/100E3)/100E3,
	-1.0/(1.0/(BUGGYBOY_R1  + BUGGYBOY_R2S + BUGGYBOY_R3  + BUGGYBOY_R4 ) + 1.0/100E3)/100E3,	
	-1.0/(1.0/(BUGGYBOY_R1  + BUGGYBOY_R2S + BUGGYBOY_R3  + BUGGYBOY_R4S) + 1.0/100E3)/100E3,
	-1.0/(1.0/(BUGGYBOY_R1  + BUGGYBOY_R2S + BUGGYBOY_R3S + BUGGYBOY_R4 ) + 1.0/100E3)/100E3,
	-1.0/(1.0/(BUGGYBOY_R1  + BUGGYBOY_R2S + BUGGYBOY_R3S + BUGGYBOY_R4S) + 1.0/100E3)/100E3,
	-1.0/(1.0/(BUGGYBOY_R1S + BUGGYBOY_R2  + BUGGYBOY_R3  + BUGGYBOY_R4 ) + 1.0/100E3)/100E3,
	-1.0/(1.0/(BUGGYBOY_R1S + BUGGYBOY_R2  + BUGGYBOY_R3  + BUGGYBOY_R4S) + 1.0/100E3)/100E3,
	-1.0/(1.0/(BUGGYBOY_R1S + BUGGYBOY_R2  + BUGGYBOY_R3S + BUGGYBOY_R4 ) + 1.0/100E3)/100E3,
	-1.0/(1.0/(BUGGYBOY_R1S + BUGGYBOY_R2  + BUGGYBOY_R3S + BUGGYBOY_R4S) + 1.0/100E3)/100E3,
	-1.0/(1.0/(BUGGYBOY_R1S + BUGGYBOY_R2S + BUGGYBOY_R3  + BUGGYBOY_R4 ) + 1.0/100E3)/100E3,
	-1.0/(1.0/(BUGGYBOY_R1S + BUGGYBOY_R2S + BUGGYBOY_R3  + BUGGYBOY_R4S) + 1.0/100E3)/100E3,
	-1.0/(1.0/(BUGGYBOY_R1S + BUGGYBOY_R2S + BUGGYBOY_R3S + BUGGYBOY_R4 ) + 1.0/100E3)/100E3,
	-1.0/(1.0/(BUGGYBOY_R1S + BUGGYBOY_R2S + BUGGYBOY_R3S + BUGGYBOY_R4S) + 1.0/100E3)/100E3,
};


/*
	8253 Programmable Interval Timer
*/
static struct
{
	union
	{
#ifdef LSB_FIRST
		struct { UINT8 LSB; UINT8 MSB; };
#else
		struct { UINT8 MSB; UINT8 LSB; };
#endif
		UINT16 val;
	} counts[3];

	int idx[3];
} pit8253;

/*
	Port A                     Port B
	======                     ======

	0: Engine 1 gain (FR) #0    0: Coin Counter 1
	1: Engine 1 gain (FR) #1    1: Coin Counter 2
	2: Engine 1 gain (FR) #2    2: Coin Counter 3 (Unused)
	3: Engine 1 gain (FR) #3    3: Engine 0 gain
	4: Engine 1 gain (FL) #0    4: Noise EN1
	5: Engine 1 gain (FL) #1    5: Noise EN2
	6: Engine 1 gain (FL) #2    6: Enable YM IC24 output on RR
	7: Engine 1 gain (FL) #3    7: Enable YM IC19 output on RL


	The engine sounds are generated by an 8253. There are two channels.

	#0 is the player's buggy
	#1 is the opponents' buggies

	          +------------> GAIN[1] +--> FL
	          |                      +--> FR
    8255 #0 --+--> BL	           
	          +--> BR

	8255 #1 --+--> GAIN[2] ---> FL
	          +--> GAIN[3] ---> FR


    [1] is used to amplify sound during tunnel.
	[2] and [3] are stereo fades

*/

WRITE8_HANDLER( bb_ym1_a_w )
{
	stream_update(stream);

	ym_outputa = data ^ 0xff;
}

WRITE8_HANDLER( bb_ym1_b_w )
{
	double gain;

	stream_update(stream);

	ym_outputb = data ^ 0xff;

    coin_counter_w(0, data & 0x01);
    coin_counter_w(1, data & 0x02);

	/* Until we support > 2 speakers, double the gain of the front speakers */

	/* Rear left speaker */
	gain = data & 0x80 ? 1.0 : 2.0;
	sndti_set_output_gain(SOUND_AY8910, 0, 0, gain);
	sndti_set_output_gain(SOUND_AY8910, 0, 1, gain);
	sndti_set_output_gain(SOUND_AY8910, 0, 2, gain);

	/* Rear right speaker */
	gain = data & 0x40 ? 1.0 : 2.0;
	sndti_set_output_gain(SOUND_AY8910, 1, 0, gain);
	sndti_set_output_gain(SOUND_AY8910, 1, 1, gain);
	sndti_set_output_gain(SOUND_AY8910, 1, 2, gain);
}


WRITE8_HANDLER( pit8253_w )
{
	stream_update(stream);

	if (offset < 3)
	{
		if (pit8253.idx[offset] == 0)
		{
			pit8253.counts[offset].LSB = data;
			pit8253.idx[offset] = 1;
		}
		else
		{
			pit8253.counts[offset].MSB = data;
			pit8253.idx[offset] = 0;
		}
	}
	else
	{
		int mode = (data >> 1) & 7;

		if (mode == 3)
		{
			int cntsel = (data >> 6) & 3;
			pit8253.idx[cntsel] = 0;
			pit8253.counts[cntsel].val = 0;
		}
		else
			mame_printf_debug("PIT8253: Unsupported mode %d.\n", mode);

	}
}

READ8_HANDLER( pit8253_r )
{
	mame_printf_debug("PIT R: %x", offset);
	return 0;
}

static void tx1_stream_update(void *param, stream_sample_t **inputs, stream_sample_t **buffer, int length)
{

}

/*
	This is admittedly a bit of a hack job...
*/
static void buggyboy_stream_update(void *param, stream_sample_t **inputs, stream_sample_t **buffer, int length)
{
	int step_0, step_1;
	int n1_en, n2_en;
	double gain0, gain1_l, gain1_r;

	stream_sample_t *fl = &buffer[0][0];
	stream_sample_t *fr = &buffer[1][0];

	/* Clear the buffers */
	memset(buffer[0], 0, length * sizeof(*buffer[0]));
	memset(buffer[1], 0, length * sizeof(*buffer[1]));

	/* 8253 outputs for the player/opponent buggy engine sounds. */
	step_0 = pit8253.counts[0].val ? (BUGGYBOY_PIT_CLOCK / pit8253.counts[0].val) * freq_to_step : 0;
	step_1 = pit8253.counts[1].val ? (BUGGYBOY_PIT_CLOCK / pit8253.counts[1].val) * freq_to_step : 0;

	gain0 = BIT(ym_outputb, 3) ? 1.0 : 2.0;
	n1_en = BIT(ym_outputb, 4);
	n2_en = BIT(ym_outputb, 5);

	gain1_l = bb_engine_gains[ym_outputa >> 4] * 5;
	gain1_r = bb_engine_gains[ym_outputa & 0xf] * 5;

	while (length--)
	{
		int i;
		stream_sample_t pit0, pit1, n1, n2;		
		pit0 = buggyboy_eng_voltages[(step0 >> 24) & 0xf];
		pit1 = buggyboy_eng_voltages[(step1 >> 24) & 0xf];

		/* Calculate the tyre screech noise source */
		for (i = 0; i < BUGGYBOY_NOISE_CLOCK / Machine->sample_rate; ++i)
		{
			/* CD4006 is a 4-4-1-4-4-1 shift register */
			int p13 = BIT(noise_lfsra, 3);
			int p12 = BIT(noise_lfsrb, 4);
			int p10 = BIT(noise_lfsrc, 3);
			int p8 = BIT(noise_lfsrd, 3);

			/* Update the register */
			noise_lfsra = p12 | ((noise_lfsra << 1) & 0xf);
			noise_lfsrb = (p8 ^ p12) | ((noise_lfsrb << 1) & 0x1f);
			noise_lfsrc = p13 | ((noise_lfsrc << 1) & 0xf);
			noise_lfsrd = p10 | ((noise_lfsrd << 1) & 0x1f);

			/* 4040 12-bit counter is clocked on the falling edge of Q13 */
			if ( !BIT(noise_lfsrc, 3) && p10 )
				noise_counter = (noise_counter + 1) & 0x0fff;
		}

		if (n1_en)
		{
			n1 = !BIT(noise_counter, 7-1) * 16000;
			if ( BIT(noise_counter, 11-1) ) n1 /=2;
		}
		else
			n1 = 8192;

		if (n2_en)
		{
			n2 = !BIT(noise_counter, 6-1) * 16000;
			if ( BIT(noise_counter, 11-1) ) n2 /=2;
		}
		else
			n2 = 8192;

		*fl++ = n1 + n2 + (pit0 * gain0) + (pit1 * gain1_l);
		*fr++ = n1 + n2 + (pit0 * gain0) + (pit1 * gain1_r);

		step0 += step_0;
		step1 += step_1;
	}
}

void *buggyboy_sh_start(int clock, const struct CustomSound_interface *config)
{
	static const int resistors[4] = { 330000, 220000, 330000, 220000 };
	double aweights[4];
	int i;
	static const int tmp[16] =
	{
		0x0, 0x1, 0xe, 0xf, 0x8, 0x9, 0x6, 0x7, 0xc, 0xd, 0xe, 0xf, 0x4, 0x5, 0x6, 0x7
	};

	compute_resistor_weights(0,	16384,	-1.0,
							4,	&resistors[0], aweights, 0,	0,
							0, 0, 0, 0, 0,
							0, 0, 0, 0, 0 );

	for (i = 0; i < 16; i++)
		buggyboy_eng_voltages[i] = combine_4_weights(aweights, BIT(tmp[i], 0), BIT(tmp[i], 1), BIT(tmp[i], 2), BIT(tmp[i], 3));

	/* Allocate the stream */
	stream = stream_create(0, 2, Machine->sample_rate, NULL, buggyboy_stream_update);

	freq_to_step = (double)(1 << 24) / (double)Machine->sample_rate;
	step0 = step1 = 0;

	return auto_malloc(1);
}

void buggyboy_sh_reset(void *token)
{
	step0 = step1 = 0;
	
	/* Reset noise LFSR */
	noise_lfsra = 0;
	noise_lfsrb = 1;
	noise_lfsrc = 0;
	noise_lfsrd = 0;	
}

void *tx1_sh_start(int clock, const struct CustomSound_interface *config)
{
	/* Allocate the stream */
	stream = stream_create(0, 2, Machine->sample_rate, NULL, tx1_stream_update);

	freq_to_step = (double)(1 << 24) / (double)Machine->sample_rate;
	step0 = 0;
	step1 = 0;

	return auto_malloc(1);
}

void tx1_sh_reset(void *token)
{
}

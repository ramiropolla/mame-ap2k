// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    clifront.h

    Command-line interface frontend for MAME.

***************************************************************************/

#pragma once

#ifndef __CLIFRONT_H__
#define __CLIFRONT_H__

#include "emuopts.h"
#include "drivenum.h"


//**************************************************************************
//  CONSTANTS
//**************************************************************************

// core commands
#define CLICOMMAND_HELP                 "help"
#define CLICOMMAND_VALIDATE             "validate"

// configuration commands
#define CLICOMMAND_CREATECONFIG         "createconfig"
#define CLICOMMAND_SHOWCONFIG           "showconfig"
#define CLICOMMAND_SHOWUSAGE            "showusage"

// frontend commands
#define CLICOMMAND_LISTXML              "listxml"
#define CLICOMMAND_LISTFULL             "listfull"
#define CLICOMMAND_LISTSOURCE           "listsource"
#define CLICOMMAND_LISTCLONES           "listclones"
#define CLICOMMAND_LISTBROTHERS         "listbrothers"
#define CLICOMMAND_LISTCRC              "listcrc"
#define CLICOMMAND_LISTROMS             "listroms"
#define CLICOMMAND_LISTSAMPLES          "listsamples"
#define CLICOMMAND_VERIFYROMS           "verifyroms"
#define CLICOMMAND_VERIFYSAMPLES        "verifysamples"
#define CLICOMMAND_ROMIDENT             "romident"
#define CLICOMMAND_LISTDEVICES          "listdevices"
#define CLICOMMAND_LISTSLOTS            "listslots"
#define CLICOMMAND_LISTMEDIA            "listmedia"     // needed by MESS
#define CLICOMMAND_LISTSOFTWARE         "listsoftware"
#define CLICOMMAND_VERIFYSOFTWARE       "verifysoftware"
#define CLICOMMAND_GETSOFTLIST          "getsoftlist"
#define CLICOMMAND_VERIFYSOFTLIST       "verifysoftlist"
#define CLICOMMAND_LIST_MIDI_DEVICES    "listmidi"


//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// cli_options wraps the general emu options with CLI-specific additions
class cli_options : public emu_options
{
public:
	// construction/destruction
	cli_options();

private:
	static const options_entry s_option_entries[];
};


// cli_frontend handles command-line processing and emulator execution
class cli_frontend
{
	typedef tagmap_t<FPTR> int_map;
public:
	// construction/destruction
	cli_frontend(cli_options &options, osd_interface &osd);
	~cli_frontend();

	// execute based on the incoming argc/argv
	int execute(int argc, char **argv);

	// direct access to the command operations
	void listxml(const char *gamename = "*");
	void listfull(const char *gamename = "*");
	void listsource(const char *gamename = "*");
	void listclones(const char *gamename = "*");
	void listbrothers(const char *gamename = "*");
	void listcrc(const char *gamename = "*");
	void listroms(const char *gamename = "*");
	void listsamples(const char *gamename = "*");
	static int compare_devices(const void *i1, const void *i2);
	void listdevices(const char *gamename = "*");
	void listslots(const char *gamename = "*");
	void listmedia(const char *gamename = "*");
	void listsoftware(const char *gamename = "*");
	void verifysoftware(const char *gamename = "*");
	void verifyroms(const char *gamename = "*");
	void verifysamples(const char *gamename = "*");
	void romident(const char *filename);
	void getsoftlist(const char *gamename = "*");
	void verifysoftlist(const char *gamename = "*");
	void listmididevices(const char *gamename = "*");

private:
	// internal helpers
	void execute_commands(const char *exename);
	void display_help();
	void display_suggestions(const char *gamename);
	void output_single_softlist(FILE *out,software_list *list, const char *listname);

	// internal state
	cli_options &       m_options;
	osd_interface &     m_osd;
	int                 m_result;
};


// media_identifier class identifies media by hash via a search in
// the driver database
class media_identifier
{
public:
	// construction/destruction
	media_identifier(cli_options &options);

	// getters
	int total() const { return m_total; }
	int matches() const { return m_matches; }
	int nonroms() const { return m_nonroms; }

	// operations
	void reset() { m_total = m_matches = m_nonroms = 0; }
	void identify(const char *name);
	void identify_file(const char *name);
	void identify_data(const char *name, const UINT8 *data, int length);
	int find_by_hash(const hash_collection &hashes, int length);

private:
	// internal state
	driver_enumerator   m_drivlist;
	int                 m_total;
	int                 m_matches;
	int                 m_nonroms;
};



#endif  /* __CLIFRONT_H__ */

/***************************************************************************

    webengine.c

    Handle MAME internal web server.

****************************************************************************

    Copyright Miodrag Milanovic
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

        * Redistributions of source code must retain the above copyright
          notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
          notice, this list of conditions and the following disclaimer in
          the documentation and/or other materials provided with the
          distribution.
        * Neither the name 'MAME' nor the names of its contributors may be
          used to endorse or promote products derived from this software
          without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY MIODRAG MILANOVIC ''AS IS'' AND ANY EXPRESS OR
    IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL MIODRAG MILANOVIC BE LIABLE FOR ANY DIRECT,
    INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
    HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
    STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
    IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

***************************************************************************/

#include "emu.h"
#include "emuopts.h"
#include "ui.h"
#include "webengine.h"
#include "web/mongoose.h"
#include "web/json/json.h"

//**************************************************************************
//  WEB ENGINE
//**************************************************************************

void web_engine::websocket_ready_handler(struct mg_connection *conn) {	
	static const char *message = "update_machine";
	mg_websocket_write(conn, WEBSOCKET_OPCODE_TEXT, message, strlen(message));
	m_websockets.append(*global_alloc(simple_list_wrapper<mg_connection>(conn)));
} 

// Arguments:
//   flags: first byte of websocket frame, see websocket RFC,
//          http://tools.ietf.org/html/rfc6455, section 5.2
//   data, data_len: payload data. Mask, if any, is already applied.
int web_engine::websocket_data_handler(struct mg_connection *conn, int flags,
                                  char *data, size_t data_len) 
{
	// just Echo example for now
	if ((flags & 0x0f) == WEBSOCKET_OPCODE_TEXT)
		mg_websocket_write(conn, WEBSOCKET_OPCODE_TEXT, data, data_len);

	// Returning zero means stoping websocket conversation.
	// Close the conversation if client has sent us "exit" string.
	return memcmp(data, "exit", 4);
} 

static void get_qsvar(const struct mg_request_info *request_info,
                      const char *name, char *dst, size_t dst_len) {
  const char *qs = request_info->query_string;
  mg_get_var(qs, strlen(qs == NULL ? "" : qs), name, dst, dst_len);
}

int web_engine::json_game_handler(struct mg_connection *conn) 
{
	Json::Value data;
	data["name"] = m_machine->system().name;
	data["description"] = m_machine->system().description;
	data["year"] = m_machine->system().year;
	data["manufacturer"] = m_machine->system().manufacturer;
	data["parent"] = m_machine->system().parent;
	data["source_file"] = m_machine->system().source_file;
	data["flags"] = m_machine->system().flags;

	Json::FastWriter writer;
	const char *json = writer.write(data).c_str();
	// Send HTTP reply to the client
	mg_printf(conn,
			"HTTP/1.1 200 OK\r\n"
			"Content-Type: application/json\r\n"
			"Content-Length: %d\r\n"        // Always set Content-Length
			"\r\n"
			"%s",
			(int)strlen(json), json);

	// Returning non-zero tells mongoose that our function has replied to
	// the client, and mongoose should not send client any more data.
	return 1;
}

int web_engine::json_slider_handler(struct mg_connection *conn) 
{
	const slider_state *curslider;
	astring tempstring;

	/* add all sliders */
	for (curslider = ui_get_slider_list(); curslider != NULL; curslider = curslider->next)
	{
		INT32 curval = (*curslider->update)(machine(), curslider->arg, &tempstring, SLIDER_NOCHANGE);
		printf("%d\n",curval);
/*		UINT32 flags = 0;
		if (curval > curslider->minval)
			flags |= MENU_FLAG_LEFT_ARROW;
		if (curval < curslider->maxval)
			flags |= MENU_FLAG_RIGHT_ARROW;
		item_append(curslider->description, tempstring, flags, (void *)curslider);

		if (menuless_mode)
			break;*/
	}

	/* add all sliders */
	for (curslider = (slider_state*)machine().osd().get_slider_list(); curslider != NULL; curslider = curslider->next)
	{
		INT32 curval = (*curslider->update)(machine(), curslider->arg, &tempstring, SLIDER_NOCHANGE);
		printf("%d\n",curval);
		/*UINT32 flags = 0;
		if (curval > curslider->minval)
			flags |= MENU_FLAG_LEFT_ARROW;
		if (curval < curslider->maxval)
			flags |= MENU_FLAG_RIGHT_ARROW;
		item_append(curslider->description, tempstring, flags, (void *)curslider);*/
	}
	return 1;
}

// This function will be called by mongoose on every new request.
int web_engine::begin_request_handler(struct mg_connection *conn) 
{
	const struct mg_request_info *request_info = mg_get_request_info(conn);
	if (!strncmp(request_info->uri, "/json/",6)) 
	{
		if (!strcmp(request_info->uri, "/json/game")) 
		{
			return json_game_handler(conn);
		}		
		if (!strcmp(request_info->uri, "/json/slider")) 
		{
			return json_slider_handler(conn);
		}		
	}
	else if (!strncmp(request_info->uri, "/cmd",4)) 
	{
		char cmd_name[64];
		get_qsvar(request_info, "name", cmd_name, sizeof(cmd_name));
		
		if(!strcmp(cmd_name,"softreset"))
		{
			m_machine->schedule_soft_reset();
		} 
		else if(!strcmp(cmd_name,"hardreset"))
		{
			m_machine->schedule_hard_reset();
		}
		else if(!strcmp(cmd_name,"exit"))
		{
			m_machine->schedule_exit();
		}		
		
		// Send HTTP reply to the client
		mg_printf(conn,
				"HTTP/1.1 200 OK\r\n"
				"Content-Type: text/plain\r\n"
				"Content-Length: 2\r\n"        // Always set Content-Length
				"\r\n"
				"OK");

		// Returning non-zero tells mongoose that our function has replied to
		// the client, and mongoose should not send client any more data.
		return 1;
	}
	else if (!strncmp(request_info->uri, "/screenshot.png",15)) 
	{
		screen_device_iterator iter(m_machine->root_device());
		screen_device *screen = iter.first();

		if (screen == NULL)
		{
			return 0;
		}

		astring fname("screenshot.png");
		emu_file file(m_machine->options().snapshot_directory(), OPEN_FLAG_WRITE | OPEN_FLAG_CREATE | OPEN_FLAG_CREATE_PATHS);
		file_error filerr = file.open(fname);

		if (filerr != FILERR_NONE)
		{
			return 0;
		}

		m_machine->video().save_snapshot(screen, file);
		astring fullpath(file.fullpath());
		file.close();
		
		mg_send_file(conn,fullpath);
		return 1;
	}
	return 0;
} 


void *web_engine::websocket_keepalive() 
{
	while(!m_exiting_core) 
	{
		osd_ticks_t curtime = osd_ticks();	
		if ((curtime - m_lastupdatetime) > osd_ticks_per_second() * 5)
		{
			m_lastupdatetime = curtime;
			for (simple_list_wrapper<mg_connection> *curitem = m_websockets.first(); curitem != NULL; curitem = curitem->next())
			{
				int status = mg_websocket_write(curitem->object(), WEBSOCKET_OPCODE_PING, NULL, 0);		
				if (status==0) m_websockets.detach(*curitem); // remove inactive clients
			}
		}
		osd_sleep(osd_ticks_per_second()/5);
	}
	return NULL;
}

//-------------------------------------------------
//  static callbacks
//-------------------------------------------------
static void websocket_ready_handler_static(struct mg_connection *conn)
{
	const struct mg_request_info *request_info = mg_get_request_info(conn);
    web_engine *engine = static_cast<web_engine *>(request_info->user_data);
	engine->websocket_ready_handler(conn);
}

static int websocket_data_handler_static(struct mg_connection *conn, int flags,
                                  char *data, size_t data_len) 
{
	const struct mg_request_info *request_info = mg_get_request_info(conn);
    web_engine *engine = static_cast<web_engine *>(request_info->user_data);
	return engine->websocket_data_handler(conn, flags, data, data_len);
}

static int begin_request_handler_static(struct mg_connection *conn) 
{
	const struct mg_request_info *request_info = mg_get_request_info(conn);	
    web_engine *engine = static_cast<web_engine *>(request_info->user_data);
	return engine->begin_request_handler(conn);
}

static void *websocket_keepalive_static(void *thread_func_param) 
{
	web_engine *engine = static_cast<web_engine *>(thread_func_param);
	return engine->websocket_keepalive();
}

//-------------------------------------------------
//  web_engine - constructor
//-------------------------------------------------

web_engine::web_engine(emu_options &options)
	: m_options(options),
	  m_machine(NULL),
	  m_ctx(NULL),
	  m_lastupdatetime(0),
	  m_exiting_core(false)
	
{
	
	struct mg_callbacks callbacks;

	// List of options. Last element must be NULL.
	const char *web_options[] = {
		"listening_ports", options.http_port(), 
		"document_root", options.http_path(),
		NULL
	};

	// Prepare callbacks structure. 
	memset(&callbacks, 0, sizeof(callbacks));
	callbacks.begin_request = begin_request_handler_static;
    callbacks.websocket_ready = websocket_ready_handler_static;
    callbacks.websocket_data = websocket_data_handler_static;	

	// Start the web server.
	if (m_options.http()) {
		m_ctx = mg_start(&callbacks, this, web_options);
		
		mg_start_thread(websocket_keepalive_static, this);
	}
	
}

//-------------------------------------------------
//  ~web_engine - destructor
//-------------------------------------------------

web_engine::~web_engine()
{
	if (m_options.http())
		close();
}

//-------------------------------------------------
//  close - close and cleanup of lua engine
//-------------------------------------------------

void web_engine::close()
{
	m_exiting_core = 1;
	osd_sleep(osd_ticks_per_second()/5);
	for (simple_list_wrapper<mg_connection> *curitem = m_websockets.first(); curitem != NULL; curitem = curitem->next())
	{
		mg_websocket_write(curitem->object(), WEBSOCKET_OPCODE_CONNECTION_CLOSE, NULL, 0);
	}
	// Stop the server.	
	mg_stop(m_ctx);
}


void web_engine::push_message(const char *message)
{
	for (simple_list_wrapper<mg_connection> *curitem = m_websockets.first(); curitem != NULL; curitem = curitem->next())
	{		
		int status = mg_websocket_write(curitem->object(), WEBSOCKET_OPCODE_TEXT, message, strlen(message));
		if (status==0) m_websockets.detach(*curitem); // remove inactive clients
	}
}
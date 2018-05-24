#ifndef __BUTTON_H__
#define __BUTTON_H__

#ifdef BUTTON_DEBUG_LOG
#deifne BUTTON_LOG 			app_trace_log
#else
#define BUTTON_LOG(...)
#endif

void button_init(void);
	
#endif

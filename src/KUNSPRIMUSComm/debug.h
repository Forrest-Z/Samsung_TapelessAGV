#ifndef _DEBUG_H_
#define _DEBUG_H_

int init_debug( char *file, bool bSaveLogFile = false );
int deinit_debug(void);
int debug(char *format, ...);
int	PrintMessage (const char *format, ...);	// LogMessage without time
char *extract_file_name(char *path);
char *_get_current_time( BOOL bFullDisplay );

#endif


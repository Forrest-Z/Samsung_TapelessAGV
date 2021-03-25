#include "stdafx.h"
#include "Windows.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "debug.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define DEBUG_FILE_SIZE_LIMIT		1000000

extern int ShowMessage(char *buff, int n_buff);

static char debug_file[256+1] = "debug.log";
static	BOOL	bDebugInitialized = FALSE;
static	bool	bSaveLogFile = true;
//static CRITICAL_SECTION hCrtclSect;

/*
 * ������ ũ�Ⱑ DEBUG_FILE_SIZE_LIMIT ���� ū ���
 * ���� ���Ͽ� ".old" Ȯ���ڸ� �ٿ��� ����� �� ����
 * ������ ��� ������ �����Ѵ�.
 */
int _limit_file_size(char *file)
{
	char file_name_old[257];
	struct stat buf[1];

	if(stat(file,buf)<0){					/* ������ ���¸� �о�� */
		return -1;
	}
	if(buf->st_size > DEBUG_FILE_SIZE_LIMIT)
	{
		/* ������ ���Ѱ��� �Ѿ �� */
		sprintf_s( file_name_old, "%.256s.old", file );
		remove(file_name_old);					/* .old ������ �����ϸ� ���� */
		rename(file,file_name_old);			/* .old ���Ϸ� ���� */

		return 1;
	}
	return 0;
}

char *_get_current_time( BOOL bFullDisplay )
{
	SYSTEMTIME	stNow;
	static char buff[32];

	// mili second�� ǥ���ϱ� ���Ͽ� SYSTEMTIME ����ü ���
	GetLocalTime( &stNow );
	if( bFullDisplay == TRUE )
 		sprintf_s( buff, "[%04i/%02i/%02i %02i:%02i:%02i-%03i] ",
			stNow.wYear,stNow.wMonth,stNow.wDay,stNow.wHour,stNow.wMinute,stNow.wSecond, stNow.wMilliseconds );
	else
 		sprintf_s( buff, "[%02i:%02i:%02i-%03i] ",
			stNow.wHour,stNow.wMinute,stNow.wSecond, stNow.wMilliseconds );
	return buff;
}

/* 
 * ����� ������ ����� ������ file �̸����� �����.
 * file �� NULL �ΰ�� "debug.log" ������ �����Ѵ�.
 */
int init_debug(char *file, bool bLogFile )
{
	int ret;
	FILE *fp;

	// InitializeCriticalSection(&hCrtclSect);
	bSaveLogFile = bLogFile;
	if( bSaveLogFile == false )
	{
		bDebugInitialized = TRUE;
		return -1;
	}
	if(file){
		if(*file){
			strncpy_s(debug_file,file,256);
			debug_file[256] = '\0';
		}
	}

	_limit_file_size(debug_file);

	fopen_s(&fp, debug_file,"a+");
	if(fp){
 		ret=fprintf(fp, "\n ========== %s ========== \n",
			_get_current_time( TRUE ));
		fclose(fp);
		bDebugInitialized = TRUE;
		return ret;
	}

	return -1;
}

int deinit_debug(void)
{
	// DeleteCriticalSection(&hCrtclSect);
	bDebugInitialized = FALSE;
	return 0;
}

/*
 * ���Ͽ� ����� ������ ����Ѵ�. 
 * ��ϵǴ� ������ "[DD HH:MM:SS] message" �� ����.
 * printf�� ������ ����.
 */
int debug(char *format, ...)
{
	#define MAX_QUEUEBUFF	32
	
	int n;
	FILE *fp;
    va_list arg_list;
	char buff[1024];

	if( bDebugInitialized != TRUE )
	{
		init_debug( 0 );	// Initialize default debug file
	}
	if( bSaveLogFile == true )
		_limit_file_size(debug_file);

 	n = 0;
	n += sprintf_s(buff + n,1024-n,"%s", _get_current_time(TRUE) );
	va_start (arg_list,format);
	n += vsprintf_s(buff + n,1024-n, format, arg_list);
	va_end (arg_list);

	ShowMessage (buff, n);

	if( bSaveLogFile == true )
	{
		fopen_s(&fp, debug_file,"a+");
		if(fp){
			fprintf (fp, buff);
			fprintf (fp, "\n");
			fclose(fp);
		}
	}

	return 0;
} // end of debug()

int PrintMessage( const char *format, ...)
{
    va_list arg_list;
	char buff[1024];
 	int n = 0;

	va_start( arg_list,format );
	n += vsprintf_s(buff + n,1024-n, format, arg_list);
	va_end( arg_list );

	ShowMessage (buff, n);

	return 0;
} // end of PrintMessage()

/*
 * ���� �н����� ���� �̸��� ���۵Ǵ� ��ġ�� �����͸� ã�´�.
 * ã�� ���� ���� ���� �н��� ó�� ��ġ�� �����Ѵ�.
 */
char *extract_file_name(char *path)
{
	char *p;
	char *file;

	file = path;

	p = path;
	while(*p){
		if(*p=='/' || *p=='\\') file = p+1;
		p++;
	}

	return file;
}

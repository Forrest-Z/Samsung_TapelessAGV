#ifndef _DATA_CNV_H_
#define _DATA_CNV_H_

#include "Windows.h"
#include "time.h"

extern	unsigned char HEXCH2NUM_TABLE[256];
extern	unsigned char NUM2HEXCH_TABLE[16];

#define NUM2HEXCH(n)	(NUM2HEXCH_TABLE[(unsigned char)(n)&0x0F])
#define HEXCH2NUM(x)	(HEXCH2NUM_TABLE[(unsigned char)(x)&0x7F])

extern	unsigned char BCD2HEX(unsigned char bcd);
extern	unsigned char HEX2BCD(unsigned char hex);

extern	void ltohex(long num, char *buff, int n);
extern	long hextol(char *buff, int n);
extern	long hextol_e(char *buff, int n);

extern	int atoi_n(char *str, int length);

extern	unsigned short	GetSwapShort( unsigned short usData );
extern	unsigned long	GetSwapLong( unsigned long ulData );
extern	unsigned short	ReverseShort( unsigned short netshort );
extern	unsigned long	ReverseLong( unsigned long netlong );
extern	unsigned __int64 ReverseHyper( unsigned __int64 nethyper );
extern	int		StringToInt( char *str );
extern  long	StringToLong( char *str );
extern	int		BinStrToInt( char* pszBuff, int iCount );
extern	int		WritePrivateProfileInt(TCHAR* lpAppName, TCHAR* lpKeyName, int value, TCHAR* lpFileName);//2017
extern	void	LowerNibbleToBinStr( unsigned char ucValue, char *pszBuff );
extern	void	UpperNibbleToBinStr( unsigned char ucValue, char *pszBuff );
extern	void	CharToBinStr( unsigned char ucValue, char *pszBuff );
extern	void	ShortToBinStr( short sValue, char *pszBuff );
extern	void	LongToBinStr( long lValue, char *pszBuff );
extern	void	UshortToHexStr( unsigned short usValue, char *pszBuff );
extern	void	UcharToHexStr( unsigned char ucValue, char *pszBuff );
extern	void	ByteToHexStr( unsigned char* pszInput, int iSize, char* pszOutput );
extern	long	RoundOff( double dInput );

extern int	CheckFileExist (TCHAR *pFileName);//2017
extern int	CheckFolderExist (TCHAR *pFolderName);//2017

/*
 * pString의 문자열이 Recipe ID로 사용가능한지 체크한다.
 * 체크하는 character는 다음과 같다.
 *   숫자 0~9
 *   문자 A~Z, a~z
 *   기호 _
 */
extern int	CheckCharIntegrity (char *pString);
extern int IsValidRecipeID( char *pString );

/*
 * 디렉토리와 file 이름을  합쳐서 하나의 문자열로 만든다.
 */
extern char *pMergeDirectoryAndFileName(char *path, char *file_name);

extern TCHAR *pStrDateTime (long t);//2017
extern TCHAR *pStrDateTime2 (long t);//2017
extern TCHAR *pStrDateTimeShort (long t);//2017
extern time_t tDateTime (char *date_time);
extern time_t tGetTime( const char *pszTimeStr, int len );
#endif

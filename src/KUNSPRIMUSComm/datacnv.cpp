#include "StdAfx.h"
#include "stdio.h"
#include "math.h"
#include "datacnv.h"

/* 
 * Data conversion routines
 *
 * These routines convert data from one form to another. 
 */

unsigned char HEXCH2NUM_TABLE[256] = {
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 
	0 , 1 , 2 , 3 , 4 , 5 , 6 , 7 , 8 , 9 , -1, -1, -1, -1, -1, -1, 
	-1, 10, 11, 12, 13, 14, 15, -1, -1, -1, -1, -1, -1, -1, -1, -1, 
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 
	-1, 10, 11, 12, 13, 14, 15, -1, -1, -1, -1, -1, -1, -1, -1, -1, 
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
};

unsigned char NUM2HEXCH_TABLE[16] = {
	'0', '1', '2', '3', '4', '5', '6', '7',
	'8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
};

char m_szReturn[25];

#define NUM2HEXCH(n)	(NUM2HEXCH_TABLE[(unsigned char)(n)&0x0F])
#define HEXCH2NUM(x)	(HEXCH2NUM_TABLE[(unsigned char)(x)&0x7F])

unsigned char BCD2HEX(unsigned char bcd)
{
	return ((bcd>>4)&0x0F)*10 + ((bcd>>0)&0x0F);
}

unsigned char HEX2BCD(unsigned char hex)
{
	return (hex/10)*16 + (hex%10)*1;
}

void ltohex(long num, char *buff, int n)
{
	int i;

	for (i=n-1; 0<=i; i--) {
		buff[i] = NUM2HEXCH(num);
		num >>= 4;
	}
}

long hextol(char *buff, int n)
{
	int i;
	long v;
	long val;

	if( n >= 32 ) return 0;
	val = 0;
	for (i=0; i<n && *buff; i++)
	{
		v = HEXCH2NUM(*buff);
		if (0 <= v)
		{
			val *= 16;
			val += v;
		}else{
			/* Not a hexa code character */
			return 0;
		}
		buff++;
	}
	return val;
}

long hextol_e(char *buff, int n)
{
	int i;
	long v;
	long val;

	if( n >= 32 ) return 0;
	val = 0;
	for (i=0; i<n && *buff; i++)
	{
		v = HEXCH2NUM(*buff);
		if (0 <= v)
		{
			val *= 16;
			val += v;
		}else{
			/* Not a hexa code character */
			return -1;
		}
		buff++;
	}
	return val;
}

int atoi_n (char *str, int n)
{
	int val = 0;

	for (int i=0; i<n; i++, str++) {
		if (isdigit (*str)) {
			val *= 10;
			val += *str - '0';
		}
		else {
			break;
		}
	}
	return val;
}

unsigned short GetSwapShort( unsigned short usData )
{
	unsigned char*	pucData = (unsigned char*)&usData;
	unsigned short*	pusData = (unsigned short*)pucData;
	unsigned char	ucBuff;

	ucBuff = pucData[1];
	pucData[1] = pucData[0];
	pucData[0] = ucBuff;

	return (*pusData);
} // end of GetSwapShort()

unsigned long GetSwapLong( unsigned long ulData )
{
	unsigned char*	pucData = (unsigned char*)&ulData;
	unsigned long*	pulData = (unsigned long*)pucData;
	unsigned char	ucBuff;

	ucBuff = pucData[3];
	pucData[3] = pucData[0];
	pucData[0] = ucBuff;

	ucBuff = pucData[2];
	pucData[2] = pucData[1];
	pucData[1] = ucBuff;

	return (*pulData);
} // end of GetSwapLong()

unsigned short ReverseShort( unsigned short netshort )
{
	unsigned short hostshort = 0;

	*(((UCHAR *)&hostshort) + 0) = *(((UCHAR *)&netshort) + 1);
	*(((UCHAR *)&hostshort) + 1) = *(((UCHAR *)&netshort) + 0);
	return hostshort;
} // end of ReverseShort()

unsigned long ReverseLong( unsigned long netlong )
{
	unsigned long hostlong = 0UL;

	*(((UCHAR *)&hostlong) + 0) = *(((UCHAR *)&netlong) + 3);
	*(((UCHAR *)&hostlong) + 1) = *(((UCHAR *)&netlong) + 2);
	*(((UCHAR *)&hostlong) + 2) = *(((UCHAR *)&netlong) + 1);
	*(((UCHAR *)&hostlong) + 3) = *(((UCHAR *)&netlong) + 0);
	return hostlong;
} // end of ReverseLong()

unsigned __int64 ReverseHyper( unsigned __int64 nethyper )
{
	unsigned __int64 hosthyper = 0UI64;

	*(((UCHAR *)&hosthyper) + 0) = *(((UCHAR *)&nethyper) + 7);
	*(((UCHAR *)&hosthyper) + 1) = *(((UCHAR *)&nethyper) + 6);
	*(((UCHAR *)&hosthyper) + 2) = *(((UCHAR *)&nethyper) + 5);
	*(((UCHAR *)&hosthyper) + 3) = *(((UCHAR *)&nethyper) + 4);
	*(((UCHAR *)&hosthyper) + 4) = *(((UCHAR *)&nethyper) + 3);
	*(((UCHAR *)&hosthyper) + 5) = *(((UCHAR *)&nethyper) + 2);
	*(((UCHAR *)&hosthyper) + 6) = *(((UCHAR *)&nethyper) + 1);
	*(((UCHAR *)&hosthyper) + 7) = *(((UCHAR *)&nethyper) + 0);
	return hosthyper;
} // end of ReverseHyper()

int StringToInt( char *str )
{
	int val;

	val = 0;
	while (*str) {
		if (isdigit(*str)) {
			val *= 10;
			val += *str - '0';
		}
		str ++;
	}
	return val;
} // end of StringToInt()

long StringToLong( char *str )
{
	long val;

	val = 0;
	while (*str) {
		if (isdigit(*str)) {
			val *= 10;
			val += *str - '0';
		}
		str ++;
	}
	return val;
} // end of StringToLong()

int	BinStrToInt( char* pszBuff, int iCount )
{
	int i;
	int		iValue;

	iValue = 0;
	for( i = 0; *pszBuff; )
	{
		if( (*pszBuff) >= '0' && (*pszBuff) <= '1' )
			iValue += ((*pszBuff) - '0');
		if( ++i >= iCount || *++pszBuff == 0 ) break;
		iValue = iValue << 1;
	}
	return iValue;
} // end of BinStrToInt()

void LowerNibbleToBinStr( unsigned char ucValue, char *pszBuff )
{
	int i;
	unsigned char	ucMask = 0x08;

	for( i = 0; i < 4; i++ )
	{
		pszBuff[i] = (ucMask & ucValue) ? '1' : '0';
		ucMask = ucMask >> 1;
	}
} // end of LowerNibbleToBinStr()

void UpperNibbleToBinStr( unsigned char ucValue, char *pszBuff )
{
	int i;
	unsigned char	ucMask = 0x80;

	for( i = 0; i < 4; i++ )
	{
		pszBuff[i] = (ucMask & ucValue) ? '1' : '0';
		ucMask = ucMask >> 1;
	}
} // end of LowerNibbleToBinStr()

void CharToBinStr( unsigned char ucValue, char *pszBuff )
{
	int i;
	unsigned char	ucMask = 0x80;

	for( i = 0; i < 8; i++ )
	{
		pszBuff[i] = (ucMask & ucValue) ? '1' : '0';
		ucMask = ucMask >> 1;
	}
} // end of CharToBinStr()

void ShortToBinStr( short sValue, char *pszBuff )
{
	int i;
	unsigned	short	usMask = 0x8000;

	for( i = 0; i < 16; i++ )
	{
		pszBuff[i] = (usMask & sValue) ? '1' : '0';
		usMask >>= 1;
	}
} // end of ShortToBinStr()

void LongToBinStr( long lValue, char *pszBuff )
{
	int i;
	unsigned	long	ulMask = 0x80000000;

	for( i = 0; i < 32; i++ )
	{
		pszBuff[i] = (ulMask & lValue) ? '1' : '0';
		ulMask >>= 1;
	}
} // end of LongToBinStr()

int WritePrivateProfileInt(TCHAR* lpAppName, TCHAR* lpKeyName, int value, TCHAR* lpFileName)//2017
{
	TCHAR szBuff[256+1];//2017

	wsprintf( szBuff, TEXT("%i"), value );//2017
	return WritePrivateProfileString( lpAppName, lpKeyName, szBuff, lpFileName );//2017
} // end of WritePrivateProfileInt()

void UshortToHexStr( unsigned short usValue, char *pszBuff )
{
	unsigned char	ucTemp;

	ucTemp = (char)((usValue & 0xF000) >> 12);
	pszBuff[0] = NUM2HEXCH_TABLE[ucTemp];
	ucTemp = (char)((usValue & 0x0F00) >> 8);
	pszBuff[1] = NUM2HEXCH_TABLE[ucTemp];
	ucTemp = (char)((usValue & 0x00F0) >> 4);
	pszBuff[2] = NUM2HEXCH_TABLE[ucTemp];
	ucTemp = (char)((usValue & 0x000F));
	pszBuff[3] = NUM2HEXCH_TABLE[ucTemp];
} // end of UshortToHexStr()

void UcharToHexStr( unsigned char ucValue, char *pszBuff )
{
	unsigned char	ucTemp;

	ucTemp = (char)((ucValue & 0xF0) >> 4);
	pszBuff[0] = NUM2HEXCH_TABLE[ucTemp];
	ucTemp = (char)((ucValue & 0x0F));
	pszBuff[1] = NUM2HEXCH_TABLE[ucTemp];
} // end of UcharToHexStr()

void ByteToHexStr( unsigned char* pszInput, int iSize, char* pszOutput )
{
	int		i;

	for( i = 0; i < iSize; i++ )
	{
		UcharToHexStr( pszInput[i], &pszOutput[i * 3] );
		pszOutput[i * 3 + 2] = ' ';
	}
	pszOutput[i * 3] = 0;
} // end of ByteToHexStr()

long	RoundOff( double dInput )
{
	double	dTemp;
	long	lTemp;

	lTemp = (long)dInput;
	dTemp = fabs(dInput - lTemp);
	if( dTemp >= 0.5 )
		return	(dInput > 0 ? lTemp + 1 : lTemp - 1);
	else
		return	lTemp;
} // end of RoundOff()

/*
 * 파일이 존재하는지 체크한다.
 * 파일이 존재한다면 1, 존재하지 않는다면 0을 리턴한다.
 */
int CheckFileExist( TCHAR *pFileName )//2017
{
	int nReturn;
	HANDLE fhandle;
	WIN32_FIND_DATA ffblk;

	nReturn = 0;
	fhandle = FindFirstFile( pFileName, &ffblk );
	if( fhandle != INVALID_HANDLE_VALUE )
	{
		if( !(ffblk.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) )
		{
			nReturn = 1;
		}
		FindClose (fhandle);
	}
	return nReturn;
} // end of CheckFileExist()

/*
 * 폴더가 존재하는지 체크한다.
 * 폴더가 존재한다면 1, 존재하지 않는다면 0을 리턴한다.
 */
int CheckFolderExist( TCHAR *pFolderName )//2017
{
	int nReturn;
	HANDLE fhandle;
	WIN32_FIND_DATA ffblk;

	nReturn = 0;
	fhandle = FindFirstFile( pFolderName, &ffblk );
	if( fhandle != INVALID_HANDLE_VALUE )
	{
		if( (ffblk.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) )
		{
			nReturn = 1;
		}
		FindClose (fhandle);
	}
	return nReturn;
} // end of CheckFolderExist()

/*
 * pString의 문자열이 Recipe ID로 사용가능한지 체크한다.
 * 체크하는 character는 다음과 같다.
 *   숫자 0~9
 *   문자 A~Z, a~z
 *   기호 '_', '-'
 */
int CheckCharIntegrity (char *pString)
{
	for (; *pString; pString++)
	{
		if (!(isalpha (*pString) || isdigit(*pString) ||
			*pString == '_' || *pString == '-' ))
		{
			return 0;
		}
	}
	return 1;
} // end of CheckCharIntegrity()

/*
 * pString의 문자열이 확장된 Recipe ID로 사용가능한지 체크한다.
 * 체크하는 character는 다음과 같다.
 *   숫자 0~9
 *   문자 A~Z, a~z
 *   기호 '!', '#', '$', '%', '&', '`', '(', ')', '-', '@', '^', '_', ''', '{', '}', '~', ' '
 */
int IsValidRecipeID( char *pString )
{
	for (; *pString; pString++)
	{
		if (!(isalpha (*pString) || isdigit(*pString) ||
			*pString == '!' || *pString == '-' ||
			*pString == '@' || *pString == '^' ||
			*pString == '_' || *pString == '\'' ||
			(*pString >= 35 && *pString <= 41)	||
			*pString == '{' || *pString == '}' || *pString == '~' || *pString == ' '))
		{
			return 0;
		}
	}
	return 1;
} // end of IsValidRecipeID()

/*
 * 디렉토리와 file 이름을  합쳐서 하나의 문자열로 만든다.
 */
char *pMergeDirectoryAndFileName(char *path, char *file_name)
{
	int l;
	static char _file_path[MAX_PATH+1];

	// 파일 이름 앞에 있을 수 있는 디렉토리 구분기호('\')를 없앴다.
	if (file_name[0] == '\\') {
		file_name ++;
	}

	l = strlen(path);
	if (l == 0) {
		// 디렉토리 문자가 비어있는 경우 file 이름만 
		_snprintf_s (_file_path, MAX_PATH, "%s", file_name);
	}
	else if (path[l-1] == '\\') {
		// 디렉토리 끝에 디렉토리 구분기호('\')가 있을 때에는 
		// 디렉토리 끝에 파일 이름만 합친다.
		_snprintf_s (_file_path, MAX_PATH, "%s%s", path, file_name);
	}
	else {
		// 디렉토리 끝에 디렉토리 구분기호('\')가 없을 때에는 
		// 디렉토리 끝에 디렉토리 구분기호('\')와 파일 이름을 합친다.
		_snprintf_s (_file_path, MAX_PATH, "%s\\%s", path, file_name);
	}
	return _file_path;
} // end of pMergeDirectoryAndFileName()

/*
 * 시간 t 를 문자열 시간으로 변경한다.
 * 문자열 포멧은 다음과 같다.
 *   YYYY/MM/DD hh:mm:ss
 */
TCHAR *pStrDateTime( time_t t )//2017
{
	struct tm pt;
	static TCHAR buff[32];

	/*if( t )
	{
		pt = localtime_s(&t);
		if( pt )
		{
 			wsprintf(buff, "%04i/%02i/%02i %02i:%02i:%02i",
				1900+pt->tm_year,pt->tm_mon+1,pt->tm_mday,pt->tm_hour,pt->tm_min,pt->tm_sec);
			return buff;
		}
	}*/
	errno_t err;
	t = time(NULL);
	err = localtime_s(&pt, &t);
	if(err)
	{
		return TEXT("                   ");
	}
	else
	{
		wsprintf(buff, TEXT("%04i/%02i/%02i %02i:%02i:%02i"),
					1900+pt.tm_year,pt.tm_mon+1,pt.tm_mday,pt.tm_hour,pt.tm_min,pt.tm_sec);//2017
				return buff;
	}
} // end of pStrDateTime()

// 문자열 포멧-> MM/DD hh:mm:ss
TCHAR *pStrDateTime2( time_t t )//2017
{
	struct tm pt;
	static TCHAR buff[32];

	errno_t err;
	t = time(NULL);
	err = localtime_s(&pt, &t);
	if( err )
	{
		return TEXT("                   ");//2017
	}
	else
	{
 		wsprintf(buff, TEXT("%2i/%02i %02i:%02i:%02i"),//2017
			pt.tm_mon+1,pt.tm_mday,pt.tm_hour,pt.tm_min,pt.tm_sec);
		return buff;
	}
} // end of pStrDateTime2()
/*
 * 시간 t 를 YYYYMMDDhhmmss 형식의 문자열 시간으로 변경한다.
 */
TCHAR *pStrDateTimeShort (time_t t)//2017
{
	struct tm pt;
	static TCHAR buff[32];

	errno_t err;
	t = time(NULL);
	err = localtime_s(&pt, &t);
	if( err )
	{
		return TEXT("                   ");//2017
	}
	else
	{
		wsprintf(buff, TEXT("%04i%02i%02i%02i%02i%02i"),
			1900+pt.tm_year,pt.tm_mon+1,pt.tm_mday,pt.tm_hour,pt.tm_min,pt.tm_sec);//2017
		return buff;
	}
} // end of pStrDateTimeShort()

time_t tDateTime (char *date_time)
{
	struct tm st;

	st.tm_year = atoi_n (date_time + 0, 4) - 1900;
	st.tm_mon  = atoi_n (date_time + 4, 2) - 1;
	st.tm_mday = atoi_n (date_time + 6, 2);
	st.tm_hour = atoi_n (date_time + 8, 2);
	st.tm_min  = atoi_n (date_time + 10, 2);
	st.tm_sec  = atoi_n (date_time + 12, 2);
	
	return mktime (&st);
} // end of tDateTime()

time_t tGetTime( const char *pszTimeStr, int len )
{
	struct tm st;

	st.tm_year = (4<=len)? atoi_n ((char *)pszTimeStr + 0, 4) - 1900 : 0;
	st.tm_mon  = (6<=len)? atoi_n ((char *)pszTimeStr + 4, 2) - 1 : 0;
	st.tm_mday = (8<=len)? atoi_n ((char *)pszTimeStr + 6, 2) : 1;
	st.tm_hour = (10<=len)? atoi_n ((char *)pszTimeStr + 8, 2) : 1;
	st.tm_min  = (12<=len)? atoi_n ((char *)pszTimeStr + 10, 2) : 0;
	st.tm_sec  = (14<=len)? atoi_n ((char *)pszTimeStr + 12, 2) : 0;
	
	return mktime (&st);
} // end of tGetTime()

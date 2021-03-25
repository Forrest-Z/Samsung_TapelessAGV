#include <stdio.h>

#include <string.h>
#include <math.h>

const          char MYBASE64_keyE[]={'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z','a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z','0','1','2','3','4','5','6','7','8','9','+','/'};
const unsigned char MYBASE64_keyD[]={
    0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
    0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
    0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,62 ,0x80,0x80,0x80,63 ,
    52 ,53 ,54 ,55 ,56 ,57 ,58 ,59 ,60 ,61 ,0x80,0x80,0x80,0x80,0x80,0x80,
    0x80,0  ,1  ,2  ,3  ,4  ,5  ,6  ,7  ,8  ,9  ,10 ,11 ,12 ,13 ,14 ,
    15 ,16 ,17 ,18 ,19 ,20 ,21 ,22 ,23 ,24 ,25 ,0x80,0x80,0x80,0x80,0x80,
    0x80,26 ,27 ,28 ,29 ,30 ,31 ,32 ,33 ,34 ,35 ,36 ,37 ,38 ,39 ,40 ,
    41 ,42 ,43 ,44 ,45 ,46 ,47 ,48 ,49 ,50 ,51 ,0x80,0x80,0x80,0x80,0x80,
    0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
    0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
    0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
    0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
    0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
    0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
    0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
    0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80
};

bool Base64_Encode(unsigned char *data, int data_length, char *encodedStr)
{
	int i, j;
	unsigned char tmpBytes[3];
	
	if((data_length>0) && (data==NULL)) return false;
	if(data_length<0)		    return false;
	if(data_length==0)		    return true;
	
	for(i=0,j=0;i<(data_length/3);i++){
		tmpBytes[0] = data[i*3];
		tmpBytes[1] = data[i*3+1];
		tmpBytes[2] = data[i*3+2];
		encodedStr[j++] = MYBASE64_keyE[(unsigned char)((tmpBytes[0]>>2)&0x3F)];
		encodedStr[j++] = MYBASE64_keyE[(unsigned char)(((tmpBytes[0]<<4)&0x30) | ((tmpBytes[1]>>4)&0x0F))];
		encodedStr[j++] = MYBASE64_keyE[(unsigned char)(((tmpBytes[1]<<2)&0x3C) | ((tmpBytes[2]>>6)&0x03))];
		encodedStr[j++] = MYBASE64_keyE[(unsigned char)(tmpBytes[2]&0x3F)];
	}
	switch(data_length%3){
	case 0:
		break;
	case 1:
		tmpBytes[0] = data[i*3];
		encodedStr[j++] = MYBASE64_keyE[(unsigned char)((tmpBytes[0]>>2)&0x3F)];
		encodedStr[j++] = MYBASE64_keyE[(unsigned char)((tmpBytes[0]<<4)&0x30)];
		encodedStr[j++] = '=';
		encodedStr[j++] = '=';
		break;
	case 2:
		tmpBytes[0] = data[i*3];
		tmpBytes[1] = data[i*3+1];
		encodedStr[j++] = MYBASE64_keyE[(tmpBytes[0]>>2)&0x3F];
		encodedStr[j++] = MYBASE64_keyE[(unsigned char)(((tmpBytes[0]<<4)&0x30) | ((tmpBytes[1]>>4)&0x0F))];
		encodedStr[j++] = MYBASE64_keyE[(unsigned char)((tmpBytes[1]<<2)&0x3C)];
		encodedStr[j++] = '=';
		break;
	};
	
	return true;
}
bool Base64_Decode(char *encodedStr, int encodedStr_length, unsigned char *decodedStr, int *decodedStr_length)
{
	int i, x, n;
	unsigned char tmpBytes[4];
	
	if((encodedStr_length>0) && (encodedStr==NULL)) return false;
	if(encodedStr_length<0)				return true;
	if(encodedStr_length==0)			return true;
	if((encodedStr_length%4))			return false;
	
	     if(encodedStr[encodedStr_length-3]=='=') *decodedStr_length=encodedStr_length/4*3-3;
	else if(encodedStr[encodedStr_length-2]=='=') *decodedStr_length=encodedStr_length/4*3-2;
	else if(encodedStr[encodedStr_length-1]=='=') *decodedStr_length=encodedStr_length/4*3-1;
	else *decodedStr_length=encodedStr_length/4*3;
	
	n = (encodedStr_length/4);
	for(i=0;i<n;i++){
		if(encodedStr[i*4+0]=='='){ tmpBytes[0]=0; }else{ tmpBytes[0] = MYBASE64_keyD[(unsigned char)encodedStr[i*4+0]]; }
		if(encodedStr[i*4+1]=='='){ tmpBytes[1]=0; }else{ tmpBytes[1] = MYBASE64_keyD[(unsigned char)encodedStr[i*4+1]]; }
		if(encodedStr[i*4+2]=='='){ tmpBytes[2]=0; }else{ tmpBytes[2] = MYBASE64_keyD[(unsigned char)encodedStr[i*4+2]]; } 
		if(encodedStr[i*4+3]=='='){ tmpBytes[3]=0; }else{ tmpBytes[3] = MYBASE64_keyD[(unsigned char)encodedStr[i*4+3]]; }
		if((tmpBytes[0]&0x80)||(tmpBytes[1]&0x80)||(tmpBytes[2]&0x80)||(tmpBytes[3]&0x80)) return false;
		x =            ( (int)(tmpBytes[0] & 0x3F) );
		x = (x << 6) | ( (int)(tmpBytes[1] & 0x3F) );
		x = (x << 6) | ( (int)(tmpBytes[2] & 0x3F) );
		x = (x << 6) | ( (int)(tmpBytes[3] & 0x3F) );
		if((i+1)<n){
			decodedStr[i*3+0] = (unsigned char)((x>>16)&0xFF);
			decodedStr[i*3+1] = (unsigned char)((x>> 8)&0xFF);
			decodedStr[i*3+2] = (unsigned char)((x>> 0)&0xFF);
		}else{
			if((i*3+0)<encodedStr_length) decodedStr[i*3+0] = (unsigned char)((x>>16)&0xFF);
			if((i*3+1)<encodedStr_length) decodedStr[i*3+1] = (unsigned char)((x>> 8)&0xFF);
			if((i*3+2)<encodedStr_length) decodedStr[i*3+2] = (unsigned char)((x>> 0)&0xFF);
		}
	}
	
	return true;
}


int main(int argc, char **argv)
{
	unsigned char *inputdata = (unsigned char*)argv[1];
	int n = ((int)ceil(((float)strlen((char*)inputdata))/3.0f)) * 4;
	char *encodeddata;
	unsigned char *decodeddata;
	int  decodeddata_length;

	encodeddata = new char[n+1];
	encodeddata[n] = '\0';
	Base64_Encode(inputdata, strlen((char*)inputdata), encodeddata);
	printf("BASE64 Encode(%s) -> %s (len:%d)\r\n", inputdata, encodeddata, n);
	
	decodeddata = new unsigned char[strlen(encodeddata)/4*3+1];
	decodeddata[strlen(encodeddata)/4*3] = '\0';
	Base64_Decode(encodeddata, strlen(encodeddata), decodeddata, &decodeddata_length);
	printf("BASE64 Decode(%s) -> %s (len:%d)\r\n", encodeddata, (char*)decodeddata, decodeddata_length);

}
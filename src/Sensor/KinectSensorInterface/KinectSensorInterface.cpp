#include "stdafx.h"

#include "KinectSensorInterface.h"

float g_pDepthHist[MAX_DEPTH];
XnRGB24Pixel* g_pTexMap = NULL;
unsigned int g_nTexMapX = 0;
unsigned int g_nTexMapY = 0;

Context g_context;
DepthGenerator g_depth;
ImageGenerator g_image;
DepthMetaData g_depthMD;
ImageMetaData g_imageMD;

KinectSensorInterface::KinectSensorInterface()
{
	//키넥트 데이터 초기화
	m_IplColorImg = cvCreateImage(cvSize(Sensor::KINECT_IMAGE_WIDTH,Sensor::KINECT_IMAGE_HEIGHT),8,3);
	m_IplDepthColorImg = cvCreateImage(cvSize(Sensor::KINECT_IMAGE_WIDTH,Sensor::KINECT_IMAGE_HEIGHT),8,3);
	m_Ipl320ColorImg = cvCreateImage(cvSize(Sensor::IMAGE_WIDTH,Sensor::IMAGE_HEIGHT),8,3);
	m_Ipl320DepthColorImg = cvCreateImage(cvSize(Sensor::IMAGE_WIDTH,Sensor::IMAGE_HEIGHT),8,3);

	m_nKinectRangeData=m_KuUtil.generateIntType1DArray(Sensor::KINECT_SENSOR_FOV,MAX_VAL);
	m_nTmpKinectRangeData=m_KuUtil.generateIntType1DArray(Sensor::KINECT_SENSOR_FOV,MAX_VAL);
	m_nHeightDataOfKinectRangeData =m_KuUtil.generateIntType1DArray(Sensor::KINECT_SENSOR_FOV,MAX_VAL);
	m_nTmpHeightDataOfKinectRangeData=m_KuUtil.generateIntType1DArray(Sensor::KINECT_SENSOR_FOV,MAX_VAL);

	m_fDistance = new float[Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT];
}
KinectSensorInterface::~KinectSensorInterface()
{
	//키넥트 메모리 해제 
	if(g_pTexMap!=NULL) free(g_pTexMap);
	if(g_pDepthHist!=NULL) free(g_pDepthHist);
	if(m_IplColorImg!=NULL)cvReleaseImage(&m_IplColorImg);
	if(m_IplDepthColorImg!=NULL)cvReleaseImage(&m_IplDepthColorImg);
	if(m_Ipl320ColorImg!=NULL)cvReleaseImage(&m_Ipl320ColorImg);
	if(m_Ipl320DepthColorImg!=NULL)cvReleaseImage(&m_Ipl320DepthColorImg);

	if(m_fDistance){
		delete[] m_fDistance;
	}
}

void KinectSensorInterface::doThread(void)
{
	KinectSensorInterface::getInstance()->execute();
}

void KinectSensorInterface::start()
{
	m_Thread.start(&doThread ,100);
}

void KinectSensorInterface::terminate()
{
	m_Thread.terminate();
	while(!g_bCallbackEnd) { Sleep(10); };

	g_context.Shutdown();
	g_depthMD.Free();
	g_imageMD.Free();
	if(g_pTexMap){
		free(g_pTexMap) ;
		g_pTexMap=NULL;
	}
}

void KinectSensorInterface::suspend()
{
	m_Thread.suspend();
}

void KinectSensorInterface::resume()
{
	m_Thread.resume();

}
/**
 @brief Korean: 2D 거리 정보를 가져감
 @brief English: 
*/
int_1DArray KinectSensorInterface::getRangeData()
{
	return m_nKinectRangeData;
}
/**
 @brief Korean: 2D 거리 정보에 따른 높이 정보를 가져감
 @brief English: 
*/
int_1DArray KinectSensorInterface::getHeightDataOfRangeData()
{
	return m_nHeightDataOfKinectRangeData;
}
/**
 @brief Korean: 640 X 480 뎁스 이미지를 가져감
 @brief English: 
*/
IplImage* KinectSensorInterface::getDepthImage()
{
	return m_IplDepthColorImg;
}
/**
 @brief Korean: 320 X 240 뎁스 이미지를 가져감
 @brief English: 
*/
IplImage* KinectSensorInterface::get320DepthImage()
{
	return m_Ipl320DepthColorImg;
}
/**
 @brief Korean: 640 X 480 RGB 영상을 가져감
 @brief English: 
*/
IplImage* KinectSensorInterface::getColorImage()
{
	return m_IplColorImg;
}
/**
 @brief Korean: 320 X 240 RGB 영상을 가져감
 @brief English: 
*/
IplImage* KinectSensorInterface::get320ColorImage()
{
	return m_Ipl320ColorImg;
}
/**
 @brief Korean: 320 X 240 뎁스 이미지를 가져감
 @brief English: 
*/
float* KinectSensorInterface::getDistanceImage()
{
	return m_fDistance;
}
/**
 @brief Korean: 3D 거리의 정보를 2D 거리 정보로 변경함
 @brief English: 
*/
void KinectSensorInterface::translateKinectDataToRangeData(KuPose* p3DDataPos)
{
	float_1DArray fSelectedX=m_KuUtil.generateFloatType1DArray(Sensor::IMAGE_WIDTH);
	float_1DArray fSelectedY=m_KuUtil.generateFloatType1DArray(Sensor::IMAGE_WIDTH);
	float_1DArray fSelectedZ=m_KuUtil.generateFloatType1DArray(Sensor::IMAGE_WIDTH);

	float fSelectedZVal=0;

	int nStart=0;
	int nEnd=0;

	int nMaxDist=KuRobotParameter::getInstance()->getKinectMaxDist();
	int nMinDist=KuRobotParameter::getInstance()->getKinectMinDist();
	int nMaxHeightDist=KuRobotParameter::getInstance()->getKinectMaxHeightDist();
	int nMinHeightDist=KuRobotParameter::getInstance()->getKinectMinHeightDist();


	for(int i=0; i<Sensor::IMAGE_WIDTH; i++){
		float dMinDistX = 1000000.; 
		float dMinDistY =0; //거리값이 가장 가까운 부분의 Y좌표 값
		for(int j=0; j<Sensor::IMAGE_HEIGHT; j++){	
			if(p3DDataPos[j*(Sensor::IMAGE_WIDTH)+i].getID()==-1) continue;
			double dTmpX = p3DDataPos[j*(Sensor::IMAGE_WIDTH)+i].getX(); //거리값 
			double dTmpY = p3DDataPos[j*(Sensor::IMAGE_WIDTH)+i].getY(); //거리값 
			double dTmpZ = p3DDataPos[j*(Sensor::IMAGE_WIDTH)+i].getZ(); //높이값

			if(dTmpZ > nMaxHeightDist) continue;
			if(dTmpZ < nMinHeightDist) continue;
			if(dTmpX <= nMaxDist&& dTmpX >=nMinDist ){

				double dMinTempDist = sqrt(pow(dTmpX ,2)+pow(dTmpY ,2));			
				if(dMinDistX >  dMinTempDist){
					dMinDistX = dMinTempDist;
					dMinDistY =dTmpY;
					fSelectedZVal = dTmpZ; //거리값이 가장 가까운 x,y값에대한 높이값
				}
			}
		}
		if(dMinDistX == MAX_VAL){ //거리정보가 유효하지 않은 경우
			if(i<Sensor::IMAGE_WIDTH/2) nStart++;
			else if(i>Sensor::IMAGE_WIDTH/2) nEnd++;

			fSelectedX[i] = -1;
			fSelectedY[i] = -1;
			fSelectedZ[i] = -1;
		}
		else{
			fSelectedX[i] = dMinDistX;
			fSelectedY[i] = dMinDistY;
			fSelectedZ[i] = fSelectedZVal;
		}
	}	

	int nTestDeg=0;
	int nDeg=0,nTmp;

	for(int i=nStart+1; i<Sensor::IMAGE_WIDTH-nEnd-1;i++){ 
		if(fSelectedX[i] != -1 && fSelectedY[i] !=-1){
			nTestDeg = (int)(atan2(fSelectedY[i],fSelectedX[i])*R2D);
			nDeg = nTestDeg+Sensor::KINECT_SENSOR_FOV/2;
			if(nDeg >=0 && nDeg < Sensor::KINECT_SENSOR_FOV){
				if(m_nTmpKinectRangeData[nDeg] > fSelectedX[i]){
					m_nTmpKinectRangeData[nDeg] = fSelectedX[i];
					m_nTmpHeightDataOfKinectRangeData[nDeg] = fSelectedZ[i];
				}

			}
		}
	}


}
/**
 @brief Korean: 3D 거리의 정보를 가져감
 @brief English: 
*/
KuPose* KinectSensorInterface::getGlobal3DPose()
{
	return m_Global3DPose;
}
/**
 @brief Korean: 키넥트에 연결함
 @brief English: 
*/
bool KinectSensorInterface::connect()
{
	XnStatus rc;

	EnumerationErrors errors;
	rc = g_context.InitFromXmlFile(SAMPLE_XML_PATH, &errors);
	if (rc == XN_STATUS_NO_NODE_PRESENT)
	{
		XnChar strError[1024];
		errors.ToString(strError, 1024);
		printf("%s\n", strError);

		return false;
	}
	else if (rc != XN_STATUS_OK)
	{
		printf("Open failed: %s\n", xnGetStatusString(rc));
		
		return false;
	}

	rc = g_context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_depth);
	rc = g_context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_image);

	g_depth.GetMetaData(g_depthMD);
	g_image.GetMetaData(g_imageMD);
	g_depth.GetAlternativeViewPointCap().SetViewPoint(g_image);//calibration

	// Hybrid mode isn't supported in this sample
	if (g_imageMD.FullXRes() != g_depthMD.FullXRes() || g_imageMD.FullYRes() != g_depthMD.FullYRes())
	{
		printf ("The device depth and image resolution must be equal!\n");
		return 1;
	}

	// RGB is the only image format supported.
	if (g_imageMD.PixelFormat() != XN_PIXEL_FORMAT_RGB24)
	{
		printf("The device image format must be RGB24\n");
		return 1;
	}

	// Texture map init
	g_nTexMapX = (((unsigned short)(g_depthMD.FullXRes()-1) / 512) + 1) * 512;
	g_nTexMapY = (((unsigned short)(g_depthMD.FullYRes()-1) / 512) + 1) * 512;
	g_pTexMap = (XnRGB24Pixel*)malloc(g_nTexMapX * g_nTexMapY * sizeof(XnRGB24Pixel));

	xnOSMemSet(g_pDepthHist, 0, MAX_DEPTH*sizeof(float));
	xnOSMemSet(g_pTexMap, 0, g_nTexMapX*g_nTexMapY*sizeof(XnRGB24Pixel));
	
	return true;
}
/**
 @brief Korean:  키넥트에서 RGB 영상을 받아옴
 @brief English: 
*/
void KinectSensorInterface::KinectRGBData()
{

	const XnRGB24Pixel* pImageRow = g_imageMD.RGB24Data();
	XnUInt MDYR=g_imageMD.YRes();
	XnUInt MDXR=g_depthMD.XRes();

	for (XnUInt y = 0; y < MDYR; ++y)
	{
		for (XnUInt x = 0; x <MDXR; ++x)
		{
			m_IplColorImg->imageData[x*3+y*640*3]=(char)pImageRow[640-x+y*640].nRed;
			m_IplColorImg->imageData[x*3+1+y*640*3]=(char)pImageRow[640-x+1+y*640].nGreen;
			m_IplColorImg->imageData[x*3+2+y*640*3]=(char)pImageRow[640-x+2+y*640].nBlue;
		}
	}


}
/**
 @brief Korean:  키넥트에서 3D 정보를 받아옴
 @brief English: 
*/
void KinectSensorInterface::Kinect3DData()
{
	const XnDepthPixel* pDepth = g_depthMD.Data();		

	g_depth.GetMetaData( g_depthMD );

	unsigned int uPointNum = g_depthMD.FullXRes() * g_depthMD.FullYRes();

	XnPoint3D* pDepthPointSet = new XnPoint3D[ uPointNum ];
	unsigned int i, j, idxShift, idx;

	pDepth = g_depthMD.Data();


	XnUInt MDYR=g_depthMD.FullYRes();
	XnUInt MDXR=g_depthMD.FullXRes();

	for( j = 0; j <MDYR ; ++j )
	{
		idxShift = j * MDXR;
		for( i = 0; i <MDXR; ++i)
		{
			idx = idxShift + i;
			pDepthPointSet[idx].X =640- i;
			pDepthPointSet[idx].Y = j;
			pDepthPointSet[idx].Z = pDepth[idx];

		}
	}

	double dXTiltOffset  =KuRobotParameter::getInstance()->getKinectXOffset();//Sensor::X_TiltOffset; // 로봇의 중앙으로 틸트 모터까지의 거리 mm단위
	double dZTiltOffset =KuRobotParameter::getInstance()->getKinectHeight();//Sensor::Z_TiltOffset; //바닥으로부터 틸팅모터까지의 높이. mm단위
	//-------------------------------------------------------------------
	double dKinectMax=KuRobotParameter::getInstance()->getKinectMaxDist();
	double dKinectMIn=KuRobotParameter::getInstance()->getKinectMinDist();

	double dTiltAngRad = 0;//임의의 값을 넣었음.
	double dCosVal = cos(-dTiltAngRad); //계산을 빠르게 하기 위해서 
	double dSinVal = sin(-dTiltAngRad); //계산을 빠르게 하기 위해서 


	double dTransX=0., dTransY=0., dTransZ=0.;

	XnPoint3D* p3DPointSet = new XnPoint3D[ uPointNum ];
	g_depth.ConvertProjectiveToRealWorld( uPointNum, pDepthPointSet, p3DPointSet );
	delete[] pDepthPointSet;

	for (XnUInt y = 0; y <MDYR; y+=2)
	{
		for (XnUInt x = 0; x < MDXR; x+=2 )
		{

			dTransX= (p3DPointSet[y*640+x].Z)*dCosVal -(p3DPointSet[y*640+x].Y)*dSinVal+ dXTiltOffset;
			dTransY =-p3DPointSet[y*640+x].X; 
			dTransZ = (p3DPointSet[y*640+x].Z)*dSinVal + (p3DPointSet[y*640+x].Y)*dCosVal + dZTiltOffset;

			if(hypot(dTransX,dTransY)>dKinectMax) {
				m_TmpGlobal3DPose[(int)(y/2)*320+(int)(x/2)].init();
				m_fDistance[(int)(y/2)*320+(int)((int)320-x/2)]=0.0;
				continue;
			}
			else if(hypot(dTransX,dTransY)<dKinectMIn) {
				m_TmpGlobal3DPose[(int)(y/2)*320+(int)(x/2)].init();
				m_fDistance[(int)(y/2)*320+(int)((int)320-x/2)]=0.0;
				continue;
			}

			m_TmpGlobal3DPose[(int)(y/2)*320+(int)(x/2)].setZ(dTransZ);
			m_TmpGlobal3DPose[(int)(y/2)*320+(int)(x/2)].setY(dTransY);
			m_TmpGlobal3DPose[(int)(y/2)*320+(int)(x/2)].setX(dTransX);
			m_TmpGlobal3DPose[(int)(y/2)*320+(int)(x/2)].setID(1);
			m_TmpGlobal3DPose[(int)(y/2)*320+(int)(x/2)].setPixX((int)320-x/2);
			m_TmpGlobal3DPose[(int)(y/2)*320+(int)(x/2)].setPixY((int)y/2);
			m_fDistance[(int)(y/2)*320+(int)((int)320-x/2)]=pow(dTransZ*dTransZ+dTransY*dTransY+dTransX*dTransX , 1/2.0 )/100;
		}
	}
	
	delete[] p3DPointSet;

}
/**
 @brief Korean: 키넥트에서 데이터를 받아옴
 @brief English: 
*/
void KinectSensorInterface::execute()
{
		g_bCallbackEnd=false;

		XnStatus rc = XN_STATUS_OK;
		// Read a new frame
		rc = g_context.WaitAnyUpdateAll();

		if (rc != XN_STATUS_OK)
		{
			printf("Read failed: %s\n", xnGetStatusString(rc));
			return;
		}

		g_depth.GetMetaData(g_depthMD);
		g_image.GetMetaData(g_imageMD);
		g_depth.GetAlternativeViewPointCap().SetViewPoint(g_image);//calibration

		KinectRGBData();//RGB 영상 데이터
		Kinect3DData();//3D 거리 정보(depth 영상의 각 픽셀에 x, y, z, distance 등의 정보를 계산하여 넣음)

		m_CriticalSection.Lock();
		for(int i=0,x=0; i<Sensor::KINECT_IMAGE_WIDTH; i+=2,x++){
			for(int j=0,y=0; j<Sensor::KINECT_IMAGE_HEIGHT; j+=2,y++){
				//color image 얻어오는 부분----------------------------------------------------------
				m_Ipl320ColorImg->imageData[(y*320+x)*3] = m_IplColorImg->imageData[(j*640+i)*3+2];
				m_Ipl320ColorImg->imageData[(y*320+x)*3+1] = m_IplColorImg->imageData[(j*640+i)*3+1];
				m_Ipl320ColorImg->imageData[(y*320+x)*3+2] = m_IplColorImg->imageData[(j*640+i)*3];
				//------------------------------------------------------------------------------------
				m_Global3DPose[(y)*320+(x)]=m_TmpGlobal3DPose[(y)*320+(x)];		
			}
		}
		m_CriticalSection.Unlock();
		
		translateKinectDataToRangeData(m_Global3DPose);//3D->2D정보로 변경		

		m_CriticalSection.Lock();
		for(int i=0; i<Sensor::KINECT_SENSOR_FOV; i++){
			m_nKinectRangeData[i]=m_nTmpKinectRangeData[i] ;
			m_nTmpKinectRangeData[i]= MAX_VAL;

			m_nHeightDataOfKinectRangeData[i]=m_nTmpHeightDataOfKinectRangeData[i] ;
			m_nTmpHeightDataOfKinectRangeData[i]=MAX_VAL ;

		}
		m_CriticalSection.Unlock();
		g_bCallbackEnd=true;

}
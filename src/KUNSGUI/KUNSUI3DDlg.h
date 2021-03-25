#pragma once
#include "atltypes.h"
#include "glut.h"
#include "glaux.h"	
#include <vector>
#include <list>
#include "../KUNSUtil/KuUtil.h"
#include "../KUNSMath/KuMath.h"
#include "../KUNSPose/KuPose.h"
#include "../KUNSGUI/KuDrawingInfo.h"
#include "../KUNSMap/KuMap.h"
#include "../Sensor/Sensor.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "../Algorithm/FeatureDetector/FDResult.h"
#include "../Algorithm/FeatureDetector/FeatureDetector.h"
#include "../Algorithm/PathBlock/PathBlock.h"
#include "../KUNSProcess/KUNSPathBlockPr/KuPathBlockPr.h"
// CKUNSUI3DDlg dialog

typedef struct _tagGLFONT
{
	GLuint base;
	int widths[256];
	int height;
} GLFONT;

class CKUNSUI3DDlg : public CDialog
{
	DECLARE_DYNAMIC(CKUNSUI3DDlg)

public:
	CKUNSUI3DDlg(CWnd* pParent = NULL);   // standard constructor
	virtual ~CKUNSUI3DDlg();

	// Dialog Data
	enum { IDD = IDD_KUNSUI3D_DIALOG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	DECLARE_MESSAGE_MAP()
private:
	CCriticalSection m_CriticalSection;
private:
	double m_dMapHeight, m_dHeightClassifedNode;
	double m_dTopologicalMapHeight;
	bool m_bLButtonDown; 	// Left mouse button down
	bool m_bRButtonDown;
	CPoint m_ptLButtonDownPos; // Position of mouse cursor when it's clicked
	CPoint m_RButtonDownPos;
	GLdouble m_yRotate; // Rotation in y direction
	GLdouble m_xRotate; // Rotation in x direction
	GLFONT* m_pFont; // Pointer variable of GLFONT
	int m_GLPixelIndex;
	HGLRC m_hGLContext;
	GLdouble m_dZoom; // Zoom
	CDC *m_pCamDC, *m_pCamDC2;

	bool m_bSetRobotPos;
	bool m_bSetGoalPos;
	bool m_bSetPathBlock;
	bool m_bSetVirtualObstacleFlag;
	int m_nPathBockID;
	KuMath m_math;

	int m_nMapSizeX;
	int m_nMapSizeY;
	double m_dCellSize;
	int m_nDrawingSizeHalf;
	int m_nLocalMapSizeX;
	int m_nLocalMapSizeY;

	int m_nLastMousePointX;
	int m_nLastMousePointY;
	float m_fXOffset;
	float m_fYOffset;

	bool m_bChangePose;

	IplImage * m_IplCeilingImage;
	IplImage * m_IplKinectImg;	
	CvvImage m_cvCeilingImage;
	CvvImage m_cvKinectImage;
	int m_nSizeWidth;
	int m_nSizeHeight;
	double m_dmidUI_x;
	double m_dmidUI_y;

	double m_dMPoint_x;
	double m_dMPoint_y;

		 bool m_bsettingflag;

private:
	void Initialize();
	GLFONT* OpenGL_Font_Create(HDC hdc, const char* typeface, int height, int weight, DWORD italic); // Create font
	void OpenGL_Font_Destroy(GLFONT* font); // Destroy the font
	void OpenGL_Font_Printf(GLFONT* font, int align, const char* format); // Print the font
	void OpenGL_Font_Puts(GLFONT* font, const char* s); // Put "s" to the font
	void Render_Axis(void); // Render x, y, z axes
	void Render_Grid(const int& nMapWidth, const int& nMapHeight); 	// Render grid
	bool OpenGL_CreateViewGLContext(HDC hDC); 	// Create OpenGL Context
	void OpenGL_RenderScene(void); 	// OpenGL rendering source codes
	bool OpenGL_SetupPixelFormat(HDC hDC); // Setup pixel formats
	void Render_3DRect(double dSizeX, double dSizeY, double dSizeZ);
	void Render_3DRectWire(double dSizeX, double dSizeY, double dSizeZ);

	void renderRobot();
	void renderVirtualRobot();

	void renderPath();
	void renderWayPointList();

	void renderLaserData();
	void renderKinectRangeData();

	void renderBuiltMap();
	void renderBuildingMap();
	void renderSample();
	void renderGoalPosition();

	void renderGoalList();	

	void renderLeastCubicSplinePath();
	void renderTargetPos();

	void renderCamImageData();

	void renderCeilingDBImageData(void);		

	void renderLocalPath();
	void renderLocalGoalPos();

	void renderZoneMap();

	void renderAlignLaserData();
	void renderTLaserData();
	void renderRobot2();

	void renderRegion();
	void renderTemplateData();

	void renderFiducialmark();

	void renderPathBlock();
	void rendervecPathBlock();
	void rendervecPathlist();

	bool checkOverlap(vector<PBlock> vecPathBlockPos);
	bool sticktoPoint(vector<PBlock> *vecPathBlockPos,PBlock PB);


public:
	void setCamInfoDC(CDC* pDC);
	void setCamInfo2DC(CDC* pDC);
	void setCamSizeInfo(int nSizeWidth,int nSizeHeight);


	void Zoom_In(double dVal);
	void Zoom_Out(double dVal);


	void setRobotPosFlag(bool bFlag);
	void setGoalPosFlag(bool bFlag);

	void setPathBlockFlag(bool bFlag);
	void setPathBlockID(int nPathBockID);

	vector<PBlock>  getPathBlockPos();
	void setPathBlockPos(vector<PBlock> vecPathBlock);

	void setPathBlocksettingFlag(bool bFlag);
	void getMousePointonUI(double* dX,double* dY);

	AUX_RGBImageRec *LoadBMPFile(char *filename);


	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnPaint();
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnDestroy();
	virtual BOOL OnInitDialog();
	afx_msg void OnSize(UINT nType, int cx, int cy);	
	afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnRButtonUp(UINT nFlags, CPoint point);


	afx_msg BOOL OnSetCursor(CWnd* pWnd, UINT nHitTest, UINT message);
};

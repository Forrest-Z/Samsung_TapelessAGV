// 3DMapDlg\3DMapDlg.cpp : implementation file
//

#include "stdafx.h"
#include "../../Samsung_TapelessAGV.h"
#include "KUNSUI3DDlg.h"
#include "../../Samsung_TapelessAGVDlg.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#endif

IMPLEMENT_DYNAMIC(CKUNSUI3DDlg, CDialog)
	CKUNSUI3DDlg::CKUNSUI3DDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CKUNSUI3DDlg::IDD, pParent)
	, m_bLButtonDown(false)
	, m_ptLButtonDownPos(0)
	, m_yRotate(0)
	, m_xRotate(90)
	, m_pFont(NULL)
	, m_GLPixelIndex(0)
	, m_dZoom(1)
{

	m_bSetRobotPos = false;
	m_bSetGoalPos = false;
	m_bSetPathBlock = false;
	m_bsettingflag=false;
	m_nPathBockID=-1;

	m_fXOffset = 0.0;
	m_fYOffset = 0.0;


	m_dMapHeight = 0.02;
	m_dHeightClassifedNode = m_dMapHeight+0.15;
	m_dTopologicalMapHeight = m_dHeightClassifedNode;

	m_IplCeilingImage = cvCreateImage(cvSize( Sensor::CEILING_IMAGE_WIDTH, Sensor::CEILING_IMAGE_HEIGHT),8,1);

	m_IplKinectImg = cvCreateImage(cvSize( Sensor::IMAGE_WIDTH, Sensor::IMAGE_HEIGHT),8,3);
}

CKUNSUI3DDlg::~CKUNSUI3DDlg()
{

}

void CKUNSUI3DDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CKUNSUI3DDlg, CDialog)
	ON_WM_CREATE()
	ON_WM_PAINT()
	ON_WM_MOUSEMOVE()
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
	ON_WM_DESTROY()
	ON_WM_SIZE()
	ON_WM_MOUSEWHEEL()
	ON_WM_RBUTTONDOWN()
	ON_WM_RBUTTONUP()
	ON_WM_SETCURSOR()
END_MESSAGE_MAP()


// Setup pixel formats
bool CKUNSUI3DDlg::OpenGL_SetupPixelFormat(HDC hDC)
{
	PIXELFORMATDESCRIPTOR pixelDesc;

	pixelDesc.nSize = sizeof(PIXELFORMATDESCRIPTOR);
	pixelDesc.nVersion = 1;

	pixelDesc.dwFlags = PFD_DRAW_TO_WINDOW | 
		PFD_SUPPORT_OPENGL |
		PFD_DOUBLEBUFFER |
		PFD_STEREO_DONTCARE;

	pixelDesc.iPixelType = PFD_TYPE_RGBA;
	pixelDesc.cColorBits = 32;
	pixelDesc.cRedBits = 8;
	pixelDesc.cRedShift = 16;
	pixelDesc.cGreenBits = 8;
	pixelDesc.cGreenShift = 8;
	pixelDesc.cBlueBits = 8;
	pixelDesc.cBlueShift = 0;
	pixelDesc.cAlphaBits = 0;
	pixelDesc.cAlphaShift = 0;
	pixelDesc.cAccumBits = 64;
	pixelDesc.cAccumRedBits = 16;
	pixelDesc.cAccumGreenBits = 16;
	pixelDesc.cAccumBlueBits = 16;
	pixelDesc.cAccumAlphaBits = 0;
	pixelDesc.cDepthBits = 32;
	pixelDesc.cStencilBits = 8;
	pixelDesc.cAuxBuffers = 0;
	pixelDesc.iLayerType = PFD_MAIN_PLANE;
	pixelDesc.bReserved = 0;
	pixelDesc.dwLayerMask = 0;
	pixelDesc.dwVisibleMask = 0;
	pixelDesc.dwDamageMask = 0;

	m_GLPixelIndex = ChoosePixelFormat(hDC,&pixelDesc);
	if(m_GLPixelIndex==0) // Choose default
	{
		m_GLPixelIndex = 1;
		if(DescribePixelFormat(hDC,m_GLPixelIndex,
			sizeof(PIXELFORMATDESCRIPTOR),&pixelDesc)==0)
		{
			return FALSE;
		}
	}

	if(SetPixelFormat(hDC,m_GLPixelIndex,&pixelDesc)==FALSE)
	{
		return FALSE;
	}

	return TRUE;
}

int CKUNSUI3DDlg::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CDialog::OnCreate(lpCreateStruct) == -1)
		return -1;

	// TODO:  Add your specialized creation code here
	//OpenGL_SetupPixelFormat(::GetDC(m_hWnd));

	return 0;
}

void CKUNSUI3DDlg::OnPaint()
{
	CPaintDC dc(this); // device context for painting
	// TODO: Add your message handler code here
	if (IsIconic())
	{
		SendMessage(WM_ICONERASEBKGND, (WPARAM) dc.GetSafeHdc(), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;
	}
	else
	{

		OpenGL_RenderScene();
		SwapBuffers(dc.m_ps.hdc);				
		CDialog::OnPaint();

	}

	// Do not call CDialog::OnPaint() for painting messages
}

// OpenGL rendering source codes
void CKUNSUI3DDlg::OpenGL_RenderScene(void)
{

	//glClearColor(1.0, 1.0, 1.0, 1.0);
	glClearColor(0.65,0.65,0.65,0.3);
	GLfloat lightPos[] = {-50, 50, 40, 0};
	glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();

/*
	// Ortho projection
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	double dZoom(13 - m_dZoom);
	glOrtho(-dZoom, dZoom, -dZoom, dZoom, -100, 100);
*/
		
	// -------------- Draw here --------------
	glPushMatrix();

	glTranslated(0.0, 0.0, -13.0 + m_dZoom);
	glTranslated(m_fXOffset, m_fYOffset, 0.0);
	glRotatef(m_xRotate, 1.0, 0.0, 0.0);
	glRotatef(m_yRotate, 0.0, 1.0, 0.0);
	//glRotatef(m_yRotate-RobotPos.getThetaDeg()+90, 0.0, 1.0, 0.0);
	glRotatef(-90.0, 1.0, 0.0, 0.0);

	glPushMatrix();

	RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
	m_dmidUI_x=-RobotPos.getXm();
	m_dmidUI_y=-RobotPos.getYm();
	glTranslated(m_dmidUI_x, m_dmidUI_y,0);

	glEnable(GL_LINE_SMOOTH);
	Render_Axis();
	glDisable(GL_LINE_SMOOTH);
	glDisable(GL_LIGHT0);
	Render_Grid(2000, 2000);
	glEnable(GL_LIGHT0);
	renderRobot(); //로봇을 그려준다.
	renderRobot2();

	if(KuDrawingInfo::getInstance()->getRenderLaserflag()==true){ 
		renderLaserData(); //레이저 데이터를 그리도록 플래그 셋팅한 후에 화면에 그려준다.
		renderAlignLaserData();
		//renderTLaserData();
	}	
	if(KuDrawingInfo::getInstance()->getRenderMapflag()==true){ //이미 만들어진 격자지도를 화면에 그려준다.
		//영역 구분 지도가 그려질때는 보통 그리드 지도는 그리지 않기위해서
		if(KuDrawingInfo::getInstance()->getRenderBuildingMapflag()==true){
			renderBuildingMap(); //레이저로 작성되는 지도를 그린다.
		}
		else
		{
			renderBuiltMap();			
		}
	}	

	//영역 구분 지도가 그려질때는 보통 그리드 지도는 그리지 않기위해서
	if(KuDrawingInfo::getInstance()->getRenderZoneMapflag()==true){
		renderZoneMap();
	}

	if(KuDrawingInfo::getInstance()->getRenderPathflag()==true){
		renderPath();	
		renderLocalPath();		
		renderWayPointList();
	}

	if(KuDrawingInfo::getInstance()->getRenderKinectflag()==true)
	{	//키넥트 데이터를 그리도록 플래그 셋팅한 후에 화면에 그려준다.
		renderKinectRangeData();
	}

	renderTargetPos();
	renderLocalGoalPos();


	renderGoalPosition();

	renderSample();

	renderCamImageData();

	if(KuDrawingInfo::getInstance()->getRenderCeilingImageflag()==true)
	{
		renderCeilingDBImageData();
	}

//	renderRegion();

//	renderTemplateData();

	renderFiducialmark();

	renderPathBlock();

	rendervecPathBlock();

	rendervecPathlist();

	glPopMatrix();

	glPopMatrix();

	// -------------- Draw here --------------

}

void CKUNSUI3DDlg::Render_3DRect(double dSizeX, double dSizeY, double dSizeZ)
{
	double dHalfSizeX = dSizeX / 2;
	double dHalfSizeY = dSizeY / 2;
	double dHalfSizeZ = dSizeZ / 2;

	GLUquadricObj *pObj;
	pObj = gluNewQuadric();
	gluQuadricNormals(pObj, GLU_SMOOTH);

	glDisable(GL_CULL_FACE );
	glEnable(GL_LINE_SMOOTH);

	glPushMatrix();

	glBegin(GL_POLYGON);
	glVertex3d(dHalfSizeX, -dHalfSizeY, -dHalfSizeZ);
	glVertex3d(-dHalfSizeX, -dHalfSizeY, -dHalfSizeZ);
	glVertex3d(-dHalfSizeX, dHalfSizeY, -dHalfSizeZ);
	glVertex3d(dHalfSizeX, dHalfSizeY, -dHalfSizeZ);
	glEnd();

	glBegin(GL_POLYGON);
	glVertex3d(-dHalfSizeX, dHalfSizeY, -dHalfSizeZ);
	glVertex3d(-dHalfSizeX, dHalfSizeY, dHalfSizeZ);
	glVertex3d(dHalfSizeX, dHalfSizeY, dHalfSizeZ);
	glVertex3d(dHalfSizeX, dHalfSizeY, -dHalfSizeZ);
	glEnd();

	glBegin(GL_POLYGON);
	glVertex3d(dHalfSizeX, dHalfSizeY, -dHalfSizeZ);
	glVertex3d(dHalfSizeX, dHalfSizeY, dHalfSizeZ);
	glVertex3d(dHalfSizeX, -dHalfSizeY, dHalfSizeZ);
	glVertex3d(dHalfSizeX, -dHalfSizeY, -dHalfSizeZ);
	glEnd();

	glBegin(GL_POLYGON);
	glVertex3d(-dHalfSizeX, -dHalfSizeY, -dHalfSizeZ);
	glVertex3d(-dHalfSizeX, -dHalfSizeY, dHalfSizeZ);
	glVertex3d(-dHalfSizeX, dHalfSizeY, dHalfSizeZ);
	glVertex3d(-dHalfSizeX, dHalfSizeY, -dHalfSizeZ);
	glEnd();

	glBegin(GL_POLYGON);
	glVertex3d(dHalfSizeX, -dHalfSizeY, -dHalfSizeZ);
	glVertex3d(dHalfSizeX, -dHalfSizeY, dHalfSizeZ);
	glVertex3d(-dHalfSizeX, -dHalfSizeY, dHalfSizeZ);
	glVertex3d(-dHalfSizeX, -dHalfSizeY, -dHalfSizeZ);
	glEnd();

	glBegin(GL_POLYGON);
	glVertex3d(dHalfSizeX, -dHalfSizeY, dHalfSizeZ);
	glVertex3d(dHalfSizeX, dHalfSizeY, dHalfSizeZ);
	glVertex3d(-dHalfSizeX, dHalfSizeY, dHalfSizeZ);
	glVertex3d(-dHalfSizeX, -dHalfSizeY, dHalfSizeZ);
	glEnd();

	glPopMatrix();

	glEnable(GL_CULL_FACE);

	gluDeleteQuadric(pObj);
}

void CKUNSUI3DDlg::Render_3DRectWire(double dSizeX, double dSizeY, double dSizeZ)
{
	double dHalfSizeX = dSizeX / 2;
	double dHalfSizeY = dSizeY / 2;
	double dHalfSizeZ = dSizeZ / 2;

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	//glDisable(GL_CULL_FACE );
	//	glDisable(GL_LINE_SMOOTH);
	glEnable(GL_LINE_SMOOTH);
	glEnable( GL_POLYGON_SMOOTH );

	glPushMatrix();

	glBegin(GL_POLYGON);
	glVertex3d(dHalfSizeX, -dHalfSizeY, -dHalfSizeZ);
	glVertex3d(-dHalfSizeX, -dHalfSizeY, -dHalfSizeZ);
	glVertex3d(-dHalfSizeX, dHalfSizeY, -dHalfSizeZ);
	glVertex3d(dHalfSizeX, dHalfSizeY, -dHalfSizeZ);
	glEnd();

	glBegin(GL_POLYGON);
	glVertex3d(dHalfSizeX, -dHalfSizeY, -dHalfSizeZ);
	glVertex3d(dHalfSizeX, -dHalfSizeY, dHalfSizeZ);
	glVertex3d(-dHalfSizeX, -dHalfSizeY, dHalfSizeZ);
	glVertex3d(-dHalfSizeX, -dHalfSizeY, -dHalfSizeZ);
	glEnd();

	glBegin(GL_POLYGON);
	glVertex3d(-dHalfSizeX, -dHalfSizeY, -dHalfSizeZ);
	glVertex3d(-dHalfSizeX, -dHalfSizeY, dHalfSizeZ);
	glVertex3d(-dHalfSizeX, dHalfSizeY, dHalfSizeZ);
	glVertex3d(-dHalfSizeX, dHalfSizeY, -dHalfSizeZ);
	glEnd();

	glBegin(GL_POLYGON);
	glVertex3d(-dHalfSizeX, dHalfSizeY, -dHalfSizeZ);
	glVertex3d(-dHalfSizeX, dHalfSizeY, dHalfSizeZ);
	glVertex3d(dHalfSizeX, dHalfSizeY, dHalfSizeZ);
	glVertex3d(dHalfSizeX, dHalfSizeY, -dHalfSizeZ);
	glEnd();

	glBegin(GL_POLYGON);
	glVertex3d(dHalfSizeX, dHalfSizeY, -dHalfSizeZ);
	glVertex3d(dHalfSizeX, dHalfSizeY, dHalfSizeZ);
	glVertex3d(dHalfSizeX, -dHalfSizeY, dHalfSizeZ);
	glVertex3d(dHalfSizeX, -dHalfSizeY, -dHalfSizeZ);
	glEnd();

	glBegin(GL_POLYGON);
	glVertex3d(dHalfSizeX, -dHalfSizeY, dHalfSizeZ);
	glVertex3d(dHalfSizeX, dHalfSizeY, dHalfSizeZ);
	glVertex3d(-dHalfSizeX, dHalfSizeY, dHalfSizeZ);
	glVertex3d(-dHalfSizeX, -dHalfSizeY, dHalfSizeZ);
	glEnd();

	glPopMatrix();
	glPolygonMode(GL_FRONT, GL_FILL);
	glDisable(GL_LINE_SMOOTH);
	glDisable(GL_POLYGON_SMOOTH);
}

void CKUNSUI3DDlg::setRobotPosFlag(bool bFlag)
{
	m_bSetRobotPos = bFlag;
	m_yRotate =0;
	m_xRotate = 90;
	m_fXOffset = 0;			//초기화
	m_fYOffset = 0;
	InvalidateRect(NULL,FALSE);
}

void CKUNSUI3DDlg::setGoalPosFlag(bool bFlag)
{
	m_bSetGoalPos = bFlag;
	m_yRotate =0;
	m_xRotate = 90;
	m_fXOffset = 0;			//초기화
	m_fYOffset = 0;
	InvalidateRect(NULL,FALSE);
}


void CKUNSUI3DDlg::setPathBlocksettingFlag(bool bFlag)
{
	m_bsettingflag = bFlag;
	m_yRotate =0;
	m_xRotate = 90;
	m_fXOffset = 0;			//초기화
	m_fYOffset = 0;
	InvalidateRect(NULL,FALSE);
}


void CKUNSUI3DDlg::setPathBlockFlag(bool bFlag)
{
	m_bSetPathBlock = bFlag;
	m_yRotate =0;
	m_xRotate = 90;
	m_fXOffset = 0;			//초기화
	m_fYOffset = 0;
	m_nPathBockID=-1;
	InvalidateRect(NULL,FALSE);
}

void CKUNSUI3DDlg::setPathBlockID(int nPathBockID)
{
	m_nPathBockID = nPathBockID;
	InvalidateRect(NULL,FALSE);
}

void CKUNSUI3DDlg::getMousePointonUI(double* dX,double* dY)
{
	(*dX)=m_dMPoint_x;
	(*dY)=m_dMPoint_y;

}


void CKUNSUI3DDlg::OnMouseMove(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	if(m_bSetRobotPos == false &&  m_bSetGoalPos == false && m_bSetPathBlock==false&&m_bLButtonDown){
		CSize rotate = m_ptLButtonDownPos - point;
		m_ptLButtonDownPos = point;

		m_yRotate -= rotate.cx;
		m_xRotate -= rotate.cy;

		InvalidateRect(NULL,FALSE);
	}
	else if((m_bSetRobotPos||m_bSetPathBlock )&& m_bLButtonDown){

		m_fXOffset -= (float)(m_nLastMousePointX - (int)point.x)/50.0;
		m_fYOffset += (float)(m_nLastMousePointY - (int)point.y)/50.0;
		m_nLastMousePointX = point.x;
		m_nLastMousePointY = point.y;
		InvalidateRect(NULL,FALSE);
	}


	//if(m_bSetPathBlock&&m_nPathBockID!=-1 )
	{
		KuPose RobotPos=KuDrawingInfo::getInstance()->getRobotPos();
		GLdouble model[16];
		GLdouble proj[16];
		GLint viewport[4];
		glGetDoublev(GL_MODELVIEW_MATRIX,model);
		glGetDoublev(GL_PROJECTION_MATRIX,proj);
		glGetIntegerv(GL_VIEWPORT,viewport);	

		double pos[3];
		memset(pos,0,sizeof(pos));
		double win[3]={ (double)point.x , viewport[3] - (double)point.y,0};

		gluUnProject(win[0], win[1],win[2],model,proj,viewport, &pos[0],&pos[1],&pos[2]);
		glPopMatrix();		

		m_dMPoint_x=pos[0] *(13.0 - m_dZoom)+RobotPos.getXm()-m_fXOffset;
		m_dMPoint_y=pos[1] *(13.0 - m_dZoom)+RobotPos.getYm()-m_fYOffset;

		InvalidateRect(NULL,FALSE);
	}
	

	CDialog::OnMouseMove(nFlags, point);

}

void CKUNSUI3DDlg::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	m_bLButtonDown = TRUE;
	m_ptLButtonDownPos = point;

	m_nLastMousePointX = point.x;
	m_nLastMousePointY = point.y;
	

	CDialog::OnLButtonDown(nFlags, point);
}

void CKUNSUI3DDlg::OnLButtonUp(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	if(m_bLButtonDown){
//		CSize rotate = m_ptLButtonDownPos - point;
		m_ptLButtonDownPos = point;
// 		m_yRotate -= rotate.cx;
// 		m_xRotate -= rotate.cy;

		if(m_bsettingflag==false&&m_bSetPathBlock==true)
		{
			vector<PBlock> vecPathBlock;
			vector<PBlock> vecPrePathBlock;
			KuDrawingInfo::getInstance()->getPathBlockPos(&vecPrePathBlock);
			vector<int>  vecErPathBlockID;

			for(int i=0; i<vecPrePathBlock.size();i++)
			{
				vecPathBlock.push_back(vecPrePathBlock[i]);
			}

			for(int i=0; i<vecPathBlock.size(); i++)
			{
				double dX= vecPathBlock[i].x;
				double dY= vecPathBlock[i].y;
				double dSizeX=1.0/2.0;
				double dSizeY=1.0/2.0;

				if(dX-dSizeX<m_dMPoint_x&&dX+dSizeX>m_dMPoint_x
					&&dY-dSizeY<m_dMPoint_y&&dY+dSizeY>m_dMPoint_y)
				{
					vecErPathBlockID.push_back(i);
				}			
			}		

			vector<PBlock>::iterator iter;

			for(int i=0; i<vecErPathBlockID.size();i++)
			{
				iter = vecPathBlock.begin()+vecErPathBlockID[i]-i;
				vecPathBlock.erase(iter);
			}

			KuDrawingInfo::getInstance()->setPathBlockPos(vecPathBlock);
		}
		
		InvalidateRect(NULL,FALSE);

	}	

	m_bLButtonDown = FALSE;

	CDialog::OnLButtonUp(nFlags, point);
}

void CKUNSUI3DDlg::OnRButtonDown(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	m_bRButtonDown = TRUE;
	m_RButtonDownPos = point;

	m_nLastMousePointX = point.x;
	m_nLastMousePointY = point.y;

	CDialog::OnRButtonDown(nFlags, point);
}

void CKUNSUI3DDlg::OnRButtonUp(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	m_bRButtonDown = FALSE;
	int nUISize = 250;		//DrawUI 사이즈 640*640;
	double dUI2Grid = 30;	//한격자간격은 4 point간격

	KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
	KuPose GoalPos;
	KuPose PathBlockPos;
	

	if(m_bSetRobotPos)
	{	
		glPushMatrix();
		glRotatef(m_xRotate, 1.0, 0.0, 0.0);
		glRotatef(m_yRotate, 0.0, 1.0, 0.0);
		glRotatef(-90.0, 1.0, 0.0, 0.0);	

		GLdouble model[16];
		GLdouble proj[16];
		GLint viewport[4];
		glGetDoublev(GL_MODELVIEW_MATRIX,model);
		glGetDoublev(GL_PROJECTION_MATRIX,proj);
		glGetIntegerv(GL_VIEWPORT,viewport);	

		double pos[3];
		memset(pos,0,sizeof(pos));
		double win[3]={ (double)point.x , viewport[3] - (double)point.y,0};

		gluUnProject(win[0], win[1],win[2],model,proj,viewport, &pos[0],&pos[1],&pos[2]);
		glPopMatrix();		

		RobotPos.setXm(pos[0] *(13.0 - m_dZoom)+RobotPos.getXm());
		RobotPos.setYm(pos[1] *(13.0 - m_dZoom)+RobotPos.getYm());
		KuDrawingInfo::getInstance()->setRobotPos(RobotPos);

	}
	else if(m_bSetGoalPos){	

		glPushMatrix();
		glRotatef(m_xRotate, 1.0, 0.0, 0.0);
		glRotatef(m_yRotate, 0.0, 1.0, 0.0);
		glRotatef(-90.0, 1.0, 0.0, 0.0);	

		GLdouble model[16];
		GLdouble proj[16];
		GLint viewport[4];
		glGetDoublev(GL_MODELVIEW_MATRIX,model);
		glGetDoublev(GL_PROJECTION_MATRIX,proj);
		glGetIntegerv(GL_VIEWPORT,viewport);	

		double pos[3];
		memset(pos,0,sizeof(pos));
		double win[3]={ (double)point.x , viewport[3] - (double)point.y,0};

		gluUnProject(win[0], win[1],win[2],model,proj,viewport, &pos[0],&pos[1],&pos[2]);
		glPopMatrix();		

		GoalPos.setXm(pos[0] *(13.0 - m_dZoom)+RobotPos.getXm());
		GoalPos.setYm(pos[1] *(13.0 - m_dZoom)+RobotPos.getYm());
		KuDrawingInfo::getInstance()->setGoalPos(GoalPos);

	}
	else if(m_bSetPathBlock&&m_bsettingflag==false)
	{
		glPushMatrix();
		glRotatef(m_xRotate, 1.0, 0.0, 0.0);
		glRotatef(m_yRotate, 0.0, 1.0, 0.0);
		glRotatef(-90.0, 1.0, 0.0, 0.0);	

		GLdouble model[16];
		GLdouble proj[16];
		GLint viewport[4];
		glGetDoublev(GL_MODELVIEW_MATRIX,model);
		glGetDoublev(GL_PROJECTION_MATRIX,proj);
		glGetIntegerv(GL_VIEWPORT,viewport);	

		double pos[3];
		memset(pos,0,sizeof(pos));
		double win[3]={ (double)point.x , viewport[3] - (double)point.y,0};

		gluUnProject(win[0], win[1],win[2],model,proj,viewport, &pos[0],&pos[1],&pos[2]);
		glPopMatrix();		
		PathBlockPos.setID(m_nPathBockID);
 		PathBlockPos.setXm(pos[0] *(13.0 - m_dZoom)+RobotPos.getXm()-m_fXOffset);
 		PathBlockPos.setYm(pos[1] *(13.0 - m_dZoom)+RobotPos.getYm()-m_fYOffset);

		m_dMPoint_x=PathBlockPos.getXm();
		m_dMPoint_y=PathBlockPos.getYm();

		vector<PBlock> vecPrePathBlock;
		KuDrawingInfo::getInstance()->getPathBlockPos(&vecPrePathBlock);
		vector<PBlock> vecPathBlockPos;
		vector<PBlock> vecCurPathBlockPos;

		KuPathBlockPr CPathBlock;
		PBlock PB= CPathBlock.getPathBlock(m_nPathBockID);
		PB.x=PathBlockPos.getXm();
		PB.y=PathBlockPos.getYm();

		for(int i=0; i<vecPrePathBlock.size();i++)
		{
			vecPathBlockPos.push_back(vecPrePathBlock[i]);
		}


		if(vecPathBlockPos.size()>0)
		{	
			//겹치면 등록 안함	
			bool bOverlap= checkOverlap(vecPathBlockPos);
			
			//끝점 맞추기
			bool bBoundary= sticktoPoint(&vecPathBlockPos, PB);

			if(bOverlap==false&&bBoundary== false)
			{
				vecPathBlockPos.push_back(PB);
			}		
		}
		else
		{
			vecPathBlockPos.push_back(PB);
		}
		

		for(int i=0; i<vecPathBlockPos.size();i++)
		{
			vecCurPathBlockPos.push_back(vecPathBlockPos[i]);
		}


 		KuDrawingInfo::getInstance()->setPathBlockPos(vecCurPathBlockPos);
		//KuDrawingInfo::getInstance()->setRobotPos(PathBlockPos);

	}



	InvalidateRect(NULL,FALSE);
	CDialog::OnRButtonUp(nFlags, point);
}

bool CKUNSUI3DDlg::sticktoPoint(vector<PBlock> *vecPathBlockPos,PBlock PB)
{
	bool bBoundary= false;
	double dstickSize=0.3;

	for(int i=0; i<(*vecPathBlockPos).size(); i++)
	{

		double dX= (*vecPathBlockPos)[i].x;
		double dY= (*vecPathBlockPos)[i].y;
		double dSizeX=(*vecPathBlockPos)[i].sizex;
		double dSizeY=(*vecPathBlockPos)[i].sizey;

		double dCurStx=PB.x+PB.dist1.start_x;
		double dCurSty=PB.y+PB.dist1.start_y;
		double dCurEndx=PB.x+PB.dist1.end_x;
		double dCurEndy=PB.y+PB.dist1.end_y;

		double dPreStx=(*vecPathBlockPos)[i].x+(*vecPathBlockPos)[i].dist1.start_x;
		double dPreSty=(*vecPathBlockPos)[i].y+(*vecPathBlockPos)[i].dist1.start_y;
		double dPreEndx=(*vecPathBlockPos)[i].x+(*vecPathBlockPos)[i].dist1.end_x;
		double dPreEndy=(*vecPathBlockPos)[i].y+(*vecPathBlockPos)[i].dist1.end_y;

		if(dstickSize>=fabs(dCurEndx-dPreStx)&&dstickSize>=fabs(dCurEndy-dPreSty))
		{
			PB.x=dPreStx-PB.dist1.end_x;
			PB.y=dPreSty-PB.dist1.end_y;

			if(PB.sizex/2.0<=fabs(PB.x-(*vecPathBlockPos)[i].x)
				||PB.sizey/2.0<=fabs(PB.y-(*vecPathBlockPos)[i].y))
			{
				(*vecPathBlockPos).push_back(PB);
				bBoundary= true;
				break;
			}

		
		}	
		else if(dstickSize>=fabs(dCurStx-dPreStx)&&dstickSize>=fabs(dCurSty-dPreSty))
		{
			PB.x=dPreStx-PB.dist1.start_x;
			PB.y=dPreSty-PB.dist1.start_y;
			if(PB.sizex/2.0<=fabs(PB.x-(*vecPathBlockPos)[i].x)
				||PB.sizey/2.0<=fabs(PB.y-(*vecPathBlockPos)[i].y))
			{
				(*vecPathBlockPos).push_back(PB);
				bBoundary= true;
				break;
			}
		
		}
		else if(dstickSize>=fabs(dCurEndx-dPreEndx)&&dstickSize>=fabs(dCurEndy-dPreEndy))
		{
			PB.x=dPreEndx-PB.dist1.end_x;
			PB.y=dPreEndy-PB.dist1.end_y;
			if(PB.sizex/2.0<=fabs(PB.x-(*vecPathBlockPos)[i].x)
				||PB.sizey/2.0<=fabs(PB.y-(*vecPathBlockPos)[i].y))
			{
				(*vecPathBlockPos).push_back(PB);
				bBoundary= true;
				break;
			}

		}	
		else if(dstickSize>=fabs(dCurStx-dPreEndx)&&dstickSize>=fabs(dCurSty-dPreEndy))
		{
			PB.x=dPreEndx-PB.dist1.start_x;
			PB.y=dPreEndy-PB.dist1.start_y;
			if(PB.sizex/2.0<=fabs(PB.x-(*vecPathBlockPos)[i].x)
				||PB.sizey/2.0<=fabs(PB.y-(*vecPathBlockPos)[i].y))
			{
				(*vecPathBlockPos).push_back(PB);
				bBoundary= true;
				break;
			}

		}	

		if((*vecPathBlockPos)[i].overlap==true)
		{
			double dCurStx=PB.x+PB.dist1.start_x;
			double dCurSty=PB.y+PB.dist1.start_y;
			double dCurEndx=PB.x+PB.dist1.end_x;
			double dCurEndy=PB.y+PB.dist1.end_y;

			double dPreStx=(*vecPathBlockPos)[i].x+(*vecPathBlockPos)[i].dist2.start_x;
			double dPreSty=(*vecPathBlockPos)[i].y+(*vecPathBlockPos)[i].dist2.start_y;
			double dPreEndx=(*vecPathBlockPos)[i].x+(*vecPathBlockPos)[i].dist2.end_x;
			double dPreEndy=(*vecPathBlockPos)[i].y+(*vecPathBlockPos)[i].dist2.end_y;

			if(dstickSize>=fabs(dCurEndx-dPreStx)&&dstickSize>=fabs(dCurEndy-dPreSty))
			{
				PB.x=dPreStx-PB.dist1.end_x;
				PB.y=dPreSty-PB.dist1.end_y;
				if(PB.sizex/2.0<=fabs(PB.x-(*vecPathBlockPos)[i].x)
					||PB.sizey/2.0<=fabs(PB.y-(*vecPathBlockPos)[i].y))
				{
					(*vecPathBlockPos).push_back(PB);
					bBoundary= true;
					break;
				}

			}	
			else if(dstickSize>=fabs(dCurStx-dPreStx)&&dstickSize>=fabs(dCurSty-dPreSty))
			{
				PB.x=dPreStx-PB.dist1.start_x;
				PB.y=dPreSty-PB.dist1.start_y;
				if(PB.sizex/2.0<=fabs(PB.x-(*vecPathBlockPos)[i].x)
					||PB.sizey/2.0<=fabs(PB.y-(*vecPathBlockPos)[i].y))
				{
					(*vecPathBlockPos).push_back(PB);
					bBoundary= true;
					break;
				}
	
			}
			else if(dstickSize>=fabs(dCurEndx-dPreEndx)&&dstickSize>=fabs(dCurEndy-dPreEndy))
			{
				PB.x=dPreEndx-PB.dist1.end_x;
				PB.y=dPreEndy-PB.dist1.end_y;
				if(PB.sizex/2.0<=fabs(PB.x-(*vecPathBlockPos)[i].x)
					||PB.sizey/2.0<=fabs(PB.y-(*vecPathBlockPos)[i].y))
				{
					(*vecPathBlockPos).push_back(PB);
					bBoundary= true;
					break;
				}

			}	
			else if(dstickSize>=fabs(dCurStx-dPreEndx)&&dstickSize>=fabs(dCurSty-dPreEndy))
			{
				PB.x=dPreEndx-PB.dist1.start_x;
				PB.y=dPreEndy-PB.dist1.start_y;
				if(PB.sizex/2.0<=fabs(PB.x-(*vecPathBlockPos)[i].x)
					||PB.sizey/2.0<=fabs(PB.y-(*vecPathBlockPos)[i].y))
				{
					(*vecPathBlockPos).push_back(PB);
					bBoundary= true;
					break;
				}

			}	

		}

		if(PB.overlap==true)
		{
			double dCurStx=PB.x+PB.dist2.start_x;
			double dCurSty=PB.y+PB.dist2.start_y;
			double dCurEndx=PB.x+PB.dist2.end_x;
			double dCurEndy=PB.y+PB.dist2.end_y;

			double dPreStx=(*vecPathBlockPos)[i].x+(*vecPathBlockPos)[i].dist1.start_x;
			double dPreSty=(*vecPathBlockPos)[i].y+(*vecPathBlockPos)[i].dist1.start_y;
			double dPreEndx=(*vecPathBlockPos)[i].x+(*vecPathBlockPos)[i].dist1.end_x;
			double dPreEndy=(*vecPathBlockPos)[i].y+(*vecPathBlockPos)[i].dist1.end_y;

			if(dstickSize>=fabs(dCurEndx-dPreStx)&&dstickSize>=fabs(dCurEndy-dPreSty))
			{
				PB.x=dPreStx-PB.dist2.end_x;
				PB.y=dPreSty-PB.dist2.end_y;
				if(PB.sizex/2.0<=fabs(PB.x-(*vecPathBlockPos)[i].x)
					||PB.sizey/2.0<=fabs(PB.y-(*vecPathBlockPos)[i].y))
				{
					(*vecPathBlockPos).push_back(PB);
					bBoundary= true;
					break;
				}

			}	
			else if(dstickSize>=fabs(dCurStx-dPreStx)&&dstickSize>=fabs(dCurSty-dPreSty))
			{
				PB.x=dPreStx-PB.dist2.start_x;
				PB.y=dPreSty-PB.dist2.start_y;
				if(PB.sizex/2.0<=fabs(PB.x-(*vecPathBlockPos)[i].x)
					||PB.sizey/2.0<=fabs(PB.y-(*vecPathBlockPos)[i].y))
				{
					(*vecPathBlockPos).push_back(PB);
					bBoundary= true;
					break;
				}
			}
			else if(dstickSize>=fabs(dCurEndx-dPreEndx)&&dstickSize>=fabs(dCurEndy-dPreEndy))
			{
				PB.x=dPreEndx-PB.dist2.end_x;
				PB.y=dPreEndy-PB.dist2.end_y;
				if(PB.sizex/2.0<=fabs(PB.x-(*vecPathBlockPos)[i].x)
					||PB.sizey/2.0<=fabs(PB.y-(*vecPathBlockPos)[i].y))
				{
					(*vecPathBlockPos).push_back(PB);
					bBoundary= true;
					break;
				}
			}	
			else if(dstickSize>=fabs(dCurStx-dPreEndx)&&dstickSize>=fabs(dCurSty-dPreEndy))
			{
				PB.x=dPreEndx-PB.dist2.start_x;
				PB.y=dPreEndy-PB.dist2.start_y;
				if(PB.sizex/2.0<=fabs(PB.x-(*vecPathBlockPos)[i].x)
					||PB.sizey/2.0<=fabs(PB.y-(*vecPathBlockPos)[i].y))
				{
					(*vecPathBlockPos).push_back(PB);
					bBoundary= true;
					break;
				}		
			}	

		}

		if((*vecPathBlockPos)[i].overlap==true&&PB.overlap==true)
		{
			double dCurStx=PB.x+PB.dist2.start_x;
			double dCurSty=PB.y+PB.dist2.start_y;
			double dCurEndx=PB.x+PB.dist2.end_x;
			double dCurEndy=PB.y+PB.dist2.end_y;

			double dPreStx=(*vecPathBlockPos)[i].x+(*vecPathBlockPos)[i].dist2.start_x;
			double dPreSty=(*vecPathBlockPos)[i].y+(*vecPathBlockPos)[i].dist2.start_y;
			double dPreEndx=(*vecPathBlockPos)[i].x+(*vecPathBlockPos)[i].dist2.end_x;
			double dPreEndy=(*vecPathBlockPos)[i].y+(*vecPathBlockPos)[i].dist2.end_y;

			if(dstickSize>=fabs(dCurEndx-dPreStx)&&dstickSize>=fabs(dCurEndy-dPreSty))
			{
				PB.x=dPreStx-PB.dist2.end_x;
				PB.y=dPreSty-PB.dist2.end_y;
				if(PB.sizex/2.0<=fabs(PB.x-(*vecPathBlockPos)[i].x)
					||PB.sizey/2.0<=fabs(PB.y-(*vecPathBlockPos)[i].y))
				{
					(*vecPathBlockPos).push_back(PB);
					bBoundary= true;
					break;
				}
			}	
			else if(dstickSize>=fabs(dCurStx-dPreStx)&&dstickSize>=fabs(dCurSty-dPreSty))
			{
				PB.x=dPreStx-PB.dist2.start_x;
				PB.y=dPreSty-PB.dist2.start_y;
				if(PB.sizex/2.0<=fabs(PB.x-(*vecPathBlockPos)[i].x)
					||PB.sizey/2.0<=fabs(PB.y-(*vecPathBlockPos)[i].y))
				{
					(*vecPathBlockPos).push_back(PB);
					bBoundary= true;
					break;
				}

			}
			else if(dstickSize>=fabs(dCurEndx-dPreEndx)&&dstickSize>=fabs(dCurEndy-dPreEndy))
			{
				PB.x=dPreEndx-PB.dist2.end_x;
				PB.y=dPreEndy-PB.dist2.end_y;
				if(PB.sizex/2.0<=fabs(PB.x-(*vecPathBlockPos)[i].x)
					||PB.sizey/2.0<=fabs(PB.y-(*vecPathBlockPos)[i].y))
				{
					(*vecPathBlockPos).push_back(PB);
					bBoundary= true;
					break;
				}	
			}	
			else if(dstickSize>=fabs(dCurStx-dPreEndx)&&dstickSize>=fabs(dCurSty-dPreEndy))
			{
				PB.x=dPreEndx-PB.dist2.start_x;
				PB.y=dPreEndy-PB.dist2.start_y;
				if(PB.sizex/2.0<=fabs(PB.x-(*vecPathBlockPos)[i].x)
					||PB.sizey/2.0<=fabs(PB.y-(*vecPathBlockPos)[i].y))
				{
					(*vecPathBlockPos).push_back(PB);
					bBoundary= true;
					break;
				}

			}	

		}
	}


	return bBoundary;
}
bool CKUNSUI3DDlg::checkOverlap(vector<PBlock> vecPathBlockPos)
{
	bool bOverlap= false;

	for(int i=0; i<vecPathBlockPos.size(); i++)
	{
		double dX= vecPathBlockPos[i].x;
		double dY= vecPathBlockPos[i].y;
		double dSizeX=vecPathBlockPos[i].sizex;
		double dSizeY=vecPathBlockPos[i].sizey;

		if(dSizeX>fabs(m_dMPoint_x-dX)&&dSizeY>fabs(m_dMPoint_y-dY))
		{
			bOverlap= true;
		}			
	}

	return bOverlap;
}
vector<PBlock>  CKUNSUI3DDlg::getPathBlockPos()
{
	vector<PBlock> vecPathBlock;
	KuDrawingInfo::getInstance()->getPathBlockPos(&vecPathBlock);
	return vecPathBlock;
}
void  CKUNSUI3DDlg::setPathBlockPos(vector<PBlock> vecPathBlock)
{
	KuDrawingInfo::getInstance()->setPathBlockPos(vecPathBlock);
}
// Create font
GLFONT* CKUNSUI3DDlg::OpenGL_Font_Create(HDC hdc, const char* typeface, int height, int weight, DWORD italic)
{
	GLFONT *font;
	HFONT fontid;
	int charset;

	if((font = (GLFONT *)calloc(1, sizeof(GLFONT))) == (GLFONT *)0)
		return((GLFONT *)0);

	if((font->base = glGenLists(256)) == 0)
	{
		free(font);
		return (0);
	}

	if(_stricmp(typeface, "symbol") == 0)
		charset = SYMBOL_CHARSET;
	else
		charset = ANSI_CHARSET;

	fontid = CreateFont(height, 0, 0, 0, weight, italic, FALSE, FALSE,
		charset, OUT_TT_PRECIS, CLIP_DEFAULT_PRECIS,
		DRAFT_QUALITY, DEFAULT_PITCH, (LPCWSTR)typeface);

	SelectObject(hdc, fontid);

	wglUseFontBitmaps(hdc, 0, 256, font->base);
	GetCharWidth(hdc, 0, 255, font->widths);
	font->height = height;

	return (font);
}

// Destroy the font
void CKUNSUI3DDlg::OpenGL_Font_Destroy(GLFONT* font)
{
	if(font == (GLFONT *)0)
		return;

	glDeleteLists(font->base, 256);
	free(font);
}

// Print the font
void CKUNSUI3DDlg::OpenGL_Font_Printf(GLFONT* font, int align, const char* format)
{
	glDisable(GL_LIGHTING);

	va_list ap;
	unsigned char s[1024], *ptr;
	int width;

	if(font == (GLFONT *)0 || format == (char *)0)
		return;

	va_start(ap, format);
	vsprintf((char *)s, format, ap);
	va_end(ap);

	for(ptr = s, width = 0; *ptr; ptr ++)
		width += font->widths[*ptr];

	if(align < 0)
		glBitmap(0, 0, 0.0f, 0.0f, (float)-width, 0.0f, NULL);
	else if(align == 0)
		glBitmap(0, 0, 0.0f, 0.0f, (float)-width / (float)2, 0.0f, NULL);

	OpenGL_Font_Puts(font, (const char *)s);

	glEnable(GL_LIGHTING);
}

// Put "s" to the font
void CKUNSUI3DDlg::OpenGL_Font_Puts(GLFONT* font, const char* s)
{
	if(font == (GLFONT *)0 || s == NULL)
		return;

	glPushAttrib(GL_LIST_BIT);
	glListBase(font->base);
	glCallLists(strlen(s), GL_UNSIGNED_BYTE, s);
	glPopAttrib();
}

// Render x, y, z axes
void CKUNSUI3DDlg::Render_Axis(void)
{
	char c[4];

	glPushMatrix();

	glColor3ub(100, 100, 100);

	glRasterPos3f(1.2, 0, -0.1);
	sprintf_s(c, "X");
	OpenGL_Font_Printf(m_pFont, 0, c);

	glRasterPos3f(0, 1.2, -0.1);
	sprintf_s(c, "Y");
	OpenGL_Font_Printf(m_pFont, 0, c);

	glRasterPos3f(0, 0, 1.2);
	sprintf_s(c, "Z");
	OpenGL_Font_Printf(m_pFont, 0, c);

	glColor3ub(255, 0, 0);

	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(1, 0, 0);
	glEnd();

	glColor3ub(0, 255, 0);

	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 1, 0);
	glEnd();

	glColor3ub(0, 0, 255);

	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, 1);
	glEnd();

	glPopMatrix();
}

void CKUNSUI3DDlg::OnDestroy()
{
	CDialog::OnDestroy();

	// TODO: Add your message handler code here
	if(wglGetCurrentContext() != NULL)
		wglMakeCurrent(NULL,NULL);

	if(m_hGLContext != NULL)
	{
		wglDeleteContext(m_hGLContext);
		m_hGLContext = NULL;
	}

	OpenGL_Font_Destroy(m_pFont);
}

BOOL CKUNSUI3DDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// TODO:  Add extra initialization here
	HWND hWnd = GetSafeHwnd();
	HDC hDC = ::GetDC(hWnd);

	if(OpenGL_SetupPixelFormat(hDC) == false)
		return 0;

	if(OpenGL_CreateViewGLContext(hDC)==false)
		return 0;

	glShadeModel(GL_SMOOTH);
	glClearColor((float)1.0, (float)1.0, (float)1.0, 1.0);
	//	glClearDepth(1.0f);

	//	glPolygonMode(GL_FRONT,GL_LINE);
	//	glPolygonMode(GL_BACK,GL_LINE);

	//	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_BLEND);
	glEnable(GL_CULL_FACE);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	glHint(GL_LINE_SMOOTH_HINT,GL_NICEST);

	// light
	GLfloat ambientLight[] = {0.5, 0.5, 0.5, 1.0};
	GLfloat diffuseLight[] = {0.5, 1.0, 1.0, 1.0};
	GLfloat specular[] = {1.0, 1.0, 1.0, 1.0};

	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);

	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

	glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
	//	glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
	glEnable(GL_LIGHT0);

	glEnable(GL_NORMALIZE);
	//	glClearColor(0, 0, 0, 1.0);

	m_pFont = OpenGL_Font_Create(hDC, "Verdana", 13, 1, 0);

	Initialize();

	return TRUE;  // return TRUE unless you set the focus to a control
	// EXCEPTION: OCX Property Pages should return FALSE
}

void CKUNSUI3DDlg::OnSize(UINT nType, int cx, int cy)
{
	CDialog::OnSize(nType, cx, cy);

	// TODO: Add your message handler code here
	GLsizei width,height;
	GLdouble aspect;

	width = cx;
	height = cy;

	if(cy==0)
		aspect = (GLdouble)width;
	else
		aspect = (GLdouble)width/(GLdouble)height;

	glViewport(0,0,width,height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45,aspect,1,200.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glDrawBuffer(GL_BACK);

	glEnable(GL_DEPTH_TEST);
}


void CKUNSUI3DDlg::Initialize()
{
	m_bChangePose = false;

	m_bLButtonDown = FALSE;
	m_bRButtonDown = FALSE;
	m_fXOffset = 0.0;
	m_fYOffset = 0.0;

	m_nMapSizeX = 0;
	m_nMapSizeY = 0;


	m_dCellSize = (double)0.1;

	// 전체 지도 중 그림을 그릴 크기
	float fTemp = (float)0.1;
	m_nDrawingSizeHalf = (int)(50.0 / fTemp);	// 상하좌우 20m 범위를 그리는 것이 초기값.

}

// Render grid
void CKUNSUI3DDlg::Render_Grid(const int& nMapWidth, const int& nMapHeight)
{

	double i, j;
	double dHalfSizeX, dHalfSizeY;

	dHalfSizeX = nMapWidth * 0.1 / 2.;
	dHalfSizeY = nMapHeight * 0.1 / 2.;

	glColor4ub(200,200,200,60);

	glPushMatrix();

	glTranslated(100,100,0);


	for(i = -dHalfSizeX; i <= dHalfSizeX; i++)
	{
		glBegin(GL_LINES);
		glVertex3f((float)i, dHalfSizeY, -0.0f);
		glVertex3f((float)i, -dHalfSizeY, -0.0f);
		glEnd();
	}

	for(j = -dHalfSizeY; j <= dHalfSizeY; j++)
	{
		glBegin(GL_LINES);
		glVertex3f(-dHalfSizeX, (float)j, -0.0f);
		glVertex3f(dHalfSizeX, (float)j, -0.0f);
		glEnd();
	}

	glColor4ub(200,200,200,10);

	for(i = -dHalfSizeX; i <= dHalfSizeX; i+=0.1)
	{
		glBegin(GL_LINES);
		glVertex3f((float)i, dHalfSizeY, -0.0f);
		glVertex3f((float)i, -dHalfSizeY, -0.0f);
		glEnd();
	}

	for(j = -dHalfSizeY; j <= dHalfSizeY; j+=0.1)
	{
		glBegin(GL_LINES);
		glVertex3f(-dHalfSizeX, (float)j, -0.0f);
		glVertex3f(dHalfSizeX, (float)j, -0.0f);
		glEnd();
	}

	glPopMatrix();
}

// Create OpenGL Context
bool CKUNSUI3DDlg::OpenGL_CreateViewGLContext(HDC hDC)
{
	m_hGLContext = wglCreateContext(hDC);

	if(m_hGLContext==NULL)
		return FALSE;

	if(wglMakeCurrent(hDC,m_hGLContext)==FALSE)
		return FALSE;

	return TRUE;
}

void CKUNSUI3DDlg::Zoom_In(double dVal)
{
	m_dZoom += dVal;
	m_nDrawingSizeHalf -= (int)(10.0/m_dCellSize);
	if (m_nDrawingSizeHalf<=(int)(5.0/m_dCellSize)){
		m_nDrawingSizeHalf = (int)(15.0/m_dCellSize);	// 최소 15m는 그리도록
	}
	InvalidateRect(NULL,FALSE);
}

void CKUNSUI3DDlg::Zoom_Out(double dVal)
{
	m_dZoom -= dVal;
	m_nDrawingSizeHalf += (int)(10.0/m_dCellSize);


	if (m_nMapSizeX>=m_nMapSizeY &&  m_nDrawingSizeHalf>m_nMapSizeX/2)
		m_nDrawingSizeHalf = m_nMapSizeX/2;
	else if (m_nMapSizeX<m_nMapSizeY &&  m_nDrawingSizeHalf>m_nMapSizeY/2)
		m_nDrawingSizeHalf = m_nMapSizeY/2;

	InvalidateRect(NULL,FALSE);
}

void CKUNSUI3DDlg::setCamSizeInfo(int nSizeWidth,int nSizeHeight)
{
	m_nSizeWidth=nSizeWidth;
	m_nSizeHeight=nSizeHeight;

}

void CKUNSUI3DDlg::setCamInfoDC(CDC* pDC)
{
	m_pCamDC = pDC;	
}

void CKUNSUI3DDlg::setCamInfo2DC(CDC* pDC)
{
	m_pCamDC2 = pDC;	
}


void CKUNSUI3DDlg::renderCamImageData()
{
	KuDrawingInfo::getInstance()->getCeilingImageData(m_IplCeilingImage);



	CRect rect = CRect(0,0,m_nSizeWidth, m_nSizeHeight);
	//칼라 이미지 그리는 부분-------------------
	m_cvCeilingImage.CopyOf(m_IplCeilingImage,8);
	HDC hDC = m_pCamDC->GetSafeHdc();
	m_cvCeilingImage.DrawToHDC(hDC,&rect);
	//칼라 이미지 그리는 부분 끝-----------------

	KuDrawingInfo::getInstance()->getKinectImageData(m_IplKinectImg);

	CRect rect2 = CRect(0,0,m_nSizeWidth, m_nSizeHeight);
	//칼라 이미지 그리는 부분-------------------
	m_cvKinectImage.CopyOf(m_IplKinectImg,8);
	HDC hDC2 = m_pCamDC2->GetSafeHdc();
	m_cvKinectImage.DrawToHDC(hDC2,&rect2);
	//칼라 이미지 그리는 부분 끝-----------------

}

void CKUNSUI3DDlg::renderVirtualRobot()
{
	glPushMatrix();	

	KuPose VirtualRobotPos;
	VirtualRobotPos = KuDrawingInfo::getInstance()->getAuxiliaryRobotPos();

	glTranslated(VirtualRobotPos.getXm(), VirtualRobotPos.getYm(), 0);
	glRotatef(VirtualRobotPos.getThetaDeg(), 0.0, 0.0, 1.0);

	//------------------------ pioneer 3AT 비슷하게.... --------------------------------------//
	// 변수 지정해서 하려 했는데 이상하게 꼬였음.... ㅠㅠ
	double h1 = 0.07;
	double h2 = 0.25;
	double h3 = 0.27;
	double dx1 = 0.2;
	double dy1 = 0.12;
	double dx11 = 0.24;
	double dx2 = 0.23;
	double dy2 = 0.15;
	double dx22 = 0.27;
	double r1 = 0.1;
	double r2 = 0.05;
	double wx = 0.13;
	double wy = 0.17;

	// ----------------------- 옆면 4개
	glColor3ub(0, 0, 255);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx1, -dy1, h2);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, h2);
	glVertex3f(dx1, -dy1, h1);

	glVertex3f(dx11, 0, h2);
	glVertex3f(dx11, 0, h1);

	glVertex3f(dx1, dy1, h2);
	glVertex3f(dx1, dy1, h1);

	glVertex3f(-dx1, dy1, h2);
	glVertex3f(-dx1, dy1, h1);

	glVertex3f(-dx11, 0, h2);
	glVertex3f(-dx11, 0, h1);

	glVertex3f(-dx1, -dy1, h2);
	glVertex3f(-dx1, -dy1, h1);
	glEnd();

	// 모서리에 라인을 그려서 강조
	glColor3ub(0, 0, 255);
	glBegin(GL_LINES);
	glVertex3f(-dx1, -dy1, h2);
	glVertex3f(-dx1, -dy1, h1);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(dx1, -dy1, h2);
	glVertex3f(dx1, -dy1, h1);
	glEnd();
	glBegin(GL_LINES);	
	glVertex3f(dx11, 0, h2);
	glVertex3f(dx11, 0, h1);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(dx1, dy1, h2);
	glVertex3f(dx1, dy1, h1);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(-dx1, dy1, h2);
	glVertex3f(-dx1, dy1, h1);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(-dx11, 0, h2);
	glVertex3f(-dx11, 0, h1);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(-dx1, -dy1, h2);
	glVertex3f(-dx1, -dy1, h1);
	glEnd();

	//------------------------------------- 윗면
	glColor3ub(0, 0, 255);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(-dx2, -dy2, h2);
	glVertex3f(dx2, -dy2, h3);
	glVertex3f(dx2, -dy2, h2);

	glVertex3f(dx22, 0, h3);
	glVertex3f(dx22, 0, h2);

	glVertex3f(dx2, dy2, h3);
	glVertex3f(dx2, dy2, h2);

	glVertex3f(-dx2, dy2, h3);
	glVertex3f(-dx2, dy2, h2);

	glVertex3f(-dx22, 0, h3);
	glVertex3f(-dx22, 0, h2);

	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(-dx2, -dy2, h2);
	glEnd();
	glBegin(GL_POLYGON);
	glVertex3f(dx22, 0, h3);
	glVertex3f(dx2, dy2, h3);
	glVertex3f(-dx2, dy2, h3);
	glVertex3f(-dx22, 0, h3);
	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(dx2, -dy2, h3);
	glEnd();
	// 기둥 4개
	glPushMatrix();
	glTranslated(0.15, 0.1, h3+0.1);
	glRotatef(-180, 1.0, 0.0, 0.0);
	glutSolidCone(0.02, 0.2, 4, 4);
	glPopMatrix();
	glPushMatrix();
	glTranslated(-0.15, 0.1, h3+0.1);
	glRotatef(-180, 1.0, 0.0, 0.0);
	glutSolidCone(0.02, 0.2, 4, 4);
	glPopMatrix();
	glPushMatrix();
	glTranslated(-0.15, -0.1, h3+0.1);
	glRotatef(-180, 1.0, 0.0, 0.0);
	glutSolidCone(0.02, 0.2, 4, 4);
	glPopMatrix();
	glPushMatrix();
	glTranslated(0.15, -0.1, h3+0.1);
	glRotatef(-180, 1.0, 0.0, 0.0);
	glutSolidCone(0.02, 0.2, 4, 4);
	glPopMatrix();

	//------------------------------------- 윗면2
	glPushMatrix();
	glTranslated(0,0,0.1);
	glColor3ub(0, 0, 255);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(-dx2, -dy2, h2);
	glVertex3f(dx2, -dy2, h3);
	glVertex3f(dx2, -dy2, h2);

	glVertex3f(dx22, 0, h3);
	glVertex3f(dx22, 0, h2);

	glVertex3f(dx2, dy2, h3);
	glVertex3f(dx2, dy2, h2);

	glVertex3f(-dx2, dy2, h3);
	glVertex3f(-dx2, dy2, h2);

	glVertex3f(-dx22, 0, h3);
	glVertex3f(-dx22, 0, h2);

	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(-dx2, -dy2, h2);
	glEnd();
	glBegin(GL_POLYGON);
	glVertex3f(dx22, 0, h3);
	glVertex3f(dx2, dy2, h3);
	glVertex3f(-dx2, dy2, h3);
	glVertex3f(-dx22, 0, h3);
	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(dx2, -dy2, h3);
	glEnd();
	glPopMatrix();

	//---------------------------------------------------- 바퀴
	glPushMatrix();
	glColor3ub(0, 0, 255);
	glTranslated(wx,wy,r1);
	glRotatef(90, 1.0, 0.0, 0.0);
	glutSolidTorus(0.05, r1-0.05, 8, 10);
	glTranslated(-2.0*wx,0,0);
	glutSolidTorus(0.05, r1-0.05, 8, 10);
	glTranslated(0,0,2.0*wy);
	glutSolidTorus(0.05, r1-0.05, 8, 10);
	glTranslated(2.0*wx,0,0);
	glutSolidTorus(0.05, r1-0.05, 8, 10);	
	glPopMatrix();

	// -------------------------------------------- 뒷 박스
	glPushMatrix();
	h3+=0.1;
	glTranslated(-0.15,0, h3);
	dx1 = 0.1;dx2 = dx1;
	dy1 = 0.12; dy2 = dy1;
	h1 = 0.1;
	//옆면 4개
	glColor3ub(0, 0, 255);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glVertex3f(dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, 0);

	glVertex3f(dx1, dy1, h1);
	glVertex3f(dx1, dy1, 0);

	glVertex3f(-dx1, dy1, h1);
	glVertex3f(-dx1, dy1, 0);

	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glEnd();
	//윗면
	glColor3ub(0, 0, 255);
	glBegin(GL_POLYGON);
	glVertex3f(dx2, dy2, h1);
	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(dx2, -dy2, h1);
	glEnd();
	glPopMatrix();

	// -------------------------------------------- 레이저스캐너 기둥
	glPushMatrix();
	glTranslated(0.1,0.1,h3);
	dx1 = 0.03;
	dy1 = 0.01;
	h1 = 0.2;
	//옆면 4개
	glColor3ub(0, 0, 255);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glVertex3f(dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, 0);

	glVertex3f(dx1, dy1, h1);
	glVertex3f(dx1, dy1, 0);

	glVertex3f(-dx1, dy1, h1);
	glVertex3f(-dx1, dy1, 0);

	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glEnd();
	//윗면
	glBegin(GL_POLYGON);
	glVertex3f(dx1, dy1, h1);
	glVertex3f(-dx1, dy1, h1);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, h1);
	glEnd();
	glTranslated(0,-0.2,0);
	//옆면 4개
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glVertex3f(dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, 0);

	glVertex3f(dx1, dy1, h1);
	glVertex3f(dx1, dy1, 0);

	glVertex3f(-dx1, dy1, h1);
	glVertex3f(-dx1, dy1, 0);

	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glEnd();
	//윗면
	glBegin(GL_POLYGON);
	glVertex3f(dx1, dy1, h1);
	glVertex3f(-dx1, dy1, h1);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, h1);
	glEnd();
	glPopMatrix();

	// -------------------------------------------- 레이저스캐너
	glPushMatrix();
	glTranslated(0.1,0,0.55);
	glRotatef(0, 0.0, 1.0, 0.0);
	glTranslated(-0.1,0,-0.55);
	// 맨 아래
	h3+=0.1;
	glTranslated(0.15,0,h3);
	dx1 = 0.075;dx2 = dx1;
	dx11 = 0.095; dx22 = dx11;
	dy1 = 0.075; dy2 = dy1;
	h1 = 0.02;
	//옆면 4개
	glColor3ub(0, 0, 255);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glVertex3f(dx2, -dy2, h1);
	glVertex3f(dx1, -dy2, 0);

	glVertex3f(dx22, 0, h1);
	glVertex3f(dx11, 0, 0);

	glVertex3f(dx2, dy2, h1);
	glVertex3f(dx1, dy2, 0);

	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, dy2, 0);

	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glEnd();
	//윗면
	glColor3ub(0, 0, 255);
	glBegin(GL_POLYGON);
	glVertex3f(dx2, dy2, h1);
	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(dx2, -dy2, h1);
	glVertex3f(dx22, 0, h1);
	glEnd();

	// 중간
	glTranslated(0,0,h1);
	dx1 = 0.075;dx2 = dx1;
	dy1 = 0.075; dy2 = dy1;
	h1 = 0.1;
	//옆면 4개
	glColor3ub(0, 0, 255);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glVertex3f(0, -dy2, h1);
	glVertex3f(0, -dy2, 0);

	glVertex3f(0, dy2, h1);
	glVertex3f(0, dy2, 0);

	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, dy2, 0);

	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glEnd();
	dx1 = 0.045;dx2 = 0.055;
	dx11 = 0.060; dx22 = 0.075;
	dy1 = 0.055; dy2 = dy1;
	h1 = 0.1;
	//옆면 4개
	glColor3ub(0, 0, 255);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(0, -dy2, h1);
	glVertex3f(0, -dy2, 0);
	glVertex3f(dx2, -dy2, h1);
	glVertex3f(dx1, -dy2, 0);

	glVertex3f(dx22, 0, h1);
	glVertex3f(dx11, 0, 0);

	glVertex3f(dx2, dy2, h1);
	glVertex3f(dx1, dy2, 0);

	glVertex3f(0, dy2, h1);
	glVertex3f(0, dy2, 0);

	glVertex3f(0, -dy2, h1);
	glVertex3f(0, -dy2, 0);
	glEnd();


	// 윗 부분
	glTranslated(0,0,h1);
	dx1 = 0.075;dx2 = dx1;
	dx11 = 0.095; dx22 = dx11;
	dy1 = 0.075; dy2 = dy1;
	h1 = 0.06;
	//옆면 4개
	glColor3ub(0, 0, 255);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glVertex3f(dx2, -dy2, h1);
	glVertex3f(dx1, -dy2, 0);

	glVertex3f(dx22, 0, h1);
	glVertex3f(dx11, 0, 0);

	glVertex3f(dx2, dy2, h1);
	glVertex3f(dx1, dy2, 0);

	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, dy2, 0);

	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glEnd();
	//윗면
	glColor3ub(0, 0, 255);
	glBegin(GL_POLYGON);
	glVertex3f(dx2, dy2, h1);
	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(dx2, -dy2, h1);
	glVertex3f(dx22, 0, h1);
	glEnd();

	glPopMatrix();
	//-------------------------------------------------------------------------------------------//
	glPopMatrix();



}


void CKUNSUI3DDlg::renderRobot()
{

	glPushMatrix();	

	KuPose RobotPos;
	RobotPos = KuDrawingInfo::getInstance()->getRobotPos();

	glTranslated(RobotPos.getXm(), RobotPos.getYm(), 0);
	glRotatef(RobotPos.getThetaDeg(), 0.0, 0.0, 1.0);

	//------------------------ pioneer 3AT 비슷하게.... --------------------------------------//
	// 변수 지정해서 하려 했는데 이상하게 꼬였음.... ㅠㅠ
	double h1 = 0.07;
	double h2 = 0.25;
	double h3 = 0.27;
	double dx1 = 0.2;
	double dy1 = 0.12;
	double dx11 = 0.24;
	double dx2 = 0.23;
	double dy2 = 0.15;
	double dx22 = 0.27;
	double r1 = 0.1;
	double r2 = 0.05;
	double wx = 0.13;
	double wy = 0.17;

	// ----------------------- 옆면 4개
	glColor3ub(200, 0, 0);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx1, -dy1, h2);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, h2);
	glVertex3f(dx1, -dy1, h1);

	glVertex3f(dx11, 0, h2);
	glVertex3f(dx11, 0, h1);

	glVertex3f(dx1, dy1, h2);
	glVertex3f(dx1, dy1, h1);

	glVertex3f(-dx1, dy1, h2);
	glVertex3f(-dx1, dy1, h1);

	glVertex3f(-dx11, 0, h2);
	glVertex3f(-dx11, 0, h1);

	glVertex3f(-dx1, -dy1, h2);
	glVertex3f(-dx1, -dy1, h1);
	glEnd();

	// 모서리에 라인을 그려서 강조
	glColor3ub(0, 0, 0);
	glBegin(GL_LINES);
	glVertex3f(-dx1, -dy1, h2);
	glVertex3f(-dx1, -dy1, h1);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(dx1, -dy1, h2);
	glVertex3f(dx1, -dy1, h1);
	glEnd();
	glBegin(GL_LINES);	
	glVertex3f(dx11, 0, h2);
	glVertex3f(dx11, 0, h1);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(dx1, dy1, h2);
	glVertex3f(dx1, dy1, h1);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(-dx1, dy1, h2);
	glVertex3f(-dx1, dy1, h1);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(-dx11, 0, h2);
	glVertex3f(-dx11, 0, h1);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(-dx1, -dy1, h2);
	glVertex3f(-dx1, -dy1, h1);
	glEnd();

	//------------------------------------- 윗면
	glColor3ub(50, 50, 50);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(-dx2, -dy2, h2);
	glVertex3f(dx2, -dy2, h3);
	glVertex3f(dx2, -dy2, h2);

	glVertex3f(dx22, 0, h3);
	glVertex3f(dx22, 0, h2);

	glVertex3f(dx2, dy2, h3);
	glVertex3f(dx2, dy2, h2);

	glVertex3f(-dx2, dy2, h3);
	glVertex3f(-dx2, dy2, h2);

	glVertex3f(-dx22, 0, h3);
	glVertex3f(-dx22, 0, h2);

	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(-dx2, -dy2, h2);
	glEnd();
	glBegin(GL_POLYGON);
	glVertex3f(dx22, 0, h3);
	glVertex3f(dx2, dy2, h3);
	glVertex3f(-dx2, dy2, h3);
	glVertex3f(-dx22, 0, h3);
	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(dx2, -dy2, h3);
	glEnd();
	// 기둥 4개
	glPushMatrix();
	glTranslated(0.15, 0.1, h3+0.1);
	glRotatef(-180, 1.0, 0.0, 0.0);
	glutSolidCone(0.02, 0.2, 4, 4);
	glPopMatrix();
	glPushMatrix();
	glTranslated(-0.15, 0.1, h3+0.1);
	glRotatef(-180, 1.0, 0.0, 0.0);
	glutSolidCone(0.02, 0.2, 4, 4);
	glPopMatrix();
	glPushMatrix();
	glTranslated(-0.15, -0.1, h3+0.1);
	glRotatef(-180, 1.0, 0.0, 0.0);
	glutSolidCone(0.02, 0.2, 4, 4);
	glPopMatrix();
	glPushMatrix();
	glTranslated(0.15, -0.1, h3+0.1);
	glRotatef(-180, 1.0, 0.0, 0.0);
	glutSolidCone(0.02, 0.2, 4, 4);
	glPopMatrix();

	//------------------------------------- 윗면2
	glPushMatrix();
	glTranslated(0,0,0.1);
	glColor3ub(50, 50, 50);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(-dx2, -dy2, h2);
	glVertex3f(dx2, -dy2, h3);
	glVertex3f(dx2, -dy2, h2);

	glVertex3f(dx22, 0, h3);
	glVertex3f(dx22, 0, h2);

	glVertex3f(dx2, dy2, h3);
	glVertex3f(dx2, dy2, h2);

	glVertex3f(-dx2, dy2, h3);
	glVertex3f(-dx2, dy2, h2);

	glVertex3f(-dx22, 0, h3);
	glVertex3f(-dx22, 0, h2);

	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(-dx2, -dy2, h2);
	glEnd();
	glBegin(GL_POLYGON);
	glVertex3f(dx22, 0, h3);
	glVertex3f(dx2, dy2, h3);
	glVertex3f(-dx2, dy2, h3);
	glVertex3f(-dx22, 0, h3);
	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(dx2, -dy2, h3);
	glEnd();
	glPopMatrix();

	//---------------------------------------------------- 바퀴
	glPushMatrix();
	glColor3ub(50, 50, 50);	
	glTranslated(wx,wy,r1);
	glRotatef(90, 1.0, 0.0, 0.0);
	glutSolidTorus(0.05, r1-0.05, 8, 10);
	glTranslated(-2.0*wx,0,0);
	glutSolidTorus(0.05, r1-0.05, 8, 10);
	glTranslated(0,0,2.0*wy);
	glutSolidTorus(0.05, r1-0.05, 8, 10);
	glTranslated(2.0*wx,0,0);
	glutSolidTorus(0.05, r1-0.05, 8, 10);	
	glPopMatrix();

	// -------------------------------------------- 뒷 박스
	glPushMatrix();
	h3+=0.1;
	glTranslated(-0.15,0, h3);
	dx1 = 0.1;dx2 = dx1;
	dy1 = 0.12; dy2 = dy1;
	h1 = 0.1;
	//옆면 4개
	glColor3ub(100, 100, 100);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glVertex3f(dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, 0);

	glVertex3f(dx1, dy1, h1);
	glVertex3f(dx1, dy1, 0);

	glVertex3f(-dx1, dy1, h1);
	glVertex3f(-dx1, dy1, 0);

	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glEnd();
	//윗면
	glColor3ub(100, 100, 100);
	glBegin(GL_POLYGON);
	glVertex3f(dx2, dy2, h1);
	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(dx2, -dy2, h1);
	glEnd();
	glPopMatrix();

	// -------------------------------------------- 레이저스캐너 기둥
	glPushMatrix();
	glTranslated(0.1,0.1,h3);
	dx1 = 0.03;
	dy1 = 0.01;
	h1 = 0.2;
	//옆면 4개
	glColor3ub(0, 220, 0);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glVertex3f(dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, 0);

	glVertex3f(dx1, dy1, h1);
	glVertex3f(dx1, dy1, 0);

	glVertex3f(-dx1, dy1, h1);
	glVertex3f(-dx1, dy1, 0);

	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glEnd();
	//윗면
	glBegin(GL_POLYGON);
	glVertex3f(dx1, dy1, h1);
	glVertex3f(-dx1, dy1, h1);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, h1);
	glEnd();
	glTranslated(0,-0.2,0);
	//옆면 4개
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glVertex3f(dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, 0);

	glVertex3f(dx1, dy1, h1);
	glVertex3f(dx1, dy1, 0);

	glVertex3f(-dx1, dy1, h1);
	glVertex3f(-dx1, dy1, 0);

	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glEnd();
	//윗면
	glBegin(GL_POLYGON);
	glVertex3f(dx1, dy1, h1);
	glVertex3f(-dx1, dy1, h1);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, h1);
	glEnd();
	glPopMatrix();

	// -------------------------------------------- 레이저스캐너
	glPushMatrix();
	glTranslated(0.1,0,0.55);
	glRotatef(0, 0.0, 1.0, 0.0);
	glTranslated(-0.1,0,-0.55);
	// 맨 아래
	h3+=0.1;
	glTranslated(0.15,0,h3);
	dx1 = 0.075;dx2 = dx1;
	dx11 = 0.095; dx22 = dx11;
	dy1 = 0.075; dy2 = dy1;
	h1 = 0.02;
	//옆면 4개
	glColor3ub(0, 0, 200);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glVertex3f(dx2, -dy2, h1);
	glVertex3f(dx1, -dy2, 0);

	glVertex3f(dx22, 0, h1);
	glVertex3f(dx11, 0, 0);

	glVertex3f(dx2, dy2, h1);
	glVertex3f(dx1, dy2, 0);

	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, dy2, 0);

	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glEnd();
	//윗면
	glColor3ub(0, 0, 200);
	glBegin(GL_POLYGON);
	glVertex3f(dx2, dy2, h1);
	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(dx2, -dy2, h1);
	glVertex3f(dx22, 0, h1);
	glEnd();

	// 중간
	glTranslated(0,0,h1);
	dx1 = 0.075;dx2 = dx1;
	dy1 = 0.075; dy2 = dy1;
	h1 = 0.1;
	//옆면 4개
	glColor3ub(0, 0, 200);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glVertex3f(0, -dy2, h1);
	glVertex3f(0, -dy2, 0);

	glVertex3f(0, dy2, h1);
	glVertex3f(0, dy2, 0);

	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, dy2, 0);

	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glEnd();
	dx1 = 0.045;dx2 = 0.055;
	dx11 = 0.060; dx22 = 0.075;
	dy1 = 0.055; dy2 = dy1;
	h1 = 0.1;
	//옆면 4개
	glColor3ub(50, 50, 50);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(0, -dy2, h1);
	glVertex3f(0, -dy2, 0);
	glVertex3f(dx2, -dy2, h1);
	glVertex3f(dx1, -dy2, 0);

	glVertex3f(dx22, 0, h1);
	glVertex3f(dx11, 0, 0);

	glVertex3f(dx2, dy2, h1);
	glVertex3f(dx1, dy2, 0);

	glVertex3f(0, dy2, h1);
	glVertex3f(0, dy2, 0);

	glVertex3f(0, -dy2, h1);
	glVertex3f(0, -dy2, 0);
	glEnd();


	// 윗 부분
	glTranslated(0,0,h1);
	dx1 = 0.075;dx2 = dx1;
	dx11 = 0.095; dx22 = dx11;
	dy1 = 0.075; dy2 = dy1;
	h1 = 0.06;
	//옆면 4개
	glColor3ub(0, 0, 200);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glVertex3f(dx2, -dy2, h1);
	glVertex3f(dx1, -dy2, 0);

	glVertex3f(dx22, 0, h1);
	glVertex3f(dx11, 0, 0);

	glVertex3f(dx2, dy2, h1);
	glVertex3f(dx1, dy2, 0);

	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, dy2, 0);

	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glEnd();
	//윗면
	glColor3ub(0, 0, 200);
	glBegin(GL_POLYGON);
	glVertex3f(dx2, dy2, h1);
	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(dx2, -dy2, h1);
	glVertex3f(dx22, 0, h1);
	glEnd();

	glPopMatrix();
	//-------------------------------------------------------------------------------------------//
	glPopMatrix();

}


void CKUNSUI3DDlg::renderBuildingMap()
{
	double** dMap = KuDrawingInfo::getInstance()->getBuildingMap(&m_nMapSizeX, &m_nMapSizeY);
	if(NULL==dMap ) return;

	KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
	double dHeightofLaserSensor =KuRobotParameter::getInstance()->getURG04LXLaserHeight()*MM2M;
	double dHeightofGround = 30*MM2M;

	GLfloat x,y,z;
	float fColor,fColor1;	
	float fCellSize = 0.1;//CELLSIZE;
	
	double dUnknownPro = 0.5;
	double dCriterionOfHighPro = 0.9;
	double dCriterionOfLowPro = 0.1;

	GLfloat step = 0.5 * fCellSize;
	
	int nCenterX = (int)( RobotPos.getXm()/(double)fCellSize );
	int nCenterY = (int)( RobotPos.getYm()/(double)fCellSize );

	glPushMatrix();

	for (int i=nCenterX-m_nDrawingSizeHalf; i<nCenterX+m_nDrawingSizeHalf; i++) {
		for (int j=nCenterY-m_nDrawingSizeHalf; j<nCenterY+m_nDrawingSizeHalf; j++) {

			if (i<0 || i>=m_nMapSizeX || j<0 || j>=m_nMapSizeY) continue;

			x = (float)(i) * fCellSize;
			y = (float)(j) * fCellSize;


			if (dMap[i][j]<=dCriterionOfLowPro) { 
				z = dHeightofGround;
				fColor = 255; fColor1 = 255;
			}
			else if (dMap[i][j]<dUnknownPro) {
				//continue;
				z = dHeightofGround;
				fColor = 255;
				fColor1 = (float)255.0*(1+(dMap[i][j]-dCriterionOfLowPro)/(dCriterionOfLowPro-0.5));
			}
			else if (dMap[i][j]==dUnknownPro) {
				continue;
			}
			else if (dMap[i][j]>=dCriterionOfHighPro) {
				z = dHeightofLaserSensor;
				fColor = 0.0; 
				fColor1 = 255;
			}
			else if (dMap[i][j]>dUnknownPro) {
				z =dHeightofLaserSensor;
				fColor = 0.0;
				fColor1 = (float)255.0*((dMap[i][j]-0.5)/(dCriterionOfHighPro-0.5)); 
			}			
			else{
				continue;				
			}

			glColor4ub(fColor,fColor,fColor,fColor1);

			// 다각형을 그리는 부분
			// 윗면
			glBegin(GL_POLYGON);
			glNormal3f(0.0f,0.0f,1.0f);
			glVertex3f(x+step,y-step,z);
			glVertex3f(x+step,y+step,z);
			glVertex3f(x-step,y+step,z);
			glVertex3f(x-step,y-step,z);
			glEnd();
			// 옆면들
/*
			glBegin(GL_POLYGON);
			glNormal3f(0.0f,-1.0f,0.0f);
			glVertex3f(x+step,y-step,z);
			glVertex3f(x-step,y-step,z);
			glVertex3f(x-step,y-step,0);
			glVertex3f(x+step,y-step,0);												
			glEnd();
			glBegin(GL_POLYGON);
			glNormal3f(1.0f,0.0f,0.0f);
			glVertex3f(x+step,y+step,z);
			glVertex3f(x+step,y-step,z);
			glVertex3f(x+step,y-step,0);
			glVertex3f(x+step,y+step,0);												
			glEnd();
			glBegin(GL_POLYGON);
			glNormal3f(0.0f,1.0f,0.0f);
			glVertex3f(x-step,y+step,z);
			glVertex3f(x+step,y+step,z);
			glVertex3f(x+step,y+step,0);
			glVertex3f(x-step,y+step,0);												
			glEnd();
			glBegin(GL_POLYGON);
			glNormal3f(-1.0f,0.0f,0.0f);
			glVertex3f(x-step,y-step,z);
			glVertex3f(x-step,y+step,z);
			glVertex3f(x-step,y+step,0);
			glVertex3f(x-step,y-step,0);												
			glEnd();
*/
			//}
		}	
	}

	glPopMatrix();
}


void CKUNSUI3DDlg::renderBuiltMap()
{	
	GLfloat x,y,z;
	GLfloat step;
	int nCenterX, nCenterY;
	float fCellSize = 0.1;//CELLSIZE;
	int nMapInitValue = 0;
	int nMapOccupiedValue = 1;
	int nMapUnknownValue = 2;
	int i, j, k, m;

	step = 0.5 * fCellSize;


	KuPose RobotPos;
	RobotPos = KuDrawingInfo::getInstance()->getRobotPos();

	int**  nMap = KuDrawingInfo::getInstance()->getMap();
	if(NULL==nMap) return;

	m_nMapSizeX= KuDrawingInfo::getInstance()->getMapSizeX();
	m_nMapSizeY =KuDrawingInfo::getInstance()->getMapSizeY();

	bool* pbMapRendered = new bool [m_nMapSizeX * m_nMapSizeY]; // for checking rendered cells
	int nRegionWidth(0), nRegionHeight(0);
	int nEndX(0), nEndY(0);
	int nCurrCellType(0);
	int nExp;
	int nRegion(0);
	int nXmin, nXmax, nYmin, nYmax;
	bool bExpandHorizontal, bExpandVertical;
	const float fLowVal(-0.5 * fCellSize);
	const float fHighVal(0.5 * fCellSize);
	float fCellPosX, fCellPosY;

	// init
	for(i = 0; i < m_nMapSizeX * m_nMapSizeY; i++)
	{
		pbMapRendered[i] = false; // not rendered
	}

	nCenterX = (int)( RobotPos.getXm()/(double)fCellSize );
	nCenterY = (int)( RobotPos.getYm()/(double)fCellSize );

	//glDisable(GL_CULL_FACE);
	glPushMatrix();

	nXmin = nCenterX - m_nDrawingSizeHalf;
	nXmax = nCenterX + m_nDrawingSizeHalf;
	nYmin = nCenterY - m_nDrawingSizeHalf;
	nYmax = nCenterY + m_nDrawingSizeHalf;

	if(nXmin < 0) nXmin = 0;
	if(nXmax > m_nMapSizeX) nXmax = m_nMapSizeX;
	if(nYmin < 0) nYmin = 0;
	if(nYmax > m_nMapSizeY) nYmax = m_nMapSizeY;

	// render
	for(i = nXmin; i < nXmax; i++)
	{
		for(j = nYmin; j < nYmax; j++)
		{
			if(i>m_nMapSizeX-1||j>m_nMapSizeY-1||i<1||j<1) continue;
			if(!pbMapRendered[j * m_nMapSizeX + i] && // if this cell is not rendered yet
				(nMap[i][j] == nMapInitValue || nMap[i][j] == nMapOccupiedValue)) // for empty or occupied cells
			{
				// init
				nRegionWidth = 1;
				nRegionHeight = 1;
				nCurrCellType = nMap[i][j];
				bExpandHorizontal = true;
				bExpandVertical = true;
				nExp = 1;

				while(bExpandHorizontal || bExpandVertical)
				{
					// expand: horizontal
					if(bExpandHorizontal)
					{
						if(i + nExp < nCenterX + m_nDrawingSizeHalf)
						{
							for(k = 0; k < nRegionHeight; k++)
							{
								if(i + nExp>m_nMapSizeX-1||j + k>m_nMapSizeY-1||i + nExp<1||j + k<1)
								{
									bExpandHorizontal = false; // different cell type is detected
									break;
								}

								if(nMap[i + nExp][j + k] != nCurrCellType || pbMapRendered[(j + k) * m_nMapSizeX + (i + nExp)])
								{
									bExpandHorizontal = false; // different cell type is detected

									break;
								}
							}

							if(bExpandHorizontal)
							{
								for(k = 0; k < nRegionHeight; k++)
								{
									if((j + k) * m_nMapSizeX + (i + nExp)>m_nMapSizeX * m_nMapSizeY-1||(j + k) * m_nMapSizeX + (i + nExp)<1) continue;
									pbMapRendered[(j + k) * m_nMapSizeX + (i + nExp)] = true;
								}

								nRegionWidth++;
							}
						}
						else
						{
							bExpandHorizontal = false;
						}
					}

					// expand: vertical
					if(bExpandVertical)
					{
						if(j + nExp < nCenterY + m_nDrawingSizeHalf)
						{
							for(k = 0; k < nRegionWidth; k++)
							{
								if(i + k>m_nMapSizeX-1||j + nExp>m_nMapSizeY-1||i + k<1||j + nExp<1)
								{
									bExpandVertical = false; // different cell type is detected

									break;
								}
								if(nMap[i + k][j + nExp] != nCurrCellType || pbMapRendered[(j + nExp) * m_nMapSizeX + (i + k)])
								{
									bExpandVertical = false; // different cell type is detected

									break;
								}
							}

							if(bExpandVertical)
							{
								for(k = 0; k < nRegionWidth; k++)
								{
									if((j + nExp) * m_nMapSizeX + (i + k)>m_nMapSizeX * m_nMapSizeY-1||(j + nExp) * m_nMapSizeX + (i + k)<1) continue;

									pbMapRendered[(j + nExp) * m_nMapSizeX + (i + k)] = true;
								}

								nRegionHeight++;
							}
						}
						else
						{
							bExpandVertical = false;
						}
					}

					nExp++;
				}

				nRegion++;

				// draw					
				fCellPosX = (float)i * fCellSize;
				fCellPosY = (float)j * fCellSize;

				if(m_xRotate == 90.)
				{
					z = 0.01;
				}
				else
				{
					z = 0.1;
				}

				glPushMatrix();

				if(nCurrCellType == nMapOccupiedValue) // occupied
				{
					glColor4ub(0, 0, 0, 255);

					// 윗면
					glBegin(GL_POLYGON);
					glNormal3f(0.0f,0.0f,1.0f);
					glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
					glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, z);
					glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, z);
					glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
					glEnd();

					if(m_xRotate != 90.)
					{
						// 옆면들
						glBegin(GL_POLYGON);
						glNormal3f(-1.0f,0.0f,0.0f);
						glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, 0);
						glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, 0);
						glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, z);
						glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
						glEnd();
						glBegin(GL_POLYGON);
						glNormal3f(0.0f,-1.0f,0.0f);
						glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, 0);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, 0);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, z);
						glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, z);
						glEnd();
						glBegin(GL_POLYGON);
						glNormal3f(1.0f,0.0f,0.0f);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, 0);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, 0);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, z);
						glEnd();
						glBegin(GL_POLYGON);
						glNormal3f(0.0f,1.0f,0.0f);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, 0);
						glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, 0);
						glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
						glEnd();
					}
				}
				else if(nCurrCellType == nMapInitValue) // empty
				{
					glColor4ub(255, 255, 255, 255);

					// 윗면
					glBegin(GL_POLYGON);
					glNormal3f(0.0f,0.0f,1.0f);
					glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, 0.01);
					glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, 0.01);
					glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, 0.01);
					glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, 0.01);
					glEnd();
				}

				glPopMatrix();
			}
		}
	}

	glPopMatrix();
	//glEnable(GL_CULL_FACE);

	delete [] pbMapRendered;
}



void CKUNSUI3DDlg::renderGoalPosition()
{

	KuPose GoalPos = KuDrawingInfo::getInstance()->getGoalPos();

	GLUquadricObj *pObj;
	pObj = gluNewQuadric();
	gluQuadricNormals(pObj, GLU_SMOOTH);

	glPushMatrix();

	glColor4ub(100,0,100,200);
	glTranslated(GoalPos.getXm(), GoalPos.getYm(), 0.17);
	gluDisk(pObj, 0., 0.3, 15, 10);
	glPopMatrix();

	gluDeleteQuadric(pObj);
}

void CKUNSUI3DDlg::renderLocalGoalPos()
{

	KuPose GoalPos = KuDrawingInfo::getInstance()->getLocalGoalPos();

	GLUquadricObj *pObj;
	pObj = gluNewQuadric();
	gluQuadricNormals(pObj, GLU_SMOOTH);

	glPushMatrix();

	glColor4ub(100,100,100,200);
	glTranslated(GoalPos.getXm(), GoalPos.getYm(), 0.15+0.05);
	gluDisk(pObj, 0., 0.2, 15, 10);
	glPopMatrix();

	gluDeleteQuadric(pObj);
}

void CKUNSUI3DDlg::renderTargetPos()
{

	KuPose GoalPos = KuDrawingInfo::getInstance()->getTargetPos();

	GLUquadricObj *pObj;
	pObj = gluNewQuadric();
	gluQuadricNormals(pObj, GLU_SMOOTH);

	glPushMatrix();

	glColor4ub(0,200,200,200);
	glTranslated(GoalPos.getXm(), GoalPos.getYm(), 0.15+0.05);
	gluDisk(pObj, 0., 0.15, 15, 10);
	glPopMatrix();

	gluDeleteQuadric(pObj);
}

void CKUNSUI3DDlg::renderSample()
{

	// MCL의 샘플데이터를 받아서 그린다
	GLfloat x,y,z;
	vector<Sample> vecParticle = KuDrawingInfo::getInstance()->getParticle();
	vector<Sample>::iterator vSample;	


	if(m_dZoom<=0){
		glPointSize((GLfloat)1.5*2.0);
	}
	else{
		glPointSize((GLfloat)m_dZoom*2.0);
	}


	glColor3f(1.0f, 0.0f, 0.0f);
	for (vSample = vecParticle.begin(); vSample != vecParticle.end(); vSample++) {
		x = vSample->x/1000.0;
		y = vSample->y/1000.0;
		z = 1.15;
		glBegin(GL_POINTS);
		glVertex3f(x,y,z);
		glEnd();
	}
}


void CKUNSUI3DDlg::renderGoalList()
{
	list<KuPose> GoalPosList = KuDrawingInfo::getInstance()->getGoalPosList();

	int nGoalNum = 0;
	for(list<KuPose>::iterator iter=GoalPosList.begin();iter !=GoalPosList.end();iter++ ){

		double x = iter->getXm();
		double y = iter->getYm();
		double 	z = 0.17;
		GLUquadricObj *pObj;
		pObj = gluNewQuadric();
		gluQuadricNormals(pObj, GLU_SMOOTH);

		glPushMatrix();
		glColor4ub(0,255,0,255);
		glTranslated(x, y, z);
		gluDisk(pObj, 0., 0.3, 15, 10);


		glColor3f (0, 0, 0);
		glRasterPos3f(0, 0, 0.1);
		char cGoalNum[20];
		memset(cGoalNum,0,sizeof(cGoalNum));
		//	sprintf(cGoalNum,"Goal_%d",nGoalNum);
		for (int i = 0; i < 20; i++) {
			glutBitmapCharacter(GLUT_BITMAP_8_BY_13, cGoalNum[i]);
			//GLUT_BITMAP_8_BY_13
			////GLUT_BITMAP_8_BY_15

		}
		nGoalNum++;
		glPopMatrix();

		gluDeleteQuadric(pObj);
	}
}



void CKUNSUI3DDlg::renderPath()
{

	list<KuPose> PathList = KuDrawingInfo::getInstance()->getPath();


	if(PathList.size()==0){
		return ;
	}

	GLfloat x,y,z;
	list<KuPose>::iterator it;



	float tempX, tempY;

	glPushMatrix();	
	glLineWidth(2.0);
	glColor3ub(0,0,255);

	//Path
	for (it = PathList.begin(); it != PathList.end(); it++) {

		if(it == PathList.begin()){
			tempX = it->getXm();
			tempY = it->getYm();
			continue;
		}	
		glBegin(GL_LINES);
		x = tempX; 
		y = tempY;
		z = 0.15+0.025;
		glVertex3f(x,y,z);
		x = it->getXm();
		y = it->getYm();
		z = 0.15+0.025;
		glVertex3f(x,y,z);
		glEnd();

		tempX = it->getXm();
		tempY = it->getYm();


	}
	glLineWidth(1.0);
	glPopMatrix();	


}



void CKUNSUI3DDlg::renderLocalPath()
{

	list<KuPose> PathList = KuDrawingInfo::getInstance()->getLocalPath();


	if(PathList.size()==0){
		return ;
	}

	GLfloat x,y,z;
	list<KuPose>::iterator it;



	float tempX, tempY;

	glPushMatrix();	
	glLineWidth(2.0);
	glColor3ub(255,0,0);

	//Path
	for (it = PathList.begin(); it != PathList.end(); it++) {

		if(it == PathList.begin()){
			tempX = it->getXm();
			tempY = it->getYm();
			continue;
		}	
		glBegin(GL_LINES);
		x = tempX; 
		y = tempY;
		z = 0.15+0.025;
		glVertex3f(x,y,z);
		x = it->getXm();
		y = it->getYm();
		z = 0.15+0.025;
		glVertex3f(x,y,z);
		glEnd();

		tempX = it->getXm();
		tempY = it->getYm();


	}
	glLineWidth(1.0);
	glPopMatrix();	


}

void CKUNSUI3DDlg::renderLaserData()
{

	int_1DArray nDataFront, nDataRear;
	double dOffsetFront =KuRobotParameter::getInstance()->getFrontLaserXOffset();
	double dOffsetRear =KuRobotParameter::getInstance()->getRearLaserXOffset();
	double dSensorHeight = (KuRobotParameter::getInstance()->getURG04LXLaserHeight()+20)*MM2M;

	nDataFront=KuDrawingInfo::getInstance()->getLaserDataFront181();
	nDataRear=KuDrawingInfo::getInstance()->getLaserDataRear181();
	KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();

	// Front
	glPushMatrix();	
	glPointSize(2);//(GLfloat)m_dZoom*2.0);
	glColor3f(0.0f, 1.0f, 0.0f);	
	glBegin(GL_POINTS);
	for(int i=0;i<181;i++){
		if(nDataFront[i]==-1||nDataFront[i]==0) continue;
		double dAngleRad = (double)(i - 90) * D2R;
		double dX = RobotPos.getX() + ((double)nDataFront[i] * cos(dAngleRad) + dOffsetFront ) * cos(RobotPos.getThetaRad()) + 
			((double)nDataFront[i] * sin(dAngleRad) ) * sin(-RobotPos.getThetaRad());

		double dY = RobotPos.getY() + ((double)nDataFront[i] * cos(dAngleRad) + dOffsetFront ) * sin(RobotPos.getThetaRad()) + 
			((double)nDataFront[i] * sin(dAngleRad) ) * cos(RobotPos.getThetaRad());

		dX = dX*MM2M;
		dY = dY*MM2M;
		glVertex3f(dX, dY, dSensorHeight);
	}
	glEnd();
	glPopMatrix();

	// Rear
	glPushMatrix();	
	glPointSize((GLfloat)m_dZoom*2.0);
	glColor3f(0.0f, 1.0f, 0.0f);	
	glBegin(GL_POINTS);
	for(int i=0;i<181;i++){
		if(nDataRear[i]==-1||nDataRear[i]==0) continue;
		double dAngleRad = (double)(i - 270) * D2R;
		double dX = RobotPos.getX() + ((double)nDataRear[i] * cos(dAngleRad) + dOffsetRear ) * cos(RobotPos.getThetaRad()) + 
			((double)nDataRear[i] * sin(dAngleRad) ) * sin(-RobotPos.getThetaRad());

		double dY = RobotPos.getY() + ((double)nDataRear[i] * cos(dAngleRad) + dOffsetRear ) * sin(RobotPos.getThetaRad()) + 
			((double)nDataRear[i] * sin(dAngleRad) ) * cos(RobotPos.getThetaRad());

		dX = dX*MM2M;
		dY = dY*MM2M;
		glVertex3f(dX, dY, dSensorHeight);
	}
	glEnd();
	glPopMatrix();

}
void CKUNSUI3DDlg::renderKinectRangeData()
{

	int_1DArray nData= KuDrawingInfo::getInstance()->getKinectRangeData();

	KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
	double dSensorHeight = (double)KuRobotParameter::getInstance()->getKinectHeight()/1000;
	double dOffset= (double)KuRobotParameter::getInstance()->getKinectXOffset();
	glPushMatrix();	
	glPointSize((GLfloat)m_dZoom*2.5);
	glColor3f(1.0f, 0.0f, 0.0f);	
	glBegin(GL_POINTS);
	for(int i=0;i<Sensor::KINECT_SENSOR_FOV;i++){

		if(nData[i] != 1000000){
			double dAngleRad = (double)(i -  Sensor::KINECT_SENSOR_FOV/2) * D2R;
			double dX = RobotPos.getX() + ((double)nData[i] * cos(dAngleRad) + dOffset ) * cos(RobotPos.getThetaRad()) + 
				((double)nData[i] * sin(dAngleRad) ) * sin(-RobotPos.getThetaRad());

			double dY = RobotPos.getY() + ((double)nData[i] * cos(dAngleRad) + dOffset ) * sin(RobotPos.getThetaRad()) + 
				((double)nData[i] * sin(dAngleRad) ) * cos(RobotPos.getThetaRad());

			glVertex3f(dX/1000., dY/1000.,dSensorHeight);
		}
	}
	glEnd();
	glPopMatrix();

}

void CKUNSUI3DDlg::renderCeilingDBImageData(void)		
{
	vector<KuPose> vecLandmarkPos = KuDrawingInfo::getInstance()->getvecLandmarkPos();

	if(vecLandmarkPos.size()==0) return; //등록된 인공표식이 없으면 함수를 빠져나온다.

	GLUquadricObj *pObj;
	pObj = gluNewQuadric();
	gluQuadricNormals(pObj, GLU_SMOOTH);

	int nLandmarkID = -1;
	double dLandmarkX=0.0;
	double dLandmarkY=0.0;
	double dLandmarkZ=0.0;
	double dLandmarkTheta=0.0;
	int nMaxSize=1000;
	vector<KuPose>::iterator it;
	int nIdX=0;
	
	AUX_RGBImageRec *texRec[1000];

	memset(texRec, 0, sizeof(void *)*1000);

	char cFilePathName[100];
	memset(cFilePathName,0,sizeof(cFilePathName));
	for(int i=0;i<vecLandmarkPos.size()-1;i++)
	{
		sprintf_s(cFilePathName,"C:/Samsung_TapelessAGV/Data/Path/Image/img_%d.bmp",i);
		texRec[i]=LoadBMPFile(cFilePathName);
	}
	GLuint tex[3];
	glGenTextures(3, &tex[0]);

	for(it=vecLandmarkPos.begin(); nIdX<vecLandmarkPos.size()-1; it++, nIdX++){
		if(nIdX>nMaxSize)continue;
		glPushMatrix();
		nLandmarkID =nIdX;// it->getID();
		dLandmarkX =  it->getXm();
		dLandmarkY = it->getYm();
		dLandmarkZ = 2.0;//it->getZm();
		dLandmarkTheta= it->getThetaDeg()-90;
		// Center point
		glColor4ub(255, 0, 0, 255);
		glTranslatef(dLandmarkX, dLandmarkY,dLandmarkZ);
		glRotatef(dLandmarkTheta, 0, 0, 1.);
		glBindTexture(GL_TEXTURE_2D, tex[0]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexImage2D(GL_TEXTURE_2D, 
			0, 
			3, 
			texRec[nIdX]->sizeX, 
			texRec[nIdX]->sizeY, 
			0, 
			GL_RGB, 
			GL_UNSIGNED_BYTE, 
			texRec[nIdX]->data);

		glEnable(GL_TEXTURE_2D);
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
		glBindTexture(GL_TEXTURE_2D, tex[0]);

		glColor3f(0.8f, 0.8f, 0.8f);

		glBindTexture(GL_TEXTURE_2D, tex[0]);
		glBegin(GL_QUADS);
		glNormal3f(0.0f, 0.0f, 1.0f);
		glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f, 1.0f, 1.0f);
		glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f, -1.0f, 1.0f);
		glTexCoord2f(1.0f, 1.0f); glVertex3f(1.0f, -1.0f, 1.0f);
		glTexCoord2f(1.0f, 0.0f); glVertex3f(1.0f, 1.0f, 1.0f);	
		glEnd();		


		// 		glColor4ub(0, 0, 255, 100); //색깔지정 
		// 
		// 		glDisable(GL_CULL_FACE); //원판이 아래서도 보이게 하는 명령어
		// 		gluDisk(pObj,0,0.8,40,40); //원판 그리는명령어
		// 		glEnable(GL_CULL_FACE);	
		// 

		glColor3f (0, 0, 0);
		glRasterPos3f(0, 0, 0.1);

		char cLandmarkID[20];
		memset(cLandmarkID,0,sizeof(cLandmarkID));
		sprintf(cLandmarkID,"ID:%d",nLandmarkID);
		for (int i = 0; i < 20; i++) {
			glutBitmapCharacter(GLUT_BITMAP_8_BY_13, cLandmarkID[i]);

		}
		glPopMatrix();
	}
	glDisable(GL_TEXTURE_2D);

	for(int i=0; i<nMaxSize; i++) {
		if(texRec[i])
		{
			if(texRec[i]->data) free(texRec[i]->data);
			free(texRec[i]);
		}
	}

	gluDeleteQuadric(pObj);
}
AUX_RGBImageRec* CKUNSUI3DDlg::LoadBMPFile(char *filename)
{
	FILE *hFile = NULL;

	if(!filename) return NULL;

	hFile = fopen(filename, "r");
	wchar_t pwstrDest[100];
	if(hFile) {
		fclose(hFile);
		int nLen = ( int )strlen( filename ) + 1;
		mbstowcs( pwstrDest, filename, nLen );
		return auxDIBImageLoad(pwstrDest);
	}

	return NULL;
}


void CKUNSUI3DDlg::renderWayPointList()
{

	list<KuPose> WayPointList = KuDrawingInfo::getInstance()->getWayPointList();

	if(WayPointList.size()==0){
		return ;
	}

	GLfloat x,y,z;
	list<KuPose>::iterator it;
	z = 0.15+0.05;

	//Path
	for (it = WayPointList.begin(); it != WayPointList.end(); it++) {

		GLUquadricObj *pObj;
		pObj = gluNewQuadric();
		gluQuadricNormals(pObj, GLU_SMOOTH);
		glPushMatrix();
		glColor4ub(0,100,200,200);
		glTranslated(it->getXm(), it->getYm(), z);
		gluDisk(pObj, 0., 0.15, 15, 10);
		glPopMatrix();

		gluDeleteQuadric(pObj);
	}


}


void CKUNSUI3DDlg::renderZoneMap()
{

	int**  nMap = KuDrawingInfo::getInstance()->getZoneMap();
	if(NULL==nMap) return;

	m_nMapSizeX= KuDrawingInfo::getInstance()->getMapSizeX();
	m_nMapSizeY =KuDrawingInfo::getInstance()->getMapSizeY();

	KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
	double dHeightofLaserSensor =KuRobotParameter::getInstance()->getURG04LXLaserHeight()*MM2M;
	double dHeightofGroud = 30*MM2M;

	GLfloat x,y,z;
	float fColor,fColor1;	
	float fCellSize = 0.1;//CELLSIZE;

	GLfloat step = 0.5 * fCellSize;


	int nCenterX = (int)( RobotPos.getXm()/(double)fCellSize );
	int nCenterY = (int)( RobotPos.getYm()/(double)fCellSize );

	glPushMatrix();

	for (int i=nCenterX-m_nDrawingSizeHalf; i<nCenterX+m_nDrawingSizeHalf; i++) {
		for (int j=nCenterY-m_nDrawingSizeHalf; j<nCenterY+m_nDrawingSizeHalf; j++) {

			if (i<0 || i>=m_nMapSizeX || j<0 || j>=m_nMapSizeY) continue;

			x = (float)(i) * fCellSize;
			y = (float)(j) * fCellSize;

			z = dHeightofGroud;

			if (nMap[i][j]==0) {
				continue;
			}	

			int nR = nMap[i][j]/100;
			int nG = nMap[i][j]/10;
			int nB = nMap[i][j]-nR*100-10*nG;

			glColor4ub(25*nR,25*nG,25*nB,255);
			
// 			if (nMap[i][j]==1) { glColor4ub(255,0,0,255);	}
// 			else if (nMap[i][j]==2) { glColor4ub(0,255,0,255);	}
// 			else if (nMap[i][j]==3) { glColor4ub(0,0,255,255);	}
// 			else if (nMap[i][j]==4) { glColor4ub(0,255,255,255); }
// 			else if (nMap[i][j]==5) { glColor4ub(255,0,255,255); }
// 			else if (nMap[i][j]==6) { glColor4ub(255,255,0,255); }
// 			else if (nMap[i][j]==7) { glColor4ub(100,0,0,255); }
// 			else if (nMap[i][j]==8) { glColor4ub(0,100,0,255); }
// 			else if (nMap[i][j]==9) { glColor4ub(0,0,100,255); }
// 			else if (nMap[i][j]==10) { glColor4ub(0,100,100,255); }
// 			else if (nMap[i][j]==11) { glColor4ub(100,0,100,255); }
// 			else if (nMap[i][j]==12) { glColor4ub(100,100,0,255); }
// 			else if (nMap[i][j]==13) { glColor4ub(100,100,100,255); }
// 			else if (nMap[i][j]==0) {
// 				continue;
// 			}	
// 			else{
// 				continue;				
// 			}


			// 다각형을 그리는 부분
			// 윗면
			glBegin(GL_POLYGON);
			glNormal3f(0.0f,0.0f,1.0f);
			glVertex3f(x+step,y-step,z);
			glVertex3f(x+step,y+step,z);
			glVertex3f(x-step,y+step,z);
			glVertex3f(x-step,y-step,z);
			glEnd();
			// 옆면들
			glBegin(GL_POLYGON);
			glNormal3f(0.0f,-1.0f,0.0f);
			glVertex3f(x+step,y-step,z);
			glVertex3f(x-step,y-step,z);
			glVertex3f(x-step,y-step,0);
			glVertex3f(x+step,y-step,0);												
			glEnd();
			glBegin(GL_POLYGON);
			glNormal3f(1.0f,0.0f,0.0f);
			glVertex3f(x+step,y+step,z);
			glVertex3f(x+step,y-step,z);
			glVertex3f(x+step,y-step,0);
			glVertex3f(x+step,y+step,0);												
			glEnd();
			glBegin(GL_POLYGON);
			glNormal3f(0.0f,1.0f,0.0f);
			glVertex3f(x-step,y+step,z);
			glVertex3f(x+step,y+step,z);
			glVertex3f(x+step,y+step,0);
			glVertex3f(x-step,y+step,0);												
			glEnd();
			glBegin(GL_POLYGON);
			glNormal3f(-1.0f,0.0f,0.0f);
			glVertex3f(x-step,y-step,z);
			glVertex3f(x-step,y+step,z);
			glVertex3f(x-step,y+step,0);
			glVertex3f(x-step,y-step,0);												
			glEnd();
			//}
		}	
	}

	glPopMatrix();
}

void CKUNSUI3DDlg::renderAlignLaserData()
{

	int_1DArray nData;
	double dOffset =KuRobotParameter::getInstance()->getFrontLaserXOffset();
	double dSensorHeight = 0.05+(KuRobotParameter::getInstance()->getURG04LXLaserHeight()+20)*MM2M;

	nData=KuDrawingInfo::getInstance()->getAlignLaserData181();
	KuPose RobotPos = KuDrawingInfo::getInstance()->getAuxiliaryRobotPos();


	glPushMatrix();	
	glPointSize((GLfloat)m_dZoom*2.0);
	glColor3f(1.0f, 0.0f, 0.0f);	
	glBegin(GL_POINTS);
	for(int i=0;i<181;i++){
		if(nData[i]==-1||nData[i]==0) continue;
		double dAngleRad = (double)(i - 90) * D2R;
		double dX = RobotPos.getX() + ((double)nData[i] * cos(dAngleRad) + dOffset ) * cos(RobotPos.getThetaRad()) + 
			((double)nData[i] * sin(dAngleRad) ) * sin(-RobotPos.getThetaRad());

		double dY = RobotPos.getY() + ((double)nData[i] * cos(dAngleRad) + dOffset ) * sin(RobotPos.getThetaRad()) + 
			((double)nData[i] * sin(dAngleRad) ) * cos(RobotPos.getThetaRad());

		dX = dX*MM2M;
		dY = dY*MM2M;
		glVertex3f(dX, dY, dSensorHeight);
	}
	glEnd();
	glPopMatrix();


}

void CKUNSUI3DDlg::renderRobot2()
{

	glPushMatrix();	

	KuPose RobotPos;
	RobotPos = KuDrawingInfo::getInstance()->getRobotPos2();

	glTranslated(RobotPos.getXm(), RobotPos.getYm(), 0);
	glRotatef(RobotPos.getThetaDeg(), 0.0, 0.0, 1.0);

	//------------------------ pioneer 3AT 비슷하게.... --------------------------------------//
	// 변수 지정해서 하려 했는데 이상하게 꼬였음.... ㅠㅠ
	double h1 = 0.07;
	double h2 = 0.25;
	double h3 = 0.27;
	double dx1 = 0.2;
	double dy1 = 0.12;
	double dx11 = 0.24;
	double dx2 = 0.23;
	double dy2 = 0.15;
	double dx22 = 0.27;
	double r1 = 0.1;
	double r2 = 0.05;
	double wx = 0.13;
	double wy = 0.17;

	// ----------------------- 옆면 4개
	glColor3ub(200, 0, 0);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx1, -dy1, h2);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, h2);
	glVertex3f(dx1, -dy1, h1);

	glVertex3f(dx11, 0, h2);
	glVertex3f(dx11, 0, h1);

	glVertex3f(dx1, dy1, h2);
	glVertex3f(dx1, dy1, h1);

	glVertex3f(-dx1, dy1, h2);
	glVertex3f(-dx1, dy1, h1);

	glVertex3f(-dx11, 0, h2);
	glVertex3f(-dx11, 0, h1);

	glVertex3f(-dx1, -dy1, h2);
	glVertex3f(-dx1, -dy1, h1);
	glEnd();

	// 모서리에 라인을 그려서 강조
	glColor3ub(0, 0, 0);
	glBegin(GL_LINES);
	glVertex3f(-dx1, -dy1, h2);
	glVertex3f(-dx1, -dy1, h1);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(dx1, -dy1, h2);
	glVertex3f(dx1, -dy1, h1);
	glEnd();
	glBegin(GL_LINES);	
	glVertex3f(dx11, 0, h2);
	glVertex3f(dx11, 0, h1);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(dx1, dy1, h2);
	glVertex3f(dx1, dy1, h1);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(-dx1, dy1, h2);
	glVertex3f(-dx1, dy1, h1);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(-dx11, 0, h2);
	glVertex3f(-dx11, 0, h1);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(-dx1, -dy1, h2);
	glVertex3f(-dx1, -dy1, h1);
	glEnd();

	//------------------------------------- 윗면
	glColor3ub(200, 0, 0);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(-dx2, -dy2, h2);
	glVertex3f(dx2, -dy2, h3);
	glVertex3f(dx2, -dy2, h2);

	glVertex3f(dx22, 0, h3);
	glVertex3f(dx22, 0, h2);

	glVertex3f(dx2, dy2, h3);
	glVertex3f(dx2, dy2, h2);

	glVertex3f(-dx2, dy2, h3);
	glVertex3f(-dx2, dy2, h2);

	glVertex3f(-dx22, 0, h3);
	glVertex3f(-dx22, 0, h2);

	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(-dx2, -dy2, h2);
	glEnd();
	glBegin(GL_POLYGON);
	glVertex3f(dx22, 0, h3);
	glVertex3f(dx2, dy2, h3);
	glVertex3f(-dx2, dy2, h3);
	glVertex3f(-dx22, 0, h3);
	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(dx2, -dy2, h3);
	glEnd();
	// 기둥 4개
	glPushMatrix();
	glTranslated(0.15, 0.1, h3+0.1);
	glRotatef(-180, 1.0, 0.0, 0.0);
	glutSolidCone(0.02, 0.2, 4, 4);
	glPopMatrix();
	glPushMatrix();
	glTranslated(-0.15, 0.1, h3+0.1);
	glRotatef(-180, 1.0, 0.0, 0.0);
	glutSolidCone(0.02, 0.2, 4, 4);
	glPopMatrix();
	glPushMatrix();
	glTranslated(-0.15, -0.1, h3+0.1);
	glRotatef(-180, 1.0, 0.0, 0.0);
	glutSolidCone(0.02, 0.2, 4, 4);
	glPopMatrix();
	glPushMatrix();
	glTranslated(0.15, -0.1, h3+0.1);
	glRotatef(-180, 1.0, 0.0, 0.0);
	glutSolidCone(0.02, 0.2, 4, 4);
	glPopMatrix();

	//------------------------------------- 윗면2
	glPushMatrix();
	glTranslated(0,0,0.1);
	glColor3ub(200, 0, 0);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(-dx2, -dy2, h2);
	glVertex3f(dx2, -dy2, h3);
	glVertex3f(dx2, -dy2, h2);

	glVertex3f(dx22, 0, h3);
	glVertex3f(dx22, 0, h2);

	glVertex3f(dx2, dy2, h3);
	glVertex3f(dx2, dy2, h2);

	glVertex3f(-dx2, dy2, h3);
	glVertex3f(-dx2, dy2, h2);

	glVertex3f(-dx22, 0, h3);
	glVertex3f(-dx22, 0, h2);

	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(-dx2, -dy2, h2);
	glEnd();
	glBegin(GL_POLYGON);
	glVertex3f(dx22, 0, h3);
	glVertex3f(dx2, dy2, h3);
	glVertex3f(-dx2, dy2, h3);
	glVertex3f(-dx22, 0, h3);
	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(dx2, -dy2, h3);
	glEnd();
	glPopMatrix();

	//---------------------------------------------------- 바퀴
	glPushMatrix();
	glColor3ub(200, 0, 0);
	glTranslated(wx,wy,r1);
	glRotatef(90, 1.0, 0.0, 0.0);
	glutSolidTorus(0.05, r1-0.05, 8, 10);
	glTranslated(-2.0*wx,0,0);
	glutSolidTorus(0.05, r1-0.05, 8, 10);
	glTranslated(0,0,2.0*wy);
	glutSolidTorus(0.05, r1-0.05, 8, 10);
	glTranslated(2.0*wx,0,0);
	glutSolidTorus(0.05, r1-0.05, 8, 10);	
	glPopMatrix();

	// -------------------------------------------- 뒷 박스
	glPushMatrix();
	h3+=0.1;
	glTranslated(-0.15,0, h3);
	dx1 = 0.1;dx2 = dx1;
	dy1 = 0.12; dy2 = dy1;
	h1 = 0.1;
	//옆면 4개
	glColor3ub(200, 0, 0);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glVertex3f(dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, 0);

	glVertex3f(dx1, dy1, h1);
	glVertex3f(dx1, dy1, 0);

	glVertex3f(-dx1, dy1, h1);
	glVertex3f(-dx1, dy1, 0);

	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glEnd();
	//윗면
	glColor3ub(200, 0, 0);
	glBegin(GL_POLYGON);
	glVertex3f(dx2, dy2, h1);
	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(dx2, -dy2, h1);
	glEnd();
	glPopMatrix();

	// -------------------------------------------- 레이저스캐너 기둥
	glPushMatrix();
	glTranslated(0.1,0.1,h3);
	dx1 = 0.03;
	dy1 = 0.01;
	h1 = 0.2;
	//옆면 4개
	glColor3ub(200, 0, 0);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glVertex3f(dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, 0);

	glVertex3f(dx1, dy1, h1);
	glVertex3f(dx1, dy1, 0);

	glVertex3f(-dx1, dy1, h1);
	glVertex3f(-dx1, dy1, 0);

	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glEnd();
	//윗면
	glBegin(GL_POLYGON);
	glVertex3f(dx1, dy1, h1);
	glVertex3f(-dx1, dy1, h1);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, h1);
	glEnd();
	glTranslated(0,-0.2,0);
	//옆면 4개
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glVertex3f(dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, 0);

	glVertex3f(dx1, dy1, h1);
	glVertex3f(dx1, dy1, 0);

	glVertex3f(-dx1, dy1, h1);
	glVertex3f(-dx1, dy1, 0);

	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glEnd();
	//윗면
	glBegin(GL_POLYGON);
	glVertex3f(dx1, dy1, h1);
	glVertex3f(-dx1, dy1, h1);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, h1);
	glEnd();
	glPopMatrix();

	// -------------------------------------------- 레이저스캐너
	glPushMatrix();
	glTranslated(0.1,0,0.55);
	glRotatef(0, 0.0, 1.0, 0.0);
	glTranslated(-0.1,0,-0.55);
	// 맨 아래
	h3+=0.1;
	glTranslated(0.15,0,h3);
	dx1 = 0.075;dx2 = dx1;
	dx11 = 0.095; dx22 = dx11;
	dy1 = 0.075; dy2 = dy1;
	h1 = 0.02;
	//옆면 4개
	glColor3ub(200, 0, 0);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glVertex3f(dx2, -dy2, h1);
	glVertex3f(dx1, -dy2, 0);

	glVertex3f(dx22, 0, h1);
	glVertex3f(dx11, 0, 0);

	glVertex3f(dx2, dy2, h1);
	glVertex3f(dx1, dy2, 0);

	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, dy2, 0);

	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glEnd();
	//윗면
	glColor3ub(200, 0, 0);
	glBegin(GL_POLYGON);
	glVertex3f(dx2, dy2, h1);
	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(dx2, -dy2, h1);
	glVertex3f(dx22, 0, h1);
	glEnd();

	// 중간
	glTranslated(0,0,h1);
	dx1 = 0.075;dx2 = dx1;
	dy1 = 0.075; dy2 = dy1;
	h1 = 0.1;
	//옆면 4개
	glColor3ub(200, 0, 0);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glVertex3f(0, -dy2, h1);
	glVertex3f(0, -dy2, 0);

	glVertex3f(0, dy2, h1);
	glVertex3f(0, dy2, 0);

	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, dy2, 0);

	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glEnd();
	dx1 = 0.045;dx2 = 0.055;
	dx11 = 0.060; dx22 = 0.075;
	dy1 = 0.055; dy2 = dy1;
	h1 = 0.1;
	//옆면 4개
	glColor3ub(200, 0, 0);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(0, -dy2, h1);
	glVertex3f(0, -dy2, 0);
	glVertex3f(dx2, -dy2, h1);
	glVertex3f(dx1, -dy2, 0);

	glVertex3f(dx22, 0, h1);
	glVertex3f(dx11, 0, 0);

	glVertex3f(dx2, dy2, h1);
	glVertex3f(dx1, dy2, 0);

	glVertex3f(0, dy2, h1);
	glVertex3f(0, dy2, 0);

	glVertex3f(0, -dy2, h1);
	glVertex3f(0, -dy2, 0);
	glEnd();


	// 윗 부분
	glTranslated(0,0,h1);
	dx1 = 0.075;dx2 = dx1;
	dx11 = 0.095; dx22 = dx11;
	dy1 = 0.075; dy2 = dy1;
	h1 = 0.06;
	//옆면 4개
	glColor3ub(200, 0, 0);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glVertex3f(dx2, -dy2, h1);
	glVertex3f(dx1, -dy2, 0);

	glVertex3f(dx22, 0, h1);
	glVertex3f(dx11, 0, 0);

	glVertex3f(dx2, dy2, h1);
	glVertex3f(dx1, dy2, 0);

	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, dy2, 0);

	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glEnd();
	//윗면
	glColor3ub(200, 0, 0);
	glBegin(GL_POLYGON);
	glVertex3f(dx2, dy2, h1);
	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(dx2, -dy2, h1);
	glVertex3f(dx22, 0, h1);
	glEnd();

	glPopMatrix();
	//-------------------------------------------------------------------------------------------//
	glPopMatrix();

}

void CKUNSUI3DDlg::renderTLaserData()
{

	int_1DArray nData;
	double dOffset =KuRobotParameter::getInstance()->getFrontLaserXOffset();
	double dSensorHeight = 0.05+(KuRobotParameter::getInstance()->getURG04LXLaserHeight()+20)*MM2M;

	nData=KuDrawingInfo::getInstance()->getTData();
	KuPose RobotPos = KuDrawingInfo::getInstance()->getAuxiliaryRobotPos();


	glPushMatrix();	
	glPointSize((GLfloat)m_dZoom*2.0);
	glColor3f(0.0f, 0.0f, 1.0f);	
	glBegin(GL_POINTS);
	for(int i=0;i<181;i++){
		if(nData[i]==-1||nData[i]==0) continue;
		double dAngleRad = (double)(i - 90) * D2R;
		double dX = RobotPos.getX() + ((double)nData[i] * cos(dAngleRad) + dOffset ) * cos(RobotPos.getThetaRad()) + 
			((double)nData[i] * sin(dAngleRad) ) * sin(-RobotPos.getThetaRad())+10;

		double dY = RobotPos.getY() + ((double)nData[i] * cos(dAngleRad) + dOffset ) * sin(RobotPos.getThetaRad()) + 
			((double)nData[i] * sin(dAngleRad) ) * cos(RobotPos.getThetaRad())+10;

		dX = dX*MM2M;
		dY = dY*MM2M;
		glVertex3f(dX, dY, dSensorHeight);
	}
	for(int i=181;i<181*2;i++){
		if(nData[i]==-1||nData[i]==0) continue;
		double dAngleRad = (double)(i - 181 - 90) * D2R;
		double dX = RobotPos.getX() + ((double)nData[i] * cos(dAngleRad) + dOffset ) * cos(RobotPos.getThetaRad()) + 
			((double)nData[i] * sin(dAngleRad) ) * sin(-RobotPos.getThetaRad());

		double dY = RobotPos.getY() + ((double)nData[i] * cos(dAngleRad) + dOffset ) * sin(RobotPos.getThetaRad()) + 
			((double)nData[i] * sin(dAngleRad) ) * cos(RobotPos.getThetaRad());

		dX = dX*MM2M;
		dY = dY*MM2M;
		glVertex3f(dX, dY, dSensorHeight);
	}
	glEnd();
	glPopMatrix();


}

void CKUNSUI3DDlg::renderRegion()
{
	vector<CLAMPData> FReagion;
	KuDrawingInfo::getInstance()->getRegion(&FReagion);
	double step=0.2;

	int aa = FReagion.size();

	for(int i=0; i<FReagion.size();i++)
	{
		if(FReagion[i].nmatchnum<10) continue;
		double x = FReagion[i].x;
		double y = FReagion[i].y;
		double 	z = FReagion[i].ceilingheight;

		double dDist=hypot((double)FReagion[i].width/2.0,(double)FReagion[i].height/2.0)/100.0;
		double dTheta=atan2((double)FReagion[i].width/2.0,(double)FReagion[i].height/2.0);

		glPushMatrix();
		glTranslatef(x, y, z);
		glColor4ub(255,0,100,200);
		GLUquadricObj *pObj;
		pObj = gluNewQuadric();

		// 다각형을 그리는 부분

		gluSphere(pObj, 0.27, 60, 30);
		glColor4ub(255, 0, 0, 250); //색깔지정 

		gluDeleteQuadric(pObj);

		// 윗면
// 		glBegin(GL_POLYGON);
// 		glNormal3f(0.0f,0.0f,1.0f);
// 		glVertex3f(+step,-step,step);
// 		glVertex3f(+step,+step,step);
// 		glVertex3f(-step,+step,step);
// 		glVertex3f(-step,-step,step);
// 		glEnd();
// 		// 옆면들
// 		glBegin(GL_POLYGON);
// 		glNormal3f(0.0f,-1.0f,0.0f);
// 		glVertex3f(+step,-step,step);
// 		glVertex3f(-step,-step,step);
// 		glVertex3f(-step,-step,-step);
// 		glVertex3f(+step,-step,-step);												
// 		glEnd();
// 		glBegin(GL_POLYGON);
// 		glNormal3f(1.0f,0.0f,0.0f);
// 		glVertex3f(+step,+step,step);
// 		glVertex3f(+step,-step,step);
// 		glVertex3f(+step,-step,-step);
// 		glVertex3f(+step,+step,-step);												
// 		glEnd();
// 		glBegin(GL_POLYGON);
// 		glNormal3f(0.0f,1.0f,0.0f);
// 		glVertex3f(-step,+step,step);
// 		glVertex3f(+step,+step,step);
// 		glVertex3f(+step,+step,-step);
// 		glVertex3f(-step,+step,-step);												
// 		glEnd();
// 		glBegin(GL_POLYGON);
// 		glNormal3f(-1.0f,0.0f,0.0f);
// 		glVertex3f(-step,-step,step);
// 		glVertex3f(-step,+step,step);
// 		glVertex3f(-step,+step,-step);
// 		glVertex3f(-step,-step,-step);												

		glEnd();
		glPopMatrix();

		if(FReagion[i].ntype==0)
		{
			glPushMatrix();
			glTranslatef(x, y, z);
			glRotatef(FReagion[i].th, 0, 0, 1.);
			glColor4ub(0, 0, 200, 100);
			Render_3DRect(1., 0.5, 0.1);
			glColor4ub(0, 0, 200, 200);
			Render_3DRectWire(1., 0.5, 0.1);
			glEnd();
			glPopMatrix();
		}
		else if(FReagion[i].ntype==1)
		{
			glPushMatrix();
			glTranslatef(x, y, z);
			glRotatef(0, 0, 0, 1.);
			glColor4ub(0, 0, 200, 100);
			Render_3DRect(1., 1.0, 0.1);
			glColor4ub(0, 0, 200, 200);
			Render_3DRectWire(1., 0.5, 0.1);
			glEnd();
			glPopMatrix();
		}

		if(FReagion[i].detect==true)
		{

			KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
			glPushMatrix();	
			glLineWidth(1.0);
			glColor3ub(0,0,255);


			glBegin(GL_LINES);
			glVertex3f(x,y,z);
			x = RobotPos.getXm();
			y = RobotPos.getYm();
			z = 0.15+0.025;
			glVertex3f(x,y,z);
			glEnd();

			glLineWidth(1.0);
			glPopMatrix();	
		}
	}


}


void CKUNSUI3DDlg::renderTemplateData()
{
vector<CFREAKData> FTamplet;
	KuDrawingInfo::getInstance()->getTemplateData(&FTamplet);

	double step=0.2;

	for(int i=0; i<FTamplet.size();i++)
	{
		double x = FTamplet[i].x;
		double y = FTamplet[i].y;
		double z = FTamplet[i].ceilingheight;
		int nRes=FTamplet[i].nmatchnum;

		if(nRes>250) nRes=255;

		
		glPushMatrix();
		glTranslatef(x, y, z);
		if(nRes>120) glColor4ub(nRes,0,0,200);
		else glColor4ub(0,nRes,0,200);
		GLUquadricObj *pObj;
		pObj = gluNewQuadric();

		// 다쩤각퉤?형u을≫ 그쐴?리Б?는쩇 부촇분Ж?

		gluSphere(pObj, 0.1, 60, 30);
		glColor4ub(255, 0, 0, 250); //색?깔푥지o정¤ 

		gluDeleteQuadric(pObj);

		glEnd();
		glPopMatrix();

		

		if(FTamplet[i].detect==true)
		{

			KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
			glPushMatrix();	
			glLineWidth(1.0);
			glColor3ub(0,0,255);


			glBegin(GL_LINES);
			glVertex3f(x,y,z);
			x = RobotPos.getXm();
			y = RobotPos.getYm();
			z = 0.15+0.025;
			glVertex3f(x,y,z);
			glEnd();

			glPopMatrix();	
		}

	}


}


/**
@brief Korean: Fiducial mark를 화면에 그려준다. 
@brief English: write in English
*/
void CKUNSUI3DDlg::renderFiducialmark()
{	
	list<KuPose> FiducialMarkList = KuDrawingInfo::getInstance()->getFiducialMarkList();

	if(FiducialMarkList.size()==0) return; //등록된 인공표식이 없으면 함수를 빠져나온다.

	GLUquadricObj *pObj;
	pObj = gluNewQuadric();
	gluQuadricNormals(pObj, GLU_SMOOTH);

	int nLandmarkID = -1;
	double dLandmarkX=0.0;
	double dLandmarkY=0.0;
	double dLandmarkZ=0.0;
	list<KuPose>::iterator it;

	for(it=FiducialMarkList.begin(); it!=FiducialMarkList.end(); it++){
		glPushMatrix();
		nLandmarkID = it->getID();
		dLandmarkX =  it->getXm();
		dLandmarkY = it->getYm();
		dLandmarkZ = it->getZm();
		// Center point
		glColor4ub(0, 0, 255, 150);
		glTranslatef(dLandmarkX, dLandmarkY,dLandmarkZ);

		gluSphere(pObj, 0.2, 40, 30);
		glColor4ub(255, 0, 0, 100); //색깔지정 

		glDisable(GL_CULL_FACE); //원판이 아래서도 보이게 하는 명령어
		gluDisk(pObj,0,0.3,40,40); //원판 그리는명령어
		glEnable(GL_CULL_FACE);	

		glColor3f (0, 0, 0);
		glRasterPos3f(0, 0, 0.1);

		char cLandmarkID[20];
		memset(cLandmarkID,0,sizeof(cLandmarkID));
		sprintf(cLandmarkID,"ID:%d",nLandmarkID);
		for (int i = 0; i < 20; i++) {
			glutBitmapCharacter(GLUT_BITMAP_8_BY_13, cLandmarkID[i]);
		}
		glPopMatrix();
	}

	gluDeleteQuadric(pObj);

}



void CKUNSUI3DDlg::renderPathBlock()
{
	KuPathBlockPr CPathBlock;
	PBlock PB;

	double x = m_dMPoint_x;
	double y = m_dMPoint_y;
	double z = m_dMapHeight+0.02;

	double dinitx;
	double dinity;
	double dinitz;

	int nPathBlockID = m_nPathBockID;
	
	if(m_nPathBockID==-1) return;

	PB=CPathBlock.getPathBlock(nPathBlockID);

	double sizex=PB.sizex/2.0;
	double sizey=PB.sizey/2.0;

	glPushMatrix();	
	glLineWidth(2.0);
	glColor3ub(0,0,0);

	glBegin(GL_LINES);
	dinitx = x-sizex;
	dinity=y-sizey;
	dinitz = m_dMapHeight+0.021;
	glVertex3f(dinitx,dinity,dinitz);
	dinitx = x-sizex;
	dinity=y+sizey;
	dinitz = m_dMapHeight+0.021;
	glVertex3f(dinitx,dinity,dinitz);
	glEnd();
	glPopMatrix();	

	glPushMatrix();	
	glBegin(GL_LINES);
	dinitx = x-sizex;
	dinity=y+sizey;
	dinitz = m_dMapHeight+0.021;
	glVertex3f(dinitx,dinity,dinitz);
	dinitx = x+sizex;
	dinity=y+sizey;
	dinitz = m_dMapHeight+0.021;
	glVertex3f(dinitx,dinity,dinitz);
	glEnd();
	glPopMatrix();	

	glPushMatrix();	
	glBegin(GL_LINES);
	dinitx = x+sizex;
	dinity=y+sizey;
	dinitz = m_dMapHeight+0.021;
	glVertex3f(dinitx,dinity,dinitz);
	dinitx = x+sizex;
	dinity=y-sizey;
	dinitz = m_dMapHeight+0.021;
	glVertex3f(dinitx,dinity,dinitz);
	glEnd();
	glPopMatrix();	

	glPushMatrix();	
	glBegin(GL_LINES);
	dinitx = x+sizex;
	dinity=y-sizey;
	dinitz = m_dMapHeight+0.021;
	glVertex3f(dinitx,dinity,dinitz);
	dinitx = x-sizex;
	dinity=y-sizey;
	dinitz = m_dMapHeight+0.021;
	glVertex3f(dinitx,dinity,dinitz);
	glEnd();
	glPopMatrix();	


	if(nPathBlockID<10)
	{
		glPushMatrix();	
		glLineWidth(2.0);
		glColor3ub(0,0,255);

		glBegin(GL_LINES);
		dinitx = x+PB.dist1.start_x;
		dinity=y+PB.dist1.start_y;
		dinitz = m_dMapHeight+0.021;
		glVertex3f(dinitx,dinity,dinitz);
		dinitx = x+PB.dist1.end_x;
		dinity=y+PB.dist1.end_y;
		dinitz = m_dMapHeight+0.021;
		glVertex3f(dinitx,dinity,dinitz);
		glEnd();
		glPopMatrix();	
	}
	else if(nPathBlockID<20)
	{
		glPushMatrix();	
		glLineWidth(2.0);
		glColor3ub(0,0,255);

		glBegin(GL_LINES);
		dinitx = x+PB.dist1.start_x;
		dinity=y+PB.dist1.start_y;
		dinitz = m_dMapHeight+0.021;
		glVertex3f(dinitx,dinity,dinitz);
		dinitx = x;
		dinity=y;
		dinitz = m_dMapHeight+0.021;
		glVertex3f(dinitx,dinity,dinitz);
		glEnd();
		glPopMatrix();	

		glPushMatrix();	
		glLineWidth(2.0);
		glColor3ub(0,0,255);

		glBegin(GL_LINES);
		dinitx = x;
		dinity=y;
		dinitz = m_dMapHeight+0.021;
		glVertex3f(dinitx,dinity,dinitz);
		dinitx = x+PB.dist1.end_x;
		dinity=y+PB.dist1.end_y;
		dinitz = m_dMapHeight+0.021;
		glVertex3f(dinitx,dinity,dinitz);
		glEnd();
		glPopMatrix();	
	}
	else if(nPathBlockID<40)
	{
		glPushMatrix();	
		glLineWidth(2.0);
		glColor3ub(0,0,255);

		glBegin(GL_LINES);
		dinitx = x+PB.dist1.start_x;
		dinity=y+PB.dist1.start_y;
		dinitz = m_dMapHeight+0.021;
		glVertex3f(dinitx,dinity,dinitz);
		dinitx=x+PB.dist1.end_x;
		dinity=y+PB.dist1.end_y;
		dinitz = m_dMapHeight+0.021;
		glVertex3f(dinitx,dinity,dinitz);
		glEnd();
		glPopMatrix();	

		glPushMatrix();	
		glLineWidth(2.0);
		glColor3ub(0,0,255);

		glBegin(GL_LINES);
		dinitx=x+PB.dist2.start_x;
		dinity=y+PB.dist2.start_y;
		dinitz = m_dMapHeight+0.021;
		glVertex3f(dinitx,dinity,dinitz);
		dinitx = x+PB.dist2.end_x;
		dinity=y+PB.dist2.end_y;
		dinitz = m_dMapHeight+0.021;
		glVertex3f(dinitx,dinity,dinitz);
		glEnd();
		glPopMatrix();	
	}
	else if(nPathBlockID<50)
	{
		glPushMatrix();	
		glLineWidth(2.0);
		glColor3ub(0,0,255);

		glBegin(GL_LINES);
		dinitx=x+PB.dist1.start_x;
		dinity=y+PB.dist1.start_y;
		dinitz = m_dMapHeight+0.021;
		glVertex3f(dinitx,dinity,dinitz);
		dinitx=x+PB.dist1.end_x;
		dinity=y+PB.dist1.end_y;
		dinitz = m_dMapHeight+0.021;
		glVertex3f(dinitx,dinity,dinitz);
		glEnd();
		glPopMatrix();	


		dinitx=x+PB.dist1.start_x;
		dinity=y+PB.dist1.start_y;
		dinitz = m_dMapHeight+0.021;

		GLUquadricObj *pObj;
		pObj = gluNewQuadric();
		gluQuadricNormals(pObj, GLU_SMOOTH);

		glPushMatrix();

		glColor4ub(255,0,0,255);
		glTranslated(dinitx, dinity, dinitz+0.01);
		gluDisk(pObj, 0., 0.1, 15, 10);
		glPopMatrix();

		gluDeleteQuadric(pObj);
	}

	
}

void CKUNSUI3DDlg::rendervecPathBlock()
{
	vector<PBlock> vecPathblock;
	KuDrawingInfo::getInstance()->getPathBlockPos(&vecPathblock);


	for(int i=0; i<vecPathblock.size();i++)
	{
		double step=0.5;
		double x = vecPathblock[i].x;
		double y = vecPathblock[i].y;

		double z = m_dMapHeight+0.02;

		double dinitx;
		double dinity;
		double dinitz;
	
		KuPathBlockPr CPathBlock;
		PBlock PB;
		PB=CPathBlock.getPathBlock(vecPathblock[i].pathdata);

 		double sizex=vecPathblock[i].sizex/2.0;
 		double sizey=vecPathblock[i].sizey/2.0;
		dinitz=m_dMapHeight+0.01;

		int nRColor,nGColor,nBColor;

		if(vecPathblock[i].velocity==0){nRColor=255;nGColor=255;nBColor=255;}
		else if(vecPathblock[i].velocity==1){nRColor=255;nGColor=0;nBColor=0;}
		else if(vecPathblock[i].velocity==2){nRColor=255;nGColor=255;nBColor=0;}
		else if(vecPathblock[i].velocity==3){nRColor=0;nGColor=255;nBColor=0;}
		else if(vecPathblock[i].velocity==4){nRColor=0;nGColor=0;nBColor=255;}
		else if(vecPathblock[i].velocity==5){nRColor=255;nGColor=0;nBColor=255;}
		else {nRColor=255;nGColor=255;nBColor=255;}

		
		/////////////////////
		glPushMatrix();
		glColor4ub(nRColor, nGColor, nBColor, 100); //색깔지정 
		glBegin(GL_POLYGON);
		glNormal3f(0.0f,0.0f,1.0f);
		glVertex3f(x+sizex,y-sizey,dinitz);
		glVertex3f(x+sizex,y+sizey,dinitz);
		glVertex3f(x-sizex,y+sizey,dinitz);
		glVertex3f(x-sizex,y-sizey,dinitz);
		glEnd();
		glPopMatrix();	

		glPushMatrix();	
		glLineWidth(2.0);
		glColor3ub(0,0,0);

		glBegin(GL_LINES);
		dinitx = x-sizex;
		dinity=y-sizey;
		dinitz = m_dMapHeight+0.021;
		glVertex3f(dinitx,dinity,dinitz);
		dinitx = x-sizex;
		dinity=y+sizey;
		dinitz = m_dMapHeight+0.021;
		glVertex3f(dinitx,dinity,dinitz);
		glEnd();
		glPopMatrix();	

		glPushMatrix();	
		glBegin(GL_LINES);
		dinitx = x-sizex;
		dinity=y+sizey;
		dinitz = m_dMapHeight+0.021;
		glVertex3f(dinitx,dinity,dinitz);
		dinitx = x+sizex;
		dinity=y+sizey;
		dinitz = m_dMapHeight+0.021;
		glVertex3f(dinitx,dinity,dinitz);
		glEnd();
		glPopMatrix();	

		glPushMatrix();	
		glBegin(GL_LINES);
		dinitx = x+sizex;
		dinity=y+sizey;
		dinitz = m_dMapHeight+0.021;
		glVertex3f(dinitx,dinity,dinitz);
		dinitx = x+sizex;
		dinity=y-sizey;
		dinitz = m_dMapHeight+0.021;
		glVertex3f(dinitx,dinity,dinitz);
		glEnd();
		glPopMatrix();	

		glPushMatrix();	
		glBegin(GL_LINES);
		dinitx = x+sizex;
		dinity=y-sizey;
		dinitz = m_dMapHeight+0.021;
		glVertex3f(dinitx,dinity,dinitz);
		dinitx = x-sizex;
		dinity=y-sizey;
		dinitz = m_dMapHeight+0.021;
		glVertex3f(dinitx,dinity,dinitz);
		glEnd();
		glPopMatrix();	

		if(vecPathblock[i].pathdata<10)
		{
			glPushMatrix();	
			glLineWidth(2.0);
			glColor3ub(0,0,255);

			glBegin(GL_LINES);
			dinitx = x+vecPathblock[i].dist1.start_x;
			dinity=y+vecPathblock[i].dist1.start_y;
			dinitz = m_dMapHeight+0.021;
			glVertex3f(dinitx,dinity,dinitz);
			dinitx = x+vecPathblock[i].dist1.end_x;
			dinity=y+vecPathblock[i].dist1.end_y;
			dinitz = m_dMapHeight+0.021;
			glVertex3f(dinitx,dinity,dinitz);
			glEnd();
			glPopMatrix();	
		}
		else if(vecPathblock[i].pathdata<20)
		{
			glPushMatrix();	
			glLineWidth(2.0);
			glColor3ub(0,0,255);

			glBegin(GL_LINES);
			dinitx = x+vecPathblock[i].dist1.start_x;
			dinity=y+vecPathblock[i].dist1.start_y;
			dinitz = m_dMapHeight+0.021;
			glVertex3f(dinitx,dinity,dinitz);
			dinitx = x;
			dinity=y;
			dinitz = m_dMapHeight+0.021;
			glVertex3f(dinitx,dinity,dinitz);
			glEnd();
			glPopMatrix();	

			glPushMatrix();	
			glLineWidth(2.0);
			glColor3ub(0,0,255);

			glBegin(GL_LINES);
			dinitx = x;
			dinity=y;
			dinitz = m_dMapHeight+0.021;
			glVertex3f(dinitx,dinity,dinitz);
			dinitx = x+vecPathblock[i].dist1.end_x;
			dinity=y+vecPathblock[i].dist1.end_y;
			dinitz = m_dMapHeight+0.021;
			glVertex3f(dinitx,dinity,dinitz);
			glEnd();
			glPopMatrix();	
		}
		else if(vecPathblock[i].pathdata<40)
		{
			glPushMatrix();	
			glLineWidth(2.0);
			glColor3ub(0,0,255);

			glBegin(GL_LINES);
			dinitx = x+vecPathblock[i].dist1.start_x;
			dinity=y+vecPathblock[i].dist1.start_y;
			dinitz = m_dMapHeight+0.021;
			glVertex3f(dinitx,dinity,dinitz);
			dinitx=x+vecPathblock[i].dist1.end_x;
			dinity=y+vecPathblock[i].dist1.end_y;
			dinitz = m_dMapHeight+0.021;
			glVertex3f(dinitx,dinity,dinitz);
			glEnd();
			glPopMatrix();	

			glPushMatrix();	
			glLineWidth(2.0);
			glColor3ub(0,0,255);

			glBegin(GL_LINES);
			dinitx=x+vecPathblock[i].dist2.start_x;
			dinity=y+vecPathblock[i].dist2.start_y;
			dinitz = m_dMapHeight+0.021;
			glVertex3f(dinitx,dinity,dinitz);
			dinitx = x+vecPathblock[i].dist2.end_x;
			dinity=y+vecPathblock[i].dist2.end_y;
			dinitz = m_dMapHeight+0.021;
			glVertex3f(dinitx,dinity,dinitz);
			glEnd();
			glPopMatrix();	
		}
		else if(vecPathblock[i].pathdata<50)
		{
			glPushMatrix();	
			glLineWidth(2.0);
			glColor3ub(0,0,255);

			glBegin(GL_LINES);
			dinitx=x+vecPathblock[i].dist1.start_x;
			dinity=y+vecPathblock[i].dist1.start_y;
			dinitz = m_dMapHeight+0.021;
			glVertex3f(dinitx,dinity,dinitz);
			dinitx=x+vecPathblock[i].dist1.end_x;
			dinity=y+vecPathblock[i].dist1.end_y;
			dinitz = m_dMapHeight+0.021;
			glVertex3f(dinitx,dinity,dinitz);
			glEnd();
			glPopMatrix();	


			dinitx=x+vecPathblock[i].dist1.start_x;
			dinity=y+vecPathblock[i].dist1.start_y;
			dinitz = m_dMapHeight+0.021;

			GLUquadricObj *pObj;
			pObj = gluNewQuadric();
			gluQuadricNormals(pObj, GLU_SMOOTH);

			glPushMatrix();

			glColor4ub(255,0,0,255);
			glTranslated(dinitx, dinity, dinitz+0.01);
			gluDisk(pObj, 0., 0.1, 15, 10);
			glPopMatrix();

			gluDeleteQuadric(pObj);
		}

		//////////////
		
		if(vecPathblock[i].waypoint==PathBlock::WAYPOINT||vecPathblock[i].nRouteOrder>0
			||vecPathblock[i].check==true||vecPathblock[i].detectObstacle==false)
		{
			dinitx=x;
			dinity=y;

			GLUquadricObj *pObj;
			pObj = gluNewQuadric();
			gluQuadricNormals(pObj, GLU_SMOOTH);
			
			if(vecPathblock[i].waypoint==PathBlock::WAYPOINT)//waypoint->blue circle
			{
				dinitz = m_dMapHeight+0.041;
				glPushMatrix();
				glColor4ub(0,0,255,255);
				glTranslated(dinitx, dinity, dinitz+0.01);
				gluDisk(pObj, 0., 0.07, 15, 10);
				glPopMatrix();
			}

			if(vecPathblock[i].nRouteOrder>0)//via->green circle
			{
				dinitz = m_dMapHeight+0.031;
				glPushMatrix();
				glColor4ub(0,255,0,255);
				glTranslated(dinitx, dinity, dinitz+0.01);
				gluDisk(pObj, 0., 0.085, 15, 10);
				glPopMatrix();
			}

			if(vecPathblock[i].check==true)//check->orange circle
			{
				dinitz = m_dMapHeight+0.036;
				glPushMatrix();
				glColor4ub(255,127,0,255);
				glTranslated(dinitx, dinity, dinitz+0.01);
				gluDisk(pObj, 0., 0.077, 15, 10);
				glPopMatrix();
			}
			if(vecPathblock[i].detectObstacle==false)//do not detect obstacle ->block circle
			{
				dinitz = m_dMapHeight+0.046;
				glPushMatrix();
				glColor4ub(0,0,0,255);
				glTranslated(dinitx, dinity, dinitz+0.01);
				gluDisk(pObj, 0., 0.063, 15, 10);
				glPopMatrix();
			}

			gluDeleteQuadric(pObj);
		}
		
	}

}


BOOL CKUNSUI3DDlg::OnSetCursor(CWnd* pWnd, UINT nHitTest, UINT message)
{

// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.
// if(m_bSetPathBlock)
// 	switch (m_nPathBockID) {
// 	case 0:
// 		::SetCursor(AfxGetApp()->LoadCursor(IDB_BITMAP1));
// 		break;
// 	case 1:
// 		::SetCursor(AfxGetApp()->LoadCursor(IDB_BITMAP2));
// 		break;
// 	case 2:
// 		::SetCursor(AfxGetApp()->LoadCursor(IDB_BITMAP3));
// 		break;
// 	case 3:
// 		::SetCursor(AfxGetApp()->LoadCursor(IDB_BITMAP4));
// 		break;
// 	case 4:
// 		::SetCursor(AfxGetApp()->LoadCursor(IDB_BITMAP5));
// 		break;
// 	case 5:
// 		::SetCursor(AfxGetApp()->LoadCursor(IDB_BITMAP6));
// 		break;
// 	case 6:
// 		::SetCursor(AfxGetApp()->LoadCursor(IDB_BITMAP7));
// 		break;
// 	default:
// 		CDialog::OnSetCursor(pWnd, nHitTest, message);
// 		break;
// 	}
// else 
	CDialog::OnSetCursor(pWnd, nHitTest, message);

	return TRUE;

	//return CDialog::OnSetCursor(pWnd, nHitTest, message);
}



void CKUNSUI3DDlg::rendervecPathlist()
{

	vector<list<KuPose>> veclistPath=KuDrawingInfo::getInstance()->getvecPathlist();

	for(int i=0; i<veclistPath.size();i++)
	{
		list<KuPose> PathList = veclistPath[i];

		if(PathList.size()==0){
			return ;
		}

		GLfloat x,y,z;
		list<KuPose>::iterator it;

		float tempX, tempY;

		glPushMatrix();	
		glLineWidth(2.0);
		glColor3ub(bcolor[i%9].val[0],bcolor[i%9].val[1],bcolor[i%9].val[2]);

		//Path
		for (it = PathList.begin(); it != PathList.end(); it++) {

			if(it == PathList.begin()){
				tempX = it->getXm();
				tempY = it->getYm();
				continue;
			}	
			glBegin(GL_LINES);
			x = tempX; 
			y = tempY;
			z = 0.15+0.025+i*0.03;
			glVertex3f(x,y,z);
			x = it->getXm();
			y = it->getYm();
			z = 0.15+0.025+i*0.03;
			glVertex3f(x,y,z);
			glEnd();

			tempX = it->getXm();
			tempY = it->getYm();


		}
		glLineWidth(1.0);
		glPopMatrix();	
	}
}

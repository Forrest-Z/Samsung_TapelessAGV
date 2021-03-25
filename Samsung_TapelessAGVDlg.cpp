
// Samsung_TapelessAGVDlg.cpp : 구현 파일
//

#include "stdafx.h"
#include "Samsung_TapelessAGV.h"
#include "Samsung_TapelessAGVDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// 응용 프로그램 정보에 사용되는 CAboutDlg 대화 상자입니다.

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

	// 대화 상자 데이터입니다.
	enum { IDD = IDD_ABOUTBOX };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.

	// 구현입니다.
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CSamsung_TapelessAGVDlg 대화 상자




CSamsung_TapelessAGVDlg::CSamsung_TapelessAGVDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CSamsung_TapelessAGVDlg::IDD, pParent)
	, m_pDC_sensorconnection(NULL)
	, m_bexecuteflag(false)	
	, m_bSetRobotPos(false)
	, m_nBehaviorName(-1)
	, m_bAvoidModeflag(false)
	, m_bsettingflag(false)
	, m_bchangpathblockflag(false)
	, m_SelID(-1)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CSamsung_TapelessAGVDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_CAM_INFO_STATIC, m_CamInfoPicCtrl);
	DDX_Control(pDX, IDC_CAM_INFO_STATIC2, m_CamInfoPicCtrl2);
	DDX_Control(pDX, IDC_XPOS_EDIT_CTRL, m_XPosEditCtrl);
	DDX_Control(pDX, IDC_YPOS_EDIT_CTRL, m_YPosEditCtrl);
	DDX_Control(pDX, IDC_TPOS_EDIT_CTRL, m_TPosEditCtrl);
	DDX_Control(pDX, IDC_CEILINGCAMERA_STATIC, m_CeilingImage);
	DDX_Control(pDX, IDC_KINECTCAMERA_STATIC, m_KinectImage);
	DDX_Control(pDX, IDC_OBSAVOID_CHECK, m_AvoidCheckbox);
	DDX_Control(pDX, IDC_SAVE_WAYPOINT_BUTTON, m_SaveWaypointButton);
	DDX_Control(pDX, IDC_LIST_INFO, m_cInfo);

}

BEGIN_MESSAGE_MAP(CSamsung_TapelessAGVDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_COMMAND(ID_GUIINFORMATION_SETROBOTPOSE, &CSamsung_TapelessAGVDlg::OnGuiinformationSetrobotpose)
	ON_COMMAND(ID_GUIINFORMATION_SETGOALPOSE, &CSamsung_TapelessAGVDlg::OnGuiinformationSetgoalpose)
	ON_COMMAND(ID_GUIINFORMATION_RENDERMAP, &CSamsung_TapelessAGVDlg::OnGuiinformationRendermap)
	ON_COMMAND(ID_GUIINFORMATION_RENDERLASER, &CSamsung_TapelessAGVDlg::OnGuiinformationRenderlaser)
	ON_COMMAND(ID_GUIINFORMATION_RENDERKINECT, &CSamsung_TapelessAGVDlg::OnGuiinformationRenderkinect)
	ON_COMMAND(ID_GUIINFORMATION_RENDERPATH, &CSamsung_TapelessAGVDlg::OnGuiinformationRenderpath)
	ON_COMMAND(ID_SENSORCONNECTION_HOKUYOUTM, &CSamsung_TapelessAGVDlg::OnSensorconnectionHokuyoutm)
	ON_COMMAND(ID_SENSORCONNECTION_KINECT, &CSamsung_TapelessAGVDlg::OnSensorconnectionKinect)
	ON_COMMAND(ID_SENSORCONNECTION_CEILINGCAMERA, &CSamsung_TapelessAGVDlg::OnSensorconnectionCeilingcamera)
	ON_COMMAND(ID_SENSORCONNECTION_E2BOXIMU, &CSamsung_TapelessAGVDlg::OnSensorconnectionE2boximu)
	ON_COMMAND(ID_SENSORCONNECTION_WHEELACTUATOR, &CSamsung_TapelessAGVDlg::OnSensorconnectionWheelactuator)
	ON_COMMAND(ID_BEHAVIOR_GOTOGOAL, &CSamsung_TapelessAGVDlg::OnBehaviorGotogoal)
	ON_COMMAND(ID_BEHAVIOR_MAPBUILDING, &CSamsung_TapelessAGVDlg::OnBehaviorMapbuilding)
	ON_WM_MOUSEWHEEL()
	ON_WM_TIMER()
	ON_COMMAND(ID_BEHAVIOR_TEACHINGPATH, &CSamsung_TapelessAGVDlg::OnBehaviorTeachingpath)
	ON_COMMAND(ID_GUIINFORMATION_RENDERCEILINGIMAGE, &CSamsung_TapelessAGVDlg::OnGuiinformationRenderceilingimage)
	ON_BN_CLICKED(IDC_SAVE_WAYPOINT_BUTTON, &CSamsung_TapelessAGVDlg::OnBnClickedSaveWaypointButton)
	ON_COMMAND(ID_BEHAVIOR_AUTONOMOUSGOTOGOAL, &CSamsung_TapelessAGVDlg::OnBehaviorAutonomousgotogoal)
	ON_BN_CLICKED(IDOK, &CSamsung_TapelessAGVDlg::OnBnClickedOk)
	ON_WM_DESTROY()
	ON_BN_CLICKED(IDC_OBSAVOID_CHECK, &CSamsung_TapelessAGVDlg::OnBnClickedObsavoidCheck)
	ON_COMMAND(ID_BEHAVIOR_GLOBALLOCALIZATION, &CSamsung_TapelessAGVDlg::OnBehaviorGloballocalization)
	ON_COMMAND(ID_GUIINFORMATION_RENDERZONEMAP, &CSamsung_TapelessAGVDlg::OnGuiinformationRenderzonemap)
	ON_COMMAND(ID_BEHAVIOR_MULTI, &CSamsung_TapelessAGVDlg::OnBehaviorMulti)
	ON_COMMAND(ID_COMMUNICATION_CONNECT, &CSamsung_TapelessAGVDlg::OnCommunicationConnect)
	//ON_NOTIFY(NM_THEMECHANGED, IDC_SCROLLBAR1, &CSamsung_TapelessAGVDlg::OnNMThemeChangedScrollbar1)
	ON_WM_VSCROLL()
	ON_WM_LBUTTONDOWN()
	ON_NOTIFY(LVN_ITEMCHANGED, IDC_LIST_INFO, &CSamsung_TapelessAGVDlg::OnLvnItemchangedListInfo)
	ON_COMMAND(ID_GUIINFORMATION_SETPATHBLOCK, &CSamsung_TapelessAGVDlg::OnGuiinformationSetpathblock)
	ON_COMMAND(ID_PATHBLOCK_GENERATEPATH, &CSamsung_TapelessAGVDlg::OnPathblockGeneratepath)
	ON_COMMAND(ID_PATHBLOCK_CLEARBLO, &CSamsung_TapelessAGVDlg::OnPathblockClearblo)
	ON_COMMAND(ID_VELOCITY_WHITE, &CSamsung_TapelessAGVDlg::OnVelocityWhite)
	ON_COMMAND(ID_MOTION_PAUSE, &CSamsung_TapelessAGVDlg::OnMotionPause)
	ON_WM_CONTEXTMENU()
	ON_COMMAND(ID_VELOCITY_RED, &CSamsung_TapelessAGVDlg::OnVelocityRed)
	ON_COMMAND(ID_VELOCITY_YELLOW, &CSamsung_TapelessAGVDlg::OnVelocityYellow)
	ON_COMMAND(ID_VELOCITY_GREEN, &CSamsung_TapelessAGVDlg::OnVelocityGreen)
	ON_COMMAND(ID_VELOCITY_BLUE, &CSamsung_TapelessAGVDlg::OnVelocityBlue)
	ON_COMMAND(ID_VELOCITY_PURPLE, &CSamsung_TapelessAGVDlg::OnVelocityPurple)
	ON_COMMAND(ID_MOTION_RESUME, &CSamsung_TapelessAGVDlg::OnMotionResume)
	ON_COMMAND(ID_ROTATE_LEFT90, &CSamsung_TapelessAGVDlg::OnRotateLeft90)
	ON_COMMAND(ID_ROTATE_RIGHT90, &CSamsung_TapelessAGVDlg::OnRotateRight90)
	ON_COMMAND(ID_ROTATE_LEFT180, &CSamsung_TapelessAGVDlg::OnRotateLeft180)
	ON_COMMAND(ID_ROTATE_RIGHT180, &CSamsung_TapelessAGVDlg::OnRotateRight180)
	ON_COMMAND(ID_DEVICE_DEVICE1, &CSamsung_TapelessAGVDlg::OnDeviceDevice1)
	ON_COMMAND(ID_DEVICE_DEVICE2, &CSamsung_TapelessAGVDlg::OnDeviceDevice2)
	ON_COMMAND(ID_DEVICE_DEVICE3, &CSamsung_TapelessAGVDlg::OnDeviceDevice3)
	ON_COMMAND(ID_BEHAVIOR_READYTOSIGNAL, &CSamsung_TapelessAGVDlg::OnBehaviorReadytosignal)
	ON_COMMAND(ID_BEHAVIOR_TOWERLAMP, &CSamsung_TapelessAGVDlg::OnBehaviorTowerlamp)
	ON_COMMAND(ID_SOUND_SONUD1, &CSamsung_TapelessAGVDlg::OnSoundSonud1)
	ON_COMMAND(ID_SOUND_SONUD2, &CSamsung_TapelessAGVDlg::OnSoundSonud2)
	ON_COMMAND(ID_SOUND_SONUD3, &CSamsung_TapelessAGVDlg::OnSoundSonud3)
	ON_COMMAND(ID_PATHBLOCK_SETTING, &CSamsung_TapelessAGVDlg::OnPathblockSetting)
	ON_COMMAND(ID_PATHBLOCK_SAVEPATHBLOCK, &CSamsung_TapelessAGVDlg::OnPathblockSavepathblock)
	ON_COMMAND(ID_POINT_WAYPOINT, &CSamsung_TapelessAGVDlg::OnPointWaypoint)
	ON_COMMAND(ID_POINT_STARTPOINT, &CSamsung_TapelessAGVDlg::OnPointStartpoint)
	ON_COMMAND(ID_POINT_ENDPOINT, &CSamsung_TapelessAGVDlg::OnPointEndpoint)
	ON_COMMAND(ID_POINT_DEFAULT, &CSamsung_TapelessAGVDlg::OnPointDefault)
	ON_COMMAND(ID_PATHBLOCK_LOADPATHBLOCK, &CSamsung_TapelessAGVDlg::OnPathblockLoadpathblock)
	ON_COMMAND(ID_SETTING_TASK, &CSamsung_TapelessAGVDlg::OnSettingTask)
	ON_COMMAND(ID_BEHAVIOR_GLOBALMAPBUILDING, &CSamsung_TapelessAGVDlg::OnBehaviorGlobalMapBuilding)
	ON_COMMAND(ID_VIA_1, &CSamsung_TapelessAGVDlg::OnVia1)
	ON_COMMAND(ID_VIA_2, &CSamsung_TapelessAGVDlg::OnVia2)
	ON_COMMAND(ID_VIA_3, &CSamsung_TapelessAGVDlg::OnVia3)
	ON_COMMAND(ID_VIA_4, &CSamsung_TapelessAGVDlg::OnVia4)
	ON_COMMAND(ID_VIA_5, &CSamsung_TapelessAGVDlg::OnVia5)
	ON_COMMAND(ID_VIA_6, &CSamsung_TapelessAGVDlg::OnVia6)
	ON_COMMAND(ID_VIA_7, &CSamsung_TapelessAGVDlg::OnVia7)
	ON_COMMAND(ID_VIA_8, &CSamsung_TapelessAGVDlg::OnVia8)
	ON_COMMAND(ID_VIA_9, &CSamsung_TapelessAGVDlg::OnVia9)
	ON_COMMAND(ID_VIA_10, &CSamsung_TapelessAGVDlg::OnVia10)
	ON_COMMAND(ID_VIA_DEFALT, &CSamsung_TapelessAGVDlg::OnViaDefalt)
	ON_COMMAND(ID_SAVEPATH_DEFAULT, &CSamsung_TapelessAGVDlg::OnSavepathDefault)
	ON_COMMAND(ID_SAVEPATH_PATH1, &CSamsung_TapelessAGVDlg::OnSavepathPath1)
	ON_COMMAND(ID_SAVEPATH_PATH2, &CSamsung_TapelessAGVDlg::OnSavepathPath2)
	ON_COMMAND(ID_SAVEPATH_PATH3, &CSamsung_TapelessAGVDlg::OnSavepathPath3)
	ON_COMMAND(ID_SAVEPATH_PATH4, &CSamsung_TapelessAGVDlg::OnSavepathPath4)
	ON_COMMAND(ID_SAVEPATH_PATH5, &CSamsung_TapelessAGVDlg::OnSavepathPath5)
	ON_COMMAND(ID_SAVEPATH_PATH6, &CSamsung_TapelessAGVDlg::OnSavepathPath6)
	ON_COMMAND(ID_SAVEPATH_PATH7, &CSamsung_TapelessAGVDlg::OnSavepathPath7)
	ON_COMMAND(ID_SAVEPATH_PATH8, &CSamsung_TapelessAGVDlg::OnSavepathPath8)
	ON_COMMAND(ID_SAVEPATH_PATH9, &CSamsung_TapelessAGVDlg::OnSavepathPath9)
	ON_COMMAND(ID_SAVEPATH_PATH10, &CSamsung_TapelessAGVDlg::OnSavepathPath10)
	ON_COMMAND(ID_LOADPATH_PATH1, &CSamsung_TapelessAGVDlg::OnLoadpathPath1)
	ON_COMMAND(ID_LOADPATH_PATH2, &CSamsung_TapelessAGVDlg::OnLoadpathPath2)
	ON_COMMAND(ID_LOADPATH_PATH3, &CSamsung_TapelessAGVDlg::OnLoadpathPath3)
	ON_COMMAND(ID_LOADPATH_PATH4, &CSamsung_TapelessAGVDlg::OnLoadpathPath4)
	ON_COMMAND(ID_LOADPATH_PATH5, &CSamsung_TapelessAGVDlg::OnLoadpathPath5)
	ON_COMMAND(ID_LOADPATH_PATH6, &CSamsung_TapelessAGVDlg::OnLoadpathPath6)
	ON_COMMAND(ID_LOADPATH_PATH7, &CSamsung_TapelessAGVDlg::OnLoadpathPath7)
	ON_COMMAND(ID_LOADPATH_PATH8, &CSamsung_TapelessAGVDlg::OnLoadpathPath8)
	ON_COMMAND(ID_LOADPATH_PATH9, &CSamsung_TapelessAGVDlg::OnLoadpathPath9)
	ON_COMMAND(ID_LOADPATH_PATH10, &CSamsung_TapelessAGVDlg::OnLoadpathPath10)
	ON_COMMAND(ID_BLOCKSIZE_SETTINGSIZE, &CSamsung_TapelessAGVDlg::OnBlocksizeSettingsize)
	ON_COMMAND(ID_CHECKPOINT_DEFAULT, &CSamsung_TapelessAGVDlg::OnCheckpointDefault)
	ON_COMMAND(ID_CHECKPOINT_CHECK, &CSamsung_TapelessAGVDlg::OnCheckpointCheck)
	ON_COMMAND(ID_OBSTACLE_DEFAULT, &CSamsung_TapelessAGVDlg::OnObstacleDefault)
	ON_COMMAND(ID_OBSTACLE_DETECT, &CSamsung_TapelessAGVDlg::OnObstacleDetect)
	ON_COMMAND(ID_COMMUNICATION_ZIGBEE, &CSamsung_TapelessAGVDlg::OnCommunicationZigbee)
	ON_COMMAND(ID_COMMUNICATION_GPIO, &CSamsung_TapelessAGVDlg::OnCommunicationGPIO)
	ON_COMMAND(ID_COMMUNICATION_DOOROPEN, &CSamsung_TapelessAGVDlg::OnCommunicationDoorOpen)
	ON_COMMAND(ID_BEHAVIOR_SAVETEMPORARYMAP, &CSamsung_TapelessAGVDlg::OnBehaviorSaveTemporaryMap)
	ON_COMMAND(ID_BEHAVIOR_LOADTEMPORARYMAP, &CSamsung_TapelessAGVDlg::OnBehaviorLoadTemporaryMap)
	ON_COMMAND(ID_FILE_LOAD_ISSACPATH, &CSamsung_TapelessAGVDlg::OnFileLoadISSACPath)
	END_MESSAGE_MAP()


// CSamsung_TapelessAGVDlg 메시지 처리기

BOOL CSamsung_TapelessAGVDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 시스템 메뉴에 "정보..." 메뉴 항목을 추가합니다.

	// IDM_ABOUTBOX는 시스템 명령 범위에 있어야 합니다.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 이 대화 상자의 아이콘을 설정합니다. 응용 프로그램의 주 창이 대화 상자가 아닐 경우에는
	//  프레임워크가 이 작업을 자동으로 수행합니다.
	SetIcon(m_hIcon, TRUE);			// 큰 아이콘을 설정합니다.
	SetIcon(m_hIcon, FALSE);		// 작은 아이콘을 설정합니다.

	//_CrtSetBreakAlloc(4974);


	// TODO: 여기에 추가 초기화 작업을 추가합니다.
	int nMainWidth = 1024;//총 폭 크기
	int nMainHeight = 768;//총 높이 크기
	int nHeightgap=120;// 높이 값
	int nTextWidthGap=nMainWidth*2/3*1/6;//텍스트 폭
	int nTextHeighthGap=25;//텍스트 높이
	int nImageWidth= 320;//nMainWidth*1/3;//영상 폭
	int nImageHeight=240;//(nMainHeight-nHeightgap)/2.0;//영상 높이

	this->MoveWindow(0, 0, nMainWidth, nMainHeight);//main UI

	m_MemDC_sensorconnection.CreateCompatibleDC(GetDlgItem(IDC_STATIC_SENSORCONNECTION)->GetDC());
	m_pDC_sensorconnection = GetDlgItem(IDC_STATIC_SENSORCONNECTION)->GetDC();

	m_KUNSUI3DDlg.Create(IDD_KUNSUI3D_DIALOG, this);
	m_KUNSUI3DDlg.MoveWindow(0, 30, nMainWidth - nImageWidth - 30, nMainHeight - nHeightgap - 30);//Drawing data UI
	m_KUNSUI3DDlg.ShowWindow(SW_SHOW);

	m_PSettingDlg.Create(IDD_POINTSETTING_DIALOG, this);
	m_PSettingDlg.MoveWindow(0, 0, 600, 400);//Drawing data UI

	m_BlockBox.Create(IDD_BLOCK_SIZE, this);
	m_BlockBox.MoveWindow(500, 200, 230, 180);

	GetDlgItem(IDC_LIST_INFO)->MoveWindow(nMainWidth*2/3-200, 0, 200, nMainHeight-nHeightgap);
	m_nitemID=-1;

	initMenu(); // Menu 초기화

	////////////////////////////////////////////////////////////////////

	// TODO: Add extra initialization here
	m_LargeImage = new CImageList;
	m_LargeImage->Create(79, 79, ILC_COLOR24, 4, 4); 

	CClientDC dc(this);

	int i;
	UINT ids[]={IDB_BITMAP1, IDB_BITMAP2, IDB_BITMAP3, IDB_BITMAP4,IDB_BITMAP5,IDB_BITMAP6,IDB_BITMAP7,IDB_BITMAP8,IDB_BITMAP9,IDB_BITMAP10,
		IDB_BITMAP11, IDB_BITMAP12, IDB_BITMAP13, IDB_BITMAP14,IDB_BITMAP15,IDB_BITMAP16,IDB_BITMAP17,IDB_BITMAP18,IDB_BITMAP19,IDB_BITMAP20,
		IDB_BITMAP21, IDB_BITMAP22, IDB_BITMAP23, IDB_BITMAP24,IDB_BITMAP25,IDB_BITMAP26,IDB_BITMAP27,IDB_BITMAP28,IDB_BITMAP29,IDB_BITMAP30,
		IDB_BITMAP31, IDB_BITMAP32, IDB_BITMAP33, IDB_BITMAP34,IDB_BITMAP35,IDB_BITMAP36,IDB_BITMAP37,IDB_BITMAP38,IDB_BITMAP39,IDB_BITMAP40,
		IDB_BITMAP41, IDB_BITMAP42, IDB_BITMAP43, IDB_BITMAP44};
	CBitmap bit;
	BITMAP binfo;
	for(i=0; i < 44; i++)
	{
		bit.LoadBitmap(ids[i]);
		bit.GetBitmapBits(sizeof(binfo), &binfo);
		m_LargeImage->Add(&bit, (CBitmap*)NULL);
		bit.DeleteObject();
	}

	m_cInfo.SetImageList(m_LargeImage, LVSIL_NORMAL);

	for(i=0; i < PATHBLOCKNUM; i++)
	{
		int n=PathBlockIDNum[i];

		m_cInfo.InsertItem(n,TEXT("Path Block"),n);
	}

	for(int i=0; i<4;i++){m_bsetWayPoint[i]=false;}
	for(int i=0; i<2;i++){m_bsetCheckPoint[i]=false;}
	for(int i=0; i<2;i++){m_bsetObstacleDetection[i]=false;}
	for(int i=0; i<6;i++){m_bsetVelocity[i]=false;}
	for(int i=0; i<11;i++){m_bsetViaPoint[i]=false;}

	CRect rect;
	this->GetClientRect(rect);
	GetDlgItem(IDC_STATIC_SENSORCONNECTION)->MoveWindow(0, 0, rect.right, 30);
	m_CamInfoPicCtrl.MoveWindow(nMainWidth - nImageWidth- 20, 30, nImageWidth, nImageHeight); //영상정보 뿌려주는 첫 번째 창
	m_KUNSUI3DDlg.setCamInfoDC(m_CamInfoPicCtrl.GetDC());
	GetDlgItem(IDC_CEILINGCAMERA_STATIC)->MoveWindow(nMainWidth - nImageWidth- 20, nImageHeight + 35, nImageWidth, 15);
	m_CamInfoPicCtrl2.MoveWindow((int)nMainWidth*2/3+5, (int)(nMainHeight-nHeightgap)/2.0+5+nTextHeighthGap, nImageWidth, nImageHeight); //영상정보 뿌려주는 첫 번째 창
	m_KUNSUI3DDlg.setCamInfo2DC(m_CamInfoPicCtrl2.GetDC());
	m_KUNSUI3DDlg.setCamSizeInfo(nImageWidth,nImageHeight);

	m_XPosEditCtrl.MoveWindow(5,nMainHeight-nHeightgap+5,nTextWidthGap,nTextHeighthGap);
	m_YPosEditCtrl.MoveWindow(5+nTextWidthGap+10,nMainHeight-nHeightgap+5,nTextWidthGap,nTextHeighthGap);
	m_TPosEditCtrl.MoveWindow(5+2*(nTextWidthGap+10),nMainHeight-nHeightgap+5,nTextWidthGap,nTextHeighthGap);
	m_AvoidCheckbox.MoveWindow(5+3*(nTextWidthGap+10),nMainHeight-nHeightgap+5,nTextWidthGap,nTextHeighthGap);
	m_SaveWaypointButton.MoveWindow(5+4*(nTextWidthGap+10),nMainHeight-nHeightgap+5,nTextWidthGap,nTextHeighthGap);

	m_XPosEditCtrl.SetWindowText(_T("X: 0 m"));
	m_YPosEditCtrl.SetWindowText(_T("Y: 0 m"));
	m_TPosEditCtrl.SetWindowText(_T("Th: 0 deg"));

	if(!KuRobotParameter::getInstance()->initialize())
	{
		exit(1);
	}

	m_KuThread.start(doThread, this, 200, "CSamsung_TapelessAGVDlg main thread"); //메인 스레드 시작

	MobileSupervisor::getInstance()->loadMap();
	SensorSupervisor::getInstance()->DataRecodingNPlay();

	KuDrawingInfo::getInstance()->setRenderMapflag(true);	
	CMenu* pMainMenu = GetMenu();
	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_RENDERMAP, MF_CHECKED);

	OnGuiinformationRenderlaser();
	OnGuiinformationRenderkinect();
	OnGuiinformationRenderpath();


	string strDataPlay = KuRobotParameter::getInstance()->getDataPlay();
/**/
	if(strDataPlay!="yes" ){ //레코딩된 데이터를 읽어와 play하는 경우이다.
 		Sleep(7000);
 		OnSensorconnectionHokuyoutm();
		Sleep(500);
 		OnSensorconnectionE2boximu();
 		Sleep(500);
 		OnSensorconnectionCeilingcamera();
 		Sleep(500);
		OnSensorconnectionKinect();
		Sleep(500);
		
 		SensorSupervisor::getInstance()->connectSwitch(); // Switch 시리얼 통신 연결
	}
 	OnSensorconnectionWheelactuator();
 	Sleep(500);	
/**/

	KuPRIMUSCommSupervisor::getInstance()->init();
	CAGVCommSupervisor::getInstance()->initZigbeeComm(); // Zigbee 통신 초기화
	CAGVCommSupervisor::getInstance()->initGPIOComm(); // GPIO 통신 초기화
	//	KuDrawingInfo::getInstance()->setRobotPos(KuRobotParameter::getInstance()->getInitRobotPose());
	
	/*Zigbee 샘플코드
	for(int i=0; i<301; i++)
	{
		CAGVCommSupervisor::getInstance()->checkAbnormalState(500,0);
	}
	Sleep(1000);
	for(int i=0; i<10; i++)
	{
		CAGVCommSupervisor::getInstance()->checkAbnormalState(501,0);
	}
	*/
	

	m_dX=0;
	m_dY=0;
	m_dThetaDeg=0;
	m_dTheta=0;

	SetTimer(0,100, NULL);

	// Autonomous navigation
	ifstream file_autonomous_navigation("./data/autonomous_navigation.txt");
	int nAutonomousNavigation(0);

	if(file_autonomous_navigation.is_open())
	{
		file_autonomous_navigation >> nAutonomousNavigation;

		file_autonomous_navigation.close();
	}

	if(nAutonomousNavigation)
	{
// 		OnBehaviorAutonomousgotogoal();
	}

	return TRUE;  // 포커스를 컨트롤에 설정하지 않으면 TRUE를 반환합니다.
}

void CSamsung_TapelessAGVDlg::displaySensorConnection(void)
{
	CWnd* pWnd = GetDlgItem(IDC_STATIC_SENSORCONNECTION);
	CRect rect;
	HFONT font, oldfont;
	CString c;

	pWnd->GetClientRect(&rect);

	CBitmap Bmp, *pOldBmp;

	if(m_pDC_sensorconnection)// && m_MemDC_info)
	{
		Bmp.CreateCompatibleBitmap(m_pDC_sensorconnection, rect.Width(), rect.Height());
		pOldBmp = m_MemDC_sensorconnection.SelectObject(&Bmp);

		m_MemDC_sensorconnection.SetBkMode(TRANSPARENT);
		m_MemDC_sensorconnection.SetTextColor(RGB(0, 0, 0));

		// Font
		font = CreateFont(11, 0, 0, 0, 0, 0, 0, 0, HANGEUL_CHARSET, 0, 0, 0, 0, _T("verdana"));
		oldfont=(HFONT)SelectObject(m_MemDC_sensorconnection,font);

		// Draw ////////////////////////////////////////////////////////////////
		CBrush brushRed, brushDarkGreen, brushGray, brushDarkGray;

		brushDarkGreen.CreateSolidBrush(RGB(0, 180, 0));
		brushRed.CreateSolidBrush(RGB(255, 0, 0));
		brushGray.CreateSolidBrush(RGB(60, 60, 60));
		brushDarkGray.CreateSolidBrush(RGB(30, 30, 30));

		// Border -------------------
		m_MemDC_sensorconnection.SelectStockObject(BLACK_PEN);
		m_MemDC_sensorconnection.SelectObject(&brushDarkGray);
		m_MemDC_sensorconnection.Rectangle(0, 0, rect.Width(), rect.Height());

		// Device status ------------
		CString sText;
// 		CANSSensorInterface::SensorStatus& sensor_status = m_ans.get_sensor_status();
		int nPosX(0), nBlockWidth((int)(110 * rect.Width() / 1280.)), nBlockHeight(30);

		m_MemDC_sensorconnection.SetTextAlign(TA_CENTER);
		m_MemDC_sensorconnection.SetTextColor(RGB(255, 255, 255));

		// Robot
		m_MemDC_sensorconnection.SelectStockObject(BLACK_PEN);
		if(SensorSupervisor::getInstance()->isRobotConnected())
		{
			m_MemDC_sensorconnection.SelectObject(&brushDarkGreen);
		}
		else
		{
			m_MemDC_sensorconnection.SelectObject(&brushGray);
		}
		m_MemDC_sensorconnection.Rectangle(nPosX, 0, nPosX + nBlockWidth, nBlockHeight);
		sText = _T("Robot");
		m_MemDC_sensorconnection.TextOut(nPosX + nBlockWidth / 2, 10, sText);
		nPosX += nBlockWidth;

		// Camera
		m_MemDC_sensorconnection.SelectStockObject(BLACK_PEN);
		if(SensorSupervisor::getInstance()->isCameraConnected())
		{
			m_MemDC_sensorconnection.SelectObject(&brushDarkGreen);
		}
		else
		{
			m_MemDC_sensorconnection.SelectObject(&brushGray);
		}
		m_MemDC_sensorconnection.Rectangle(nPosX, 0, nPosX + nBlockWidth, nBlockHeight);
		sText = _T("Camera");
		m_MemDC_sensorconnection.TextOut(nPosX + nBlockWidth / 2, 10, sText);
		nPosX += nBlockWidth;

		// Front laser scanner
		m_MemDC_sensorconnection.SelectStockObject(BLACK_PEN);
		if(SensorSupervisor::getInstance()->isFrontLRFConnected())
		{
			m_MemDC_sensorconnection.SelectObject(&brushDarkGreen);
		}
		else
		{
			m_MemDC_sensorconnection.SelectObject(&brushGray);
		}
		m_MemDC_sensorconnection.Rectangle(nPosX, 0, nPosX + nBlockWidth, nBlockHeight);
		sText = _T("Front LRF");
		m_MemDC_sensorconnection.TextOut(nPosX + nBlockWidth / 2, 10, sText);
		nPosX += nBlockWidth;

		// Rear laser scanner
		m_MemDC_sensorconnection.SelectStockObject(BLACK_PEN);
		if(SensorSupervisor::getInstance()->isRearLRFConnected())
		{
			m_MemDC_sensorconnection.SelectObject(&brushDarkGreen);
		}
		else
		{
			m_MemDC_sensorconnection.SelectObject(&brushGray);
		}
		m_MemDC_sensorconnection.Rectangle(nPosX, 0, nPosX + nBlockWidth, nBlockHeight);
		sText = _T("Rear LRF");
		m_MemDC_sensorconnection.TextOut(nPosX + nBlockWidth / 2, 10, sText);
		nPosX += nBlockWidth;

		// Kinect
		m_MemDC_sensorconnection.SelectStockObject(BLACK_PEN);
		if(SensorSupervisor::getInstance()->isKinectConnected())
		{
			m_MemDC_sensorconnection.SelectObject(&brushDarkGreen);
		}
		else
		{
			m_MemDC_sensorconnection.SelectObject(&brushGray);
		}
		m_MemDC_sensorconnection.Rectangle(nPosX, 0, nPosX + nBlockWidth, nBlockHeight);
		sText = _T("Kinect");
		m_MemDC_sensorconnection.TextOut(nPosX + nBlockWidth / 2, 10, sText);
		nPosX += nBlockWidth;

		// Gyro
		m_MemDC_sensorconnection.SelectStockObject(BLACK_PEN);
		if(SensorSupervisor::getInstance()->isGyroConnected())
		{
			m_MemDC_sensorconnection.SelectObject(&brushDarkGreen);
		}
		else
		{
			m_MemDC_sensorconnection.SelectObject(&brushGray);
		}
		m_MemDC_sensorconnection.Rectangle(nPosX, 0, nPosX + nBlockWidth, nBlockHeight);
		sText = _T("Gyro");
		m_MemDC_sensorconnection.TextOut(nPosX + nBlockWidth / 2, 10, sText);
		nPosX += nBlockWidth;

		// Encoder
		m_MemDC_sensorconnection.SelectStockObject(BLACK_PEN);
		if(0)//sensor_status.encoder_connection == CANSSensorInterface::SENSOR_CONNECTED)
		{
			m_MemDC_sensorconnection.SelectObject(&brushDarkGreen);
		}
		else
		{
			m_MemDC_sensorconnection.SelectObject(&brushGray);
		}
		m_MemDC_sensorconnection.Rectangle(nPosX, 0, nPosX + nBlockWidth, nBlockHeight);
		sText = _T("Encoder");
		m_MemDC_sensorconnection.TextOut(nPosX + nBlockWidth / 2, 10, sText);
		nPosX += nBlockWidth;

		// Button box
		m_MemDC_sensorconnection.SelectStockObject(BLACK_PEN);
		if(SensorSupervisor::getInstance()->isButtonBoxConnected())
		{
			m_MemDC_sensorconnection.SelectObject(&brushDarkGreen);
		}
		else
		{
			m_MemDC_sensorconnection.SelectObject(&brushGray);
		}
		m_MemDC_sensorconnection.Rectangle(nPosX, 0, nPosX + nBlockWidth, nBlockHeight);
		sText = _T("Button Box");
		m_MemDC_sensorconnection.TextOut(nPosX + nBlockWidth / 2, 10, sText);
		nPosX += nBlockWidth + 20;

		// Comm. status -------------
		// ISSAC
		m_MemDC_sensorconnection.SelectStockObject(BLACK_PEN);
		if(TotalTcpipCommunication::getInstance()->isISSACConnected())
		{
			m_MemDC_sensorconnection.SelectObject(&brushDarkGreen);
		}
		else
		{
			m_MemDC_sensorconnection.SelectObject(&brushGray);
		}
		m_MemDC_sensorconnection.Rectangle(nPosX, 0, nPosX + nBlockWidth, nBlockHeight);
		sText = _T("ISSAC");
		m_MemDC_sensorconnection.TextOut(nPosX + nBlockWidth / 2, 10, sText);
		nPosX += nBlockWidth;

		// IoT module
		m_MemDC_sensorconnection.SelectStockObject(BLACK_PEN);
		if(TotalTcpipCommunication::getInstance()->isIoTModuleConnected())
		{
			m_MemDC_sensorconnection.SelectObject(&brushDarkGreen);
		}
		else
		{
			m_MemDC_sensorconnection.SelectObject(&brushGray);
		}
		m_MemDC_sensorconnection.Rectangle(nPosX, 0, nPosX + nBlockWidth, nBlockHeight);
		sText = _T("IoT Module");
		m_MemDC_sensorconnection.TextOut(nPosX + nBlockWidth / 2, 10, sText);
		nPosX += nBlockWidth;

		// Zigbee
		m_MemDC_sensorconnection.SelectStockObject(BLACK_PEN);
		if(CAGVCommSupervisor::getInstance()->isZigbeeConnected())
		{
			m_MemDC_sensorconnection.SelectObject(&brushDarkGreen);
		}
		else
		{
			m_MemDC_sensorconnection.SelectObject(&brushGray);
		}
		m_MemDC_sensorconnection.Rectangle(nPosX, 0, nPosX + nBlockWidth, nBlockHeight);
		sText = _T("Zigbee");
		m_MemDC_sensorconnection.TextOut(nPosX + nBlockWidth / 2, 10, sText);
		nPosX += nBlockWidth;

		////////////////////////////////////////////////////////////////////////

		// Delete pens, brushes
		brushDarkGreen.DeleteObject();
		brushRed.DeleteObject();
		brushGray.DeleteObject();
		brushDarkGray.DeleteObject();

		// Copy from MemDC to pDC
		m_pDC_sensorconnection->BitBlt(0, 0, rect.Width(), rect.Height(), &m_MemDC_sensorconnection, 0, 0, SRCCOPY);
		m_MemDC_sensorconnection.SelectObject(pOldBmp);
		Bmp.DeleteObject();
		DeleteObject(font);
	}
}

void CSamsung_TapelessAGVDlg::doThread(void* arg)
{
	CKUNSUI3DDlg* p3DMapDlg = (CKUNSUI3DDlg*)arg;
	CSamsung_TapelessAGVDlg* pAGVDlg = (CSamsung_TapelessAGVDlg*)arg;

	pAGVDlg->	m_CriticalSection.Lock();
	p3DMapDlg->UpdateWindow();
	pAGVDlg->m_CriticalSection.Unlock();

}

void CSamsung_TapelessAGVDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 대화 상자에 최소화 단추를 추가할 경우 아이콘을 그리려면
//  아래 코드가 필요합니다. 문서/뷰 모델을 사용하는 MFC 응용 프로그램의 경우에는
//  프레임워크에서 이 작업을 자동으로 수행합니다.

void CSamsung_TapelessAGVDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 그리기를 위한 디바이스 컨텍스트입니다.

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 클라이언트 사각형에서 아이콘을 가운데에 맞춥니다.
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 아이콘을 그립니다.
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// 사용자가 최소화된 창을 끄는 동안에 커서가 표시되도록 시스템에서
//  이 함수를 호출합니다.
HCURSOR CSamsung_TapelessAGVDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CSamsung_TapelessAGVDlg::OnGuiinformationSetrobotpose()
{
	printf("set Robot Pose\n");

	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_GUIINFORMATION_SETROBOTPOSE, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETROBOTPOSE, MF_UNCHECKED);
		m_KUNSUI3DDlg.setRobotPosFlag( false );
		m_bSetRobotPos=false;

	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETROBOTPOSE, MF_CHECKED);
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETGOALPOSE, MF_UNCHECKED);
		m_KUNSUI3DDlg.setRobotPosFlag( true );
		m_KUNSUI3DDlg.setGoalPosFlag( false );
		m_bSetRobotPos=true;

	} 	
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
}


void CSamsung_TapelessAGVDlg::OnGuiinformationSetgoalpose()
{
	printf("set Goal Pose\n");

	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_GUIINFORMATION_SETGOALPOSE, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETGOALPOSE, MF_UNCHECKED);
		m_KUNSUI3DDlg.setGoalPosFlag( false );
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETGOALPOSE, MF_CHECKED);
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETROBOTPOSE, MF_UNCHECKED);
		m_KUNSUI3DDlg.setRobotPosFlag( false );
		m_KUNSUI3DDlg.setGoalPosFlag( true );
		m_bSetRobotPos=false;

	} 	
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
}


void CSamsung_TapelessAGVDlg::OnGuiinformationRendermap()
{
	printf("Render Mape\n");

	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_GUIINFORMATION_RENDERMAP, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_RENDERMAP, MF_UNCHECKED);
		KuDrawingInfo::getInstance()->setRenderMapflag(false);

	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_RENDERMAP, MF_CHECKED);
		KuDrawingInfo::getInstance()->setRenderMapflag(true);


	} 	
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
}


void CSamsung_TapelessAGVDlg::OnGuiinformationRenderlaser()
{
	printf("Render Laser\n");

	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_GUIINFORMATION_RENDERLASER, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_RENDERLASER, MF_UNCHECKED);
		KuDrawingInfo::getInstance()->setRenderLaserflag(false);
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_RENDERLASER, MF_CHECKED);
		KuDrawingInfo::getInstance()->setRenderLaserflag(true);

	} 	
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
}


void CSamsung_TapelessAGVDlg::OnGuiinformationRenderkinect()
{
	printf("Render Kinect\n");

	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_GUIINFORMATION_RENDERKINECT, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_RENDERKINECT, MF_UNCHECKED);
		KuDrawingInfo::getInstance()->setRenderKinectflag(false);
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_RENDERKINECT, MF_CHECKED);
		KuDrawingInfo::getInstance()->setRenderKinectflag(true);

	} 	
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
}


void CSamsung_TapelessAGVDlg::OnGuiinformationRenderpath()
{
	printf("Render Path\n");

	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_GUIINFORMATION_RENDERPATH, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_RENDERPATH, MF_UNCHECKED);
		KuDrawingInfo::getInstance()->setRenderPathflag(false);
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_RENDERPATH, MF_CHECKED);
		KuDrawingInfo::getInstance()->setRenderPathflag(true);
	} 	
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
}

void CSamsung_TapelessAGVDlg::OnGuiinformationRenderceilingimage()
{
	printf("Render Ceilingimageh\n");

	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_GUIINFORMATION_RENDERCEILINGIMAGE, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_RENDERCEILINGIMAGE, MF_UNCHECKED);
		KuDrawingInfo::getInstance()->setRenderCeilingImageflag(false);
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_RENDERCEILINGIMAGE, MF_CHECKED);
		KuDrawingInfo::getInstance()->setRenderCeilingImageflag(true);
	} 	
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
}

void CSamsung_TapelessAGVDlg::OnSensorconnectionHokuyoutm()
{
	printf("connect Laser\n");

	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_SENSORCONNECTION_HOKUYOUTM, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SENSORCONNECTION_HOKUYOUTM, MF_UNCHECKED);
		HokuyoURG04LXInterface::getInstance()->disconnectLaserScanner();
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SENSORCONNECTION_HOKUYOUTM, MF_CHECKED);
		if(!SensorSupervisor::getInstance()->connectLaserScanner()) 	
			CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SENSORCONNECTION_HOKUYOUTM, MF_UNCHECKED);
	} 	
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
}


void CSamsung_TapelessAGVDlg::OnSensorconnectionKinect()
{
	printf("connect Kinect\n");

	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_SENSORCONNECTION_KINECT, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SENSORCONNECTION_KINECT, MF_UNCHECKED);
#ifdef USE_KINECT_VER_1
		CKinect::getInstance()->close_connection();
#else
		KinectSensorInterface::getInstance()->terminate();
#endif
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SENSORCONNECTION_KINECT, MF_CHECKED);
		if(!SensorSupervisor::getInstance()->connectionKinect())	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SENSORCONNECTION_KINECT, MF_UNCHECKED);

	} 	
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
}


void CSamsung_TapelessAGVDlg::OnSensorconnectionCeilingcamera()
{

	printf("connect Ceiling camera\n");

	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_SENSORCONNECTION_CEILINGCAMERA, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SENSORCONNECTION_CEILINGCAMERA, MF_UNCHECKED);

	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SENSORCONNECTION_CEILINGCAMERA, MF_CHECKED);
		if(!SensorSupervisor::getInstance()->connectionCeilingcamera()) CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SENSORCONNECTION_CEILINGCAMERA, MF_UNCHECKED);
	} 	 
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
}


void CSamsung_TapelessAGVDlg::OnSensorconnectionE2boximu()
{
	printf("connect IMU\n");

	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_SENSORCONNECTION_E2BOXIMU, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SENSORCONNECTION_E2BOXIMU, MF_UNCHECKED);
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SENSORCONNECTION_E2BOXIMU, MF_CHECKED);
		if(!SensorSupervisor::getInstance()->connectionGYRO()) CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SENSORCONNECTION_E2BOXIMU, MF_UNCHECKED);
	} 	
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
}


void CSamsung_TapelessAGVDlg::OnSensorconnectionWheelactuator()
{
	printf("connect Wheel\n");
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_SENSORCONNECTION_WHEELACTUATOR, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SENSORCONNECTION_WHEELACTUATOR, MF_UNCHECKED);
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SENSORCONNECTION_WHEELACTUATOR, MF_CHECKED);
		if(!SensorSupervisor::getInstance()->connectionWheelactuator()) CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SENSORCONNECTION_WHEELACTUATOR, MF_UNCHECKED);
	} 	
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
}


void CSamsung_TapelessAGVDlg::OnBehaviorGotogoal()
{
	printf("execute Go to goal \n");
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_BEHAVIOR_GOTOGOAL, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_BEHAVIOR_GOTOGOAL, MF_UNCHECKED);
		m_bexecuteflag=false;
		KuCommandMessage CMessage;
		CMessage.setCommandName(KuCommandMessage::TERMINATE_THREAD);
		CMessage.setBehaviorName(KuCommandMessage::GOTOGOAL_BH);
		MobileSupervisor::getInstance()->execute(CMessage);

	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_BEHAVIOR_GOTOGOAL, MF_CHECKED);
		m_bexecuteflag=true;
		KuCommandMessage CMessage;
		m_nBehaviorName=KuCommandMessage::GOTOGOAL_BH;
		CMessage.setCommandName(KuCommandMessage::START_THREAD);
		CMessage.setBehaviorName(KuCommandMessage::GOTOGOAL_BH);
		CMessage.setAvoidMode(m_bAvoidModeflag);		
		CMessage.setGoalPos(KuDrawingInfo::getInstance()->getGoalPos());
		CMessage.setRobotPos(KuDrawingInfo::getInstance()->getRobotPos());		 
		MobileSupervisor::getInstance()->execute(CMessage);
		m_pLocalizer=MobileSupervisor::getInstance()->getLocalizer(CMessage);
	} 	

	// TODO: 여기에 명령 처리기 코드를 추가합니다.
}


void CSamsung_TapelessAGVDlg::OnBehaviorMapbuilding()
{
	printf("execute Mapbuilding\n");
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_BEHAVIOR_MAPBUILDING, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_BEHAVIOR_MAPBUILDING, MF_UNCHECKED);
		EnableMenuItem(pMainMenu->GetSafeHmenu(), ID_BEHAVIOR_SAVETEMPORARYMAP, MF_DISABLED);
		EnableMenuItem(pMainMenu->GetSafeHmenu(), ID_BEHAVIOR_LOADTEMPORARYMAP, MF_DISABLED);
		m_bexecuteflag=false;		
		KuCommandMessage CMessage;
		CMessage.setCommandName(KuCommandMessage::TERMINATE_THREAD);
		CMessage.setBehaviorName(KuCommandMessage::HYBRID_MAP_BUILDING_BH);
		MobileSupervisor::getInstance()->execute(CMessage);
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 

		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_BEHAVIOR_MAPBUILDING, MF_CHECKED);
		EnableMenuItem(pMainMenu->GetSafeHmenu(), ID_BEHAVIOR_SAVETEMPORARYMAP, MF_ENABLED);
		EnableMenuItem(pMainMenu->GetSafeHmenu(), ID_BEHAVIOR_LOADTEMPORARYMAP, MF_DISABLED);
		KuDrawingInfo::getInstance()->setRenderBuildingMapflag(true);
		m_bexecuteflag=true;
		KuCommandMessage CMessage;
		m_nBehaviorName=KuCommandMessage::HYBRID_MAP_BUILDING_BH;
		CMessage.setCommandName(KuCommandMessage::START_THREAD);
		CMessage.setBehaviorName(KuCommandMessage::HYBRID_MAP_BUILDING_BH);
		CMessage.setRobotPos(KuRobotParameter::getInstance()->getInitRobotPoseForMap());
		CMessage.setMapSizeXmYm(KuRobotParameter::getInstance()->getMapSizeXm(),KuRobotParameter::getInstance()->getMapSizeYm());
		MobileSupervisor::getInstance()->execute(CMessage);
		m_pLocalizer=MobileSupervisor::getInstance()->getLocalizer(CMessage);
	} 	

	// TODO: 여기에 명령 처리기 코드를 추가합니다.
}

void CSamsung_TapelessAGVDlg::OnBehaviorTeachingpath()
{
	printf("execute Teachingpath\n");

	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_BEHAVIOR_TEACHINGPATH, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_BEHAVIOR_TEACHINGPATH, MF_UNCHECKED);
		m_bexecuteflag=false;		
		KuCommandMessage CMessage;
		CMessage.setCommandName(KuCommandMessage::TERMINATE_THREAD);
		CMessage.setBehaviorName(KuCommandMessage::PATH_TEACHING_BH);
		MobileSupervisor::getInstance()->execute(CMessage);
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 

		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_BEHAVIOR_TEACHINGPATH, MF_CHECKED);
		m_bexecuteflag=true;
		KuCommandMessage CMessage;
		m_nBehaviorName=KuCommandMessage::PATH_TEACHING_BH;
		CMessage.setCommandName(KuCommandMessage::START_THREAD);
		CMessage.setBehaviorName(KuCommandMessage::PATH_TEACHING_BH);
		CMessage.setRobotPos(KuDrawingInfo::getInstance()->getRobotPos());
		MobileSupervisor::getInstance()->execute(CMessage);
		m_pLocalizer=MobileSupervisor::getInstance()->getLocalizer(CMessage);
	} 	
}

void CSamsung_TapelessAGVDlg::OnBehaviorAutonomousgotogoal()
{
	printf("execute Autonomous Go to goal \n");

	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_BEHAVIOR_AUTONOMOUSGOTOGOAL, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_BEHAVIOR_AUTONOMOUSGOTOGOAL, MF_UNCHECKED);
		m_bexecuteflag=false;
		KuCommandMessage CMessage;
		CMessage.setCommandName(KuCommandMessage::TERMINATE_THREAD);
		CMessage.setBehaviorName(KuCommandMessage::AUTONOMOUS_GOTOGOAL);
		MobileSupervisor::getInstance()->execute(CMessage);
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_BEHAVIOR_AUTONOMOUSGOTOGOAL, MF_CHECKED);
		m_bexecuteflag=true;
		KuCommandMessage CMessage;
		m_nBehaviorName=KuCommandMessage::AUTONOMOUS_GOTOGOAL;
		CMessage.setCommandName(KuCommandMessage::START_THREAD);
		CMessage.setBehaviorName(KuCommandMessage::AUTONOMOUS_GOTOGOAL);
		CMessage.setAvoidMode(m_bAvoidModeflag);		
		CMessage.setGoalPos(KuDrawingInfo::getInstance()->getGoalPos());
		CMessage.setRobotPos(KuDrawingInfo::getInstance()->getRobotPos());		 
		MobileSupervisor::getInstance()->execute(CMessage);
		m_pLocalizer=MobileSupervisor::getInstance()->getLocalizer(CMessage);

		ofstream file_autonomous_navigation;
		file_autonomous_navigation.open("./data/autonomous_navigation.txt", ios::out);

		if(file_autonomous_navigation.is_open())
		{
			file_autonomous_navigation << 1; // enable

			file_autonomous_navigation.close();
		}
	} 	
}



void CSamsung_TapelessAGVDlg::OnBnClickedSaveWaypointButton()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	KuDrawingInfo::getInstance()->setWayPointflag(true);
}




BOOL CSamsung_TapelessAGVDlg::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.
	if(zDelta > 0) m_KUNSUI3DDlg.Zoom_In(0.5);
	else m_KUNSUI3DDlg.Zoom_Out(0.5);
	return CDialogEx::OnMouseWheel(nFlags, zDelta, pt);
}


void CSamsung_TapelessAGVDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.
	displaySensorConnection();

	KuPose RobotPos=KuDrawingInfo::getInstance()->getRobotPos();

	CString strRobotPos;
	strRobotPos.Format(_T("X: %.2f m"),RobotPos.getXm());
	m_XPosEditCtrl.SetWindowText(strRobotPos);
	strRobotPos.Format(_T("Y: %.2f m"),RobotPos.getYm());
	m_YPosEditCtrl.SetWindowText(strRobotPos);
	strRobotPos.Format(_T("Th: %.2f deg"),RobotPos.getThetaDeg());
	m_TPosEditCtrl.SetWindowText(strRobotPos);

	if(m_bexecuteflag==false)
	{
		// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.	
		if( KuRobotParameter::getInstance()->getDataRecoding()=="yes" ){ //데이터 레코딩을 수행하는 경우이다.
			SensorSupervisor::getInstance()->readSensorData();
		}
		else
		{
			SensorSupervisor::getInstance()->readOnlySensorData();
		}
		int_1DArray nLaserData181 = SensorSupervisor::getInstance()->getLaserDataFront();
		KuPose DelEncoder =SensorSupervisor::getInstance()->getEncoderDelPos();
		int_1DArray nKinectRangeData=SensorSupervisor::getInstance()->getKinectRangeData();
		IplImage *IplCeilingImage=SensorSupervisor::getInstance()->getCeilingImageData();
		double dTheta=SensorSupervisor::getInstance()->getGyroData();
		IplImage *IplKinectImage=SensorSupervisor::getInstance()->getKinectImageData();
		DelEncoder.setThetaDeg(dTheta);
		KuDrawingInfo::getInstance()->setKinectRangeData(nKinectRangeData);
		KuDrawingInfo::getInstance()->setFrontLaserData181(nLaserData181);
		KuDrawingInfo::getInstance()->setCeilingImageData(IplCeilingImage);
		KuDrawingInfo::getInstance()->setKinectImageData(IplKinectImage);

		double dX = RobotPos.getX() + DelEncoder.getX() * cos(RobotPos.getThetaRad()+DelEncoder.getThetaRad()/2.0) + 
			DelEncoder.getY() * sin(-RobotPos.getThetaRad()-DelEncoder.getThetaRad()/2.0);
		double dY = RobotPos.getY() + DelEncoder.getX() * sin(RobotPos.getThetaRad()+DelEncoder.getThetaRad()/2.0) + 
			DelEncoder.getY() * cos(RobotPos.getThetaRad()+DelEncoder.getThetaRad()/2.0);
		double dThetaDeg = RobotPos.getThetaDeg() + DelEncoder.getThetaDeg();
		// pose update	
		RobotPos.setX(dX);
		RobotPos.setY(dY);
		RobotPos.setThetaDeg(dThetaDeg);
		//RobotPos.setThetaDeg(m_dTheta);
		m_dX=dX;
		m_dY=dY;
		m_dThetaDeg=dThetaDeg;
		m_dTheta+=dTheta;

		//printf("m_dTheta=%f\n",DelEncoder.getThetaDeg());
		KuDrawingInfo::getInstance()->setRobotPos(RobotPos);

		switch(m_nBehaviorName){
		case KuCommandMessage::GOTOGOAL_BH:
			CheckMenuItem(GetMenu()->GetSafeHmenu(), ID_BEHAVIOR_GOTOGOAL, MF_UNCHECKED);
			break;
		case KuCommandMessage::HYBRID_MAP_BUILDING_BH:
			CheckMenuItem(GetMenu()->GetSafeHmenu(), ID_BEHAVIOR_MAPBUILDING, MF_UNCHECKED);
			break;
		case KuCommandMessage::AUTONOMOUS_GOTOGOAL:
			CheckMenuItem(GetMenu()->GetSafeHmenu(), ID_BEHAVIOR_AUTONOMOUSGOTOGOAL, MF_UNCHECKED);
			break;
		case KuCommandMessage::GLOBAL_LOCALIZATION_BH:
			CheckMenuItem(GetMenu()->GetSafeHmenu(), ID_BEHAVIOR_GLOBALLOCALIZATION, MF_UNCHECKED);
			break;
		default:break;
		}
	}
	else
	{
		KuCommandMessage CMessage;
		CMessage.setBehaviorName(m_nBehaviorName);
		if(!MobileSupervisor::getInstance()->getBehaviorStates(CMessage))
			m_bexecuteflag=false;
	}
	::InvalidateRect(m_KUNSUI3DDlg.GetSafeHwnd(), 0,FALSE);
	CDialogEx::OnTimer(nIDEvent);
}


BOOL CSamsung_TapelessAGVDlg::PreTranslateMessage(MSG* pMsg)
{
	// TODO: 여기에 특수화된 코드를 추가 및/또는 기본 클래스를 호출합니다.
	double dGRID=100.0; //mm 단위 10cm격자
	double dTVelIncrement=200;
	double dRVelIncrement =1;
	switch(pMsg->message){

	case WM_KEYDOWN:{
		//printf("%d\n", pMsg->wParam);
		switch(pMsg->wParam) {			 
			// 맵 확대,축소
		case VK_OEM_COMMA :
			if(m_bSetRobotPos){ //화면상에서 로봇의 위치좌표 정보만을 변화시켜준다. 로봇은 실제로 움직이지 않는다.
				KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
				RobotPos.setThetaDeg(RobotPos.getThetaDeg()+1);	
				//m_pLocalizer->setRobotPos(RobotPos); 
				KuDrawingInfo::getInstance()->setRobotPos(RobotPos);
				if(m_bexecuteflag)m_pLocalizer->setRobotPos(RobotPos);
			}

			break;
		case VK_OEM_PERIOD :
			if(m_bSetRobotPos){ //화면상에서 로봇의 위치좌표 정보만을 변화시켜준다. 로봇은 실제로 움직이지 않는다.
				KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
				RobotPos.setThetaDeg(RobotPos.getThetaDeg()-1);	
				//	m_pLocalizer->setRobotPos(RobotPos); 
				KuDrawingInfo::getInstance()->setRobotPos(RobotPos);
				if(m_bexecuteflag)m_pLocalizer->setRobotPos(RobotPos);
			}

			break;
		case VK_OEM_PLUS:
			m_KUNSUI3DDlg.Zoom_In(2.);
			return TRUE;

		case VK_OEM_MINUS:
			m_KUNSUI3DDlg.Zoom_Out(2.);
			return TRUE;

		case VK_UP:
			{ 
				if(m_bSetRobotPos){ //화면상에서 로봇의 위치좌표 정보만을 변화시켜준다. 로봇은 실제로 움직이지 않는다.
					KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
					RobotPos.setY(RobotPos.getY()+dGRID);	
					//m_pLocalizer->setRobotPos(RobotPos); 
					KuDrawingInfo::getInstance()->setRobotPos(RobotPos);
					if(m_bexecuteflag)m_pLocalizer->setRobotPos(RobotPos);


				}else{ //실제로 로봇을 구동한다.
					// 로봇 컨트롤
					//double dTVelIncrement = SSAGVWheelActuatorInterface::getInstance()->getTVelIncrement();
					double dTVel = SSAGVWheelActuatorInterface::getInstance()->getTVel();
					SSAGVWheelActuatorInterface::getInstance()->moveByTRVelocity(dTVel+dTVelIncrement,0.0);
					//SSAGVWheelActuatorInterface::getInstance()->moveByTRVelocity(200.0,6.0);

				}
				//현재 로봇 위치, 병진,회전 속도를 얻어와서 화면에 표시하는 부분.......
				//m_RobotPosStaticCtrl.SetWindowText(getRobotStatus());

				break;

			}
		case VK_DOWN:
			{
				if(m_bSetRobotPos){ //화면상에서 로봇의 위치좌표 정보만을 변화시켜준다. 로봇은 실제로 움직이지 않는다.
					KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
					RobotPos.setY(RobotPos.getY()-dGRID);	
					//m_pLocalizer->setRobotPos(RobotPos); 
					KuDrawingInfo::getInstance()->setRobotPos(RobotPos);
					if(m_bexecuteflag)m_pLocalizer->setRobotPos(RobotPos);


				}else{ //실제로 로봇을 구동한다.
					//double dTVelDecrement = SSAGVWheelActuatorInterface::getInstance()->getTVelIncrement();
					double dTVel = SSAGVWheelActuatorInterface::getInstance()->getTVel();
					//SSAGVWheelActuatorInterface::getInstance()->moveByTRVelocity(200.0,0.0);
					SSAGVWheelActuatorInterface::getInstance()->moveByTRVelocity(dTVel - dTVelIncrement,0.0);
				}
				//현재 로봇 위치, 병진,회전 속도를 얻어와서 화면에 표시하는 부분.......
				//	m_RobotPosStaticCtrl.SetWindowText(getRobotStatus());
				break;

			}

		case VK_LEFT:
			{
				if(m_bSetRobotPos){ //화면상에서 로봇의 위치좌표 정보만을 변화시켜준다. 로봇은 실제로 움직이지 않는다.
					KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
					RobotPos.setX(RobotPos.getX()-dGRID);	
					//m_pLocalizer->setRobotPos(RobotPos); 
					KuDrawingInfo::getInstance()->setRobotPos(RobotPos);
					if(m_bexecuteflag)m_pLocalizer->setRobotPos(RobotPos);


				}else{ //실제로 로봇을 구동한다.

					//	double dRVelIncrement = SSAGVWheelActuatorInterface::getInstance()->getRVelIncrement();
					double dRVel = SSAGVWheelActuatorInterface::getInstance()->getRVel();
					double dTVel = SSAGVWheelActuatorInterface::getInstance()->getTVel();

					SSAGVWheelActuatorInterface::getInstance()->moveByTRVelocity(dTVel,dRVel+dRVelIncrement);
				}
				//현재 로봇 위치, 병진,회전 속도를 얻어와서 화면에 표시하는 부분.......
				//		m_RobotPosStaticCtrl.SetWindowText(getRobotStatus());
				break;
			}

		case VK_RIGHT:
			{
				if(m_bSetRobotPos){ //화면상에서 로봇의 위치좌표 정보만을 변화시켜준다. 로봇은 실제로 움직이지 않는다.
					KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
					RobotPos.setX(RobotPos.getX()+dGRID);	
					//	m_pLocalizer->setRobotPos(RobotPos); 
					KuDrawingInfo::getInstance()->setRobotPos(RobotPos);
					if(m_bexecuteflag)m_pLocalizer->setRobotPos(RobotPos);


				}else{ //실제로 로봇을 구동한다.

					//double dRVelIncrement = SSAGVWheelActuatorInterface::getInstance()->getRVelIncrement();
					double dRVel = SSAGVWheelActuatorInterface::getInstance()->getRVel();
					double dTVel = SSAGVWheelActuatorInterface::getInstance()->getTVel();

					SSAGVWheelActuatorInterface::getInstance()->moveByTRVelocity(dTVel,dRVel-dRVelIncrement);
				}
				//현재 로봇 위치, 병진,회전 속도를 얻어와서 화면에 표시하는 부분.......
				//		m_RobotPosStaticCtrl.SetWindowText(getRobotStatus());
				break;
			}

		case VK_SHIFT:
			{
				SSAGVWheelActuatorInterface::getInstance()->stop();				
				//현재 로봇 위치, 병진,회전 속도를 얻어와서 화면에 표시하는 부분.......
				//m_RobotPosStaticCtrl.SetWindowText(getRobotStatus());
				return TRUE;
			}

		case 65: //a키 자율 주행 셋팅

			break;
		case 77: //m키 수동 주행 셋팅

			break; 
		case 48: case 49: case 50: case 51: case 52: case 53: case 54: case 55: case 56: case 57: return false;  
		}
		// 			if(m_bSetRobotPos){
		// 				KuPose RobotPos = m_pLocalizer->getRobotPos();
		// 				KuDrawingInfo::getInstance()->setRobotPos(RobotPos);
		// 		//		::SendMessage(m_KUNSUI3DDlg.GetSafeHwnd(), WM_PAINT, 0, NULL);	// 그림그리는 함수 호출
		// 			}
		return TRUE;
					}
	}

	return FALSE;
}



void CSamsung_TapelessAGVDlg::OnBnClickedOk()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CDialogEx::OnOK();
	OnDestroy();
}

void CSamsung_TapelessAGVDlg::OnDestroy()
{

	m_KuThread.terminate();
	KuCommandMessage CMessage;
	CMessage.setCommandName(KuCommandMessage::TERMINATE_THREAD);
	CMessage.setBehaviorName(KuCommandMessage::AUTONOMOUS_GOTOGOAL);
	MobileSupervisor::getInstance()->execute(CMessage);	
	CMessage.setCommandName(KuCommandMessage::TERMINATE_THREAD);
	CMessage.setBehaviorName(KuCommandMessage::HYBRID_MAP_BUILDING_BH);
	MobileSupervisor::getInstance()->execute(CMessage);
	CMessage.setCommandName(KuCommandMessage::TERMINATE_THREAD);
	CMessage.setBehaviorName(KuCommandMessage::PATH_TEACHING_BH);
	MobileSupervisor::getInstance()->execute(CMessage);	
	CMessage.setCommandName(KuCommandMessage::TERMINATE_THREAD);
	CMessage.setBehaviorName(KuCommandMessage:: HYBRID_MAP_BUILDING_BH);
	MobileSupervisor::getInstance()->execute(CMessage);
	CMessage.setCommandName(KuCommandMessage::TERMINATE_THREAD);
	CMessage.setBehaviorName(KuCommandMessage::GOTOGOAL_BH);
	MobileSupervisor::getInstance()->execute(CMessage);

	SensorSupervisor::getInstance()->stopAllSensor();

	m_MemDC_sensorconnection.DeleteDC();
	GetDlgItem(IDC_STATIC_SENSORCONNECTION)->ReleaseDC(m_pDC_sensorconnection);

//	::ShellExecute (NULL, _T("close"), _T("C:\atras_codes\[131202]Samsung_TapelessAGV_Total.ver2\Release\Samsung_TapelessAGVDlg.exe"),NULL, NULL, SW_HIDE);
// 	::ShellExecute (NULL, _T("open"), _T("Shutdown.exe"), _T(" -s -f -t 1"), NULL, SW_HIDE);

	FreeConsole();
	PostQuitMessage(0);

	AfxGetMainWnd()->SendMessage(WM_CLOSE);
	AfxGetMainWnd()->SendMessage(WM_DESTROY);
	AfxGetMainWnd()->SendMessage(WM_COMMAND,ID_APP_EXIT,NULL);

	CDialog::EndDialog(0);  
	CDialogEx::OnDestroy();
	CDialogEx::OnClose();

	KillTimer(1);	

	PostQuitMessage(WM_QUIT);	
	PostQuitMessage(WM_DESTROY);	


	DWORD processID = 0;
	GetWindowThreadProcessId(this->m_hWnd, &processID);
	HANDLE hProcess=OpenProcess(PROCESS_ALL_ACCESS,FALSE,processID);
	if(hProcess)
	{
		if(TerminateProcess(hProcess,0))
		{
			unsigned long nCode;
			GetExitCodeProcess(hProcess,&nCode);
		}
		CloseHandle(hProcess);
	}


	//exit(1);

	// TODO: 여기에 메시지 처리기 코드를 추가합니다.
}


void CSamsung_TapelessAGVDlg::OnBnClickedObsavoidCheck()
{
	// TODO: Add your control notification handler code here
	if(m_bAvoidModeflag == true){
		m_bAvoidModeflag = false;
	}
	else{
		m_bAvoidModeflag = true;
	}
}


void CSamsung_TapelessAGVDlg::OnBehaviorGloballocalization()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_BEHAVIOR_GLOBALLOCALIZATION, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_BEHAVIOR_GLOBALLOCALIZATION, MF_UNCHECKED);
		m_bexecuteflag=false;
		KuCommandMessage CMessage;
		CMessage.setCommandName(KuCommandMessage::TERMINATE_THREAD);	
		CMessage.setBehaviorName(KuCommandMessage::GLOBAL_LOCALIZATION_BH);
		MobileSupervisor::getInstance()->execute(CMessage);
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_BEHAVIOR_GLOBALLOCALIZATION, MF_CHECKED);
		m_bexecuteflag=true;
		KuCommandMessage CMessage;
		m_nBehaviorName=KuCommandMessage::GLOBAL_LOCALIZATION_BH;
		CMessage.setCommandName(KuCommandMessage::START_THREAD);
		CMessage.setBehaviorName(KuCommandMessage::GLOBAL_LOCALIZATION_BH);
		//CMessage.setRobotPos(m_pLocalizer->getRobotPos() );
		MobileSupervisor::getInstance()->execute(CMessage);
		m_pLocalizer=MobileSupervisor::getInstance()->getLocalizer(CMessage);
		printf("BehaviorGlobalLocalization\n");
	} 
}


void CSamsung_TapelessAGVDlg::OnBehaviorGlobalMapBuilding()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_BEHAVIOR_GLOBALMAPBUILDING, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_BEHAVIOR_GLOBALMAPBUILDING, MF_UNCHECKED);
		m_bexecuteflag=false;
		KuCommandMessage CMessage;
		CMessage.setCommandName(KuCommandMessage::TERMINATE_THREAD);	
		CMessage.setBehaviorName(KuCommandMessage::GLOBAL_MAP_BUILDING_BH);
		MobileSupervisor::getInstance()->execute(CMessage);
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_BEHAVIOR_GLOBALMAPBUILDING, MF_CHECKED);
		m_bexecuteflag=true;
		KuCommandMessage CMessage;
		m_nBehaviorName=KuCommandMessage::GLOBAL_MAP_BUILDING_BH;
		CMessage.setCommandName(KuCommandMessage::START_THREAD);
		CMessage.setBehaviorName(KuCommandMessage::GLOBAL_MAP_BUILDING_BH);
		MobileSupervisor::getInstance()->execute(CMessage);
		m_pLocalizer=MobileSupervisor::getInstance()->getLocalizer(CMessage);
		printf("BehaviorGlobalMapBuilding\n");
	} 
}


void CSamsung_TapelessAGVDlg::OnGuiinformationRenderzonemap()
{
	printf("Render Zone Map\n");

	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_GUIINFORMATION_RENDERZONEMAP, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_RENDERZONEMAP, MF_UNCHECKED);
		KuDrawingInfo::getInstance()->setRenderZoneMapflag(false);

	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_RENDERZONEMAP, MF_CHECKED);
		KuDrawingInfo::getInstance()->setRenderZoneMapflag(true);


	} 	
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
}


void CSamsung_TapelessAGVDlg::OnBehaviorMulti()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_BEHAVIOR_MULTI, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_BEHAVIOR_MULTI, MF_UNCHECKED);
		m_bexecuteflag=false;
		KuCommandMessage CMessage;
		CMessage.setCommandName(KuCommandMessage::TERMINATE_THREAD);	
		CMessage.setBehaviorName(KuCommandMessage::MULTI_GOTOGOAL_BH);
		MobileSupervisor::getInstance()->execute(CMessage);
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_BEHAVIOR_MULTI, MF_CHECKED);
		m_bexecuteflag=true;
		KuCommandMessage CMessage;
		m_nBehaviorName=KuCommandMessage::MULTI_GOTOGOAL_BH;
		CMessage.setCommandName(KuCommandMessage::START_THREAD);
		CMessage.setBehaviorName(KuCommandMessage::MULTI_GOTOGOAL_BH);
		//CMessage.setRobotPos(m_pLocalizer->getRobotPos() );
		MobileSupervisor::getInstance()->execute(CMessage);
		m_pLocalizer=MobileSupervisor::getInstance()->getLocalizer(CMessage);
		printf("BehaviorGlobalLocalization\n");
	} 
}


void CSamsung_TapelessAGVDlg::OnCommunicationConnect()
{
	printf("connect Communication\n");

	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_COMMUNICATION_CONNECT, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_COMMUNICATION_CONNECT, MF_UNCHECKED);
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_COMMUNICATION_CONNECT, MF_CHECKED);
		if(!MobileSupervisor::getInstance()->connectCommunication()) CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_COMMUNICATION_CONNECT, MF_UNCHECKED);
	} 	
	// TODO: 여기
}


void CSamsung_TapelessAGVDlg::OnNMThemeChangedScrollbar1(NMHDR *pNMHDR, LRESULT *pResult)
{
	// 이 기능을 사용하려면 Windows XP 이상이 필요합니다.
	// _WIN32_WINNT 기호는 0x0501보다 크거나 같아야 합니다.
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	*pResult = 0;
}


void CSamsung_TapelessAGVDlg::OnVScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.
	SCROLLINFO  scrinfo;

	if(pScrollBar)
	{

		// 스크롤 바 검사
		//	if(pScrollBar == (CScrollBar*)&m_ctrVScroll)
		{
			SCROLLINFO  scrinfo;
			// 스크롤바 정보를 가져온다.
			if(pScrollBar->GetScrollInfo(&scrinfo))
			{
				switch(nSBCode)
				{
				case SB_PAGEUP:   // 스크롤 바의 위쪽 바를 클릭
					scrinfo.nPos -= scrinfo.nPage;
					break;
				case SB_PAGEDOWN:  // 스크롤 바의 아래쪽 바를 클릭
					scrinfo.nPos += scrinfo.nPage;
					break;
				case SB_LINEUP:   // 스크롤 바의 위쪽 화살표를 클릭
					scrinfo.nPos -= scrinfo.nPage/10;
					break;
				case SB_LINEDOWN:  // 스크롤 바의 아래쪽 화살표를 클릭
					scrinfo.nPos += scrinfo.nPage/10;
					break;
				case SB_THUMBPOSITION: // 스크롤바의 트랙이 움직이고 나서
				case SB_THUMBTRACK:  // 스크롤바의 트랙이 움직이는 동안
					scrinfo.nPos = scrinfo.nTrackPos;   // 16bit값 이상을 사용

					break;
				}

				// 스크롤바의 위치를 변경한다.
				pScrollBar->SetScrollPos(scrinfo.nPos);
			}

		}
	}

	else

	{

		// 이 부분은 Scroll기능이 있는 뷰를 사용시 사용된다.
		int ScrollPos = GetScrollPos(SB_VERT); // 세로 스크롤바 포지션 구하기
		// 가로는: SB_HORZ

		switch(nSBCode)
		{
		case SB_PAGEUP:        // 스크롤 바의 위쪽 바를 클릭
		case SB_PAGEDOWN:  // 스크롤 바의 아래쪽 바를 클릭
		case SB_LINEUP:         // 스크롤 바의 위쪽 화살표를 클릭
		case SB_LINEDOWN:    // 스크롤 바의 아래쪽 화살표를 클릭
			CDialog::OnVScroll(nSBCode, nPos, pScrollBar);
			break;
		case SB_THUMBPOSITION:  // 스크롤바의 트랙이 움직이고 나서
		case SB_THUMBTRACK:      // 스크롤바의 트랙이 움직이는 동안

			scrinfo.nPos = scrinfo.nTrackPos;
			// 스크롤바의 위치를 변경한다.
			SetScrollPos(SB_VERT, scrinfo.nPos);
			Invalidate(FALSE);
			break;
		}

	}



	//CDialogEx::OnVScroll(nSBCode, nPos, pScrollBar);
}


void CSamsung_TapelessAGVDlg::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.

	CDialogEx::OnLButtonDown(nFlags, point);
}


void CSamsung_TapelessAGVDlg::OnLvnItemchangedListInfo(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMLISTVIEW pNMLV = reinterpret_cast<LPNMLISTVIEW>(pNMHDR);
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	if(pNMLV->uOldState==0)
	{
		//m_nitemID=pNMLV->iItem;
		m_nitemID=PathBlockIDNum[pNMLV->iItem];
		printf("item ID:%d\n",m_nitemID);
		m_KUNSUI3DDlg.setPathBlockID(m_nitemID);
	}
	*pResult = 0;
}


void CSamsung_TapelessAGVDlg::OnGuiinformationSetpathblock()
{
	printf("set Path Block\n");

	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_GUIINFORMATION_SETPATHBLOCK, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETPATHBLOCK, MF_UNCHECKED);
		m_KUNSUI3DDlg.setPathBlockFlag( false );
		m_bSetRobotPos=false;

	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETPATHBLOCK, MF_CHECKED);
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETROBOTPOSE, MF_UNCHECKED);
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETGOALPOSE, MF_UNCHECKED);
		m_KUNSUI3DDlg.setPathBlockFlag( true );
		m_KUNSUI3DDlg.setRobotPosFlag( false );
		m_KUNSUI3DDlg.setGoalPosFlag( false );
		m_bSetRobotPos=true;

	} 	
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
}


void CSamsung_TapelessAGVDlg::OnPathblockGeneratepath()
{
	vector<PBlock> PBPack;
	KuDrawingInfo::getInstance()->getPathBlockPos(&PBPack);

	KuPathBlockPr PB;

	if(KuRobotParameter::getInstance()->getUsingISSAC() == 1) // ISSAC을 사용하는 경우
	{
		PB.setPathBlockPosForISSAC(PBPack);
	}
	else	
	{
		PB.setPathBlockPos(PBPack);
	}
	vector<list<KuPose>>Pathlist=PB.getPathlist();

	KuDrawingInfo::getInstance()->setvecPathlist(Pathlist);
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
}


void CSamsung_TapelessAGVDlg::OnPathblockClearblo()
{

	vector<PBlock> PBPack;
	PBPack.clear();
	KuDrawingInfo::getInstance()->setPathBlockPos(PBPack);
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
}

void CSamsung_TapelessAGVDlg::OnContextMenu(CWnd* /*pWnd*/, CPoint point)
{

	double dMPointX=0.0;
	double dMPointY=0.0;

	m_KUNSUI3DDlg.getMousePointonUI(&dMPointX,&dMPointY);

	if(m_bsettingflag==true)
	{

		vector<PBlock> vecPathBlock;
		KuDrawingInfo::getInstance()->getPathBlockPos(&vecPathBlock);

		int SelID=-1;

		for(int i=0;i<vecPathBlock.size();i++)
		{
			if((vecPathBlock[i].x-vecPathBlock[i].sizex/2.0)<=dMPointX
				&&(vecPathBlock[i].x+vecPathBlock[i].sizex/2.0)>=dMPointX
				&&(vecPathBlock[i].y-vecPathBlock[i].sizey/2.0)<=dMPointY
				&&(vecPathBlock[i].y+vecPathBlock[i].sizey/2.0)>=dMPointY)
			{
				SelID=i;
				break;
			}				
		}

		if(m_bchangpathblockflag==true)
		{
			m_bchangpathblockflag=false;
			if(m_SelID!=-1)
			{
				vecPathBlock[m_SelID].waypoint=changedPathBlock.waypoint;
				vecPathBlock[m_SelID].check=changedPathBlock.check;
				vecPathBlock[m_SelID].detectObstacle=changedPathBlock.detectObstacle;
				vecPathBlock[m_SelID].velocity=changedPathBlock.velocity;
				vecPathBlock[m_SelID].nRouteOrder=changedPathBlock.nRouteOrder;
				changedPathBlock.TaskList.clear();
				vecPathBlock[m_SelID].TaskList.clear();
				m_PSettingDlg.getBehavior(&changedPathBlock.TaskList);
				for(int i=0; i<changedPathBlock.TaskList.size();i++)
				{
					vecPathBlock[m_SelID].TaskList.push_back(changedPathBlock.TaskList[i]);
				}
				KuDrawingInfo::getInstance()->setPathBlockPos(vecPathBlock);
				m_SelID=-1;
			}
		}


		if(m_bchangpathblockflag==false&&SelID!=-1)
		{
			m_SelID=SelID;
			for(int i=0; i<4;i++){m_bsetWayPoint[i]=false;}
			if(vecPathBlock[SelID].waypoint>=0){m_bsetWayPoint[vecPathBlock[SelID].waypoint]=true;}
			for(int i=0; i<2;i++){m_bsetCheckPoint[i]=false;}
			m_bsetCheckPoint[vecPathBlock[SelID].check]=true;
			for(int i=0; i<2;i++){m_bsetObstacleDetection[i]=false;}
			m_bsetObstacleDetection[vecPathBlock[SelID].detectObstacle]=true;
			for(int i=0; i<6;i++){m_bsetVelocity[i]=false;}
			if(vecPathBlock[SelID].velocity>=0){m_bsetVelocity[vecPathBlock[SelID].velocity]=true;}
			for(int i=0; i<11;i++){m_bsetViaPoint[i]=false;}
			if(vecPathBlock[SelID].nRouteOrder>=0){m_bsetViaPoint[vecPathBlock[SelID].nRouteOrder]=true;}

			changedPathBlock.waypoint=vecPathBlock[SelID].waypoint;
			changedPathBlock.check=vecPathBlock[SelID].check;
			changedPathBlock.velocity=vecPathBlock[SelID].velocity;
			changedPathBlock.nRouteOrder=vecPathBlock[SelID].nRouteOrder;

			changedPathBlock.TaskList.clear();
			for(int i=0; i<vecPathBlock[m_SelID].TaskList.size();i++)
			{
				changedPathBlock.TaskList.push_back(vecPathBlock[m_SelID].TaskList[i]);
			}
		}

		// TODO: 여기에 메시지 처리기 코드를 추가합니다.
		CMenu muiTemp, *pContextMenu;
		muiTemp.LoadMenuW(IDR_SETTING_MENU);
		pContextMenu=muiTemp.GetSubMenu(0);
		CMenu *pSubm=pContextMenu->GetSubMenu(0);
		if(pContextMenu==NULL)
			return;

		for(int i=0; i<4;i++)
		{
			if(m_bsetWayPoint[i]==false){ pContextMenu->CheckMenuItem(ID_POINT_DEFAULT+i,MF_UNCHECKED|MF_BYCOMMAND );}
			else {pContextMenu->CheckMenuItem(ID_POINT_DEFAULT+i,MF_CHECKED|MF_BYCOMMAND );}
		}

		for(int i=0; i<2;i++)
		{
			if(m_bsetCheckPoint[i]==false){ pContextMenu->CheckMenuItem(ID_CHECKPOINT_DEFAULT+i,MF_UNCHECKED|MF_BYCOMMAND );}
			else {pContextMenu->CheckMenuItem(ID_CHECKPOINT_DEFAULT+i,MF_CHECKED|MF_BYCOMMAND );}
		}

		for(int i=0; i<2;i++)
		{
			if(m_bsetObstacleDetection[i]==false){ pContextMenu->CheckMenuItem(ID_OBSTACLE_DEFAULT+i,MF_UNCHECKED|MF_BYCOMMAND );}
			else {pContextMenu->CheckMenuItem(ID_OBSTACLE_DEFAULT+i,MF_CHECKED|MF_BYCOMMAND );}
		}

		for(int i=0; i<6;i++)
		{
			if(m_bsetVelocity[i]==false){ pContextMenu->CheckMenuItem(ID_VELOCITY_WHITE+i,MF_UNCHECKED|MF_BYCOMMAND );	}
			else {	pContextMenu->CheckMenuItem(ID_VELOCITY_WHITE+i,MF_CHECKED|MF_BYCOMMAND );	}
		}

		for(int i=0; i<11;i++)
		{
			if(m_bsetViaPoint[i]==false){ pContextMenu->CheckMenuItem(ID_VIA_DEFALT+i,MF_UNCHECKED|MF_BYCOMMAND );	}
			else {	pContextMenu->CheckMenuItem(ID_VIA_DEFALT+i,MF_CHECKED|MF_BYCOMMAND );	}
		}


		// 		if(m_bsetMotionPause==true){pContextMenue->CheckMenuItem(ID_MOTION_PAUSE,MF_CHECKED|MF_BYCOMMAND );	}
		// 		else {	pContextMenue->CheckMenuItem(ID_MOTION_PAUSE,MF_UNCHECKED|MF_BYCOMMAND );}
		// 
		// 		if(m_bsetMotionResume==true){pContextMenue->CheckMenuItem(ID_MOTION_RESUME,MF_CHECKED|MF_BYCOMMAND );}
		// 		else{pContextMenue->CheckMenuItem(ID_MOTION_RESUME,MF_UNCHECKED|MF_BYCOMMAND );	}
		// 
		// 		for(int i=0; i<4;i++)
		// 		{
		// 			if(m_bsetMotionRotaion[i]==false){ pContextMenue->CheckMenuItem(ID_ROTATE_LEFT90+i,MF_UNCHECKED|MF_BYCOMMAND );}
		// 			else {pContextMenue->CheckMenuItem(ID_ROTATE_LEFT90+i,MF_CHECKED|MF_BYCOMMAND );}
		// 		}
		// 
		// 		for(int i=0; i<3;i++)
		// 		{
		// 			if(m_bsetBehaviorSound[i]==false){ pContextMenue->CheckMenuItem(ID_SOUND_SONUD1+i,MF_UNCHECKED|MF_BYCOMMAND );}
		// 			else {pContextMenue->CheckMenuItem(ID_SOUND_SONUD1+i,MF_CHECKED|MF_BYCOMMAND );}
		// 		}
		// 
		// 		for(int i=0; i<3;i++)
		// 		{
		// 			if(m_bsetBehaviorDevice[i]==false){ pContextMenue->CheckMenuItem(ID_DEVICE_DEVICE1+i,MF_UNCHECKED|MF_BYCOMMAND );}
		// 			else {pContextMenue->CheckMenuItem(ID_DEVICE_DEVICE1+i,MF_CHECKED|MF_BYCOMMAND );}
		// 		}
		// 
		// 		if(m_bsetBehaviorTowerLamp==true){pContextMenue->CheckMenuItem(ID_BEHAVIOR_TOWERLAMP,MF_CHECKED|MF_BYCOMMAND );}
		// 		else{pContextMenue->CheckMenuItem(ID_BEHAVIOR_TOWERLAMP,MF_UNCHECKED|MF_BYCOMMAND );	}
		// 
		// 		if(m_bsetBehaviorReadytoSignal==true){pContextMenue->CheckMenuItem(ID_BEHAVIOR_READYTOSIGNAL,MF_CHECKED|MF_BYCOMMAND );}
		// 		else{pContextMenue->CheckMenuItem(ID_BEHAVIOR_READYTOSIGNAL,MF_UNCHECKED|MF_BYCOMMAND );	}
		// 
		pContextMenu->TrackPopupMenu(TPM_LEFTALIGN|TPM_LEFTBUTTON,point.x,point.y,this);
	}

}



void CSamsung_TapelessAGVDlg::OnVelocityWhite()
{
	m_bchangpathblockflag=true;
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	if(m_bsetVelocity[0]==false)
	{
		for(int i=0; i<6;i++){ m_bsetVelocity[i]=false;}
		m_bsetVelocity[0]=true;
	}
	else
	{
		m_bsetVelocity[0]=false;
	}
	changedPathBlock.velocity=0;
}

void CSamsung_TapelessAGVDlg::OnVelocityRed()
{
	m_bchangpathblockflag=true;
	if(m_bsetVelocity[1]==false)
	{
		for(int i=0; i<6;i++){ m_bsetVelocity[i]=false;}
		m_bsetVelocity[1]=true;
	}
	else
	{
		m_bsetVelocity[1]=false;
	}
	changedPathBlock.velocity=1;

}


void CSamsung_TapelessAGVDlg::OnVelocityYellow()
{

	m_bchangpathblockflag=true;
	if(m_bsetVelocity[2]==false)
	{
		for(int i=0; i<6;i++){ m_bsetVelocity[i]=false;}
		m_bsetVelocity[2]=true;
	}
	else
	{
		m_bsetVelocity[2]=false;
	}
	changedPathBlock.velocity=2;
}


void CSamsung_TapelessAGVDlg::OnVelocityGreen()
{
	m_bchangpathblockflag=true;
	if(m_bsetVelocity[3]==false)
	{
		for(int i=0; i<6;i++){ m_bsetVelocity[i]=false;}
		m_bsetVelocity[3]=true;
	}
	else
	{
		m_bsetVelocity[3]=false;
	}
	changedPathBlock.velocity=3;
}


void CSamsung_TapelessAGVDlg::OnVelocityBlue()
{
	m_bchangpathblockflag=true;
	if(m_bsetVelocity[4]==false)
	{
		for(int i=0; i<6;i++){ m_bsetVelocity[i]=false;}
		m_bsetVelocity[4]=true;
	}
	else
	{
		m_bsetVelocity[4]=false;
	}
	changedPathBlock.velocity=4;

}



void CSamsung_TapelessAGVDlg::OnVelocityPurple()
{
	m_bchangpathblockflag=true;
	if(m_bsetVelocity[5]==false)
	{
		for(int i=0; i<6;i++){ m_bsetVelocity[i]=false;}
		m_bsetVelocity[5]=true;
	}
	else
	{
		m_bsetVelocity[5]=false;
	}
	changedPathBlock.velocity=5;
}


void CSamsung_TapelessAGVDlg::OnMotionPause()
{
	m_bchangpathblockflag=true;
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	if(m_bsetMotionPause==false)
	{
		m_bsetMotionPause=true;
	}
	else
	{
		m_bsetMotionPause=false;
	}
	//changedPathBlock.Motion.Pause=m_bsetMotionPause;
}


void CSamsung_TapelessAGVDlg::OnMotionResume()
{
	m_bchangpathblockflag=true;
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	if(m_bsetMotionResume==false)
	{
		m_bsetMotionResume=true;
	}
	else
	{
		m_bsetMotionResume=false;
	}
	//changedPathBlock.Motion.Pause=m_bsetMotionResume;

}


void CSamsung_TapelessAGVDlg::OnRotateLeft90()
{
	m_bchangpathblockflag=true;
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	if(m_bsetMotionRotaion[0]==false)
	{
		for(int i=0; i<4;i++){ m_bsetMotionRotaion[i]=false;}
		m_bsetMotionRotaion[0]=true;
	}
	else
	{
		m_bsetMotionRotaion[0]=false;
	}
	//changedPathBlock.Motion.Rotaion=0;
}


void CSamsung_TapelessAGVDlg::OnRotateRight90()
{
	m_bchangpathblockflag=true;
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	if(m_bsetMotionRotaion[1]==false)
	{
		for(int i=0; i<4;i++){ m_bsetMotionRotaion[i]=false;}
		m_bsetMotionRotaion[1]=true;
	}
	else
	{
		m_bsetMotionRotaion[1]=false;
	}
	//changedPathBlock.Motion.Rotaion=1;

}


void CSamsung_TapelessAGVDlg::OnRotateLeft180()
{
	m_bchangpathblockflag=true;
	if(m_bsetMotionRotaion[2]==false)
	{
		for(int i=0; i<4;i++){ m_bsetMotionRotaion[i]=false;}
		m_bsetMotionRotaion[2]=true;
	}
	else
	{
		m_bsetMotionRotaion[2]=false;
	}
	//changedPathBlock.Motion.Rotaion=2;

}


void CSamsung_TapelessAGVDlg::OnRotateRight180()
{
	m_bchangpathblockflag=true;
	if(m_bsetMotionRotaion[3]==false)
	{
		for(int i=0; i<4;i++){ m_bsetMotionRotaion[i]=false;}
		m_bsetMotionRotaion[3]=true;
	}
	else
	{
		m_bsetMotionRotaion[3]=false;
	}
	//	changedPathBlock.Motion.Rotaion=3;

}



void CSamsung_TapelessAGVDlg::OnDeviceDevice1()
{
	m_bchangpathblockflag=true;
	if(m_bsetBehaviorDevice[0]==false)
	{
		for(int i=0; i<3;i++){ m_bsetBehaviorDevice[i]=false;}
		m_bsetBehaviorDevice[0]=true;
	}
	else
	{
		m_bsetBehaviorDevice[0]=false;
	}
	//changedPathBlock.Behavior.Device=0;

}


void CSamsung_TapelessAGVDlg::OnDeviceDevice2()
{
	m_bchangpathblockflag=true;
	if(m_bsetBehaviorDevice[1]==false)
	{
		for(int i=0; i<3;i++){ m_bsetBehaviorDevice[i]=false;}
		m_bsetBehaviorDevice[1]=true;
	}
	else
	{
		m_bsetBehaviorDevice[1]=false;
	}
	//changedPathBlock.Behavior.Device=1;
}


void CSamsung_TapelessAGVDlg::OnDeviceDevice3()
{
	m_bchangpathblockflag=true;
	if(m_bsetBehaviorDevice[2]==false)
	{
		for(int i=0; i<3;i++){ m_bsetBehaviorDevice[i]=false;}
		m_bsetBehaviorDevice[2]=true;
	}
	else
	{
		m_bsetBehaviorDevice[2]=false;
	}
	//changedPathBlock.Behavior.Device=2;
}


void CSamsung_TapelessAGVDlg::OnBehaviorReadytosignal()
{
	m_bchangpathblockflag=true;
	if(m_bsetBehaviorReadytoSignal==false)
	{
		m_bsetBehaviorReadytoSignal=true;
	}
	else
	{
		m_bsetBehaviorReadytoSignal=false;
	}
	//changedPathBlock.Behavior.ReadytoSignal=m_bsetBehaviorReadytoSignal;
}


void CSamsung_TapelessAGVDlg::OnBehaviorTowerlamp()
{
	m_bchangpathblockflag=true;
	if(m_bsetBehaviorTowerLamp==false)
	{
		m_bsetBehaviorTowerLamp=true;
	}
	else
	{
		m_bsetBehaviorTowerLamp=false;
	}
	//changedPathBlock.Behavior.TowerLamp=m_bsetBehaviorTowerLamp;
}


void CSamsung_TapelessAGVDlg::OnSoundSonud1()
{
	m_bchangpathblockflag=true;
	if(m_bsetBehaviorSound[0]==false)
	{
		for(int i=0; i<3;i++){ m_bsetBehaviorSound[i]=false;}
		m_bsetBehaviorSound[0]=true;
	}
	else
	{
		m_bsetBehaviorSound[0]=false;
	}
	//changedPathBlock.Behavior.Sound=0;
}


void CSamsung_TapelessAGVDlg::OnSoundSonud2()
{
	m_bchangpathblockflag=true;
	if(m_bsetBehaviorSound[1]==false)
	{
		for(int i=0; i<3;i++){ m_bsetBehaviorSound[i]=false;}
		m_bsetBehaviorSound[1]=true;
	}
	else
	{
		m_bsetBehaviorSound[1]=false;
	}
	//changedPathBlock.Behavior.Sound=1;
}


void CSamsung_TapelessAGVDlg::OnSoundSonud3()
{
	m_bchangpathblockflag=true;
	if(m_bsetBehaviorSound[2]==false)
	{
		for(int i=0; i<3;i++){ m_bsetBehaviorSound[i]=false;}
		m_bsetBehaviorSound[2]=true;
	}
	else
	{
		m_bsetBehaviorSound[2]=false;
	}
	//changedPathBlock.Behavior.Sound=2;
}


void CSamsung_TapelessAGVDlg::OnPathblockSetting()
{

	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_PATHBLOCK_SETTING, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		m_bsettingflag=false;
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_PATHBLOCK_SETTING, MF_UNCHECKED);
		m_KUNSUI3DDlg.setPathBlocksettingFlag(false);
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		m_bsettingflag=true;
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_PATHBLOCK_SETTING, MF_CHECKED);
		m_KUNSUI3DDlg.setPathBlocksettingFlag(true);

	} 	

	// TODO: 여기에 명령 처리기 코드를 추가합니다.
}


void CSamsung_TapelessAGVDlg::OnPointDefault()
{
	m_bchangpathblockflag=true;
	for(int i=0; i<4;i++)	m_bsetWayPoint[i]=false;
	m_bsetWayPoint[0]=true;
	changedPathBlock.waypoint=0;
}

void CSamsung_TapelessAGVDlg::OnPointWaypoint()
{
	m_bchangpathblockflag=true;
	for(int i=0; i<4;i++)	m_bsetWayPoint[i]=false;
	m_bsetWayPoint[1]=true;
	changedPathBlock.waypoint=1;
}


void CSamsung_TapelessAGVDlg::OnPointStartpoint()
{
	m_bchangpathblockflag=true;
	for(int i=0; i<4;i++)	m_bsetWayPoint[i]=false;
	m_bsetWayPoint[2]=true;
	changedPathBlock.waypoint=2;
}


void CSamsung_TapelessAGVDlg::OnPointEndpoint()
{
	m_bchangpathblockflag=true;
	for(int i=0; i<4;i++)	m_bsetWayPoint[i]=false;
	m_bsetWayPoint[3]=true;
	changedPathBlock.waypoint=3;
}


void CSamsung_TapelessAGVDlg::OnPathblockSavepathblock()
{
	string strDataPath =KuRobotParameter::getInstance()->getTeachingPathNameNPath();
	string strNewPath;
	char cFilePathName[300];
	memset(cFilePathName,0,sizeof(cFilePathName));
	sprintf(cFilePathName,"%s/Pathblock.txt",strDataPath.c_str());
	strNewPath=cFilePathName;//char 포인터값을 string에 대입
	vector<PBlock> PBPack;
	KuDrawingInfo::getInstance()->getPathBlockPos(&PBPack);
	KuPathBlockPr PB;
	PB.savePathBlock(PBPack,strNewPath);
}



void CSamsung_TapelessAGVDlg::OnPathblockLoadpathblock()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_PATHBLOCK_LOADPATHBLOCK, MF_BYCOMMAND); 
	if (nMenuState == MF_UNCHECKED){ //메뉴가 체크되지 않은 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_PATHBLOCK_LOADPATHBLOCK, MF_CHECKED);
		string strDataPath =KuRobotParameter::getInstance()->getTeachingPathNameNPath();
		string strNewPath;
		char cFilePathName[300];
		memset(cFilePathName,0,sizeof(cFilePathName));
		sprintf(cFilePathName,"%s/Pathblock.txt",strDataPath.c_str());
		strNewPath=cFilePathName;//char 포인터값을 string에 대입
		KuPathBlockPr PB;
		vector<PBlock> PBPack=PB.loadPathBlock(strNewPath);
		KuDrawingInfo::getInstance()->setPathBlockPos(PBPack);
	}
	else{ //메뉴가 체크 된 상태이기 때문에 체크표시 제거 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_PATHBLOCK_LOADPATHBLOCK, MF_UNCHECKED);	
		vector<PBlock> PBPack;
		PBPack.clear();
		KuDrawingInfo::getInstance()->setPathBlockPos(PBPack);

	}
}


void CSamsung_TapelessAGVDlg::OnSettingTask()
{
	m_bchangpathblockflag=true;
	m_PSettingDlg.setBehavior(changedPathBlock.TaskList);
	m_PSettingDlg.ShowWindow(SW_SHOW);
}


void CSamsung_TapelessAGVDlg::OnViaDefalt()
{
	m_bchangpathblockflag=true;
	for(int i=0; i<11;i++)	m_bsetViaPoint[i]=false;
	m_bsetViaPoint[0]=true;
	changedPathBlock.nRouteOrder=0;
}


void CSamsung_TapelessAGVDlg::OnVia1()
{
	m_bchangpathblockflag=true;
	for(int i=0; i<11;i++)	m_bsetViaPoint[i]=false;
	m_bsetViaPoint[1]=true;
	changedPathBlock.nRouteOrder=1;
}


void CSamsung_TapelessAGVDlg::OnVia2()
{
	m_bchangpathblockflag=true;
	for(int i=0; i<11;i++)	m_bsetViaPoint[i]=false;
	m_bsetViaPoint[2]=true;
	changedPathBlock.nRouteOrder=2;
}


void CSamsung_TapelessAGVDlg::OnVia3()
{
	m_bchangpathblockflag=true;
	for(int i=0; i<11;i++)	m_bsetViaPoint[i]=false;
	m_bsetViaPoint[3]=true;
	changedPathBlock.nRouteOrder=3;
}


void CSamsung_TapelessAGVDlg::OnVia4()
{
	m_bchangpathblockflag=true;
	for(int i=0; i<11;i++)	m_bsetViaPoint[i]=false;
	m_bsetViaPoint[4]=true;
	changedPathBlock.nRouteOrder=4;
}


void CSamsung_TapelessAGVDlg::OnVia5()
{
	m_bchangpathblockflag=true;
	for(int i=0; i<11;i++)	m_bsetViaPoint[i]=false;
	m_bsetViaPoint[5]=true;
	changedPathBlock.nRouteOrder=5;
}


void CSamsung_TapelessAGVDlg::OnVia6()
{
	m_bchangpathblockflag=true;
	for(int i=0; i<11;i++)	m_bsetViaPoint[i]=false;
	m_bsetViaPoint[6]=true;
	changedPathBlock.nRouteOrder=6;
}


void CSamsung_TapelessAGVDlg::OnVia7()
{
	m_bchangpathblockflag=true;
	for(int i=0; i<11;i++)	m_bsetViaPoint[i]=false;
	m_bsetViaPoint[7]=true;
	changedPathBlock.nRouteOrder=7;
}


void CSamsung_TapelessAGVDlg::OnVia8()
{
	m_bchangpathblockflag=true;
	for(int i=0; i<11;i++)	m_bsetViaPoint[i]=false;
	m_bsetViaPoint[8]=true;
	changedPathBlock.nRouteOrder=8;
}


void CSamsung_TapelessAGVDlg::OnVia9()
{
	m_bchangpathblockflag=true;
	for(int i=0; i<11;i++)	m_bsetViaPoint[i]=false;
	m_bsetViaPoint[9]=true;
	changedPathBlock.nRouteOrder=9;
}


void CSamsung_TapelessAGVDlg::OnVia10()
{
	m_bchangpathblockflag=true;
	for(int i=0; i<11;i++)	m_bsetViaPoint[i]=false;
	m_bsetViaPoint[10]=true;
	changedPathBlock.nRouteOrder=10;
}


void CSamsung_TapelessAGVDlg::OnSavepathDefault()
{
	CMenu* pMainMenu = GetMenu();

	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SAVEPATH_PATH1, MF_UNCHECKED);
	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SAVEPATH_PATH2, MF_UNCHECKED);
	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SAVEPATH_PATH3, MF_UNCHECKED);
	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SAVEPATH_PATH4, MF_UNCHECKED);
	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SAVEPATH_PATH5, MF_UNCHECKED);
	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SAVEPATH_PATH6, MF_UNCHECKED);
	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SAVEPATH_PATH7, MF_UNCHECKED);
	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SAVEPATH_PATH8, MF_UNCHECKED);
	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SAVEPATH_PATH9, MF_UNCHECKED);
	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SAVEPATH_PATH10, MF_UNCHECKED);

	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH1, MF_UNCHECKED);
	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH2, MF_UNCHECKED);
	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH3, MF_UNCHECKED);
	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH4, MF_UNCHECKED);
	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH5, MF_UNCHECKED);
	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH6, MF_UNCHECKED);
	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH7, MF_UNCHECKED);
	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH8, MF_UNCHECKED);
	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH9, MF_UNCHECKED);
	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH10, MF_UNCHECKED);
	m_bexecuteflag=false;

	KuDrawingInfo::getInstance()->clearGlobalPathBlockPos();


}


void CSamsung_TapelessAGVDlg::OnSavepathPath1()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_SAVEPATH_PATH1 , MF_BYCOMMAND); 
	if (nMenuState == MF_UNCHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SAVEPATH_PATH1, MF_CHECKED);
		m_bexecuteflag=true;
		char cFilePathName[300];
		string strDataPath;int nIdx =1;
		sprintf(cFilePathName,"./data/path/LoadPathBlock%d.txt",nIdx);
		strDataPath=cFilePathName;//char 포인터값을 string에 대입
		vector<PBlock> PBPose;
		KuDrawingInfo::getInstance()->getPathBlockPos(&PBPose);
		KuPathBlockPr PB;
		PB.savePathBlock(PBPose,strDataPath);
	} 
}


void CSamsung_TapelessAGVDlg::OnSavepathPath2()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_SAVEPATH_PATH2 , MF_BYCOMMAND); 
	if (nMenuState == MF_UNCHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SAVEPATH_PATH2, MF_CHECKED);
		m_bexecuteflag=true;
		char cFilePathName[300];
		string strDataPath;int nIdx =2;
		sprintf(cFilePathName,"./data/path/LoadPathBlock%d.txt",nIdx);
		strDataPath=cFilePathName;//char 포인터값을 string에 대입
		vector<PBlock> PBPose;
		KuDrawingInfo::getInstance()->getPathBlockPos(&PBPose);
		KuPathBlockPr PB;
		PB.savePathBlock(PBPose,strDataPath);
	} 
}


void CSamsung_TapelessAGVDlg::OnSavepathPath3()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_SAVEPATH_PATH3 , MF_BYCOMMAND); 
	if (nMenuState == MF_UNCHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SAVEPATH_PATH3, MF_CHECKED);
		m_bexecuteflag=true;
		char cFilePathName[300];
		string strDataPath;int nIdx =3;
		sprintf(cFilePathName,"./data/path/LoadPathBlock%d.txt",nIdx);
		strDataPath=cFilePathName;//char 포인터값을 string에 대입
		vector<PBlock> PBPose;
		KuDrawingInfo::getInstance()->getPathBlockPos(&PBPose);
		KuPathBlockPr PB;
		PB.savePathBlock(PBPose,strDataPath);
	} 
}


void CSamsung_TapelessAGVDlg::OnSavepathPath4()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_SAVEPATH_PATH4 , MF_BYCOMMAND); 
	if (nMenuState == MF_UNCHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SAVEPATH_PATH4, MF_CHECKED);
		m_bexecuteflag=true;
		char cFilePathName[300];
		string strDataPath;int nIdx =4;
		sprintf(cFilePathName,"./data/path/LoadPathBlock%d.txt",nIdx);
		strDataPath=cFilePathName;//char 포인터값을 string에 대입
		vector<PBlock> PBPose;
		KuDrawingInfo::getInstance()->getPathBlockPos(&PBPose);
		KuPathBlockPr PB;
		PB.savePathBlock(PBPose,strDataPath);
	} 
}


void CSamsung_TapelessAGVDlg::OnSavepathPath5()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_SAVEPATH_PATH5 , MF_BYCOMMAND); 
	if (nMenuState == MF_UNCHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SAVEPATH_PATH5, MF_CHECKED);
		m_bexecuteflag=true;
		char cFilePathName[300];
		string strDataPath;int nIdx =5;
		sprintf(cFilePathName,"./data/path/LoadPathBlock%d.txt",nIdx);
		strDataPath=cFilePathName;//char 포인터값을 string에 대입
		vector<PBlock> PBPose;
		KuDrawingInfo::getInstance()->getPathBlockPos(&PBPose);
		KuPathBlockPr PB;
		PB.savePathBlock(PBPose,strDataPath);
	} 
}


void CSamsung_TapelessAGVDlg::OnSavepathPath6()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_SAVEPATH_PATH6 , MF_BYCOMMAND); 
	if (nMenuState == MF_UNCHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SAVEPATH_PATH6, MF_CHECKED);
		m_bexecuteflag=true;
		char cFilePathName[300];
		string strDataPath;int nIdx =6;
		sprintf(cFilePathName,"./data/path/LoadPathBlock%d.txt",nIdx);
		strDataPath=cFilePathName;//char 포인터값을 string에 대입
		vector<PBlock> PBPose;
		KuDrawingInfo::getInstance()->getPathBlockPos(&PBPose);
		KuPathBlockPr PB;
		PB.savePathBlock(PBPose,strDataPath);
	} 
}


void CSamsung_TapelessAGVDlg::OnSavepathPath7()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_SAVEPATH_PATH7 , MF_BYCOMMAND); 
	if (nMenuState == MF_UNCHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SAVEPATH_PATH7, MF_CHECKED);
		m_bexecuteflag=true;
		char cFilePathName[300];
		string strDataPath;int nIdx =7;
		sprintf(cFilePathName,"./data/path/LoadPathBlock%d.txt",nIdx);
		strDataPath=cFilePathName;//char 포인터값을 string에 대입
		vector<PBlock> PBPose;
		KuDrawingInfo::getInstance()->getPathBlockPos(&PBPose);
		KuPathBlockPr PB;
		PB.savePathBlock(PBPose,strDataPath);
	} 
}


void CSamsung_TapelessAGVDlg::OnSavepathPath8()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_SAVEPATH_PATH8 , MF_BYCOMMAND); 
	if (nMenuState == MF_UNCHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SAVEPATH_PATH8, MF_CHECKED);
		m_bexecuteflag=true;
		char cFilePathName[300];
		string strDataPath;int nIdx =8;
		sprintf(cFilePathName,"./data/path/LoadPathBlock%d.txt",nIdx);
		strDataPath=cFilePathName;//char 포인터값을 string에 대입
		vector<PBlock> PBPose;
		KuDrawingInfo::getInstance()->getPathBlockPos(&PBPose);
		KuPathBlockPr PB;
		PB.savePathBlock(PBPose,strDataPath);
	} 
}


void CSamsung_TapelessAGVDlg::OnSavepathPath9()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_SAVEPATH_PATH9 , MF_BYCOMMAND); 
	if (nMenuState == MF_UNCHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SAVEPATH_PATH9, MF_CHECKED);
		m_bexecuteflag=true;
		char cFilePathName[300];
		string strDataPath;int nIdx =9;
		sprintf(cFilePathName,"./data/path/LoadPathBlock%d.txt",nIdx);
		strDataPath=cFilePathName;//char 포인터값을 string에 대입
		vector<PBlock> PBPose;
		KuDrawingInfo::getInstance()->getPathBlockPos(&PBPose);
		KuPathBlockPr PB;
		PB.savePathBlock(PBPose,strDataPath);
	} 
}


void CSamsung_TapelessAGVDlg::OnSavepathPath10()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_SAVEPATH_PATH10 , MF_BYCOMMAND); 
	if (nMenuState == MF_UNCHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SAVEPATH_PATH10, MF_CHECKED);
		m_bexecuteflag=true;
		char cFilePathName[300];
		string strDataPath;int nIdx =10;
		sprintf(cFilePathName,"./data/path/LoadPathBlock%d.txt",nIdx);
		strDataPath=cFilePathName;//char 포인터값을 string에 대입
		vector<PBlock> PBPose;
		KuDrawingInfo::getInstance()->getPathBlockPos(&PBPose);
		KuPathBlockPr PB;
		PB.savePathBlock(PBPose,strDataPath);
	} 
}


void CSamsung_TapelessAGVDlg::OnLoadpathPath1()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_LOADPATH_PATH1 , MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH1, MF_UNCHECKED);
		m_bexecuteflag=false;
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH1, MF_CHECKED);
		m_bexecuteflag=true;
		char cFilePathName[300];
		string strDataPath;int nIdx =1;
		sprintf(cFilePathName,"./data/path/LoadPathBlock%d.txt",nIdx);
		strDataPath=cFilePathName;//char 포인터값을 string에 대입
		vector<PBlock> PBPack;
		KuPathBlockPr PB;
		PBPack=PB.loadPathBlock(strDataPath);

		if(KuRobotParameter::getInstance()->getUsingISSAC() == 1) // ISSAC을 사용하는 경우
		{
			PB.setPathBlockPosForISSAC(PBPack);
		}
		else	
		{
			PB.setPathBlockPos(PBPack);
		}
		vector<list<KuPose>>Pathlist=PB.getPathlist();
		KuDrawingInfo::getInstance()->setPathBlockPos(PBPack);
		KuDrawingInfo::getInstance()->setvecPathlist(Pathlist);
	} 
}


void CSamsung_TapelessAGVDlg::OnLoadpathPath2()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_LOADPATH_PATH2 , MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH2, MF_UNCHECKED);
		m_bexecuteflag=false;
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH2, MF_CHECKED);
		m_bexecuteflag=true;
		char cFilePathName[300];
		string strDataPath;int nIdx =2;
		sprintf(cFilePathName,"./data/path/LoadPathBlock%d.txt",nIdx);
		strDataPath=cFilePathName;//char 포인터값을 string에 대입
		vector<PBlock> PBPack;
		KuPathBlockPr PB;
		PBPack=PB.loadPathBlock(strDataPath);
		if(KuRobotParameter::getInstance()->getUsingISSAC() == 1) // ISSAC을 사용하는 경우
		{
			PB.setPathBlockPosForISSAC(PBPack);
		}
		else	
		{
			PB.setPathBlockPos(PBPack);
		}
		vector<list<KuPose>>Pathlist=PB.getPathlist();
		KuDrawingInfo::getInstance()->setPathBlockPos(PBPack);
		KuDrawingInfo::getInstance()->setvecPathlist(Pathlist);
	} 
}


void CSamsung_TapelessAGVDlg::OnLoadpathPath3()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_LOADPATH_PATH3 , MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH3, MF_UNCHECKED);
		m_bexecuteflag=false;
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH3, MF_CHECKED);
		m_bexecuteflag=true;
		char cFilePathName[300];
		string strDataPath;int nIdx =3;
		sprintf(cFilePathName,"./data/path/LoadPathBlock%d.txt",nIdx);
		strDataPath=cFilePathName;//char 포인터값을 string에 대입
		vector<PBlock> PBPack;
		KuPathBlockPr PB;
		PBPack=PB.loadPathBlock(strDataPath);
		if(KuRobotParameter::getInstance()->getUsingISSAC() == 1) // ISSAC을 사용하는 경우
		{
			PB.setPathBlockPosForISSAC(PBPack);
		}
		else	
		{
			PB.setPathBlockPos(PBPack);
		}
		vector<list<KuPose>>Pathlist=PB.getPathlist();
		KuDrawingInfo::getInstance()->setPathBlockPos(PBPack);
		KuDrawingInfo::getInstance()->setvecPathlist(Pathlist);
	} 
}


void CSamsung_TapelessAGVDlg::OnLoadpathPath4()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_LOADPATH_PATH4 , MF_BYCOMMAND);
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH4, MF_UNCHECKED);
		m_bexecuteflag=false;
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH4, MF_CHECKED);
		m_bexecuteflag=true;
		char cFilePathName[300];
		string strDataPath;int nIdx =4;
		sprintf(cFilePathName,"./data/path/LoadPathBlock%d.txt",nIdx);
		strDataPath=cFilePathName;//char 포인터값을 string에 대입
		vector<PBlock> PBPack;
		KuPathBlockPr PB;
		PBPack=PB.loadPathBlock(strDataPath);
		if(KuRobotParameter::getInstance()->getUsingISSAC() == 1) // ISSAC을 사용하는 경우
		{
			PB.setPathBlockPosForISSAC(PBPack);
		}
		else	
		{
			PB.setPathBlockPos(PBPack);
		}
		vector<list<KuPose>>Pathlist=PB.getPathlist();
		KuDrawingInfo::getInstance()->setPathBlockPos(PBPack);
		KuDrawingInfo::getInstance()->setvecPathlist(Pathlist);
	}
}


void CSamsung_TapelessAGVDlg::OnLoadpathPath5()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_LOADPATH_PATH5 , MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH5, MF_UNCHECKED);
		m_bexecuteflag=false;
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH5, MF_CHECKED);
		m_bexecuteflag=true;
		char cFilePathName[300];
		string strDataPath;int nIdx =5;
		sprintf(cFilePathName,"./data/path/LoadPathBlock%d.txt",nIdx);
		strDataPath=cFilePathName;//char 포인터값을 string에 대입
		vector<PBlock> PBPack;
		KuPathBlockPr PB;
		PBPack=PB.loadPathBlock(strDataPath);
		if(KuRobotParameter::getInstance()->getUsingISSAC() == 1) // ISSAC을 사용하는 경우
		{
			PB.setPathBlockPosForISSAC(PBPack);
		}
		else	
		{
			PB.setPathBlockPos(PBPack);
		}
		vector<list<KuPose>>Pathlist=PB.getPathlist();
		KuDrawingInfo::getInstance()->setPathBlockPos(PBPack);
		KuDrawingInfo::getInstance()->setvecPathlist(Pathlist);
	} 
}


void CSamsung_TapelessAGVDlg::OnLoadpathPath6()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_LOADPATH_PATH6 , MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH6, MF_UNCHECKED);
		m_bexecuteflag=false;
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH6, MF_CHECKED);
		m_bexecuteflag=true;
		char cFilePathName[300];
		string strDataPath;int nIdx =6;
		sprintf(cFilePathName,"./data/path/LoadPathBlock%d.txt",nIdx);
		strDataPath=cFilePathName;//char 포인터값을 string에 대입
		vector<PBlock> PBPack;
		KuPathBlockPr PB;
		PBPack=PB.loadPathBlock(strDataPath);
		if(KuRobotParameter::getInstance()->getUsingISSAC() == 1) // ISSAC을 사용하는 경우
		{
			PB.setPathBlockPosForISSAC(PBPack);
		}
		else	
		{
			PB.setPathBlockPos(PBPack);
		}
		vector<list<KuPose>>Pathlist=PB.getPathlist();
		KuDrawingInfo::getInstance()->setPathBlockPos(PBPack);
		KuDrawingInfo::getInstance()->setvecPathlist(Pathlist);
	} 
}


void CSamsung_TapelessAGVDlg::OnLoadpathPath7()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_LOADPATH_PATH7 , MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH7, MF_UNCHECKED);
		m_bexecuteflag=false;
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH7, MF_CHECKED);
		m_bexecuteflag=true;
		char cFilePathName[300];
		string strDataPath;int nIdx =7;
		sprintf(cFilePathName,"./data/path/LoadPathBlock%d.txt",nIdx);
		strDataPath=cFilePathName;//char 포인터값을 string에 대입
		vector<PBlock> PBPack;
		KuPathBlockPr PB;
		PBPack=PB.loadPathBlock(strDataPath);
		if(KuRobotParameter::getInstance()->getUsingISSAC() == 1) // ISSAC을 사용하는 경우
		{
			PB.setPathBlockPosForISSAC(PBPack);
		}
		else	
		{
			PB.setPathBlockPos(PBPack);
		}
		vector<list<KuPose>>Pathlist=PB.getPathlist();
		KuDrawingInfo::getInstance()->setPathBlockPos(PBPack);
		KuDrawingInfo::getInstance()->setvecPathlist(Pathlist);
	} 
}


void CSamsung_TapelessAGVDlg::OnLoadpathPath8()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_LOADPATH_PATH8 , MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH8, MF_UNCHECKED);
		m_bexecuteflag=false;
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH8, MF_CHECKED);
		m_bexecuteflag=true;
		char cFilePathName[300];
		string strDataPath;int nIdx =8;
		sprintf(cFilePathName,"./data/path/LoadPathBlock%d.txt",nIdx);
		strDataPath=cFilePathName;//char 포인터값을 string에 대입
		vector<PBlock> PBPack;
		KuPathBlockPr PB;
		PBPack=PB.loadPathBlock(strDataPath);
		if(KuRobotParameter::getInstance()->getUsingISSAC() == 1) // ISSAC을 사용하는 경우
		{
			PB.setPathBlockPosForISSAC(PBPack);
		}
		else	
		{
			PB.setPathBlockPos(PBPack);
		}
		vector<list<KuPose>>Pathlist=PB.getPathlist();
		KuDrawingInfo::getInstance()->setPathBlockPos(PBPack);
		KuDrawingInfo::getInstance()->setvecPathlist(Pathlist);
	} 
}


void CSamsung_TapelessAGVDlg::OnLoadpathPath9()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_LOADPATH_PATH9 , MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH9, MF_UNCHECKED);
		m_bexecuteflag=false;
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH9, MF_CHECKED);
		m_bexecuteflag=true;
		char cFilePathName[300];
		string strDataPath;int nIdx =9;
		sprintf(cFilePathName,"./data/path/LoadPathBlock%d.txt",nIdx);
		strDataPath=cFilePathName;//char 포인터값을 string에 대입
		vector<PBlock> PBPack;
		KuPathBlockPr PB;
		PBPack=PB.loadPathBlock(strDataPath);
		if(KuRobotParameter::getInstance()->getUsingISSAC() == 1) // ISSAC을 사용하는 경우
		{
			PB.setPathBlockPosForISSAC(PBPack);
		}
		else	
		{
			PB.setPathBlockPos(PBPack);
		}
		vector<list<KuPose>>Pathlist=PB.getPathlist();
		KuDrawingInfo::getInstance()->setPathBlockPos(PBPack);
		KuDrawingInfo::getInstance()->setvecPathlist(Pathlist);
	} 
}


void CSamsung_TapelessAGVDlg::OnLoadpathPath10()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_LOADPATH_PATH10 , MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH10, MF_UNCHECKED);
		m_bexecuteflag=false;
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_LOADPATH_PATH10, MF_CHECKED);
		m_bexecuteflag=true;
		char cFilePathName[300];
		string strDataPath;int nIdx =10;
		sprintf(cFilePathName,"./data/path/LoadPathBlock%d.txt",nIdx);
		strDataPath=cFilePathName;//char 포인터값을 string에 대입
		vector<PBlock> PBPack;
		KuPathBlockPr PB;
		PBPack=PB.loadPathBlock(strDataPath);
		if(KuRobotParameter::getInstance()->getUsingISSAC() == 1) // ISSAC을 사용하는 경우
		{
			PB.setPathBlockPosForISSAC(PBPack);
		}
		else	
		{
			PB.setPathBlockPos(PBPack);
		}
		vector<list<KuPose>>Pathlist=PB.getPathlist();
		KuDrawingInfo::getInstance()->setPathBlockPos(PBPack);
		KuDrawingInfo::getInstance()->setvecPathlist(Pathlist);
	} 
}

void CSamsung_TapelessAGVDlg::OnBlocksizeSettingsize()
{
	m_BlockBox.showCurrentSize();
	m_BlockBox.ShowWindow(SW_SHOW);
}

void CSamsung_TapelessAGVDlg::OnCheckpointDefault()
{
	m_bchangpathblockflag=true;
	for(int i=0; i<2;i++)	m_bsetCheckPoint[i]=false;
	m_bsetCheckPoint[0]=true;
	changedPathBlock.check=false;
}


void CSamsung_TapelessAGVDlg::OnCheckpointCheck()
{
	m_bchangpathblockflag=true;
	for(int i=0; i<2;i++)	m_bsetCheckPoint[i]=false;
	m_bsetCheckPoint[1]=true;
	changedPathBlock.check=true;
}


void CSamsung_TapelessAGVDlg::OnObstacleDefault()
{
	m_bchangpathblockflag=true;
	for(int i=0; i<2;i++)	m_bsetObstacleDetection[i]=false;
	m_bsetObstacleDetection[0]=true;
	changedPathBlock.detectObstacle=false;
}


void CSamsung_TapelessAGVDlg::OnObstacleDetect()
{
	m_bchangpathblockflag=true;
	for(int i=0; i<2;i++)	m_bsetObstacleDetection[i]=false;
	m_bsetObstacleDetection[1]=true;
	changedPathBlock.detectObstacle=true;
}


void CSamsung_TapelessAGVDlg::OnCommunicationZigbee()
{
	CMenu* pMainMenu = GetMenu();
	
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_COMMUNICATION_ZIGBEE, MF_BYCOMMAND); 
	
	if (nMenuState != MF_CHECKED)
	{
		CAGVCommSupervisor::getInstance()->enableZigbeeComm(true); // Connect
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_COMMUNICATION_ZIGBEE, MF_CHECKED);
	}
	else
	{
		CAGVCommSupervisor::getInstance()->enableZigbeeComm(false); // Disconnect
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_COMMUNICATION_ZIGBEE, MF_UNCHECKED);
	}
}


void CSamsung_TapelessAGVDlg::OnCommunicationGPIO()
{
	CMenu* pMainMenu = GetMenu();

	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_COMMUNICATION_GPIO, MF_BYCOMMAND); 

	if (nMenuState != MF_CHECKED)
	{
		CAGVCommSupervisor::getInstance()->enableGPIOComm(true); // Connect
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_COMMUNICATION_GPIO, MF_CHECKED);
	}
	else
	{
		CAGVCommSupervisor::getInstance()->enableGPIOComm(false); // Disconnect
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_COMMUNICATION_GPIO, MF_UNCHECKED);
	}
}


void CSamsung_TapelessAGVDlg::OnCommunicationDoorOpen()
{
	AfxMessageBox(_T("Still working on this..."));
}

void CSamsung_TapelessAGVDlg::initMenu(void)
{
	CMenu* pMainMenu = GetMenu();
	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_COMMUNICATION_ZIGBEE, MF_CHECKED);
	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_COMMUNICATION_GPIO, MF_CHECKED);
	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_COMMUNICATION_DOOROPEN, MF_CHECKED);

	EnableMenuItem(pMainMenu->GetSafeHmenu(), ID_BEHAVIOR_SAVETEMPORARYMAP, MF_DISABLED);
	EnableMenuItem(pMainMenu->GetSafeHmenu(), ID_BEHAVIOR_LOADTEMPORARYMAP, MF_DISABLED);
}

// 지도작성 중 현재까지 작성된 지도를 임시로 저장한다.
void CSamsung_TapelessAGVDlg::OnBehaviorSaveTemporaryMap()
{
	m_CriticalSection.Lock();

	MobileSupervisor::getInstance()->getMapBuildingBehavior()->convertProbMapData2BMPFile();
	KuILBPFLocalizerPr::getInstance()->saveFeatureData();
	KuILBPFLocalizerPr::getInstance()->saveTemplateData();
	KuFiducialbasedLocalizerPr::getInstance()->saveFiducialFeatureMap();

	m_CriticalSection.Unlock();

	CMenu* pMainMenu = GetMenu();
	EnableMenuItem(pMainMenu->GetSafeHmenu(), ID_BEHAVIOR_LOADTEMPORARYMAP, MF_ENABLED);

	AfxMessageBox(_T("Saved."));
}

// 지도작성 중 임시저장한 지도를 불러온다.
void CSamsung_TapelessAGVDlg::OnBehaviorLoadTemporaryMap()
{
	m_CriticalSection.Lock();

	// 확률 격자지도
 	bool bRes = MobileSupervisor::getInstance()->getMapBuildingBehavior()->loadTemporaryMap();

	// 조명
	vector<CLAMPData> FRData= KuILBPFLocalizerPr::getInstance()->loadFeatureData( );
	KuDrawingInfo::getInstance()->setRegion(FRData);

	// Freak
 	vector<CFREAKData> FTData= KuILBPFLocalizerPr::getInstance()->loadTemplateData( );
 	KuDrawingInfo::getInstance()->setTemplateData(FTData);

	// Fiducial
	KuFiducialbasedLocalizerPr::getInstance()->loadFiducialFeatureMap( );
	list<KuPose> FiducialMarkList = KuFiducialbasedLocalizerPr::getInstance()->getFiducialMarkList();
	KuDrawingInfo::getInstance()->setFiducialMarkList(FiducialMarkList);

	m_CriticalSection.Unlock();

	AfxMessageBox(_T("Lamp data has not been loaded (will be fixed)."));
}


void CSamsung_TapelessAGVDlg::OnFileLoadISSACPath()
{
	TCHAR szFilter[] = _T("ISSAC path files (*.xml) |*.xml|");
	CString sFilePath, sFolderPath, sFileName;
	char cFilePath[500];

	CFileDialog dlg(TRUE, _T("*.xml"), NULL, OFN_HIDEREADONLY | OFN_PATHMUSTEXIST, szFilter);

	dlg.m_ofn.lpstrTitle = _T("Open");

	if(dlg.DoModal() == IDOK)
	{
		USES_CONVERSION;

		sFilePath = dlg.GetPathName();

		strcpy(cFilePath, W2A(sFilePath.GetBuffer(0)));

		TiXmlDocument doc(cFilePath);

		if(doc.LoadFile())
		{
			Serverpart::getInstance()->XmlPathParsing(&doc);
		}
	}
}

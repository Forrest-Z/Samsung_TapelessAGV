
// Samsung_TapelessAGVDlg.h : 헤더 파일
//

#pragma once
#include "./src/KUNSGUI/OwnImgListCtrl.h"
#include "./src/KUNSGUI/KUNSUI3DDlg.h"
#include "./src/KUNSGUI/PointSettingDlg.h"
#include "./src/KUNSGUI/BlockBox.h"
#include "./src/MobileSupervisor/MobileSupervisor.h"
#include"./src/MobileSupervisor/KuCommandMessage.h"
#include"./src/MobileSupervisor/MobileSupervisor.h"
#include "./src/Sensor/WheelActuatorInterface/SSAGVWheelActuatorInterface.h"
#include "./src/Sensor/SensorSupervisor.h"
#include "./src/MobileSupervisor/KuRobotParameter.h"
#include "./src/KUNSUtil/KUNSThread/KuThread.h"
#include "./src/KUNSUtil/KUNSCriticalSection/KuCriticalSection.h"
#include "./src/Algorithm/PathBlock/PathBlock.h"
#include "./src/KUNSPRIMUSComm/KuPRIMUSCommSupervisor.h"
#include "./src/AGVComm/AGVCommSupervisor.h"

// CSamsung_TapelessAGVDlg 대화 상자
class CSamsung_TapelessAGVDlg : public CDialogEx
{
	// 생성입니다.
public:
	CSamsung_TapelessAGVDlg(CWnd* pParent = NULL);	// 표준 생성자입니다.

	// 대화 상자 데이터입니다.
	enum { IDD = IDD_SAMSUNG_TAPELESSAGV_DIALOG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 지원입니다.
private:
	CCriticalSection m_CriticalSection;
	CStatic m_CamInfoPicCtrl;
	CStatic m_CamInfoPicCtrl2;

private:
	CEdit m_XPosEditCtrl; //로봇의 위치정보를 나타내는 edit control
	CEdit m_YPosEditCtrl; //로봇의 위치정보를 나타내는 edit control
	CEdit m_TPosEditCtrl; //로봇의 위치정보를 나타내는 edit control
private:
	CEdit m_CeilingImage; //로봇의 위치정보를 나타내는 edit control
	CEdit m_KinectImage; //로봇의 위치정보를 나타내는 edit control
	CEdit m_AvoidCheckbox; //로봇의 위치정보를 나타내는 edit control
	CEdit m_SaveWaypointButton; //로봇의 위치정보를 나타내는 edit control

	CScrollBar      m_ctrVScroll;
	COwnImgListCtrl m_cInfo;
	CImageList* m_LargeImage;
	int m_nitemID;

	bool m_bsetWayPoint[5];
	bool m_bsetCheckPoint[2];
	bool m_bsetObstacleDetection[2];
	bool m_bsetVelocity[6];
	bool m_bsetViaPoint[11];
	bool m_bsetMotionPause;
	bool m_bsetMotionResume;
	bool m_bsetMotionRotaion[4];
	bool m_bsetBehaviorSound[3];
	bool m_bsetBehaviorTowerLamp;
	bool m_bsetBehaviorDevice[3];
	bool m_bsetBehaviorReadytoSignal;
	bool m_bsettingflag;
	bool m_bchangpathblockflag;
	PBlock changedPathBlock;
	int m_SelID;
private:
	KuThread m_KuThread;
	static void doThread(void* arg);
	void initMenu(void); // Menu 초기화
private:
	CKUNSUI3DDlg m_KUNSUI3DDlg;
	Localizer * m_pLocalizer;
	int m_nBehaviorName;
	int m_nSetGoalPosID,m_nSetRobotPosID;
	int m_nBhID;
	bool m_bexecuteflag;
	bool m_bSetRobotPos;
	bool m_bAvoidModeflag;
	double m_dX;
	double m_dY;
	double m_dThetaDeg;
	double m_dTheta;
	vector<int> m_nvecBehavior;
	PointSettingDlg m_PSettingDlg;
	BlockBox m_BlockBox;
	CDC* m_pDC_sensorconnection; // Top info box
	CDC m_MemDC_sensorconnection; // Top info box
	// 구현입니다.
protected:
	HICON m_hIcon;

	// 생성된 메시지 맵 함수
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
private:
	void displaySensorConnection(void);
public:
	afx_msg void OnGuiinformationSetrobotpose();
	afx_msg void OnGuiinformationSetgoalpose();
	afx_msg void OnGuiinformationRendermap();
	afx_msg void OnGuiinformationRenderlaser();
	afx_msg void OnGuiinformationRenderkinect();
	afx_msg void OnGuiinformationRenderpath();
	afx_msg void OnSensorconnectionHokuyoutm();
	afx_msg void OnSensorconnectionKinect();
	afx_msg void OnSensorconnectionCeilingcamera();
	afx_msg void OnSensorconnectionE2boximu();
	afx_msg void OnSensorconnectionWheelactuator();
	afx_msg void OnBehaviorGotogoal();
	afx_msg void OnBehaviorMapbuilding();
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	virtual BOOL PreTranslateMessage(MSG* pMsg);
	afx_msg void OnBehaviorTeachingpath();
	afx_msg void OnGuiinformationRenderceilingimage();
	afx_msg void OnBnClickedSaveWaypointButton();
	afx_msg void OnBehaviorAutonomousgotogoal();
	afx_msg void OnBnClickedOk();
	afx_msg void OnDestroy();
	afx_msg void OnBnClickedObsavoidCheck();
	afx_msg void OnBehaviorGloballocalization();
	afx_msg void OnGuiinformationRenderzonemap();
	afx_msg void OnBehaviorMulti();
	afx_msg void OnCommunicationConnect();
	afx_msg void OnNMThemeChangedScrollbar1(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnVScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLvnItemchangedListInfo(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnGuiinformationSetpathblock();
	afx_msg void OnPathblockGeneratepath();
	afx_msg void OnPathblockClearblo();
	afx_msg void OnVelocityWhite();
	afx_msg void OnMotionPause();
	afx_msg void OnContextMenu(CWnd* /*pWnd*/, CPoint /*point*/);
	afx_msg void OnVelocityRed();
	afx_msg void OnVelocityYellow();
	afx_msg void OnVelocityGreen();
	afx_msg void OnVelocityBlue();
	afx_msg void OnVelocityPurple();
	afx_msg void OnMotionResume();
	afx_msg void OnRotateLeft90();
	afx_msg void OnRotateRight90();
	afx_msg void OnRotateLeft180();
	afx_msg void OnRotateRight180();
	afx_msg void OnDeviceDevice1();
	afx_msg void OnDeviceDevice2();
	afx_msg void OnDeviceDevice3();
	afx_msg void OnBehaviorReadytosignal();
	afx_msg void OnBehaviorTowerlamp();
	afx_msg void OnSoundSonud1();
	afx_msg void OnSoundSonud2();
	afx_msg void OnSoundSonud3();
	afx_msg void OnPathblockSetting();
	afx_msg void OnPathblockSavepathblock();
	afx_msg void OnPointWaypoint();
	afx_msg void OnPointStartpoint();
	afx_msg void OnPointEndpoint();
	afx_msg void OnPointDefault();
	afx_msg void OnPathblockLoadpathblock();
	afx_msg void OnSettingTask();
	afx_msg void OnBehaviorGlobalMapBuilding();
	afx_msg void OnVia1();
	afx_msg void OnVia2();
	afx_msg void OnVia3();
	afx_msg void OnVia4();
	afx_msg void OnVia5();
	afx_msg void OnVia6();
	afx_msg void OnVia7();
	afx_msg void OnVia8();
	afx_msg void OnVia9();
	afx_msg void OnVia10();
	afx_msg void OnViaDefalt();
	afx_msg void OnPathblockSavepath();
	afx_msg void OnSavepathDefault();
	afx_msg void OnSavepathPath1();
	afx_msg void OnSavepathPath2();
	afx_msg void OnSavepathPath3();
	afx_msg void OnSavepathPath4();
	afx_msg void OnSavepathPath5();
	afx_msg void OnSavepathPath6();
	afx_msg void OnSavepathPath7();
	afx_msg void OnSavepathPath8();
	afx_msg void OnSavepathPath9();
	afx_msg void OnSavepathPath10();
	afx_msg void OnLoadpathPath1();
	afx_msg void OnLoadpathPath2();
	afx_msg void OnLoadpathPath3();
	afx_msg void OnLoadpathPath4();
	afx_msg void OnLoadpathPath5();
	afx_msg void OnLoadpathPath6();
	afx_msg void OnLoadpathPath7();
	afx_msg void OnLoadpathPath8();
	afx_msg void OnLoadpathPath9();
	afx_msg void OnLoadpathPath10();
	afx_msg void OnSaveSave();
	afx_msg void OnSaveLoad();
	afx_msg void OnBlocksizeSettingsize();
	afx_msg void OnCheckpointDefault();
	afx_msg void OnCheckpointCheck();
	afx_msg void OnObstacleDefault();
	afx_msg void OnObstacleDetect();
	afx_msg void OnCommunicationZigbee();
	afx_msg void OnCommunicationGPIO();
	afx_msg void OnCommunicationDoorOpen();
	afx_msg void OnBehaviorSaveTemporaryMap();
	afx_msg void OnBehaviorLoadTemporaryMap();
	afx_msg void OnFileLoadISSACPath();
};

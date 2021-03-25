// PointSettingDlg.cpp : 구현 파일입니다.
//

#include "stdafx.h"
#include "../../Samsung_TapelessAGV.h"
#include "PointSettingDlg.h"
#include "afxdialogex.h"


// PointSettingDlg 대화 상자입니다.

IMPLEMENT_DYNAMIC(PointSettingDlg, CDialog)

PointSettingDlg::PointSettingDlg(CWnd* pParent /*=NULL*/)
	: CDialog(PointSettingDlg::IDD, pParent)
	,m_nListIDX(0)
{

}

PointSettingDlg::~PointSettingDlg()
{
}

void PointSettingDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_BEHAVIOR_LIST, m_behaviorlist);

}


BEGIN_MESSAGE_MAP(PointSettingDlg, CDialog)
	ON_BN_CLICKED(IDOK, &PointSettingDlg::OnBnClickedOk)
	ON_BN_CLICKED(IDC_ADD_BEHAVIOR_BUTTON, &PointSettingDlg::OnBnClickedAddBehaviorButton)
	ON_BN_CLICKED(IDC_MOTION_PAUSE, &PointSettingDlg::OnBnClickedMotionPause)
	ON_BN_CLICKED(IDC_MOTION_RESUME, &PointSettingDlg::OnBnClickedMotionResume)
	ON_BN_CLICKED(IDC_MOTION_Roation_LEFT90, &PointSettingDlg::OnBnClickedMotionRoationLeft90)
	ON_BN_CLICKED(IDC_MOTION_Roation_LEFT180, &PointSettingDlg::OnBnClickedMotionRoationLeft180)
	ON_BN_CLICKED(IDC_MOTION_Roation_Right90, &PointSettingDlg::OnBnClickedMotionRoationRight90)
	ON_BN_CLICKED(IDC_MOTION_Roation_Right180, &PointSettingDlg::OnBnClickedMotionRoationRight180)
	ON_BN_CLICKED(IDC_SOUND1, &PointSettingDlg::OnBnClickedSound1)
	ON_BN_CLICKED(IDC_SOUND3, &PointSettingDlg::OnBnClickedSound3)
	ON_BN_CLICKED(IDC_SOUND4, &PointSettingDlg::OnBnClickedSound4)
	ON_BN_CLICKED(IDC_SOUND2, &PointSettingDlg::OnBnClickedSound2)
	ON_BN_CLICKED(IDC_DEVICE1, &PointSettingDlg::OnBnClickedDevice1)
	ON_BN_CLICKED(IDC_DEVICE2, &PointSettingDlg::OnBnClickedDevice2)
	ON_BN_CLICKED(IDC_DEVICE3, &PointSettingDlg::OnBnClickedDevice3)
	ON_BN_CLICKED(IDC_DEVICE4, &PointSettingDlg::OnBnClickedDevice4)
	ON_BN_CLICKED(IDC_READYTOSIGNAL, &PointSettingDlg::OnBnClickedReadytosignal)
	ON_BN_CLICKED(IDC_TOWERLAMP, &PointSettingDlg::OnBnClickedTowerlamp)
	ON_BN_CLICKED(IDC_ELIMINATE_BUTTON, &PointSettingDlg::OnBnClickedEliminateButton)
END_MESSAGE_MAP()


// PointSettingDlg 메시지 처리기입니다.


void PointSettingDlg::OnBnClickedOk()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	CDialog::OnOK();	
}


void PointSettingDlg::OnBnClickedAddBehaviorButton()
{
	// TODO: Add your control notification handler code here
	UpdateData();
	if(m_strValue.GetString() != NULL) 
	{	
		//m_behaviorlist.AddString(m_strValue);
		m_behaviorlist.InsertString(m_nListIDX,m_strValue);
		m_nListIDX++;
		m_nvecBehavior.push_back(m_nBehavior);
	}
}
void PointSettingDlg::getBehavior(vector<int>* nvecBehavior)
{
	for(int i=0; i<m_nvecBehavior.size();i++)
	{
		(*nvecBehavior).push_back(m_nvecBehavior[i]);
	}
	
}
void PointSettingDlg::setBehavior(vector<int> nvecBehavior)
{
	m_nvecBehavior.clear();
	m_behaviorlist.ResetContent();
	m_nListIDX=nvecBehavior.size();
	for(int i=0; i<nvecBehavior.size();i++)
	{
		m_nvecBehavior.push_back(nvecBehavior[i]);

		switch(nvecBehavior[i])
		{
		case PathBlock::MOTION_PAUSE:
			m_behaviorlist.InsertString(i,_T("Pause"));
			break;
		case PathBlock::MOTION_RESUME:
			m_behaviorlist.InsertString(i,_T("Resume"));
			break;
		case PathBlock::MOTION_ROTATION_LEFT90:
			m_behaviorlist.InsertString(i,_T("Left 90"));
			break;
		case PathBlock::MOTION_ROTATION_LEFT180:
			m_behaviorlist.InsertString(i,_T("Left 180"));
			break;
		case PathBlock::MOTION_ROTATION_RIGHT90:
			m_behaviorlist.InsertString(i,_T("Right 90"));
			break;
		case PathBlock::MOTION_ROTATION_RIGHT180:
			m_behaviorlist.InsertString(i,_T("Right 180"));
			break;
		case PathBlock::SOUND1:
			m_behaviorlist.InsertString(i,_T("Sound 1"));
			break;
		case PathBlock::SOUND2:
			m_behaviorlist.InsertString(i,_T("Sound 2"));
			break;
		case PathBlock::SOUND3:
			m_behaviorlist.InsertString(i,_T("Sound 3"));
			break;
		case PathBlock::SOUND4:
			m_behaviorlist.InsertString(i,_T("Sound 4"));
			break;
		case PathBlock::DEVICE1:
			m_behaviorlist.InsertString(i,_T("Device 1"));
			break;
		case PathBlock::DEVICE2:
			m_behaviorlist.InsertString(i,_T("Device 2"));
			break;
		case PathBlock::DEVICE3:
			m_behaviorlist.InsertString(i,_T("Device 3"));
			break;
		case PathBlock::DEVICE4:
			m_behaviorlist.InsertString(i,_T("Device 4"));
			break;
		case PathBlock::READYTOSIGNAL:
			m_behaviorlist.InsertString(i,_T("Ready to signal"));
			break;
		case PathBlock::TOWERLAMP:
			m_behaviorlist.InsertString(i,_T("Tower lamp"));
			break;
		default:
			break;
		}
	}
}

void PointSettingDlg::OnBnClickedMotionPause()
{
	m_strValue="Pause";
	m_nBehavior=PathBlock::MOTION_PAUSE;
}


void PointSettingDlg::OnBnClickedMotionResume()
{
	m_strValue="Resume";
	m_nBehavior=PathBlock::MOTION_RESUME;

}


void PointSettingDlg::OnBnClickedMotionRoationLeft90()
{
	m_strValue="Left 90";
	m_nBehavior=PathBlock::MOTION_ROTATION_LEFT90  ;

}


void PointSettingDlg::OnBnClickedMotionRoationLeft180()
{
	m_strValue="Left 180";
	m_nBehavior=PathBlock::MOTION_ROTATION_LEFT180;

}


void PointSettingDlg::OnBnClickedMotionRoationRight90()
{
	m_strValue="Right 90";
	m_nBehavior=PathBlock::MOTION_ROTATION_RIGHT90 ;

}


void PointSettingDlg::OnBnClickedMotionRoationRight180()
{
	m_strValue="Right 180";
	m_nBehavior=PathBlock::MOTION_ROTATION_RIGHT180;

}


void PointSettingDlg::OnBnClickedSound1()
{
	m_strValue="Sound 1";
	m_nBehavior=PathBlock::SOUND1  ;

}

void PointSettingDlg::OnBnClickedSound2()
{
	m_strValue="Sound 2";
	m_nBehavior=PathBlock::SOUND2  ;

}

void PointSettingDlg::OnBnClickedSound3()
{
	m_strValue="Sound 3";
	m_nBehavior=PathBlock::SOUND3  ;

}


void PointSettingDlg::OnBnClickedSound4()
{
	m_strValue="Sound 4";
	m_nBehavior=PathBlock::SOUND4 ;

}


void PointSettingDlg::OnBnClickedDevice1()
{
	m_strValue="Device 1";
	m_nBehavior=PathBlock::DEVICE1  ;

}


void PointSettingDlg::OnBnClickedDevice2()
{
	m_strValue="Device 2";
	m_nBehavior=PathBlock::DEVICE2  ;

	AfxMessageBox(_T("Warning: Only 1 task should be assigned for device 2 and 3."));
}


void PointSettingDlg::OnBnClickedDevice3()
{
	m_strValue="Device 3";
	m_nBehavior=PathBlock::DEVICE3  ;

	AfxMessageBox(_T("Warning: Only 1 task should be assigned for device 2 and 3."));
}


void PointSettingDlg::OnBnClickedDevice4()
{
	m_strValue="Device 4";
	m_nBehavior=PathBlock::DEVICE4  ;

}


void PointSettingDlg::OnBnClickedReadytosignal()
{
	m_strValue="Ready to signal";
	m_nBehavior=PathBlock::READYTOSIGNAL  ;

}


void PointSettingDlg::OnBnClickedTowerlamp()
{
	m_strValue="Tower lamp";
	m_nBehavior=PathBlock::TOWERLAMP  ;

}


void PointSettingDlg::OnBnClickedEliminateButton()
{
	// TODO: Add your control notification handler code here
	CString tmp;
	int nIndex = m_behaviorlist.GetCurSel();
	vector<int>::iterator iter;
	if(nIndex != LB_ERR)
	{
		m_behaviorlist.DeleteString(nIndex);
		m_nListIDX--;
		iter=m_nvecBehavior.begin();
		m_nvecBehavior.erase(iter+nIndex);
	}
}

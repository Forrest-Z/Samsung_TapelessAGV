#pragma once
#include "../Algorithm/PathBlock/PathBlock.h"
#include <vector>
// PointSettingDlg ��ȭ �����Դϴ�.
using namespace std;

class PointSettingDlg : public CDialog
{
	DECLARE_DYNAMIC(PointSettingDlg)

public:
	PointSettingDlg(CWnd* pParent = NULL);   // ǥ�� �������Դϴ�.
	virtual ~PointSettingDlg();

// ��ȭ ���� �������Դϴ�.
	enum { IDD = IDD_POINTSETTING_DIALOG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV �����Դϴ�.

	DECLARE_MESSAGE_MAP()
public:
	CListBox m_behaviorlist;
	CString m_strValue;
	int m_nListIDX;
	vector<int> m_nvecBehavior;
	int m_nBehavior;
	void getBehavior(vector<int>* nvecBehavior);
	void setBehavior(vector<int> nvecBehavior);

public:
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedAddBehaviorButton();
	afx_msg void OnBnClickedMotionPause();
	afx_msg void OnBnClickedMotionResume();
	afx_msg void OnBnClickedMotionRoationLeft90();
	afx_msg void OnBnClickedMotionRoationLeft180();
	afx_msg void OnBnClickedMotionRoationRight90();
	afx_msg void OnBnClickedMotionRoationRight180();
	afx_msg void OnBnClickedSound1();
	afx_msg void OnBnClickedSound3();
	afx_msg void OnBnClickedSound4();
	afx_msg void OnBnClickedSound2();
	afx_msg void OnBnClickedDevice1();
	afx_msg void OnBnClickedDevice2();
	afx_msg void OnBnClickedDevice3();
	afx_msg void OnBnClickedDevice4();
	afx_msg void OnBnClickedReadytosignal();
	afx_msg void OnBnClickedTowerlamp();
	afx_msg void OnBnClickedEliminateButton();
};

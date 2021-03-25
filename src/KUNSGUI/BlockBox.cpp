// ./src/KUNSUI/BlockBox.cpp : 구현 파일입니다.
//

#include "stdafx.h"
#include "../Samsung_TapelessAGV.h"
#include "BlockBox.h"
#include "../src/MobileSupervisor/KuRobotParameter.h"
#include "afxdialogex.h"


// BlockBox 대화 상자입니다.

IMPLEMENT_DYNAMIC(BlockBox, CDialogEx)

BlockBox::BlockBox(CWnd* pParent /*=NULL*/)
	: CDialogEx(BlockBox::IDD, pParent)
{
	
}

BlockBox::~BlockBox()
{
}

void BlockBox::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(BlockBox, CDialogEx)
	ON_EN_CHANGE(IDC_EDIT1, &BlockBox::OnEnChangeEdit1)
	ON_EN_CHANGE(IDC_EDIT2, &BlockBox::OnEnChangeEdit2)
	ON_EN_CHANGE(IDC_EDIT3, &BlockBox::OnEnChangeEdit3)
	ON_EN_CHANGE(IDC_EDIT4, &BlockBox::OnEnChangeEdit4)
	ON_BN_CLICKED(IDC_BUTTON1, &BlockBox::OnBnClickedButton1)
END_MESSAGE_MAP()

void BlockBox::showCurrentSize()
{
	CDialog::OnInitDialog();

	CString str;
	double dTempX = KuRobotParameter::getInstance()->getBlockSizeX();
	str.Format(L"%.2f",dTempX);
	SetDlgItemText(IDC_EDIT3,str);

	double dTempY = KuRobotParameter::getInstance()->getBlockSizeY();
	str.Format(L"%.2f",dTempY);
	SetDlgItemText(IDC_EDIT4,str);
}


// BlockBox 메시지 처리기입니다.


void BlockBox::OnEnChangeEdit1()
{

}


void BlockBox::OnEnChangeEdit2()
{

}


void BlockBox::OnEnChangeEdit3()
{

}


void BlockBox::OnEnChangeEdit4()
{

}


void BlockBox::OnBnClickedButton1()
{
	double dTempX, dTempY;
	CString str;
	GetDlgItemText(IDC_EDIT1,str);
	dTempX = _wtof(str);
	str.Format(L"%.2f",dTempX);
	KuRobotParameter::getInstance()->setBlockSizeX(dTempX);
	SetDlgItemText(IDC_EDIT3,str);

	GetDlgItemText(IDC_EDIT2,str);
	dTempY = _wtof(str);
	str.Format(L"%.2f",dTempY);
	KuRobotParameter::getInstance()->setBlockSizeY(dTempY);
	SetDlgItemText(IDC_EDIT4,str);
}

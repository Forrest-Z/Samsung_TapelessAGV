#pragma once


// BlockBox ��ȭ �����Դϴ�.

class BlockBox : public CDialogEx
{
	DECLARE_DYNAMIC(BlockBox)

public:
	BlockBox(CWnd* pParent = NULL);   // ǥ�� �������Դϴ�.
	virtual ~BlockBox();

// ��ȭ ���� �������Դϴ�.
	enum { IDD = IDD_BLOCK_SIZE };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV �����Դϴ�.

	DECLARE_MESSAGE_MAP()
public:
	void showCurrentSize();
	afx_msg void OnEnChangeEdit1();
	afx_msg void OnEnChangeEdit2();
	afx_msg void OnEnChangeEdit3();
	afx_msg void OnEnChangeEdit4();
	afx_msg void OnBnClickedButton1();
};

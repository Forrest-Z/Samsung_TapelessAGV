#pragma once


// BlockBox 대화 상자입니다.

class BlockBox : public CDialogEx
{
	DECLARE_DYNAMIC(BlockBox)

public:
	BlockBox(CWnd* pParent = NULL);   // 표준 생성자입니다.
	virtual ~BlockBox();

// 대화 상자 데이터입니다.
	enum { IDD = IDD_BLOCK_SIZE };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.

	DECLARE_MESSAGE_MAP()
public:
	void showCurrentSize();
	afx_msg void OnEnChangeEdit1();
	afx_msg void OnEnChangeEdit2();
	afx_msg void OnEnChangeEdit3();
	afx_msg void OnEnChangeEdit4();
	afx_msg void OnBnClickedButton1();
};

#if !defined(AFX_OWNIMGLISTCTRL_H__48E8D3A3_CC2E_4EDD_86B8_010896E5EF71__INCLUDED_)
#define AFX_OWNIMGLISTCTRL_H__48E8D3A3_CC2E_4EDD_86B8_010896E5EF71__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// OwnImgListCtrl.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// COwnImgListCtrl window

class COwnImgListCtrl : public CListCtrl
{
// Construction
public:
	COwnImgListCtrl();

// Attributes
public:

// Operations
public:

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(COwnImgListCtrl)
	//}}AFX_VIRTUAL

// Implementation
public:
	virtual ~COwnImgListCtrl();

	// Generated message map functions
protected:
	//{{AFX_MSG(COwnImgListCtrl)
		afx_msg void OnNMCustomdraw(NMHDR *pNMHDR, LRESULT *pResult);
	//}}AFX_MSG

	DECLARE_MESSAGE_MAP()
};

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_OWNIMGLISTCTRL_H__48E8D3A3_CC2E_4EDD_86B8_010896E5EF71__INCLUDED_)

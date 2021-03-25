// OwnImgListCtrl.cpp : implementation file
//

#include "stdafx.h"
#include "../../Samsung_TapelessAGV.h"
#include "OwnImgListCtrl.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// COwnImgListCtrl

COwnImgListCtrl::COwnImgListCtrl()
{
}

COwnImgListCtrl::~COwnImgListCtrl()
{
}


BEGIN_MESSAGE_MAP(COwnImgListCtrl, CListCtrl)
	//{{AFX_MSG_MAP(COwnImgListCtrl)
	ON_NOTIFY_REFLECT(NM_CUSTOMDRAW, OnNMCustomdraw)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// COwnImgListCtrl message handlers


void COwnImgListCtrl::OnNMCustomdraw(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMCUSTOMDRAW     pNMCD = reinterpret_cast<LPNMCUSTOMDRAW>(pNMHDR);
	LPNMLVCUSTOMDRAW  lplvcd = reinterpret_cast<LPNMLVCUSTOMDRAW>(pNMHDR);
	LVITEM   LI={0};
	CDC   dc;
	int   nItem;
	TCHAR szBuff[256];
	CPoint pt;
	CRect  crt;
	IMAGEINFO ImageInfo;
	int    w, h;
	CImageList *pImg;
	UINT   fStyle;
	
	switch(lplvcd->nmcd.dwDrawStage)
	{
	case CDDS_PREPAINT:
		*pResult = CDRF_NOTIFYITEMDRAW;
		return ;
		
	case CDDS_ITEMPREPAINT:
		*pResult = CDRF_SKIPDEFAULT;
		 
		nItem = (int)lplvcd->nmcd.dwItemSpec;
		
		LI.mask=LVIF_TEXT | LVIF_IMAGE;
		LI.iItem = nItem;
		LI.iSubItem = 0;
		LI.pszText = szBuff;
		LI.cchTextMax = sizeof(szBuff);
		GetItem(&LI);
		
		dc.Attach(lplvcd->nmcd.hdc);
		pImg=GetImageList(LVSIL_NORMAL);
		
		if ((lplvcd->nmcd.uItemState&CDIS_SELECTED) || (GetItemState(nItem,LVIS_SELECTED|LVIS_FOCUSED)!=0))
		{
			fStyle=ILD_SELECTED;
		}else{
			fStyle=ILD_NORMAL;
		}
		
		GetItemRect(nItem, &crt, LVIR_BOUNDS);
		pImg->GetImageInfo(LI.iImage, &ImageInfo);
		w=ImageInfo.rcImage.right-ImageInfo.rcImage.left;
		h=ImageInfo.rcImage.bottom-ImageInfo.rcImage.top;
		pt.x=crt.left+(crt.Width()-w)/2;
		pt.y=crt.top+(crt.Height()-h)/2;
		
		pImg->Draw(&dc,LI.iImage,pt,fStyle);
		dc.SetBkMode(TRANSPARENT);
		dc.SetTextColor(lplvcd->clrText);
		dc.DrawText(szBuff,-1,&crt,DT_SINGLELINE|DT_CENTER|DT_VCENTER);
		dc.Detach();
		return ;
		
	default:
		*pResult = CDRF_DODEFAULT;
		break;
	}
}

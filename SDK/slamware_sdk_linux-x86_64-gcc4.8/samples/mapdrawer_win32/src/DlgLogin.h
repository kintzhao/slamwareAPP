/*
 * Copyright (C) 2014 SLAMTEC Co., Ltd.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


// DlgLogin.h : interface of the CDlgLogin class
//
/////////////////////////////////////////////////////////////////////////////
#if !defined(VFC_DLGLOGIN_H__45531F42_FAEB_491c_AD14_152A40B5F164__INCLUDED_)
#define VFC_DLGLOGIN_H__45531F42_FAEB_491c_AD14_152A40B5F164__INCLUDED_

#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000

class CDlgLogin : public CDialogImpl<CDlgLogin>,
	public CWinDataExchange<CDlgLogin>
{
public:
	WTL::CString	m_ipaddr;
	CDlgLogin();
	enum { IDD = IDD_DLG_LOGIN };


	BEGIN_DDX_MAP(CDlgLogin)
		DDX_TEXT(IDC_EDT_IPADDR, m_ipaddr)
	END_DDX_MAP()

	BEGIN_MSG_MAP(CDlgLogin)
		MESSAGE_HANDLER(WM_INITDIALOG, OnInitDialog)
		COMMAND_ID_HANDLER(IDOK, OnOK)
		COMMAND_ID_HANDLER(IDCANCEL, OnCancel)
	END_MSG_MAP()

// Handler prototypes (uncomment arguments if needed):
//	LRESULT MessageHandler(UINT /*uMsg*/, WPARAM /*wParam*/, LPARAM /*lParam*/, BOOL& /*bHandled*/)
//	LRESULT CommandHandler(WORD /*wNotifyCode*/, WORD /*wID*/, HWND /*hWndCtl*/, BOOL& /*bHandled*/)
//	LRESULT NotifyHandler(int /*idCtrl*/, LPNMHDR /*pnmh*/, BOOL& /*bHandled*/)

	LRESULT OnInitDialog(UINT /*uMsg*/, WPARAM /*wParam*/, LPARAM /*lParam*/, BOOL& /*bHandled*/);
	LRESULT OnOK(WORD /*wNotifyCode*/, WORD wID, HWND /*hWndCtl*/, BOOL& /*bHandled*/);
	LRESULT OnCancel(WORD /*wNotifyCode*/, WORD wID, HWND /*hWndCtl*/, BOOL& /*bHandled*/);
};

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// VisualFC AppWizard will insert additional declarations immediately before the previous line.
#endif // !defined(VFC_DLGLOGIN_H__45531F42_FAEB_491c_AD14_152A40B5F164__INCLUDED_)

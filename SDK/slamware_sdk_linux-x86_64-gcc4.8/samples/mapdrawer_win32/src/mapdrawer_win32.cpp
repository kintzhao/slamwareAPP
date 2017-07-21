// mapdrawer_win32.cpp : main source file for mapdrawer_win32.exe
//

#include "stdafx.h"

#include "resource.h"

#include "mapdrawer_win32View.h"
#include "aboutdlg.h"
#include "MainFrm.h"
#include "DlgLogin.h"

using namespace rpos::robot_platforms;
using namespace rpos::features;
using namespace rpos::features::location_provider;
using namespace rpos::system::detail;

CAppModule _Module;

// SLAMWARE SDP Main Instance
SlamwareCorePlatform sdp;

int Run(LPTSTR /*lpstrCmdLine*/ = NULL, int nCmdShow = SW_SHOWDEFAULT)
{
	CMessageLoop theLoop;
	_Module.AddMessageLoop(&theLoop);

    // ask user to connect to the SDP first...

    CDlgLogin dlgLogin;

    if (dlgLogin.DoModal() == IDCANCEL) {
        return 0;
    }

    try {
        sdp = SlamwareCorePlatform::connect((const char *)dlgLogin.m_ipaddr, 1445);
    } catch (ExceptionBase & e)
    {
        CString errMsg;
        errMsg = "Failed to connect to the SDP: " + dlgLogin.m_ipaddr + "\n"
            + e.what();
        MessageBox(NULL, errMsg, "Error", MB_OK);
        return 0;
    }

	CMainFrame wndMain;

	if(wndMain.CreateEx() == NULL)
	{
		ATLTRACE(_T("Main window creation failed!\n"));
		return 0;
	}

	wndMain.ShowWindow(nCmdShow);

	int nRet = theLoop.Run();

    // disconnect from the SDP
    sdp.disconnect();

	_Module.RemoveMessageLoop();
	return nRet;
}

int WINAPI _tWinMain(HINSTANCE hInstance, HINSTANCE /*hPrevInstance*/, LPTSTR lpstrCmdLine, int nCmdShow)
{
	HRESULT hRes = ::CoInitialize(NULL);
// If you are running on NT 4.0 or higher you can use the following call instead to 
// make the EXE free threaded. This means that calls come in on a random RPC thread.
//	HRESULT hRes = ::CoInitializeEx(NULL, COINIT_MULTITHREADED);
	ATLASSERT(SUCCEEDED(hRes));

	// this resolves ATL window thunking problem when Microsoft Layer for Unicode (MSLU) is used
	::DefWindowProc(NULL, 0, 0, 0L);

	AtlInitCommonControls(ICC_COOL_CLASSES | ICC_BAR_CLASSES);	// add flags to support other controls

	hRes = _Module.Init(NULL, hInstance);
	ATLASSERT(SUCCEEDED(hRes));

	AtlAxWinInit();

	int nRet = Run(lpstrCmdLine, nCmdShow);

	_Module.Term();
	::CoUninitialize();

	return nRet;
}

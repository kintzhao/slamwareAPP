// mapdrawer_win32View.h : interface of the CMapdrawer_win32View class
//
/////////////////////////////////////////////////////////////////////////////

#pragma once

class CMapdrawer_win32View : public CScrollWindowImpl<CMapdrawer_win32View>
{
public:
	
    enum {
        MAP_SCALE_FACTOR = 2,

    };
    DECLARE_WND_CLASS(NULL)

    CMapdrawer_win32View();

	BOOL PreTranslateMessage(MSG* pMsg);

	void DoPaint(CDCHandle dc);

	BEGIN_MSG_MAP(CMapdrawer_win32View)
		CHAIN_MSG_MAP(CScrollWindowImpl<CMapdrawer_win32View>)
	END_MSG_MAP()

    void drawmap();

protected:
    // Map Off-screen GUI buffer
    CBitmap     _memBitmap;
    CDC         _memDC;
    BITMAPINFO  _memBitmapDesc;
    DWORD     * _memBitmapData;
    bool        _isMemDCValid;
};

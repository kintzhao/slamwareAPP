// mapdrawer_win32View.cpp : implementation of the CMapdrawer_win32View class
//
/////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "resource.h"

#include "mapdrawer_win32View.h"

#include <rpos/features/location_provider/map.h>
using namespace rpos::robot_platforms;
using namespace rpos::features;
using namespace rpos::features::location_provider;

// SLAMWARE SDP Main Instance
extern SlamwareCorePlatform sdp;

CMapdrawer_win32View::CMapdrawer_win32View()
    : _memBitmapData(NULL)
    , _isMemDCValid()
{

}

BOOL CMapdrawer_win32View::PreTranslateMessage(MSG* pMsg)
{
	pMsg;
	return FALSE;
}

void CMapdrawer_win32View::DoPaint(CDCHandle dc)
{
	if (!_isMemDCValid) return;

    dc.StretchBlt(0,0,_memBitmapDesc.bmiHeader.biWidth*MAP_SCALE_FACTOR, _memBitmapDesc.bmiHeader.biHeight *MAP_SCALE_FACTOR,
        _memDC, 0, 0, _memBitmapDesc.bmiHeader.biWidth, _memBitmapDesc.bmiHeader.biHeight, SRCCOPY);
}


void CMapdrawer_win32View::drawmap()
{
    // delete the previous memory bitmap....
    if (_isMemDCValid)
    {
        _isMemDCValid = false;
        _memBitmapData = NULL;
        _memDC.DeleteDC();
        _memBitmap.DeleteObject();
    }

    // alloc the bitmap...

    try {
        // fetch the whole map...
        rpos::core::RectangleF knownArea = sdp.getLocationProvider().getKnownArea(MapTypeBitmap8Bit, location_provider::EXPLORERMAP);

        location_provider::Map map = sdp.getMap(MapTypeBitmap8Bit, knownArea, rpos::features::location_provider::EXPLORERMAP);
        int mapCellWidth = map.getMapDimension().x();
        int mapCellHeight = map.getMapDimension().y();


        // padding to LONG boundary
        size_t scanlineSize = ( ((mapCellWidth *4 + sizeof(LONG)-1))/sizeof(LONG) )*sizeof(LONG);
        

        memset(&_memBitmapDesc, 0, sizeof(_memBitmapDesc));
        _memBitmapDesc.bmiHeader.biSize = sizeof(_memBitmapDesc);
        _memBitmapDesc.bmiHeader.biWidth = mapCellWidth;
        _memBitmapDesc.bmiHeader.biHeight = mapCellHeight;
        _memBitmapDesc.bmiHeader.biPlanes = 1;
        _memBitmapDesc.bmiHeader.biBitCount = 32; //32bit Color
        _memBitmapDesc.bmiHeader.biCompression = BI_RGB;
        _memBitmapDesc.bmiHeader.biSizeImage = mapCellHeight * scanlineSize;


        _memBitmap.CreateDIBSection(::GetDC(NULL), & _memBitmapDesc, DIB_RGB_COLORS, (void **)&_memBitmapData, NULL, 0);

        _memDC.CreateCompatibleDC(::GetDC(NULL));
        ::DeleteObject(_memDC.SelectBitmap(_memBitmap));

        _isMemDCValid = true;

        // fill the cell data...

        for (int posY = 0; posY < mapCellHeight; ++posY)
        {
            for (int posX = 0; posX < mapCellWidth; ++posX)
            {            
                rpos::system::types::_u8 mapValue_8bit = map.getMapData()[posX + posY * mapCellWidth] + 127;

                // write back to the bitmap...
                _memBitmapData[posX + posY*scanlineSize/sizeof(*_memBitmapData)] = RGB(mapValue_8bit,mapValue_8bit,mapValue_8bit);
            }
        }

        // set scroll data...

        this->SetScrollSize(mapCellWidth*MAP_SCALE_FACTOR, mapCellHeight*MAP_SCALE_FACTOR, FALSE, false);
        this->Invalidate();
    } 
    catch(...)
    {
        //something wring, simply quit...
        return;
    }
    
}
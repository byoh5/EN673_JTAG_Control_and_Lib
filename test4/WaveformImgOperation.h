#pragma once

#include <afxwin.h>
#include <atlstr.h> 
//#include <windows.h>
#pragma once

//   Waveform Parameter
// ========================================================================================
#define BACKGROUND_BRUSH_COLOR			RGB(0, 0, 0)
#define BACKGROUND_LINE_COLOR			RGB(255, 128, 0)

#define BIT_FREQUENCY					256

#define VECTORSCOPE_DIAMETER			BIT_FREQUENCY
#define VECTORSCOPE_RECT(X, Y)			CRect(X, Y, X + VECTORSCOPE_DIAMETER, Y + VECTORSCOPE_DIAMETER)
#define VECTORSCOPE_BACKGROUND_WIDTH	BIT_FREQUENCY
#define VECTORSCOPE_BACKGROUND_HEIGHT	BIT_FREQUENCY
#define VECTORSCOPE_INTERVAL			22
#define VECTORSCOPE_INTERVAL_WIDTH		22
#define VECTORSCOPE_INTERVAL_HEIGHT		22
#define VECTORSCOPE_BACKGROUND_RECT		CRect(0, 0, VECTORSCOPE_BACKGROUND_WIDTH, VECTORSCOPE_BACKGROUND_HEIGHT)

#define VECTORSCOPE_CENTER_X			(VECTORSCOPE_BACKGROUND_WIDTH / 2)
#define VECTORSCOPE_CENTER_Y			(VECTORSCOPE_BACKGROUND_HEIGHT / 2)
#define VECTORSCOPE_COLOR_POINT			6

#define PI					3.14159265358979
// Magenta, Red, Yellow, Green, Cyan, Blue
// Reference by 75% YCbCr Color Bar 
const double COLOR_DEGREE[VECTORSCOPE_COLOR_POINT] = { 49.46, 102.75, 174.56, 229.46, 282.75, 354.56 };
const double COLOR_SCALE[VECTORSCOPE_COLOR_POINT] = { 0.78, 0.67, 0.66, 0.78, 0.67, 0.66 };


#define WAVEFORM_WIDTH					320
#define WAVEFORM_HEIGHT					BIT_FREQUENCY
#define WAVEFORM_RECT(X, Y)				CRect(X, Y, X + WAVEFORM_WIDTH, Y + WAVEFORM_HEIGHT)
#define WAVEFORM_BACKGROUND_WIDTH		WAVEFORM_WIDTH
#define WAVEFORM_BACKGROUND_HEIGHT		BIT_FREQUENCY
#define WAVEFORM_INTERVAL_WIDTH			20
#define WAVEFORM_INTERVAL_HEIGHT		22
#define WAVEFORM_BACKGROUND_RECT		CRect(0, 0, WAVEFORM_BACKGROUND_WIDTH, WAVEFORM_BACKGROUND_HEIGHT)

#define WAVE_GRADATION_POINT			9


#define SUBWINDOW_RECT					CRect(0, 0, VECTORSCOPE_BACKGROUND_WIDTH + WAVEFORM_BACKGROUND_WIDTH, VECTORSCOPE_BACKGROUND_HEIGHT)
#define SUBIMAGE_HEIGHT					WAVEFORM_BACKGROUND_HEIGHT	// WAVEFORM HEIGHT == VECTOR HEIGHT

// ========================================================================================
class CWaveformImgOperation
{
public:
    CWaveformImgOperation(void);
    ~CWaveformImgOperation(void);

public:
    void	InitWaveformDraw(int imgWidth, int imgHeight);
    BYTE*	GetWaveform(BYTE *imgColor, int imgWidth, int imgHeight, int scale, int ch); 
    BYTE*	GetVectorScope(BYTE *imgCb, BYTE *imgCr, int imgWidth, int imgHeight, int scale);

private:
    // Buffer
    BYTE*	m_WaveformBuffer;
    BYTE*	m_VectorScopeBuffer;
    short*	m_LumaFreqBuffer;
    short*	m_ChromaFreqBuffer;

    // Initial Function & variable
    bool	CreateColorTable(void);
 //   void	CreateBITMAPHEADER(int width, int height); 
    bool	bColorTableDone;
 //   BITMAPINFO	m_BmpInfo;

    // Color Frequency of A image Calculate
    short*	CalculateWaveform(BYTE *buffer, int width, int height);
    short*	CalculateVectorScope(BYTE *cb, BYTE *cr, int width, int height);

    // Background brush & pen
    CPen	m_Pen;
    CBrush	m_Brush;
    CFont	m_Font;

    // Color Table for Graph Draw
    BYTE*	m_ColorTable_R;
    BYTE*	m_ColorTable_G;
    BYTE*	m_ColorTable_B;

    int		Y[256];
    int		CbToB[256];
    int		CbToG[256];
    int		CrToR[256];
    int		CrToG[256];
};


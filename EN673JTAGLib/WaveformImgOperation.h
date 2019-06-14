#pragma once
#include <windows.h>
#include <afxwin.h>
#include <atlstr.h> 

//   Waveform Parameter
// ========================================================================================
#define BACKGROUND_BRUSH_COLOR			RGB(0, 0, 0)
#define BACKGROUND_LINE_COLOR			RGB(255, 128, 0)

#define BIT_FREQUENCY					256

#define VECTORSCOPE_DIAMETER			BIT_FREQUENCY
#define VECTORSCOPE_RECT(X, Y)			CRect(X, Y, X + VECTORSCOPE_DIAMETER, Y + VECTORSCOPE_DIAMETER)
#define VECTORSCOPE_BACKGROUND_WIDTH	300
#define VECTORSCOPE_BACKGROUND_HEIGHT	300
#define VECTORSCOPE_INTERVAL_WIDTH		22
#define VECTORSCOPE_INTERVAL_HEIGHT		22
#define VECTORSCOPE_BACKGROUND_RECT		CRect(0, 0, VECTORSCOPE_BACKGROUND_WIDTH, VECTORSCOPE_BACKGROUND_HEIGHT)

#define VECTORSCOPE_CENTER_X			(VECTORSCOPE_BACKGROUND_WIDTH / 2)
#define VECTORSCOPE_CENTER_Y			(VECTORSCOPE_BACKGROUND_HEIGHT / 2)
#define VECTORSCOPE_COLOR_POINT			6

// Magenta, Red, Yellow, Green, Cyan, Blue
// Reference by 75% YCbCr Color Bar 
const double COLOR_DEGREE[VECTORSCOPE_COLOR_POINT] = {49.46, 102.75, 174.56, 229.46, 282.75, 354.56};
const double COLOR_SCALE[VECTORSCOPE_COLOR_POINT] = {0.78, 0.67, 0.66, 0.78, 0.67, 0.66};
const CString COLOR_NAME[VECTORSCOPE_COLOR_POINT] = {_T("Mg"), _T("R"), _T("Y"), _T("G"), _T("Cy"), _T("B")};

#define WAVEFORM_WIDTH					320
#define WAVEFORM_HEIGHT					BIT_FREQUENCY
#define WAVEFORM_RECT(X, Y)				CRect(X, Y, X + WAVEFORM_WIDTH, Y + WAVEFORM_HEIGHT)
#define WAVEFORM_BACKGROUND_WIDTH		360
#define WAVEFORM_BACKGROUND_HEIGHT		300
#define WAVEFORM_INTERVAL_WIDTH			20
#define WAVEFORM_INTERVAL_HEIGHT		22
#define WAVEFORM_BACKGROUND_RECT		CRect(0, 0, WAVEFORM_BACKGROUND_WIDTH, WAVEFORM_BACKGROUND_HEIGHT)

#define WAVE_GRADATION_POINT			5
const CString WAVE_GRAD[WAVE_GRADATION_POINT] = {_T("235"), _T("180"), _T("125"), _T("70"), _T("16")};

#define SUBWINDOW_RECT					CRect(0, 0, VECTORSCOPE_BACKGROUND_WIDTH + WAVEFORM_BACKGROUND_WIDTH, VECTORSCOPE_BACKGROUND_HEIGHT)
#define SUBIMAGE_HEIGHT					WAVEFORM_BACKGROUND_HEIGHT	// WAVEFORM HEIGHT == VECTOR HEIGHT

// ========================================================================================
class CWaveformImgOperation
{
public:
	CWaveformImgOperation(void);
	~CWaveformImgOperation(void);

public:
	void	InitWaveformDraw(void);
	
	void	OnDrawWaveformBackGnd(CDC *pDC, CRect *r);
	BYTE*	GetWaveform(BYTE *imgColor, int imgWidth, int imgHeight, int scale);
	CDC*	OnDrawWaveform(BYTE *imgColor, int imgWidth, int imgHeight, int scale);

	void	OnDrawVectorScopeBackGnd(CDC *pDC, CRect *r);
	BYTE*	GetVectorScope(BYTE *imgCb, BYTE *imgCr, int imgWidth, int imgHeight, int scale);
	CDC*	OnDrawVectorScope(BYTE *imgCb, BYTE *imgCr, int imgWidth, int imgHeight, int scale);

	
private:
// Buffer
	BYTE	*m_WaveformBuffer, *m_VectorScopeBuffer;
	short	*m_LumaFreqBuffer, *m_ChromaFreqBuffer;

// Initial Function & variable
	bool	CreateColorTable(void);
	void	CreateBITMAPHEADER(int width, int height);
	bool	bColorTableDone;
	BITMAPINFO	m_BmpInfo;

// Color Frequency of A image Calculate
	short*	CalculateWaveform(BYTE *buffer, int width, int height);
	short*	CalculateVectorScope(BYTE *cb, BYTE *cr, int width, int height);
	
// Background brush & pen
	CPen	m_Pen;
	CBrush	m_Brush;
	CFont	m_Font;
	
	int		Y[256], CbToB[256], CbToG[256], CrToR[256], CrToG[256];
};

	
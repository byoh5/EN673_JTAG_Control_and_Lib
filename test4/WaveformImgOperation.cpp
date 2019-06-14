#include "StdAfx.h"
#include "WaveformImgOperation.h"
#include <math.h>

//#include <afxwin.h>
#include <atlstr.h> 
#include <Wingdi.h>



double deg2rad(double degree)
{
    return degree*(PI / 180.0);
}

__forceinline unsigned char valClip(const int x) {
    return (x < 0) ? 0 : ((x > 0xFF) ? 0xFF : (unsigned char)x);
}

CWaveformImgOperation::CWaveformImgOperation(void)
{
 
    m_WaveformBuffer = new BYTE[WAVEFORM_WIDTH * WAVEFORM_HEIGHT * 4];
    m_VectorScopeBuffer = new BYTE[VECTORSCOPE_DIAMETER * VECTORSCOPE_DIAMETER * 4];

    m_LumaFreqBuffer = new short[WAVEFORM_WIDTH * WAVEFORM_HEIGHT];
    m_ChromaFreqBuffer = new short[VECTORSCOPE_DIAMETER * VECTORSCOPE_DIAMETER];

    m_ColorTable_R = new BYTE[BIT_FREQUENCY * BIT_FREQUENCY];
    m_ColorTable_G = new BYTE[BIT_FREQUENCY * BIT_FREQUENCY * BIT_FREQUENCY];
    m_ColorTable_B = new BYTE[BIT_FREQUENCY * BIT_FREQUENCY];

    bColorTableDone = false;
}


CWaveformImgOperation::~CWaveformImgOperation(void)
{
 
    if (m_ColorTable_R)		{ delete[] m_ColorTable_R;		m_ColorTable_R = NULL; }
    if (m_ColorTable_G)		{ delete[] m_ColorTable_G;		m_ColorTable_G = NULL; }
    if (m_ColorTable_B)		{ delete[] m_ColorTable_B;		m_ColorTable_B = NULL; }

    if (m_WaveformBuffer)	{ delete[] m_WaveformBuffer; 	m_WaveformBuffer = NULL; }
    if (m_VectorScopeBuffer)	{ delete[] m_VectorScopeBuffer;	m_VectorScopeBuffer = NULL; }

    if (m_LumaFreqBuffer)	{ delete[] m_LumaFreqBuffer;		m_LumaFreqBuffer = NULL; }
    if (m_ChromaFreqBuffer)	{ delete[] m_ChromaFreqBuffer;	m_ChromaFreqBuffer = NULL; }
}

// ====================================================================================================
// Initial
// ====================================================================================================
void CWaveformImgOperation::InitWaveformDraw(int imgWidth, int imgHeight){
  //  CreateBITMAPHEADER(imgWidth, imgHeight);
    if (!bColorTableDone) bColorTableDone = CreateColorTable();
}

// Waveform & Vectorscope Graph Color Table Create
bool CWaveformImgOperation::CreateColorTable(void){
    int i, j, k;
    BYTE *pt_R = m_ColorTable_R;
    BYTE *pt_G = m_ColorTable_G;
    BYTE *pt_B = m_ColorTable_B;

    for (i = 0; i<256; i++){
        Y[i] = i << 8;
        CbToB[i] = 454 * (i - 128);
        CbToG[i] = 88 * (i - 128);
        CrToR[i] = 359 * (i - 128);
        CrToG[i] = 183 * (i - 128);
    }

    for (i = 0; i<256; i++){
        for (j = 0; j<256; j++){
            *pt_R++ = valClip((Y[i] + CrToR[j] + 128) >> 8);
            *pt_B++ = valClip((Y[i] + CbToB[j] + 128) >> 8);
            for (k = 0; k<256; k++){
                *pt_G++ = valClip((Y[i] - CbToG[j] - CrToG[k] + 128) >> 8);
            }
        }
    }
    return true;
}

// ====================================================================================================
// Waveform Create Operation
// ====================================================================================================
// Calculate Color Histogram
short* CWaveformImgOperation::CalculateWaveform(BYTE *buffer, int width, int height){
    int scaleValue = width / WAVEFORM_WIDTH;
    unsigned int value;
    short scaledWidth;

    BYTE *inputData = buffer;

    // Initialize
    memset(m_LumaFreqBuffer, 0, WAVEFORM_WIDTH * WAVEFORM_HEIGHT * sizeof(short));

    for (int i = 0; i<height; i++){
        for (int j = 0; j<width; j++){
            scaledWidth = j / scaleValue;
            value = (int)inputData[i * width + j];
            m_LumaFreqBuffer[value * WAVEFORM_WIDTH + scaledWidth]++;
        }
    }
    return m_LumaFreqBuffer;
}

// Waveform Draw & Byte Array Out
BYTE* CWaveformImgOperation::GetWaveform(BYTE *imgColor, int imgWidth, int imgHeight, int scale, int ch){
    int val;
    short *lumaData = CalculateWaveform(imgColor, imgWidth, imgHeight);

    BYTE *waveData = m_WaveformBuffer;

    for (int i = 0; i<WAVEFORM_HEIGHT; i++){
        for (int j = 0; j<WAVEFORM_WIDTH; j++){
            val = lumaData[i * WAVEFORM_WIDTH + j];
            if (val >(int) (255 / scale)){
                *waveData++ = 0xFF;
                *waveData++ = 0xFF;
                *waveData++ = 0xFF;
                *waveData++ = 0;
            }
            else{
                *waveData++ = (ch == 0) ? (int)val * scale : 0;
                *waveData++ = (ch == 1) ? (int)val * scale : 0; // frequency * brightness
                *waveData++ = (ch == 2) ? (int)val * scale : 0;
                *waveData++ = 0;
            }
        }
    }
    return m_WaveformBuffer;
}

// ====================================================================================================
// VectorScope Create Operation
// ====================================================================================================
// Background Draw

short* CWaveformImgOperation::CalculateVectorScope(BYTE *cb, BYTE *cr, int width, int height){
    short crValue, cbValue;

    // Cb & Cr Frequency Check (in Image)
    memset(m_ChromaFreqBuffer, 0, VECTORSCOPE_DIAMETER * VECTORSCOPE_DIAMETER * sizeof(short));

    for (int i = 0; i<height; i++){
        for (int j = 0; j<width; j++){
            crValue = (short)cr[i * width + j];
            cbValue = (short)cb[i * width + j];
            m_ChromaFreqBuffer[crValue * VECTORSCOPE_DIAMETER + cbValue]++;
        }
    }
    return m_ChromaFreqBuffer;
}

// Vectorscope Draw & Byte Array Out
BYTE* CWaveformImgOperation::GetVectorScope(BYTE *imgCb, BYTE *imgCr, int imgWidth, int imgHeight, int scale){
    short crValue, cbValue, yValue;
    unsigned int val;
    short *chromaData = CalculateVectorScope(imgCb, imgCr, imgWidth , imgHeight);

    BYTE *vectorData = m_VectorScopeBuffer;
    for (int i = 0; i<VECTORSCOPE_DIAMETER; i++){			// Cr
        for (int j = 0; j<VECTORSCOPE_DIAMETER; j++){		// Cb
            val = chromaData[i * VECTORSCOPE_DIAMETER + j] * scale; // frequency * brightness

            yValue = (val<16) ? 16 : (val>235) ? 235 : val;
            cbValue = (j<16) ? 16 : (j>240) ? 240 : j;
            crValue = (i<16) ? 16 : (i>240) ? 240 : i;

            if (val > 1){
                *vectorData++ = m_ColorTable_R[yValue * VECTORSCOPE_DIAMETER + cbValue];
                *vectorData++ = m_ColorTable_G[yValue * VECTORSCOPE_DIAMETER * VECTORSCOPE_DIAMETER + cbValue * VECTORSCOPE_DIAMETER + crValue];
                *vectorData++ = m_ColorTable_B[yValue * VECTORSCOPE_DIAMETER + crValue];
            //    *vectorData++ = 0;
            }
            else{
                *vectorData++ = 0;
                *vectorData++ = 0;
                *vectorData++ = 0;
            //    *vectorData++ = 0;
            }
        }
    }
    return m_VectorScopeBuffer;
}


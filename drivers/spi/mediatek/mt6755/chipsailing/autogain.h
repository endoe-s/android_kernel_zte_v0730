#ifndef AUTOGAIN_H
#define AUTOGAIN_H

#define Is_Use_z_Gain 1

//是否是通过改变第一极
#define Change_One_Gain 1

// 建议初始配置  fc2e=04f8  fc30=260
#define DEFAULT_GAIN_VALUE 0x04F8
#define DEFAULT_BASE_VALUE 0x0260

#ifdef __cplusplus
extern "C"
{
#endif

int ChipSailing_AutoGain(unsigned char * data, int nWidth, int nHeight, int *pGainBase, int *pNewGainBase );

#ifdef __cplusplus
}
#endif


#endif
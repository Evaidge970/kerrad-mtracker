#pragma once
#include "hardware_interface.h"
#include <stdint.h>
#include <math.h>

#define M_PI   3.14159265358979323846

inline float sign(float f)
{
    return (f < 0) ? -1 : 1;
}

inline float AngleLocalCorrection(float th)
{
	if (th >= M_PI)
		th -= 2*M_PI;
	else if (th < -M_PI)
		th += 2*M_PI;

	return th;
}


inline uint32_t Get32bitDataFromBuf(uint16_t * &p)
{
	uint32_t i;
	i = (0xFF & *(p)) | (0xFF & *(p+1))<<8 | ((Uint32)(0xFF & *(p+2)))<<16 | ((Uint32)(0xFF & *(p+3)))<<24;
	p += 4;
	return i;
}

inline uint16_t Get16bitDataFromBuf(uint16_t * &p)
{
	uint16_t i;
	i = (0xFF & *(p)) | (0xFF & *(p+1))<<8;
	p += 2;
	return i;
}

inline float GetFloatDataFromBuf(uint16_t * &p)
{
	uint32_t i;
	i = (0xFF & *(p)) | (0xFF & *(p+1))<<8 | ((Uint32)(0xFF & *(p+2)))<<16 | ((Uint32)(0xFF & *(p+3)))<<24;
	p += 4;
	return *((float*)&i);
}

inline void StoreFloatDataInBuf(uint16_t * &p, float m)
{
	Uint32 n = *((Uint32*)&m);
	p[0] = n & 0x00FF;
	p[1] = (n>>8) & 0x00FF;
	p[2] = (n>>16) & 0x00FF;
	p[3] = (n>>24) & 0x00FF;
	p += 4;
}

inline void Store32bitDataInBuf(uint16_t * &p, uint32_t n)
{
	p[0] = n & 0x00FF;
	p[1] = (n>>8) & 0x00FF;
	p[2] = (n>>16) & 0x00FF;
	p[3] = (n>>24) & 0x00FF;
	p += 4;
}

inline void Store16bitDataInBuf(uint16_t * &p, uint16_t n)
{
	p[0] = n & 0x00FF;
	p[1] = (n>>8) & 0x00FF;
	p += 2;
}

inline void Store8bitDataInBuf(uint16_t * &p, uint16_t n)
{
	p[0] = n;
	p++;
}

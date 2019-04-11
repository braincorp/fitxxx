/*
 * lim.cpp
*/

#include <stdio.h>
#include <string.h>

#include "LIM/lim.h"

unsigned int LIM_CheckSum(LIM_HEAD * _lim)
{
	unsigned int checksum = _lim->TAG;
	checksum ^= _lim->VER;
	checksum ^= _lim->nCode;
	checksum ^= _lim->nLIMLen;

	int n;
	for (n = 0; n < LIM_DATA_LEN; n++)
		checksum ^= _lim->Data[n];

	return checksum;
}

void* LIM_ExData(LIM_HEAD* _lim)
{
	return (void*)(_lim + 1);
}

LMD_INFO* LMD_Info(LIM_HEAD* _lim)
{
	if ((LIM_CODE_LMD != _lim->nCode) && (LIM_CODE_LMD_RSSI != _lim->nCode))
		return NULL;
	else
		return (LMD_INFO*)LIM_ExData(_lim);
}
LMD_D_Type* LMD_D(LIM_HEAD* _lim)
{
	if ((LIM_CODE_LMD != _lim->nCode) && (LIM_CODE_LMD_RSSI != _lim->nCode))
		return NULL;
	else
		return (LMD_D_Type*)(LMD_Info(_lim) + 1);
}
LMD_D_RSSI_Type* LMD_D_RSSI(LIM_HEAD* _lim)
{
	if (LIM_CODE_LMD_RSSI != _lim->nCode)
		return NULL;
	else
	{
		return (LMD_D_RSSI_Type*)(LMD_D(_lim) + LMD_Info(_lim)->nMDataNum);
	}
}

bool LIM_Pack(LIM_HEAD*& _lim, unsigned int _cid, unsigned int _code, unsigned int* _data, unsigned int _ext_data_len, void* _ext_data)
{
	int s = sizeof(LIM_HEAD)+_ext_data_len;
	_lim = (LIM_HEAD*) new unsigned char[s];
	if (NULL == _lim)
		return false;

	_lim->TAG = LIM_TAG;
	_lim->VER = LIM_VER;
	_lim->nCID = _cid;
	_lim->nCode = _code;
	_lim->nLIMLen = s;

	if (NULL != _data)
		memcpy((void*)_lim->Data, (void*)_data, sizeof(_lim->Data));
	else
		memset((void*)_lim->Data, 0, sizeof(_lim->Data));

	_lim->CheckSum = LIM_CheckSum(_lim);

	if (NULL != _ext_data)
	{
		void* ext_data = LIM_ExData(_lim);
		memcpy(ext_data, _ext_data, _ext_data_len);
	}

	return true;
}
bool LIM_Copy(LIM_HEAD*& _dlim, LIM_HEAD* _slim)
{
	if (NULL == _slim)
		return false;

	int s = _slim->nLIMLen;

	_dlim = (LIM_HEAD *)new unsigned char[s];
	if (NULL == _dlim)
		return false;

	memcpy((void*)_dlim, (void*)_slim, s);

	return true;
}
void LIM_Release(LIM_HEAD*& _lim)
{
	delete (unsigned char*)_lim;
	_lim = NULL;
}

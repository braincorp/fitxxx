/*
 * lim.h
 * laser hardware info
*/
#ifndef __LIM_H__
#define __LIM_H__

#if defined (LINUX)

#ifndef STRUCTPACKED
#define STRUCTPACKED //__attribute__((__packed__))
#endif

#elif defined(WIN32)||defined(WINNT)

#ifndef STRUCTPACKED
#define STRUCTPACKED
#endif

#endif

#define LIM_DT_IP      "237.1.1.200"  // Laser IP
#define LIM_DT_PORT    2111           // Port

/*
	LIM(Lidar Interaction Message)
	LIM ::= LIM_HEAD + Extended_Data
	Remark:
	1) Extended_Data : optional.
	
	Extended_Data:
	1) Lidar Measurement Data (LMD) Structure:
	   LMD ::= LMD_INFO + Data_Array
	   Remark:
	   1) Lidar mesaurement: distance value, unit: cm;
	   2) Data_Array: array of measurement; type: LMD_D_Type; length: LMD_INFO.nMDataNum.
*/




#define LIM_USER_PORT    2112           // User port

/*
	LIM报文信息
*/
#define LIM_TAG          0xF5EC96A5     // LIM Tag
#define	LIM_VER          0x01000000     
#define LIM_DATA_LEN     4              

typedef struct
{
	unsigned int TAG;                   
	unsigned int VER;                  
	unsigned int nCID;               
	unsigned int nCode;                 
	unsigned int Data[LIM_DATA_LEN];    
	unsigned int nLIMLen;   
	unsigned int CheckSum;         
} LIM_HEAD;



#define LIM_CODE_LDBCONFIG          111      
#define LIM_CODE_START_LDBCONFIG    110      
#define LIM_CODE_STOP_LDBCONFIG     112     
#define LIM_CODE_GET_LDBCONFIG      114      

//LIM Code.
#define LIM_CODE_HB                10            
#define LIM_CODE_HBACK             11            
#define LIM_CODE_LMD               901             
#define LIM_CODE_LMD_RSSI          911            
#define LIM_CODE_START_LMD         1900           
#define LIM_CODE_STOP_LMD          1902            
#define LIM_CODE_ALARM             9001           
#define LIM_CODE_DISALARM          9003           

#define LIM_CODE_FMSIG_QUERY       1912       
#define LIM_CODE_FMSIG             1911       


#define LIM_CODE_IOREAD            1920         
#define LIM_CODE_IOSET             1922          
#define LIM_CODE_IOSET_RELEASE     1924        
#define LIM_CODE_IOSTATUS          1921          


// LMD_INFO
typedef struct
{
	unsigned int nRange;            
	int          nBAngle;           
	int          nEAngle;          
	unsigned int nAnglePrecision; 
	unsigned int nRPM;              
	unsigned int nMDataNum;         
} LMD_INFO;


typedef unsigned short LMD_D_Type;            


typedef unsigned short LMD_D_RSSI_Type;      


#define LIM_DATA_ALARMCODE_INTERNAL            1      
#define LIM_DATA_ALARMCODE_Occluded            101    
#define LIM_DATA_ALARMCODE_High_Temperature    1001  
#define LIM_DATA_ALARMCODE_Low_Temperature     1002  


#define ULDINI_MAX_ATTR_STR_LEN                0x20    

// laser configuration
typedef struct
{

	char szType[ULDINI_MAX_ATTR_STR_LEN];         
	char szManufacturer[ULDINI_MAX_ATTR_STR_LEN];  
	char szReleaseDate[ULDINI_MAX_ATTR_STR_LEN];  
	char szSerialNo[ULDINI_MAX_ATTR_STR_LEN];     


	char szMAC[ULDINI_MAX_ATTR_STR_LEN];    
	char szIP[ULDINI_MAX_ATTR_STR_LEN];     
	char szMask[ULDINI_MAX_ATTR_STR_LEN];  
	char szGate[ULDINI_MAX_ATTR_STR_LEN];   
	char szDNS[ULDINI_MAX_ATTR_STR_LEN];   


	int nMR;        // measurement range
	int nESAR;     
	int nESA[2];  
	int nSAR;      
	int nSA[2];     
	int nSAV;      
	int nSAP;      
	int nPF;      

}ULDINI_Type;

#ifdef __cplusplus
extern "C"{
#endif

/*
	LIM utilities.
*/

/*
	LIM_CheckSum:
	Function: calculating the checksum of LIM.
	Return value: checksum of _lim.
	Remark:
	1) Checksum calculation for LIM creating & sending;
	2) Checksum checking for received LIM.
*/
unsigned int LIM_CheckSum(LIM_HEAD * _lim);

/*
LIM_ExData:
Function: memory address of Extended_Data of LIM.
Return value: pointer to Extended_Data.
Remark:
1) When a LIM has extended data, e.g., LMD LIM, use LIM_ExData to obtain the memory address;
2) LIM_HEAD and Extended_Data locate in continuous memory, so the address of Extended_Data equals to (void*)(_lim + 1).
*/
void* LIM_ExData(LIM_HEAD* _lim);

/*
LMD_Info & LMD_D:
Function: memory address of LMD_INFO & measurement data array in LIM.
Return value: pointer to LMD_INFO & measurement data array.
Remark:
1) For an LMD LIM, address of LMD_INFO equals to (LMD_INFO*)LIM_ExData(_lim);
2) The whole LMD LIM locates in continuous memory, so the address of measurement data equals to (LMD_D_Type*)(LMD_Info(_lim) + 1).
*/
LMD_INFO* LMD_Info(LIM_HEAD* _lim);
LMD_D_Type* LMD_D(LIM_HEAD* _lim);
LMD_D_RSSI_Type* LMD_D_RSSI(LIM_HEAD* _lim);

/*
LIM_Pack:
Function: compose a LIM.
return value: true / false
Remark:
1) composed LIM is returned in _lim.
*/
bool LIM_Pack(LIM_HEAD*& _lim, unsigned int _cid, unsigned int _code, unsigned int* _data = NULL, unsigned int _ext_data_len = 0, void* _ext_data = NULL);
/*
LIM_Copy:
Function: copy a LIM.
return value: true / false
Remark:
1) LIM copied from _slim is returned in _dlim.
*/
bool LIM_Copy(LIM_HEAD*& _dlim, LIM_HEAD* _slim);
/*
LIM_Release:
Function: release a LIM.
Remark:
1) memory of _lim is released, and _lim is cleared to NULL.
*/
void LIM_Release(LIM_HEAD*& _lim);

#ifdef __cplusplus
}
#endif

#endif

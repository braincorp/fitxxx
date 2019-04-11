/*
 * FITXXX.cpp
 * Author: Zhigang Wu
 * Date: 2018-01-12
*/

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>

#include "LIM/lim.h"
#include "FITXXX/FITXXX.h"
#include "console_bridge/console.h"

FITXXX::FITXXX() 
: connected_(false)
, loaded_config_(false)
{
}

FITXXX::~FITXXX()
{
}

void FITXXX::connect(std::string host_ip, int port)
{
  if (!connected_)
  {
    std::cout <<"Creating non-blocking socket." << std::endl;
    socket_fd_ = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socket_fd_)
    {
      struct sockaddr_in stSockAddr;
      stSockAddr.sin_family = PF_INET;
      stSockAddr.sin_port = htons(port);
      inet_pton(AF_INET, host_ip.c_str(), &stSockAddr.sin_addr);

      std::cout << "Connecting socket to laser."  << std::endl;
      int ret = ::connect(socket_fd_, (struct sockaddr *) &stSockAddr, sizeof(stSockAddr));

      if (ret == 0)
      {
        connected_ = true;
        std::cout << "Connected succeeded." << std::endl;
      }
    }
  }
}

bool FITXXX::initializedLaserConfig()
{
  return loaded_config_;
}
void FITXXX::disconnect()
{
  if (connected_)
  {
    close(socket_fd_);
    connected_ = false;
  }
}

bool FITXXX::isConnected()
{
  return connected_;
}

void FITXXX::startMeas()
{
  int _cid = 1;
  LIM_HEAD * lim = NULL;
  LIM_Pack(lim, _cid, LIM_CODE_START_LMD, NULL);

  write(socket_fd_, lim, lim->nLIMLen);

  LIM_Release(lim);
}

void FITXXX::sendHB()
{
  int _cid = 1;
  LIM_HEAD * lim = NULL;
  LIM_Pack(lim, _cid, LIM_CODE_HB, NULL);

  write(socket_fd_, lim, lim->nLIMLen);

  LIM_Release(lim);
}


bool FITXXX::getConfig()
{
  int _cid = 1;
  LIM_HEAD *lim = NULL;
  LIM_Pack(lim, _cid, LIM_CODE_GET_LDBCONFIG, NULL);

  write(socket_fd_, lim, lim->nLIMLen);
  LIM_Release(lim);
}


bool FITXXX::LIM_CODE_LMD_Decoding(LIM_HEAD* lim, sensor_msgs::LaserScan *scan_data)
{
  if (lim->nCode != LIM_CODE_LMD)
    return false;
  LMD_INFO* lmd_info = LMD_Info(lim); 
  LMD_D_Type* lmd = LMD_D(lim);  

  scan_data->angle_increment = lmd_info->nAnglePrecision *M_PI / (1000.*180);
  scan_data->ranges.resize(lmd_info->nMDataNum);

  for(int i=0; i<lmd_info->nMDataNum; i++)
  {
    scan_data->ranges[i] = lmd[i]/100.;
	if(scan_data->ranges[i] > scan_data->range_max)
    {
      scan_data->ranges[i] = 0;
    }
  }
}

// LIM_CODE_LDBCONFIG LIM
bool FITXXX::LIM_CODE_LDBCONFIG_Decoding(LIM_HEAD* lim, ULDINI_Type *uld)
{
  if (lim->nCode != LIM_CODE_LDBCONFIG)
  {
    std::cout << "Error: config packet decode failed.." << std::endl;
    return false;
  }

  //uld = (ULDINI_Type*)LIM_ExData(lim);
  *uld = *(ULDINI_Type*)LIM_ExData(lim);
  std::cout << "Config packet decoded...." << std::endl;
  return true;
}


bool FITXXX::packetDecodeExt(ULDINI_Type *uld)
{
  fd_set rfds;    // file description set(fds)
  FD_ZERO(&rfds);   // empty fds
  FD_SET(socket_fd_, &rfds);  // add fd in fds
  //FD_CLR(fdset *fdset);     // delete one fd in fds
  //FD_ISSET(int fd, fd_set *fdset); //test if certain fd in such set

  // Block a total of up to 100ms waiting for more data from the laser.
  while (1)
  {
    // Would be great to depend on linux's behaviour of updating the timeval, but unfortunately
    // that's non-POSIX (doesn't work on OS X, for example).
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;

    logDebug("entering select()", tv.tv_usec);
    //(max file description value +1, read fds, write fds, error exit fds, time interval)
    int retval = select(socket_fd_ + 1, &rfds, NULL, NULL, &tv); // if receive fd return 1, else return 0

    //std::cout << "from select() returned: " << retval << std::endl;
    if (retval)
    {
      buffer_.readFrom(socket_fd_);
      return true;

    }
    else
    {
      //std::cout << "Error: packet decode failed.." << std::endl;
      // Select timed out or there was an fd error.
      return false;
    }
  }
}

bool FITXXX::GetALim(ULDINI_Type *uld, sensor_msgs::LaserScan *scan_data)
{
  char* buffer_data = buffer_.getNextBuffer_LIM();

  if (buffer_data)
  {

    LIM_HEAD *lim = (LIM_HEAD*)buffer_data;
    unsigned int checksum = LIM_CheckSum(lim);
    if (checksum != lim->CheckSum)
    //校验数据是否正确
    {
      printf("\tLIM checksum error!\n");
      return false;
    }
    if (LIM_CODE_LDBCONFIG == lim->nCode)
    {
      if(LIM_CODE_LDBCONFIG_Decoding(lim, uld))
      {
        loaded_config_ = true;
      }
    }

    if ((LIM_CODE_LMD == lim->nCode)&&(loaded_config_==true))
    {
      LIM_CODE_LMD_Decoding(lim, scan_data);
    }

    //parseScanData(buffer_data, scan_data);
    buffer_.popLastBuffer();
    return true;
  }else
    return false;
}

void FITXXX::stopMeas()
{
  int _cid = 1;
  LIM_HEAD * lim = NULL;
  LIM_Pack(lim, _cid, LIM_CODE_STOP_LMD, NULL);

  write(socket_fd_, lim, lim->nLIMLen);

  LIM_Release(lim);
}

/*
void FITXXX::areaAlarm()
{
  int _cid = 1;
  LIM_HEAD * lim = NULL;
  LIM_Pack(lim, _cid, LIM_CODE_FMSIG_QUERY,NULL);
  lim->Data[0] = 0;
  write(socket_fd_, lim, lim->nLIMLen);

  LIM_Release(lim);
}
*/

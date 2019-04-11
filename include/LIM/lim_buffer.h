/*
 * LIM_buffer.h
*/

#ifndef LIM_BUFFER_H_
#define LIM_BUFFER_H_

#include "console_bridge/console.h"
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include "LIM/lim.h"

#define LIM_BUFFER_SIZE 60000
#define LIM_TAG0 0xa5
#define LIM_TAG1 0x96
#define LIM_TAG2 0xec
#define LIM_TAG3 0xf5

class LIMBuffer
{
public:
  LIMBuffer() : total_length_(0), end_of_first_message_(0)
  {
  }
  // Reading from tcp.
  void readFrom(int fd)
  {
    int ret = read(fd, buffer_ + total_length_, sizeof(buffer_) - total_length_);

    if (ret > 0)
    {
      total_length_ += ret;
      logDebug("Read %d bytes from fd, total length is %d.", ret, total_length_);
    }
    else
    {
      logWarn("Buffer read() returned error.  %d  ret %d",total_length_,ret);
    }
  }
  // Find Lim pack start.
   char* findLIMPktStart()
  {
    unsigned char *dbuffer_ = (unsigned char*)buffer_;
    for( std::size_t i=0; i<total_length_-4; i++)
    {
        if(dbuffer_[i] == LIM_TAG0
           && (dbuffer_[i+1]) == LIM_TAG1
           && (dbuffer_[i+2]) == LIM_TAG2
           && (dbuffer_[i+3]) == LIM_TAG3 )
        {
            return buffer_+i;
        }
    }
    return NULL;
  }
  
  // Find Lim pack end.
     char* findLIMPktEnd() 
  {
  
    if( total_length_< 40 )
        return NULL;
        
    LIM_HEAD *lim = (LIM_HEAD*)buffer_;
    if( total_length_ < lim->nLIMLen )
        return NULL;
    
      
    
    return buffer_ + lim->nLIMLen -1;
  }
  
   char* getNextBuffer_LIM()
  {
    if (total_length_ == 0)
    {
      // Buffer is empty, no scan data present.
      logDebug("Empty buffer, nothing to return.");
      return NULL;
    }

  
    char* start_of_message = findLIMPktStart();
    if (start_of_message == NULL)
    {
      // Not foundt LIM TAG, reset buffer.
      logWarn("No STX found, dropping %d bytes from buffer.", total_length_);
      total_length_ = 0;
    }
    else if (buffer_ != start_of_message)
    {
      // Shift buffer if start found.
      logWarn("Shifting buffer, dropping %d bytes, %d bytes remain.",
              (start_of_message - buffer_), total_length_ - (start_of_message - buffer_));
      shiftBuffer(start_of_message);
    }

    // Find Lim pack end.
    end_of_first_message_ = findLIMPktEnd();
    if (end_of_first_message_ == NULL)
    {
      // Return null if no end.
      logDebug("No ETX found, nothing to return.");
      return NULL;
    }

    return buffer_;
  }

 
  void popLastBuffer()
  {
    if (end_of_first_message_)
    {
      shiftBuffer(end_of_first_message_ + 1);
      end_of_first_message_ = NULL;
    }
  }

private:
  void shiftBuffer(char* new_start)
  {
    // Shift buffer.
    uint16_t remaining_length = total_length_ - (new_start - buffer_);

    if (remaining_length > 0)
    {
      memmove(buffer_, new_start, remaining_length);
    }
    total_length_ = remaining_length;
  }

  char buffer_[LIM_BUFFER_SIZE];
  uint16_t total_length_;

  char* end_of_first_message_;
};

#endif  // LIM_BUFFER_H_

#include "TimeUtils.h"
#include <Homie.h>

long getCurrentTime()
{
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  return tv_now.tv_sec;
}

int getCurrentHour()
{
  struct tm info;
  time_t now;
  time(&now);
  localtime_r(&now, &info);
  return info.tm_hour;
}
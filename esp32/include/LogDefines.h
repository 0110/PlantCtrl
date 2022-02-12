#pragma once

#define LOG_LEVEL_ERROR 0
#define LOG_LEVEL_WARN 1
#define LOG_LEVEL_INFO 2
#define LOG_LEVEL_DEBUG 3


#define LOG_TANKSENSOR_FAIL_DETECT "Failed to detect and initialize distance sensor!"
#define LOG_TANKSENSOR_FAIL_DETECT_CODE -1

#define LOG_BACKUP_SUCCESSFUL "Backup sucessful"
#define LOG_BACKUP_SUCCESSFUL_CODE 1

#define LOG_BACKUP_FAILED "Backup error"
#define LOG_BACKUP_FAILED_CODE -2

#define LOG_PUMP_BUTNOTANK_MESSAGE "Want to pump but no water"
#define LOG_PUMP_BUTNOTANK_CODE -3

#define LOG_HARDWARECOUNTER_ERROR_MESSAGE "PCNR returned error"
#define LOG_HARDWARECOUNTER_ERROR_CODE -4

#define LOG_SENSORMODE_UNKNOWN "Unknown sensor mode requested"
#define LOG_SENSORMODE_UNKNOWN_CODE -5

#define LOG_PUMP_AND_DOWNLOADMODE "Download mode, ignoring pump request"
#define LOG_PUMP_AND_DOWNLOADMODE_CODE 2

//msg is dynamic defined
#define LOG_PUMP_INEFFECTIVE -4
#define LOG_PUMP_STARTED_CODE 10
#define LOG_DEBUG_CODE 1001
#define LOG_SLEEP_NIGHT 100
#define LOG_SLEEP_DAY 101
#define LOG_SLEEP_CYCLE 102
#define LOG_MISSING_PUMP -4
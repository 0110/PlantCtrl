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

#define LOG_PUMP_AND_DOWNLOADMODE "Download mode, ignoring pump request"
#define LOG_PUMP_AND_DOWNLOADMODE_CODE 2

#define LOG_DEBUG_CODE 1001
#define LOG_NOPUMP_LOWLIGHT 100
#define LOG_NOPUMPS         101
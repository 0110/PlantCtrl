/**
 * @file WakeReason.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2020-11-28
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef WAKEUP_REASON_H
#define WAKEUP_REASON_H

#define WAKEUP_REASON_UNDEFINED 0
#define WAKEUP_REASON_TEMP1_CHANGE 2
#define WAKEUP_REASON_TEMP2_CHANGE 3
#define WAKEUP_REASON_BATTERY_CHANGE 4
#define WAKEUP_REASON_SOLAR_CHANGE 5
#define WAKEUP_REASON_RTC_MISSING 6
#define WAKEUP_REASON_TIME_UNSET 7
#define WAKEUP_REASON_MODE2_WAKEUP_TIMER 8


#define WAKEUP_REASON_MOIST_CHANGE 20   /**< <code>20-26</code> for plant0 to plant9  */
#define WAKEUP_REASON_PLANT_DRY 30      /**< <code>30-36</code> for plant0 to plant9  */

#endif
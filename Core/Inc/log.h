/*
 * dbg.h
 *
 *  Created on: Apr 4, 2023
 *      Author: hiseob
 */

#ifndef INC_LOG_H_
#define INC_LOG_H_

#include <stdio.h>

#ifndef _LOG_LEVEL_
#define _LOG_LEVEL_ (4)
#endif

#define _LOG_E_ (0)
#define _LOG_W_ (1)
#define _LOG_I_ (2)
#define _LOG_V_ (3)
#define _LOG_D_ (4)

#define LOGE(x, ...) \
  if(_LOG_LEVEL_ >= _LOG_E_)  printf("%10lu:E:%s:" x "\r\n", HAL_GetTick(), __func__, ##__VA_ARGS__)
#define LOGW(x, ...) \
  if(_LOG_LEVEL_ >= _LOG_W_)  printf("%10lu:W:%s:" x "\r\n", HAL_GetTick(), __func__, ##__VA_ARGS__)
#define LOGI(x, ...) \
  if(_LOG_LEVEL_ >= _LOG_I_)  printf("%10lu:I:%s:" x "\r\n", HAL_GetTick(), __func__, ##__VA_ARGS__)
#define LOGV(x, ...) \
  if(_LOG_LEVEL_ >= _LOG_V_)  printf("%10lu:V:%s:" x "\r\n", HAL_GetTick(), __func__, ##__VA_ARGS__)
#define LOGD(x, ...) \
  if(_LOG_LEVEL_ >= _LOG_D_)  printf("%10lu:D:%s:" x "\r\n", HAL_GetTick(), __func__, ##__VA_ARGS__)

#endif /* INC_LOG_H_ */

/**
 * @file main.c
 * @author qingyu
 * @brief 系统入口：调用 System_Startup() 启动架构层后空转
 * @version 0.1
 * @date 2026-08-01
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <zephyr/kernel.h>
#include "System_startup.h"

int main(void)
{
    System_Startup();
	
    while (1) 
	{ 
		k_msleep(1000); 
	}
}

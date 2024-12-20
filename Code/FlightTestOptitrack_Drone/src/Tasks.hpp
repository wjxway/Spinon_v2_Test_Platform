/**
 * @file Tasks.hpp
 * @brief User defined tasks to run
 */
#ifndef TASKS_HPP__
#define TASKS_HPP__

void Set_Throttle_Task(void *pvParameters);

void Send_FB_Task(void* pvParameters);

void PrintTask(void* pvParameters);

#endif
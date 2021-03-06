// Created by Brain Ofrim
#ifndef  __GLOBALS__
#define  __GLOBALS__
#include <ucos_ii.h>
#include <circular_buf_position.h>

extern OS_EVENT *RxDataAvailableSemaphore;
extern OS_EVENT *SonarDataAvailableSemaphore;
extern char* userMessage;
extern bool enable_sonar;
extern circular_buf_position* distance_buffer;

#endif

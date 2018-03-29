// Created by Brain Ofrim
#ifndef  __GLOBALS__
#define  __GLOBALS__
#include <ucos_ii.h>
#include "sonar.h"

extern OS_EVENT *RxDataAvailableSemaphore;
extern OS_EVENT *SonarDataAvailableSemaphore;
extern char* userMessage;
extern circular_buf* distance_buffer;
extern circular_buf* velocity_buffer;

#endif

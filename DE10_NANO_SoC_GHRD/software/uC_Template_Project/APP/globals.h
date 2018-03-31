// Created by Brain Ofrim
#ifndef  __GLOBALS__
#define  __GLOBALS__
#include <ucos_ii.h>
#include <circular_buf_position.h>
#include <circular_buf_velocity.h>

extern OS_EVENT *RxDataAvailableSemaphore;
extern OS_EVENT *SonarDataAvailableSemaphore;
extern char* userMessage;
extern circular_buf_position* distance_buffer;
extern circular_buf_velocity* velocity_buffer;

#endif

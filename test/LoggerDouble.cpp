#ifndef RSJ_2017_PICK_AND_PLACER_LOGGER_H
#define RSJ_2017_PICK_AND_PLACER_LOGGER_H

#include <ros/ros.h>
#include <string>
#include <iostream>

class Logger {
public:
  template<typename... Args> void DEBUG(const char* fmt, Args... args) {
  }

  template<typename... Args> void INFO(const char* fmt, Args... args) {
  }

  template<typename... Args> void WARN(const char* fmt, Args... args) {
  }

  template<typename... Args> void ERROR(const char* fmt, Args... args) {
  }

  template<typename... Args> void FATAL(const char* fmt, Args... args) {
  }
};

#endif //RSJ_2017_PICK_AND_PLACER_LOGGER_H

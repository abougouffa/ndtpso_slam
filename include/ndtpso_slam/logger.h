#ifndef LOGGER_H
#define LOGGER_H

#include "ndtpso_slam/config.h"

#if USE_LOGGER
#include <fstream>

class MyLogger {
private:
    static std::ofstream s_log_file;
    static std::string s_buffer[LOGGER_BUFFER_SIZE_LINES];
    static size_t s_current_item;
    static void s_save();
    MyLogger() {}

public:
    static bool init(const char* filename);
    static void write(std::string log_line);
    static void close();
};

#endif

#endif // LOGGER_H

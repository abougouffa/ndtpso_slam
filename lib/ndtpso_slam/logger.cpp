#include "ndtpso_slam/logger.h"

#if USE_LOGGER
bool MyLogger::init(const char *filename) {
  s_log_file = std::ofstream(filename, std::ofstream::out);
  s_current_item = 0;

  return static_cast<bool>(s_log_file);
}

void MyLogger::write(std::string log_line) {
  s_buffer[s_current_item] = log_line;
  s_current_item = (s_current_item + 1) % LOGGER_BUFFER_SIZE_LINES;

  if (0 == s_current_item) {
    s_save();
  }
}

void MyLogger::s_save() {
  for (size_t i = 0; i < s_current_item; ++i) {
    s_log_file << s_buffer[i];
  }

  s_current_item = 0;
  s_log_file.flush();
}

void MyLogger::close() {
  s_save();
  s_log_file.close();
}
#endif

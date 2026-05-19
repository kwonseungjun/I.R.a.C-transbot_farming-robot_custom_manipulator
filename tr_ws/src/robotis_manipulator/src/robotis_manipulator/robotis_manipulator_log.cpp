

#include "../../include/robotis_manipulator/robotis_manipulator_log.h"

void robotis_manipulator::log::print(STRING str, STRING color)
{
#if defined(__OPENCR__)
  DEBUG.print(str);
#else
       if(color == "RED")      printf(ANSI_COLOR_RED);
  else if(color == "GREEN")    printf(ANSI_COLOR_GREEN);
  else if(color == "YELLOW")   printf(ANSI_COLOR_YELLOW);
  else if(color == "BLUE")     printf(ANSI_COLOR_BLUE);
  else if(color == "MAGENTA")  printf(ANSI_COLOR_MAGENTA);
  else if(color == "CYAN")     printf(ANSI_COLOR_CYAN);
  printf("%s", str.c_str());
  printf(ANSI_COLOR_RESET);
#endif
}
void robotis_manipulator::log::print(STRING str, double data, uint8_t decimal_point, STRING color)
{
#if defined(__OPENCR__)
  DEBUG.print(str);
  DEBUG.print(data, decimal_point);
#else
       if(color == "RED")      printf(ANSI_COLOR_RED);
  else if(color == "GREEN")    printf(ANSI_COLOR_GREEN);
  else if(color == "YELLOW")   printf(ANSI_COLOR_YELLOW);
  else if(color == "BLUE")     printf(ANSI_COLOR_BLUE);
  else if(color == "MAGENTA")  printf(ANSI_COLOR_MAGENTA);
  else if(color == "CYAN")     printf(ANSI_COLOR_CYAN);
  printf("%s %.*lf", str.c_str(), decimal_point, data);
  printf(ANSI_COLOR_RESET);
#endif
}
void robotis_manipulator::log::print(const char* str, STRING color)
{
#if defined(__OPENCR__)
  DEBUG.print(str);
#else
       if(color == "RED")      printf(ANSI_COLOR_RED);
  else if(color == "GREEN")    printf(ANSI_COLOR_GREEN);
  else if(color == "YELLOW")   printf(ANSI_COLOR_YELLOW);
  else if(color == "BLUE")     printf(ANSI_COLOR_BLUE);
  else if(color == "MAGENTA")  printf(ANSI_COLOR_MAGENTA);
  else if(color == "CYAN")     printf(ANSI_COLOR_CYAN);
  printf("%s", str);
  printf(ANSI_COLOR_RESET);
#endif
}
void robotis_manipulator::log::print(const char* str, double data, uint8_t decimal_point, STRING color)
{
#if defined(__OPENCR__)
  DEBUG.print(str);
  DEBUG.print(data, decimal_point);
#else
       if(color == "RED")      printf(ANSI_COLOR_RED);
  else if(color == "GREEN")    printf(ANSI_COLOR_GREEN);
  else if(color == "YELLOW")   printf(ANSI_COLOR_YELLOW);
  else if(color == "BLUE")     printf(ANSI_COLOR_BLUE);
  else if(color == "MAGENTA")  printf(ANSI_COLOR_MAGENTA);
  else if(color == "CYAN")     printf(ANSI_COLOR_CYAN);
  printf("%s %.*lf", str, decimal_point, data);
  printf(ANSI_COLOR_RESET);
#endif
}


void robotis_manipulator::log::println(STRING str, STRING color)
{
#if defined(__OPENCR__)
  DEBUG.println(str);
#else
       if(color == "RED")      printf(ANSI_COLOR_RED);
  else if(color == "GREEN")    printf(ANSI_COLOR_GREEN);
  else if(color == "YELLOW")   printf(ANSI_COLOR_YELLOW);
  else if(color == "BLUE")     printf(ANSI_COLOR_BLUE);
  else if(color == "MAGENTA")  printf(ANSI_COLOR_MAGENTA);
  else if(color == "CYAN")     printf(ANSI_COLOR_CYAN);
  printf("%s\n", str.c_str());
  printf(ANSI_COLOR_RESET);
#endif
}
void robotis_manipulator::log::println(STRING str, double data, uint8_t decimal_point, STRING color)
{
#if defined(__OPENCR__)
  DEBUG.print(str);
  DEBUG.println(data, decimal_point);
#else
     if(color == "RED")      printf(ANSI_COLOR_RED);
  else if(color == "GREEN")    printf(ANSI_COLOR_GREEN);
  else if(color == "YELLOW")   printf(ANSI_COLOR_YELLOW);
  else if(color == "BLUE")     printf(ANSI_COLOR_BLUE);
  else if(color == "MAGENTA")  printf(ANSI_COLOR_MAGENTA);
  else if(color == "CYAN")     printf(ANSI_COLOR_CYAN);
  printf("%s %.*lf\n", str.c_str(), decimal_point, data);
  printf(ANSI_COLOR_RESET);
#endif
}
void robotis_manipulator::log::println(const char* str, STRING color)
{
#if defined(__OPENCR__)
  DEBUG.println(str);
#else
       if(color == "RED")      printf(ANSI_COLOR_RED);
  else if(color == "GREEN")    printf(ANSI_COLOR_GREEN);
  else if(color == "YELLOW")   printf(ANSI_COLOR_YELLOW);
  else if(color == "BLUE")     printf(ANSI_COLOR_BLUE);
  else if(color == "MAGENTA")  printf(ANSI_COLOR_MAGENTA);
  else if(color == "CYAN")     printf(ANSI_COLOR_CYAN);
  printf("%s\n", str);
  printf(ANSI_COLOR_RESET);
#endif
}
void robotis_manipulator::log::println(const char* str, double data, uint8_t decimal_point, STRING color)
{
#if defined(__OPENCR__)
  DEBUG.print(str);
  DEBUG.println(data, decimal_point);
#else
       if(color == "RED")      printf(ANSI_COLOR_RED);
  else if(color == "GREEN")    printf(ANSI_COLOR_GREEN);
  else if(color == "YELLOW")   printf(ANSI_COLOR_YELLOW);
  else if(color == "BLUE")     printf(ANSI_COLOR_BLUE);
  else if(color == "MAGENTA")  printf(ANSI_COLOR_MAGENTA);
  else if(color == "CYAN")     printf(ANSI_COLOR_CYAN);
  printf("%s %.*lf\n", str, decimal_point, data);
  printf(ANSI_COLOR_RESET);
#endif
}

void robotis_manipulator::log::info(STRING str)
{
#if defined(__OPENCR__)
  DEBUG.print("[INFO] ");
  DEBUG.println(str);
#else
  printf("[INFO] %s\n", str.c_str());
#endif
}
void robotis_manipulator::log::info(STRING str, double data, uint8_t decimal_point)
{
#if defined(__OPENCR__)
  DEBUG.print("[INFO] ");
  DEBUG.print(str);
  DEBUG.println(data, decimal_point);
#else
  printf("[INFO] %s %.*lf\n", str.c_str(), decimal_point, data);
#endif
}
void robotis_manipulator::log::info(const char* str)
{
#if defined(__OPENCR__)
  DEBUG.print("[INFO] ");
  DEBUG.println(str);
#else
  printf("[INFO] %s\n", str);
#endif
}
void robotis_manipulator::log::info(const char* str, double data, uint8_t decimal_point)
{
#if defined(__OPENCR__)
  DEBUG.print("[INFO] ");
  DEBUG.print(str);
  DEBUG.println(data, decimal_point);
#else
  printf("[INFO] %s %.*lf\n", str, decimal_point, data);
#endif
}
void robotis_manipulator::log::warn(STRING str)
{
#if defined(__OPENCR__)
  DEBUG.print("[WARN] ");
  DEBUG.println(str);
#else
  printf(ANSI_COLOR_YELLOW);
  printf("[WARN] %s\n", str.c_str());
  printf(ANSI_COLOR_RESET);
#endif
}
void robotis_manipulator::log::warn(STRING str, double data, uint8_t decimal_point)
{
#if defined(__OPENCR__)
  DEBUG.print("[WARN] ");
  DEBUG.print(str);
  DEBUG.println(data, decimal_point);
#else
  printf(ANSI_COLOR_YELLOW);
  printf("[WARN] %s %.*lf\n",str.c_str(), decimal_point, data);
  printf(ANSI_COLOR_RESET);
#endif
}
void robotis_manipulator::log::warn(const char* str)
{
#if defined(__OPENCR__)
  DEBUG.print("[WARN] ");
  DEBUG.println(str);
#else
  printf(ANSI_COLOR_YELLOW);
  printf("[WARN] %s\n", str);
  printf(ANSI_COLOR_RESET);
#endif
}
void robotis_manipulator::log::warn(const char* str, double data, uint8_t decimal_point)
{
#if defined(__OPENCR__)
  DEBUG.print("[WARN] ");
  DEBUG.print(str);
  DEBUG.println(data, decimal_point);
#else
  printf(ANSI_COLOR_YELLOW);
  printf("[WARN] %s %.*lf\n", str, decimal_point, data);
  printf(ANSI_COLOR_RESET);
#endif
}
void robotis_manipulator::log::error(STRING str)
{
#if defined(__OPENCR__)
  DEBUG.print("[ERROR] ");
  DEBUG.println(str);
#else
  printf(ANSI_COLOR_RED);
  printf("[ERROR] %s\n", str.c_str());
  printf(ANSI_COLOR_RESET);
#endif
}
void robotis_manipulator::log::error(STRING str, double data, uint8_t decimal_point)
{
#if defined(__OPENCR__)
  DEBUG.print("[ERROR] ");
  DEBUG.print(str);
  DEBUG.println(data, decimal_point);
#else
  printf(ANSI_COLOR_RED);
  printf("[ERROR] %s %.*lf\n", str.c_str(), decimal_point, data);
  printf(ANSI_COLOR_RESET);
#endif
}

void robotis_manipulator::log::error(const char* str)
{
#if defined(__OPENCR__)
  DEBUG.print("[ERROR] ");
  DEBUG.println(str);
#else
  printf(ANSI_COLOR_RED);
  printf("[ERROR] %s\n", str);
  printf(ANSI_COLOR_RESET);
#endif
}
void robotis_manipulator::log::error(const char* str, double data, uint8_t decimal_point)
{
#if defined(__OPENCR__)
  DEBUG.print("[ERROR] ");
  DEBUG.print(str);
  DEBUG.println(data, decimal_point);
#else
  printf(ANSI_COLOR_RED);
  printf("[ERROR] %s %.*lf\n", str, decimal_point, data);
  printf(ANSI_COLOR_RESET);
#endif
}

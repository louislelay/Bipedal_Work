#ifndef RASPI_SERVO_INTERFACE__VISIBILITY_CONTROL_H_
#define RASPI_SERVO_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RASPI_SERVO_INTERFACE_EXPORT __attribute__ ((dllexport))
    #define RASPI_SERVO_INTERFACE_IMPORT __attribute__ ((dllimport))
  #else
    #define RASPI_SERVO_INTERFACE_EXPORT __declspec(dllexport)
    #define RASPI_SERVO_INTERFACE_IMPORT __declspec(dllimport)
  #endif
  #ifdef RASPI_SERVO_INTERFACE_BUILDING_LIBRARY
    #define RASPI_SERVO_INTERFACE_PUBLIC RASPI_SERVO_INTERFACE_EXPORT
  #else
    #define RASPI_SERVO_INTERFACE_PUBLIC RASPI_SERVO_INTERFACE_IMPORT
  #endif
  #define RASPI_SERVO_INTERFACE_PUBLIC_TYPE RASPI_SERVO_INTERFACE_PUBLIC
  #define RASPI_SERVO_INTERFACE_LOCAL
#else
  #define RASPI_SERVO_INTERFACE_EXPORT __attribute__ ((visibility("default")))
  #define RASPI_SERVO_INTERFACE_IMPORT
  #if __GNUC__ >= 4
    #define RASPI_SERVO_INTERFACE_PUBLIC __attribute__ ((visibility("default")))
    #define RASPI_SERVO_INTERFACE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RASPI_SERVO_INTERFACE_PUBLIC
    #define RASPI_SERVO_INTERFACE_LOCAL
  #endif
  #define RASPI_SERVO_INTERFACE_PUBLIC_TYPE
#endif

#endif  // RASPI_SERVO_INTERFACE__VISIBILITY_CONTROL_H_

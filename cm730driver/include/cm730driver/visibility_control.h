#ifndef CM730DRIVER__VISIBILITY_CONTROL_H_
#define CM730DRIVER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CM730DRIVER_EXPORT __attribute__ ((dllexport))
    #define CM730DRIVER_IMPORT __attribute__ ((dllimport))
  #else
    #define CM730DRIVER_EXPORT __declspec(dllexport)
    #define CM730DRIVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef CM730DRIVER_BUILDING_LIBRARY
    #define CM730DRIVER_PUBLIC CM730DRIVER_EXPORT
  #else
    #define CM730DRIVER_PUBLIC CM730DRIVER_IMPORT
  #endif
  #define CM730DRIVER_PUBLIC_TYPE CM730DRIVER_PUBLIC
  #define CM730DRIVER_LOCAL
#else
  #define CM730DRIVER_EXPORT __attribute__ ((visibility("default")))
  #define CM730DRIVER_IMPORT
  #if __GNUC__ >= 4
    #define CM730DRIVER_PUBLIC __attribute__ ((visibility("default")))
    #define CM730DRIVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CM730DRIVER_PUBLIC
    #define CM730DRIVER_LOCAL
  #endif
  #define CM730DRIVER_PUBLIC_TYPE
#endif

#endif  // CM730DRIVER__VISIBILITY_CONTROL_H_

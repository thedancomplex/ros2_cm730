#ifndef CM730CONTROLLER__VISIBILITY_CONTROL_H_
#define CM730CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CM730CONTROLLER_EXPORT __attribute__ ((dllexport))
    #define CM730CONTROLLER_IMPORT __attribute__ ((dllimport))
  #else
    #define CM730CONTROLLER_EXPORT __declspec(dllexport)
    #define CM730CONTROLLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef CM730CONTROLLER_BUILDING_LIBRARY
    #define CM730CONTROLLER_PUBLIC CM730CONTROLLER_EXPORT
  #else
    #define CM730CONTROLLER_PUBLIC CM730CONTROLLER_IMPORT
  #endif
  #define CM730CONTROLLER_PUBLIC_TYPE CM730CONTROLLER_PUBLIC
  #define CM730CONTROLLER_LOCAL
#else
  #define CM730CONTROLLER_EXPORT __attribute__ ((visibility("default")))
  #define CM730CONTROLLER_IMPORT
  #if __GNUC__ >= 4
    #define CM730CONTROLLER_PUBLIC __attribute__ ((visibility("default")))
    #define CM730CONTROLLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CM730CONTROLLER_PUBLIC
    #define CM730CONTROLLER_LOCAL
  #endif
  #define CM730CONTROLLER_PUBLIC_TYPE
#endif

#endif  // CM730CONTROLLER__VISIBILITY_CONTROL_H_

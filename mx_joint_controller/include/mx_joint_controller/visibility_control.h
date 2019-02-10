#ifndef MX_JOINT_CONTROLLER__VISIBILITY_CONTROL_H_
#define MX_JOINT_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MX_JOINT_CONTROLLER_EXPORT __attribute__ ((dllexport))
    #define MX_JOINT_CONTROLLER_IMPORT __attribute__ ((dllimport))
  #else
    #define MX_JOINT_CONTROLLER_EXPORT __declspec(dllexport)
    #define MX_JOINT_CONTROLLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef MX_JOINT_CONTROLLER_BUILDING_LIBRARY
    #define MX_JOINT_CONTROLLER_PUBLIC MX_JOINT_CONTROLLER_EXPORT
  #else
    #define MX_JOINT_CONTROLLER_PUBLIC MX_JOINT_CONTROLLER_IMPORT
  #endif
  #define MX_JOINT_CONTROLLER_PUBLIC_TYPE MX_JOINT_CONTROLLER_PUBLIC
  #define MX_JOINT_CONTROLLER_LOCAL
#else
  #define MX_JOINT_CONTROLLER_EXPORT __attribute__ ((visibility("default")))
  #define MX_JOINT_CONTROLLER_IMPORT
  #if __GNUC__ >= 4
    #define MX_JOINT_CONTROLLER_PUBLIC __attribute__ ((visibility("default")))
    #define MX_JOINT_CONTROLLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MX_JOINT_CONTROLLER_PUBLIC
    #define MX_JOINT_CONTROLLER_LOCAL
  #endif
  #define MX_JOINT_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // MX_JOINT_CONTROLLER__VISIBILITY_CONTROL_H_

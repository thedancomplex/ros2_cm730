#ifndef MX_JOINT_STATE_PUBLISHER__VISIBILITY_CONTROL_H_
#define MX_JOINT_STATE_PUBLISHER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MX_JOINT_STATE_PUBLISHER_EXPORT __attribute__ ((dllexport))
    #define MX_JOINT_STATE_PUBLISHER_IMPORT __attribute__ ((dllimport))
  #else
    #define MX_JOINT_STATE_PUBLISHER_EXPORT __declspec(dllexport)
    #define MX_JOINT_STATE_PUBLISHER_IMPORT __declspec(dllimport)
  #endif
  #ifdef MX_JOINT_STATE_PUBLISHER_BUILDING_LIBRARY
    #define MX_JOINT_STATE_PUBLISHER_PUBLIC MX_JOINT_STATE_PUBLISHER_EXPORT
  #else
    #define MX_JOINT_STATE_PUBLISHER_PUBLIC MX_JOINT_STATE_PUBLISHER_IMPORT
  #endif
  #define MX_JOINT_STATE_PUBLISHER_PUBLIC_TYPE MX_JOINT_STATE_PUBLISHER_PUBLIC
  #define MX_JOINT_STATE_PUBLISHER_LOCAL
#else
  #define MX_JOINT_STATE_PUBLISHER_EXPORT __attribute__ ((visibility("default")))
  #define MX_JOINT_STATE_PUBLISHER_IMPORT
  #if __GNUC__ >= 4
    #define MX_JOINT_STATE_PUBLISHER_PUBLIC __attribute__ ((visibility("default")))
    #define MX_JOINT_STATE_PUBLISHER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MX_JOINT_STATE_PUBLISHER_PUBLIC
    #define MX_JOINT_STATE_PUBLISHER_LOCAL
  #endif
  #define MX_JOINT_STATE_PUBLISHER_PUBLIC_TYPE
#endif

#endif  // MX_JOINT_STATE_PUBLISHER__VISIBILITY_CONTROL_H_

#ifndef OCTOMAP_DEPRECATED_H
#define OCTOMAP_DEPRECATED_H

// define multi-platform deprecation mechanism
#ifndef DEPRECATED
  #ifdef __GNUC__
    #define DEPRECATED(func) func __attribute__ ((deprecated))
  #elif defined(_MSC_VER)
    #define DEPRECATED(func) __declspec(deprecated) func
  #else
    #pragma message("WARNING: You need to implement DEPRECATED for this compiler")
    #define DEPRECATED(func) func
  #endif
#endif

#endif

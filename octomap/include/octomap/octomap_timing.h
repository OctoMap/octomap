#ifndef OCTOMAP_TIMING_H_
#define OCTOMAP_TIMING_H_

#ifdef _MSC_VER
	// MS compilers
  #include <sys/timeb.h>
  #include <sys/types.h>
  #include <winsock.h>
  void gettimeofday(struct timeval* t, void* timezone) {
    struct _timeb timebuffer;
    _ftime64_s( &timebuffer );
    t->tv_sec= (long) timebuffer.time;
    t->tv_usec=1000*timebuffer.millitm;
  }
#else 
	// GCC and minGW
  #include <sys/time.h>
#endif


#endif
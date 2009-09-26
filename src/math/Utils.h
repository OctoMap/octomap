#ifndef OCTOMATH_UTILS_H
#define OCTOMATH_UTILS_H

#include <fstream>
#include <string>
#include <cmath>
#include <complex>
#include <cassert>
#include <vector>

#ifndef SQR
#define SQR(x) ((x)*(x))
#endif

#define OUR_MAX(a,b) (a>b)?(a):(b)


//#define USE_FLOAT_IN_OUR




/* GNU C predefined mathematical constants: */

/* M_E         The base of natural logarithms.  */
/* M_LOG2E     The logarithm to base 2 of M_E.  */
/* M_LOG10E    The logarithm to base 10 of M_E.  */
/* M_LN2       The natural logarithm of 2.  */
/* M_LN10      The natural logarithm of 10.  */
/* M_PI        Pi, the ratio of a circle's circumference to its diameter.  */
/* M_PI_2      Pi divided by two.  */
/* M_PI_4      Pi divided by four.  */
/* M_1_PI      The reciprocal of pi (1/pi)  */
/* M_2_PI      Two times the reciprocal of pi.  */
/* M_2_SQRTPI  Two times the reciprocal of the square root of pi.  */
/* M_SQRT2     The square root of two.  */
/* M_SQRT1_2   The reciprocal of the square root of two (also the square root of 1/2). */

#ifndef M_E
#define M_E 2.718281828459045235
#endif

#ifndef M_LOG2E
#define M_LOG2E 1.442695040888963357
#endif

#ifndef M_LOG10E
#define M_LOG10E 0.4342944819032518167
#endif

#ifndef M_LN2
#define M_LN2 0.6931471805599452862
#endif

#ifndef M_LN10
#define M_LN10 2.302585092994045901
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.570796326794896619
#endif

#ifndef M_PI_4
#define M_PI_4 0.7853981633974483096
#endif

#ifndef M_1_PI
#define M_1_PI 0.3183098861837906715
#endif

#ifndef M_2_PI
#define M_2_PI 0.6366197723675813431
#endif

#ifndef M_2_SQRTPI
#define M_2_SQRTPI 1.128379167095512574
#endif

#ifndef M_SQRT2
#define M_SQRT2 1.414213562373095049
#endif

#ifndef M_SQRT1_2
#define M_SQRT1_2 0.7071067811865475244
#endif


/* additional constants */

/* M_TWO_PI        Two times Pi. */
/* M_SQRT_TWO_PI   Square root of two times Pi. */

#ifndef M_TWO_PI
#define M_TWO_PI 6.283185307179586477
#endif

#ifndef M_SQRT_TWO_PI
#define M_SQRT_TWO_PI 2.506628274631000502
#endif



namespace octomath
{

  double normalizeAngle(double angle);
  double angleDistance(double angle1, double angle2);

}


#endif

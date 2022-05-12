#include <math.h>
#include <stdlib.h>

// this is mimicing gtest expressions

#define EXPECT_TRUE(args) {                                             \
    if (!(args)) { fprintf(stderr, "test failed (EXPECT_TRUE) in %s, line %d\n", __FILE__, __LINE__); \
      exit(1);                                                         \
    } }

#define EXPECT_FALSE(args) {                                             \
    if (args) { fprintf(stderr, "test failed (EXPECT_FALSE) in %s, line %d\n", __FILE__, __LINE__); \
      exit(1);                                                         \
    } }

#define EXPECT_EQ(a,b) {                                                \
    if (!((a) == (b))) { std::cerr << "test failed: " <<a<<"!="<<b<< " in " \
                      << __FILE__ << ", line " <<__LINE__ << std::endl; \
      exit(1);                                                          \
    } }

#define EXPECT_FLOAT_EQ(a,b) {                                          \
    if (!(fabs((a) - (b)) <= 1e-5)) { fprintf(stderr, "test failed: %f != %f in %s, line %d\n", a, b, __FILE__, __LINE__); \
      exit(1);                                                         \
    } }

#define EXPECT_NEAR(a,b,prec) {                                         \
    if (!(fabs((a) - (b)) <= prec)) { fprintf(stderr, "test failed: |%f - %f| > %f in %s, line %d\n", a, b, prec, __FILE__, __LINE__); \
      exit(1);                                                         \
    } }


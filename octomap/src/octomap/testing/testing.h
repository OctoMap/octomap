#include <math.h>
#include <stdlib.h>

// this is mimicing gtest expressions

bool EXPECT_EQ (int a, int b) {
  if (a == b) return true;
  fprintf(stderr, "test failed: %d != %d\n", a, b);
  exit(-1);
}

bool EXPECT_FLOAT_EQ (float a, float b) {
  if (fabs(a-b) <= 1e-5) return true;
  fprintf(stderr, "test failed: %f != %f\n", a, b);
  exit(-1);
}

bool EXPECT_TRUE (bool b) {
  if (b) return true;
  fprintf(stderr, "test failed\n");
  exit(-1);
}

bool EXPECT_NEAR (float a, float b, float prec) {
  if (fabs(a-b) <= prec) return true;
  fprintf(stderr, "test failed: |%f - %f| > %f\n", a, b, prec);
  exit(-1);
}


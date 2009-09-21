#ifndef OUR_TYPES_H
#define OUR_TYPES_H


// Turn FLOAT ON/OFF HERE!
// #define USE_FLOAT_IN_OUR

#ifdef USE_FLOAT_IN_OUR

typedef float OUR_REAL; 

#define OUR_COERCE gsl_coerce_float
#define OUR_VECTOR gsl_vector_float
#define OUR_SORT_VECTOR gsl_sort_vector_float
#define OUR_SORT_VECTOR_INDEX gsl_sort_vector_float_index
#define OUR_COMPLEX gsl_complex_float
#define OUR_VECTOR_COMPLEX gsl_vector_complex_float
#define OUR_PERMUTE_VECTOR_COMPLEX gsl_permute_vector_complex_float
#define OUR_MATRIX_COMPLEX gsl_matrix_complex_float
#define OUR_VECTOR_COMPLEX_FUN(fun) gsl_vector_complex_float_##fun
#define OUR_PERMUTE_VECTOR_COMPLEX_FUN(fun) gsl_permute_vector_complex_float_##fun
#define OUR_PERMUTE_VECTOR_FUN(fun) gsl_permute_vector_float_##fun
#define OUR_MATRIX_COMPLEX_FUN(fun) gsl_matrix_complex_float_##fun
#define OUR_VECTOR_FUN(fun) gsl_vector_float_##fun
#define OUR_COMPLEX_FUN(fun) gsl_complex_float_##fun
#define OUR_MATRIX gsl_matrix_float
#define OUR_MATRIX_FUN(fun) gsl_matrix_float_##fun
#define OUR_BLAS_FUN(fun)  gsl_blas_s##fun
#define OUR_BLAS_Z_FUN(fun)  gsl_blas_c##fun
#define OUR_BLAS_ZDSCAL  gsl_blas_csscal
#define OUR_PERMUTE_VECTOR  gsl_permute_vector_float
#define OUR_BLAS_DZ_FUN(fun)  gsl_blas_sc##fun

#else

typedef double OUR_REAL; 

#define OUR_COERCE gsl_coerce_double
#define OUR_VECTOR gsl_vector
#define OUR_SORT_VECTOR gsl_sort_vector
#define OUR_SORT_VECTOR_INDEX gsl_sort_vector_index
#define OUR_COMPLEX gsl_complex
#define OUR_VECTOR_COMPLEX gsl_vector_complex
#define OUR_PERMUTE_VECTOR_COMPLEX gsl_permute_vector_complex
#define OUR_MATRIX_COMPLEX gsl_matrix_complex
#define OUR_VECTOR_COMPLEX_FUN(fun) gsl_vector_complex_##fun
#define OUR_PERMUTE_VECTOR_COMPLEX_FUN(fun) gsl_permute_vector_complex_##fun
#define OUR_PERMUTE_VECTOR_FUN(fun) gsl_permute_vector_##fun
#define OUR_MATRIX_COMPLEX_FUN(fun) gsl_matrix_complex_##fun
#define OUR_VECTOR_FUN(fun) gsl_vector_##fun
#define OUR_COMPLEX_FUN(fun) gsl_complex_##fun
#define OUR_MATRIX gsl_matrix
#define OUR_MATRIX_FUN(fun) gsl_matrix_##fun
#define OUR_BLAS_FUN(fun)  gsl_blas_d##fun
#define OUR_BLAS_Z_FUN(fun)  gsl_blas_z##fun
#define OUR_BLAS_ZDSCAL  gsl_blas_zdscal
#define OUR_PERMUTE_VECTOR  gsl_permute_vector
#define OUR_BLAS_DZ_FUN(fun)  gsl_blas_dz##fun


#endif


#endif

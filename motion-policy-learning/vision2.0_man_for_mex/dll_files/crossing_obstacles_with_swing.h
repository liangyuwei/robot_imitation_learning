//
// MATLAB Compiler: 6.2 (R2016a)
// Date: Mon May 14 11:24:16 2018
// Arguments: "-B" "macro_default" "-W" "cpplib:crossing_obstacles_with_swing"
// "-T" "lnik:lib" "crossing_obstacles_with_swing.m" "-d" "dll_files" 
//

#ifndef __crossing_obstacles_with_swing_h
#define __crossing_obstacles_with_swing_h 1

#if defined(__cplusplus) && !defined(mclmcrrt_h) && defined(__linux__)
#  pragma implementation "mclmcrrt.h"
#endif
#include "mclmcrrt.h"
#include "mclcppclass.h"
#ifdef __cplusplus
extern "C" {
#endif

#if defined(__SUNPRO_CC)
/* Solaris shared libraries use __global, rather than mapfiles
 * to define the API exported from a shared library. __global is
 * only necessary when building the library -- files including
 * this header file to use the library do not need the __global
 * declaration; hence the EXPORTING_<library> logic.
 */

#ifdef EXPORTING_crossing_obstacles_with_swing
#define PUBLIC_crossing_obstacles_with_swing_C_API __global
#else
#define PUBLIC_crossing_obstacles_with_swing_C_API /* No import statement needed. */
#endif

#define LIB_crossing_obstacles_with_swing_C_API PUBLIC_crossing_obstacles_with_swing_C_API

#elif defined(_HPUX_SOURCE)

#ifdef EXPORTING_crossing_obstacles_with_swing
#define PUBLIC_crossing_obstacles_with_swing_C_API __declspec(dllexport)
#else
#define PUBLIC_crossing_obstacles_with_swing_C_API __declspec(dllimport)
#endif

#define LIB_crossing_obstacles_with_swing_C_API PUBLIC_crossing_obstacles_with_swing_C_API


#else

#define LIB_crossing_obstacles_with_swing_C_API

#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_crossing_obstacles_with_swing_C_API 
#define LIB_crossing_obstacles_with_swing_C_API /* No special import/export declaration */
#endif

extern LIB_crossing_obstacles_with_swing_C_API 
bool MW_CALL_CONV crossing_obstacles_with_swingInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_crossing_obstacles_with_swing_C_API 
bool MW_CALL_CONV crossing_obstacles_with_swingInitialize(void);

extern LIB_crossing_obstacles_with_swing_C_API 
void MW_CALL_CONV crossing_obstacles_with_swingTerminate(void);



extern LIB_crossing_obstacles_with_swing_C_API 
void MW_CALL_CONV crossing_obstacles_with_swingPrintStackTrace(void);

extern LIB_crossing_obstacles_with_swing_C_API 
bool MW_CALL_CONV mlxCrossing_obstacles_with_swing(int nlhs, mxArray *plhs[], int nrhs, 
                                                   mxArray *prhs[]);


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__BORLANDC__)

#ifdef EXPORTING_crossing_obstacles_with_swing
#define PUBLIC_crossing_obstacles_with_swing_CPP_API __declspec(dllexport)
#else
#define PUBLIC_crossing_obstacles_with_swing_CPP_API __declspec(dllimport)
#endif

#define LIB_crossing_obstacles_with_swing_CPP_API PUBLIC_crossing_obstacles_with_swing_CPP_API

#else

#if !defined(LIB_crossing_obstacles_with_swing_CPP_API)
#if defined(LIB_crossing_obstacles_with_swing_C_API)
#define LIB_crossing_obstacles_with_swing_CPP_API LIB_crossing_obstacles_with_swing_C_API
#else
#define LIB_crossing_obstacles_with_swing_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_crossing_obstacles_with_swing_CPP_API void MW_CALL_CONV crossing_obstacles_with_swing(int nargout, mwArray& the_tot, mwArray& f_tot, mwArray& Steps, const mwArray& traj_setting, const mwArray& motion_req, const mwArray& envionment_setting);

#endif
#endif

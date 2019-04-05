//
// MATLAB Compiler: 6.2 (R2016a)
// Date: Mon May 14 11:38:40 2018
// Arguments: "-B" "macro_default" "-W" "cpplib:crossing_obstacles_with_swing"
// "-T" "link:lib" "-d"
// "C:\Graduation_Project\One_Step_V8_improving\vision2.0_man_for_mex\dll_files2
// \crossing_obstacles_with_swing\for_testing" "-v"
// "C:\Graduation_Project\One_Step_V8_improving\vision2.0_man_for_mex\crossing_o
// bstacles_with_swing.m" 
//

#include <stdio.h>
#define EXPORTING_crossing_obstacles_with_swing 1
#include "crossing_obstacles_with_swing.h"

static HMCRINSTANCE _mcr_inst = NULL;


#if defined( _MSC_VER) || defined(__BORLANDC__) || defined(__WATCOMC__) || defined(__LCC__)
#ifdef __LCC__
#undef EXTERN_C
#endif
#include <windows.h>

static char path_to_dll[_MAX_PATH];

BOOL WINAPI DllMain(HINSTANCE hInstance, DWORD dwReason, void *pv)
{
    if (dwReason == DLL_PROCESS_ATTACH)
    {
        if (GetModuleFileName(hInstance, path_to_dll, _MAX_PATH) == 0)
            return FALSE;
    }
    else if (dwReason == DLL_PROCESS_DETACH)
    {
    }
    return TRUE;
}
#endif
#ifdef __cplusplus
extern "C" {
#endif

static int mclDefaultPrintHandler(const char *s)
{
  return mclWrite(1 /* stdout */, s, sizeof(char)*strlen(s));
}

#ifdef __cplusplus
} /* End extern "C" block */
#endif

#ifdef __cplusplus
extern "C" {
#endif

static int mclDefaultErrorHandler(const char *s)
{
  int written = 0;
  size_t len = 0;
  len = strlen(s);
  written = mclWrite(2 /* stderr */, s, sizeof(char)*len);
  if (len > 0 && s[ len-1 ] != '\n')
    written += mclWrite(2 /* stderr */, "\n", sizeof(char));
  return written;
}

#ifdef __cplusplus
} /* End extern "C" block */
#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_crossing_obstacles_with_swing_C_API
#define LIB_crossing_obstacles_with_swing_C_API /* No special import/export declaration */
#endif

LIB_crossing_obstacles_with_swing_C_API 
bool MW_CALL_CONV crossing_obstacles_with_swingInitializeWithHandlers(
    mclOutputHandlerFcn error_handler,
    mclOutputHandlerFcn print_handler)
{
    int bResult = 0;
  if (_mcr_inst != NULL)
    return true;
  if (!mclmcrInitialize())
    return false;
  if (!GetModuleFileName(GetModuleHandle("crossing_obstacles_with_swing"), path_to_dll, _MAX_PATH))
    return false;
    {
        mclCtfStream ctfStream = 
            mclGetEmbeddedCtfStream(path_to_dll);
        if (ctfStream) {
            bResult = mclInitializeComponentInstanceEmbedded(   &_mcr_inst,
                                                                error_handler, 
                                                                print_handler,
                                                                ctfStream);
            mclDestroyStream(ctfStream);
        } else {
            bResult = 0;
        }
    }  
    if (!bResult)
    return false;
  return true;
}

LIB_crossing_obstacles_with_swing_C_API 
bool MW_CALL_CONV crossing_obstacles_with_swingInitialize(void)
{
  return crossing_obstacles_with_swingInitializeWithHandlers(mclDefaultErrorHandler, 
                                                             mclDefaultPrintHandler);
}

LIB_crossing_obstacles_with_swing_C_API 
void MW_CALL_CONV crossing_obstacles_with_swingTerminate(void)
{
  if (_mcr_inst != NULL)
    mclTerminateInstance(&_mcr_inst);
}

LIB_crossing_obstacles_with_swing_C_API 
void MW_CALL_CONV crossing_obstacles_with_swingPrintStackTrace(void) 
{
  char** stackTrace;
  int stackDepth = mclGetStackTrace(&stackTrace);
  int i;
  for(i=0; i<stackDepth; i++)
  {
    mclWrite(2 /* stderr */, stackTrace[i], sizeof(char)*strlen(stackTrace[i]));
    mclWrite(2 /* stderr */, "\n", sizeof(char)*strlen("\n"));
  }
  mclFreeStackTrace(&stackTrace, stackDepth);
}


LIB_crossing_obstacles_with_swing_C_API 
bool MW_CALL_CONV mlxCrossing_obstacles_with_swing(int nlhs, mxArray *plhs[], int nrhs, 
                                                   mxArray *prhs[])
{
  return mclFeval(_mcr_inst, "crossing_obstacles_with_swing", nlhs, plhs, nrhs, prhs);
}

LIB_crossing_obstacles_with_swing_CPP_API 
void MW_CALL_CONV crossing_obstacles_with_swing(int nargout, mwArray& the_tot, mwArray& 
                                                f_tot, mwArray& Steps, const mwArray& 
                                                traj_setting, const mwArray& motion_req, 
                                                const mwArray& envionment_setting)
{
  mclcppMlfFeval(_mcr_inst, "crossing_obstacles_with_swing", nargout, 3, 3, &the_tot, &f_tot, &Steps, &traj_setting, &motion_req, &envionment_setting);
}


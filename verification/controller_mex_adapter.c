#include "mex.h"
#include "vtol_controller.h"
#include <math.h>
#include <string.h>

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    const double *x, *y;
    double den;
    int k;
    if (nrhs != 2 || nlhs != 2) {
        mexErrMsgIdAndTxt("VTOL:Interface", "Use [u1,u2] = vtol_controller_mex(z1,z2).");
    }
    for (k = 0; k < 2; ++k) {
        if (!mxIsDouble(prhs[k]) || mxIsComplex(prhs[k]) || mxIsSparse(prhs[k]) ||
            mxGetNumberOfElements(prhs[k]) != 5) {
            mexErrMsgIdAndTxt("VTOL:Interface", "Each reference must contain five real doubles.");
        }
    }
    x = mxGetPr(prhs[0]); y = mxGetPr(prhs[1]);
    for (k = 0; k < 5; ++k) {
        if (!isfinite(x[k]) || !isfinite(y[k])) {
            mexErrMsgIdAndTxt("VTOL:Nonfinite", "Reference must be finite.");
        }
    }
    /* Preserve the model's singularity guard; normalized gravity is one. */
    den = x[2]*x[2] + (y[2]+1.0)*(y[2]+1.0);
    if (!(den > 0.05*0.05)) {
        mexErrMsgIdAndTxt("VTOL:Singular", "Thrust inversion is singular.");
    }
    /* This controller is stateless; each call is independent of solver order. */
    vtol_controller_initialize();
    memcpy(vtol_controller_U.z1_d_x, x, 5*sizeof(double));
    memcpy(vtol_controller_U.z2_d_y, y, 5*sizeof(double));
    vtol_controller_step();
    if (rtmGetErrorStatus(vtol_controller_M) != NULL ||
        !isfinite(vtol_controller_Y.u1_ff) || !isfinite(vtol_controller_Y.u2_ff)) {
        mexErrMsgIdAndTxt("VTOL:Controller", "Generated controller returned invalid outputs.");
    }
    plhs[0] = mxCreateDoubleScalar(vtol_controller_Y.u1_ff);
    plhs[1] = mxCreateDoubleScalar(vtol_controller_Y.u2_ff);
    vtol_controller_terminate();
}

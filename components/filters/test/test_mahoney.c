#include "test_attitude.c"

// d = test_attitude(a, w, dt, kp, ki);
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    if (nrhs < 2)
    {
        mexErrMsgIdAndTxt("test_attitude:InvalidInput",
            "At least two input arguments are required!");
    }

    double dt = 1.0f;
    if (nrhs > 2)
    {
        dt = mxGetScalar(prhs[2]);
    }

    double kp = 1.0f;
    if (nrhs > 3)
    {
        kp = mxGetScalar(prhs[3]);
    }

    double ki = 1.0f;
    if (nrhs > 4)
    {
        ki = mxGetScalar(prhs[4]);
    }

    if (nlhs > 1)
    {
        mexErrMsgIdAndTxt("test_attitude:InvalidOutput",
            "Only one output argument is allowed!");
    }

    mwSize ndims_a = mxGetNumberOfDimensions(prhs[0]);
    mwSize ndims_w = mxGetNumberOfDimensions(prhs[1]);
    if ((ndims_a != 2) || (ndims_w != 2))
    {
        mexErrMsgIdAndTxt("test_attitude:InvalidInput",
            "Input arguments must be 2D matrices!");
    }

    mwSize ncols_a = mxGetN(prhs[0]);
    mwSize ncols_w = mxGetN(prhs[1]);
    mwSize nrows_a = mxGetM(prhs[0]);
    mwSize nrows_w = mxGetM(prhs[1]);
    if ((nrows_a != 3) || (nrows_w != 3))
    {
        mexErrMsgIdAndTxt(
            "test_attitude:InvalidInput", "Input arguments must have 3 rows!");
    }

    if ((ncols_a != ncols_w) || (nrows_a != nrows_w))
    {
        mexErrMsgIdAndTxt("test_attitude:InvalidInput",
            "Input arguments must have the same dimensions!");
    }

    initialize_attitude_filters(dt, kp, ki);

    mxDouble* a = mxGetDoubles(prhs[0]);
    mxDouble* w = mxGetDoubles(prhs[1]);

    mxDouble R[3][3] = EYE3;
    mxDouble b[3] = {0.0};

    mxDouble down[3] = {a[0], a[1], a[2]};
    mxDouble dmag =
        sqrt(down[0] * down[0] + down[1] * down[1] + down[2] * down[2]);
    down[0] /= dmag;
    down[1] /= dmag;
    down[2] /= dmag;

    plhs[0] = mxCreateDoubleMatrix(nrows_a, ncols_a, mxREAL);
    mxDouble* output = mxGetDoubles(plhs[0]);

    for (mwSize i = 0; i < ncols_a; i++)
    {
        double ai[3] = {0.0};
        ai[0] = *a;
        a++;
        ai[1] = *a;
        a++;
        ai[2] = *a;
        a++;

        double wi[3] = {0.0};
        wi[0] = *w;
        w++;
        wi[1] = *w;
        w++;
        wi[2] = *w;
        w++;

        mahoney_filter(R, b, down, ai, wi);

        mxDouble amag = sqrt(ai[0] * ai[0] + ai[1] * ai[1] + ai[2] * ai[2]);
        ai[0] /= amag;
        ai[1] /= amag;
        ai[2] /= amag;

        mxDouble d[3] = {0.0};
        get_past_direction(R, ai, d);
        *output = d[0];
        output++;
        *output = d[1];
        output++;
        *output = d[2];
        output++;
    }
}

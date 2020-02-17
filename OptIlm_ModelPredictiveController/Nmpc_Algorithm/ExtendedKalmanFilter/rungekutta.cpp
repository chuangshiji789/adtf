#include "rungekutta.h"

RungeKutta::RungeKutta(vector <double (*)(double*, double*, double)> f,
                       int s) :
    funcs(f),
    size(s)
{
    k1.resize(size);
    k2.resize(size);
    k3.resize(size);
    k4.resize(size);
    tmpState.resize(size);
}

vector <double> RungeKutta::calcState(double *sVars, double *u, double h)
{
    for(int i = 0; i < size; ++i)
    {
        k1[i] = funcs[i](sVars, u, h);
    }
    for(int i = 0; i < size; ++i)
    {
        tmpState[i] = sVars[i] + 0.5 * k1[i];
    }
    for(int i = 0; i < size; ++i)
    {
        k2[i] = funcs[i](tmpState.data(), u, h);
    }
    for(int i = 0; i < size; ++i)
    {
        tmpState[i] = sVars[i] + 0.5 * k2[i];
    }
    for(int i = 0; i < size; ++i)
    {
        k3[i] = funcs[i](tmpState.data(), u, h);
    }
    for(int i = 0; i < size; ++i)
    {
        tmpState[i] = sVars[i] + k3[i];
    }
    for(int i = 0; i < size; ++i)
    {
        k4[i] = funcs[i](tmpState.data(), u, h);
    }
    for(int i = 0; i < size; ++i)
    {
        tmpState[i] = (k1[i] + 2. * k2[i] + 2. * k3[i] + k4[i]) / 6.;
    }

    return tmpState;
}




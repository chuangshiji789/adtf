#ifndef RUNGEKUTTA_H
#define RUNGEKUTTA_H

#include <vector>

using namespace std;

class RungeKutta
{
public:
    RungeKutta(vector <double (*)(double*, double*, double)> f, int s);
    vector <double> calcState(double *sVars, double *u, double h);

private:
    vector <double (*)(double*, double*, double)> funcs;
    vector <double> k1;
    vector <double> k2;
    vector <double> k3;
    vector <double> k4;
    vector <double> tmpState;
    int size;
};


#endif // RUNGEKUTTA_H

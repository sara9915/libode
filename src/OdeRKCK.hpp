/*
This class implements a 5th order method developed by Cash and Karp [1] which includes embeddes solutions of order 1, 2, 3, and 4. It's a complete family of solutions up to order 5. The lower order solutions can be used to vary the solution's order and control the step size in sophisticated ways, canceling a step early if the error estimate is large. The fact that function evaluations are made fairly evenly through the interval of a time step is also advantageous for surveying potential trouble and controlling the step.

So far, the sophisticated time stepping is not implemented, so this solver simply uses the 4th and 5th order solutions for adaptation.

    [1] J. R. Cash, A. H. Karp. "A variable order Runge-Kutta method for initial value problems with rapidly varying right-hand sides", ACM Transactions on Mathematical Software 16: 201-222, 1990. doi:10.1145/79505.79507
*/

#ifndef ODE_RKCK_HPP
#define ODE_RKCK_HPP

#include "OdeBaseARK.hpp"

class OdeRKCK : public OdeBaseARK {
public:
    //constructor
    OdeRKCK (unsigned long long neq_);
    //destructor
    ~OdeRKCK ();
    //arrays for time stepping routine
    double *k1, *k2, *k3, *k4, *k5, *k6;
    //function for taking a single time step
    void step (double dt);
private:
    //coefficents of tableau
    double c2, a21,
           c3, a31, a32,
           c4, a41, a42, a43,
           c5, a51, a52, a53, a54,
           c6, a61, a62, a63, a64, a65,
                b1,  b2,  b3,  b4,  b5, b6,
                d1,  d2,  d3,  d4,  d5, d6,
                e1,  e2,  e3,  e4,
                f1, f2,
                g1;

};

#endif

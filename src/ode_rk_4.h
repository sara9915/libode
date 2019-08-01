/*
THE classic Runge-Kutta 4th order method
*/

#ifndef ODE_RK4_H_
#define ODE_RK4_H_

#include "ode_base.h"
#include "ode_rk.h"
#include "ode_erk.h"

class OdeRK4 : public OdeBase, private OdeRK, private OdeERK {

    public:
        //constructor
        OdeRK4 (unsigned long neq);

    private:
        //take a time step
        void step_ (double dt);
        //coefficents of tableau
        double c2, a21,
               c3,      a32,
               c4,           a43,
                    b1,  b2,  b3,  b4;
};


#endif

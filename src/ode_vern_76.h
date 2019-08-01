/*
Jim Verner's "most efficient" 7/6 pair
    http://people.math.sfu.ca/~jverner/
*/

#ifndef ODE_VERN76_H_
#define ODE_VERN76_H_

#include "ode_embedded.h"
#include "ode_rk.h"
#include "ode_erk.h"

class OdeVern76 : public OdeEmbedded, private OdeRK, private OdeERK {

    public:
        //constructor
        OdeVern76 (unsigned long neq);

    private:
        //function for taking a single time step
        void step_ (double dt);
        //coefficents of tableau
        double c2,   a21,
               c3,   a31,  a32,
               c4,   a41,        a43,
               c5,   a51,        a53,  a54,
               c6,   a61,        a63,  a64,  a65,
               c7,   a71,        a73,  a74,  a75,  a76,
               c8,   a81,        a83,  a84,  a85,  a86,  a87,
               c9,   a91,        a93,  a94,  a95,  a96,  a97,  a98,
               c10, a101,       a103, a104, a105, a106, a107,
                      b1,               b4,   b5,   b6,   b7,   b8,  b9,
                      d1,               d4,   d5,   d6,   d7,            d10;
};

#endif

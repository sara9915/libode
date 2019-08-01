/* A sixth-order, A stable Rosenbrock method from:
    Kaps, Peter, and Gerhard Wanner. "A study of Rosenbrock-type methods of high order." Numerische Mathematik 38.2 (1981): 279-298.
*/

#ifndef ODE_ROW6A_H_
#define ODE_ROW6A_H_

#include "ode_linalg.h"
#include "ode_base.h"
#include "ode_rosenbrock.h"

class OdeROW6A : public OdeBase, private OdeRosenbrock {

    public:
        //constructor
        OdeROW6A (unsigned long neq);

    private:
        //function for taking a single step
        void step_ (double dt);
        //coefficients
        double a21,
               a31, a32,
               a41, a42, a43,
               a51, a52, a53, a54,
               a61, a62, a63, a64, a65;
        double c21,
               c31, c32,
               c41, c42, c43,
               c51, c52, c53, c54,
               c61, c62, c63, c64, c65;
        double m1, m2, m3, m4, m5, m6;

};

#endif

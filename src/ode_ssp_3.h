#ifndef ODE_SSP3_H_
#define ODE_SSP3_H_

#include "ode_base.h"
#include "ode_rk.h"
#include "ode_erk.h"

/*!
This is a strong stability preserving method of order 3, from Shu and Osher:
    + C. W. Shu and S. Osher, Effcient implementation of essentially nonoscillatory shock-capturing schemes, J. Comput. Phys., 77, 1988, pp. 439-471.
*/
class OdeSsp3 : public OdeBase, private OdeRK, private OdeERK {

    public:
        //!constructs
        /*!
        \param[in] neq size of ODE sytem
        */
        OdeSsp3 (unsigned long neq);

    private:
        //take a time step
        void step_ (double dt);
        //coefficents of tableau
        double c2, a21,
               c3, a31, a32,
                    b1,  b2, b3;
};


#endif

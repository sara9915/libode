#ifndef ODE_SDIRK_43_H_
#define ODE_SDIRK_43_H_

#include "ode_embedded.h"
#include "ode_rk.h"
#include "ode_newton_bridge.h"

//forward declaration to set up Newton class
class OdeSDIRK43;

//!Nonlinear system solver for Backward Euler's method
class NewtonSDIRK43 : public OdeNewtonSDIRK<OdeSDIRK43> {
    public:
        //!constructs
        /*!
        \param[in] neq size of ODE system
        \param[in] integrator pointer to OdeSDIRK43 object
        */
        NewtonSDIRK43 (unsigned long neq, OdeSDIRK43 *integrator) : OdeNewtonSDIRK (neq, integrator) {};
    private:
        void f_Newton (double *x, double *y);
        void J_Newton (double *x, double **J);
};

/*!
L-stable 4/3 SDIRK pair from section IV.6 of:
    + Wanner, Gerhard, and Ernst Hairer. Solving ordinary differential equations II. Springer Berlin Heidelberg, 1996.
*/
class OdeSDIRK43 : public OdeEmbedded, private OdeRK {
    //friends!
    friend class OdeNewtonBridge<OdeSDIRK43>;
    friend class OdeNewtonSDIRK<OdeSDIRK43>;

    public:
        //!constructs
        /*!
        \param[in] neq size of ODE sytem
        */
        OdeSDIRK43 (unsigned long neq);

        //!destructs
        ~OdeSDIRK43 ();

        //!returns the solver'sNewton system object
        NewtonSDIRK43 get_newton () { return(*newton_); }
    private:
        double gam;
        double **a;
        double *b;
        double *d;
        NewtonSDIRK43 *newton_;
        void step_ (double dt);
};

#endif

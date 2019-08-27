#ifndef ODE_ERK_H_
#define ODE_ERK_H_

/*!
Provides a temporary solution array for moving through RK stages with explicit solvers
*/
class OdeERK {

    public:
        //!constructs
        /*!
        \param[in] neq number of equations in ODE system
        */
        OdeERK (unsigned long neq);
        //destructs
        ~OdeERK ();

    protected:
        //!temporary solution vector
        double *soltemp_;
};

#endif

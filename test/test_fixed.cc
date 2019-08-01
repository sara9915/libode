#include <cstdio>

#include "test_systems.h"

int main () {

    //choose an integration time
    double tint = 4;
    //time step
    double dt = 0.01;
    //choose which system and solver to use
    Osc1<OdeBackwardEuler> sys;

    printf("Solving system '%s' with method '%s'\n",
        sys.get_name(), sys.get_method());
    sys.solve_fixed(tint, dt, "out", 1);
    printf("%llu function evaluations, %llu steps\n",
        sys.get_neval(), sys.get_nstep());

    return(0);
}

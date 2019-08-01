#include <cstdio>
#include <cmath>

#include "ode_trapz.h"
#include "burgers.h"

int main () {

    //number of spatial cells
    int nx = 3000;
    //domain length
    double L = 1.75;
    //integration time
    double tint = 3.0;
    //time step
    double dt = tint/5e4;

    //create system
    Burgers<OdeTrapz> sys(nx, L);
    sys.set_name("burgers");
    //initial conditions
    for (int i=0; i<nx; i++) sys.set_sol(i, exp(-(sys.xc[i]-0.5)*(sys.xc[i]-0.5)*50));

    //integrate
    sys.solve_fixed(tint, dt, 31, "out");

    //write the grid to file
    ode_write("out/x", sys.xc, nx);

    return(0);
}

#include <cstdio>
#include <fstream> // Add this line
#include <chrono>

#include "ode_gauss_6.h"
#include "inverted_pendulum.h"

using namespace ode;
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

int main () {

    //create integrator
    InvertedPendulum<OdeGauss6> sys;
    sys.set_name("inverted_pendulum");

    //physical parameters
    sys.g = 9.8, 
    sys.M = 0.5, 
    sys.m = 0.2, 
    sys.b = 0.1, 
    sys.I = 0.006,
    sys.l = 0.3;
    sys.F = 0.0;

    double x0 = 0, xdot0 = 0, theta0 = pi/3, thetadot0 = 0;

    // Controller parameters
    double kpp = 100;
    double kdp = 20;
    double kip = 1;

    double kpc = 0;
    double kdc = 0;
    double kic = 0;

    //initial conditions
    sys.set_sol(0,  x0); //initial position of cart
    sys.set_sol(1,  xdot0); //initial velocity of cart
    sys.set_sol(2, theta0); //initial angle of pendulum
    sys.set_sol(3,  thetadot0); //initial angular velocity of pendulum

    //integration parameters
    double tend = 2;
    double dt = 1e-4;
    double tstart = 0;

    int nsteps = (tend-tstart)/dt+1;

    double x[nsteps], xdot[nsteps], theta[nsteps], thetadot[nsteps];
    x[0] = x0;
    xdot[0] = xdot0;
    theta[0] = theta0;
    thetadot[0] = thetadot0;
    

    //integrate
    printf("integrating system '%s' with method '%s'...\n", sys.get_name(), sys.get_method());

    while (tstart < tend) {
        tstart += dt;
        int index = int(tstart/dt);
        //evalate time to solve

        auto t1 = high_resolution_clock::now();        
        sys.solve_fixed(dt, dt);   
        auto t2 = high_resolution_clock::now();

        /* Getting number of milliseconds as an integer. */
        auto ms_int = duration_cast<milliseconds>(t2 - t1);
        duration<double, std::milli> ms_double = t2 - t1;
        std::cout << ms_double.count() << "ms\n";

        x[int(index)] = sys.get_sol(0);
        xdot[int(index)] = sys.get_sol(1);
        theta[int(index)] = sys.get_sol(2);
        thetadot[int(index)] = sys.get_sol(3);

        double up = -(kpp * theta[index] + kdp * thetadot[index] + kip * (theta[index-1]+dt*thetadot[index-1]));
        double uc = -(kpc * x[index] + kdc * xdot[index] + kic * (x[index-1]+dt*xdot[index-1]));

        sys.F =  up+uc;
    }

    // for (int i = 0; i < nsteps; i++) {
    //     printf("%f %f %f %f\n", x[i], xdot[i], theta[i], thetadot[i]);
    // }

    // write to file_x x the variable x
    std::ofstream file_x("x.txt");
    for (int i = 0; i < nsteps; i++) {
        file_x << x[i] << std::endl;
    }
    file_x.close();

    // write to file xdot the variable xdot
    std::ofstream file_xdot("xdot.txt");
    for (int i = 0; i < nsteps; i++) {
        file_xdot << xdot[i] << std::endl;
    }
    file_xdot.close();

    // write to file theta the variable theta
    std::ofstream file_theta("theta.txt");
    for (int i = 0; i < nsteps; i++) {
        file_theta << theta[i] << std::endl;
    }
    file_theta.close();

    // write to file thetadot the variable thetadot
    std::ofstream file_thetadot("thetadot.txt");
    for (int i = 0; i < nsteps; i++) {
        file_thetadot << thetadot[i] << std::endl;
    }
    file_thetadot.close();

    
    printf("finished.\n%lu steps, %lu function evaluations\n",
    sys.get_nstep(), sys.get_neval());

    return(0);
}

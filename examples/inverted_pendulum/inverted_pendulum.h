#include <cmath>
#include <vector>

#include "ode_io.h"
#include <iostream>
#define pi 3.14159265358979323846264338327950288419716939937510582

namespace ode
{

    template <class Integrator>
    class InvertedPendulum : public Integrator
    {
    public:
        // physical parameters
        double g, // gravity (m/s^2)
            M,    // cart mass (kg)
            m,    // pendulum mass (kg)
            b,    // coefficient of friction for cart (N/m/s)
            l,    // length of pendulum (m)
            I,    // moment of inertia of pendulum (kg*m^2)
            F;    // force applied to cart (N)

        // position vectors
        std::vector<double> x, xdot, theta, thetadot;
        // rclcpp::Node::SharedPtr node_;

        // constructor
        InvertedPendulum() : Integrator(4)
        {
            for (int i = 0; i < 4; i++)
                Integrator::set_sol(i, 0.0);

            // force_subscriber_ = node_->create_subscription<std_msgs::msg::Float64>(
            //     force_topic_, 1, std::bind(&InvertedPendulum::force_callback, this, std::placeholders::_1));
        }

        // system of equations
        void ode_fun(double *solin, double *fout)
        {
            // alias
            double x = solin[0],
                   xdot = solin[1],
                   theta = solin[2],
                   thetadot = solin[3];

            // evaluate
            double den = I * (M + m) + M * m * pow(l, 2);
            fout[0] = xdot;
            fout[1] = -((I + m * l * l) * b * xdot) / den + (pow(m, 2) * g * pow(l, 2) * theta) / den + ((I + m * pow(l, 2)) / den) * F;
            fout[2] = thetadot;
            fout[3] = -((m * l * b * xdot) / den) + ((M + m) * m * g * l * theta) / den + (m * l * F) / den;
        }

    private:
        double f_x()
        {
            return (Integrator::get_sol(0));
        }

        double f_xdot()
        {
            return (Integrator::get_sol(1));
        }

        double f_theta()
        {
            return (Integrator::get_sol(2));
        }

        double f_thetadot()
        {
            return (Integrator::get_sol(3));
        }

        void before_solve()
        {
            x.clear();
            x.push_back(f_x());
            xdot.clear();
            xdot.push_back(f_xdot());
            theta.clear();
            theta.push_back(f_theta());
            thetadot.clear();
            thetadot.push_back(f_thetadot());
        }

        void after_capture(double t)
        {
            (void)t;
            x.push_back(f_x());
            xdot.push_back(f_xdot());
            theta.push_back(f_theta());
            thetadot.push_back(f_thetadot());
        }
        void after_solve()
        {
            // ode_write("out/x", x.data(), x.size());
            // ode_write("out/xdot", xdot.data(), xdot.size());
            // ode_write("out/theta", theta.data(), theta.size());
            // ode_write("out/thetadot", thetadot.data(), thetadot.size());
        }

        // void force_callback(const std_msgs::msg::Float64::SharedPtr msg)
        // {
        //     F = msg->data;
        // }

        // rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr force_subscriber_;
    };

} // namespace ode

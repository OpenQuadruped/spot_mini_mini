#ifndef MINITAUR_INCLUDE_GUARD_HPP
#define MINITAUR_INCLUDE_GUARD_HPP
/// \file
/// \brief MINITAURs library which contains high-level control functionality for the Ghost Minitaur.
#include <vector>
#include <ros/ros.h>

namespace mini
{

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// Note: the fabs function in <cmath> (c++ equivalent of math.h) will
    /// be useful here
    // constexpr are all define in .hpp
    // constexpr allows fcn to be run at compile time and interface with 
    // static_assert tests.
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
        if (fabs(d1 - d2) < epsilon)
        {
            return true;
        } else {
            return false;
        }
    }

    enum Motion {Forward, Backward, Left, Right, CW, CCW, Stop, Recover, ForwardLeft, ForwardRight};
    enum Movement {FB, LR};

    // \brief Struct to store the commanded type of motion, velocity and rate
    struct MiniCommand
    {
        Motion motion = Stop;
        double velocity = 0.0;
        double rate = 0.0;
    };

    // \brief Minitaur class responsible for high-level motion commands
    class Minitaur
    {

    public:
        // \brief Constructor for Minitaur class
        Minitaur();

        // \brief updates the type and velocity of motion to be commanded to the minitaur
        // \param v: linear velocity
        // \param w: angular velocity
        void update_command(const double & v, const double & w);

        // \brief changes the commanded motion from Forward/Backward to Left/Right or vice-versa
        void switch_movement();

        // \brief returns the Minitaur's current command (Motion, v,w) for external use
        // \returns MiniCommand
        MiniCommand return_command();

    private:
        MiniCommand cmd;
        Movement movement;
    };
    
}

#endif

#ifndef STAIRCLIMB_HPP
#define STAIRCLIMB_HPP

#include <array>
#include <complex>

class StairClimb {
    public:
        // Constructor
        StairClimb(bool sim = true);

        std::array<std::array<double, 4>, 2> step();

    private:

};//end class StairClimb

#endif // STAIRCLIMB_HPP

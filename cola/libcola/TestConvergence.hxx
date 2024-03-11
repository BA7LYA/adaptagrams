///
/// @file TestConvergence.hxx
/// @author BA7LYA (1042140025@qq.com)
/// @brief
/// @version 0.1
/// @date 2024-03-12
/// @copyright Copyright (c) 2024
///

namespace cola {

/**
 * @brief  A default functor that is called after each iteration of the layout
 *         algorithm.
 *
 * You can either instantiate ConstrainedFDLayout with an instance of this
 * class setting the tolerance and maxiterations as desired, or create a
 * derived class implementing the operator() to do your own convergence test,
 * or create your own operator() that calls the TestConvergence::operator() in
 * order to do any other post processing you might need, e.g., to animate
 * changes.
 */
class TestConvergence
{
public:
    double old_stress;

    TestConvergence(const double tol = 1e-4, const unsigned maxiterations = 100)
        : tolerance(tol)
        , maxiterations(maxiterations)
    {
        reset();
    }

    virtual ~TestConvergence() {}

public:
    virtual bool operator()(
        const double           new_stress,
        std::valarray<double>& X,
        std::valarray<double>& Y
    )
    {
        COLA_UNUSED(X);
        COLA_UNUSED(Y);

        iterations++;
        // std::cout<<"iteration="<<iterations<<", old_stress="<<old_stress<<",
        // new_stress="<<new_stress<<std::endl;
        if (old_stress == DBL_MAX)
        {
            old_stress = new_stress;
            return iterations >= maxiterations;
        }
        // converged if relative decrease in stress falls below threshold
        // or if stress increases (shouldn't happen for straight majorization)
        bool converged
            = (old_stress - new_stress) / (new_stress + 1e-10) < tolerance
           || iterations > maxiterations;
        old_stress = new_stress;
        return converged;
    }

    void reset()
    {
        old_stress = DBL_MAX;
        iterations = 0;
    }

    const double   tolerance;
    const unsigned maxiterations;
    unsigned       iterations;
};

}  // namespace cola

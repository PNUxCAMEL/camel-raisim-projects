#include <iostream>
#include "camel-tools/optimizer.hpp"

double objFunc_main(Eigen::VectorXd x)
{
    return (x[0] - 5.0) * (x[0] - 5.0) + (x[1] - 3.0) * (x[1] - 3.0);
}

int main()
{
    GradientDescentOptimizer GDOptimizer;
    double (* objFunc)(Eigen::VectorXd) = objFunc_main;
    GDOptimizer.SetObjectiveFunction(objFunc);

    Eigen::VectorXd y = Eigen::VectorXd(2);
    y << 0.0, 0.0;
    GDOptimizer.SetInitialState(y);
    GDOptimizer.Solve();
    std::cout << GDOptimizer.GetState() << std::endl;
}
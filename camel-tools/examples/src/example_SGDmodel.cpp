//
// Created by hs on 22. 7. 8.
//

#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

//TODO: Hosun have to clean up this class.
int main(int argc, char* argv[]) {
    raisim::World world;
    world.setTimeStep(0.001);

    auto ground = world.addGround(0, "gnd");
    ground->setAppearance("wheat");

    auto quad = world.addArticulatedSystem("\\home\\hs\\raisimLib\\camel-code-raisim-cpp\\rsc\\camel_SRB_quad.urdf");
    auto a1 = world.addArticulatedSystem("\\home\\hs\\raisimLib\\rsc\\a1\\urdf\\a1.urdf");

    Eigen::VectorXd jointNominalConfig(quad->getGeneralizedCoordinateDim()), jointVelocityTarget(quad->getDOF());
    jointNominalConfig << 0, 0, 0.5, 1.0, 0.0, 0.0, 0.0, 0.0, 0.5, -0.9, -0.0, 0.5, -0.9, 0.0, 0.5, -0.9, -0.0, 0.5, -0.9;
    jointVelocityTarget.setZero();

    Eigen::VectorXd jointPgain(quad->getDOF()), jointDgain(quad->getDOF());
    jointPgain.tail(12).setConstant(80.0);
    jointDgain.tail(12).setConstant(10.0);

    quad->setGeneralizedCoordinate(jointNominalConfig);
    quad->setGeneralizedForce(Eigen::VectorXd::Zero(quad->getDOF()));
    quad->setPdGains(jointPgain, jointDgain);
    quad->setPdTarget(jointNominalConfig, jointVelocityTarget);
    quad->setName("quad");

    jointNominalConfig[1] = 1.0;
    a1->setGeneralizedCoordinate(jointNominalConfig);
    a1->setGeneralizedForce(Eigen::VectorXd::Zero(a1->getDOF()));
    a1->setPdGains(jointPgain, jointDgain);
    a1->setPdTarget(jointNominalConfig, jointVelocityTarget);
    a1->setName("a1");


    raisim::RaisimServer server(&world);
    server.launchServer();
    server.focusOn(quad);

    for (int i = 0; i < 200000000; i++)
    {
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        server.integrateWorldThreadSafe();
    }
    server.killServer();
}
#include <gtest/gtest.h>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/tasks/torque/Postural.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <cmath>
#include <idynutils/tests_utils.h>
#include <idynutils/RobotUtils.h>
#include <fstream>
#include <ros/ros.h>

using namespace yarp::math;

#define toRad(X) (X*M_PI/180.)

namespace{

class testTorquePostural : public ::testing::Test {
 protected:

  testTorquePostural()
  {

  }

  virtual ~testTorquePostural() {
  }

  virtual void SetUp() {
  }

  virtual void TearDown() {
  }


};

TEST_F(testTorquePostural, testTorquePostural0) {
    // Start YARP Server
    tests_utils::startYarpServer();
    // Load a world
    std::string world_path = std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.world";
    //if(OPENSOT_SIMULATION_TESTS_VISUALIZATION)
        tests_utils::startGazebo(world_path);
    //else
    //    tests_utils::startGZServer(world_path);
    sleep(4);

    //To control the robot we need RobotUtils
    RobotUtils coman_robot("testTorquePostural",
                     "coman",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");

    yarp::sig::Vector q = coman_robot.sensePosition();

    coman_robot.idynutils.updateiDyn3Model(q, true);
    coman_robot.idynutils.setFloatingBaseLink(coman_robot.idynutils.left_leg.end_effector_name);
    coman_robot.setTorqueMode();

    //SET UP OPTIMIZATION
    yarp::sig::Vector qd = q;
    double stiffnessd = 100.0;
    yarp::sig::Vector taud(q.size(), 0.0);

    OpenSoT::tasks::torque::Postural::Ptr torque_postural_task(
                new OpenSoT::tasks::torque::Postural(qd, stiffnessd));

    OpenSoT::solvers::QPOases_sot::Stack stack_of_tasks_torque;
    stack_of_tasks_torque.push_back(torque_postural_task);
    OpenSoT::solvers::QPOases_sot::Ptr sot(
                new OpenSoT::solvers::QPOases_sot(stack_of_tasks_torque,2E10));
    sot->solve(taud);
    coman_robot.move(taud);

    int steps = 30000;
    double dT = 0.001;
    for(unsigned int i = 0; i < steps; ++i)
    {
        double tic = yarp::os::Time::now();

        q = coman_robot.sensePosition();

        coman_robot.idynutils.updateiDyn3Model(q, true);


        if(i == 5000)
        {
            qd[coman_robot.idynutils.left_arm.joint_numbers[0]] = -1.52;
            qd[coman_robot.idynutils.left_arm.joint_numbers[1]] = 0.96;
            qd[coman_robot.idynutils.left_arm.joint_numbers[2]] = -0.94;
            qd[coman_robot.idynutils.left_arm.joint_numbers[3]] = -1.18;
            qd[coman_robot.idynutils.left_arm.joint_numbers[4]] = 1.25;
            qd[coman_robot.idynutils.left_arm.joint_numbers[5]] = -0.32;
            qd[coman_robot.idynutils.left_arm.joint_numbers[6]] = 0.66;

            torque_postural_task->setReference(qd);
        }
        torque_postural_task->update(q);

        if(sot->solve(taud))
            coman_robot.move(taud);


        double toc = yarp::os::Time::now();

        if((toc-tic) < dT)
            yarp::os::Time::delay(dT - (toc-tic));
        else
            std::cout<<"we are too slow!"<<std::endl;
    }


    tests_utils::stopGazebo();
    sleep(10);
    tests_utils::stopYarpServer();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}

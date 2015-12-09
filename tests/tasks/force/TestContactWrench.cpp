#include <gtest/gtest.h>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/tasks/force/ContactWrench.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <cmath>
#include <idynutils/tests_utils.h>
#include <idynutils/RobotUtils.h>
#include <fstream>

using namespace yarp::math;

namespace{
class testForceContactWrench : public ::testing::Test {
 protected:

  testForceContactWrench()
  {

  }

  virtual ~testForceContactWrench() {
  }

  virtual void SetUp() {
  }

  virtual void TearDown() {
  }
};

TEST_F(testForceContactWrench, testContactWrenchDesired) {
    iDynUtils coman("coman",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");

    yarp::sig::Vector w_desired(12, 0.0);

    OpenSoT::tasks::force::ContactWrench::Ptr cw(new OpenSoT::tasks::force::ContactWrench(w_desired, coman));
    EXPECT_EQ(cw->getA().rows(), 12);
    EXPECT_EQ(cw->getA().cols(), 12);

    OpenSoT::solvers::QPOases_sot::Stack stack_of_tasks;
    stack_of_tasks.push_back(cw);

    yarp::sig::Vector wrench_d(12, 0.0);
    OpenSoT::solvers::QPOases_sot::Ptr sot(
                new OpenSoT::solvers::QPOases_sot(stack_of_tasks,2E10));
    sot->solve(wrench_d);

    for(unsigned int i = 0; i < w_desired.size(); ++i)
        EXPECT_EQ(wrench_d[i], w_desired[i]);

    for(unsigned int i = 0; i < w_desired.size(); ++i)
        w_desired[i] = i;

    cw->setReference(w_desired);
    cw->update(w_desired);

    sot->solve(wrench_d);
    for(unsigned int i = 0; i < w_desired.size(); ++i)
        EXPECT_EQ(wrench_d[i], w_desired[i]);

}
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#include <idynutils/idynutils.h>
#include <idynutils/tests_utils.h>
#include <kdl/frames.hpp>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/utils/DefaultHumanoidStack.h>
#include <OpenSoT/utils/Previewer.h>
#include <gtest/gtest.h>

#define dT 1.0e-2

using namespace yarp::math;

namespace OpenSoT {


/**
 * @brief The MyTrajGen class dummy trajectory generator:
 *        it creates a trajectory that lasts t seconds, where the position
 *        of the frame gets interpolated from the starting position to the
 *        end position. The rotation remains constant.
 *        The goal pose is with p=finalPose, R=initialR
 */
class MyTrajGen
{
    KDL::Frame b;
    KDL::Frame f;
    double t;
public:
    typedef boost::shared_ptr<MyTrajGen> Ptr;
    MyTrajGen(yarp::sig::Matrix yb,
              yarp::sig::Matrix yf, double t) : t(t) { bool res = YarptoKDL(yb, b) &&
                                             YarptoKDL(yf, f); assert( res &&
                                             "Error converting yarp::Matrix to kdl::Frame"); }
    KDL::Frame Pos(double time) {
        KDL::Frame ref = b;
        if(time > t)
            time = t;

        ref.p = (t-time)/t*b.p + time/t*f.p;
        return ref;
    }
    KDL::Twist Vel(double time) { return KDL::Twist(); }
    double Duration() { return t; }
};

yarp::sig::Vector getGoodInitialPosition(iDynUtils& idynutils) {
    yarp::sig::Vector q(idynutils.iDyn3_model.getNrOfDOFs(), 0.0);
    yarp::sig::Vector leg(idynutils.left_leg.getNrOfDOFs(), 0.0);
    leg[0] = -25.0 * M_PI/180.0;
    leg[3] =  50.0 * M_PI/180.0;
    leg[5] = -25.0 * M_PI/180.0;
    idynutils.fromRobotToIDyn(leg, q, idynutils.left_leg);
    idynutils.fromRobotToIDyn(leg, q, idynutils.right_leg);
    yarp::sig::Vector arm(idynutils.left_arm.getNrOfDOFs(), 0.0);
    arm[0] = 20.0 * M_PI/180.0;
    arm[1] = 20.0 * M_PI/180.0;
    arm[3] = -80.0 * M_PI/180.0;
    idynutils.fromRobotToIDyn(arm, q, idynutils.left_arm);
    arm[1] = -arm[1];
    idynutils.fromRobotToIDyn(arm, q, idynutils.right_arm);
    return q;
}

class testPreviewer: public ::testing::Test
{
public:
    typedef OpenSoT::Previewer<MyTrajGen> Previewer;

protected:
    iDynUtils _robot;
    yarp::sig::Vector q;
    OpenSoT::DefaultHumanoidStack DHS;
    Previewer::Ptr previewer;
    OpenSoT::AutoStack::Ptr autostack;

    testPreviewer() :
        _robot("coman",
                std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf"),
        q(getGoodInitialPosition(_robot)),
        DHS((_robot.updateiDyn3Model(q, true),_robot),
              dT,
              q)
    {
        // defining a stack composed of size two,
        // where the task of first priority is an aggregated of leftArm and rightArm,
        // rightArm contains a convexHull constraint;
        // the task at the second priority level is an aggregated of rightLeg and leftLeg,
        // and the stack is subject to bounds jointLimits and velocityLimits

        OpenSoT::AutoStack::Ptr temp = ( (DHS.leftArm + DHS.rightArm) /
                                         (DHS.rightLeg + DHS.leftLeg) ) << DHS.jointLimits << DHS.velocityLimits;
        autostack = temp;
        //autostack->update(q);

        /* by default, we have constant references */
        MyTrajGen::Ptr trajLeftArm(new MyTrajGen(DHS.leftArm->getActualPose(),
                                                 DHS.leftArm->getActualPose(),
                                                 1.0));
        MyTrajGen::Ptr trajRightArm(new MyTrajGen(DHS.rightArm->getActualPose(),
                                                  DHS.rightArm->getActualPose(),
                                                  1.0));

        Previewer::TrajectoryBindings bindings;
        bindings.push_back(Previewer::TrajBinding(trajLeftArm, DHS.leftArm));
        bindings.push_back(Previewer::TrajBinding(trajRightArm, DHS.rightArm));

        previewer.reset(new Previewer(dT, _robot, autostack, bindings));
    }

    virtual ~testPreviewer() {

    }

    virtual void SetUp() {
        DHS.leftArm->setOrientationErrorGain(0.1);
        DHS.rightArm->setOrientationErrorGain(0.1);
    }

    virtual void TearDown() {

    }

    bool dynamicCastWorks()
    {
        OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr comTask(DHS.com);
        OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr lArmTask(DHS.leftArm);
        
        using namespace OpenSoT::tasks::velocity;

        return
                CoM::isCoM(DHS.com) &&
                CoM::isCoM(comTask) &&
                CoM::isCoM(CoM::asCoM(comTask)) &&
                !Cartesian::isCartesian(DHS.com) &&
                !Cartesian::isCartesian(comTask) &&
                !Cartesian::isCartesian(CoM::asCoM(comTask)) &&
                Cartesian::isCartesian(DHS.leftArm) &&
                Cartesian::isCartesian(lArmTask) &&
                Cartesian::isCartesian(Cartesian::asCartesian(lArmTask)) &&
                !CoM::isCoM(DHS.leftArm) &&
                !CoM::isCoM(lArmTask) &&
                !CoM::isCoM(Cartesian::asCartesian(lArmTask));
    }

    bool cartesianPoseChangedWorks()
    {
        double threshold = 1e-2;
        q = _robot.iDyn3_model.getAng();
        _robot.updateiDyn3Model(q, true);
        autostack->update(q);
        previewer->resetPreviewer();

        // first time we call it, it always returns true
        if(!previewer->cartesianPoseChanged(DHS.leftArm,threshold))
        {
            std::cout << "--- cartesianPoseChanged should return true "
                      << "when called first time after a reset" << std::endl;
            return false;
        }

        // but if we didn't move, it will then return false
        if(previewer->cartesianPoseChanged(DHS.leftArm,threshold))
        {
            std::cout << "--- cartesianPoseChanged is returning true while "
                      << "the cartesian pose did not change" << std::endl;
            return false;
        }

        yarp::sig::Matrix x = DHS.leftArm->getActualPose();
        yarp::sig::Matrix x_des = x;
        x_des(0,3) = x(0,3) + 10.0*threshold;
        DHS.leftArm->setReference(x_des);

        yarp::sig::Vector dq;
        for(unsigned int i = 0; i < 100; ++i)
        {
            _robot.updateiDyn3Model(q, true);
            autostack->update(q);
            if(previewer->solver->solve(dq))
                q+=dq;
            else
                std::cout << "--- Error: unable to solve stack" << std::endl;
        }

        if(yarp::math::norm(DHS.leftArm->getActualPose().getCol(3) - x.getCol(3)) < threshold)
        {
            std::cout << "--- Error following desired reference:" << std::endl;
            std::cout << "--- actual pose is: " << DHS.leftArm->getActualPose().getCol(3).toString() << std::endl
                      << "--- desired pose is: " << DHS.leftArm->getReference().getCol(3).toString() << std::endl;
            return false; // why didn't we move?
        }

        // but if we didn't move, it will then return false
        if(!previewer->cartesianPoseChanged(DHS.leftArm,threshold))
        {
            std::cout << "--- cartesianPoseChanged is returning false while "
                      << "the cartesian pose changed" << std::endl;
            return false;
        }


        return true;
    }

    bool jointSpaceConfigurationChangedWorks()
    {
        yarp::sig::Vector zero = _robot.iDyn3_model.getAng(); zero.zero();
        _robot.updateiDyn3Model(zero);
        previewer->resetPreviewer();

        if(previewer->jointSpaceConfigurationChanged(1e2))
            return false;   //by default, qNode is 0

        double threshold = 1e-3;

        do
        {
            previewer->q = tests_utils::getRandomAngles(
                _robot.iDyn3_model.getJointBoundMin(),
                _robot.iDyn3_model.getJointBoundMax(),
                _robot.iDyn3_model.getNrOfDOFs());
        } while(yarp::math::norm(previewer->q - zero) <= threshold);


        if(!previewer->jointSpaceConfigurationChanged(threshold))
        {
            std::cout << "Joint space configuration changed, but "
                      << "jointSpaceConfigurationChanged returns false" << std::endl;
            return false;
        }


        return true;
    }

    bool shouldCheckCollisionWorks()
    {

        double threshold = 1e-2;
        q = _robot.iDyn3_model.getAng();
        _robot.updateiDyn3Model(q, true);
        autostack->update(q);
        previewer->resetPreviewer();

        // first time we call it, it always returns true
        if(!previewer->shouldCheckCollision(threshold))
        {
            std::cout << "--- shouldCheckCollision() should return true "
                      << "when called first time after a reset" << std::endl;
            return false;
        } else std::cout << "+++ First call to shouldCheckCollision "
                         << "succesfully returns true" << std::endl;

        // but if we didn't move, it will then return false
        if(previewer->shouldCheckCollision(threshold))
        {
            std::cout << "--- shouldCheckCollision is returning true while "
                      << "the cartesian pose did not change" << std::endl;
            return false;
        }

        yarp::sig::Matrix x = DHS.leftArm->getActualPose();
        yarp::sig::Matrix x_des = x;
        x_des(0,3) = x(0,3) + 10.0*threshold;
        DHS.leftArm->setReference(x_des);
        DHS.rightArm->setReference(DHS.rightArm->getActualPose());

        DHS.leftArm->setOrientationErrorGain(0.0);
        DHS.rightArm->setOrientationErrorGain(0.0);
        yarp::sig::Vector dq;
        for(unsigned int i = 0; i < 100; ++i)
        {
            if(previewer->solver->solve(dq))
            {
                q+=dq;

                _robot.updateiDyn3Model(q, true);
                autostack->update(q);

                double dXL = yarp::math::norm(DHS.leftArm->getActualPose().getCol(3) - x.getCol(3));
                double dXR = yarp::math::norm(DHS.rightArm->getActualPose().getCol(3) - DHS.rightArm->getReference().getCol(3));
                std::cout << "dXL: " << dXL << std::endl;
                std::cout << "dXR: " << dXR << std::endl;
                if( dXL >= threshold || dXR >= threshold)
                {
                    x = DHS.leftArm->getActualPose();
                    if(!previewer->shouldCheckCollision(threshold))
                    {
                        std::cout << "--- Error! cartesian reference changed, but "
                                  << "shouldCheckCollision did not return true" << std::endl;
                        return false;
                    }
                } else {
                    if(previewer->shouldCheckCollision(threshold))
                    {
                        std::cout << "--- Error! cartesian reference did not change much, but "
                                  << "shouldCheckCollision did return true" << std::endl;
                        return false;
                    }
                }
            }
            else
                std::cout << "--- Error: unable to solve stack" << std::endl;
        }

        // we didn't move, shouldCheckCollision should return false
        if(previewer->shouldCheckCollision(threshold))
        {
            std::cout << "--- shouldCheckCollision is returning true while "
                      << "the cartesian pose did not change" << std::endl;
            return false;
        }

        yarp::sig::Vector previous_q = q;
        do
        {
            q = tests_utils::getRandomAngles(
                _robot.iDyn3_model.getJointBoundMin(),
                _robot.iDyn3_model.getJointBoundMax(),
                _robot.iDyn3_model.getNrOfDOFs());
        } while(yarp::math::norm(previous_q - q) <= threshold);
        _robot.updateiDyn3Model(q,true);
        autostack->update(q);

        if(!previewer->shouldCheckCollision())
        {
            std::cout << "Joint space configuration changed, but "
                      << "shouldCheckCollision returns false" << std::endl;
            return false;
        }


        return true;
    }

};

TEST_F(testPreviewer, testDynamicCast)
{
    ASSERT_TRUE(dynamicCastWorks());
}

TEST_F(testPreviewer, checkStaticConvergence)
{
    ASSERT_FALSE(_robot.checkCollision());

    Previewer::Results results;
    bool preview_success = previewer->check(0.1,3,&results);
    EXPECT_FALSE(preview_success);
    EXPECT_EQ(results.num_failures,0);

    preview_success = previewer->check(1.0,3,&results);
    EXPECT_TRUE(preview_success);

    if(!preview_success)
    {
        std::cout << "Trajectory unfeasible!";

        std::cout << "Logged " << results.num_failures << " failures:" << std::endl;
        for(typename Previewer::Results::LogMap::iterator it =
            results.log.begin();
            it != results.log.end(); ++it)
        {
            if(it->second.failures.size() > 0)
            {
                std::cout << "@t:" << it->first;
                for(typename std::list<Previewer::Results::Reason>::iterator
                        it_f = it->second.failures.begin();
                    it_f != it->second.failures.end();
                    ++it_f)
                    std::cout << " - " << Previewer::Results::reasonToString(*it_f);
                std::cout << std::endl;
            }
        }
    }
    else
    {
        std::cout << "\nLogged " << results.log.size() << " trajectory nodes:" << std::endl;
        for(typename Previewer::Results::LogMap::iterator it =
            results.log.begin();
            it != results.log.end(); ++it)
        {
            std::cout << "@t:" << it->first
                      << " - " << it->second.q.toString()
                      << std::endl;
        }
    }
}

TEST_F(testPreviewer, checkFeasibleConvergence)
{
    /* feasible trajectory: 10 cm in 10secs */
    yarp::sig::Matrix lb = DHS.leftArm->getActualPose();
    yarp::sig::Matrix lf = lb; lf(0,3) = lb(0,3)+.1;
    yarp::sig::Matrix rb = DHS.rightArm->getActualPose();
    yarp::sig::Matrix rf = rb; rf(0,3) = rb(0,3)+.1;
    double eL, eR;
    double duration = 10.0;

    MyTrajGen::Ptr trajLeftArm(new MyTrajGen(lb, lf, duration));
    MyTrajGen::Ptr trajRightArm(new MyTrajGen(rb, rf, duration));

    Previewer::TrajectoryBindings bindings;
    bindings.push_back(Previewer::TrajBinding(trajLeftArm, DHS.leftArm, 1e-2, 1e-3));
    bindings.push_back(Previewer::TrajBinding(trajRightArm, DHS.rightArm, 1e-2, 1e-3));

    previewer.reset(); // TODO create issue to find out the causes of conflict when two solvers are created at the same time
    previewer.reset(new Previewer(dT, _robot, autostack, bindings));

    eL = yarp::math::norm(
        DHS.leftArm->getActualPose().getCol(3).subVector(0,2)-
        lf.getCol(3).subVector(0,2));
    eR = yarp::math::norm(
        DHS.rightArm->getActualPose().getCol(3).subVector(0,2)-
        rf.getCol(3).subVector(0,2));
    EXPECT_FALSE(eL < 1e-2);
    EXPECT_FALSE(eR < 1e-2);

    Previewer::Results results;
    bool preview_success = previewer->check(duration,3,&results);
    eL = yarp::math::norm(
        DHS.leftArm->getActualPose().getCol(3).subVector(0,2)-
        lf.getCol(3).subVector(0,2));
    eR = yarp::math::norm(
        DHS.rightArm->getActualPose().getCol(3).subVector(0,2)-
        rf.getCol(3).subVector(0,2));
    EXPECT_TRUE(eL < 1e-3);
    EXPECT_TRUE(eR < 1e-3);

    EXPECT_TRUE(preview_success);
}

TEST_F(testPreviewer, checkAutoConvergenceCheck)
{
    /* feasible trajectory: 10 cm in 10secs */
    yarp::sig::Matrix lb = DHS.leftArm->getActualPose();
    yarp::sig::Matrix lf = lb; lf(0,3) = lb(0,3)+.1;
    yarp::sig::Matrix rb = DHS.rightArm->getActualPose();
    yarp::sig::Matrix rf = rb; rf(0,3) = rb(0,3)+.1;
    double duration = 10.0;

    MyTrajGen::Ptr trajLeftArm(new MyTrajGen(lb, lf, duration));
    MyTrajGen::Ptr trajRightArm(new MyTrajGen(rb, rf, duration));

    Previewer::TrajectoryBindings bindings;
    bindings.push_back(Previewer::TrajBinding(trajLeftArm, DHS.leftArm));
    bindings.push_back(Previewer::TrajBinding(trajRightArm, DHS.rightArm));

    previewer.reset();
    previewer.reset(new Previewer(dT, _robot, autostack, bindings));

    Previewer::Results results;
    bool preview_success = previewer->check(std::numeric_limits<double>::infinity(),
                                            3,&results);
    ASSERT_TRUE(preview_success);
    EXPECT_TRUE(results.log.rbegin()->first < 2.0*duration);
}

TEST_F(testPreviewer, checkUnfeasibleConvergence)
{
    /* feasible trajectory: 1 cm in .2sec -> commanding 10cmin .2sec (too fast) */
    yarp::sig::Matrix lb = DHS.leftArm->getActualPose();
    yarp::sig::Matrix lf = lb; lf(0,3) = lb(0,3)+.1;
    yarp::sig::Matrix rb = DHS.rightArm->getActualPose();
    yarp::sig::Matrix rf = rb; rf(0,3) = rb(0,3)+.1;
    double eL, eR;
    double duration = .2;

    MyTrajGen::Ptr trajLeftArm(new MyTrajGen(lb, lf, duration));
    MyTrajGen::Ptr trajRightArm(new MyTrajGen(rb, rf, duration));

    Previewer::TrajectoryBindings bindings;
    bindings.push_back(Previewer::TrajBinding(trajLeftArm, DHS.leftArm,.1));
    bindings.push_back(Previewer::TrajBinding(trajRightArm, DHS.rightArm,.1));

    previewer.reset();
    previewer.reset(new Previewer(dT, _robot, autostack, bindings));

    eL = yarp::math::norm(
        DHS.leftArm->getActualPose().getCol(3).subVector(0,2)-
        lf.getCol(3).subVector(0,2));
    eR = yarp::math::norm(
        DHS.rightArm->getActualPose().getCol(3).subVector(0,2)-
        rf.getCol(3).subVector(0,2));
    EXPECT_FALSE(eL < 1e-2);
    EXPECT_FALSE(eR < 1e-2);

    Previewer::Results results;
    bool preview_success = previewer->check(duration,3,&results);

    eL = yarp::math::norm(
        DHS.leftArm->getActualPose().getCol(3).subVector(0,2)-
        lf.getCol(3).subVector(0,2));
    eR = yarp::math::norm(
        DHS.rightArm->getActualPose().getCol(3).subVector(0,2)-
        rf.getCol(3).subVector(0,2));
    EXPECT_FALSE(eL < 1e-2);
    EXPECT_FALSE(eR < 1e-2);

    ASSERT_FALSE(preview_success);
    EXPECT_EQ(results.num_failures, 0);
    EXPECT_DOUBLE_EQ(results.log.rbegin()->first, duration);
}

TEST_F(testPreviewer, checkUnfeasibleBoundedness)
{
    /* feasible trajectory: 1 cm in .2sec */
    yarp::sig::Matrix lb = DHS.leftArm->getActualPose();
    yarp::sig::Matrix lf = lb; lf(0,3) = lb(0,3)+.1;
    yarp::sig::Matrix rb = DHS.rightArm->getActualPose();
    yarp::sig::Matrix rf = rb; rf(0,3) = rb(0,3)+.1;
    double eL, eR;
    double duration = .2;

    MyTrajGen::Ptr trajLeftArm(new MyTrajGen(lb, lf, duration));
    MyTrajGen::Ptr trajRightArm(new MyTrajGen(rb, rf, duration));

    Previewer::TrajectoryBindings bindings;
    bindings.push_back(Previewer::TrajBinding(trajLeftArm, DHS.leftArm, 5.e-2));
    bindings.push_back(Previewer::TrajBinding(trajRightArm, DHS.rightArm, 5.e-2));

    previewer.reset();
    previewer.reset(new Previewer(dT, _robot, autostack, bindings));

    eL = yarp::math::norm(
        DHS.leftArm->getActualPose().getCol(3).subVector(0,2)-
        lf.getCol(3).subVector(0,2));
    eR = yarp::math::norm(
        DHS.rightArm->getActualPose().getCol(3).subVector(0,2)-
        rf.getCol(3).subVector(0,2));
    EXPECT_FALSE(eL < 1e-2);
    EXPECT_FALSE(eR < 1e-2);

    Previewer::Results results;
    bool preview_success = previewer->check(10.0*duration,3,&results);

    eL = yarp::math::norm(
        DHS.leftArm->getActualPose().getCol(3).subVector(0,2)-
        lf.getCol(3).subVector(0,2));
    eR = yarp::math::norm(
        DHS.rightArm->getActualPose().getCol(3).subVector(0,2)-
        rf.getCol(3).subVector(0,2));

    EXPECT_TRUE(eL < 1e-2);
    EXPECT_TRUE(eR < 1e-2);

    ASSERT_FALSE(preview_success);

    bool error_unbounded = false;
    for(typename Previewer::Results::LogMap::iterator it =
        results.log.begin();
        it != results.log.end(); ++it)
    {
        if(it->second.failures.size() > 0)
        {
            for(typename std::list<Previewer::Results::Reason>::iterator
                    it_f = it->second.failures.begin();
                it_f != it->second.failures.end();
                ++it_f)
                if(*it_f == Previewer::Results::ERROR_UNBOUNDED)
                    error_unbounded = true;
        }
    }

    EXPECT_TRUE(error_unbounded);
}

TEST_F(testPreviewer, checkCollision)
{
    /* feasible trajectory with collision on l_arm: 10 cm in .1sec */
    yarp::sig::Matrix lb = DHS.leftArm->getActualPose();
    yarp::sig::Matrix lf = lb; lf(1,3) = 0.;
    yarp::sig::Matrix rb = DHS.rightArm->getActualPose();
    yarp::sig::Matrix rf = rb;
    double duration = 10;

    MyTrajGen::Ptr trajLeftArm(new MyTrajGen(lb, lf, duration));
    MyTrajGen::Ptr trajRightArm(new MyTrajGen(rb, rf, duration));

    Previewer::TrajectoryBindings bindings;
    bindings.push_back(Previewer::TrajBinding(trajLeftArm, DHS.leftArm));
    bindings.push_back(Previewer::TrajBinding(trajRightArm, DHS.rightArm));

    previewer.reset();
    previewer.reset(new Previewer(dT, _robot, autostack, bindings));

    Previewer::Results results;
    bool preview_success = previewer->check(duration,3,&results);
    EXPECT_FALSE(preview_success);

    bool self_collision = false;
    for(typename Previewer::Results::LogMap::iterator it =
        results.log.begin();
        it != results.log.end(); ++it)
    {
        if(it->second.failures.size() > 0)
        {
            for(typename std::list<Previewer::Results::Reason>::iterator
                    it_f = it->second.failures.begin();
                it_f != it->second.failures.end();
                ++it_f)
                if(*it_f == Previewer::Results::COLLISION_EVENT)
                    self_collision = true;
        }
    }

    EXPECT_TRUE(self_collision);
}

TEST_F(testPreviewer, testShouldCheckCollision)
{
    ASSERT_TRUE(cartesianPoseChangedWorks());

    ASSERT_TRUE(jointSpaceConfigurationChangedWorks());

    ASSERT_TRUE(shouldCheckCollisionWorks());
}

TEST_F(testPreviewer, resultsSum)
{
    /* feasible trajectory: 10 cm in 10secs */
    yarp::sig::Matrix lb = DHS.leftArm->getActualPose();
    yarp::sig::Matrix lf = lb; lf(0,3) = lb(0,3)+.1;
    yarp::sig::Matrix rb = DHS.rightArm->getActualPose();
    yarp::sig::Matrix rf = rb; rf(0,3) = rb(0,3)+.1;
    double duration = 10.0;

    MyTrajGen::Ptr trajLeftArm(new MyTrajGen(lb, lf, duration));
    MyTrajGen::Ptr trajRightArm(new MyTrajGen(rb, rf, duration));

    MyTrajGen::Ptr trajLeftArmBack(new MyTrajGen(lf, lb, duration));
    MyTrajGen::Ptr trajRightArmBack(new MyTrajGen(rf, rb, duration));

    Previewer::TrajectoryBindings bindings, bindingsBack;
    bindings.push_back(Previewer::TrajBinding(trajLeftArm, DHS.leftArm));
    bindings.push_back(Previewer::TrajBinding(trajRightArm, DHS.rightArm));
    bindingsBack.push_back(Previewer::TrajBinding(trajLeftArmBack, DHS.leftArm));
    bindingsBack.push_back(Previewer::TrajBinding(trajRightArmBack, DHS.rightArm));

    previewer.reset();
    previewer.reset(new Previewer(dT, _robot, autostack, bindings));

    Previewer::Results results_1;
    bool preview_success = previewer->check(std::numeric_limits<double>::infinity(),
                                            3,&results_1);

    previewer.reset();
    previewer.reset(new Previewer(dT, _robot, autostack, bindingsBack));

    Previewer::Results results_2;
    preview_success = previewer->check(std::numeric_limits<double>::infinity(),
                                       3,&results_2);

    std::cout << "Collating results.." << std::endl; std::cout.flush();
    Previewer::Results results = results_1 + results_2;
    ASSERT_TRUE(results.log.rbegin()->first >= 20.0);
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

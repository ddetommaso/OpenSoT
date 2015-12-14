#include <gtest/gtest.h>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/tasks/force/CoM.h>
#include <OpenSoT/tasks/force/CoP.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <cmath>
#include <idynutils/tests_utils.h>
#include <idynutils/RobotUtils.h>
#include <fstream>
#include <OpenSoT/tasks/velocity/Interaction.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/all.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <idynutils/cartesian_utils.h>
#include <OpenSoT/constraints/force/FrictionCone.h>
#include <OpenSoT/constraints/velocity/Dynamics.h>
#include <OpenSoT/tasks/force/DesiredWrench.h>
#include <ros/ros.h>

using namespace yarp::math;

namespace{
class logger{
public:
    logger(const std::string& name, const int size = 0)
    {
        if(size > 0)
            data.reserve(size);

        std::string file_name = "testCoPForce_" + name + ".m";
        file.open(file_name);
        file<<name<<" = ["<<std::endl;
    }

    ~logger()
    {
        if(file.is_open()){
            for(unsigned int i = 0; i < data.size(); ++i)
                file<<data[i].toString()<<std::endl;
            file<<"];"<<std::endl;
            file.close();}
    }

    void log(const yarp::sig::Vector& data_)
    {
        data.push_back(data_);
    }

    void write()
    {
        for(unsigned int i = 0; i < data.size(); ++i)
            file<<data[i].toString()<<std::endl;
        file<<"];"<<std::endl;
        file.close();
    }

    std::vector<yarp::sig::Vector> data;
    std::ofstream file;

};

class testForceCoP : public ::testing::Test {
 protected:

  testForceCoP()
  {

  }

  void startEverything()
  {
      // Start YARP Server
//      tests_utils::startYarpServer();
//      // Load a world
//      std::string world_path = std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman_fixed.world";
//      if(OPENSOT_SIMULATION_TESTS_VISUALIZATION)
//          tests_utils::startGazebo(world_path);
//      else
//          tests_utils::startGZServer(world_path);
//      sleep(4);
  }

  void closeEverything()
  {
      tests_utils::stopGazebo();
      sleep(10);
      tests_utils::stopYarpServer();
  }

  virtual ~testForceCoP() {
  }

  virtual void SetUp() {
  }

  virtual void TearDown() {
  }


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
    arm[0] = 0.0 * M_PI/180.0;
    arm[1] = 10.0 * M_PI/180.0;
    arm[3] = 0.0 * M_PI/180.0;
    idynutils.fromRobotToIDyn(arm, q, idynutils.left_arm);
    arm[1] = -arm[1];
    idynutils.fromRobotToIDyn(arm, q, idynutils.right_arm);
    return q;
}

void contactWrenchInFTFrame(const RobotUtils& coman_robot, yarp::sig::Vector& contact_wrench_d)
{
    yarp::sig::Vector contact_wrench_d_lankle = yarp::math::cat(
                contact_wrench_d.subVector(0,2),contact_wrench_d.subVector(6,8));
    yarp::sig::Vector contact_wrench_d_rankle = yarp::math::cat(
                contact_wrench_d.subVector(3,5),contact_wrench_d.subVector(9,11));
    KDL::Wrench contact_wrench_d_lankle_KDL;
    cartesian_utils::fromYarpVectortoKDLWrench(contact_wrench_d_lankle, contact_wrench_d_lankle_KDL);
    KDL::Wrench contact_wrench_d_rankle_KDL;
    cartesian_utils::fromYarpVectortoKDLWrench(contact_wrench_d_rankle, contact_wrench_d_rankle_KDL);

    KDL::Frame ft_left_leg_T_world = coman_robot.idynutils.iDyn3_model.getPositionKDL(
        coman_robot.idynutils.iDyn3_model.getLinkIndex("l_leg_ft"),
        true);
    KDL::Frame ft_right_leg_T_world = coman_robot.idynutils.iDyn3_model.getPositionKDL(
        coman_robot.idynutils.iDyn3_model.getLinkIndex("r_leg_ft"),
        true);

    contact_wrench_d_lankle_KDL = ft_left_leg_T_world.M * contact_wrench_d_lankle_KDL;
    contact_wrench_d_rankle_KDL = ft_right_leg_T_world.M * contact_wrench_d_rankle_KDL;

    cartesian_utils::fromKDLWrenchtoYarpVector(contact_wrench_d_lankle_KDL, contact_wrench_d_lankle);
    cartesian_utils::fromKDLWrenchtoYarpVector(contact_wrench_d_rankle_KDL, contact_wrench_d_rankle);

    contact_wrench_d = yarp::math::cat(
                yarp::math::cat(contact_wrench_d_lankle.subVector(0,2), contact_wrench_d_rankle.subVector(0,2)),
                yarp::math::cat(contact_wrench_d_lankle.subVector(3,5), contact_wrench_d_rankle.subVector(3,5)));

}

void ftWrenchInXLinkFrame(const yarp::sig::Vector& ft_wrench_d, const RobotUtils& coman_robot,
                          yarp::sig::Vector& wrench_d_lankle, yarp::sig::Vector& wrench_d_rankle)
{
    wrench_d_lankle = -1.0*yarp::math::cat(
                ft_wrench_d.subVector(0,2),ft_wrench_d.subVector(6,8));
    wrench_d_rankle = -1.0*yarp::math::cat(
                ft_wrench_d.subVector(3,5),ft_wrench_d.subVector(9,11));
    KDL::Wrench wrench_d_lankle_KDL;
    cartesian_utils::fromYarpVectortoKDLWrench(wrench_d_lankle, wrench_d_lankle_KDL);
    KDL::Wrench wrench_d_rankle_KDL;
    cartesian_utils::fromYarpVectortoKDLWrench(wrench_d_rankle, wrench_d_rankle_KDL);


    KDL::Frame base_link_T_ft_left = coman_robot.idynutils.iDyn3_model.getPositionKDL(
        coman_robot.idynutils.iDyn3_model.getLinkIndex("r_sole"),
        coman_robot.idynutils.iDyn3_model.getLinkIndex("l_leg_ft"));
    KDL::Frame base_link_T_ft_right = coman_robot.idynutils.iDyn3_model.getPositionKDL(
        coman_robot.idynutils.iDyn3_model.getLinkIndex("l_sole"),
        coman_robot.idynutils.iDyn3_model.getLinkIndex("r_leg_ft"));
    wrench_d_lankle_KDL = base_link_T_ft_left.M*wrench_d_lankle_KDL;
    wrench_d_rankle_KDL = base_link_T_ft_right.M*wrench_d_rankle_KDL;
    cartesian_utils::fromKDLWrenchtoYarpVector(wrench_d_lankle_KDL, wrench_d_lankle);
    cartesian_utils::fromKDLWrenchtoYarpVector(wrench_d_rankle_KDL, wrench_d_rankle);
}

using namespace OpenSoT;

TEST_F(testForceCoP, testForceCoP) {
    startEverything();

    //To control the robot we need RobotUtils
    RobotUtils coman_robot("testConstraint",
                     "coman",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");

    yarp::sig::Vector q = getGoodInitialPosition(coman_robot.idynutils);

    //Homing
    std::vector<iDynUtils::ft_measure> _ft_measurements;
    RobotUtils::ftPtrMap ft_sensors = coman_robot.getftSensors();
    for(RobotUtils::ftPtrMap::iterator it = ft_sensors.begin();
        it != ft_sensors.end(); it++)
    {
        iDynUtils::ft_measure ft_measurement;
        ft_measurement.first = it->second->getReferenceFrame();
        yarp::sig::Vector dummy_measure(6 ,0.0);
        ft_measurement.second = dummy_measure;

        _ft_measurements.push_back(ft_measurement);
    }

    coman_robot.idynutils.updateiDyn3Model(q, _ft_measurements, true);
    coman_robot.idynutils.setFloatingBaseLink(coman_robot.idynutils.left_leg.end_effector_name);
    coman_robot.setPositionMode();
    double speed = 0.8;
    yarp::sig::Vector legs_speed(6,speed);
    legs_speed[3] = 2.0*legs_speed[3];
    coman_robot.left_leg.setReferenceSpeeds(legs_speed);
    coman_robot.right_leg.setReferenceSpeeds(legs_speed);
    coman_robot.left_arm.setReferenceSpeed(speed);
    coman_robot.right_arm.setReferenceSpeed(speed);
    coman_robot.torso.setReferenceSpeed(speed);
    coman_robot.move(q);
    sleep(5);
    coman_robot.setPositionDirectMode();
    sleep(2);
    q = coman_robot.sensePosition();

    getchar();

    std::list<std::string> links_in_contact = coman_robot.idynutils.getLinksInContact();
    links_in_contact.push_back("l_sole");
    links_in_contact.push_back("r_sole");
    coman_robot.idynutils.setLinksInContact(links_in_contact);

    // LOG //
    logger ft_wrench_d_log("ft_wrench_d");
    logger error_wrench_left_ft_log("error_wrench_left_ft");
    logger error_wrench_right_ft_log("error_wrench_right_ft");

    //SET UP FORCE OPTIMIZATION
    //1) Contact Force Optimization
        yarp::sig::Vector contact_wrench_d(12, 0.0);
        OpenSoT::tasks::force::CoM::Ptr com_task(
                    new OpenSoT::tasks::force::CoM(contact_wrench_d, coman_robot.idynutils));
        com_task->setLambda(50.0, 0.0);
        yarp::sig::Matrix W(3,3); W.zero(); W(2,2) = 1.0;
        com_task->setWeight(W);

        OpenSoT::solvers::QPOases_sot::Stack stack_of_tasks_force;
        stack_of_tasks_force.push_back(com_task);

        OpenSoT::solvers::QPOases_sot::Ptr solver_contacts_wrench(
                    new OpenSoT::solvers::QPOases_sot(stack_of_tasks_force,2E10));
        solver_contacts_wrench->solve(contact_wrench_d);

        contactWrenchInFTFrame(coman_robot, contact_wrench_d);

    //2) From contact_wrench to ft_wrench
        yarp::sig::Vector ft_wrench_d(12,0.0);
        OpenSoT::tasks::force::CoP::Ptr cop_task(new OpenSoT::tasks::force::CoP(ft_wrench_d, coman_robot.idynutils));
        yarp::sig::Vector CoP_d(4,0.0);
        CoP_d(0) = 0.05;
        CoP_d(2) = 0.05;
        cop_task->setReference(CoP_d);
        cop_task->update(ft_wrench_d);

        OpenSoT::tasks::force::DesiredWrench::Ptr wrench_task(new OpenSoT::tasks::force::DesiredWrench(ft_wrench_d, coman_robot.idynutils));
        wrench_task->setReference(contact_wrench_d);
        wrench_task->update(ft_wrench_d);

        stack_of_tasks_force.clear();
        stack_of_tasks_force.push_back(cop_task);
        stack_of_tasks_force.push_back(wrench_task);

        OpenSoT::solvers::QPOases_sot::Ptr solver_ft_wrench(
                    new OpenSoT::solvers::QPOases_sot(stack_of_tasks_force,2E10));
        solver_ft_wrench->solve(ft_wrench_d);

        yarp::sig::Vector wrench_d_lankle(6, 0.0);
        yarp::sig::Vector wrench_d_rankle(6, 0.0);

        ftWrenchInXLinkFrame(ft_wrench_d, coman_robot, wrench_d_lankle, wrench_d_rankle);

        ft_wrench_d_log.log(ft_wrench_d);

    // BOUNDS
    Constraint<Matrix, Vector>::ConstraintPtr boundsJointLimits =
            constraints::velocity::JointLimits::ConstraintPtr(
                new constraints::velocity::JointLimits(
                    q,
                    coman_robot.idynutils.iDyn3_model.getJointBoundMax(),
                    coman_robot.idynutils.iDyn3_model.getJointBoundMin()));

    double dT = 0.001;
    Constraint<Matrix, Vector>::ConstraintPtr boundsJointVelocity =
            constraints::velocity::VelocityLimits::ConstraintPtr(
                new constraints::velocity::VelocityLimits(M_PI_2, dT,q.size()));

    constraints::Aggregated::Ptr bounds = OpenSoT::constraints::Aggregated::Ptr(
                new constraints::Aggregated(boundsJointLimits, boundsJointVelocity,
                                            q.size()));

    RobotUtils::ftReadings ft_readings = coman_robot.senseftSensors();
    std::vector<yarp::sig::Vector> filter_ft;
    for(unsigned int i = 0; i < _ft_measurements.size(); ++i){
        _ft_measurements[i].second = -1.0*ft_readings[_ft_measurements[i].first];
        filter_ft.push_back(-1.0*ft_readings[_ft_measurements[i].first]);}
    coman_robot.idynutils.updateiDyn3Model(q, _ft_measurements, false);

    yarp::sig::Matrix C(6,6); C = C.eye();
    for(unsigned int i = 0; i < 3; ++i){
        C(i,i) *= 1E-6;
        C(i+3,i+3) *=1E-5;
    }


    tasks::velocity::Interaction::Ptr interaction_lankle_task(
                new tasks::velocity::Interaction("interaction::l_foot",
                                q, coman_robot.idynutils, "l_leg_ft", "r_sole", "l_leg_ft"));
    std::vector<bool> active_joints = interaction_lankle_task->getActiveJointsMask();
    for(unsigned int i = 0; i < coman_robot.idynutils.right_leg.joint_numbers.size(); ++i)
        active_joints[coman_robot.idynutils.right_leg.joint_numbers[i]] = false;
    interaction_lankle_task->setActiveJointsMask(active_joints);
    interaction_lankle_task->setCompliance(C);
    interaction_lankle_task->setReferenceWrench(wrench_d_lankle);
    interaction_lankle_task->update(q);
    interaction_lankle_task->setOrientationErrorGain(10.0);
    W.resize(6,6);
    W.eye(); W(0,0) = 0.0; W(1,1) = 0.0; W(2,2) = 0.0;
    interaction_lankle_task->setWeight(W);

    tasks::velocity::Interaction::Ptr interaction_rankle_task(
                new tasks::velocity::Interaction("interaction::r_foot",
                                q, coman_robot.idynutils, "r_leg_ft", "l_sole", "r_leg_ft"));
    active_joints = interaction_rankle_task->getActiveJointsMask();
    for(unsigned int i = 0; i < coman_robot.idynutils.left_leg.joint_numbers.size(); ++i)
        active_joints[coman_robot.idynutils.left_leg.joint_numbers[i]] = false;
    interaction_rankle_task->setActiveJointsMask(active_joints);
    interaction_rankle_task->setCompliance(C);
    interaction_rankle_task->setReferenceWrench(wrench_d_rankle);
    interaction_rankle_task->update(q);
    interaction_rankle_task->setOrientationErrorGain(10.0);
    interaction_rankle_task->setWeight(W);


    std::list<tasks::velocity::Cartesian::TaskPtr> aggregated_list;
    aggregated_list.push_back(interaction_lankle_task);
    aggregated_list.push_back(interaction_rankle_task);
    Task<Matrix, Vector>::TaskPtr taskAggregatedHighest =
            tasks::Aggregated::TaskPtr(
       new tasks::Aggregated(aggregated_list,q.size()));

    tasks::velocity::Postural::Ptr postural_task=
            tasks::velocity::Postural::Ptr(new tasks::velocity::Postural(q));

    solvers::QPOases_sot::Stack stack_of_tasks;
    stack_of_tasks.push_back(taskAggregatedHighest);
    stack_of_tasks.push_back(postural_task);

    Solver<yarp::sig::Matrix, yarp::sig::Vector>::SolverPtr sot;
    sot = solvers::QPOases_sot::Ptr(new solvers::QPOases_sot(stack_of_tasks, bounds, 1E10));

    yarp::sig::Vector dq(q.size(), 0.0);
    if(sot->solve(dq)){
        q += dq;}
    coman_robot.move(q);

    int steps = 10000;

    for(unsigned int i = 0; i < steps; ++i)
    {
        double tic = yarp::os::Time::now();
        ft_readings = coman_robot.senseftSensors();
        for(unsigned int i = 0; i < _ft_measurements.size(); ++i){
            filter_ft[i] += (-1.0*ft_readings[_ft_measurements[i].first]-filter_ft[i])*0.7;
            _ft_measurements[i].second = filter_ft[i];}
        coman_robot.idynutils.updateiDyn3Model(q, dq/dT, _ft_measurements, true);


        com_task->update(yarp::sig::Vector(12,0.0));


        yarp::sig::Vector wrench(12, 0.0);
        if(solver_contacts_wrench->solve(wrench))
            contact_wrench_d = wrench;
        else
            std::cout<<"ERROR CONTACTS FORCE Optimization"<<std::endl;
        contactWrenchInFTFrame(coman_robot, contact_wrench_d);

        cop_task->update(yarp::sig::Vector(12,0.0));
        wrench_task->setReference(contact_wrench_d);
        wrench_task->update(yarp::sig::Vector(12,0.0));

        if(solver_ft_wrench->solve(wrench))
            ft_wrench_d = wrench;
        else
            std::cout<<"ERROR FT FORCE Optimization"<<std::endl;

        ftWrenchInXLinkFrame(ft_wrench_d, coman_robot, wrench_d_lankle, wrench_d_rankle);

        ft_wrench_d_log.log(ft_wrench_d);

        interaction_lankle_task->setReferenceWrench(wrench_d_lankle);
        interaction_rankle_task->setReferenceWrench(wrench_d_rankle);

        bounds->update(q);
        taskAggregatedHighest->update(q);
        postural_task->update(q);


        error_wrench_left_ft_log.log(interaction_lankle_task->getWrenchError());
        error_wrench_right_ft_log.log(interaction_rankle_task->getWrenchError());

        if(sot->solve(dq))
            q += dq;
        else
            std::cout<<"ERROR IK Optimization"<<std::endl;
        coman_robot.move(q);


        double toc = yarp::os::Time::now();

        if((toc-tic) < dT)
            yarp::os::Time::delay(dT - (toc-tic));
    }
    ft_wrench_d_log.write();
    error_wrench_left_ft_log.write();
    error_wrench_right_ft_log.write();



    closeEverything();
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

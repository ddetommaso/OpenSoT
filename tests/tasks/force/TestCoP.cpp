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
      tests_utils::startYarpServer();
      // Load a world
      std::string world_path = std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.world";
      if(OPENSOT_SIMULATION_TESTS_VISUALIZATION)
          tests_utils::startGazebo(world_path);
      else
          tests_utils::startGZServer(world_path);
      sleep(4);
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

    //SET UP FORCE OPTIMIZATION
    logger wrench_d_log("wrench_d");
    yarp::sig::Vector wrench_d(12,0.0);
    OpenSoT::tasks::force::CoP::Ptr cop_task(new OpenSoT::tasks::force::CoP(wrench_d, coman_robot.idynutils));
    yarp::sig::Vector CoP_d(4,0.0);
    CoP_d(0) = -0.05;
    CoP_d(2) = -0.05;
    cop_task->setReference(CoP_d);
    std::cout<<"CoP reference: [ "<<cop_task->getReference().toString()<<" ]"<<std::endl;
    cop_task->update(wrench_d);

    OpenSoT::tasks::force::CoM::Ptr com_task(
                new OpenSoT::tasks::force::CoM(wrench_d, coman_robot.idynutils));
    com_task->setLambda(50.0, 0.0);
    yarp::sig::Matrix W(3,3); W.zero(); W(2,2) = 1.0;
    com_task->setWeight(W);

    std::list<tasks::velocity::Cartesian::TaskPtr> aggregated_list_force;
    aggregated_list_force.push_back(cop_task);
    aggregated_list_force.push_back(com_task);
    Task<Matrix, Vector>::TaskPtr taskAggregatedForce =
            tasks::Aggregated::TaskPtr(
       new tasks::Aggregated(aggregated_list_force,wrench_d.size()));

    std::cout<<"STARTING FORCE SOT"<<std::endl;
    OpenSoT::solvers::QPOases_sot::Stack stack_of_tasks_force;
    stack_of_tasks_force.push_back(taskAggregatedForce);

    OpenSoT::solvers::QPOases_sot::Ptr sot_force(
                new OpenSoT::solvers::QPOases_sot(stack_of_tasks_force,1E10));
    std::cout<<"FORCE SOT INITIALIZED"<<std::endl;
    sot_force->solve(wrench_d);
    wrench_d_log.log(wrench_d);
    std::cout<<"FORCE SOT SOLVED"<<std::endl;

    //This is the wrench in sensor frame expressed in world, we have to tranform it!
    yarp::sig::Vector wrench_d_lankle = -1.0*yarp::math::cat(
                wrench_d.subVector(0,2),wrench_d.subVector(6,8));
    yarp::sig::Vector wrench_d_rankle = -1.0*yarp::math::cat(
                wrench_d.subVector(3,5),wrench_d.subVector(9,11));
    KDL::Wrench wrench_d_lankle_KDL;
    cartesian_utils::fromYarpVectortoKDLWrench(wrench_d_lankle, wrench_d_lankle_KDL);
    KDL::Wrench wrench_d_rankle_KDL;
    cartesian_utils::fromYarpVectortoKDLWrench(wrench_d_rankle, wrench_d_rankle_KDL);


    KDL::Frame base_link_T_world = coman_robot.idynutils.iDyn3_model.getPositionKDL(
        coman_robot.idynutils.iDyn3_model.getLinkIndex("Waist"),
        true);
    wrench_d_lankle_KDL = base_link_T_world.M*wrench_d_lankle_KDL;
    wrench_d_rankle_KDL = base_link_T_world.M*wrench_d_rankle_KDL;
    cartesian_utils::fromKDLWrenchtoYarpVector(wrench_d_lankle_KDL, wrench_d_lankle);
    cartesian_utils::fromKDLWrenchtoYarpVector(wrench_d_rankle_KDL, wrench_d_rankle);

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
                new constraints::velocity::VelocityLimits(M_PI/2.0, dT,q.size()));

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
    for(unsigned int i = 0; i < 3; ++i)
    {
        C(i,i) *= 1E-6;
        C(i+3,i+3) *= 1E-6;
    }

    tasks::velocity::Interaction::Ptr interaction_lankle_task(
                new tasks::velocity::Interaction("interaction::l_sole",
                                q, coman_robot.idynutils, "l_sole", "Waist", "l_leg_ft"));
    interaction_lankle_task->setCompliance(C);
    interaction_lankle_task->setReferenceWrench(wrench_d_lankle);
    interaction_lankle_task->update(q);

    tasks::velocity::Interaction::Ptr interaction_rankle_task(
                new tasks::velocity::Interaction("interaction::r_sole",
                                q, coman_robot.idynutils, "r_sole", "Waist", "r_leg_ft"));
    interaction_rankle_task->setCompliance(C);
    interaction_rankle_task->setReferenceWrench(wrench_d_rankle);
    interaction_rankle_task->update(q);

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


        yarp::sig::Vector footZMP1 = cartesian_utils::computeFootZMP(filter_ft[0].subVector(0,2),
                                        filter_ft[0].subVector(3,5),
                                        0.025, 10);
        std::cout<<"footZMP1: ["<<footZMP1.toString()<<"]"<<std::endl;



        cop_task->update(wrench_d);
        com_task->update(wrench_d);

        yarp::sig::Vector wrench(wrench_d.size(), 0.0);
        if(sot_force->solve(wrench))
            wrench_d = wrench;
        else
            std::cout<<"ERROR FORCE Optimization"<<std::endl;
        wrench_d_log.log(wrench_d);


        wrench_d_lankle = -1.0*yarp::math::cat(
                    wrench_d.subVector(0,2),wrench_d.subVector(6,8));
        wrench_d_rankle = -1.0*yarp::math::cat(
                    wrench_d.subVector(3,5),wrench_d.subVector(9,11));

        cartesian_utils::fromYarpVectortoKDLWrench(wrench_d_lankle, wrench_d_lankle_KDL);
        cartesian_utils::fromYarpVectortoKDLWrench(wrench_d_rankle, wrench_d_rankle_KDL);

        base_link_T_world = coman_robot.idynutils.iDyn3_model.getPositionKDL(
            coman_robot.idynutils.iDyn3_model.getLinkIndex("Waist"),
            true);
        wrench_d_lankle_KDL = base_link_T_world.M*wrench_d_lankle_KDL;
        wrench_d_rankle_KDL = base_link_T_world.M*wrench_d_rankle_KDL;
        cartesian_utils::fromKDLWrenchtoYarpVector(wrench_d_lankle_KDL, wrench_d_lankle);
        cartesian_utils::fromKDLWrenchtoYarpVector(wrench_d_rankle_KDL, wrench_d_rankle);


        interaction_lankle_task->setReferenceWrench(wrench_d_lankle);
        interaction_rankle_task->setReferenceWrench(wrench_d_rankle);

        bounds->update(q);
        taskAggregatedHighest->update(q);
        postural_task->update(q);


        if(sot->solve(dq)){
            q += dq;}
        coman_robot.move(q);


        double toc = yarp::os::Time::now();

        if((toc-tic) < dT)
            yarp::os::Time::delay(dT - (toc-tic));
    }
    wrench_d_log.write();


    closeEverything();
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

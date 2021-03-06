#include <OpenSoT/utils/DefaultHumanoidStack.h>


using namespace OpenSoT;

DefaultHumanoidStack::DefaultHumanoidStack(iDynUtils& model,
                                           const double dT,
                                           const yarp::sig::Vector& state) :
     leftArm( new tasks::velocity::Cartesian("cartesian::l_wrist",
                                             state,
                                             model,
                                             model.left_arm.end_effector_name,
                                             "world") ),
     leftArm_Position( new SubTask(leftArm, Indices::range(0,2)) ),
     leftArm_Orientation( new SubTask(leftArm, Indices::range(3,5)) ),
     rightArm( new tasks::velocity::Cartesian("cartesian::r_wrist",
                                              state,
                                              model,
                                              model.right_arm.end_effector_name,
                                             "world") ),
     rightArm_Position( new SubTask(rightArm, Indices::range(0,2)) ),
     rightArm_Orientation( new SubTask(rightArm, Indices::range(3,5)) ),
     TCP_L( new tasks::velocity::Cartesian("cartesian::TCP_L",
                                           state,
                                           model,
                                           (model.iDyn3_model.getLinkIndex("TCP_L") > 0 ?
                                                "TCP_L" :
                                                model.left_arm.end_effector_name),
                                           "world") ),
     TCP_R( new tasks::velocity::Cartesian("cartesian::TCP_R",
                                           state,
                                           model,
                                           (model.iDyn3_model.getLinkIndex("TCP_R") > 0 ?
                                                "TCP_R" :
                                                model.right_arm.end_effector_name),
                                          "world") ),
     waist2LeftArm( new tasks::velocity::Cartesian("cartesian::l_wrist",
                                                   state,
                                                   model,
                                                   model.left_arm.end_effector_name,
                                                   model.getBaseLink()) ),
     waist2LeftArm_Position( new SubTask(waist2LeftArm, Indices::range(0,2)) ),
     waist2LeftArm_Orientation( new SubTask(waist2LeftArm, Indices::range(3,5)) ),
     waist2RightArm( new tasks::velocity::Cartesian("cartesian::r_wrist",
                                                    state,
                                                    model,
                                                    model.right_arm.end_effector_name,
                                                    model.getBaseLink()) ),
     waist2RightArm_Position( new SubTask(waist2RightArm, Indices::range(0,2)) ),
     waist2RightArm_Orientation( new SubTask(waist2RightArm, Indices::range(3,5)) ),
     leftLeg( new tasks::velocity::Cartesian("cartesian::l_sole",
                                             state,
                                             model,
                                             model.left_leg.end_effector_name,
                                             "world") ),
     leftLeg_Position( new SubTask(leftLeg,    Indices::range(0,2)) ),
     leftLeg_Orientation( new SubTask(leftLeg, Indices::range(3,5)) ),
     rightLeg( new tasks::velocity::Cartesian("cartesian::r_sole",
                                              state,
                                              model,
                                              model.right_leg.end_effector_name,
                                              "world") ),
     rightLeg_Position( new SubTask(rightLeg,    Indices::range(0,2)) ),
     rightLeg_Orientation( new SubTask(rightLeg, Indices::range(3,5)) ),
     waist( new tasks::velocity::Cartesian("cartesian::waist",
                                            state,
                                            model,
                                            model.getBaseLink(),
                                            "world") ),
     right2LeftLeg( new tasks::velocity::Cartesian("cartesian:r2l_sole",
                                                  state,
                                                  model,
                                                  model.right_leg.end_effector_name,
                                                  model.left_leg.end_effector_name) ),
     waist_Position( new SubTask(waist,    Indices::range(0,2)) ),
     waist_Position_XY( new SubTask(waist, Indices::range(0,1)) ),
     waist_Position_Z( new SubTask(waist,  Indices::range(2,2)) ),
     waist_Orientation( new SubTask(waist, Indices::range(3,5)) ),
     com( new tasks::velocity::CoM(state,
                                   model) ),
     com_XY( new SubTask(com, Indices::range(0,1)) ),
     com_Z( new SubTask(com,  Indices::range(2,2)) ),
     gaze( new tasks::velocity::Gaze("cartesian::gaze", state, model, "world") ),
     waist2gaze( new tasks::velocity::Gaze("cartesian::w2gaze", state, model, "Waist") ),
     minimumEffort( new tasks::velocity::MinimumEffort(state,
                                                       model) ),
     minimumVelocity( new tasks::velocity::MinimumVelocity(state.size()) ),
     minimumAcceleration( new tasks::velocity::MinimizeAcceleration(state) ),
     postural( new tasks::velocity::Postural(state) ),
     posturalForTorso( new tasks::velocity::Postural(state) ),
     postural_Torso( new SubTask(posturalForTorso,
                                 Indices(model.torso.joint_numbers)) ),
     posturalForArms( new tasks::velocity::Postural(state) ),
     postural_Arms( new SubTask(posturalForArms,
                                (Indices(model.left_arm.joint_numbers) +
                                 Indices(model.right_arm.joint_numbers)) ) ),
     posturalForLegs( new tasks::velocity::Postural(state) ),
     postural_Legs( new SubTask(posturalForLegs,
                                (Indices(model.left_leg.joint_numbers) +
                                 Indices(model.right_leg.joint_numbers)) ) ),
     posturalForLimbs( new tasks::velocity::Postural(state) ),
     postural_Limbs( new SubTask(posturalForLimbs,
                                 (Indices(model.left_arm.joint_numbers) +
                                  Indices(model.right_arm.joint_numbers) +
                                  Indices(model.left_leg.joint_numbers) +
                                  Indices(model.right_leg.joint_numbers)) ) ),
     posturalForLimbsAndHead( new tasks::velocity::Postural(state) ),
     postural_LimbsAndHead( new SubTask(posturalForLimbsAndHead,
                                 (Indices(model.left_arm.joint_numbers) +
                                  Indices(model.right_arm.joint_numbers) +
                                  Indices(model.left_leg.joint_numbers) +
                                  Indices(model.right_leg.joint_numbers) +
                                  Indices(model.head.joint_numbers)) ) ),
     comVelocity( new constraints::velocity::CoMVelocity(yarp::sig::Vector(3,.3),
                                                         dT,
                                                         state,
                                                         model) ),
     convexHull( new constraints::velocity::ConvexHull(state,
                                                       model) ),
     jointLimits( new constraints::velocity::JointLimits(state,
                                                         model.iDyn3_model.getJointBoundMax(),
                                                         model.iDyn3_model.getJointBoundMin()) ),
     selfCollisionAvoidance( new constraints::velocity::SelfCollisionAvoidance( state,
                                                                                model,
                                                                                std::numeric_limits<double>::infinity(),
                                                                                0.01)),
     torqueLimits( new constraints::velocity::Dynamics(state, state*0.0,
                         model.iDyn3_model.getJointTorqueMax(),
                         model, dT, 0.65) ),
     velocityLimits( new constraints::velocity::VelocityLimits(0.3,
                                                               dT,
                                                               state.size()) )


 {

 }



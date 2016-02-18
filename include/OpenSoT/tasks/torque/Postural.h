#ifndef __TASKS_TORQUE_POSTURAL_H__
#define __TASKS_TORQUE_POSTURAL_H__

 #include <OpenSoT/Task.h>
 #include <idynutils/idynutils.h>
 #include <kdl/frames.hpp>
 #include <yarp/sig/all.h>
 #include <yarp/os/all.h>
 #include <OpenSoT/tasks/velocity/Postural.h>

/**
 * @example example_postural.cpp
 * The Postural class implements a task that tries to bring the robust posture to a reference posture.
 */

 namespace OpenSoT {
    namespace tasks {
        namespace torque {
            /**
             * @brief The Postural class implements a task that tries to bring the robust posture to a reference posture.
             * You can see an example of it in @ref example_postural.cpp
             */
            class Postural : public OpenSoT::tasks::velocity::Postural{
            public:
                typedef boost::shared_ptr<Postural> Ptr;

                Postural(const yarp::sig::Vector& x, const double& lambda):
                    OpenSoT::tasks::velocity::Postural::Postural(x){
                    OpenSoT::tasks::velocity::Postural::setLambda(lambda);}

                void setReference(const yarp::sig::Vector& x_desired){
                    OpenSoT::tasks::velocity::Postural::setReference(x_desired);}

                yarp::sig::Vector getReference() const{
                    return OpenSoT::tasks::velocity::Postural::getReference();}


                void setLambda(double lambda){
                    OpenSoT::tasks::velocity::Postural::setLambda(lambda);}

                yarp::sig::Vector getActualPositions(){
                    return OpenSoT::tasks::velocity::Postural::getActualPositions();}

                yarp::sig::Vector getError(){
                    return OpenSoT::tasks::velocity::Postural::getError();}

            };
        }
    }
 }

#endif

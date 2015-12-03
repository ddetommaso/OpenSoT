#ifndef FRICTIONCONE_H
#define FRICTIONCONE_H


 #include <OpenSoT/Constraint.h>
 #include <yarp/sig/all.h>
 #include <idynutils/idynutils.h>
 #include <idynutils/collision_utils.h>
 #include <kdl/frames.hpp>

#include <Eigen/Dense>


 namespace OpenSoT {
    namespace constraints {
        namespace force {


            class FrictionCone: public Constraint<yarp::sig::Matrix, yarp::sig::Vector> {
            public:
                typedef boost::shared_ptr<FrictionCone> Ptr;
            protected:


        std::map<std::string, double> _mu; //Friction Coefficient associated to each contact surface
        iDynUtils& _robot;
        std::vector<std::string> _ft_in_contact;

            public:

        /**
                 * @brief FrictionCone
                 * @param x
                 * @param robot
                 * @param mu
                 *
                 * Note that we want to express the constraint locally
                 */
                FrictionCone(const yarp::sig::Vector& x,
                                       iDynUtils &robot,
                                       const std::map<std::string, double>& mu);


                void update(const yarp::sig::Vector &x);

                void setMu(const std::map<std::string, double> mu){ _mu = mu;}

            private:
        void computeAineq();
        void computeUpperBound();



            };
        }
    }
 }


#endif // FRICTIONCONE_H

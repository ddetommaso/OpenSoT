#include <OpenSoT/tasks/force/CoP.h>
#include <yarp/math/Math.h>
#include <idynutils/cartesian_utils.h>
#include <exception>
#include <cmath>
#include <iCub/iDynTree/yarp_kdl.h>
#include <OpenSoT/constraints/velocity/Dynamics.h>

using namespace OpenSoT::tasks::force;
using namespace yarp::math;

CoP::CoP(const yarp::sig::Vector &x, iDynUtils &robot):
    Task("CoP", x.size()), _robot(robot)
{
    cartesian_utils::computeRealLinksFromFakeLinks(_robot.getLinksInContact(), _robot.urdf_model, _CoP_frames);

    _desired_CoP.resize(2*_CoP_frames.size(), 0.0);

    _update(x);

    _lambda = 1.0;

    _hessianType = HST_SEMIDEF;

}

CoP::~CoP()
{

}

void CoP::_update(const yarp::sig::Vector &x)
{
    cartesian_utils::computeRealLinksFromFakeLinks(_robot.getLinksInContact(), _robot.urdf_model, _CoP_frames);
    _b.resize(_desired_CoP.size(), 0.0);

    computeD();

    _A.resize(2*_CoP_frames.size(), 6*_CoP_frames.size());
    _A.zero();

    //Check this...W change with the number of contacts
    _W.resize(2*_CoP_frames.size(), 2*_CoP_frames.size());
    _W.eye();


    for(unsigned int i = 0; i < _CoP_frames.size(); ++i)
    {
        yarp::sig::Matrix A(2,6); A.zero();
        A(0,0) = _d(i); A(0,2) = _desired_CoP(2*i);   A(0,4) = 1.0;
        A(1,1) = _d(i); A(1,2) = _desired_CoP(2*i+1); A(1,3) = -1.0;


        yarp::sig::Matrix Adj(6,6); Adj.eye();
        yarp::sig::Matrix cross(3,3); cross.zero();
        cross(0,1) = -(-_d(i)); cross(0,2) = _desired_CoP(2*i+1);
        cross(1,0) = -_d(i); cross(1,2) = -_desired_CoP(2*i);
        cross(2,0) = -_desired_CoP(2*i+1); cross(2,1) = _desired_CoP(2*i);
        Adj.setSubmatrix(-1.0*cross, 3, 0);

        A = A*Adj;

        _A.setSubmatrix(A.submatrix(0, 1, 0, 2), 2*i, 3*i);
        _A.setSubmatrix(A.submatrix(0, 1, 3, 5), 2*i, _CoP_frames.size()*3+3*i);
    }

}

void CoP::computeD()
{
    std::list<std::string>::const_iterator iter;
    for(iter = _CoP_frames.begin(); iter != _CoP_frames.end(); iter++)
    {
        bool is_child = false;
        std::vector< boost::shared_ptr<urdf::Link> > child_links = _robot.urdf_model->getLink(*iter)->child_links;
        std::vector<std::string> child_links_names;
        for(unsigned int i = 0; i < child_links.size(); ++i)
            child_links_names.push_back(child_links.at(i)->name);


        std::list<std::string>::const_iterator iter2 = _robot.getLinksInContact().begin();
        while(!is_child)
        {
            if(std::find(child_links_names.begin(), child_links_names.end(), *iter2) != child_links_names.end()){

                yarp::sig::Matrix T = _robot.iDyn3_model.getPosition(
                            _robot.iDyn3_model.getLinkIndex(*iter),
                            _robot.iDyn3_model.getLinkIndex(*iter2));
                _d.push_back(fabs(T(2,3)));
                is_child = true;
            }
            else
                iter2++;

        }
    }
}

void CoP::setReference(const yarp::sig::Vector& desiredCoP)
{
    assert(desiredCoP.size() == 2*_CoP_frames.size());

    _desired_CoP = desiredCoP;
}

yarp::sig::Vector CoP::getReference() const
{
    return _desired_CoP;
}

std::list<std::string> CoP::getCoPFrames()
{
    return _CoP_frames;
}


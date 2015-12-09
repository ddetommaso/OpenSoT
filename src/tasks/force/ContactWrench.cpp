/*
 * Copyright (C) 2014 Walkman
 * Author: Enrico Mingo Hoffman, Alessio Rocchi
 * email:  enrico.mingo@iit.it, alessio.rocchi@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <OpenSoT/constraints/velocity/Dynamics.h>
#include <OpenSoT/tasks/force/ContactWrench.h>
#include <yarp/math/Math.h>
#include <idynutils/cartesian_utils.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::tasks::force;
using namespace yarp::math;

ContactWrench::ContactWrench(const yarp::sig::Vector& x, iDynUtils &robot) :
    Task("ContactWrench", x.size()),
    _w_desired(x.size(),0.0),
    _robot(robot)
{
    _W.resize(_x_size, _x_size);
    _W.eye();

    _A.resize(_x_size, _x_size);
    _A.eye();

    _hessianType = HST_IDENTITY;

    _lambda = 1.0;

    /* first update. Setting desired contatc wrench equal to 0 */
    this->_update(x);
}

ContactWrench::~ContactWrench()
{
}

void ContactWrench::_update(const yarp::sig::Vector &x) {
    /**
      * Now I have to compute the Jacobian of the task that in this case is the matrix I
    **/
    // First I have to know which are the contacts
    std::vector<std::string> ft_in_contact;
    OpenSoT::constraints::velocity::Dynamics::crawlLinks(_robot.getForceTorqueFrameNames(),
                                        _robot.getLinksInContact(),
                                        _robot,
                                        ft_in_contact);
    _A.resize(ft_in_contact.size()*6, ft_in_contact.size()*6);
    _A.eye();


    update_b();
}

void ContactWrench::setReference(const yarp::sig::Vector& x_desired) {
    assert(x_desired.size() == _A.rows());

    _w_desired = x_desired;
    update_b();
}

yarp::sig::Vector ContactWrench::getReference() const
{
    return _w_desired;
}

void ContactWrench::update_b() {
    _b = _w_desired;
}

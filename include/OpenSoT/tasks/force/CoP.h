/*
 * Copyright (C) 2015 Walkman
 * Authors: Enrico Mingo Hoffman, Alessio Rocchi
 * email: enrico.mingo@iit.it, alessio.rocchi@iit.it
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

#ifndef __TASKS_FORCE_COP_H__
#define __TASKS_FORCE_COP_H__

#include <OpenSoT/Task.h>
#include <idynutils/idynutils.h>
#include <kdl/frames.hpp>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>

namespace OpenSoT {
   namespace tasks {
       namespace force {
       class CoP : public Task < yarp::sig::Matrix, yarp::sig::Vector > {
       public:
           typedef boost::shared_ptr<CoP> Ptr;
       private:
           iDynUtils& _robot;

           /**
            * @brief _desired_CoP is a vector of size 2*#_of_contacts, we consider just x and y since the CoP is defined
            * at the contact surface
            */
           yarp::sig::Vector _desired_CoP;

           /**
            * @brief _d is a vector of z-distances between a frame in the link in contact and the contact surface
            */
           yarp::sig::Vector _d;

           /**
            * @brief _CoP_frames is a vector of bodies that are in contact with the environment and we want to control
            * the CoP
            */
           std::list<std::string> _CoP_frames;

           void computeD();

           public:
           CoP(const yarp::sig::Vector& x,
               iDynUtils& robot);

           ~CoP();

           void _update(const yarp::sig::Vector& x);

           /**
            * @brief setReference set CoPs desired
            * @param desiredCoP vector of 2*#_of_contacts
            * NOTE: CoPs are defined in the reference frames in _bodies_in_contact at the body surface
            */
           void setReference(const yarp::sig::Vector& desiredCoP);

           yarp::sig::Vector getReference() const;

           std::list<std::string> getCoPFrames();
       };

       }
   }
}
#endif

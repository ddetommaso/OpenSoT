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

#ifndef __TASKS_FORCE_CONTACT_WRENCH_H__
#define __TASKS_FORCE_CONTACT_WRENCH_H__

 #include <OpenSoT/Task.h>
 #include <idynutils/idynutils.h>
 #include <yarp/sig/all.h>


 namespace OpenSoT {
    namespace tasks {
        namespace force {

            class ContactWrench : public Task < yarp::sig::Matrix, yarp::sig::Vector > {
            public:
                typedef boost::shared_ptr<ContactWrench> Ptr;
            protected:
                /**
                 * @brief _w_desired are the desired wrenches at the contacts.
                 * NOTE: w = [f1 f2 ... fn tau1 tau2 ... taun]
                 * NOTE: Setting w_desired to 0 the task became the minimization of contact forces
                 */
                yarp::sig::Vector _w_desired;
                iDynUtils& _robot;

                void update_b();

            public:

                ContactWrench(const yarp::sig::Vector& x, iDynUtils& robot);

                ~ContactWrench();

                void _update(const yarp::sig::Vector& x);

                void setReference(const yarp::sig::Vector& x_desired);

                yarp::sig::Vector getReference() const;

            };
        }
    }
 }

#endif

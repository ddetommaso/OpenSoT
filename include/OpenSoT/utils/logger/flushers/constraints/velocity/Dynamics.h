/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi, Enrico Mingo Hoffman
 * email:  alessio.rocchi@iit.it, enrico.mingo@iit.it
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


#ifndef __CONSTRAINT_FLUSHER_DYNAMICS_H__
#define __CONSTRAINT_FLUSHER_DYNAMICS_H__

#include <OpenSoT/constraints/velocity/Dynamics.h>
#include <OpenSoT/utils/logger/flushers/ConstraintFlusher.h>
#include <idynutils/idynutils.h>

namespace OpenSoT
{
    namespace flushers
    {
        namespace constraints
        {
            namespace velocity
            {
                class Dynamics: public ConstraintFlusher
                {
                public:
                    enum labels
                    {
                        TORQUE_LIMITS   = 1,
                        ESTIMATED_TORQUE= 2,
                        BUPPERBOUND     = 4,
                        BLOWERBOUND     = 8,
                        SIGMA           = 16
                    };

                    Dynamics(OpenSoT::constraints::velocity::Dynamics::Ptr dynamics, const iDynUtils& model);

                    std::string toString() const;

                    OpenSoT::Indices getIndices(int label) const;

                    int getSize() const;

                    typedef boost::shared_ptr<Dynamics> Ptr;
                };
            }
        }
    }
}

#endif

/*
 * Copyright (C) 2016 Walkman
 * Authors:Enrico Mingo Hoffman, Alessio Rocchi
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

#ifndef __TASKS_VELOCITY_GAZE_H__
#define __TASKS_VELOCITY_GAZE_H__

#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/SubTask.h>
#include <idynutils/cartesian_utils.h>

namespace OpenSoT {
namespace tasks {
namespace velocity {

class Gaze: public OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>
{
public:
    typedef boost::shared_ptr<Gaze> Ptr;

    Gaze(std::string task_id,
         const yarp::sig::Vector &x,
         iDynUtils &robot,
         std::string base_link);

    ~Gaze();

    /**
     * @brief setGaze
     * @param desiredGaze pose of the object to observe in base_link
     */
    void setGaze(const yarp::sig::Matrix& desiredGaze);

    void setOrientationErrorGain(const double& orientationErrorGain);

    const double getOrientationErrorGain() const;

    /**
     * @brief setWeight sets the task weight.
     * Note the Weight needs to be positive definite.
     * If your original intent was to get a subtask
     * (i.e., reduce the number of rows of the task Jacobian),
     * please use the class SubTask
     * @param W matrix weight
     */
    virtual void setWeight(const yarp::sig::Matrix& W);

    /**
     * @brief getConstraints return a reference to the constraint list. Use the standard list methods
     * to add, remove, clear, ... the constraints list.
     * e.g.:
     *              task.getConstraints().push_back(new_constraint)
     * Notice that in subtasks, you will get the constraint list of the father Task from which the SubTask
     * is generated.
     * @return the list of constraints to which the father Task is subject.
     */
    virtual std::list< ConstraintPtr >& getConstraints();

    /** Gets the task size.
        @return the number of rows of A */
    virtual const unsigned int getTaskSize() const;

    /** Updates the A, b, Aeq, beq, Aineq, b*Bound matrices
        @param x variable state at the current step (input) */
    virtual void _update(const yarp::sig::Vector &x);

    /**
     * @brief getActiveJointsMask return a vector of length NumberOfDOFs.
     * If an element is false the corresponding column of the task jacobian is set to 0.
     * @return a vector of bool
     */
    virtual std::vector<bool> getActiveJointsMask();

    /**
     * @brief setActiveJointsMask set a mask on the jacobian
     * @param active_joints_mask
     * @return true if success
     */
    virtual bool setActiveJointsMask(const std::vector<bool>& active_joints_mask);


private:
    std::string _distal_link;
    Cartesian::Ptr _cartesian_task;
    SubTask::Ptr   _subtask;

    iDynUtils& _robot;
};

}

}

}

#endif

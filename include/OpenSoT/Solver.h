/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi
 * email:  alessio.rocchi@iit.it
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

#ifndef __SOLVER_H__
#define __SOLVER_H__

#include <OpenSoT/Task.h>
#include <OpenSoT/Constraint.h>
#include <Eigen/Core>
#include <vector>

 namespace OpenSoT {
    class Solver {
    public:
        typedef Eigen::MatrixXd Matrix_type;
        typedef Eigen::VectorXd Vector_type;
        typedef Task TaskType;
        typedef boost::shared_ptr<TaskType> TaskPtr;
        typedef Constraint ConstraintType;
        typedef boost::shared_ptr<ConstraintType> ConstraintPtr;
        typedef Solver SolverType;
        typedef boost::shared_ptr<SolverType> SolverPtr;
        typedef std::vector <TaskPtr> Stack;

    protected:
        std::vector <TaskPtr> _tasks;
        ConstraintPtr _bounds;
        ConstraintPtr _globalConstraints;


    public:

        /**
         * @brief Solver an interface for a generic solver
         * @param stack a vector of pointers to tasks
         */
        Solver(std::vector <TaskPtr>& stack) : _tasks(stack){}

        /**
         * @brief Solver an interface for a generic solver
         * @param stack a vector of pointers to tasks
         * @param bounds a global bound for the problem
         */
        Solver(std::vector <TaskPtr>& stack,
               ConstraintPtr bounds) : _tasks(stack), _bounds(bounds) {}

        /**
         * @brief Solver an interface for a generic solver
         * @param stack a vector of pointers to tasks
         * @param bounds a global bound for the problem
         * @param globalConstraints a global constrains for the problem
         */
        Solver(std::vector<TaskPtr> &stack, ConstraintPtr bounds, ConstraintPtr globalConstraints):
            _tasks(stack), _bounds(bounds), _globalConstraints(globalConstraints) {}

        virtual ~Solver(){}

        /**
         * @brief solve solve an Optimization problem
         * @param solution the solution
         * @return  true if solved/solvable
         */
        virtual bool solve(Vector_type& solution) = 0;
    };
 }

#endif

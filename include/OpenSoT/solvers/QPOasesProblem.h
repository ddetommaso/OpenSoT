#ifndef _WB_SOT_SOLVERS_QP_OASES_PROBLEM_H_
#define _WB_SOT_SOLVERS_QP_OASES_PROBLEM_H_

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <boost/shared_ptr.hpp>
#include <OpenSoT/Task.h>

using namespace yarp::sig;

#define DEFAULT_EPS_REGULARISATION 2E2

namespace qpOASES {
    class SQProblem;
    class Options;
    class Bounds;
    class Constraints;
    class SymSparseMat;
    class DenseMatrix;
}

namespace OpenSoT{
    namespace solvers{

    /**
     * @brief The QPOasesProblem class handle variables, options and execution of a
     * single qpOases problem. Is implemented using yarp::sig Matrix and yarp::sig::Vector.
     */
    class QPOasesProblem {
    public:
        /**
         * @brief QPOasesProblem constructor with creation of a QP problem.
         * @param number_of_variables of the QP problem
         * @param number_of_constraints of the QP problem
         * @param hessian_type of the QP problem
         * @param eps_regularization set the Scaling factor of identity matrix used for Hessian regularisation.
         *             final_eps_regularisation = standard_eps_regularisation * eps_regularisation
         *        this parameter is particular important for the optimization!
         */
        QPOasesProblem(const int number_of_variables,
                       const int number_of_constraints,
                       OpenSoT::HessianType hessian_type = OpenSoT::HST_UNKNOWN,
                       const double eps_regularisation = DEFAULT_EPS_REGULARISATION); //2E2

        /**
          * @brief ~QPOasesProblem destructor
          */
        ~QPOasesProblem();

        /**
         * @brief setDefaultOptions to internal qpOases problem.
         * Default are set to:
         *  opt.setToMPC();
         *  opt.printLevel = qpOASES::PL_NONE;
         *  opt.enableRegularisation = qpOASES::BT_TRUE;
         *  opt.epsRegularisation *= _epsRegularisation;
         *  opt.numRegularisationSteps = 2;
         *  opt.numRefinementSteps = 1;
         *  opt.enableFlippingBounds = qpOASES::BT_TRUE;
         *
         */
        void setDefaultOptions();

        /**
         * @brief getProblem return the internal QP problem
         * @return reference to internal QP problem
         */
        const boost::shared_ptr<qpOASES::SQProblem>& getProblem(){return _problem;}

        /**
         * @brief getOptions return the options of the QP problem
         * @return options
         */
        qpOASES::Options getOptions();

        /**
         * @brief setOptions of the QP problem.
         * @param options
         */
        void setOptions(const qpOASES::Options& options);

        /**
         * @brief initProblem initialize the QP problem and get the solution, the dual solution,
         * bounds and constraints.
         * The QP problem has the following structure:
         *
         *      min = ||Hx - g||
         *  st.     lA <= Ax <= uA
         *           l <=  x <= u
         * @param H Task Matrix
         * @param g Task references
         * @param A Constraint Matrix
         * @param lA lower constraint yarp::sig::Vector
         * @param uA upper constraint yarp::sig::Vector
         * @param l lower bounds
         * @param u upper bounds
         * @return true if the problem can be solved
         */
        bool initProblem(const yarp::sig::Matrix& H, const yarp::sig::Vector& g,
                        const yarp::sig::Matrix& A,
                        const yarp::sig::Vector& lA, const yarp::sig::Vector& uA,
                        const yarp::sig::Vector& l, const yarp::sig::Vector& u);

        /**
         * This set of function update current problem copying input data. Use these
         * methods to update existing matrices of the QP problem.
         */

        /**
         * @brief updateTask update internal H and g:
         * _H = H
         * _g = g
         * for now is not possible to have different size of H and g wrt internal ones
         * @param H updated task matrix
         * @param g updated reference yarp::sig::Vector
         * @return true if task is correctly updated
         */
        bool updateTask(const yarp::sig::Matrix& H, const yarp::sig::Vector& g);

        /**
         * @brief updateConstraints update internal A, lA and uA
         * _A = A
         * _lA = lA
         * _uA = uA
         * A, lA and uA can change rows size to allow variable constraints
         * @param A update constraint matrix
         * @param lA update lower constraint yarp::sig::Vector
         * @param uA update upper constraint yarp::sig::Vector
         * @return true if constraints are correctly updated
         */
        bool updateConstraints(const yarp::sig::Matrix& A, const yarp::sig::Vector& lA, const yarp::sig::Vector& uA);

        /**
         * @brief updateBounds update internal l and u
         * _l = l
         * _u = u
         * @param l update lower bounds
         * @param u update upper bounds
         * @return true if bounds are correctly updated
         */
        bool updateBounds(const yarp::sig::Vector& l, const yarp::sig::Vector& u);

        /**
         * @brief updateProblem update the whole problem see updateTask(), updateConstraints() and updateBounds()
         * @param H updated task matrix
         * @param g updated reference yarp::sig::Vector
         * @param A update constraint matrix
         * @param lA update lower constraint yarp::sig::Vector
         * @param uA update upper constraint yarp::sig::Vector
         * @param l update lower bounds
         * @param u update upper bounds
         * @return if the problem is correctly updated
         */
        bool updateProblem(const yarp::sig::Matrix& H, const yarp::sig::Vector& g,
                           const yarp::sig::Matrix& A,
                           const yarp::sig::Vector& lA, const yarp::sig::Vector& uA,
                           const yarp::sig::Vector& l, const yarp::sig::Vector& u);

        /**
         * @brief addTask pile a matrix H to internal _H and g to internal _g
         * so that _H = [_H; H] and _g = [_g; g]
         * @param H extra Task Matrix
         * @param g extra reference yarp::sig::Vector
         * @return true if the problem is initiazlized correctly
         */
        bool addTask(const yarp::sig::Matrix& H, const yarp::sig::Vector& g);

        /**
         * @brief addConstraints pile a matrix A to internal _A and lA/uA to internal _lA/_uA
         * so that _A = [_A; A], _lA = [_lA; lA], _uA = [_uA; uA]
         * @param A extra constraint matrix
         * @param lA extra lower constraint yarp::sig::Vector
         * @param uA extra upper constraint yarp::sig::Vector
         * @return true if the problem is initiazlized correctly
         */
        bool addConstraints(const yarp::sig::Matrix& A, const yarp::sig::Vector& lA, const yarp::sig::Vector& uA);

        /**
         * @brief solve the QP problem
         * @return true if the QP problem is solved
         */
        bool solve();

        /**
         * @brief getSolution return the actual solution of the QP problem
         * @return solution
         */
        const yarp::sig::Vector& getSolution(){return _solution;}

        /**
         * @brief getHessianType return the hessian type f the problem
         * @return hessian type
         */
        OpenSoT::HessianType getHessianType();

        /**
         * @brief setHessianType of the problem
         * @param ht hessian type
         */
        void setHessianType(const OpenSoT::HessianType ht);

        /**
         * @brief getnWSR return maximum number of working set recalculations
         * @return maximum number of working set recalculations
         */
        int getnWSR(){return _nWSR;}

        /**
         * @brief setnWSR set maximum number of working set recalculations
         * @param nWSR Maximum number of working set recalculations
         */
        void setnWSR(const int nWSR){_nWSR = nWSR;}

        /**
         * @brief getActiveBounds return the active bounds of the solved QP problem
         * @return active bounds
         */
        const qpOASES::Bounds& getActiveBounds(){return *_bounds;}

        /**
         * @brief getActiveConstraints return the active constraints of the solved QP problem
         * @return active constraints
         */
        const qpOASES::Constraints& getActiveConstraints(){return *_constraints;}

        /**
         * Getters for internal matrices and yarp::sig::Vectors
         */
        const yarp::sig::Matrix& getH(){return _H;}
        const yarp::sig::Vector& getg(){return _g;}
        const yarp::sig::Matrix& getA(){return _A;}
        const yarp::sig::Vector& getlA(){return _lA;}
        const yarp::sig::Vector& getuA(){return _uA;}
        const yarp::sig::Vector& getl(){return _l;}
        const yarp::sig::Vector& getu(){return _u;}

        /**
         * @brief printProblemInformation print some extra information about the problem
         * @param problem_number a number to identify the problem
         * @param problem_id a string to identify the problem
         * @param constraints_id a string to identify the constraints associated to the problem
         * @param bounds_id a string to identify the bounds associated to the problem
         */
        void printProblemInformation(const int problem_number, const std::string& problem_id,
                                     const std::string& constraints_id, const std::string& bounds_id);

        bool writeQPIntoMFile(const std::string& file_name);

    protected:
        /**
         * @brief checkInfeasibility function that print informations when the problem is not feasible
         */
        void checkInfeasibility();

        /**
         * @brief checkINFTY if a bound/constraint is set to a value less than -INFTY then the bound/constraint is
         * set to -INFTY, if a bound/constraint is set to a value more than INFTY then the bound/constraint is
         * set to INFTY.
         */
        void checkINFTY();

        /**
         * @brief _problem is the internal SQProblem
         */
        boost::shared_ptr<qpOASES::SQProblem> _problem;

        /**
         * @brief _bounds are the active bounds of the SQProblem
         */
        boost::shared_ptr<qpOASES::Bounds> _bounds;

        /**
         * @brief _constraints are the active constraints of the SQProblem
         */
        boost::shared_ptr<qpOASES::Constraints> _constraints;

        /**
         * @brief _nWSR is the maximum number of working set recalculations
         */
        int _nWSR;

        /**
         * @brief _epsRegularisation is a factor that multiplies standard epsRegularisation of qpOases
         */
        double _epsRegularisation;

        /**
         * Define a cost function: ||Hx - g||
         */
        yarp::sig::Matrix _H;
        boost::shared_ptr<qpOASES::SymSparseMat> H_sparse;
        yarp::sig::Vector _g;

        /**
         * Define a set of constraints weighted with A: lA <= Ax <= uA
         */
        yarp::sig::Matrix _A;
        boost::shared_ptr<qpOASES::DenseMatrix> A_dense;
        yarp::sig::Vector _lA;
        yarp::sig::Vector _uA;

        /**
         * Define a set of bounds on solution: l <= x <= u
         */
        yarp::sig::Vector _l;
        yarp::sig::Vector _u;

        /**
         * Solution and dual solution of the QP problem
         */
        yarp::sig::Vector _solution;
        yarp::sig::Vector _dual_solution;

        /**
         * @brief _opt solver options
         */
        boost::shared_ptr<qpOASES::Options> _opt;
    };
    }
}
#endif

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    QPSolver.h
 * @brief   A quadratic programming solver implements the active set method
 * @date    Apr 15, 2014
 * @author  Duy-Nguyen Ta
 */

#pragma once

#include <gtsam/linear/VectorValues.h>
#include <gtsam_unstable/linear/QP.h>

#include <vector>
#include <set>

namespace gtsam {

/// This struct holds the state of QPSolver at each iteration
struct QPState {
  VectorValues values;
  VectorValues duals;
  InequalityFactorGraph workingSet;
  bool converged;
  size_t iterations;

  /// default constructor
  QPState() :
      values(), duals(), workingSet(), converged(false), iterations(0) {
  }

  /// constructor with initial values
  QPState(const VectorValues& initialValues, const VectorValues& initialDuals,
      const InequalityFactorGraph& initialWorkingSet, bool _converged, size_t _iterations) :
      values(initialValues), duals(initialDuals), workingSet(initialWorkingSet), converged(
          _converged), iterations(_iterations) {
  }
};

/**
 * This QPSolver uses the active set method to solve a quadratic programming problem
 * defined in the QP struct.
 * Note: This version of QPSolver only works with a feasible initial value.
 */
class QPSolver {

  const QP& qp_; //!< factor graphs of the QP problem, can't be modified!
  GaussianFactorGraph baseGraph_; //!< factor graphs of cost factors and linear equalities.
                                  //!< used to initialize the working set factor graph,
                                  //!< to which active inequalities will be added
  VariableIndex costVariableIndex_, equalityVariableIndex_,
      inequalityVariableIndex_;  //!< index to corresponding factors to build dual graphs
  FastSet<Key> constrainedKeys_; //!< all constrained keys, will become factors in dual graphs

public:
  /// Constructor
  QPSolver(const QP& qp);

  /// Find solution with the current working set
  VectorValues solveWithCurrentWorkingSet(
      const InequalityFactorGraph& workingSet) const;

  /// @name Build the dual graph
  /// @{

  /// Collect the Jacobian terms for a dual factor
  template<typename FACTOR>
  std::vector<std::pair<Key, Matrix> > collectDualJacobians(Key key,
      const FactorGraph<FACTOR>& graph,
      const VariableIndex& variableIndex) const {
    std::vector<std::pair<Key, Matrix> > Aterms;
    if (variableIndex.find(key) != variableIndex.end()) {
      BOOST_FOREACH(size_t factorIx, variableIndex[key]) {
        typename FACTOR::shared_ptr factor = graph.at(factorIx);
        if (!factor->active()) continue;
        Matrix Ai = factor->getA(factor->find(key)).transpose();
        Aterms.push_back(std::make_pair(factor->dualKey(), Ai));
      }
    }
    return Aterms;
  }

  /// Create a dual factor
  JacobianFactor::shared_ptr createDualFactor(Key key,
      const InequalityFactorGraph& workingSet,
      const VectorValues& delta) const;

  /**
   * Build the dual graph to solve for the Lagrange multipliers.
   *
   * The Lagrangian function is:
   *        L(X,lambdas) = f(X) - \sum_k lambda_k * c_k(X),
   * where the unconstrained part is
   *        f(X) = 0.5*X'*G*X - X'*g + 0.5*f0
   * and the linear equality constraints are
   *        c1(X), c2(X), ..., cm(X)
   *
   * Take the derivative of L wrt X at the solution and set it to 0, we have
   *    \grad f(X) = \sum_k lambda_k * \grad c_k(X)   (*)
   *
   * For each set of rows of (*) corresponding to a variable xi involving in some constraints
   * we have:
   *    \grad f(xi) = \frac{\partial f}{\partial xi}' = \sum_j G_ij*xj - gi
   *    \grad c_k(xi) = \frac{\partial c_k}{\partial xi}'
   *
   * Note: If xi does not involve in any constraint, we have the trivial condition
   * \grad f(Xi) = 0, which should be satisfied as a usual condition for unconstrained variables.
   *
   * So each variable xi involving in some constraints becomes a linear factor A*lambdas - b = 0
   * on the constraints' lambda multipliers, as follows:
   *    - The jacobian term A_k for each lambda_k is \grad c_k(xi)
   *    - The constant term b is \grad f(xi), which can be computed from all unconstrained
   *    Hessian factors connecting to xi: \grad f(xi) = \sum_j G_ij*xj - gi
   */
  GaussianFactorGraph::shared_ptr buildDualGraph(
      const InequalityFactorGraph& workingSet,
      const VectorValues& delta) const;

  /// @}

  /**
   * The goal of this function is to find currently active inequality constraints
   * that violate the condition to be active. The one that violates the condition
   * the most will be removed from the active set. See Nocedal06book, pg 469-471
   *
   * Find the BAD active inequality that pulls x strongest to the wrong direction
   * of its constraint (i.e. it is pulling towards >0, while its feasible region is <=0)
   *
   * For active inequality constraints (those that are enforced as equality constraints
   * in the current working set), we want lambda < 0.
   * This is because:
   *   - From the Lagrangian L = f - lambda*c, we know that the constraint force
   *     is (lambda * \grad c) = \grad f. Intuitively, to keep the solution x stay
   *     on the constraint surface, the constraint force has to balance out with
   *     other unconstrained forces that are pulling x towards the unconstrained
   *     minimum point. The other unconstrained forces are pulling x toward (-\grad f),
   *     hence the constraint force has to be exactly \grad f, so that the total
   *     force is 0.
   *   - We also know that  at the constraint surface c(x)=0, \grad c points towards + (>= 0),
   *     while we are solving for - (<=0) constraint.
   *   - We want the constraint force (lambda * \grad c) to pull x towards the - (<=0) direction
   *     i.e., the opposite direction of \grad c where the inequality constraint <=0 is satisfied.
   *     That means we want lambda < 0.
   *   - This is because when the constrained force pulls x towards the infeasible region (+),
   *     the unconstrained force is pulling x towards the opposite direction into
   *     the feasible region (again because the total force has to be 0 to make x stay still)
   *     So we can drop this constraint to have a lower error but feasible solution.
   *
   * In short, active inequality constraints with lambda > 0 are BAD, because they
   * violate the condition to be active.
   *
   * And we want to remove the worst one with the largest lambda from the active set.
   *
   */
  int identifyLeavingConstraint(const InequalityFactorGraph& workingSet,
      const VectorValues& lambdas) const;

  /**
   * Compute step size alpha for the new solution x' = xk + alpha*p, where alpha \in [0,1]
   *
   *    @return a tuple of (alpha, factorIndex, sigmaIndex) where (factorIndex, sigmaIndex)
   *            is the constraint that has minimum alpha, or (-1,-1) if alpha = 1.
   *            This constraint will be added to the working set and become active
   *            in the next iteration
   */
  boost::tuple<double, int> computeStepSize(
      const InequalityFactorGraph& workingSet, const VectorValues& xk,
      const VectorValues& p) const;

  /** Iterate 1 step, return a new state with a new workingSet and values */
  QPState iterate(const QPState& state) const;

  /**
   * Identify active constraints based on initial values.
   */
  InequalityFactorGraph identifyActiveConstraints(
      const InequalityFactorGraph& inequalities,
      const VectorValues& initialValues,
      const VectorValues& duals = VectorValues(), bool useWarmStart = true) const;

  /** Optimize with a provided initial values
   * For this version, it is the responsibility of the caller to provide
   * a feasible initial value, otherwise, an exception will be thrown.
   * @return a pair of <primal, dual> solutions
   */
  std::pair<VectorValues, VectorValues> optimize(
      const VectorValues& initialValues, const VectorValues& duals = VectorValues(), bool useWarmStart = true) const;

};

/* ************************************************************************* */
/** An exception indicating that the noise model dimension passed into a
 * JacobianFactor has a different dimensionality than the factor. */
class InfeasibleInitialValues : public ThreadsafeException<InfeasibleInitialValues> {
public:
  InfeasibleInitialValues() {}
  virtual ~InfeasibleInitialValues() throw() {}

  virtual const char* what() const throw() {
    if(description_.empty())
      description_ = "An infeasible intial value was provided for the QPSolver.\n"
                     "This current version of QPSolver does not handle infeasible"
                     "initial point due to the lack of a LPSolver.\n";
    return description_.c_str();
  }

private:
  mutable std::string description_;
};

} /* namespace gtsam */
/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    HybridBayesNet.h
 * @brief   A bayes net of Gaussian Conditionals indexed by discrete keys.
 * @author  Varun Agrawal
 * @author  Fan Jiang
 * @author  Frank Dellaert
 * @date    December 2021
 */

#pragma once

#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/hybrid/GaussianMixture.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianConditional.h>

#include <iostream>  // TODO!

namespace gtsam {

/**
 * A hybrid Bayes net can have discrete conditionals, Gaussian mixtures,
 * or pure Gaussian conditionals.
 */
class GTSAM_EXPORT HybridBayesNet : public BayesNet<GaussianMixture> {
 public:
  using Base = BayesNet<GaussianMixture>;
  using This = HybridBayesNet;
  using ConditionalType = GaussianMixture;
  using shared_ptr = boost::shared_ptr<HybridBayesNet>;
  using sharedConditional = boost::shared_ptr<ConditionalType>;

  /** Construct empty bayes net */
  HybridBayesNet() : Base() {}

  void add(const DiscreteKey &key, const std::string &table) {
    GaussianMixture discrete(key, DiscreteKeys(),
                             Signature(key, DiscreteKeys(), table));
    push_back(discrete);
  }

  /**
   * Get a specific Gaussian mixture factor by index
   * (this checks array bounds and may throw an exception, as opposed to
   * operator[] which does not).
   */
  GaussianMixture::shared_ptr atGaussian(size_t i) const;

  /**
   * Get a specific Gaussian mixture factor by index
   * (this checks array bounds and may throw an exception, as opposed to
   * operator[] which does not).
   */
  DiscreteConditional::shared_ptr atDiscrete(size_t i) const;

  GaussianBayesNet choose(const DiscreteValues &assignment) const;
};

}  // namespace gtsam

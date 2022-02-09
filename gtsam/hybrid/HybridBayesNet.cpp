/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    HybridBayesNet.cpp
 * @brief   A set of GaussianFactors, indexed by a set of discrete keys.
 * @author  Fan Jiang
 * @date    January 2022
 */

#include <gtsam/hybrid/HybridBayesNet.h>

namespace gtsam {

GaussianMixture::shared_ptr HybridBayesNet::atGaussian(size_t i) {
  return boost::dynamic_pointer_cast<GaussianMixture>(factors_.at(i));
}
DiscreteConditional::shared_ptr HybridBayesNet::atDiscrete(size_t i) {
  return boost::dynamic_pointer_cast<DiscreteConditional>(factors_.at(i));
}

}

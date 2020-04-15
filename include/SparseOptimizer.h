/*
 * SparseOptimizer.h
 *
 *  Created on: Jan 29, 2018
 *      Author: Jerry Hsiung
 */

#ifndef SPARSEOPTIMIZER_H_
#define SPARSEOPTIMIZER_H_

#include <vector>
#include <list>
#include <Eigen/Core>

/* Jacobian wrt vertex, start index of vertex in info matrix */
typedef std::pair<Eigen::MatrixXd, int> JacobianEntry;
typedef std::list<JacobianEntry> MeasurementJacobian;
typedef std::list<MeasurementJacobian> JacobianMapping;

std::list<Eigen::MatrixXd> optimizeInformation(
        const JacobianMapping &mapping,
        const Eigen::MatrixXd &targetInformation);

#endif /* SPARSEOPTIMIZER_H_ */

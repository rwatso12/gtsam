/**
 * @file PseudorangeFactor.h
 * @date  27 Sept 2016 
 * @author Ryan Watson
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>  // for 
#include <gtsam/navigation/NavState.h> 
#include <gtsam/geometry/Point3.h>  // for 3d points 
#include <gtsam/base/LieScalar.h>  // for vectors

namespace gtsam {

class PseudorangeFactor: public NoiseModelFactor1 {

	const double measuredRho_;
	const Point3 satXYZ_;
	const Vector statesPre_;  // States ={X,Y,Z,Bias_clock,Trop}

public:

	/**
	* Constructor of pseudorange factors
	**/

	PseudorangeFactor(Key, key, const double measuredRho, Point3& satXYZ,
		Vector& statesPre_, const SharedNoiseModel& model):
	NoiseModelFactor1(model, key)
	measuredRho_(measuredRho), satXYZ_(satXYZ), statesPre_(statesPre){
	}


	/// @return a deep copy of this factor
	virtual NonlinearFactor::shared_ptr clone() const {
		return boost::static_pointer_cast<NonlinearFactor>(
		NonlinearFactor::shared_ptr(new PseudorangeFactor(*this)));


	vector evaluateError() {
	
		double bias = trop + 
		Point3 hx = sumSquares + bias;
		return (hx-measured_).vector();
	}
}  // class PseudorangeFactor
} // namespace gtsam

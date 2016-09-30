#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Vector.h> 
#include <gtsam/base/Matrix.h>
#include <gtsam/slam/PriorFactor.h>

using namespace std;
using namespace gtsam;

inline Matrix ones( size_t m, size_t n ) { return Matrix::Ones(m,n); }
inline double norm_2(const Vector& v) {return v.norm();}

class Pseudorange: public NoiseModelFactor1<Vector> {

  Vector rho(4);       // measured pseudorange values
  Matrix satXYZ(4,3);    // (ECEF)
  Vector stateEst(4);  // (ECEF)
  Matrix obsMap(4,4);   // mapping from states to observables
  Vector rhoComp(4);    // computed pseudorange values.
  Vector z(4);          // Measured - Computed;

public:

  typedef boost::shared_ptr<Pseudorange> shared_ptr;

  Pseudorange(Key j, Vector meas, Matrix sat, Vector x, const SharedNoiseModel& model):
    NoiseModelFactor1<Vector>(model, j), stateEst(x), satXYZ(sat), rho(meas) {}

  virtual ~Pseudorange() {}

  Vector evaluateError(const Vector x, boost::optional<Matrix&> H = boost::none) const
  {
    obsMap = ones(4,4);
    Vector nom = x(0:2,0);
    for (int i=0;i<4;i++) {
      Vector sat = satXYZ<1,3>(i,0);
      //double trop = estTrop(i);
      double clk = x(4,0);
      Vector rhoComp(i,0) = ((sat-nom).norm()) + clk;
      //Vector rhoComp(i,0) = ((sat-nom).norm()) + trop + clk;
      obsMap.block<1,3>(i,0) = (sat-nom)/((sat-nom).norm());
    }
    Vector z = rho - rhoComp;
    if (H) (*H) = (obsMap).finished();
    return (Vector(4) << obsMap*x-z).finished();
  }
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new UnaryFactor(*this))); }
}; // Pseudoragne Factor

int main(int argc, char** argv) {

  NonlinearFactorGraph graph;

  // init  
  Vector noise(4); noise << 200,200,200,10000;  // meters
  Vector rho(4), initEst1(4), initEst2(4), initEst3(4);
  Vector  priorEst(4); priorEst << 12309, 23424, 23434, 22222;

  noiseModel::Diagonal::shared_ptr pseudoragneNoise = noiseModel::Diagonal::Sigmas(noise);
  graph.add(PriorFactor<Vector>(1, priorEst, pseudoragneNoise));
  //graph.add(boost::make_shared<Pseudorange>(1, rho1, sat1, states1, pseudoragneNoise));
  //graph.add(boost::make_shared<Pseudorange>(2, rho2, sat2, states2, pseudoragneNoise));
  //graph.add(boost::make_shared<Pseudorange>(3, rho3, sat3, states3, pseudoragneNoise));
  graph.print("\nFactor Graph:\n"); 

  Values initialEstimate;
  //initialEstimate.insert(1, initEst1);
  //initialEstimate.insert(2, initEst2);
  //initialEstimate.insert(3, initEst3);
  initialEstimate.print("\nInitial Estimate:\n"); 

  //LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
  //Values result = optimizer.optimize();
  //result.print("Final Result:\n");

  return 0;
}

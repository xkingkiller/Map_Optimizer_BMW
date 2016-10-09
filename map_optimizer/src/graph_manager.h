#ifndef GRAPH_MANAGER_H_
#define GRAPH_MANAGER_H_

#include "g2o/types/slam2d/types_slam2d.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include <vector>
#include <map>
#include <Eigen/StdVector>
#include <boost/shared_ptr.hpp>

class GraphManager
{
public:
	struct OdomPose
	{
//		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		int id;
		g2o::SE2 rawPose;
		g2o::SE2 refinedPose;
	};
	typedef std::vector<OdomPose, Eigen::aligned_allocator<OdomPose> >  OdomPosesVector;

	struct PoseEdge
	{
	  int from;
	  int to;
	  g2o::SE2 rawTransf;
	  g2o::SE2 refinedTransf;
	  Eigen::Matrix3d information;
//	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	};
	typedef std::vector<PoseEdge, Eigen::aligned_allocator<PoseEdge> >  PoseEdgeVector;

	typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

public:
	GraphManager();
	~GraphManager();
	void addVertex(const OdomPose& pose);
	void addEdge(const PoseEdge& edge);
	void setFixed(int id);
	void optimize(int intrations = 10);
	OdomPose getVertex(int id);
private:
	g2o::SparseOptimizer optimizer_;
	boost::shared_ptr<SlamLinearSolver> linearSolver_;
	boost::shared_ptr<SlamBlockSolver> blockSolver_;
	boost::shared_ptr<g2o::OptimizationAlgorithmGaussNewton> solver_;
};
#endif //GRAPH_MANAGER_H_

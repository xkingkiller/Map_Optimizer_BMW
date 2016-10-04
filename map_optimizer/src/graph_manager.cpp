#include "graph_manager.h"

GraphManager::GraphManager()
{
	linearSolver_.reset( new SlamLinearSolver() );
	blockSolver_.reset( new SlamBlockSolver(linearSolver_.get()) );
	solver_.reset( new g2o::OptimizationAlgorithmGaussNewton(blockSolver_.get()) );
	optimizer_.setAlgorithm(solver_.get());
}
GraphManager::~GraphManager()
{
	// freeing the graph memory
	optimizer_.clear();

	// destroy all the singletons
	g2o::Factory::destroy();
	g2o::OptimizationAlgorithmFactory::destroy();
	g2o::HyperGraphActionLibrary::destroy();
}
void GraphManager::addVertex(const OdomPose& pose)
{
	const g2o::SE2& t = pose.rawPose;
	g2o::VertexSE2* robot =  new g2o::VertexSE2;
	robot->setId(pose.id);
	robot->setEstimate(t);
	optimizer_.addVertex(robot);
}
void GraphManager::addEdge(const PoseEdge& edge)
{
	g2o::EdgeSE2* odometry = new g2o::EdgeSE2;
	odometry->vertices()[0] = optimizer_.vertex(edge.from);
	odometry->vertices()[1] = optimizer_.vertex(edge.to);
	odometry->setMeasurement(edge.rawTransf);
	odometry->setInformation(edge.information);
	optimizer_.addEdge(odometry);
}

void GraphManager::setFixed(int id)
{
	// fix the first robot pose to account for gauge freedom
	g2o::VertexSE2* firstRobotPose = dynamic_cast<g2o::VertexSE2*>(optimizer_.vertex(id));
	firstRobotPose->setFixed(true);
}

void GraphManager::optimize(int intrations)
{
	optimizer_.setVerbose(true);

	optimizer_.initializeOptimization();
	optimizer_.optimize(intrations);
}

GraphManager::OdomPose GraphManager::getVertex(int id)
{
	OdomPose pose;
	pose.id = -1;
//	if(optimizer_) return pose;
	g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2*>(optimizer_.vertex( id ));
	if(v != 0)
	{
		pose.id = id;
		pose.rawPose = v->estimate();
		pose.refinedPose = v->estimate();
	}
	return pose;
}

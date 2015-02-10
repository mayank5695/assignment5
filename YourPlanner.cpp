#include "YourPlanner.h"

#include <rl/plan/SimpleModel.h>

YourPlanner::YourPlanner() :  
    RrtConConBase(),
	i_chooser(0)
{
}

YourPlanner::~YourPlanner()
{
}

::std::string
YourPlanner::getName() const
{
	return "Your Planner";
}

// copied from assignment 4
double
YourPlanner::gaussianRandom( double mean, double std ) {
	const double norm = 1.0 / (RAND_MAX + 1.0);
	double u = 1.0 - rand()*norm;
	double v = rand()*norm;
	double z = sqrt(-2.0*log(u))* cos(2.0*M_PI*v);
	return mean + std*z;
}

void
YourPlanner::gaussianSample(::rl::math::Vector& dest, const ::rl::math::Vector& reference, ::rl::math::Real stdDev)
{
    for(int i = 0; i < dest.size();i++){
        dest[i] = YourPlanner::gaussianRandom(reference[i], stdDev);
    }
}

void
YourPlanner::choose(::rl::math::Vector& chosen)
{
	::rl::math::Real std = 1.0; // depends on q dimension
	//::rl::math::Vector connectingVector = *this->start - *this->goal;
	//::rl::math::Vector VectorToPointBetweenStartAndGoal = connectingVector*rand()*(.5/(RAND_MAX+1.));
	::rl::math::Vector viaPoint(this->model->getDof());

	// This code cycles through different ways of sampling configurations.
	// (0): This is the default uniform distribution.
	// (1): This is a gaussian distribution around the initial configuration.
	// (2): This is a gaussian distribution around the desired configuration.
	//
	// Since the number of cycles of this loop and the number of trees are
	// relatively prime, this ensures that any choosing algorithm implemented
	// below is applied to any of the trees.
	
	switch(i_chooser++)
	{
	case 0:
        RrtConConBase::choose(chosen);

		break;

	case 1:
        viaPoint = *this->start;
        //	viaPoint = *this->goal + VectorToPointBetweenStartAndGoal;
		this->gaussianSample(chosen, viaPoint, std);

		break;

	case 2:
	default:
		i_chooser = 0;

		viaPoint = *this->goal;
        //	viaPoint = *this->goal + VectorToPointBetweenStartAndGoal;
		this->gaussianSample(chosen, viaPoint, std);

		break;
    }

	// TODO: remove configurations outside of joint limits
}

RrtConConBase::Vertex
YourPlanner::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
    RrtConConBase::Vertex tempvertex = RrtConConBase::connect(tree, nearest, chosen);

	// RrtConConBase::connect tries to connect the configuration `chosen` with
	// the closest configration (given as `nearest`).
	// It returns NULL if it is not possible to extend the `nearest` configuration.
	//
	// Those configurations that cannot be extended (or fail to be extended many times)
	// are most likely 'dead ends' and thus can be purged from the tree.
	//
	// For that purpose, a counter is incremented once everytime an extension fails.
	// If a specified limit is exceeded (`NEAREST_MAX_FAILS`), that configuration is
	// purged from the tree.

    if (tempvertex == NULL)
    {
        //nearest.first;
		tree[nearest.first].fail_counter++;
    }
    else
    {
        //remove extended node
        if (tree[nearest.first].fail_counter >= NEAREST_MAX_FAILS)
        {
            for (int i=0;i<RrtConConBase::tree.size();i++)
            {
                RrtConConBase::tree[i].removing_vertex(nearest.first);
            }

        }
    }

    return tempvertex ;
}
	
RrtConConBase::Vertex 
YourPlanner::extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
    //your modifications here
	return RrtConConBase::extend(tree, nearest, chosen);
}

bool
YourPlanner::solve()
{
	//your modifications here
	return RrtConConBase::solve();

}


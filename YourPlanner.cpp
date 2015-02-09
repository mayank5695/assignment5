#include "YourPlanner.h"

YourPlanner::YourPlanner() :  
    RrtConConBase()
{
    i_chooser = 0;
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
YourPlanner::choose(::rl::math::Vector& chosen)
{
	double std = 1.; // depends on q dimension
	::rl::math::Vector connectingVector = *this->start - *this->goal;
	::rl::math::Vector VectorToPointBetweenStartAndGoal = connectingVector*rand()*(.5/(RAND_MAX+1.));
	::rl::math::Vector viaPoint;
	
	i_chooser = 0;
	viaPoint = *this->start*0;
	
//	viaPoint = *this->goal + VectorToPointBetweenStartAndGoal;
	for(int i = 0; i < chosen.size();i++){
		chosen[i] = YourPlanner::gaussianRandom(viaPoint[i],std);
	}
	// TODO: remove configurations outside of joint limits
}

RrtConConBase::Vertex
YourPlanner::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
	//your modifications here
    //git test
    //return RrtConConBase::connect(tree, nearest, chosen);
    RrtConConBase::Vertex tempvertex = RrtConConBase::connect(tree, nearest, chosen);
    if (tempvertex == NULL)
    {
        //nearest.first;
                tree[nearest.first].fail_counter++;
    }
    else
    {
        //remove extended note
        if (tree[nearest.first].fail_counter >= 20)
        {
            //std::cout << "removing" << std::endl;
            for (int i=0;i<RrtConConBase::tree.size();i++)
            {
                RrtConConBase::tree[i].removing_vertex(tempvertex);
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


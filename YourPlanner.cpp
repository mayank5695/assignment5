#include "YourPlanner.h"

YourPlanner::YourPlanner() :
	RrtConConBase()
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

void
YourPlanner::choose(::rl::math::Vector& chosen)
{
	//your modifications here
    double i;
    if (i!=0) {i=0;}
    if (i==10)
    {
    chosen = this->goal;
    i=0;
    }
	RrtConConBase::choose(chosen);
    i++;
}

RrtConConBase::Vertex 
YourPlanner::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
	//your modifications here
    //git test
	return RrtConConBase::connect(tree, nearest, chosen);
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


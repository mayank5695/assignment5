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

void
YourPlanner::choose(::rl::math::Vector& chosen)
{
    //your modifications here
    if (i_chooser >= 10)
    {
    chosen = *(this->goal);
    i_chooser = 0;
    }
    else
    {
        RrtConConBase::choose(chosen);
    }
    i_chooser ++;

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


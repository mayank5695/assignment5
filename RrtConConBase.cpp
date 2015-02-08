//
// Copyright (c) 2009, Markus Rickert
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include "RrtConConBase.h"
#include <rl/plan/Sampler.h>
#include <rl/plan/SimpleModel.h>
#include <rl/plan/Verifier.h>
#include <rl/plan/Viewer.h>
#include <boost/make_shared.hpp>

RrtConConBase::RrtConConBase() :
	Planner(),
	delta(1.0f),
	epsilon(1.0e-3f),
	sampler(NULL),
	begin(2, NULL),
	end(2, NULL),
	tree(2)
{
}

RrtConConBase::~RrtConConBase()
{
}

RrtConConBase::Edge
RrtConConBase::addEdge(const Vertex& u, const Vertex& v, Tree& tree)
{
	Edge e = ::boost::add_edge(u, v, tree).first;
	
	if (NULL != this->viewer)
	{
		this->viewer->drawConfigurationEdge(*tree[u].q, *tree[v].q);
	}
	
	return e;
}


void
RrtConConBase::addPoint(NearestNeighbors& nn, const QueryItem& p)
{
	NearestNeighbors::iterator i;
	
	for (i = nn.begin(); i != nn.end(); ++i)
	{
		if (NULL == *i)
		{
			break;
		}
	}
	
	NeighborSearchTreePtr tree(new NeighborSearchTree());
	
	if (nn.end() == i)
	{
		i = nn.insert(i, tree);
	}
	else
	{
		*i = tree;
	}
	
	for (NearestNeighbors::iterator j = nn.begin(); j != i; ++j)
	{
		if (NULL != *j)
		{
			(*i)->insert((*j)->begin(), (*j)->end());
			j->reset();
		}
	}
	
	tree->insert(p);
}

RrtConConBase::Vertex
RrtConConBase::addVertex(Tree& tree, const ::rl::plan::VectorPtr& q)
{
	Vertex v = ::boost::add_vertex(tree);
	tree[v].index = ::boost::num_vertices(tree) - 1;
	tree[v].q = q;
	
	//Also add the vertex as a point into the kd-tree
	this->addPoint(tree[::boost::graph_bundle].nn, QueryItem(q.get(), v));
	
	
	if (NULL != this->viewer)
	{
		this->viewer->drawConfigurationVertex(*tree[v].q);
	}
	
	return v;
}

bool
RrtConConBase::areEqual(const ::rl::math::Vector& lhs, const ::rl::math::Vector& rhs) const
{
	if (this->model->distance(lhs, rhs) > this->epsilon)
	{
		return false;
	}
	else
	{
		return true;
	}
}

void
RrtConConBase::choose(::rl::math::Vector& chosen)
{
	this->sampler->generate(chosen);
}

RrtConConBase::Vertex
RrtConConBase::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
	//Do first extend step

	::rl::math::Real distance = nearest.second;
	::rl::math::Real step = distance;
	
	bool reached = false;
	
	if (step <= this->delta)
	{
		reached = true;
	}
	else
	{
		step = this->delta;
	}
	
	::rl::plan::VectorPtr last = ::boost::make_shared< ::rl::math::Vector >(this->model->getDof());
	
	// move "last" along the line q<->chosen by distance "step / distance"
	this->model->interpolate(*tree[nearest.first].q, chosen, step / distance, *last);
	
	this->model->setPosition(*last);
	this->model->updateFrames();
	
	if (this->model->isColliding())
	{
		return NULL;
	}
	
	::rl::math::Vector next(this->model->getDof());
	
	while (!reached)
	{
		//Do further extend step

		distance = this->model->distance(*last, chosen);
		step = distance;
		
		if (step <= this->delta)
		{
			reached = true;
		}
		else
		{
			step = this->delta;
		}
		
		// move "next" along the line last<->chosen by distance "step / distance"
		this->model->interpolate(*last, chosen, step / distance, next);
		
		this->model->setPosition(next);
		this->model->updateFrames();
		
		if (this->model->isColliding())
		{
			break;
		}
		
		*last = next;
	}
	
	// "last" now points to the vertex where the connect step collided with the environment.
	// Add it to the tree
	Vertex connected = this->addVertex(tree, last);
	this->addEdge(nearest.first, connected, tree);
	return connected;
}

RrtConConBase::Vertex
RrtConConBase::extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
	::rl::math::Real distance = nearest.second;
	::rl::math::Real step = (::std::min)(distance, this->delta);
	
	::rl::plan::VectorPtr next = ::boost::make_shared< ::rl::math::Vector >(this->model->getDof());
	
	this->model->interpolate(*tree[nearest.first].q, chosen, step / distance, *next);
	
	this->model->setPosition(*next);
	this->model->updateFrames();
	
	if (!this->model->isColliding())
	{
		Vertex extended = this->addVertex(tree, next);
		this->addEdge(nearest.first, extended, tree);
		return extended;
	}
	
	return NULL;
}

::std::string
RrtConConBase::getName() const
{
	return "RrtConConBase";
}

::std::size_t
RrtConConBase::getNumEdges() const
{
	::std::size_t edges = 0;
	
	for (::std::size_t i = 0; i < this->tree.size(); ++i)
	{
		edges += ::boost::num_edges(this->tree[i]);
	}
	
	return edges;
}

::std::size_t
RrtConConBase::getNumVertices() const
{
	::std::size_t vertices = 0;
	
	for (::std::size_t i = 0; i < this->tree.size(); ++i)
	{
		vertices += ::boost::num_vertices(this->tree[i]);
	}
	
	return vertices;
}

void
RrtConConBase::getPath(::rl::plan::VectorList& path)
{
	Vertex i = this->end[0];
	
	while (i != this->begin[0])
	{
		path.push_front(*this->tree[0][i].q);
		i = ::boost::source(*::boost::in_edges(i, this->tree[0]).first, this->tree[0]);
	}
	
	path.push_front(*this->tree[0][i].q);
	
	i = ::boost::source(*::boost::in_edges(this->end[1], this->tree[1]).first, this->tree[1]);
	
	while (i != this->begin[1])
	{
		path.push_back(*this->tree[1][i].q);
		i = ::boost::source(*::boost::in_edges(i, this->tree[1]).first, this->tree[1]);
	}
	
	path.push_back(*this->tree[1][i].q);
}

RrtConConBase::Neighbor
RrtConConBase::nearest(const Tree& tree, const ::rl::math::Vector& chosen)
{
	Neighbor p(Vertex(), (::std::numeric_limits< ::rl::math::Real >::max)());
	
	QueryItem query(&chosen, Vertex());
		
	for (NearestNeighbors::const_iterator i = tree[::boost::graph_bundle].nn.begin(); i != tree[::boost::graph_bundle].nn.end(); ++i)
	{
		if (NULL != *i)
		{
			NeighborSearch search(
				*i->get(),
				query,
				1,
				0,
				true,
				Distance(this->model)
			);
				
			if (search.begin()->second < p.second)
			{
				p.first = search.begin()->first.second;
				p.second = search.begin()->second;
			}
		}
	}

	
	p.second = this->model->inverseOfTransformedDistance(p.second);
	
	return p;
}

void
RrtConConBase::reset()
{
	for (::std::size_t i = 0; i < this->tree.size(); ++i)
	{
		this->tree[i].clear();
		this->tree[i][::boost::graph_bundle].nn.clear();
		this->begin[i] = NULL;
		this->end[i] = NULL;
	}
}

bool
RrtConConBase::solve()
{
	// Define the roots of both trees
	this->begin[0] = this->addVertex(this->tree[0], ::boost::make_shared< ::rl::math::Vector >(*this->start));
	this->begin[1] = this->addVertex(this->tree[1], ::boost::make_shared< ::rl::math::Vector >(*this->goal));
			
	Tree* a = &this->tree[0];
	Tree* b = &this->tree[1];
			
	::rl::math::Vector chosen(this->model->getDof());
			
	timer.start();
	timer.stop();
			
	while (timer.elapsed() < this->duration)
	{
		//First grow tree a and then try to connect b. 
		//then swap roles and first grow tree b and then connect to a.
		for (::std::size_t j = 0; j < 2; ++j)
		{
			//Try to connect sample to tree a
			this->choose(chosen);
					
			Neighbor aNearest = this->nearest(*a, chosen);
					
			Vertex aConnected = this->connect(*a, aNearest, chosen);
					
			if (NULL != aConnected)
			{
				// Now try to connect sample to tree b
				Neighbor bNearest = this->nearest(*b, *(*a)[aConnected].q);
						
				Vertex bConnected = this->connect(*b, bNearest, *(*a)[aConnected].q);
						
				if (NULL != bConnected)
				{
					//Test if both trees are connected
					if (this->areEqual(*(*a)[aConnected].q, *(*b)[bConnected].q))
					{
						this->end[0] = &this->tree[0] == a ? aConnected : bConnected;
						this->end[1] = &this->tree[1] == b ? bConnected : aConnected;
						return true;
					}
				}
			}
			
			::std::swap(a, b);
		}
				
		timer.stop();
	}
			
	return false;
	
}


//////////////////////////////////////////////////////////
// Function definitions for kd tree //////////////////////
//////////////////////////////////////////////////////////

const ::rl::math::Real*
RrtConConBase::CartesianIterator::operator()(const QueryItem& p) const
{
	return p.first->data(); // TODO
}

const ::rl::math::Real*
RrtConConBase::CartesianIterator::operator()(const QueryItem& p, const int&) const
{
	return p.first->data() + p.first->size(); // TODO
}


RrtConConBase::Distance::Distance() :
	model(NULL)
{
}

RrtConConBase::Distance::Distance(::rl::plan::Model* model) :
	model(model)
{
}

template<>
::rl::math::Real
#if (CGAL_VERSION_NR > 1030801000)
RrtConConBase::Distance::max_distance_to_rectangle(const Query_item& q, const ::CGAL::Kd_tree_rectangle< RrtConConBase::SearchTraits::FT >& r) const
#else
RrtConConBase::Distance::max_distance_to_rectangle(const Query_item& q, const ::CGAL::Kd_tree_rectangle< RrtConConBase::SearchTraits >& r) const
#endif
{
	::rl::math::Vector min_dist(r.dimension());
	::rl::math::Vector max_dist(r.dimension());
	
	for (int i = 0; i < r.dimension(); ++i)
	{
		min_dist(i) = r.min_coord(i);
		max_dist(i) = r.max_coord(i);
	}
	
	return this->model->maxDistanceToRectangle(*q.first, min_dist, max_dist);
}

template<>
::rl::math::Real
#if (CGAL_VERSION_NR > 1030801000)
RrtConConBase::Distance::min_distance_to_rectangle(const Query_item& q, const ::CGAL::Kd_tree_rectangle< RrtConConBase::SearchTraits::FT >& r) const
#else
RrtConConBase::Distance::min_distance_to_rectangle(const Query_item& q, const ::CGAL::Kd_tree_rectangle< RrtConConBase::SearchTraits >& r) const
#endif
{
	::rl::math::Vector min_dist(r.dimension());
	::rl::math::Vector max_dist(r.dimension());
	
	for (int i = 0; i < r.dimension(); ++i)
	{
		min_dist(i) = r.min_coord(i);
		max_dist(i) = r.max_coord(i);
	}
	
	return this->model->minDistanceToRectangle(*q.first, min_dist, max_dist);
}

::rl::math::Real
RrtConConBase::Distance::min_distance_to_rectangle(const ::rl::math::Real& q, const ::rl::math::Real& min, const ::rl::math::Real& max, const ::std::size_t& cutting_dimension) const
{
	return this->model->minDistanceToRectangle(q, min, max, cutting_dimension);
}

::rl::math::Real
RrtConConBase::Distance::new_distance(const ::rl::math::Real& dist, const ::rl::math::Real& old_off, const ::rl::math::Real& new_off, const int& cutting_dimension) const
{
	return this->model->newDistance(dist, old_off, new_off, cutting_dimension);
}

::rl::math::Real
RrtConConBase::Distance::transformed_distance(const ::rl::math::Real& d) const
{
	return this->model->transformedDistance(d);
}

::rl::math::Real
RrtConConBase::Distance::transformed_distance(const Query_item& q1, const Query_item& q2) const
{
	return this->model->transformedDistance(*q1.first, *q2.first);
}

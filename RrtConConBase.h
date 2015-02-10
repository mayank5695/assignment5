//
// Copyright (c) 2009, Markus Rickert
// All rights reserved.
//
// Modified and commented by Arne Sieverling
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

#ifndef RRT_CON_CON_BASE_H
#define RRT_CON_CON_BASE_H

#include <boost/graph/adjacency_list.hpp>
#include <CGAL/Search_traits.h>

#include <rl/plan/MatrixPtr.h>
#include <rl/plan/Model.h>
#include <rl/plan/Orthogonal_k_neighbor_search.h>
#include <rl/plan/Planner.h>
#include <rl/plan/Sampler.h>
#include <rl/plan/TransformPtr.h>
#include <rl/plan/VectorPtr.h>
#include <rl/plan/Verifier.h>

/**
 * Rapidly-Exploring Random Trees.
 * 
 * Steven M. LaValle. Rapidly-exploring random trees: A new tool for path
 * planning. Technical Report TR 98-11, Iowa State University, Ames, IA,
 * USA, October 1998.
 * 
 * http://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf
 */
class RrtConConBase : public rl::plan::Planner
{
public:
	RrtConConBase();
	
	virtual ~RrtConConBase();
	
	virtual ::std::string getName() const;
	
	virtual ::std::size_t getNumEdges() const;
	
	virtual ::std::size_t getNumVertices() const;
	
	virtual void getPath(rl::plan::VectorList& path);
	
	virtual void reset();
	
    virtual bool solve();

	/////////////////////////////////////////////////////////////////////////
	// Planner parameters ///////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////
	
	/** Configuration step size. */
	::rl::math::Real delta;
	
	/** Epsilon for configuration comparison. */
	::rl::math::Real epsilon;
	
	/** The sampler used for planning */
	::rl::plan::Sampler* sampler;
	
protected:
	/////////////////////////////////////////////////////////////////////////
	// boost graph definitions //////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////

	/** This struct defines all variables that are stored in each tree vertex. 
	If you need additional parameters for vertices add them here */
    class VertexBundle
	{
    public:
		::std::size_t index;
		
		::rl::plan::VectorPtr q;

        unsigned int fail_counter;
        bool is_connected;

        VertexBundle()
        {
            is_connected = true;
            fail_counter = 0;
        }

        void set_connected(bool connected = true)
        {
            this->is_connected = connected;
            if(connected)
                this->fail_counter = 0;
        }

        unsigned int fail()
        {
            if(!this->is_connected)
                return ++this->fail_counter;
        }

	};


	
	typedef ::boost::adjacency_list_traits<
		::boost::listS,
		::boost::listS,
		::boost::bidirectionalS,
		::boost::listS
	>::vertex_descriptor Vertex; 
	
	typedef ::std::pair< const ::rl::math::Vector*, Vertex > QueryItem;
	
	struct CartesianIterator
	{
		typedef const ::rl::math::Real* result_type;
		
		const ::rl::math::Real* operator()(const QueryItem& p) const;
		
		const ::rl::math::Real* operator()(const QueryItem& p, const int&) const;
	};

	
	/////////////////////////////////////////////////////////////////////////
	// Kd-tree definitions //////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////

	struct Distance
	{
		typedef QueryItem Query_item;
		
		Distance();
		
		Distance(::rl::plan::Model* model);
		
		template< typename SearchTraits > ::rl::math::Real max_distance_to_rectangle(const Query_item& q, const ::CGAL::Kd_tree_rectangle< SearchTraits >& r) const;
		
		template< typename SearchTraits > ::rl::math::Real min_distance_to_rectangle(const Query_item& q, const ::CGAL::Kd_tree_rectangle< SearchTraits >& r) const;
		
		::rl::math::Real min_distance_to_rectangle(const ::rl::math::Real& q, const ::rl::math::Real& min, const ::rl::math::Real& max, const ::std::size_t& cutting_dimension) const;
		
		::rl::math::Real new_distance(const ::rl::math::Real& dist, const ::rl::math::Real& old_off, const ::rl::math::Real& new_off, const int& cutting_dimension) const;
		
		::rl::math::Real transformed_distance(const ::rl::math::Real& d) const;
		
		::rl::math::Real transformed_distance(const Query_item& q1, const Query_item& q2) const;
		
		::rl::plan::Model* model;
	};
	
	typedef ::CGAL::Search_traits< ::rl::math::Real, QueryItem, const ::rl::math::Real*, CartesianIterator > SearchTraits;
	
	typedef ::rl::plan::Orthogonal_k_neighbor_search< SearchTraits, Distance > NeighborSearch;
	
	typedef NeighborSearch::Tree NeighborSearchTree;
	
	typedef ::boost::shared_ptr< NeighborSearchTree > NeighborSearchTreePtr;
	
	typedef ::std::vector< NeighborSearchTreePtr > NearestNeighbors;
	
	/////////////////////////////////////////////////////////////////////////
	// more boost graph definitions /////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////

	struct TreeBundle
	{
		NearestNeighbors nn;
	};

    /** This defines a boost graph */
    typedef ::boost::adjacency_list<
        ::boost::listS,
        ::boost::listS,
        ::boost::bidirectionalS,
        VertexBundle,
        ::boost::no_property,
        TreeBundle
    > Tree;

    typedef ::boost::graph_traits< Tree >::edge_descriptor Edge;

    typedef ::boost::graph_traits< Tree >::edge_iterator EdgeIterator;

    typedef ::std::pair< EdgeIterator, EdgeIterator > EdgeIteratorPair;

    typedef ::boost::graph_traits< Tree >::vertex_iterator VertexIterator;

    typedef ::std::pair< VertexIterator, VertexIterator > VertexIteratorPair;

    typedef ::std::pair< Vertex, ::rl::math::Real > Neighbor;
		
	////////////////////////////////////////////////////////////////////////
	// helper functions ////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////

	virtual Edge addEdge(const Vertex& u, const Vertex& v, Tree& tree);
	
	void addPoint(NearestNeighbors& nn, const QueryItem& p);
	
    Vertex addVertex(Tree& tree, const ::rl::plan::VectorPtr& q);
	
	bool areEqual(const ::rl::math::Vector& lhs, const ::rl::math::Vector& rhs) const;

	////////////////////////////////////////////////////////////////////////
	// RRT functions ///////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////
	
	/** Draws a random sample configuration*/
	virtual void choose(::rl::math::Vector& chosen);
	
	/** Extends vertex nearest of tree towards sample chosen*/
	virtual Vertex extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);

	/** Tries to connect vertex nearest of tree to sample chosen*/
	virtual Vertex connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);
	
	/** Returns the nearest neighbour of chosen in tree
	uses a kd-tree internally*/
	virtual Neighbor nearest(const Tree& tree, const ::rl::math::Vector& chosen);
	
	////////////////////////////////////////////////////////////////////////
	// members /////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////
	
	/** A vector of RRTs - here it's size 2 because we use two trees that grow towards each other */
	::std::vector< Tree > tree;

	/** Start and end of the solution path */
	::std::vector< Vertex > begin;	
	::std::vector< Vertex > end;
	
private:
	
};


#endif // RL_PLAN_RRT_CON_CON_BASE_H

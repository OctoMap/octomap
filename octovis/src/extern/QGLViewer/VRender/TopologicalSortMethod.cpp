/*
 This file is part of the VRender library.
 Copyright (C) 2005 Cyril Soler (Cyril.Soler@imag.fr)
 Version 1.0.0, released on June 27, 2005.

 http://artis.imag.fr/Members/Cyril.Soler/VRender

 VRender is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 VRender is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with VRender; if not, write to the Free Software Foundation, Inc.,
 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

/****************************************************************************

 Copyright (C) 2002-2008 Gilles Debunne. All rights reserved.

 This file is part of the QGLViewer library version 2.3.1.

 http://www.libqglviewer.com - contact@libqglviewer.com

 This file may be used under the terms of the GNU General Public License 
 versions 2.0 or 3.0 as published by the Free Software Foundation and
 appearing in the LICENSE file included in the packaging of this file.
 In addition, as a special exception, Gilles Debunne gives you certain 
 additional rights, described in the file GPL_EXCEPTION in this package.

 libQGLViewer uses dual licensing. Commercial/proprietary software must
 purchase a libQGLViewer Commercial License.

 This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.

*****************************************************************************/

#include <assert.h>
#include <climits>

#include "VRender.h"
#include "Primitive.h"
#include "PrimitivePositioning.h"
#include "AxisAlignedBox.h"
#include "SortMethod.h"
#include "Vector2.h"

using namespace std ;
using namespace vrender ;

// #define DEBUG_TS

namespace vrender
{
class TopologicalSortUtils
{
	public:
		static void buildPrecedenceGraph(vector<PtrPrimitive>& primitive_tab, vector< vector<int> >& precedence_graph) ;

		static void recursFindNeighbors(	const vector<PtrPrimitive>& primitive_tab,
													const vector<int>& pindices,
													vector< vector<int> >& precedence_graph,
													const AxisAlignedBox_xy&,int) ;

		static void checkAndAddEdgeToGraph(int a,int b,vector< vector<int> >& precedence_graph) ;
		static void suppressPrecedence(int a,int b,vector< vector<int> >& precedence_graph) ;

		static void recursTopologicalSort(vector< vector<int> >& precedence_graph,
													 vector<PtrPrimitive>& primitive_tab,
													 vector<bool>& alread_rendered,
													 vector<bool>& alread_visited,
													 vector<PtrPrimitive>&,int,int&,
													 VRenderParams& vparams,
													 int info_cnt,int& nbrendered) ;

		static void recursTopologicalSort(vector< vector<int> >& precedence_graph,
													 vector<PtrPrimitive>& primitive_tab,
													 vector<bool>& alread_rendered,
													 vector<bool>& alread_visited,
													 vector<PtrPrimitive>&,int,
													 vector<int>& ancestors,
													 int&, int&,
													 VRenderParams& vparams,
													 int info_cnt,int& nbrendered) ;

		static void topologicalSort(	vector< vector<int> >& precedence_graph,
												vector<PtrPrimitive>& primitive_tab,
												VRenderParams&) ;

		static void topologicalSortBreakCycles(vector< vector<int> >& precedence_graph,
															vector<PtrPrimitive>& primitive_tab,
															VRenderParams&) ;

#ifdef DEBUG_TS
		static void printPrecedenceGraph(const vector< vector<int> >& precedence_graph,
													const vector<PtrPrimitive>& primitive_tab) ;
#endif
};

TopologicalSortMethod::TopologicalSortMethod()
{
	_break_cycles = false ;
}

void TopologicalSortMethod::sortPrimitives(vector<PtrPrimitive>& primitive_tab,VRenderParams& vparams)
{
	// 1 - build a precedence graph

#ifdef DEBUG_TS
	cout << "Computing precedence graph." << endl ;
	cout << "Old order: " ;
	for(unsigned int i=0;i<primitive_tab.size();++i) cout << (void *)(primitive_tab[i]) << " " ;
	cout << endl ;
#endif
	vector< vector<int> > precedence_graph(primitive_tab.size());
	TopologicalSortUtils::buildPrecedenceGraph(primitive_tab,precedence_graph) ;

#ifdef DEBUG_TS
	TopologicalSortUtils::printPrecedenceGraph(precedence_graph,primitive_tab) ;
#endif
	// 2 - perform a topological sorting of the graph

#ifdef DEBUG_TS
	cout << "Sorting." << endl ;
#endif

	if(_break_cycles)
		TopologicalSortUtils::topologicalSortBreakCycles(precedence_graph, primitive_tab,vparams) ;
	else
		TopologicalSortUtils::topologicalSort(precedence_graph, primitive_tab,vparams) ;

#ifdef DEBUG_TS
	cout << "New order: " ;
	for(unsigned int i=0;i<primitive_tab.size();++i) cout << (void *)(primitive_tab[i]) << " " ;
	cout << endl ;
#endif
}

#ifdef DEBUG_TS
void TopologicalSortUtils::printPrecedenceGraph(const vector< vector<int> >& precedence_graph,
																const vector<PtrPrimitive>& primitive_tab)
{
	for(unsigned int i=0;i<precedence_graph.size();++i)
	{
		cout << i << " (" << primitive_tab[i]->nbVertices() << ") : " ;
		for(unsigned int j=0;j<precedence_graph[i].size();++j)
			cout << precedence_graph[i][j] << " " ;

		cout << endl ;
	}
}
#endif

void TopologicalSortUtils::buildPrecedenceGraph(vector<PtrPrimitive>& primitive_tab,
																vector< vector<int> >& precedence_graph)
{
	// The precedence graph is constructed by first conservatively determining which
	// primitives can possibly intersect using a quadtree. Candidate pairs of
	// primitives are then carefully checked to compute their exact relative positionning.
	//
	// Because of the conservativeness of the quadtree, some pairs of primitives may be checked
	// multiple times for intersection. Using a buffer of already computed results may proove
	// very efficient.

	// 0 - compute bounding box of the set of primitives.

	AxisAlignedBox_xy BBox ;

	for(unsigned int i=0;i<primitive_tab.size();++i)
	{
		BBox.include(Vector2(primitive_tab[i]->bbox().mini().x(),primitive_tab[i]->bbox().mini().y())) ;
		BBox.include(Vector2(primitive_tab[i]->bbox().maxi().x(),primitive_tab[i]->bbox().maxi().y())) ;
	}

	// 1 - recursively find pairs.

	vector<int> pindices(primitive_tab.size()) ;
	for(unsigned int j=0;j<pindices.size();++j)
		pindices[j] = j ;

	recursFindNeighbors(primitive_tab, pindices, precedence_graph, BBox,0) ;
}

void TopologicalSortUtils::recursFindNeighbors(const vector<PtrPrimitive>& primitive_tab,
																const vector<int>& pindices,
																vector< vector<int> >& precedence_graph,
																const AxisAlignedBox_xy& bbox,
																int depth)
{
	static const unsigned int MAX_PRIMITIVES_IN_CELL = 5 ;

	// Refinment: first decide which sub-cell each primitive meets, then call
	// algorithm recursively.

	if(primitive_tab.size() > MAX_PRIMITIVES_IN_CELL)
	{
		vector<int> p_indices_min_min ;
		vector<int> p_indices_min_max ;
		vector<int> p_indices_max_min ;
		vector<int> p_indices_max_max ;

		double xmin = bbox.mini().x() ;
		double ymin = bbox.mini().y() ;
		double xmax = bbox.maxi().x() ;
		double ymax = bbox.maxi().y() ;

		double xMean = 0.5*(xmin+xmax) ;
		double yMean = 0.5*(ymin+ymax) ;

		for(unsigned int i=0;i<pindices.size();++i)
		{
			bool left  = primitive_tab[pindices[i]]->bbox().mini().x() <= xMean ;
			bool right = primitive_tab[pindices[i]]->bbox().maxi().x() >= xMean ;
			bool down  = primitive_tab[pindices[i]]->bbox().mini().y() <= yMean ;
			bool up    = primitive_tab[pindices[i]]->bbox().maxi().y() >= yMean ;

			if(left  && down) p_indices_min_min.push_back(pindices[i]) ;
			if(right && down) p_indices_max_min.push_back(pindices[i]) ;
			if(left  && up  ) p_indices_min_max.push_back(pindices[i]) ;
			if(right && up  ) p_indices_max_max.push_back(pindices[i]) ;
		}

		// checks if refining is not too much stupid

		if(p_indices_min_min.size() < pindices.size() && p_indices_max_min.size() < pindices.size()
				&& p_indices_min_max.size() < pindices.size() && p_indices_max_max.size() < pindices.size())
		{
			recursFindNeighbors(primitive_tab,p_indices_min_min,precedence_graph,AxisAlignedBox_xy(Vector2(xmin,xMean),Vector2(ymin,yMean)),depth+1) ;
			recursFindNeighbors(primitive_tab,p_indices_min_max,precedence_graph,AxisAlignedBox_xy(Vector2(xmin,xMean),Vector2(yMean,ymax)),depth+1) ;
			recursFindNeighbors(primitive_tab,p_indices_max_min,precedence_graph,AxisAlignedBox_xy(Vector2(xMean,xmax),Vector2(ymin,yMean)),depth+1) ;
			recursFindNeighbors(primitive_tab,p_indices_max_max,precedence_graph,AxisAlignedBox_xy(Vector2(xMean,xmax),Vector2(yMean,ymax)),depth+1) ;
			return ;
		}
	}

	// No refinment either because it could not be possible, or because the number of primitives is below
	// the predefined limit.

	for(unsigned int i=0;i<pindices.size();++i)
		for(unsigned int j=i+1;j<pindices.size();++j)
		{
			// Compute the position of j as regard to i

			int prp = PrimitivePositioning::computeRelativePosition(	primitive_tab[pindices[i]], primitive_tab[pindices[j]]) ;

			if(prp & PrimitivePositioning::Upper) checkAndAddEdgeToGraph(pindices[j],pindices[i],precedence_graph) ;
			if(prp & PrimitivePositioning::Lower) checkAndAddEdgeToGraph(pindices[i],pindices[j],precedence_graph) ;
		}
}

void TopologicalSortUtils::checkAndAddEdgeToGraph(int a,int b,vector< vector<int> >& precedence_graph)
{
#ifdef DEBUG_TS
	cout << "Trying to add " << a << " -> " << b << " " ;
#endif
	bool found = false ;

	for(unsigned int k=0;k<precedence_graph[a].size() && !found;++k)
		if(precedence_graph[a][k] == b)
			found = true ;

#ifdef DEBUG_TS
	if(found)
		cout << "already" << endl ;
	else
		cout << "ok" << endl ;
#endif
	if(!found)
		precedence_graph[a].push_back(b) ;
}

void TopologicalSortUtils::suppressPrecedence(int a,int b,vector< vector<int> >& precedence_graph)
{
	vector<int> prec_tab = vector<int>(precedence_graph[a]) ;
	bool trouve = false ;

	for(unsigned int k=0;k<prec_tab.size();++k)
		if(prec_tab[k] == b)
		{
			prec_tab[k] = prec_tab[prec_tab.size()-1] ;
			prec_tab.pop_back() ;
		}

	if(!trouve)
		throw runtime_error("Unexpected error in suppressPrecedence") ;
}

void TopologicalSortUtils::topologicalSort(vector< vector<int> >& precedence_graph,
														 vector<PtrPrimitive>& primitive_tab,
														 VRenderParams& vparams)
{
	vector<PtrPrimitive> new_pr_tab ;
	vector<bool> already_visited(primitive_tab.size(),false) ;
	vector<bool> already_rendered(primitive_tab.size(),false) ;
	int nb_skews = 0 ;

	int info_cnt = primitive_tab.size()/200 + 1 ;
	int nbrendered = 0 ;

	// 1 - sorts primitives by rendering order

	for(unsigned int i=0;i<primitive_tab.size();++i)
		if(!already_rendered[i])
			recursTopologicalSort(precedence_graph,primitive_tab,already_rendered,already_visited,new_pr_tab,i,nb_skews,vparams,info_cnt,nbrendered);

#ifdef DEBUG_TS
	if(nb_skews > 0)
		cout << nb_skews << " cycles found." << endl ;
	else
		cout << "No cycles found." << endl ;
#endif
	primitive_tab = new_pr_tab ;
}

void TopologicalSortUtils::topologicalSortBreakCycles(vector< vector<int> >& precedence_graph,
																		vector<PtrPrimitive>& primitive_tab,
																		VRenderParams& vparams)
{
	vector<PtrPrimitive> new_pr_tab ;
	vector<bool> already_visited(primitive_tab.size(),false) ;
	vector<bool> already_rendered(primitive_tab.size(),false) ;
	vector<int> ancestors ;
	int nb_skews = 0 ;
	int ancestors_backward_index ;

	int info_cnt = primitive_tab.size()/200 + 1 ;
	int nbrendered = 0 ;

	// 1 - sorts primitives by rendering order

	for(unsigned int i=0;i<primitive_tab.size();++i)
		if(!already_rendered[i])
			recursTopologicalSort(precedence_graph,primitive_tab,already_rendered,already_visited,
										new_pr_tab,i,ancestors,ancestors_backward_index,nb_skews,vparams,info_cnt,nbrendered) ;

#ifdef DEBUG_TS
	if(nb_skews > 0)
		cout << nb_skews << " cycles found." << endl ;
	else
		cout << "No cycles found." << endl ;
#endif
	primitive_tab = new_pr_tab ;
}

void TopologicalSortUtils::recursTopologicalSort(	vector< vector<int> >& precedence_graph,
																	vector<PtrPrimitive>& primitive_tab,
																	vector<bool>& already_rendered,
																	vector<bool>& already_visited,
																	vector<PtrPrimitive>& new_pr_tab,
																	int indx,
																	int& nb_cycles,
																	VRenderParams& vparams,
																	int info_cnt,int& nbrendered)
{
	// One must first render the primitives indicated by the precedence graph,
	// then render the current primitive. Skews are detected, but and treated.

	already_visited[indx] = true ;

	for(unsigned int j=0;j<precedence_graph[indx].size();++j)
	{
		// Both tests are important. If we ommit the second one, the recursion is
		// always performed down to the next cycle, although this is useless if
		// the current primitive was rendered already.

		if(!already_visited[precedence_graph[indx][j]])
		{
			if(!already_rendered[precedence_graph[indx][j]])
				recursTopologicalSort(	precedence_graph,primitive_tab,already_rendered,already_visited,
												new_pr_tab,precedence_graph[indx][j],nb_cycles,vparams,info_cnt,nbrendered) ;
		}
		else //  A cycle is detected, but in this version, it is not broken.
			++nb_cycles ;
	}

	if(!already_rendered[indx])
	{
		new_pr_tab.push_back(primitive_tab[indx]) ;

		if((++nbrendered)%info_cnt==0)
			vparams.progress(nbrendered/(float)primitive_tab.size(), QGLViewer::tr("Topological sort")) ;
	}

	already_rendered[indx] = true ;
	already_visited[indx] = false ;
}

void TopologicalSortUtils::recursTopologicalSort(	vector< vector<int> >& precedence_graph,
																	vector<PtrPrimitive>& primitive_tab,
																	vector<bool>& already_rendered,
																	vector<bool>& already_visited,
																	vector<PtrPrimitive>& new_pr_tab,
																	int indx,
																	vector<int>& ancestors,
																	int& ancestors_backward_index,
																	int& nb_cycles,
																	VRenderParams& vparams,
																	int info_cnt,int& nbrendered)
{
	// One must first render the primitives indicated by the precedence graph,
	// then render the current primitive. Skews are detected, but and treated.

	already_visited[indx] = true ;
	ancestors.push_back(indx) ;

	for(unsigned int j=0;j<precedence_graph[indx].size();++j)
	{
		// Both tests are important. If we ommit the second one, the recursion is
		// always performed down to the next cycle, although this is useless if
		// the current primitive was rendered already.

		if(!already_visited[precedence_graph[indx][j]])
		{
			if(!already_rendered[precedence_graph[indx][j]])
			{
				recursTopologicalSort(	precedence_graph,primitive_tab,already_rendered,already_visited,
												new_pr_tab,precedence_graph[indx][j],ancestors,ancestors_backward_index,nb_cycles,vparams,info_cnt,nbrendered) ;

				if(ancestors_backward_index != INT_MAX && ancestors.size() > (unsigned int)(ancestors_backward_index+1))
				{
#ifdef DEBUG_TS
					cout << "Returning early" << endl ;
#endif
					already_visited[indx] = false ;
					ancestors.pop_back() ;
					return;
				}
				if(ancestors_backward_index != INT_MAX) // we are returning from emergency. j must be re-tried
					--j ;
			}
		}
		else
		{
			//  A cycle is detected. It must be broken. The algorithm is the following: primitives of the cycle
			// are successively split by a chosen splitting plane and the precendence graph is updated
			// at the same time by re-computing primitive precedence. As soon as the cycle is broken,
			// the algorithm stops and calls recursively calls on the new precedence graph. This necessarily
			// happens because of the BSP-node nature of the current set of primitives.

			// 0 - stats
			++nb_cycles ;

			// 0.5 - determine cycle beginning

			int cycle_beginning_index = -1 ;
			for(int i=ancestors.size()-1; i >= 0 && cycle_beginning_index < 0;--i)
				if(ancestors[i] == precedence_graph[indx][j])
					cycle_beginning_index = i ;
#ifdef DEBUG_TS
			cout << "Unbreaking cycle : " ;
			for(unsigned int i=0;i<ancestors.size();++i)
				cout << ancestors[i] << " " ;
			cout << precedence_graph[indx][j] << endl ;
#endif
#ifdef DEBUG_TS
			assert(cycle_beginning_index >= 0) ;
#endif
			// 1 - determine splitting plane

			int split_prim_ancestor_indx = -1 ;
			int split_prim_indx = -1 ;

			// Go down ancestors tab, starting from the skewing primitive, and stopping at it.

			for(unsigned int i2=cycle_beginning_index;i2<ancestors.size() && split_prim_ancestor_indx < 0;++i2)
				if(primitive_tab[ancestors[i2]]->nbVertices() > 2)
				{
					split_prim_ancestor_indx = i2 ;
					split_prim_indx = ancestors[i2] ;
				}

#ifdef DEBUG_TS
			cout << "Split primitive index = " << split_prim_ancestor_indx << "(primitive = " << split_prim_indx << ")" << endl ;
#endif
			if(split_prim_indx < 0)	// no need to unskew cycles between segments and points
				continue ;

			// 2 - split all necessary primitives

			const Polygone *P = dynamic_cast<const Polygone *>(primitive_tab[split_prim_indx]) ;
			const NVector3& normal = NVector3(P->normal()) ;
			double c(P->c()) ;
			ancestors.push_back(precedence_graph[indx][j]) ;				// sentinel
			ancestors.push_back(ancestors[cycle_beginning_index+1]) ;	// sentinel
			bool cycle_broken = false ;

			for(unsigned int i3=cycle_beginning_index+1;i3<ancestors.size()-1 && !cycle_broken;++i3)
				if(ancestors[i3] != split_prim_indx)
				{
					bool prim_lower_ante_contains_im1 = false ;
					bool prim_upper_ante_contains_im1 = false ;
					bool prim_lower_prec_contains_ip1 = false ;
					bool prim_upper_prec_contains_ip1 = false ;

					Primitive *prim_upper = NULL ;
					Primitive *prim_lower = NULL ;

					PrimitivePositioning::splitPrimitive(primitive_tab[ancestors[i3]],normal,c,prim_upper,prim_lower) ;

					if(prim_upper == NULL || prim_lower == NULL)
						continue ;
#ifdef DEBUG_TS
					cout << "Splitted primitive " << ancestors[i3] << endl ;
#endif

					vector<int> prim_upper_prec ;
					vector<int> prim_lower_prec ;

					vector<int> old_prec = vector<int>(precedence_graph[ancestors[i3]]) ;

					int upper_indx = precedence_graph.size() ;
					int lower_indx = ancestors[i3] ;

					//  Updates the precedence graph downwards.

					for(unsigned int k=0;k<old_prec.size();++k)
					{
						int prp1 = PrimitivePositioning::computeRelativePosition(prim_upper,primitive_tab[old_prec[k]]) ;
#ifdef DEBUG_TS
						cout << "Compariing " << upper_indx << " and " << old_prec[k] << ": " ;
#endif
						// It can not be Upper, because it was lower from the original primitive, but it is not
						// necessary lower any longer because of the split.

						if(prp1 & PrimitivePositioning::Lower)
						{
#ifdef DEBUG_TS
							cout << " > " << endl ;
#endif
							prim_upper_prec.push_back(old_prec[k]) ;

							if(old_prec[k] == ancestors[i3+1])
								prim_upper_prec_contains_ip1 = true ;
						}
#ifdef DEBUG_TS
						else
							cout << " I " << endl ;
#endif

						int prp2 = PrimitivePositioning::computeRelativePosition(prim_lower,primitive_tab[old_prec[k]]) ;
#ifdef DEBUG_TS
						cout << "Compariing " << lower_indx << " and " << old_prec[k] << ": " ;
#endif
						if(prp2 & PrimitivePositioning::Lower)
						{
#ifdef DEBUG_TS
							cout << " > " << endl ;
#endif
							prim_lower_prec.push_back(old_prec[k]) ;

							if(old_prec[k] == ancestors[i3+1])
								prim_lower_prec_contains_ip1 = true ;
						}
#ifdef DEBUG_TS
						else
							cout << " I " << endl ;
#endif
					}

					// We also have to update the primitives which are upper to the
					// current one, because some of them may not be upper anymore.
					// This would requires either a O(n^2) algorithm, or to store an
					// dual precedence graph. For now it's O(n^2). This test can not
					// be skipped because upper can still be lower to ancestors[i-1].

					for(unsigned int l=0;l<precedence_graph.size();++l)
						if(l != (unsigned int)lower_indx)
							for(int k=0;k<(int)precedence_graph[l].size();++k)
								if(precedence_graph[l][k] == ancestors[i3])
								{
									int prp1 = PrimitivePositioning::computeRelativePosition(prim_upper,primitive_tab[l]) ;

									// It can not be Lower, because it was upper from the original primitive, but it is not
									// necessary upper any longer because of the split.

									if(prp1 & PrimitivePositioning::Upper)
									{
										// Still upper. Add the new index at end of the array

										precedence_graph[l].push_back(upper_indx) ;

										if(l == (unsigned int)ancestors[i3-1])
											prim_upper_ante_contains_im1 = true ;
									}
									// If the primitive is not upper anymore there is
									// nothing to change since the index has changed.

									int prp2 = PrimitivePositioning::computeRelativePosition(prim_lower,primitive_tab[l]) ;
#ifdef DEBUG_TS
									cout << "Compariing " << l << " and " << lower_indx  << ": " ;
#endif
									if(prp2 & PrimitivePositioning::Upper)
									{
#ifdef DEBUG_TS
										cout << " > " << endl ;
#endif
										if(l == (unsigned int)ancestors[i3-1])						 // The index is the same => nothing to change.
											prim_lower_ante_contains_im1 = true ;
									}
									else
									{
#ifdef DEBUG_TS
										cout << " I " << endl ;
#endif
										// Not upper anymore. We have to suppress this entry from the tab.

										precedence_graph[l][k] = precedence_graph[l][precedence_graph[l].size()-1] ;
										precedence_graph[l].pop_back() ;
										--k ;
									}

									break ;	// each entry is present only once.
								}

					// setup recorded new info

					primitive_tab.push_back(prim_upper) ;
					delete primitive_tab[lower_indx] ;
					primitive_tab[lower_indx] = prim_lower ;

					// Adds the info to the precedence graph

					precedence_graph.push_back(prim_upper_prec) ;
					precedence_graph[lower_indx] = prim_lower_prec ;

					// Adds new entries to the 'already_rendered' and 'already_visited' vectors
					already_visited.push_back(false) ;
					already_rendered.push_back(false) ;
#ifdef DEBUG_TS
					cout << "New precedence graph: " << endl ;
					printPrecedenceGraph(precedence_graph,primitive_tab) ;
#endif
					// Checks if the cycle is broken. Because the graph is only
					// updated downwards, we check wether lower (which still is
					// lower to ancestors[i-1]) is upper to ancestors[i+1], or
					// if upper is still .

					if(( !(prim_lower_ante_contains_im1 && prim_lower_prec_contains_ip1))
					  &&(!(prim_upper_ante_contains_im1 && prim_upper_prec_contains_ip1)))
						cycle_broken = true ;
				}

			ancestors.pop_back() ;
			ancestors.pop_back() ;

			// 3 - recurs call

			if(cycle_broken)
			{
				ancestors_backward_index = cycle_beginning_index ;
#ifdef DEBUG_TS
				cout << "Cycle broken. Jumping back to rank " << ancestors_backward_index << endl ;
#endif
				already_visited[indx] = false ;
				ancestors.pop_back() ;
				return;
			}
#ifdef DEBUG_TS
			else
				cout << "Cycle could not be broken !!" << endl ;
#endif
		}
	}

	if(!already_rendered[indx])
	{
#ifdef DEBUG_TS
		cout << "Returning ok. Rendered primitive " << indx << endl ;
#endif
		new_pr_tab.push_back(primitive_tab[indx]) ;

		if((++nbrendered)%info_cnt==0)
			vparams.progress(nbrendered/(float)primitive_tab.size(), QGLViewer::tr("Advanced topological sort")) ;
	}

	already_rendered[indx] = true ;
	ancestors_backward_index = INT_MAX ;
	already_visited[indx] = false ;
	ancestors.pop_back() ;
}
} // namespace





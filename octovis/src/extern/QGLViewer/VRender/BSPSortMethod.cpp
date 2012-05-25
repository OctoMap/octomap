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
 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
*/

/****************************************************************************

 Copyright (C) 2002-2011 Gilles Debunne. All rights reserved.

 This file is part of the QGLViewer library version 2.3.17.

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

#include "VRender.h"
#include "Primitive.h"
#include "SortMethod.h"
#include "math.h" // fabs

using namespace vrender;
using namespace std;

double EGALITY_EPS 		= 0.0001;
double LINE_EGALITY_EPS = 0.0001;

typedef enum { BSP_CROSS_PLANE, BSP_UPPER, BSP_LOWER } BSPPosition;

class BSPNode;

class BSPTree
{
	public:
		BSPTree();
		~BSPTree();

		void insert(Polygone *);
		void insert(Segment *);
		void insert(Point *);

		void recursFillPrimitiveArray(vector<PtrPrimitive>&) const;
	private:
		BSPNode *_root;
		vector<Segment *> _segments;	// these are for storing segments and points when _root is null
		vector<Point *> _points;
};

void BSPSortMethod::sortPrimitives(std::vector<PtrPrimitive>& primitive_tab,VRenderParams& vparams)
{
	// 1 - build BSP using polygons only

	BSPTree tree;
	Polygone *P;

        unsigned int N = primitive_tab.size()/200 +1;
	int nbinserted = 0;

	vector<PtrPrimitive> segments_and_points;	// Store segments and points for pass 2, because polygons are deleted
																// by the insertion and can not be dynamic_casted anymore.
	for(unsigned int i=0;i<primitive_tab.size();++i,++nbinserted)
	{
		if((P = dynamic_cast<Polygone *>(primitive_tab[i])) != NULL)
			tree.insert(P);
		else
			segments_and_points.push_back(primitive_tab[i]);

		if(nbinserted%N==0)
			vparams.progress(nbinserted/(float)primitive_tab.size(), QGLViewer::tr("BSP Construction"));
	}

	// 2 - insert points and segments into the BSP

	Segment *S;
	Point *p;

	for(unsigned int j=0;j<segments_and_points.size();++j,++nbinserted)
	{
		if((S = dynamic_cast<Segment *>(segments_and_points[j])) != NULL)
			tree.insert(S);
		else if((p = dynamic_cast<Point *>(segments_and_points[j])) != NULL)
			tree.insert(p);

		if(nbinserted%N==0)
			vparams.progress(nbinserted/(float)primitive_tab.size(), QGLViewer::tr("BSP Construction"));
	}

	// 3 - refill the array with the content of the BSP

	primitive_tab.resize(0);
	tree.recursFillPrimitiveArray(primitive_tab);
}

/////////////////////////////////////////////////////////////////////////////////////////////////

class BSPNode
{
	public:
		BSPNode(Polygone *);
		~BSPNode();

		void recursFillPrimitiveArray(vector<PtrPrimitive>&) const;

		void insert(Polygone *);
		void insert(Segment *);
		void insert(Point *);

	private:
		double a,b,c,d;

		BSPNode *fils_moins;
		BSPNode *fils_plus;

		vector<Segment *> seg_plus;
		vector<Segment *> seg_moins;

		vector<Point *> pts_plus;
		vector<Point *> pts_moins;

		Polygone *polygone;

		void Classify(Polygone *, Polygone * &, Polygone * &);
		void Classify(Segment *, Segment * &, Segment * &);
		int  Classify(Point *);

		void initEquation(const Polygone *P,double & a, double & b, double & c, double & d);
};

BSPTree::BSPTree()
{
	_root = NULL;
}

BSPTree::~BSPTree()
{
	delete _root;
}

void BSPTree::insert(Point *P) 	{ if(_root == NULL) _points.push_back(P) 	; else _root->insert(P); }
void BSPTree::insert(Segment *S) { if(_root == NULL) _segments.push_back(S); else _root->insert(S); }
void BSPTree::insert(Polygone *P){ if(_root == NULL) _root = new BSPNode(P); else _root->insert(P); }

void BSPTree::recursFillPrimitiveArray(vector<PtrPrimitive>& tab) const
{
	if(_root != NULL) _root->recursFillPrimitiveArray(tab);

	for(unsigned int i=0;i<_points.size();++i) tab.push_back(_points[i]);
	for(unsigned int j=0;j<_segments.size();++j) tab.push_back(_segments[j]);
}

//----------------------------------------------------------------------------//

BSPNode::~BSPNode()
{
	delete fils_moins;
	delete fils_plus;
}

int BSPNode::Classify(Point *P)
{
  double Z = P->sommet3DColor(0).x() * a + P->sommet3DColor(0).y() * b + P->sommet3DColor(0).z() * c - d;

  if(Z > EGALITY_EPS)
    return 1;
  else
    return -1;
}

void BSPNode::Classify(Segment *S, Segment * & moins_, Segment * & plus_)
{
	double Z1 = S->sommet3DColor(0).x() * a + S->sommet3DColor(0).y() * b + S->sommet3DColor(0).z() * c - d;
	double Z2 = S->sommet3DColor(1).x() * a + S->sommet3DColor(1).y() * b + S->sommet3DColor(1).z() * c - d;

	int s1, s2;

	if(Z1 < -LINE_EGALITY_EPS)
		s1 = -1;
	else if(Z1 > EGALITY_EPS)
		s1 = 1;
	else
		s1 = 0;

	if(Z2 < -LINE_EGALITY_EPS)
		s2 = -1;
	else if(Z2 > EGALITY_EPS)
		s2 = 1;
	else
		s2 = 0;

	if(s1 == -s2)
	{
		if(s1 == 0)
		{
			moins_ = S;
			plus_  = NULL;
			return;
		}
		else
		{
			double t = fabs(Z1/(Z2 - Z1));

			if((t < 0.0)||(t > 1.0))
			{
				if(t > 1.0) t = 0.999;
				if(t < 0.0) t = 0.001;
			}

			Feedback3DColor newVertex((1-t)*S->sommet3DColor(0) + t*S->sommet3DColor(1));

			if(s1 > 0)
			{
				plus_  = new Segment(S->sommet3DColor(0), newVertex);
				moins_ = new Segment(newVertex, S->sommet3DColor(1));
			}
			else
			{
				plus_  = new Segment(newVertex, S->sommet3DColor(1));
				moins_ = new Segment(S->sommet3DColor(0), newVertex);
			}

			delete S;
			return;
		}
	}
	else if(s1 == s2)
	{
		if(s1 == -1)
		{
			moins_ = S;
			plus_ = NULL;
			return;
		}
		else
		{
			moins_ = NULL;
			plus_  = S;
			return;
		}
	}
	else if(s1 == 0)
	{
		if(s2 > 0)
		{
			moins_ = NULL;
			plus_  = S;
			return;
		}
		else
		{
			moins_ = S;
			plus_  = NULL;
			return;
		}
	}
	else if(s2 == 0)
	{
		if(s1 > 0)
		{
			moins_ = NULL;
			plus_  = S;
			return;
		}
		else
		{
			moins_ = S;
			plus_  = NULL;
			return;
		}
	}
	//else
		//printf("BSPNode::Classify: unexpected classification case !!\n");
}

void BSPNode::Classify(Polygone *P, Polygone * & moins_, Polygone * & plus_)
{
	static int Signs[100];
	static double Zvals[100];

	moins_ = NULL;
	plus_ = NULL;

	if(P == NULL)
	{
		//printf("BSPNode::Classify: Error. Null polygon.\n");
		return;
	}

	int n = P->nbVertices();

	int Smin = 1;
	int Smax = -1;

	// On classe les sommets en fonction de leur signe

	for(int i=0;i<n;i++)
	{
		double Z = P->vertex(i).x() * a + P->vertex(i).y() * b + P->vertex(i).z() * c - d;

		if(Z < -EGALITY_EPS)
			Signs[i] = -1;
		else if(Z > EGALITY_EPS)
			Signs[i] = 1;
		else
			Signs[i] = 0;

		Zvals[i] = Z;

		if(Smin > Signs[i]) Smin = Signs[i];
		if(Smax < Signs[i]) Smax = Signs[i];
	}

	// Polygone inclus dans le plan

	if((Smin == 0)&&(Smax == 0))
	{
		moins_ = P;
		plus_  = NULL;
		return;
	}

	// Polygone tout positif

	if(Smin == 1)
	{
		plus_  = P;
		moins_ = NULL;
		return;
	}

	// Polygone tout negatif

	if(Smax == -1)
	{
		plus_ = NULL;
		moins_ = P;
		return;
	}

	if((Smin == -1)&&(Smax == 0))
	{
		plus_ = NULL;
		moins_ = P;
		return;
	}

	if((Smin == 0)&&(Smax == 1))
	{
		plus_ = P;
		moins_ = NULL;
		return;
	}

	// Reste le cas Smin = -1 et Smax = 1. Il faut couper

	vector<Feedback3DColor> Ps;
	vector<Feedback3DColor> Ms;

	// On teste la coherence des signes.

	int nZero = 0;
	int nconsZero = 0;

	for(int j=0;j<n;j++)
	{
		if(Signs[j] == 0)
		{
			nZero++;

			if(Signs[(j+1)%n] == 0)
				nconsZero++;
		}
	}

	if((nZero > 2)||(nconsZero > 0))
	{
		// Ils y a des imprecisions numeriques dues au fait que le poly estpres du plan.

		moins_ = P;
		plus_  = NULL;
		return;
	}

	int dep=0;

	while(Signs[dep] == 0) dep++;

	int prev_sign = Signs[dep];

	for(int k=1;k<=n;k++)
	{
		int sign = Signs[(k+dep)%n];

		if(sign == prev_sign)
		{
			if(sign == 1)
				Ps.push_back(P->sommet3DColor(k+dep));

			if(sign == -1)
				Ms.push_back(P->sommet3DColor(k+dep));
		}
		else if(sign == -prev_sign)
		{
			//  Il faut effectuer le calcul en utilisant les memes valeurs que pour le calcul des signes,
			// sinon on risque des incoherences dues aux imprecisions numeriques.

			double Z1 = Zvals[(k+dep-1)%n];
			double Z2 = Zvals[(k+dep)%n];

			double t = fabs(Z1/(Z2 - Z1));

			if((t < 0.0)||(t > 1.0))
			{
				if(t > 1.0) t = 0.999;
				if(t < 0.0) t = 0.001;
			}

			Feedback3DColor newVertex((1-t)*P->sommet3DColor(k+dep-1) + t*P->sommet3DColor(k+dep));

			Ps.push_back(newVertex);
			Ms.push_back(newVertex);

			if(sign == 1)
				Ps.push_back(P->sommet3DColor(k+dep));

			if(sign == -1)
				Ms.push_back(P->sommet3DColor(k+dep));

			prev_sign = sign;
		} // prev_sign != 0 donc necessairement sign = 0. Le sommet tombe dans le plan
		else
		{
			Feedback3DColor newVertex = P->sommet3DColor(k+dep);

			Ps.push_back(newVertex);
			Ms.push_back(newVertex);

			prev_sign = -prev_sign;
		}
	}

	//if((Ps.size() > 100)||(Ms.size() > 100))
		//printf("BSPNode::Classify: Error. nPs = %d, nMs = %d.\n",int(Ps.size()),int(Ms.size()));

	//if(Ps.size() < 3)
		//printf("BSPNode::Classify: Error. nPs = %d.\n",int(Ps.size()));

	//if(Ms.size() < 3)
		//printf("BSPNode::Classify: Error. nMs = %d.\n",int(Ms.size()));

	// Les polygones sont convexes, car OpenGL les clip lui-meme.

	//  Si les parents ne sont pas degeneres, plus et moins ne le
	// sont pas non plus.

	plus_  = new Polygone(Ps);
	moins_ = new Polygone(Ms);

	delete  P;
}

void BSPNode::insert(Polygone *P)
{
	Polygone *side_plus = NULL, *side_moins = NULL;

	// 1 - Check on which size the polygon is, possibly split.

	Classify(P,side_moins,side_plus);

	// 2 - insert polygons

	if(side_plus != NULL) {
		if(fils_plus == NULL)
			fils_plus = new BSPNode(side_plus);
		else
			fils_plus->insert(side_plus);
	}
	
	if(side_moins != NULL) {
		if(fils_moins == NULL)
			fils_moins = new BSPNode(side_moins);
		else
			fils_moins->insert(side_moins);
	}
}

void BSPNode::recursFillPrimitiveArray(vector<PtrPrimitive>& primitive_tab) const
{
  if(fils_plus != NULL)
    fils_plus->recursFillPrimitiveArray(primitive_tab);

  for(unsigned int i=0;i<seg_plus.size();++i)
    primitive_tab.push_back(seg_plus[i]);
  for(unsigned int j=0;j<pts_plus.size();++j)
    primitive_tab.push_back(pts_plus[j]);

  if(polygone != NULL)
    primitive_tab.push_back(polygone);

  if(fils_moins != NULL)
    fils_moins->recursFillPrimitiveArray(primitive_tab);

  for(unsigned int i2=0;i2<seg_moins.size();++i2)
    primitive_tab.push_back(seg_moins[i2]);
  for(unsigned int j2=0;j2<pts_moins.size();++j2)
    primitive_tab.push_back(pts_moins[j2]);
}

void BSPNode::insert(Point *P)
{
	int res = Classify(P);

	if(res == -1) {
		if(fils_moins == NULL)
			pts_moins.push_back(P);
		else
			fils_moins->insert(P);
	}

	if(res == 1) {
		if(fils_plus == NULL)
			pts_plus.push_back(P);
		else
			fils_plus->insert(P);
	}
}

void BSPNode::insert(Segment *S)
{
	Segment *side_plus = NULL, *side_moins = NULL;

	Classify(S,side_moins,side_plus);

	if(side_plus != NULL) {
		if(fils_plus == NULL)
			seg_plus.push_back(side_plus);
		else
			fils_plus->insert(side_plus);
	}

	if(side_moins != NULL) {
		if(fils_moins == NULL)
			seg_moins.push_back(side_moins);
		else
			fils_moins->insert(side_moins);
	}
}

BSPNode::BSPNode(Polygone *P)
{
  polygone = P;

  initEquation(P,a,b,c,d);

  fils_moins = NULL;
  fils_plus  = NULL;
}

void BSPNode::initEquation(const Polygone *P,double & a, double & b, double & c, double & d)
{
	Vector3 n(0.,0.,0.);
        unsigned int j = 0;

	while((j < P->nbVertices())&& n.infNorm() <= 0.00001)
	{
		n = (P->vertex(j+2) - P->vertex(j+1))^(P->vertex(j) - P->vertex(j+1));
		j++;
	}

	if(n.infNorm() <= 0.00001)
	{
                unsigned int ind = P->nbVertices();

                for(unsigned int i=0;i<P->nbVertices();i++)
			if((P->vertex(i+1)-P->vertex(i)).infNorm() > 0.00001)
			{
				ind = i;
				i = P->nbVertices();
			}

		if(ind < P->nbVertices())	// the polygon is a true segment
		{
			if((P->vertex(ind+1).x() != P->vertex(ind).x())||(P->vertex(ind+1).y() != P->vertex(ind).y()))
			{
				n[0] = - P->vertex(ind+1).y() + P->vertex(ind).y();
				n[1] =   P->vertex(ind+1).x() - P->vertex(ind).x();
				n[2] =   0;
			}
			else
			{
				n[0] = - P->vertex(ind+1).z() + P->vertex(ind).z();
				n[1] =   0;
				n[2] =   P->vertex(ind+1).x() - P->vertex(ind).x();
			}
		}
		else				// the polygon is a point
		{
			n[0] = 1.0;
			n[1] = 0.0;
			n[2] = 0.0;
		}
	}

	double D = n.norm();

	if(n[2] < 0.0)
		n /= -D;
	else
		n /= D;

	d = n*P->vertex(0);

	a = n[0];
	b = n[1];
	c = n[2];
}


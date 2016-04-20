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

 Copyright (C) 2002-2014 Gilles Debunne. All rights reserved.

 This file is part of the QGLViewer library version 2.6.3.

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

#include "Primitive.h"
#include "AxisAlignedBox.h"
#include "PrimitivePositioning.h"
#include "math.h"
#include <algorithm>
#include "Vector2.h"

#include <algorithm>

using namespace vrender ;
using namespace std ;

#define DEBUG_TS

double PrimitivePositioning::_EPS = 0.00001 ;

// Computes relative position of the second primitive toward the first.
// As a general rule, the smaller the Z of a primitive, the Upper the primitive.

int PrimitivePositioning::computeRelativePosition(const Primitive *p1,const Primitive *p2)
{
	AxisAlignedBox_xyz bb1(p1->bbox()) ;
	AxisAlignedBox_xyz bb2(p2->bbox()) ;

	// 1 - check if bounding boxes are disjoint. In such a case, a rapid answer is possible.

	if( bb1.maxi().x() < bb2.mini().x() || bb1.mini().x() > bb2.maxi().x()) return Independent ;
	if( bb1.maxi().y() < bb2.mini().y() || bb1.mini().y() > bb2.maxi().y()) return Independent ;

	// 2 - call specific tests for each case.

	if(p1->nbVertices() >= 3)
		if(p2->nbVertices() >= 3)
			return computeRelativePosition( dynamic_cast<const Polygone *>(p1),dynamic_cast<const Polygone *>(p2)) ;
		else if(p2->nbVertices() == 2) // Case of a segment versus a polygon
			return computeRelativePosition( dynamic_cast<const Polygone *>(p1),dynamic_cast<const Segment *>(p2)) ;
		else
			return computeRelativePosition( dynamic_cast<const Polygone *>(p1),dynamic_cast<const Point *>(p2)) ;
	else if(p1->nbVertices() == 2)
		if(p2->nbVertices() >= 3)
			return inverseRP(computeRelativePosition( dynamic_cast<const Polygone *>(p2),dynamic_cast<const Segment *>(p1))) ;
		else if(p2->nbVertices() == 2)
			return computeRelativePosition( dynamic_cast<const Segment *>(p1),dynamic_cast<const Segment *>(p2)) ;
		else
			return Independent ;	// segment vs point => independent
	else
		if(p2->nbVertices() >= 3)
			return inverseRP(computeRelativePosition( dynamic_cast<const Polygone *>(p2),dynamic_cast<const Point *>(p1))) ;
		else if(p2->nbVertices() == 2)
			return Independent ;	// point vs segment => independent
		else
			return Independent ;	// point vs point => independent
}

// Computes the relative position of the point toward a *convex* polygon.

int PrimitivePositioning::computeRelativePosition(const Polygone *Q,const Point *P)
{
	if(pointOutOfPolygon_XY(P->vertex(0),Q,(double)_EPS)) // On met un eps > 0, pour que les
		return Independent ;							 // points du bords soient inclus dans le polygone.

	// now compute the relative position of the point toward the polygon

	if(Q->equation(P->vertex(0)) >= 0)
		return Upper ;
	else
		return Lower ;
}

// Computes the relative position of the segment toward a *convex* polygon.

int PrimitivePositioning::computeRelativePosition(const Polygone *P,const Segment *S)
{
	//  Computes the intersection of the segment and the polygon in 2D, then
	// project the extremities of the intersection onto the segment, and compare
	// the points to the polygon.

	// 1 - 2D-intersection of segment and polygon

	vector<double> intersections ;

	if(!pointOutOfPolygon_XY(S->vertex(0),P,_EPS)) intersections.push_back(0.0);
	if(!pointOutOfPolygon_XY(S->vertex(1),P,_EPS)) intersections.push_back(1.0);

	double t1,t2 ;

		for(size_t i=0;i<P->nbVertices();++i)
		if(intersectSegments_XY(Vector2(S->vertex(0)),Vector2(S->vertex(1)),Vector2(P->vertex(i)),Vector2(P->vertex(i+1)),_EPS,t1,t2))
			intersections.push_back(t1) ;

	// 2 - Checks wether the intersection segment is reduced to a point. In this case,
	// 	both primitives are independent.

	double tmin = FLT_MAX ;
	double tmax = -FLT_MAX ;

	for(unsigned int j=0;j<intersections.size();++j)
	{
		tmin = std::min(tmin,intersections[j]) ;
		tmax = std::max(tmax,intersections[j]) ;
	}

	if(tmax - tmin < 2*_EPS)
		return Independent ;

	// 3 - The intersection segment is not reduced to a point. Compares 3D
	//   projections of the intersections with the plane of the polygon.

	int res = Independent ;

	for(unsigned int k=0;k<intersections.size();++k)
	{
		Vector3 v( (1-intersections[k])*S->vertex(0) + intersections[k]*S->vertex(1) ) ;

		if(P->equation(v) < -_EPS) res |= Lower ;
		if(P->equation(v) >  _EPS) res |= Upper ;
	}

	if(intersections.size() > 1 && res == Independent)	// case of segments tangent to the polygon
		res = Upper ;

	return res ;
}

// Computes the relative position of a polygon toward a convex polygon.

int PrimitivePositioning::computeRelativePosition(const Polygone *P1,const Polygone *P2)
{
	// 1 - use gpc to conservatively check for intersection. This works fine because
	//    gpc produces a null intersection for polygons sharing an edge, which
	//    is exactly what we need.

	gpc_polygon gpc_int ;

	try
	{
		gpc_polygon gpc_p1 = createGPCPolygon_XY(P1) ;
		gpc_polygon gpc_p2 = createGPCPolygon_XY(P2) ;

		gpc_polygon_clip(GPC_INT,&gpc_p1,&gpc_p2,&gpc_int) ;

		gpc_free_polygon(&gpc_p1) ;
		gpc_free_polygon(&gpc_p2) ;
	}
	catch(exception&)
	{
		return Independent ;				// no free, because we don't really now what happenned.
	}

	int res = Independent ;

	if (gpc_int.num_contours != 1) // There is some numerical error in gpc. Let's skip.
	  {
		gpc_free_polygon(&gpc_int) ;
		return res ;
		// throw runtime_error("Intersection with more than 1 contour ! Non convex polygons ?") ;
	  }

	// 2 - polygons are not independent. Compute their relative position.
	//    For this, we project the vertices of the 2D intersection onto the
	//   support plane of each polygon. The epsilon-signs of each point toward
	//   both planes give the relative position of the polygons.

	for(long i=0;i<gpc_int.contour[0].num_vertices && (res < (Upper | Lower));++i)
	{
		if(P1->normal().z() == 0.0) throw runtime_error("could not project point. Unexpected case !") ;
		if(P2->normal().z() == 0.0) throw runtime_error("could not project point. Unexpected case !") ;

		// project point onto support planes

		double f1 = P1->normal().x() * gpc_int.contour[0].vertex[i].x + P1->normal().y() * gpc_int.contour[0].vertex[i].y - P1->c() ;
		double f2 = P2->normal().x() * gpc_int.contour[0].vertex[i].x + P2->normal().y() * gpc_int.contour[0].vertex[i].y - P2->c() ;

		Vector3 v1(gpc_int.contour[0].vertex[i].x,gpc_int.contour[0].vertex[i].y, -f1/P1->normal().z()) ;
		Vector3 v2(gpc_int.contour[0].vertex[i].x,gpc_int.contour[0].vertex[i].y, -f2/P2->normal().z()) ;

		if(P1->equation(v2) < -_EPS) res |= Lower ;
		if(P1->equation(v2) >  _EPS) res |= Upper ;
		if(P2->equation(v1) < -_EPS) res |= Upper ;
		if(P2->equation(v1) >  _EPS) res |= Lower ;
	}
	gpc_free_polygon(&gpc_int) ;
	return res ;
}

// Computes the relative position of a segment toward another segment.

int PrimitivePositioning::computeRelativePosition(const Segment *S1,const Segment *S2)
{
	double t1,t2 ;

	if(!intersectSegments_XY(	Vector2(S1->vertex(0)),Vector2(S1->vertex(1)),
										Vector2(S2->vertex(0)),Vector2(S2->vertex(1)),
										-(double)_EPS,t1,t2 ))
		return Independent ;
	else
	{
		double z1 = (1.0 - t1)*S1->vertex(0).z() + t1*S1->vertex(1).z() ;
		double z2 = (1.0 - t2)*S2->vertex(0).z() + t2*S2->vertex(1).z() ;

		if(z1 <= z2)
			return Lower ;
		else
			return Upper ;
	}
}


// Teste si le point est exterieur au polygone (convexe). Plus I_EPS est grand
// plus il faut etre loin pour que ca soit vrai. EPS=0 correspond au polygone
// lui-meme bords inclus. Pour EPS<0, des points interieurs pres de la frontiere sont
// declares exterieurs.  Plus I_EPS est grand, plus l'ensemble des points
// consideres comme interieur est dilate.

bool PrimitivePositioning::pointOutOfPolygon_XY(const Vector3& P,const Polygone *Q,double I_EPS)
{
	size_t nq = Q->nbVertices() ;
	Vector2 p = Vector2(P) ;

	FLOAT MaxZ = -FLT_MAX ;
	FLOAT MinZ =  FLT_MAX ;

	for(size_t j=0;j<nq;j++)  				//  Regarde si P.(x,y) est a l'interieur
	{                               	// ou a l'exterieur du polygone.
		Vector2 q1 = Vector2(Q->vertex(j)) ;
		Vector2 q2 = Vector2(Q->vertex(j+1)) ;

		double Z = (q1-p)^(q2-p) ;

		MinZ = std::min(Z,MinZ) ;
		MaxZ = std::max(Z,MaxZ) ;
	}

	if((MaxZ <= -I_EPS*I_EPS)||(MinZ >= I_EPS*I_EPS))	// the point is inside the polygon
		return false ;
	else
		return true ;
}

int PrimitivePositioning::inverseRP(int pos)
{
	// Basically switch bits of Lower and Upper

	switch(pos)
	{
		case Independent: return Independent ;
		case Lower: return Upper ;
		case Upper: return Lower ;
		case Upper | Lower: return Upper | Lower ;
		default:
								  throw runtime_error("Unexpected value.") ;
								  return pos ;
	}
}

// Calcule l'intersection des segments [P1,Q1] et [P2,Q2]
// En retour, (1-t1,t1) et (1-t2,t2) sont les coordonnees
// barycentriques de l'intersection dans chaque segment.

bool PrimitivePositioning::intersectSegments_XY(const Vector2& P1,const Vector2& Q1,
																const Vector2& P2,const Vector2& Q2,
																double I_EPS,
																double & t1,double & t2)
{
	double P1x(P1.x()) ;
	double P1y(P1.y()) ;
	double P2x(P2.x()) ;
	double P2y(P2.y()) ;
	double Q1x(Q1.x()) ;
	double Q1y(Q1.y()) ;
	double Q2x(Q2.x()) ;
	double Q2y(Q2.y()) ;

	double a2 = -(Q2y - P2y) ;
	double b2 =  (Q2x - P2x) ;
	double c2 =  P2x*a2+P2y*b2 ;

	double a1 = -(Q1y - P1y) ;
	double b1 =  (Q1x - P1x) ;
	double c1 =  P1x*a1+P1y*b1 ;

	double d2 = a2*(Q1x-P1x)+b2*(Q1y-P1y) ;
	double d1 = a1*(Q2x-P2x)+b1*(Q2y-P2y) ;

	if((fabs(d2) <= fabs(I_EPS))||(fabs(d1) <= fabs(I_EPS)))	// les segments sont paralleles
	{
		if(fabs(a2*P1x + b2*P1y - c2) >= I_EPS)
			return false ;

		double tP1,tQ1 ;

		if(P1x != Q1x)
		{
			tP1 = (P2x-P1x)/(Q1x-P1x) ;
			tQ1 = (Q2x-P1x)/(Q1x-P1x) ;
		}
		else if(P1y != Q1y)
		{
			tP1 = (P2y-P1y)/(Q1y-P1y) ;
			tQ1 = (Q2y-P1y)/(Q1y-P1y) ;
		}
		else
		{
#ifdef DEBUG_TS
			printf("IntersectSegments2D:: Error ! One segment has length 0\n") ;
			printf("This special case is not treated yet.\n") ;
#endif
			return false ;
		}

		double tPQM = std::max(tP1,tQ1) ;
		double tPQm = std::min(tP1,tQ1) ;

		if(( tPQM < -I_EPS) || (tPQm > 1.0+I_EPS))
			return false ;

		if(tPQm > 0.0)
		{
			t1 = tPQm ;
			t2 = 0.0 ;
		}
		else
		{
			t1 = 0.0 ;
			if(P2x != Q2x)
				t2 = (P1x-P2x)/(Q2x-P2x) ;
			else if(P2y != Q2y)
				t2 = (P1y-P2y)/(Q2y-P2y) ;
			else
			{
#ifdef DEBUG_TS
				printf("IntersectSegments2D:: Error ! One segment has length 0\n") ;
				printf("This special case is not treated yet.\n") ;
#endif
				return false ;
			}
		}

		return true ;
	}
	else
	{
		t2 = (c1 - a1*P2x - b1*P2y)/d1 ;
		t1 = (c2 - a2*P1x - b2*P1y)/d2 ;

		if((t2 > 1+I_EPS)||(t2 < -I_EPS)||(t1 > 1+I_EPS)||(t1 < -I_EPS))
			return false ;

		return true ;
	}
}

gpc_polygon PrimitivePositioning::createGPCPolygon_XY(const Polygone *P)
{
	gpc_polygon p ;

	p.num_contours = 0 ;
	p.hole = NULL ;
	p.contour = NULL ;

	gpc_vertex_list *gpc_p_verts = new gpc_vertex_list ;

	gpc_p_verts->num_vertices = P->nbVertices() ;
	gpc_p_verts->vertex = new gpc_vertex[P->nbVertices()] ;

		for(size_t i=0;i<P->nbVertices();++i)
	{
		gpc_p_verts->vertex[i].x = P->vertex(i).x() ;
		gpc_p_verts->vertex[i].y = P->vertex(i).y() ;
	}

	gpc_add_contour(&p,gpc_p_verts,false) ;

	return p ;
}

void PrimitivePositioning::getsigns(const Primitive *P,const NVector3& v,double C,
												vector<int>& signs,vector<double>& zvals,int& Smin,int& Smax,double I_EPS)
{
	if(P == NULL)
		throw runtime_error("Null primitive in getsigns !") ;

	size_t n = P->nbVertices() ;

	Smin =  1 ;
	Smax = -1 ;

	// On classe les sommets en fonction de leur signe

	double zmax = -FLT_MAX ;
	double zmin =  FLT_MAX ;
	zvals.resize(n) ;

	for(size_t i=0;i<n;i++)
	{
		double Z = P->vertex(i) * v - C ;

		if(Z > zmax) zmax = Z ;
		if(Z < zmin) zmin = Z ;

		zvals[i] = Z ;
	}

	signs.resize(n) ;

	for(size_t j=0;j<n;j++)
	{
		if(zvals[j] < -I_EPS)
			signs[j] = -1 ;
		else if(zvals[j] > I_EPS)
			signs[j] = 1 ;
		else
			signs[j] = 0 ;

		if(Smin > signs[j]) Smin = signs[j] ;
		if(Smax < signs[j]) Smax = signs[j] ;
	}
}

void PrimitivePositioning::split(Polygone *P,const NVector3& v,double C,Primitive *& P_plus,Primitive *& P_moins)
{
	vector<int> Signs ;
	vector<double> Zvals ;

	P_plus = NULL ;
	P_moins = NULL ;

	int Smin = 1 ;
	int Smax = -1 ;

	getsigns(P,v,C,Signs,Zvals,Smin,Smax,_EPS) ;

	size_t n = P->nbVertices() ;

	if((Smin == 0)&&(Smax == 0)){ P_moins = P ; P_plus = NULL ; return ; }	// Polygone inclus dans le plan
	if(Smin == 1) 					{ P_plus = P ; P_moins = NULL ; return ; }	// Polygone tout positif
	if(Smax == -1) 					{ P_plus = NULL ; P_moins = P ; return ; }	// Polygone tout negatif

	if((Smin == -1)&&(Smax == 0)) { P_plus = NULL ; P_moins = P ; return ; }	// Polygone tout negatif ou null
	if((Smin == 0)&&(Smax == 1))  { P_plus = P ; P_moins = NULL ; return ; }	// Polygone tout positif ou null

	// Reste le cas Smin = -1 et Smax = 1. Il faut couper

	vector<Feedback3DColor> Ps ;
	vector<Feedback3DColor> Ms ;

	// On teste la coherence des signes.

	int nZero = 0 ;
	int nconsZero = 0 ;

	for(size_t i=0;i<n;i++)
	{
		if(Signs[i] == 0)
		{
			nZero++ ;

			if(Signs[(i+1)%n] == 0)
				nconsZero++ ;
		}
	}

	// Ils y a des imprecisions numeriques dues au fait que le poly estpres du plan.
	if((nZero > 2)||(nconsZero > 0)) { P_moins = P ; P_plus  = NULL ; return ; }

	int dep=0 ; while(Signs[dep] == 0) dep++ ;
	int prev_sign = Signs[dep] ;

	for(size_t j=1;j<=n;j++)
	{
		int sign = Signs[(j+dep)%n] ;

		if(sign == prev_sign)
		{
			if(sign ==  1) Ps.push_back(P->sommet3DColor(j+dep)) ;
			if(sign == -1) Ms.push_back(P->sommet3DColor(j+dep)) ;
		}
		else if(sign == -prev_sign)
		{
			//  Il faut effectuer le calcul en utilisant les memes valeurs que pour le calcul des signes,
			// sinon on risque des incoherences dues aux imprecisions numeriques.

			double Z1 = Zvals[(j+dep-1)%n] ;
			double Z2 = Zvals[(j+dep)%n] ;

			double t = fabs(Z1/(Z2 - Z1)) ;

			if((t < 0.0)||(t > 1.0))
			{
				if(t > 1.0) t = 1.0 ;
				if(t < 0.0) t = 0.0 ;
			}
			Feedback3DColor newVertex((1-t)*P->sommet3DColor(j+dep-1) + t*P->sommet3DColor(j+dep)) ;

			Ps.push_back(newVertex) ;
			Ms.push_back(newVertex) ;

			if(sign == 1)
				Ps.push_back(P->sommet3DColor(j+dep)) ;

			if(sign == -1)
				Ms.push_back(P->sommet3DColor(j+dep)) ;

			prev_sign = sign ;
		} // prev_sign != 0 donc necessairement sign = 0. Le sommet tombe dans le plan
		else
		{
			Feedback3DColor newVertex = P->sommet3DColor(j+dep) ;

			Ps.push_back(newVertex) ;
			Ms.push_back(newVertex) ;

			prev_sign = -prev_sign ;
		}
	}

	if(Ps.size() > 100 || Ms.size() > 100 )
		printf("Primitive::split: Error. nPs = %d, nMs = %d.\n",int(Ps.size()),int(Ms.size())) ;

	// on suppose pour l'instant que les polygones sont convexes

	if(Ps.size() == 1)
		P_plus = new Point(Ps[0]) ;
	else if(Ps.size() == 2)
		P_plus = new Segment(Ps[0],Ps[1]) ;
	else
		P_plus  = new Polygone(Ps) ;

	if(Ms.size() == 1)
		P_moins = new Point(Ms[0]) ;
	else if(Ms.size() == 2)
		P_moins = new Segment(Ms[0],Ms[1]) ;
	else
		P_moins = new Polygone(Ms) ;
}

void PrimitivePositioning::split(Point *P,const NVector3& v,double C,Primitive * & P_plus,Primitive * & P_moins)
{
	if(v*P->vertex(0)-C > -_EPS)
	{
		P_plus = P ;
		P_moins = NULL ;
	}
	else
	{
		P_moins = P ;
		P_plus = NULL ;
	}
}

void PrimitivePositioning::split(Segment *S,const NVector3& v,double C,Primitive * & P_plus,Primitive * & P_moins)
{
	vector<int> Signs ;
	vector<double> Zvals ;

	P_plus = NULL ;
	P_moins = NULL ;

	int Smin = 1 ;
	int Smax = -1 ;

	getsigns(S,v,C,Signs,Zvals,Smin,Smax,_EPS) ;

	size_t n = S->nbVertices() ;

	if((Smin == 0)&&(Smax == 0)) 	{ P_moins = S ; P_plus = NULL ; return ; }	// Polygone inclus dans le plan
	if(Smin == 1) 						{ P_plus = S ; P_moins = NULL ; return ; }	// Polygone tout positif
	if(Smax == -1) 					{ P_plus = NULL ; P_moins = S ; return ; }	// Polygone tout negatif

	if((Smin == -1)&&(Smax == 0)) { P_plus = NULL ; P_moins = S ; return ; }	// Polygone tout negatif ou null
	if((Smin == 0)&&(Smax == 1))  { P_plus = S ; P_moins = NULL ; return ; }	// Polygone tout positif ou null

	// Reste le cas Smin = -1 et Smax = 1. Il faut couper
	// On teste la coherence des signes.

	int nZero = 0 ;
	int nconsZero = 0 ;

	for(size_t i=0;i<n;i++)
	{
		if(Signs[i] == 0)
		{
			nZero++ ;

			if(Signs[(i+1)%n] == 0)
				nconsZero++ ;
		}
	}

	// Ils y a des imprecisions numeriques dues au fait que le poly estpres du plan.
	if((nZero > 2)||(nconsZero > 0)) { P_moins = S ; P_plus  = NULL ; return ; }

	double Z1 = Zvals[0] ;
	double Z2 = Zvals[1] ;

	double t = fabs(Z1/(Z2 - Z1)) ;

	if((t < 0.0)||(t > 1.0))
	{
		if(t > 1.0) t = 1.0 ;
		if(t < 0.0) t = 0.0 ;
	}

	Feedback3DColor newVertex = S->sommet3DColor(0) * (1-t) + S->sommet3DColor(1) * t ;

	if(Signs[0] < 0)
	{
		P_plus = new Segment(newVertex,S->sommet3DColor(1)) ;
		P_moins = new Segment(S->sommet3DColor(0),newVertex) ;
	}
	else
	{
		P_plus = new Segment(S->sommet3DColor(0),newVertex) ;
		P_moins = new Segment(newVertex,S->sommet3DColor(1)) ;
	}
}

// splits primitive P by plane of equation v.X=c. The upper part is setup in a new primitive called prim_up and
// the lower part is in prim_lo.

void PrimitivePositioning::splitPrimitive(Primitive *P,const NVector3& v,double c, Primitive *& prim_up,Primitive *& prim_lo)
{
	Polygone *p1 = dynamic_cast<Polygone *>(P) ; if(p1 != NULL) PrimitivePositioning::split(p1,v,c,prim_up,prim_lo) ;
	Segment  *p2 = dynamic_cast<Segment  *>(P) ; if(p2 != NULL) PrimitivePositioning::split(p2,v,c,prim_up,prim_lo) ;
	Point    *p3 = dynamic_cast<Point    *>(P) ; if(p3 != NULL) PrimitivePositioning::split(p3,v,c,prim_up,prim_lo) ;
}


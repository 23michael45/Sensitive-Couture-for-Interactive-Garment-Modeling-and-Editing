/*
DelFEM (Finite Element Analysis)
Copyright (C) 2009  Nobuyuki Umetani    n.umetani@gmail.com

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

/*! @file
@brief Interfaces define the geometry of 2d cad elements
@author Nobuyuki Umetani
*/

#if !defined(CAD_ELEM_2D_H)
#define CAD_ELEM_2D_H

#if defined(__VISUALC__)
#pragma warning( disable : 4786 )
#endif

#include <vector>
#include <assert.h>
#include <iostream> // needed only in debug

#include "delfem/vector2d.h"

////////////////////////////////////////////////////////////////

namespace Cad {

	/*!
	@addtogroup CAD
	*/
	//!@{

	//! 2dim loop class
	class CLoop2D {
	public:
		CLoop2D(const CLoop2D& rhs) {
			m_color[0] = rhs.m_color[0];  m_color[1] = rhs.m_color[1];  m_color[2] = rhs.m_color[2];
			ilayer = rhs.ilayer;
		}
		CLoop2D() {
			m_color[0] = 0.8; m_color[1] = 0.8; m_color[2] = 0.8;
			ilayer = 0;
		}
	public:
		double m_color[3];
		unsigned int ilayer;
	};


	double GetDist_LineSeg_Point(const Com::CVector2D& po_c,
		const Com::CVector2D& po_s, const Com::CVector2D& po_e);

	double GetDist_LineSeg_LineSeg(const Com::CVector2D& po_s0, const Com::CVector2D& po_e0,
		const Com::CVector2D& po_s1, const Com::CVector2D& po_e1);



	// line-line intersection detection
	bool IsCross_LineSeg_LineSeg(const Com::CVector2D& po_s0, const Com::CVector2D& po_e0,
		const Com::CVector2D& po_s1, const Com::CVector2D& po_e1);

	//! circle-circle interseciton detection
	bool IsCross_Circle_Circle(const Com::CVector2D& po_c0, double radius0,
		const Com::CVector2D& po_c1, double radius1,
		Com::CVector2D& po0, Com::CVector2D& po1);
	/*!
	 @brief 墌屖偲捈慄偺岎揰傪媮傔傞
	 岎揰偑偁傞応崌偼俀偮偺岎揰偺pos偐傜poe傊偺僷儔儊乕僞偑t1,t2偵擖傞丏
	 @retval true 岎揰偑偁傞応崌
	 @retval false 岎揰偑柍偄応崌
	 */
	bool IsCross_Line_Circle(const Com::CVector2D& po_c, const double radius,
		const Com::CVector2D& po_s, const Com::CVector2D& po_e, double& t0, double& t1);
	//! 揰偲捈慄偺堦斣嬤偄揰傪扵偡
	double FindNearestPointParameter_Line_Point(const Com::CVector2D& po_c,
		const Com::CVector2D& po_s, const Com::CVector2D& po_e);

	Com::CVector2D GetProjectedPointOnCircle(const Com::CVector2D& c, double r,
		const Com::CVector2D& v);


	//! 2dim edge
	class CEdge2D {
	public:
		CEdge2D(const CEdge2D& rhs) :
			itype(rhs.itype),
			is_left_side(rhs.is_left_side), dist(rhs.dist),
			aRelCoMesh(rhs.aRelCoMesh),
			id_v_s(rhs.id_v_s), id_v_e(rhs.id_v_e), po_s(rhs.po_s), po_e(rhs.po_e) {}
		CEdge2D() : id_v_s(0), id_v_e(0), itype(0) {}
		CEdge2D(unsigned int id_v_s, unsigned int id_v_e) : id_v_s(id_v_s), id_v_e(id_v_e), itype(0) {}
		/*	CEdge2D(const int id_v_s, const int id_v_e,
				  const int itype,
				  const bool is_left_side, const double dist)
		  :	itype(itype),
		  is_left_side(is_left_side), dist(dist),
		  id_v_s(id_v_s), id_v_e(id_v_e)
			{
				po_s = Com::CVector2D(0,0);
				po_e = Com::CVector2D(0,0);
			}  ///////////
		 */
		double Distance(const CEdge2D& e1) const;	// distance between me and e1

		///////////

		/*!
		 @brief get bounding box of edge
		 @remarks make sure the value is set in po_s, po_e
		 */
		const Com::CBoundingBox2D& GetBoundingBox() const {
			if (bb_.isnt_empty) { return bb_; }
			double xmin, xmax, ymin, ymax;
			this->GetBoundingBox(xmin, xmax, ymin, ymax);
			bb_ = Com::CBoundingBox2D(xmin, xmax, ymin, ymax);
			return bb_;
		}

		bool IsCrossEdgeSelf() const;	// check self intersection
		bool IsCrossEdge(const CEdge2D& e1) const;	// intersection between me and e1
		//! 堦抂偑嫟桳偝傟偨曈摨巑偺岎嵎敾掕
		bool IsCrossEdge_ShareOnePoint(const CEdge2D& e1, bool is_share_s0, bool is_share_s1) const;
		//! 椉抂偑嫟桳偝傟偨曈摨巑偺岎嵎敾掕
		bool IsCrossEdge_ShareBothPoints(const CEdge2D& e1, bool is_share_s1s0) const;

		/*！
		@ brief计算连接曲线和边的两个顶点的直线所包围的区域（如果它在直线的右侧，则为 + ）
		@remarks用于获取循环的区域
		*/
		double AreaEdge() const;

		//！计算边缘起点/终点的切线
		Com::CVector2D GetTangentEdge(bool is_s) const;
		//！返回输入点最近侧的点和距离
		Com::CVector2D GetNearestPoint(const Com::CVector2D& po_in) const;

		// get number of intersection between half line direction (=dir) from point (=org)
		// this function is used for in-out detection
		int NumIntersect_AgainstHalfLine(const Com::CVector2D& org, const Com::CVector2D& dir) const;
		bool GetNearestIntersectionPoint_AgainstHalfLine(Com::CVector2D& sec, const Com::CVector2D& org, const Com::CVector2D& dir) const;
		bool GetCurve_Mesh(std::vector<Com::CVector2D>& aCo, int ndiv) const;
		double GetCurveLength() const;

		////////////////////////////////

		/*！
		@ brief当圆是圆弧时，计算圆的中心和半径
		@remarks如果它不是弧，则返回false
		*/
		bool GetCenterRadius(Com::CVector2D& po_c, double& radius) const;
		bool GetCenterRadiusThetaLXY(Com::CVector2D& pc, double& radius,
			double& theta, Com::CVector2D& lx, Com::CVector2D& ly) const;


		////////////////////////////////


		// 尰嵼偺曈偑俀偮偵暘妱偝傟偰丆堦抂偑edge_a偵擖傞
		bool Split(Cad::CEdge2D& edge_a, const Com::CVector2D& pa);
		// is_add_ahead偼e1偑偙偺曈偺慜偵偁傞偐丆is_same_dir偼e1偑偙偺曈偲摨偠岦偒偐
		bool ConnectEdge(const Cad::CEdge2D& e1, bool is_add_ahead, bool is_same_dir);

		// get vertex on edge with distance (len) from point v0 along the edge
		// is_front==true:same direction is_front==false:opposite direciton
		bool GetPointOnCurve_OnCircle(const Com::CVector2D& v0, double len, bool is_front,
			bool& is_exceed, Com::CVector2D& out) const;

	private:
		void GetBoundingBox(double& x_min, double& x_max, double& y_min, double& y_max) const;
		//! 慄暘偲墌屖偺岎嶖傪敾掕偡傞
		int NumCross_Arc_LineSeg(const Com::CVector2D& po_s1, const Com::CVector2D& po_e1) const;
		//! 尫偲屖偱挘傜傟傞椞堟撪晹偵揰po偑擖偭偰偄傞偐傪挷傋傞
		int IsInsideArcSegment(const Com::CVector2D& po) const;
		//! 墌屖偺拞怱偐傜傒偰丆揰po偲墌屖偑摨偠曽岦偵廳側偭偰偄傞偐丠
		int IsDirectionArc(const Com::CVector2D& po) const;
	public:
		unsigned int itype;		//!< 0:Line, 1:Arc, 2:Mesh
		  // type Arc
		bool is_left_side;      //!< is the arc is formed left side of the line po_s, po_e
		double dist;            //!< 慄暘偲墌偺拞怱偺嫍棧
		  // type Mesh
		std::vector<double> aRelCoMesh;	//!< 儊僢僔儏偺愡揰偺曈偵懳偡傞憡懳嵗昗(曈偺嵍懁偵偁偭偨傜倷幉亄)
	public:
		//! 姳徛僠僃僢僋偺帪偵偩偗堦帪揑偵曈偺俀捀揰偺嵗昗偑戙擖偝傟傞
		mutable unsigned int id_v_s, id_v_e;	//!< start vertex
		mutable Com::CVector2D po_s, po_e;
		mutable Com::CBoundingBox2D bb_;
	};

	//! 俀師尦婔壗捀揰僋儔僗
	class CVertex2D {
	public:
		CVertex2D(const Com::CVector2D& point) : point(point) {}
		CVertex2D(const CVertex2D& rhs)
			: point(rhs.point) {}
	public:
		Com::CVector2D point;   //!< coordinate
	};

	/*!
	姳徛僠僃僢僋傪峴偆
	偦偺偆偪岎嶖埵抲偺忣曬傕曉偟偨偄
	*/
	int CheckEdgeIntersection(const std::vector<CEdge2D>& aEdge);

	//! @}
}

#endif

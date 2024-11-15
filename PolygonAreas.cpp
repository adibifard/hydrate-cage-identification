

#include "Planes.h"

double Planes::PolygonAreas::AreaQuad(const double L)
{
	double A = pow(L, 2);
	return A;
}

double Planes::PolygonAreas::AreaPentagon(const double L)
{
	double A = pow(L, 2) * sqrt(5 * (5 + 2 * sqrt(2))) / 4;
	return A;
}

double Planes::PolygonAreas::AreaHex(const double L)
{
	double A = pow(L, 2) * 3 * sqrt(3) / 2;
	return A;
}



Planes::PolygonAreas::VecDouble Planes::PolygonAreas::CalcAreaPolygons(const VecOfVec Planes, std::vector<HBondedWaters::Molecule> Water)
{
	VecDouble PlaneAreas;

	for (int i = 0; i < Planes.size(); i++)
	{
		switch (Planes[i].size()) {
		case 4: 
		{
			double Lq = (Water[Planes[i][0]].ro - Water[Planes[i][1]].ro).norm();
			double Aq = AreaQuad(Lq);
			PlaneAreas.push_back(Aq);
		}
			

		case 5: 
		{
			double Lp = (Water[Planes[i][0]].ro - Water[Planes[i][1]].ro).norm();
			double Ap = AreaQuad(Lp);
			PlaneAreas.push_back(Ap);
		}
			

		case 6: 
		{
			double Lh = (Water[Planes[i][0]].ro - Water[Planes[i][1]].ro).norm();
			double Ah = AreaQuad(Lh);
			PlaneAreas.push_back(Ah);
		}
			
		}
	}
	return PlaneAreas;
}
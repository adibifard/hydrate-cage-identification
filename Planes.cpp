
#include "Planes.h"


// Construct the methods declared in the .h file
//typedef  std::vector<std::vector<int>> VecOfVec;
// Construct the method to detect the quadrilateral polygons
Planes::VecOfVec Planes::FindQuadrilateral(size_t numWaters, const VecOfVec HbondNeigh)
{
	VecOfVec FourSidedPlanes;
	for (int i = 0; i < numWaters; i++)
	{
		int p1 = i;
		for (int j = 0; j < HbondNeigh[p1].size(); j++)
		{
			int p2 = HbondNeigh[p1][j];
			if (p2 != p1)
			{
				for (int k = 0; k < HbondNeigh[p2].size(); k++)
				{
					int p3 = HbondNeigh[p2][k];
					if (p3 != p2 && p3 != p1)
					{
						for (int l = 0; l < HbondNeigh[p3].size(); l++)
						{
							int p4 = HbondNeigh[p3][l];
							if (p4 != p3 && p4 != p2 && p4 != p1)
							{
								for (int m = 0; m < HbondNeigh[p4].size(); m++)
								{
									int p5 = HbondNeigh[p4][m];
									if (p5 == p1)
									{
										// This is a quadrilateral
										std::vector<int> tmpQuadrilateral = { p1, p2, p3, p4 };
										std::sort(begin(tmpQuadrilateral), end(tmpQuadrilateral));
										// Check if the identified polygon has been found before
										if (!isInVecofVec(FourSidedPlanes, tmpQuadrilateral))
										{
											FourSidedPlanes.push_back(tmpQuadrilateral);
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}
	return FourSidedPlanes;
}

// Construct the method to detect the pentagon polygons
Planes::VecOfVec Planes::FindPentagons(size_t numWaters, const VecOfVec HbondNeigh)
{
	VecOfVec FiveSidedPlanes;
	for (int i = 0; i < numWaters; i++)
	{
		int p1 = i;
		for (int j = 0; j < HbondNeigh[p1].size(); j++)
		{
			int p2 = HbondNeigh[p1][j];
			if (p2 != p1)
			{
				for (int k = 0; k < HbondNeigh[p2].size(); k++)
				{
					int p3 = HbondNeigh[p2][k];
					if (p3 != p2 && p3 != p1)
					{
						for (int l = 0; l < HbondNeigh[p3].size(); l++)
						{
							int p4 = HbondNeigh[p3][l];
							if (p4 != p3 && p4 != p2 && p4 != p1)
							{
								for (int m = 0; m < HbondNeigh[p4].size(); m++)
								{
									int p5 = HbondNeigh[p4][m];
									if (p5 != p4 && p5 != p3 && p5 != p2 && p5 != p1)
									{
										for (int n = 0; n < HbondNeigh[p5].size(); n++)
										{
											int p6 = HbondNeigh[p5][n];
											if (p6 == p1)
											{
												// This is a Pentagon
												std::vector<int> tmpPentagon = { p1, p2, p3, p4,p5 };
												std::sort(begin(tmpPentagon), end(tmpPentagon));
												// Check if the identified polygon has been found before
												if (!isInVecofVec(FiveSidedPlanes, tmpPentagon))
												{
													FiveSidedPlanes.push_back(tmpPentagon);
												}
											}
										}

									}
								}
							}
						}
					}
				}
			}
		}
	}
	return FiveSidedPlanes;
}

// A method to identify the hexagons formed by the molecules
Planes::VecOfVec Planes::FindHexagonals(size_t numWaters, const VecOfVec HbondNeigh)
{
	VecOfVec SixSidedPlanes;
	for (int i = 0; i < numWaters; i++)
	{
		int p1 = i;
		for (int j = 0; j < HbondNeigh[p1].size(); j++)
		{
			int p2 = HbondNeigh[p1][j];
			if (p2 != p1)
			{
				for (int k = 0; k < HbondNeigh[p2].size(); k++)
				{
					int p3 = HbondNeigh[p2][k];
					if (p3 != p2 && p3 != p1)
					{
						for (int l = 0; l < HbondNeigh[p3].size(); l++)
						{
							int p4 = HbondNeigh[p3][l];
							if (p4 != p3 && p4 != p2 && p4 != p1)
							{
								for (int m = 0; m < HbondNeigh[p4].size(); m++)
								{
									int p5 = HbondNeigh[p4][m];
									if (p5 != p4 && p5 != p3 && p5 != p2 && p5 != p1)
									{
										for (int n = 0; n < HbondNeigh[p5].size(); n++)
										{
											int p6 = HbondNeigh[p5][n];
											if (p6 != p5 && p6 != p4 && p6 != p3 && p6 != p2 && p6 != p1)
											{
												for (int o = 0; o < HbondNeigh[p6].size(); o++)
												{
													int p7 = HbondNeigh[p6][o];
													if (p7 == p1)
													{
														// This is a Hexagon
														std::vector<int> tmpHexagon = { p1, p2, p3, p4,p5,p6 };
														std::sort(begin(tmpHexagon), end(tmpHexagon));
														// Check if the identified polygon has been found before
														if (!isInVecofVec(SixSidedPlanes, tmpHexagon))
														{
															SixSidedPlanes.push_back(tmpHexagon);
														}
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}
	return SixSidedPlanes;
}

// A method to append all different planes into one single list
Planes::VecOfVec Planes::CollectAllPlanes(VecOfVec quad, VecOfVec pentagon, VecOfVec hexagon, std::vector<int>& AllPlaneType)
{
	VecOfVec AllPlanes;
	std::vector<std::vector<int>>::iterator itr;
	for (itr = quad.begin(); itr != quad.end(); itr++)
	{
		AllPlanes.push_back(*itr);
		AllPlaneType.push_back(4);
	}

	for (itr = pentagon.begin(); itr != pentagon.end(); itr++)
	{
		AllPlanes.push_back(*itr);
		AllPlaneType.push_back(5);
	}

	for (itr = hexagon.begin(); itr != hexagon.end(); itr++)
	{
		AllPlanes.push_back(*itr);
		AllPlaneType.push_back(6);
	}
	return AllPlanes;
}

// A method to find the edge-sharing planes for each plane in the AllPlane list
Planes::VecOfVec Planes::FindSharingEdgePlanes(const VecOfVec AllPlanes)
{
	VecOfVec sharedPlanes;
	std::unordered_map<int, int> sharedPlanesMap;
	VecOfUnorMap VecsharedPlanesMap(AllPlanes.size());
	for (int i = 0; i < AllPlanes.size(); i++)
	{
		std::vector<int> tmpShareList;
		for (int j = 0; j < AllPlanes.size(); j++)
		{
			if (i != j)
			{
				// find the number of shared vertices between the planes
				std::vector<int> VerticesOverlap(std::min(AllPlanes[i].size(), AllPlanes[j].size()));
				auto it=std::set_intersection(AllPlanes[i].begin(), AllPlanes[i].end(), AllPlanes[j].begin(), AllPlanes[j].end(), VerticesOverlap.begin());
				VerticesOverlap.resize(it - VerticesOverlap.begin());
				//std::vector<int> SharedVertices = NumSharedElements(AllPlanes[i], AllPlanes[j]);
				if (VerticesOverlap.size() == 2)
				{
					// The polygons share an edge
					tmpShareList.push_back(j);
					std::pair<int, int> PlanePairs(j,j);
					VecsharedPlanesMap[i].insert(PlanePairs);
					//sharedPlanesMap.insert(PlanePairs);
				}
			}
		}
		sharedPlanes.push_back(tmpShareList);
	}
	return sharedPlanes;
	//return VecsharedPlanesMap;
}

// A method to filter the regular planes
Planes::VecOfVec Planes::CheckPlaneConstraints::CheckRegularPlaneConstraint(VecOfVec Planes, std::vector<HBondedWaters::Molecule> Waters, const double Lth)
{
	// Check for congruent edges
	// Length threshold in nm
	// Iterate over the planes in the list
	VecOfVec PlanesRegular;
	VecOfVec::iterator PlnItr;
	for (PlnItr = Planes.begin(); PlnItr != Planes.end(); PlnItr++)
	{
		std::vector<double> L;
		for (int i = 0; i < PlnItr->size(); i++)
		{
			double dist;
			if (i == PlnItr->size() - 1)
			{
				dist = (Waters[(*PlnItr)[i]].ro - Waters[(*PlnItr)[0]].ro).norm();
			}
			else
			{
				dist = (Waters[(*PlnItr)[i]].ro - Waters[(*PlnItr)[i + 1]].ro).norm();
			}
			L.push_back(dist);
		}
		// Check if all the edges are close enough by a t
		double RefEdgeLength = L[0];
		bool isRegular=true;

		for (double EdgeLength : L) 
		{
			if (abs(EdgeLength-RefEdgeLength)> Lth)
			{
				isRegular = false;
				break;
			}
		}

		if (isRegular)
		{
			PlanesRegular.push_back(*PlnItr);
		}
	}

	return PlanesRegular;
}

// A method to fliter out the the planes that do not satisfy the coplanarity requirement (Not Completed Yet)
Planes::VecOfVec  Planes::CheckPlaneConstraints::CheckCoplanarityConstraint(VecOfVec Planes, std::vector<HBondedWaters::Molecule> Waters, const double Coplanaritory_thr)
{
	VecOfVec Planes_Coplanar;
	for (int i = 0; i < Planes.size(); i++) 
	{
		// Pick a reference point for a plane
		int RefWater = Planes[i][0];
		std::vector<vector<double>> dist; // distance vector wrt the reference point
		for (int j = 1; j < Planes[i].size(); j++) 
		{
			dist.push_back(Waters[Planes[i][j]].ro-Waters[RefWater].ro);
		}
		std::vector<double> CoplanarMet;

		vector<double> v1v2Cross = dist[0] * dist[1]; // cross-product

		for (int k = 2; k < dist.size(); k++) 
		{
			//double a = abs(v1v2Cross.DotProduct(dist[k]));
			CoplanarMet.push_back(abs(v1v2Cross.DotProduct(dist[k])));
		}

		if (*max_element(CoplanarMet.begin(),CoplanarMet.end())< Coplanaritory_thr) // only if the plamne is coplanar
		{
			Planes_Coplanar.push_back(Planes[i]);
		}
	}
	return Planes_Coplanar;
}

// A method to determine the centroid of a plane
std::vector<vector<double>> Planes::PlaneCentroid(VecOfVec AllPlanes, std::vector<HBondedWaters::Molecule> Water)
{
	std::vector<vector<double>> PlanesCentroids;

	for (int i = 0; i < AllPlanes.size(); i++) 
	{
		vector<double> s = { 0,0,0 };
		std::vector<int>::iterator itr;
		for (int j = 0; j < AllPlanes[i].size(); j++)
		{
			s = s + Water[AllPlanes[i][j]].ro;
		}
		vector<double> centroid = s / AllPlanes[i].size();

		PlanesCentroids.push_back(centroid);
	}
	return PlanesCentroids;
	
}

// A method to find the neighborng planes of a plane restricted by a threshold radius
//Planes::VecOfVec Planes::FindNeighboringPlanes(VecOfVec Planes, std::vector<HBondedWaters::Molecule> Waters, double Rth)
//{
//	VecOfVec NeighboringPlanes;
//
//	for (unsigned int i = 0; i < Planes.size(); i++)
//	{
//		vector<double> CentroidOfPlaneMain = Planes::PlaneCentroid(Planes[i], Waters);
//		std::vector<int> tmpList;
//		for (unsigned int j = 0; j < Planes.size(); j++)
//		{
//			if (i != j)
//			{
//				vector<double> CentroidOfPlaneJ = Planes::PlaneCentroid(Planes[j], Waters);
//				double r = (CentroidOfPlaneMain - CentroidOfPlaneJ).norm();
//				if (r <= Rth)
//				{
//					tmpList.push_back(j);
//				}
//			}
//		}
//		NeighboringPlanes.push_back(tmpList);
//	}
//	return NeighboringPlanes;
//}




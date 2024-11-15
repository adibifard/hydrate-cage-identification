#include "AlgCage.h"

int  CageDetectionAlgorithms::CageIdentViaCup::CheckMutualConnection(std::vector<int> plane, VecOfVec SharedPlanes)
{
	// the 1st plane is always the bottom of the cup
	std::vector<int> sortedPlane = plane;
	std::sort(sortedPlane.begin(), sortedPlane.end());

	int NumPlaneMutual = 0;
	for (int i = 0; i < plane.size(); i++)
	{
		std::vector<int> PlaneOverlap(std::min(SharedPlanes[plane[i]].size(), sortedPlane.size()));
		auto it = std::set_intersection(SharedPlanes[plane[i]].begin(), SharedPlanes[plane[i]].end(), sortedPlane.begin(), sortedPlane.end(), PlaneOverlap.begin());
		PlaneOverlap.resize(it - PlaneOverlap.begin());

		if (PlaneOverlap.size() == 2)
		{
			NumPlaneMutual += 1;
		}

	}
	return NumPlaneMutual;
}


int CageDetectionAlgorithms::CageIdentViaCup::CheckCupOverlap5_12(std::vector<int> cup1, std::vector<int> cup2, VecOfVec SharedPlanes)
{
	int NumPlanesSatisfied = 0;
	std::vector<int> sortedCup = cup2;
	//std::sort(sortedCup.begin(), sortedCup.end());
	for (int i = 0; i < cup1.size(); i++)
	{
		std::vector<int> PlaneOverlap(std::min(SharedPlanes[cup1[i]].size(), sortedCup.size()));
		auto it = std::set_intersection(SharedPlanes[cup1[i]].begin(), SharedPlanes[cup1[i]].end(), sortedCup.begin(), sortedCup.end(), PlaneOverlap.begin());
		PlaneOverlap.resize(it - PlaneOverlap.begin());
		if (PlaneOverlap.size() == 2)
		{
			NumPlanesSatisfied += 1;
		}
	}
	return NumPlanesSatisfied;
}


std::vector<CageDetectionAlgorithms::Cage> CageDetectionAlgorithms::CageIdentViaCup::cup5_6(VecOfVec AllPlanes, VecOfVec SharedPlanes)
{
	std::vector<CageDetectionAlgorithms::Cage> Cup5_6;
	CageDetectionAlgorithms::Cage TmpCup;
	TmpCup.label = "56";
	for (int i = 0; i < AllPlanes.size(); i++)
	{
		int p1 = i;
		if (AllPlanes[p1].size() == 5 && SharedPlanes[p1].size() >= AllPlanes[p1].size()) // p1 is lateral
		{
			// Look into the shared plane list of p1W
			for (int j = 0; j < SharedPlanes[p1].size(); j++)
			{
				int p2 = SharedPlanes[p1][j];
				if (AllPlanes[p2].size() == 5 && SharedPlanes[p2].size() >= AllPlanes[p2].size())
				{
					// Look into the shared planes of p2
					for (int k = 0; k < SharedPlanes[p2].size(); k++)
					{
						int p3 = SharedPlanes[p2][k];
						if (AllPlanes[p3].size() == 5 && p3 != p1 && SharedPlanes[p3].size() >= AllPlanes[p3].size() && std::find(begin(SharedPlanes[p1]), end(SharedPlanes[p1]), p3) != SharedPlanes[p1].end())
						{
							// Look into the shared planes of p3
							for (int l = 0; l < SharedPlanes[p3].size(); l++)
							{
								int p4 = SharedPlanes[p3][l];
								if (AllPlanes[p4].size() == 5 && p4 != p2 && p4 != p1 && SharedPlanes[p4].size() >= AllPlanes[p4].size() && std::find(begin(SharedPlanes[p1]), end(SharedPlanes[p1]), p4) != SharedPlanes[p1].end())
								{
									// Look into the shared planes of p4
									for (int m = 0; m < SharedPlanes[p4].size(); m++)
									{
										int p5 = SharedPlanes[p4][m];
										if (AllPlanes[p5].size() == 5 && p5 != p3 && p5 != p2 && p5 != p1 && SharedPlanes[p5].size() >= AllPlanes[p5].size() && std::find(begin(SharedPlanes[p1]), end(SharedPlanes[p1]), p5) != SharedPlanes[p1].end())
										{
											// Look into the shared planes of p5
											for (int n = 0; n < SharedPlanes[p5].size(); n++)
											{
												int p6 = SharedPlanes[p5][n];
												if (AllPlanes[p6].size() == 5 && p6 != p4 && p6 != p3 && p6 != p2 && p6 != p1 && SharedPlanes[p6].size() >= AllPlanes[p6].size() && std::find(begin(SharedPlanes[p1]), end(SharedPlanes[p1]), p6) != SharedPlanes[p1].end())
												{
													// This means that all the 12 planes are lateral planes
													std::vector<int> PlanesListTmp = { p1, p2, p3, p4, p5, p6 };
													// Check if p2 to p6 are mutually connected
													std::vector<int> planeMutualityCheck = { p2, p3, p4, p5, p6 };
													int NumMutualPlanes = CheckMutualConnection(planeMutualityCheck, SharedPlanes);

													if (NumMutualPlanes == planeMutualityCheck.size()) // it is a Cup
													{
														TmpCup.planes = PlanesListTmp;
														Cup5_6.push_back(TmpCup);
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
	return Cup5_6;
}


std::vector<CageDetectionAlgorithms::Cage> CageDetectionAlgorithms::CageIdentViaCup::twoCupCagesSym(std::vector<CageDetectionAlgorithms::Cage> Cups, VecOfVec AllPlanes, VecOfVec SharedPlanes, std::string cageType)
{
	// iterate over the identified 5_6 cups
	std::vector<CageDetectionAlgorithms::Cage>  cages; // a vector of cage classes
	CageDetectionAlgorithms cagedetect;
	CageDetectionAlgorithms::Cage tmpCage;
	for (int i = 0; i < Cups.size(); i++)
	{
		for (int j = i + 1; j < Cups.size(); j++)
		{
			std::vector<int> PlaneOverlap(std::min(Cups[i].planes.size(), Cups[j].planes.size()));
			auto it = std::set_intersection(Cups[i].planes.begin(), Cups[i].planes.end(), Cups[j].planes.begin(), Cups[j].planes.end(), PlaneOverlap.begin());
			PlaneOverlap.resize(it - PlaneOverlap.begin());
			if (PlaneOverlap.empty())
			{
				// Check how many planes in the list satisfy the cage constraint
				int NumPlanesSatisfied1 = CheckCupOverlap5_12(Cups[i].planes, Cups[j].planes, SharedPlanes);
				//int NumPlanesSatisfied2 = CheckCupOverlap5_12(cup5_6[j], cup5_6[i], SharedPlanes);

				if (NumPlanesSatisfied1 == Cups[i].planes.size() - 1)
				{
					// concatanate the cups
					std::vector<int> planesInCage = Cups[i].planes;
					planesInCage.insert(planesInCage.end(), Cups[j].planes.begin(), Cups[j].planes.end());

					tmpCage.planes = planesInCage;
					tmpCage.label = cageType;

					cages.push_back(tmpCage);
				}
			}
		}
	}
	return cages;
}


int CageDetectionAlgorithms::CageIdentViaCup::FindCupsOverlap(std::vector<int> cup1, std::vector<int> cup2)
{
	// cup1 and cup2 need to be sorted
	int NumSharedPlanes;
	// find the sharing edges between each pair of cups (The cups are sorted)
	std::vector<int> PlaneOverlap(std::min(cup1.size(), cup1.size()));
	auto it = std::set_intersection(cup1.begin(), cup1.end(), cup2.begin(), cup2.end(), PlaneOverlap.begin());
	PlaneOverlap.resize(it - PlaneOverlap.begin());
	NumSharedPlanes = PlaneOverlap.size();
	return NumSharedPlanes;
}


// This function iterates over four different cups with shared polygons to find the cage formed by all of them
std::vector<CageDetectionAlgorithms::Cage> CageDetectionAlgorithms::CageIdentViaCup::FourCupCagesSym(std::vector<CageDetectionAlgorithms::Cage> Cups, VecOfVec AllPlanes, VecOfVec SharedPlanes, std::string cageType)
{
	// iterate over the identified cups (Assumption: Each two cups share two planes)
	std::vector<CageDetectionAlgorithms::Cage> cages; // a vector of cage classes
	CageDetectionAlgorithms cagedetect;
	CageDetectionAlgorithms::Cage tmpCage;
	for (int i = 0; i < Cups.size(); i++)
	{
		for (int j = 0; j < Cups.size(); j++)
		{
			if (j != i)
			{
				int NumSharedIJ = FindCupsOverlap(Cups[i].planes, Cups[j].planes);
				if (NumSharedIJ == 2)
				{
					for (int k = 0; k < Cups.size(); k++)
					{
						if (k != j && k != i)
						{
							int NumSharedIK = FindCupsOverlap(Cups[i].planes, Cups[k].planes);
							int NumSharedJK = FindCupsOverlap(Cups[j].planes, Cups[k].planes);
							if (NumSharedIK == 2 && NumSharedJK == 2)
							{
								for (int l = 0; l < Cups.size(); l++)
								{
									if (l != k && l != j && l != i) // which means all the four cups are different
									{
										int NumSharedIL = FindCupsOverlap(Cups[i].planes, Cups[l].planes);
										int NumSharedJL = FindCupsOverlap(Cups[j].planes, Cups[l].planes);
										int NumSharedKL = FindCupsOverlap(Cups[k].planes, Cups[l].planes);
										if (NumSharedIL == 2 && NumSharedJL == 2 && NumSharedKL == 2) // which means the four-cup system forms a cage
										{
											std::vector<int> planesInCage = Cups[i].planes;
											planesInCage.insert(planesInCage.end(), Cups[j].planes.begin(), Cups[j].planes.end());
											planesInCage.insert(planesInCage.end(), Cups[k].planes.begin(), Cups[k].planes.end());
											planesInCage.insert(planesInCage.end(), Cups[l].planes.begin(), Cups[l].planes.end());
											// Now remove duplicated planes from the list
											std::sort(planesInCage.begin(), planesInCage.end());
											planesInCage.erase(std::unique(planesInCage.begin(), planesInCage.end()), planesInCage.end());

											tmpCage.planes = planesInCage;
											tmpCage.label = cageType;

											cages.push_back(tmpCage);
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
	return cages;
}


// This function iterates over three different cups with shared polygons to find the cage formed by all of them (An example cage would be 5(12)6(3) cage)
std::vector<CageDetectionAlgorithms::Cage> CageDetectionAlgorithms::CageIdentViaCup::ThreeCupCagesSym(std::vector<CageDetectionAlgorithms::Cage> Cups, VecOfVec AllPlanes, VecOfVec SharedPlanes, std::string cageType)
{
	// iterate over the identified cups (Assumption: Each two cups share two planes)
	std::vector<CageDetectionAlgorithms::Cage> cages; // a vector of cage classes
	CageDetectionAlgorithms cagedetect;
	CageDetectionAlgorithms::Cage tmpCage;
	for (int i = 0; i < Cups.size(); i++)
	{
		for (int j = 0; j < Cups.size(); j++)
		{
			if (j != i)
			{
				int NumSharedIJ = FindCupsOverlap(Cups[i].planes, Cups[j].planes);
				if (NumSharedIJ == 2)
				{
					for (int k = 0; k < Cups.size(); k++)
					{
						if (k != j && k != i)
						{
							int NumSharedIK = FindCupsOverlap(Cups[i].planes, Cups[k].planes);
							int NumSharedJK = FindCupsOverlap(Cups[j].planes, Cups[k].planes);
							if (NumSharedIK == 2 && NumSharedJK == 2)
							{
								std::vector<int> planesInCage = Cups[i].planes;
								planesInCage.insert(planesInCage.end(), Cups[j].planes.begin(), Cups[j].planes.end());
								planesInCage.insert(planesInCage.end(), Cups[k].planes.begin(), Cups[k].planes.end());
								// Now remove duplicated planes from the list
								std::sort(planesInCage.begin(), planesInCage.end());
								planesInCage.erase(std::unique(planesInCage.begin(), planesInCage.end()), planesInCage.end());

								tmpCage.planes = planesInCage;
								tmpCage.label = cageType;

								cages.push_back(tmpCage);
							}

						}
					}
				}
			}
		}
	}
	return cages;
}


std::vector<CageDetectionAlgorithms::Cage> CageDetectionAlgorithms::CageIdentViaCup::cup5_6_6_1(VecOfVec AllPlanes, VecOfVec SharedPlanes)
{
	std::vector<CageDetectionAlgorithms::Cage> cup5_6_6_1;
	CageDetectionAlgorithms::Cage TmpCup;
	TmpCup.label = "5_6_6_1";
	for (int i = 0; i < AllPlanes.size(); i++)
	{
		int p1 = i;
		if (AllPlanes[p1].size() == 6 && SharedPlanes[p1].size() >= AllPlanes[p1].size()) // p1 is lateral and hexagonal
		{
			// Look into the shared plane list of p1
			for (int j = 0; j < SharedPlanes[p1].size(); j++)
			{
				int p2 = SharedPlanes[p1][j];
				if (AllPlanes[p2].size() == 5 && SharedPlanes[p2].size() >= AllPlanes[p2].size())
				{
					// Look into the shared planes of p2
					for (int k = 0; k < SharedPlanes[p2].size(); k++)
					{
						int p3 = SharedPlanes[p2][k];
						if (AllPlanes[p3].size() == 5 && p3 != p1 && SharedPlanes[p3].size() >= AllPlanes[p3].size() && std::find(begin(SharedPlanes[p1]), end(SharedPlanes[p1]), p3) != SharedPlanes[p1].end())
						{
							// Look into the shared planes of p3
							for (int l = 0; l < SharedPlanes[p3].size(); l++)
							{
								int p4 = SharedPlanes[p3][l];
								if (AllPlanes[p4].size() == 5 && p4 != p2 && p4 != p1 && SharedPlanes[p4].size() >= AllPlanes[p4].size() && std::find(begin(SharedPlanes[p1]), end(SharedPlanes[p1]), p4) != SharedPlanes[p1].end())
								{
									// Look into the shared planes of p4
									for (int m = 0; m < SharedPlanes[p4].size(); m++)
									{
										int p5 = SharedPlanes[p4][m];
										if (AllPlanes[p5].size() == 5 && p5 != p3 && p5 != p2 && p5 != p1 && SharedPlanes[p5].size() >= AllPlanes[p5].size() && std::find(begin(SharedPlanes[p1]), end(SharedPlanes[p1]), p5) != SharedPlanes[p1].end())
										{
											// Look into the shared planes of p5
											for (int n = 0; n < SharedPlanes[p5].size(); n++)
											{
												int p6 = SharedPlanes[p5][n];
												if (AllPlanes[p6].size() == 5 && p6 != p4 && p6 != p3 && p6 != p2 && p6 != p1 && SharedPlanes[p6].size() >= AllPlanes[p6].size() && std::find(begin(SharedPlanes[p1]), end(SharedPlanes[p1]), p6) != SharedPlanes[p1].end())
												{
													for (int o = 0; o < SharedPlanes[p6].size(); o++)
													{
														int p7 = SharedPlanes[p6][o];
														if (AllPlanes[p7].size() == 5 && p7 != p5 && p7 != p4 && p7 != p3 && p7 != p2 && p7 != p1 && SharedPlanes[p7].size() >= AllPlanes[p7].size() && std::find(begin(SharedPlanes[p1]), end(SharedPlanes[p1]), p7) != SharedPlanes[p1].end())
														{
															// This means that all the 12 planes are lateral planes
															std::vector<int> PlanesListTmp = { p1, p2, p3, p4, p5, p6, p7 };
															bool mutuallyConnected = CheckMutualConnection(PlanesListTmp, SharedPlanes);
															// Check if p2 to p6 are mutually connected
															std::vector<int> planeMutualityCheck = { p2, p3, p4, p5, p6, p7 };
															int NumMutualPlanes = CheckMutualConnection(planeMutualityCheck, SharedPlanes);

															if (NumMutualPlanes == planeMutualityCheck.size()) // it is a Cup
															{
																TmpCup.planes = PlanesListTmp;
																cup5_6_6_1.push_back(TmpCup);
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
	}
	return cup5_6_6_1;
}



CageDetectionAlgorithms::VecOfVec CageDetectionAlgorithms::CageIdentViaCup::FourCupCages(VecOfVec Cups, VecOfVec AllPlanes, VecOfVec SharedPlanes)
{
	// iterate over the identified 5_6 cups
	VecOfVec cage;
	CageDetectionAlgorithms cagedetect;
	for (int i = 0; i < Cups.size(); i++)
	{
		for (int j = i + 1; j < Cups.size(); j++)
		{
			std::vector<int> PlaneOverlap(std::min(Cups[i].size(), Cups[j].size()));
			auto it = std::set_intersection(Cups[i].begin(), Cups[i].end(), Cups[j].begin(), Cups[j].end(), PlaneOverlap.begin());
			PlaneOverlap.resize(it - PlaneOverlap.begin());
			if (PlaneOverlap.empty())
			{
				// Check how many planes in the list satisfy the cage constraint
				int NumPlanesSatisfied1 = CheckCupOverlap5_12(Cups[i], Cups[j], SharedPlanes);
				//int NumPlanesSatisfied2 = CheckCupOverlap5_12(cup5_6[j], cup5_6[i], SharedPlanes);

				if (NumPlanesSatisfied1 == Cups[i].size() - 1)
				{
					// concatanate the cups
					std::vector<int> planesInCage = Cups[i];
					planesInCage.insert(planesInCage.end(), Cups[j].begin(), Cups[j].end());
					cage.push_back(planesInCage);
				}
			}

		}
	}
	return cage;
}





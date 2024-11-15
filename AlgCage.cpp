#include "AlgCage.h"


CageDetectionAlgorithms::VecOfVec CageDetectionAlgorithms::Detect02PlaneCages(
    VecOfVec AllPlanes, VecOfVec SharedPlanes, VecOfVec NeighboringPlanes) {
    VecOfVec Cages02Plane;
    unsigned int NumplanesToLookFor = 2;

    for (int i = 0; i < AllPlanes.size(); i++) {
        int p1 = i;
        if (SharedPlanes[p1].size() >= AllPlanes[p1].size()) {
            // Check if it is worth to search through the parental node P1
            if (NeighboringPlanes[p1].size() >= (NumplanesToLookFor - 1)) {
                // Look into the shared plane list of p1
                for (int j = 0; j < SharedPlanes[p1].size(); j++) {
                    int p2 = SharedPlanes[p1][j];
                    if (p2 != p1 && SharedPlanes[p2].size() >= AllPlanes[p2].size()) {
                        // This means that all the 3 planes are lateral planes
                        std::vector<int> PlanesListTmp = {p1, p2};

                        // check if the planes are mutually connected
                        std::sort(begin(PlanesListTmp), end(PlanesListTmp));
                        // Check if the detected cage is already in the list
                        if (!isInVecofVec(Cages02Plane, PlanesListTmp)) {
                            Cages02Plane.push_back(PlanesListTmp);
                        }
                    }
                }
            }
        }
    }
    return Cages02Plane;
}


CageDetectionAlgorithms::VecOfVec CageDetectionAlgorithms::Detect03PlaneCages(
    VecOfVec AllPlanes, VecOfVec SharedPlanes, VecOfVec &NumOccEd3Planes) {
    VecOfVec Cages03PlaneSorted;
    unsigned int NumplanesToLookFor = 3;
    std::vector<int> OccpiedEdges = {2, 2, 2};
    for (int i = 0; i < AllPlanes.size(); i++) {
        int p1 = i;
        if (SharedPlanes[p1].size() >= AllPlanes[p1].size()) {
            // Look into the shared plane list of p1
            for (int j = 0; j < SharedPlanes[p1].size(); j++) {
                int p2 = SharedPlanes[p1][j];
                if (p2 != p1 && SharedPlanes[p2].size() >= AllPlanes[p2].size()) {
                    // Look into the shared planes of p2
                    for (int k = 0; k < SharedPlanes[p2].size(); k++) {
                        int p3 = SharedPlanes[p2][k];
                        if (p3 != p2 && p3 != p1 && SharedPlanes[p3].size() >= AllPlanes[p3].size() && std::find(
                                begin(SharedPlanes[p1]), end(SharedPlanes[p1]), p3) != SharedPlanes[p1].end()) {
                            // This means that all the 3 planes are lateral planes
                            std::vector<int> PlanesListTmp = {p1, p2, p3};

                            // Check if the detected cage is already in the list
                            std::vector<int> SortedPlanesListTmp = PlanesListTmp;
                            std::sort(begin(SortedPlanesListTmp), end(SortedPlanesListTmp));
                            Cages03PlaneSorted.push_back(SortedPlanesListTmp);
                            NumOccEd3Planes.push_back(OccpiedEdges);
                        }
                    }
                }
            }
        }
    }
    return Cages03PlaneSorted;
}


CageDetectionAlgorithms::VecOfVec CageDetectionAlgorithms::Detect04PlaneCages(
    VecOfVec AllPlanes, VecOfVec SharedPlanes, VecOfVec NeighboringPlanes) {
    VecOfVec Cages04Plane;
    unsigned int NumplanesToLookFor = 4;

    for (int i = 0; i < AllPlanes.size(); i++) {
        int p1 = i;
        if (SharedPlanes[p1].size() >= AllPlanes[p1].size()) {
            // Check if it is worth to search through the parental node P1
            if (NeighboringPlanes[p1].size() >= (NumplanesToLookFor - 1)) {
                // Look into the shared plane list of p1
                for (int j = 0; j < SharedPlanes[p1].size(); j++) {
                    int p2 = SharedPlanes[p1][j];
                    if (p2 != p1 && SharedPlanes[p2].size() >= AllPlanes[p2].size()) {
                        // Look into the shared planes of p2
                        for (int k = 0; k < SharedPlanes[p2].size(); k++) {
                            int p3 = SharedPlanes[p2][k];
                            if (p3 != p2 && p3 != p1 && SharedPlanes[p3].size() >= AllPlanes[p3].size() && std::find(
                                    begin(SharedPlanes[p1]), end(SharedPlanes[p1]), p3) != SharedPlanes[p1].end()) {
                                // Look into the shared planes of p3
                                for (int l = 0; l < SharedPlanes[p3].size(); l++) {
                                    int p4 = SharedPlanes[p3][l];
                                    if (p4 != p3 && p4 != p2 && p4 != p1 && SharedPlanes[p4].size() >= AllPlanes[p4].
                                        size() && std::find(begin(SharedPlanes[p2]), end(SharedPlanes[p2]), p4) !=
                                        SharedPlanes[p2].end()) {
                                        // This means that all the 3 planes are lateral planes
                                        std::vector<int> PlanesListTmp = {p1, p2, p3, p4};
                                        // Check if the detected cage is already in the list
                                        std::sort(begin(PlanesListTmp), end(PlanesListTmp));
                                        if (!isInVecofVec(Cages04Plane, PlanesListTmp)) {
                                            Cages04Plane.push_back(PlanesListTmp);
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
    return Cages04Plane;
}


int CageDetectionAlgorithms::DFSatisifed(std::vector<int> plane1, std::vector<int> plane2, VecOfVec AllPlanes,
                                         VecOfVec SharedPlanes, std::vector<int> NumEdOccPlaneX,
                                         std::vector<int> NumEdOccGoal, std::vector<int> &NumOccEdUpdated) {
    int PlanesSatisifed = 0;
    for (int i = 0; i < plane1.size(); i++) // iterate over the planes in the set PlaneX
    {
        std::vector<int> PlaneOverlap(std::min(plane1.size(), plane2.size()));
        //std::sort(begin(SharedPlanes[planesX[i]]), end(SharedPlanes[planesX[i]]));
        auto it = std::set_intersection(SharedPlanes[plane1[i]].begin(), SharedPlanes[plane1[i]].end(), plane2.begin(),
                                        plane2.end(), PlaneOverlap.begin());
        PlaneOverlap.resize(it - PlaneOverlap.begin());
        NumOccEdUpdated.push_back(NumEdOccPlaneX[i] + PlaneOverlap.size());
        int DFnew = AllPlanes[plane1[i]].size() - (NumEdOccPlaneX[i] + PlaneOverlap.size());
        bool isIngoal = false;
        for (int j = 0; j < NumEdOccGoal.size(); j++) {
            if (DFnew == AllPlanes[plane1[i]].size() - NumEdOccGoal[j]) {
                isIngoal = true;
                NumEdOccGoal.erase(NumEdOccGoal.begin() + j);
                break;
            }
        }
        if (isIngoal) {
            PlanesSatisifed += 1;
        } else {
            break;
        }
        /*if (ResDFnew == ResDFGoal[i])
        {
            PlanesSatisifed += 1;
        }
        else if (ResDFnew == ResDFGoal[i])
        {

        }*/
    }
    return PlanesSatisifed;
}


CageDetectionAlgorithms::VecOfVec CageDetectionAlgorithms::Detect2XPlaneCagesfromXPlanes(
    VecOfVec planesX, VecOfVec planesY, VecOfVec planesXYSorted, VecOfVec AllPlanes, VecOfVec SharedPlanes,
    VecOfVec NumOccEdPlane) {
    VecOfVec Cages2XPlane;
    for (int i = 0; i < planesX.size(); i++) {
        std::vector<int> X = planesXYSorted[i];
        std::vector<int> NumOccEdGoal;
        for (int k = 0; k < planesX[i].size(); k++) {
            NumOccEdGoal.push_back(AllPlanes[planesX[i][k]].size());
        }

        for (int j = i + 1; j < planesY.size(); j++) {
            // Concatenate the plane lists only if they don't share a plane
            //std::vector<int> planeListtmp(planesX[i].size() + planesX[j].size());
            std::vector<int> Y = planesXYSorted[j];
            std::vector<int> PlaneOverlap(std::min(X.size(), Y.size()));
            auto it = std::set_intersection(X.begin(), X.end(), Y.begin(), Y.end(), PlaneOverlap.begin());
            PlaneOverlap.resize(it - PlaneOverlap.begin());
            if (PlaneOverlap.empty()) {
                // Check if the degrees of freedoms of the planes in one set are filled by the planes from the other set

                std::vector<int> NumOccEdUpdated;
                int NumSatPlanes = DFSatisifed(planesX[i], planesXYSorted[j], AllPlanes, SharedPlanes, NumOccEdPlane[i],
                                               NumOccEdGoal, NumOccEdUpdated);
                if (NumSatPlanes == planesX[i].size()) {
                    std::vector<int> planeListtmp = planesXYSorted[i];
                    planeListtmp.insert(planeListtmp.end(), planesXYSorted[j].begin(), planesXYSorted[j].end());
                    Cages2XPlane.push_back(planeListtmp);
                    planeListtmp.clear();
                }

                //std::sort(begin(planeListtmp), end(planeListtmp));
                //if (!isInVecofVec(Cages2XPlane, planeListtmp))
                //{
                //	Cages2XPlane.push_back(planeListtmp);
                //}
                ////Cages2XPlane.push_back(planeListtmp);
                //planeListtmp.clear();
            }
        }
    }
    return Cages2XPlane;
}


CageDetectionAlgorithms::VecOfVec CageDetectionAlgorithms::Plane2XfromXPlanes(
    VecOfVec planesX, VecOfVec planesXSorted, VecOfVec AllPlanes, VecOfVec SharedPlanes, VecOfVec NumOccEdPlane,
    std::vector<int> NumOccEdGoal, VecOfVec &NumOccEd6Planes) {
    VecOfVec Planes2X;
    for (int i = 0; i < planesX.size(); i++) {
        std::vector<int> X = planesXSorted[i];
        for (int j = i + 1; j < planesX.size(); j++) {
            std::vector<int> Y = planesXSorted[j];
            std::vector<int> PlaneOverlap(std::min(X.size(), Y.size()));
            auto it = std::set_intersection(X.begin(), X.end(), Y.begin(), Y.end(), PlaneOverlap.begin());
            PlaneOverlap.resize(it - PlaneOverlap.begin());
            if (PlaneOverlap.empty()) {
                // Check if the degrees of freedoms of the planes in one set are filled by the planes from the other set
                std::vector<int> NumOccEdUpdatedX;
                if (i == 0 && j == 44) {
                    int b = 1;
                }
                int NumSatPlanesX = DFSatisifed(planesX[i], Y, AllPlanes, SharedPlanes, NumOccEdPlane[i], NumOccEdGoal,
                                                NumOccEdUpdatedX);
                if (NumSatPlanesX == planesX[i].size()) {
                    std::vector<int> NumOccEdUpdatedTY;
                    int NumSatPlanesY = DFSatisifed(planesX[j], X, AllPlanes, SharedPlanes, NumOccEdPlane[j],
                                                    NumOccEdGoal, NumOccEdUpdatedTY);

                    if (NumSatPlanesY == planesX[j].size()) {
                        std::vector<int> planeListtmp = planesX[i];
                        planeListtmp.insert(planeListtmp.end(), planesX[j].begin(), planesX[j].end());
                        Planes2X.push_back(planeListtmp);

                        NumOccEdUpdatedX.insert(NumOccEdUpdatedX.end(), NumOccEdUpdatedTY.begin(),
                                                NumOccEdUpdatedTY.end());

                        NumOccEd6Planes.push_back(NumOccEdUpdatedX);
                        planeListtmp.clear();
                    }


                    /*std::vector<int> ResDFGoal2 = ResDFGoal;
                    std::reverse(ResDFGoal2.begin(), ResDFGoal2.end());
                    int NumSatPlanesY = DFSatisifed(planesX[j], planesXSorted[i], SharedPlanes, ResDFAllPlanes, ResDFGoal2);
                    if (NumSatPlanesY == planesX[j].size())
                    {
                        std::vector<int> planeListtmp = planesX[i];
                        planeListtmp.insert(planeListtmp.end(), planesX[j].begin(), planesX[j].end());
                        Planes2X.push_back(planeListtmp);
                        planeListtmp.clear();
                    }*/
                }
            }
        }
    }
    return Planes2X;
}


//CageDetectionAlgorithms::VecOfVec CageDetectionAlgorithms::Detect2XPlanefromXPlanesUnordMap(VecOfVec planesX, VecOfVec planesY, VecOfVec AllPlanes, VecOfUnorMap SharedPlanes)
//{
//	VecOfVec Cages2XPlane;
//	for (int i = 0; i < planesX.size(); i++)
//	{
//		//std::sort(begin(planesX[i]), end(planesX[i]));
//		for (int j = i + 1; j < planesY.size(); j++)
//		{
//			//std::sort(begin(planesX[j]), end(planesX[j]));
//			//int s = 0;
//			//// Determine if the plane lists share at least one plane
//			//for (int k = 0; k < planesX[j].size(); k++)
//			//{
//			//	if (std::binary_search(planesX[i].begin(), planesX[i].end(), planesX[j][k]))
//			//	{
//			//		s += 1;
//			//		break;
//			//	}
//			//}
//			// Concatenate the plane lists only if they don't share a plane
//			//std::vector<int> planeListtmp(planesX[i].size() + planesX[j].size());
//			std::vector<int> PlaneOverlap;
//			std::set_intersection(planesX[i].begin(), planesX[i].end(), planesX[j].begin(), planesX[j].end(), PlaneOverlap.begin());
//			if (PlaneOverlap.empty())
//			{
//				std::vector<int> planeListtmp = planesX[i];
//				planeListtmp.insert(planeListtmp.end(), planesY[j].begin(), planesY[j].end());
//				std::sort(begin(planeListtmp), end(planeListtmp));
//				if (!isInVecofVec(Cages2XPlane, planeListtmp))
//				{
//					Cages2XPlane.push_back(planeListtmp);
//				}
//				planeListtmp.clear();
//			}
//
//			//std::merge(planesX[i].begin(), planesX[i].end(), planesX[j].begin(), planesX[j].end(), planeListtmp.begin());
//			// Check if the detected cage is already in the list
//			/*if (!isInVecofVec(Cages2XPlane, planeListtmp))
//			{
//				Cages2XPlane.push_back(planeListtmp);
//			}*/
//			//std::sort(begin(planeListtmp), end(planeListtmp));
//			//auto it = std::unique(planeListtmp.begin(), planeListtmp.end());
//
//			// Ensure there is no plane duplication in the concatanated plane
//			//if (std::adjacent_find(planeListtmp.begin(), planeListtmp.end())== planeListtmp.end())
//			//{
//
//			//	// Make sure if this set of planes has not been pushed back to the list before (planeListtmp is already sorted)
//			//	if (!isInVecofVec(Cages2XPlane, planeListtmp))
//			//	{
//			//		Cages2XPlane.push_back(planeListtmp);
//			//	}
//			//	//Cages2XPlane.push_back(planeListtmp);
//			//
//			//
//			//}
//			//planeListtmp.clear(); // clear the temporary 12-plane list to be used for the subsequent analyses
//			//if (s == 0)
//			//{
//			//	std::vector<int> planeListtmp(planesX[i].size() + planesX[j].size());
//			//	std::merge(planesX[i].begin(), planesX[i].end(), planesX[j].begin(), planesX[j].end(), planeListtmp.begin());
//			//	// Check if the detected cage is already in the list
//			//	if (!isInVecofVec(Cages2XPlane, planeListtmp))
//			//	{
//			//		Cages2XPlane.push_back(planeListtmp);
//			//	}
//			//
//			//	planeListtmp.clear(); // clear the temporary 12-plane list to be used for the subsequent analyses
//			//}
//
//			//std::set_intersection(begin(planes06[i]), end(planes06[i]), begin(planes06[j]), end(planes06[j]), back_inserter(vec3))
//		}
//	}
//	return Cages2XPlane;
//}


CageDetectionAlgorithms::VecOfVec CageDetectionAlgorithms::CheckCageConstraint(
    VecOfVec planesX, VecOfVec AllPlanes, VecOfVec SharedPlanes) {
    VecOfVec planesXCages;
    for (int i = 0; i < planesX.size(); i++) {
        int NumPlanesInCage = 0;
        // Check how many planes in the list satisfy the cage constraint

        for (int j = 0; j < planesX[i].size(); j++) {
            int ptmp = planesX[i][j];
            size_t numSharedinList = NumSharedPlanesInSequence(planesX[i], SharedPlanes, ptmp);
            if (numSharedinList == AllPlanes[ptmp].size()) {
                NumPlanesInCage += 1;
            }
            if (NumPlanesInCage != j + 1) {
                break;
            }
        }
        // Check if the sequence of planes form a cage
        if (NumPlanesInCage == planesX[i].size()) {
            // This forms a closed loop so it is a cage
            //std::sort(PlanesListTmp.begin(), PlanesListTmp.end());
            // Check if the detected cage is already in the list
            std::sort(planesX[i].begin(), planesX[i].end());
            if (!isInVecofVec(planesXCages, planesX[i])) {
                planesXCages.push_back(planesX[i]);
            }
        }
    }
    return planesXCages;
}


CageDetectionAlgorithms::VecOfVec CageDetectionAlgorithms::Detect06PlaneCages(
    VecOfVec AllPlanes, VecOfVec SharedPlanes, VecOfVec &Cages6PlaneUnsorted, VecOfVec &DFAllPlanes) {
    VecOfVec Cages06PlaneSorted;
    unsigned int NumplanesToLookFor = 6;
    std::vector<int> OccpiedEdges = {2, 3, 4, 4, 3, 2};
    for (int i = 0; i < AllPlanes.size(); i++) {
        int p1 = i;
        if (SharedPlanes[p1].size() >= AllPlanes[p1].size()) {
            // Look into the shared plane list of p1
            for (int j = 0; j < SharedPlanes[p1].size(); j++) {
                int p2 = SharedPlanes[p1][j];
                if (p2 != p1 && SharedPlanes[p2].size() >= AllPlanes[p2].size()) {
                    // Look into the shared planes of p2
                    for (int k = 0; k < SharedPlanes[p2].size(); k++) {
                        int p3 = SharedPlanes[p2][k];
                        if (p3 != p2 && p3 != p1 && SharedPlanes[p3].size() >= AllPlanes[p3].size() && std::find(
                                begin(SharedPlanes[p1]), end(SharedPlanes[p1]), p3) != SharedPlanes[p1].end()) {
                            // Look into the shared planes of p3
                            for (int l = 0; l < SharedPlanes[p3].size(); l++) {
                                int p4 = SharedPlanes[p3][l];
                                if (p4 != p3 && p4 != p2 && p4 != p1 && SharedPlanes[p4].size() >= AllPlanes[p4].size()
                                    && std::find(begin(SharedPlanes[p2]), end(SharedPlanes[p2]), p4) != SharedPlanes[p2]
                                    .end()) {
                                    // Look into the shared planes of p4
                                    for (int m = 0; m < SharedPlanes[p4].size(); m++) {
                                        int p5 = SharedPlanes[p4][m];
                                        if (p5 != p4 && p5 != p3 && p5 != p2 && p5 != p1 && SharedPlanes[p5].size() >=
                                            AllPlanes[p5].size() && std::find(
                                                begin(SharedPlanes[p3]), end(SharedPlanes[p3]),
                                                p5) != SharedPlanes[p3].end()) {
                                            // Look into the shared planes of p5
                                            for (int n = 0; n < SharedPlanes[p5].size(); n++) {
                                                int p6 = SharedPlanes[p5][n];
                                                if (p6 != p5 && p6 != p4 && p6 != p3 && p6 != p2 && p6 != p1 &&
                                                    SharedPlanes[p6].size() >= AllPlanes[p6].size() && std::find(
                                                        begin(SharedPlanes[p4]), end(SharedPlanes[p4]),
                                                        p6) != SharedPlanes[p4].end()) {
                                                    // This means that all the 12 planes are lateral planes
                                                    std::vector<int> PlanesListTmp = {p1, p2, p3, p4, p5, p6};

                                                    std::vector<int> Df_Planelisttmp(PlanesListTmp.size());
                                                    for (int o = 0; o < PlanesListTmp.size(); o++) {
                                                        Df_Planelisttmp[o] = OccpiedEdges[o];
                                                    }
                                                    // Check if the detected cage is already in the list
                                                    std::vector<int> SortedPlanesListTmp = PlanesListTmp;
                                                    std::sort(begin(SortedPlanesListTmp), end(SortedPlanesListTmp));
                                                    if (!isInVecofVec(Cages06PlaneSorted, SortedPlanesListTmp)) {
                                                        Cages06PlaneSorted.push_back(SortedPlanesListTmp);
                                                        Cages6PlaneUnsorted.push_back(PlanesListTmp);
                                                        DFAllPlanes.push_back(Df_Planelisttmp);
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
    return Cages06PlaneSorted;
}


CageDetectionAlgorithms::VecOfVec CageDetectionAlgorithms::Detect06PlaneCagesUnorderMap(
    VecOfVec AllPlanes, VecOfUnorMap SharedPlanes) {
    VecOfVec Cages06Plane;
    unsigned int NumplanesToLookFor = 6;

    for (int i = 0; i < AllPlanes.size(); i++) {
        int p1 = i;
        if (SharedPlanes[p1].size() >= AllPlanes[p1].size()) {
            // Look into the shared plane list of p1
            for (std::pair<int, int> element1: SharedPlanes[p1]) {
                int p2 = element1.first;
                if (SharedPlanes[p2].size() >= AllPlanes[p2].size()) {
                    for (std::pair<int, int> element2: SharedPlanes[p2]) {
                        int p3 = element2.first;
                        if (p3 != p1 && SharedPlanes[p3].size() >= AllPlanes[p3].size() && SharedPlanes[p1].find(p3) !=
                            SharedPlanes[p1].end()) {
                            for (std::pair<int, int> element3: SharedPlanes[p3]) {
                                int p4 = element3.first;
                                if (p4 != p2 && p4 != p1 && SharedPlanes[p4].size() >= AllPlanes[p4].size() &&
                                    SharedPlanes[p2].find(p4) != SharedPlanes[p2].end()) {
                                    for (std::pair<int, int> element4: SharedPlanes[p4]) {
                                        int p5 = element4.first;
                                        if (p5 != p3 && p5 != p2 && p5 != p1 && SharedPlanes[p5].size() >= AllPlanes[p5]
                                            .size() && SharedPlanes[p3].find(p5) != SharedPlanes[p3].end()) {
                                            for (std::pair<int, int> element5: SharedPlanes[p5]) {
                                                int p6 = element5.first;
                                                if (p6 != p4 && p6 != p3 && p6 != p2 && p6 != p1 && SharedPlanes[p6].
                                                    size() >= AllPlanes[p6].size() && SharedPlanes[p4].find(p6) !=
                                                    SharedPlanes[p4].end()) {
                                                    // This means that all the six planes are lateral planes
                                                    std::vector<int> PlanesListTmp = {p1, p2, p3, p4, p5, p6};
                                                    // Check if the detected cage is already in the list
                                                    std::sort(begin(PlanesListTmp), end(PlanesListTmp));
                                                    if (!isInVecofVec(Cages06Plane, PlanesListTmp)) {
                                                        Cages06Plane.push_back(PlanesListTmp);
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
    return Cages06Plane;
}


CageDetectionAlgorithms::VecOfVec CageDetectionAlgorithms::Detect07PlaneCages(
    VecOfVec AllPlanes, VecOfVec SharedPlanes, VecOfVec NeighboringPlanes) {
    VecOfVec Cages07Plane;
    unsigned int NumplanesToLookFor = 7;

    for (int i = 0; i < AllPlanes.size(); i++) {
        int p1 = i;
        if (SharedPlanes[p1].size() >= AllPlanes[p1].size()) {
            // Check if it is worth to search through the parental node P1
            if (NeighboringPlanes[p1].size() >= (NumplanesToLookFor - 1)) {
                // Look into the shared plane list of p1
                for (int j = 0; j < SharedPlanes[p1].size(); j++) {
                    int p2 = SharedPlanes[p1][j];
                    if (p2 != p1 && SharedPlanes[p2].size() >= AllPlanes[p2].size()) {
                        // Look into the shared planes of p2
                        for (int k = 0; k < SharedPlanes[p2].size(); k++) {
                            int p3 = SharedPlanes[p2][k];
                            if (p3 != p2 && p3 != p1 && SharedPlanes[p3].size() >= AllPlanes[p3].size() && std::find(
                                    begin(SharedPlanes[p1]), end(SharedPlanes[p1]), p3) != SharedPlanes[p1].end()) {
                                // Look into the shared planes of p3
                                for (int l = 0; l < SharedPlanes[p3].size(); l++) {
                                    int p4 = SharedPlanes[p3][l];
                                    if (p4 != p3 && p4 != p2 && p4 != p1 && SharedPlanes[p4].size() >= AllPlanes[p4].
                                        size() && std::find(begin(SharedPlanes[p2]), end(SharedPlanes[p2]), p4) !=
                                        SharedPlanes[p2].end()) {
                                        // Look into the shared planes of p4
                                        for (int m = 0; m < SharedPlanes[p4].size(); m++) {
                                            int p5 = SharedPlanes[p4][m];
                                            if (p5 != p4 && p5 != p3 && p5 != p2 && p5 != p1 && SharedPlanes[p5].size()
                                                >= AllPlanes[p5].size() && std::find(
                                                    begin(SharedPlanes[p3]), end(SharedPlanes[p3]),
                                                    p5) != SharedPlanes[p3].end()) {
                                                // Look into the shared planes of p5
                                                for (int n = 0; n < SharedPlanes[p5].size(); n++) {
                                                    int p6 = SharedPlanes[p5][n];
                                                    if (p6 != p5 && p6 != p4 && p6 != p3 && p6 != p2 && p6 != p1 &&
                                                        SharedPlanes[p6].size() >= AllPlanes[p6].size() && std::find(
                                                            begin(SharedPlanes[p4]), end(SharedPlanes[p4]),
                                                            p6) != SharedPlanes[p4].end()) {
                                                        for (int o = 0; o < SharedPlanes[p6].size(); o++) {
                                                            int p7 = SharedPlanes[p6][o];
                                                            if (p7 != p6 && p7 != p5 && p7 != p4 && p7 != p3 && p7 != p2
                                                                && p7 != p1 && SharedPlanes[p7].size() >= AllPlanes[p7].
                                                                size() && std::find(
                                                                    begin(SharedPlanes[p5]), end(SharedPlanes[p5]),
                                                                    p7) != SharedPlanes[p5].end()) {
                                                                // This means that all the 12 planes are lateral planes
                                                                std::vector<int> PlanesListTmp = {
                                                                    p1, p2, p3, p4, p5, p6, p7
                                                                };
                                                                // Check if the detected cage is already in the list
                                                                if (!isInVecofVec(Cages07Plane, PlanesListTmp)) {
                                                                    Cages07Plane.push_back(PlanesListTmp);
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
    }
    return Cages07Plane;
}


CageDetectionAlgorithms::VecOfVec CageDetectionAlgorithms::Detect12PlaneCagesfrom06Planes(
    VecOfVec planes06, VecOfVec AllPlanes, VecOfVec SharedPlanes) {
    VecOfVec Cages12Plane;
    for (int i = 0; i < planes06.size(); i++) {
        for (int j = 0; j < planes06.size(); j++) {
            std::sort(begin(planes06[i]), end(planes06[i]));
            std::sort(begin(planes06[j]), end(planes06[j]));
            int s = 0;
            // Determine if the plane lists share at least one plane
            for (int k = 0; k < planes06[j].size(); k++) {
                if (std::binary_search(planes06[i].begin(), planes06[i].end(), planes06[j][k])) {
                    s += 1;
                    break;
                }
            }
            // Concatenate the plane lists only if they don't share a plane
            if (s == 0) {
                std::vector<int> planeListtmp(planes06[i].size() + planes06[j].size());
                std::merge(planes06[i].begin(), planes06[i].end(), planes06[j].begin(), planes06[j].end(),
                           planeListtmp.begin());
                //planeListtmp.insert(planes06[i].end(), planes06[j].begin(), planes06[j].end()); // construct the new list

                // Check how many planes in the list satisfy the cage constraint
                int NumPlanesInCage = 0;
                for (int w = 0; w < planeListtmp.size(); w++) {
                    int ptmp = planeListtmp[w];
                    size_t numSharedinList = NumSharedPlanesInSequence(planeListtmp, SharedPlanes, ptmp);
                    if (numSharedinList == AllPlanes[ptmp].size()) {
                        NumPlanesInCage += 1;
                    }
                }
                // Check if the sequence of planes form a cage
                if (NumPlanesInCage == planeListtmp.size()) {
                    // This forms a closed loop so it is a cage
                    //std::sort(PlanesListTmp.begin(), PlanesListTmp.end());
                    // Check if the detected cage is already in the list
                    Cages12Plane.push_back(planeListtmp);
                }
                planeListtmp.clear(); // clear the temporary 12-plane list to be used for the subsequent analyses
            }

            //std::set_intersection(begin(planes06[i]), end(planes06[i]), begin(planes06[j]), end(planes06[j]), back_inserter(vec3))
        }
    }
    return Cages12Plane;
}


CageDetectionAlgorithms::VecOfVec CageDetectionAlgorithms::Detect12PlaneCages(
    VecOfVec AllPlanes, VecOfVec SharedPlanes, VecOfVec NeighboringPlanes) {
    VecOfVec Cages12Plane;
    unsigned int NumplanesToLookFor = 12;

    for (unsigned int i = 0; i < AllPlanes.size(); i++) {
        int p1 = i;
        bool isLateral = (SharedPlanes[p1].size() >= AllPlanes[p1].size());
        if (isLateral) {
            // Check if it is worth to search through the parental node P1
            if (NeighboringPlanes[p1].size() >= (NumplanesToLookFor - 1)) {
                // Look into the shared plane list of p1
                for (unsigned int j = 0; j < SharedPlanes[p1].size(); j++) {
                    int p2 = SharedPlanes[p1][j];
                    if (p2 != p1) {
                        bool isLateral = (SharedPlanes[p2].size() >= AllPlanes[p2].size());
                        bool inNeighborhood = (std::find(NeighboringPlanes[p1].begin(), NeighboringPlanes[p1].end(), p2)
                                               != NeighboringPlanes[p1].end());
                        if (isLateral && inNeighborhood) {
                            // Look into the shared planes of p2
                            for (unsigned int k = 0; k < SharedPlanes[p2].size(); k++) {
                                int p3 = SharedPlanes[p2][k];
                                if (p3 != p2 && p3 != p1) {
                                    bool isLateral = (SharedPlanes[p3].size() >= AllPlanes[p3].size());
                                    bool inNeighborhood = (std::find(NeighboringPlanes[p1].begin(),
                                                                     NeighboringPlanes[p1].end(),
                                                                     p3) != NeighboringPlanes[p1].end());
                                    if (isLateral && inNeighborhood) {
                                        // Look into the shared planes of p3
                                        for (unsigned int l = 0; l < SharedPlanes[p3].size(); l++) {
                                            int p4 = SharedPlanes[p3][l];
                                            if (p4 != p3 && p4 != p2 && p4 != p1) {
                                                bool isLateral = (SharedPlanes[p4].size() >= AllPlanes[p4].size());
                                                bool inNeighborhood = (std::find(NeighboringPlanes[p1].begin(),
                                                                           NeighboringPlanes[p1].end(),
                                                                           p4) != NeighboringPlanes[p1].end());
                                                if (isLateral && inNeighborhood) {
                                                    // Look into the shared planes of p4
                                                    for (unsigned int m = 0; m < SharedPlanes[p4].size(); m++) {
                                                        int p5 = SharedPlanes[p4][m];
                                                        if (p5 != p4 && p5 != p3 && p5 != p2 && p5 != p1) {
                                                            bool isLateral = (
                                                                SharedPlanes[p5].size() >= AllPlanes[p5].size());
                                                            bool inNeighborhood = (
                                                                std::find(NeighboringPlanes[p1].begin(),
                                                                          NeighboringPlanes[p1].end(),
                                                                          p5) != NeighboringPlanes[p1].end());
                                                            if (isLateral && inNeighborhood) {
                                                                // Look into the shared planes of p5
                                                                for (unsigned int n = 0; n < SharedPlanes[p5].size(); n
                                                                     ++) {
                                                                    int p6 = SharedPlanes[p5][n];
                                                                    if (p6 != p5 && p6 != p4 && p6 != p3 && p6 != p2 &&
                                                                        p6 != p1) {
                                                                        bool isLateral = (
                                                                            SharedPlanes[p6].size() >= AllPlanes[p6].
                                                                            size());
                                                                        bool inNeighborhood = (
                                                                            std::find(NeighboringPlanes[p1].begin(),
                                                                                NeighboringPlanes[p1].end(),
                                                                                p6) != NeighboringPlanes[p1].end());
                                                                        if (isLateral && inNeighborhood) {
                                                                            // Look into the shared planes of p6
                                                                            for (unsigned int o = 0;
                                                                                o < SharedPlanes[p6].size(); o++) {
                                                                                int p7 = SharedPlanes[p6][o];
                                                                                if (p7 != p6 && p7 != p5 && p7 != p4 &&
                                                                                    p7 != p3 && p7 != p2 && p7 != p1) {
                                                                                    bool isLateral = (
                                                                                        SharedPlanes[p7].size() >=
                                                                                        AllPlanes[p7].size());
                                                                                    bool inNeighborhood = (
                                                                                        std::find(
                                                                                            NeighboringPlanes[p1].
                                                                                            begin(),
                                                                                            NeighboringPlanes[p1].end(),
                                                                                            p7) != NeighboringPlanes[p1]
                                                                                        .end());
                                                                                    if (isLateral && inNeighborhood) {
                                                                                        // Look into the shared planes of p7
                                                                                        for (unsigned int p = 0;
                                                                                            p < SharedPlanes[p7].size();
                                                                                            p++) {
                                                                                            int p8 = SharedPlanes[p7][
                                                                                                p];
                                                                                            if (p8 != p7 && p8 != p6 &&
                                                                                                p8 != p5 && p8 != p4 &&
                                                                                                p8 != p3 && p8 != p2 &&
                                                                                                p8 != p1) {
                                                                                                bool isLateral = (
                                                                                                    SharedPlanes[p8].
                                                                                                    size() >= AllPlanes[
                                                                                                        p8].size());
                                                                                                bool inNeighborhood = (
                                                                                                    std::find(
                                                                                                        NeighboringPlanes
                                                                                                        [p1].begin(),
                                                                                                        NeighboringPlanes
                                                                                                        [p1].end(), p8)
                                                                                                    != NeighboringPlanes
                                                                                                    [p1].end());
                                                                                                if (isLateral &&
                                                                                                    inNeighborhood) {
                                                                                                    // Look into the shared planes of p8
                                                                                                    for (unsigned int q
                                                                                                                = 0;
                                                                                                        q < SharedPlanes
                                                                                                        [p8].size(); q
                                                                                                        ++) {
                                                                                                        int p9 =
                                                                                                                SharedPlanes
                                                                                                                [p8][q];
                                                                                                        if (p9 != p8 &&
                                                                                                            p9 != p7 &&
                                                                                                            p9 != p6 &&
                                                                                                            p9 != p5 &&
                                                                                                            p9 != p4 &&
                                                                                                            p9 != p3 &&
                                                                                                            p9 != p2 &&
                                                                                                            p9 != p1) {
                                                                                                            bool
                                                                                                                    isLateral
                                                                                                                            = (
                                                                                                                                SharedPlanes
                                                                                                                                [p9]
                                                                                                                                .size()
                                                                                                                                >=
                                                                                                                                AllPlanes
                                                                                                                                [p9]
                                                                                                                                .size());
                                                                                                            bool
                                                                                                                    inNeighborhood
                                                                                                                            = (
                                                                                                                                std::find(
                                                                                                                                    NeighboringPlanes
                                                                                                                                    [p1]
                                                                                                                                    .begin(),
                                                                                                                                    NeighboringPlanes
                                                                                                                                    [p1]
                                                                                                                                    .end(),
                                                                                                                                    p9)
                                                                                                                                !=
                                                                                                                                NeighboringPlanes
                                                                                                                                [p1]
                                                                                                                                .end());
                                                                                                            if (
                                                                                                                isLateral
                                                                                                                &&
                                                                                                                inNeighborhood) {
                                                                                                                // Look into the shared planes of p9
                                                                                                                for (
                                                                                                                    unsigned
                                                                                                                    int
                                                                                                                    r =
                                                                                                                            0;
                                                                                                                    r <
                                                                                                                    SharedPlanes
                                                                                                                    [p9]
                                                                                                                    .size()
                                                                                                                    ; r
                                                                                                                    ++) {
                                                                                                                    int
                                                                                                                            p10
                                                                                                                                    = SharedPlanes
                                                                                                                                    [p9]
                                                                                                                                    [r];
                                                                                                                    if (
                                                                                                                        p10
                                                                                                                        !=
                                                                                                                        p9
                                                                                                                        &&
                                                                                                                        p10
                                                                                                                        !=
                                                                                                                        p8
                                                                                                                        &&
                                                                                                                        p10
                                                                                                                        !=
                                                                                                                        p7
                                                                                                                        &&
                                                                                                                        p10
                                                                                                                        !=
                                                                                                                        p6
                                                                                                                        &&
                                                                                                                        p10
                                                                                                                        !=
                                                                                                                        p5
                                                                                                                        &&
                                                                                                                        p10
                                                                                                                        !=
                                                                                                                        p4
                                                                                                                        &&
                                                                                                                        p10
                                                                                                                        !=
                                                                                                                        p3
                                                                                                                        &&
                                                                                                                        p10
                                                                                                                        !=
                                                                                                                        p2
                                                                                                                        &&
                                                                                                                        p10
                                                                                                                        !=
                                                                                                                        p1) {
                                                                                                                        bool
                                                                                                                                isLateral
                                                                                                                                        = (
                                                                                                                                            SharedPlanes
                                                                                                                                            [p10]
                                                                                                                                            .size()
                                                                                                                                            >=
                                                                                                                                            AllPlanes
                                                                                                                                            [p10]
                                                                                                                                            .size());
                                                                                                                        bool
                                                                                                                                inNeighborhood
                                                                                                                                        = (
                                                                                                                                            std::find(
                                                                                                                                                NeighboringPlanes
                                                                                                                                                [p1]
                                                                                                                                                .begin(),
                                                                                                                                                NeighboringPlanes
                                                                                                                                                [p1]
                                                                                                                                                .end(),
                                                                                                                                                p10)
                                                                                                                                            !=
                                                                                                                                            NeighboringPlanes
                                                                                                                                            [p1]
                                                                                                                                            .end());
                                                                                                                        if
                                                                                                                        (isLateral
                                                                                                                            &&
                                                                                                                            inNeighborhood) {
                                                                                                                            // Look into the shared planes of p10
                                                                                                                            for
                                                                                                                            (unsigned
                                                                                                                                int
                                                                                                                                s =
                                                                                                                                        0;
                                                                                                                                s <
                                                                                                                                SharedPlanes
                                                                                                                                [p10]
                                                                                                                                .size()
                                                                                                                                ; s
                                                                                                                                ++) {
                                                                                                                                int
                                                                                                                                        p11
                                                                                                                                                = SharedPlanes
                                                                                                                                                [p10]
                                                                                                                                                [s];
                                                                                                                                if
                                                                                                                                (p11
                                                                                                                                    !=
                                                                                                                                    p10
                                                                                                                                    &&
                                                                                                                                    p11
                                                                                                                                    !=
                                                                                                                                    p9
                                                                                                                                    &&
                                                                                                                                    p11
                                                                                                                                    !=
                                                                                                                                    p8
                                                                                                                                    &&
                                                                                                                                    p11
                                                                                                                                    !=
                                                                                                                                    p7
                                                                                                                                    &&
                                                                                                                                    p11
                                                                                                                                    !=
                                                                                                                                    p6
                                                                                                                                    &&
                                                                                                                                    p11
                                                                                                                                    !=
                                                                                                                                    p5
                                                                                                                                    &&
                                                                                                                                    p11
                                                                                                                                    !=
                                                                                                                                    p4
                                                                                                                                    &&
                                                                                                                                    p11
                                                                                                                                    !=
                                                                                                                                    p3
                                                                                                                                    &&
                                                                                                                                    p11
                                                                                                                                    !=
                                                                                                                                    p2
                                                                                                                                    &&
                                                                                                                                    p11
                                                                                                                                    !=
                                                                                                                                    p1) {
                                                                                                                                    bool
                                                                                                                                            isLateral
                                                                                                                                                    = (
                                                                                                                                                        SharedPlanes
                                                                                                                                                        [p11]
                                                                                                                                                        .size()
                                                                                                                                                        >=
                                                                                                                                                        AllPlanes
                                                                                                                                                        [p11]
                                                                                                                                                        .size());
                                                                                                                                    bool
                                                                                                                                            inNeighborhood
                                                                                                                                                    = (
                                                                                                                                                        std::find(
                                                                                                                                                            NeighboringPlanes
                                                                                                                                                            [p1]
                                                                                                                                                            .begin(),
                                                                                                                                                            NeighboringPlanes
                                                                                                                                                            [p1]
                                                                                                                                                            .end(),
                                                                                                                                                            p11)
                                                                                                                                                        !=
                                                                                                                                                        NeighboringPlanes
                                                                                                                                                        [p1]
                                                                                                                                                        .end());
                                                                                                                                    if
                                                                                                                                    (isLateral
                                                                                                                                        &&
                                                                                                                                        inNeighborhood) {
                                                                                                                                        // Look into the shared planes of p11
                                                                                                                                        for
                                                                                                                                        (unsigned
                                                                                                                                            int
                                                                                                                                            t =
                                                                                                                                                    0;
                                                                                                                                            t <
                                                                                                                                            SharedPlanes
                                                                                                                                            [p11]
                                                                                                                                            .size()
                                                                                                                                            ; t
                                                                                                                                            ++) {
                                                                                                                                            int
                                                                                                                                                    p12
                                                                                                                                                            = SharedPlanes
                                                                                                                                                            [p11]
                                                                                                                                                            [t];
                                                                                                                                            if
                                                                                                                                            (p12
                                                                                                                                                !=
                                                                                                                                                p11
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p10
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p9
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p8
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p7
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p6
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p5
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p4
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p3
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p2
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p1) {
                                                                                                                                                bool
                                                                                                                                                        isLateral
                                                                                                                                                                = (
                                                                                                                                                                    SharedPlanes
                                                                                                                                                                    [p12]
                                                                                                                                                                    .size()
                                                                                                                                                                    >=
                                                                                                                                                                    AllPlanes
                                                                                                                                                                    [p12]
                                                                                                                                                                    .size());
                                                                                                                                                bool
                                                                                                                                                        inNeighborhood
                                                                                                                                                                = (
                                                                                                                                                                    std::find(
                                                                                                                                                                        NeighboringPlanes
                                                                                                                                                                        [p1]
                                                                                                                                                                        .begin(),
                                                                                                                                                                        NeighboringPlanes
                                                                                                                                                                        [p1]
                                                                                                                                                                        .end(),
                                                                                                                                                                        p12)
                                                                                                                                                                    !=
                                                                                                                                                                    NeighboringPlanes
                                                                                                                                                                    [p1]
                                                                                                                                                                    .end());
                                                                                                                                                if
                                                                                                                                                (isLateral
                                                                                                                                                    &&
                                                                                                                                                    inNeighborhood) {
                                                                                                                                                    // This means that all the 12 planes are lateral planes
                                                                                                                                                    std::vector
                                                                                                                                                            <int>
                                                                                                                                                            PlanesListTmp
                                                                                                                                                                    = {
                                                                                                                                                                        p1,
                                                                                                                                                                        p2,
                                                                                                                                                                        p3,
                                                                                                                                                                        p4,
                                                                                                                                                                        p5,
                                                                                                                                                                        p6,
                                                                                                                                                                        p7,
                                                                                                                                                                        p8,
                                                                                                                                                                        p9,
                                                                                                                                                                        p10,
                                                                                                                                                                        p11,
                                                                                                                                                                        p12
                                                                                                                                                                    };

                                                                                                                                                    /* Now we need to check if these 12 planes form a close loop(i.e.a cage)
                                                                                                                                                     They form a closed loop if the SharedPlanes of each plane contains five of all other planes */
                                                                                                                                                    int
                                                                                                                                                            NumPlanesInCage
                                                                                                                                                                    = 0;
                                                                                                                                                    // Check how many planes in the list satisfy the cage constraint
                                                                                                                                                    for
                                                                                                                                                    (unsigned
                                                                                                                                                        int
                                                                                                                                                        u =
                                                                                                                                                                0;
                                                                                                                                                        u <
                                                                                                                                                        PlanesListTmp
                                                                                                                                                        .size()
                                                                                                                                                        ; u
                                                                                                                                                        ++) {
                                                                                                                                                        int
                                                                                                                                                                ptmp
                                                                                                                                                                        = PlanesListTmp
                                                                                                                                                                        [u];
                                                                                                                                                        size_t
                                                                                                                                                                numSharedinList
                                                                                                                                                                        = NumSharedPlanesInSequence(
                                                                                                                                                                            PlanesListTmp,
                                                                                                                                                                            SharedPlanes,
                                                                                                                                                                            ptmp);
                                                                                                                                                        if
                                                                                                                                                        (numSharedinList
                                                                                                                                                            ==
                                                                                                                                                            AllPlanes
                                                                                                                                                            [ptmp]
                                                                                                                                                            .size()) {
                                                                                                                                                            NumPlanesInCage
                                                                                                                                                                    +=
                                                                                                                                                                    1;
                                                                                                                                                        }
                                                                                                                                                    }
                                                                                                                                                    // Check if the sequence of planes form a cage
                                                                                                                                                    if
                                                                                                                                                    (NumPlanesInCage
                                                                                                                                                        ==
                                                                                                                                                        PlanesListTmp
                                                                                                                                                        .size()) {
                                                                                                                                                        // This forms a closed loop so it is a cage
                                                                                                                                                        //std::sort(PlanesListTmp.begin(), PlanesListTmp.end());
                                                                                                                                                        // Check if the detected cage is already in the list
                                                                                                                                                        if
                                                                                                                                                        (!
                                                                                                                                                            isInVecofVec(
                                                                                                                                                                Cages12Plane,
                                                                                                                                                                PlanesListTmp)) {
                                                                                                                                                            Cages12Plane
                                                                                                                                                                    .push_back(
                                                                                                                                                                        PlanesListTmp);
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
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return Cages12Plane;
}


CageDetectionAlgorithms::VecOfVec CageDetectionAlgorithms::Detect14PlaneCages(
    VecOfVec AllPlanes, VecOfVec SharedPlanes, VecOfVec NeighboringPlanes) {
    VecOfVec Cages14Plane;
    unsigned int NumplanesToLookFor = 14;

    for (unsigned int i = 0; i < AllPlanes.size(); i++) {
        int p1 = i;
        bool isLateral = (SharedPlanes[p1].size() >= AllPlanes[p1].size());
        if (isLateral) {
            // Check if it is worth to search through the parental node P1
            if (NeighboringPlanes[p1].size() >= (NumplanesToLookFor - 1)) {
                // Look into the shared plane list of p1
                for (unsigned int j = 0; j < SharedPlanes[p1].size(); j++) {
                    int p2 = SharedPlanes[p1][j];
                    if (p2 != p1) {
                        bool isLateral = (SharedPlanes[p2].size() >= AllPlanes[p2].size());
                        bool inNeighborhood = (std::find(NeighboringPlanes[p1].begin(), NeighboringPlanes[p1].end(), p2)
                                               != NeighboringPlanes[p1].end());
                        if (isLateral && inNeighborhood) {
                            // Look into the shared planes of p2
                            for (unsigned int k = 0; k < SharedPlanes[p2].size(); k++) {
                                int p3 = SharedPlanes[p2][k];
                                if (p3 != p2 && p3 != p1) {
                                    bool isLateral = (SharedPlanes[p3].size() >= AllPlanes[p3].size());
                                    bool inNeighborhood = (std::find(NeighboringPlanes[p1].begin(),
                                                                     NeighboringPlanes[p1].end(),
                                                                     p3) != NeighboringPlanes[p1].end());
                                    if (isLateral && inNeighborhood) {
                                        // Look into the shared planes of p3
                                        for (unsigned int l = 0; l < SharedPlanes[p3].size(); l++) {
                                            int p4 = SharedPlanes[p3][l];
                                            if (p4 != p3 && p4 != p2 && p4 != p1) {
                                                bool isLateral = (SharedPlanes[p4].size() >= AllPlanes[p4].size());
                                                bool inNeighborhood = (std::find(NeighboringPlanes[p1].begin(),
                                                                           NeighboringPlanes[p1].end(),
                                                                           p4) != NeighboringPlanes[p1].end());
                                                if (isLateral && inNeighborhood) {
                                                    // Look into the shared planes of p4
                                                    for (unsigned int m = 0; m < SharedPlanes[p4].size(); m++) {
                                                        int p5 = SharedPlanes[p4][m];
                                                        if (p5 != p4 && p5 != p3 && p5 != p2 && p5 != p1) {
                                                            bool isLateral = (
                                                                SharedPlanes[p5].size() >= AllPlanes[p5].size());
                                                            bool inNeighborhood = (
                                                                std::find(NeighboringPlanes[p1].begin(),
                                                                          NeighboringPlanes[p1].end(),
                                                                          p5) != NeighboringPlanes[p1].end());
                                                            if (isLateral && inNeighborhood) {
                                                                // Look into the shared planes of p5
                                                                for (unsigned int n = 0; n < SharedPlanes[p5].size(); n
                                                                     ++) {
                                                                    int p6 = SharedPlanes[p5][n];
                                                                    if (p6 != p5 && p6 != p4 && p6 != p3 && p6 != p2 &&
                                                                        p6 != p1) {
                                                                        bool isLateral = (
                                                                            SharedPlanes[p6].size() >= AllPlanes[p6].
                                                                            size());
                                                                        bool inNeighborhood = (
                                                                            std::find(NeighboringPlanes[p1].begin(),
                                                                                NeighboringPlanes[p1].end(),
                                                                                p6) != NeighboringPlanes[p1].end());
                                                                        if (isLateral && inNeighborhood) {
                                                                            // Look into the shared planes of p6
                                                                            for (unsigned int o = 0;
                                                                                o < SharedPlanes[p6].size(); o++) {
                                                                                int p7 = SharedPlanes[p6][o];
                                                                                if (p7 != p6 && p7 != p5 && p7 != p4 &&
                                                                                    p7 != p3 && p7 != p2 && p7 != p1) {
                                                                                    bool isLateral = (
                                                                                        SharedPlanes[p7].size() >=
                                                                                        AllPlanes[p7].size());
                                                                                    bool inNeighborhood = (
                                                                                        std::find(
                                                                                            NeighboringPlanes[p1].
                                                                                            begin(),
                                                                                            NeighboringPlanes[p1].end(),
                                                                                            p7) != NeighboringPlanes[p1]
                                                                                        .end());
                                                                                    if (isLateral && inNeighborhood) {
                                                                                        // Look into the shared planes of p7
                                                                                        for (unsigned int p = 0;
                                                                                            p < SharedPlanes[p7].size();
                                                                                            p++) {
                                                                                            int p8 = SharedPlanes[p7][
                                                                                                p];
                                                                                            if (p8 != p7 && p8 != p6 &&
                                                                                                p8 != p5 && p8 != p4 &&
                                                                                                p8 != p3 && p8 != p2 &&
                                                                                                p8 != p1) {
                                                                                                bool isLateral = (
                                                                                                    SharedPlanes[p8].
                                                                                                    size() >= AllPlanes[
                                                                                                        p8].size());
                                                                                                bool inNeighborhood = (
                                                                                                    std::find(
                                                                                                        NeighboringPlanes
                                                                                                        [p1].begin(),
                                                                                                        NeighboringPlanes
                                                                                                        [p1].end(), p8)
                                                                                                    != NeighboringPlanes
                                                                                                    [p1].end());
                                                                                                if (isLateral &&
                                                                                                    inNeighborhood) {
                                                                                                    // Look into the shared planes of p8
                                                                                                    for (unsigned int q
                                                                                                                = 0;
                                                                                                        q < SharedPlanes
                                                                                                        [p8].size(); q
                                                                                                        ++) {
                                                                                                        int p9 =
                                                                                                                SharedPlanes
                                                                                                                [p8][q];
                                                                                                        if (p9 != p8 &&
                                                                                                            p9 != p7 &&
                                                                                                            p9 != p6 &&
                                                                                                            p9 != p5 &&
                                                                                                            p9 != p4 &&
                                                                                                            p9 != p3 &&
                                                                                                            p9 != p2 &&
                                                                                                            p9 != p1) {
                                                                                                            bool
                                                                                                                    isLateral
                                                                                                                            = (
                                                                                                                                SharedPlanes
                                                                                                                                [p9]
                                                                                                                                .size()
                                                                                                                                >=
                                                                                                                                AllPlanes
                                                                                                                                [p9]
                                                                                                                                .size());
                                                                                                            bool
                                                                                                                    inNeighborhood
                                                                                                                            = (
                                                                                                                                std::find(
                                                                                                                                    NeighboringPlanes
                                                                                                                                    [p1]
                                                                                                                                    .begin(),
                                                                                                                                    NeighboringPlanes
                                                                                                                                    [p1]
                                                                                                                                    .end(),
                                                                                                                                    p9)
                                                                                                                                !=
                                                                                                                                NeighboringPlanes
                                                                                                                                [p1]
                                                                                                                                .end());
                                                                                                            if (
                                                                                                                isLateral
                                                                                                                &&
                                                                                                                inNeighborhood) {
                                                                                                                // Look into the shared planes of p9
                                                                                                                for (
                                                                                                                    unsigned
                                                                                                                    int
                                                                                                                    r =
                                                                                                                            0;
                                                                                                                    r <
                                                                                                                    SharedPlanes
                                                                                                                    [p9]
                                                                                                                    .size()
                                                                                                                    ; r
                                                                                                                    ++) {
                                                                                                                    int
                                                                                                                            p10
                                                                                                                                    = SharedPlanes
                                                                                                                                    [p9]
                                                                                                                                    [r];
                                                                                                                    if (
                                                                                                                        p10
                                                                                                                        !=
                                                                                                                        p9
                                                                                                                        &&
                                                                                                                        p10
                                                                                                                        !=
                                                                                                                        p8
                                                                                                                        &&
                                                                                                                        p10
                                                                                                                        !=
                                                                                                                        p7
                                                                                                                        &&
                                                                                                                        p10
                                                                                                                        !=
                                                                                                                        p6
                                                                                                                        &&
                                                                                                                        p10
                                                                                                                        !=
                                                                                                                        p5
                                                                                                                        &&
                                                                                                                        p10
                                                                                                                        !=
                                                                                                                        p4
                                                                                                                        &&
                                                                                                                        p10
                                                                                                                        !=
                                                                                                                        p3
                                                                                                                        &&
                                                                                                                        p10
                                                                                                                        !=
                                                                                                                        p2
                                                                                                                        &&
                                                                                                                        p10
                                                                                                                        !=
                                                                                                                        p1) {
                                                                                                                        bool
                                                                                                                                isLateral
                                                                                                                                        = (
                                                                                                                                            SharedPlanes
                                                                                                                                            [p10]
                                                                                                                                            .size()
                                                                                                                                            >=
                                                                                                                                            AllPlanes
                                                                                                                                            [p10]
                                                                                                                                            .size());
                                                                                                                        bool
                                                                                                                                inNeighborhood
                                                                                                                                        = (
                                                                                                                                            std::find(
                                                                                                                                                NeighboringPlanes
                                                                                                                                                [p1]
                                                                                                                                                .begin(),
                                                                                                                                                NeighboringPlanes
                                                                                                                                                [p1]
                                                                                                                                                .end(),
                                                                                                                                                p10)
                                                                                                                                            !=
                                                                                                                                            NeighboringPlanes
                                                                                                                                            [p1]
                                                                                                                                            .end());
                                                                                                                        if
                                                                                                                        (isLateral
                                                                                                                            &&
                                                                                                                            inNeighborhood) {
                                                                                                                            // Look into the shared planes of p10
                                                                                                                            for
                                                                                                                            (unsigned
                                                                                                                                int
                                                                                                                                s =
                                                                                                                                        0;
                                                                                                                                s <
                                                                                                                                SharedPlanes
                                                                                                                                [p10]
                                                                                                                                .size()
                                                                                                                                ; s
                                                                                                                                ++) {
                                                                                                                                int
                                                                                                                                        p11
                                                                                                                                                = SharedPlanes
                                                                                                                                                [p10]
                                                                                                                                                [s];
                                                                                                                                if
                                                                                                                                (p11
                                                                                                                                    !=
                                                                                                                                    p10
                                                                                                                                    &&
                                                                                                                                    p11
                                                                                                                                    !=
                                                                                                                                    p9
                                                                                                                                    &&
                                                                                                                                    p11
                                                                                                                                    !=
                                                                                                                                    p8
                                                                                                                                    &&
                                                                                                                                    p11
                                                                                                                                    !=
                                                                                                                                    p7
                                                                                                                                    &&
                                                                                                                                    p11
                                                                                                                                    !=
                                                                                                                                    p6
                                                                                                                                    &&
                                                                                                                                    p11
                                                                                                                                    !=
                                                                                                                                    p5
                                                                                                                                    &&
                                                                                                                                    p11
                                                                                                                                    !=
                                                                                                                                    p4
                                                                                                                                    &&
                                                                                                                                    p11
                                                                                                                                    !=
                                                                                                                                    p3
                                                                                                                                    &&
                                                                                                                                    p11
                                                                                                                                    !=
                                                                                                                                    p2
                                                                                                                                    &&
                                                                                                                                    p11
                                                                                                                                    !=
                                                                                                                                    p1) {
                                                                                                                                    bool
                                                                                                                                            isLateral
                                                                                                                                                    = (
                                                                                                                                                        SharedPlanes
                                                                                                                                                        [p11]
                                                                                                                                                        .size()
                                                                                                                                                        >=
                                                                                                                                                        AllPlanes
                                                                                                                                                        [p11]
                                                                                                                                                        .size());
                                                                                                                                    bool
                                                                                                                                            inNeighborhood
                                                                                                                                                    = (
                                                                                                                                                        std::find(
                                                                                                                                                            NeighboringPlanes
                                                                                                                                                            [p1]
                                                                                                                                                            .begin(),
                                                                                                                                                            NeighboringPlanes
                                                                                                                                                            [p1]
                                                                                                                                                            .end(),
                                                                                                                                                            p11)
                                                                                                                                                        !=
                                                                                                                                                        NeighboringPlanes
                                                                                                                                                        [p1]
                                                                                                                                                        .end());
                                                                                                                                    if
                                                                                                                                    (isLateral
                                                                                                                                        &&
                                                                                                                                        inNeighborhood) {
                                                                                                                                        // Look into the shared planes of p11
                                                                                                                                        for
                                                                                                                                        (unsigned
                                                                                                                                            int
                                                                                                                                            t =
                                                                                                                                                    0;
                                                                                                                                            t <
                                                                                                                                            SharedPlanes
                                                                                                                                            [p11]
                                                                                                                                            .size()
                                                                                                                                            ; t
                                                                                                                                            ++) {
                                                                                                                                            int
                                                                                                                                                    p12
                                                                                                                                                            = SharedPlanes
                                                                                                                                                            [p11]
                                                                                                                                                            [t];
                                                                                                                                            if
                                                                                                                                            (p12
                                                                                                                                                !=
                                                                                                                                                p11
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p10
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p9
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p8
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p7
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p6
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p5
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p4
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p3
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p2
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p1) {
                                                                                                                                                bool
                                                                                                                                                        isLateral
                                                                                                                                                                = (
                                                                                                                                                                    SharedPlanes
                                                                                                                                                                    [p12]
                                                                                                                                                                    .size()
                                                                                                                                                                    >=
                                                                                                                                                                    AllPlanes
                                                                                                                                                                    [p12]
                                                                                                                                                                    .size());
                                                                                                                                                bool
                                                                                                                                                        inNeighborhood
                                                                                                                                                                = (
                                                                                                                                                                    std::find(
                                                                                                                                                                        NeighboringPlanes
                                                                                                                                                                        [p1]
                                                                                                                                                                        .begin(),
                                                                                                                                                                        NeighboringPlanes
                                                                                                                                                                        [p1]
                                                                                                                                                                        .end(),
                                                                                                                                                                        p12)
                                                                                                                                                                    !=
                                                                                                                                                                    NeighboringPlanes
                                                                                                                                                                    [p1]
                                                                                                                                                                    .end());
                                                                                                                                                if
                                                                                                                                                (isLateral
                                                                                                                                                    &&
                                                                                                                                                    inNeighborhood) {
                                                                                                                                                    // Look into the shared planes of p12
                                                                                                                                                    for
                                                                                                                                                    (unsigned
                                                                                                                                                        int
                                                                                                                                                        u =
                                                                                                                                                                0;
                                                                                                                                                        u <
                                                                                                                                                        SharedPlanes
                                                                                                                                                        [p12]
                                                                                                                                                        .size()
                                                                                                                                                        ; u
                                                                                                                                                        ++) {
                                                                                                                                                        int
                                                                                                                                                                p13
                                                                                                                                                                        = SharedPlanes
                                                                                                                                                                        [p12]
                                                                                                                                                                        [u];
                                                                                                                                                        if
                                                                                                                                                        (p13
                                                                                                                                                            !=
                                                                                                                                                            p12
                                                                                                                                                            &&
                                                                                                                                                            p13
                                                                                                                                                            !=
                                                                                                                                                            p11
                                                                                                                                                            &&
                                                                                                                                                            p13
                                                                                                                                                            !=
                                                                                                                                                            p10
                                                                                                                                                            &&
                                                                                                                                                            p13
                                                                                                                                                            !=
                                                                                                                                                            p9
                                                                                                                                                            &&
                                                                                                                                                            p13
                                                                                                                                                            !=
                                                                                                                                                            p8
                                                                                                                                                            &&
                                                                                                                                                            p13
                                                                                                                                                            !=
                                                                                                                                                            p7
                                                                                                                                                            &&
                                                                                                                                                            p13
                                                                                                                                                            !=
                                                                                                                                                            p6
                                                                                                                                                            &&
                                                                                                                                                            p13
                                                                                                                                                            !=
                                                                                                                                                            p5
                                                                                                                                                            &&
                                                                                                                                                            p13
                                                                                                                                                            !=
                                                                                                                                                            p4
                                                                                                                                                            &&
                                                                                                                                                            p13
                                                                                                                                                            !=
                                                                                                                                                            p3
                                                                                                                                                            &&
                                                                                                                                                            p13
                                                                                                                                                            !=
                                                                                                                                                            p2
                                                                                                                                                            &&
                                                                                                                                                            p13
                                                                                                                                                            !=
                                                                                                                                                            p1) {
                                                                                                                                                            bool
                                                                                                                                                                    isLateral
                                                                                                                                                                            = (
                                                                                                                                                                                SharedPlanes
                                                                                                                                                                                [p13]
                                                                                                                                                                                .size()
                                                                                                                                                                                >=
                                                                                                                                                                                AllPlanes
                                                                                                                                                                                [p13]
                                                                                                                                                                                .size());
                                                                                                                                                            bool
                                                                                                                                                                    inNeighborhood
                                                                                                                                                                            = (
                                                                                                                                                                                std::find(
                                                                                                                                                                                    NeighboringPlanes
                                                                                                                                                                                    [p1]
                                                                                                                                                                                    .begin(),
                                                                                                                                                                                    NeighboringPlanes
                                                                                                                                                                                    [p1]
                                                                                                                                                                                    .end(),
                                                                                                                                                                                    p13)
                                                                                                                                                                                !=
                                                                                                                                                                                NeighboringPlanes
                                                                                                                                                                                [p1]
                                                                                                                                                                                .end());
                                                                                                                                                            if
                                                                                                                                                            (isLateral
                                                                                                                                                                &&
                                                                                                                                                                inNeighborhood) {
                                                                                                                                                                // Look into the shared planes of p13
                                                                                                                                                                for
                                                                                                                                                                (unsigned
                                                                                                                                                                    int
                                                                                                                                                                    v =
                                                                                                                                                                            0;
                                                                                                                                                                    v <
                                                                                                                                                                    SharedPlanes
                                                                                                                                                                    [p13]
                                                                                                                                                                    .size()
                                                                                                                                                                    ; v
                                                                                                                                                                    ++) {
                                                                                                                                                                    int
                                                                                                                                                                            p14
                                                                                                                                                                                    = SharedPlanes
                                                                                                                                                                                    [p13]
                                                                                                                                                                                    [v];
                                                                                                                                                                    if
                                                                                                                                                                    (p14
                                                                                                                                                                        !=
                                                                                                                                                                        p13
                                                                                                                                                                        &&
                                                                                                                                                                        p14
                                                                                                                                                                        !=
                                                                                                                                                                        p12
                                                                                                                                                                        &&
                                                                                                                                                                        p14
                                                                                                                                                                        !=
                                                                                                                                                                        p11
                                                                                                                                                                        &&
                                                                                                                                                                        p14
                                                                                                                                                                        !=
                                                                                                                                                                        p10
                                                                                                                                                                        &&
                                                                                                                                                                        p14
                                                                                                                                                                        !=
                                                                                                                                                                        p9
                                                                                                                                                                        &&
                                                                                                                                                                        p14
                                                                                                                                                                        !=
                                                                                                                                                                        p8
                                                                                                                                                                        &&
                                                                                                                                                                        p14
                                                                                                                                                                        !=
                                                                                                                                                                        p7
                                                                                                                                                                        &&
                                                                                                                                                                        p14
                                                                                                                                                                        !=
                                                                                                                                                                        p6
                                                                                                                                                                        &&
                                                                                                                                                                        p14
                                                                                                                                                                        !=
                                                                                                                                                                        p5
                                                                                                                                                                        &&
                                                                                                                                                                        p14
                                                                                                                                                                        !=
                                                                                                                                                                        p4
                                                                                                                                                                        &&
                                                                                                                                                                        p14
                                                                                                                                                                        !=
                                                                                                                                                                        p3
                                                                                                                                                                        &&
                                                                                                                                                                        p14
                                                                                                                                                                        !=
                                                                                                                                                                        p2
                                                                                                                                                                        &&
                                                                                                                                                                        p14
                                                                                                                                                                        !=
                                                                                                                                                                        p1) {
                                                                                                                                                                        bool
                                                                                                                                                                                isLateral
                                                                                                                                                                                        = (
                                                                                                                                                                                            SharedPlanes
                                                                                                                                                                                            [p14]
                                                                                                                                                                                            .size()
                                                                                                                                                                                            >=
                                                                                                                                                                                            AllPlanes
                                                                                                                                                                                            [p14]
                                                                                                                                                                                            .size());
                                                                                                                                                                        bool
                                                                                                                                                                                inNeighborhood
                                                                                                                                                                                        = (
                                                                                                                                                                                            std::find(
                                                                                                                                                                                                NeighboringPlanes
                                                                                                                                                                                                [p1]
                                                                                                                                                                                                .begin(),
                                                                                                                                                                                                NeighboringPlanes
                                                                                                                                                                                                [p1]
                                                                                                                                                                                                .end(),
                                                                                                                                                                                                p14)
                                                                                                                                                                                            !=
                                                                                                                                                                                            NeighboringPlanes
                                                                                                                                                                                            [p1]
                                                                                                                                                                                            .end());
                                                                                                                                                                        if
                                                                                                                                                                        (isLateral
                                                                                                                                                                            &&
                                                                                                                                                                            inNeighborhood) {
                                                                                                                                                                            // This means that all the 14 planes are lateral planes
                                                                                                                                                                            std::vector
                                                                                                                                                                                    <int>
                                                                                                                                                                                    PlanesListTmp
                                                                                                                                                                                            = {
                                                                                                                                                                                                p1,
                                                                                                                                                                                                p2,
                                                                                                                                                                                                p3,
                                                                                                                                                                                                p4,
                                                                                                                                                                                                p5,
                                                                                                                                                                                                p6,
                                                                                                                                                                                                p7,
                                                                                                                                                                                                p8,
                                                                                                                                                                                                p9,
                                                                                                                                                                                                p10,
                                                                                                                                                                                                p11,
                                                                                                                                                                                                p12,
                                                                                                                                                                                                p13,
                                                                                                                                                                                                p14
                                                                                                                                                                                            };

                                                                                                                                                                            /* Now we need to check if these 14 planes form a close loop(i.e.a cage)
                                                                                                                                                                             They form a closed loop if the SharedPlanes of each plane contains five of all other planes */
                                                                                                                                                                            int
                                                                                                                                                                                    NumPlanesInCage
                                                                                                                                                                                            = 0;
                                                                                                                                                                            // Check how many planes in the list satisfy the cage constraint
                                                                                                                                                                            for
                                                                                                                                                                            (int
                                                                                                                                                                                w =
                                                                                                                                                                                        0;
                                                                                                                                                                                w <
                                                                                                                                                                                PlanesListTmp
                                                                                                                                                                                .size()
                                                                                                                                                                                ; w
                                                                                                                                                                                ++) {
                                                                                                                                                                                int
                                                                                                                                                                                        ptmp
                                                                                                                                                                                                = PlanesListTmp
                                                                                                                                                                                                [w];
                                                                                                                                                                                size_t
                                                                                                                                                                                        numSharedinList
                                                                                                                                                                                                = NumSharedPlanesInSequence(
                                                                                                                                                                                                    PlanesListTmp,
                                                                                                                                                                                                    SharedPlanes,
                                                                                                                                                                                                    ptmp);
                                                                                                                                                                                if
                                                                                                                                                                                (numSharedinList
                                                                                                                                                                                    ==
                                                                                                                                                                                    AllPlanes
                                                                                                                                                                                    [ptmp]
                                                                                                                                                                                    .size()) {
                                                                                                                                                                                    NumPlanesInCage
                                                                                                                                                                                            +=
                                                                                                                                                                                            1;
                                                                                                                                                                                }
                                                                                                                                                                            }
                                                                                                                                                                            // Check if the sequence of planes form a cage
                                                                                                                                                                            if
                                                                                                                                                                            (NumPlanesInCage
                                                                                                                                                                                ==
                                                                                                                                                                                PlanesListTmp
                                                                                                                                                                                .size()) {
                                                                                                                                                                                // This forms a closed loop so it is a cage
                                                                                                                                                                                //std::sort(PlanesListTmp.begin(), PlanesListTmp.end());
                                                                                                                                                                                // Check if the detected cage is already in the list
                                                                                                                                                                                if
                                                                                                                                                                                (!
                                                                                                                                                                                    isInVecofVec(
                                                                                                                                                                                        Cages14Plane,
                                                                                                                                                                                        PlanesListTmp)) {
                                                                                                                                                                                    Cages14Plane
                                                                                                                                                                                            .push_back(
                                                                                                                                                                                                PlanesListTmp);
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
    return Cages14Plane;
}


CageDetectionAlgorithms::VecOfVec CageDetectionAlgorithms::RemoveDuplicates(
    VecOfVec InputVec, VecOfVec &InputVecRemDupUnsorted, VecOfVec &NumOccEdPlanesAll,
    VecOfVec &NumOccEdPlanesUniqueUnsorted) {
    // The input vector is not sorted
    VecOfVec InputVecRemDupSorted;
    for (int i = 0; i < InputVec.size(); i++) {
        std::vector<int> unsortedVec = InputVec[i];
        std::sort(begin(InputVec[i]), end(InputVec[i]));
        /*InputVecRemDupSorted.push_back(InputVec[i]);
        InputVecRemDupUnsorted.push_back(unsortedVec);*/


        if (!isInVecofVec(InputVecRemDupSorted, InputVec[i])) {
            InputVecRemDupSorted.push_back(InputVec[i]);
            InputVecRemDupUnsorted.push_back(unsortedVec);
            NumOccEdPlanesUniqueUnsorted.push_back(NumOccEdPlanesAll[i]);
        }
    }
    return InputVecRemDupSorted;
}

std::vector<CageDetectionAlgorithms::Cage> CageDetectionAlgorithms::RemoveDuplicatesCages(
    std::vector<CageDetectionAlgorithms::Cage> InputVec,
    std::vector<CageDetectionAlgorithms::Cage> &InputVecRemDupUnsorted) {
    // The input vector is not sorted
    VecOfVec InputVecRemDupSorted;
    std::vector<CageDetectionAlgorithms::Cage> CagesDuplicatesRemovedSorted;
    CageDetectionAlgorithms::Cage Tmpcage;
    for (int i = 0; i < InputVec.size(); i++) {
        //std::vector<int>  unsortedVec = InputVec[i].planes;
        Tmpcage = InputVec[i];
        std::sort(begin(InputVec[i].planes), end(InputVec[i].planes));
        /*InputVecRemDupSorted.push_back(InputVec[i]);
        InputVecRemDupUnsorted.push_back(unsortedVec);*/
        if (!isInVecofVec(InputVecRemDupSorted, InputVec[i].planes)) {
            InputVecRemDupSorted.push_back(InputVec[i].planes);
            CagesDuplicatesRemovedSorted.push_back(InputVec[i]);

            InputVecRemDupUnsorted.push_back(Tmpcage);
        }
    }
    return CagesDuplicatesRemovedSorted;
}


int CageDetectionAlgorithms::OverlapWithPlanes(std::vector<int> Plast, std::vector<int> Pcur, VecOfVec SharedPlanes) {
    std::vector<int> prob;
    for (unsigned int i = 0; i < Pcur.size(); i++) {
        int olap = 0;
        if (std::count(Plast.begin(), Plast.end(), Pcur[i]) == 0)
        // that means the new sharing point shuold not be in the previously found's planed list
        {
            for (unsigned int j = 0; j < SharedPlanes[Pcur[i]].size(); j++) {
                if (std::count(Plast.begin(), Plast.end(), SharedPlanes[Pcur[i]][j])) {
                    olap += 1;
                }
            }
        }
        prob.push_back(olap);
    }

    // find the argmax of the prob vector
    int MaxIndx = 0;

    for (unsigned int i = 1; i < prob.size(); i++) {
        if (prob[i] >= prob[i - 1]) {
            MaxIndx = i;
        }
    }
    return Pcur[MaxIndx];
}


CageDetectionAlgorithms::VecOfVec CageDetectionAlgorithms::Detect14PlaneCagesProbMove(
    VecOfVec AllPlanes, VecOfVec SharedPlanes, VecOfVec NeighboringPlanes) {
    VecOfVec Cages14Plane;
    unsigned int NumplanesToLookFor = 14;

    for (unsigned int i = 0; i < AllPlanes.size(); i++) {
        int p1 = i;
        bool isLateral = (SharedPlanes[p1].size() >= AllPlanes[p1].size());
        if (isLateral) {
            // Check if it is worth to search through the parental node P1
            if (NeighboringPlanes[p1].size() >= (NumplanesToLookFor - 1)) {
                // Look into the shared plane list of p1
                for (unsigned int j = 0; j < SharedPlanes[p1].size(); j++) {
                    int p2 = SharedPlanes[p1][j];
                    if (p2 != p1) {
                        bool isLateral = (SharedPlanes[p2].size() >= AllPlanes[p2].size());
                        bool inNeighborhood = (std::find(NeighboringPlanes[p1].begin(), NeighboringPlanes[p1].end(), p2)
                                               != NeighboringPlanes[p1].end());
                        if (isLateral && inNeighborhood) {
                            // Look into the shared planes of p2
                            for (unsigned int k = 0; k < SharedPlanes[p2].size(); k++) {
                                // Select p3 such that it has the largest number of shared edges with the previously found planes

                                //int p3 = SharedPlanes[p2][k];
                                int p3 = OverlapWithPlanes({p1, p2}, SharedPlanes[p2], SharedPlanes);
                                if (p3 != p2 && p3 != p1) {
                                    bool isLateral = (SharedPlanes[p3].size() >= AllPlanes[p3].size());
                                    bool inNeighborhood = (std::find(NeighboringPlanes[p1].begin(),
                                                                     NeighboringPlanes[p1].end(),
                                                                     p3) != NeighboringPlanes[p1].end());
                                    if (isLateral && inNeighborhood) {
                                        // Look into the shared planes of p3
                                        for (unsigned int l = 0; l < SharedPlanes[p3].size(); l++) {
                                            //int p4 = SharedPlanes[p3][l];
                                            int p4 = OverlapWithPlanes({p1, p2, p3}, SharedPlanes[p3], SharedPlanes);
                                            if (p4 != p3 && p4 != p2 && p4 != p1) {
                                                bool isLateral = (SharedPlanes[p4].size() >= AllPlanes[p4].size());
                                                bool inNeighborhood = (std::find(NeighboringPlanes[p1].begin(),
                                                                           NeighboringPlanes[p1].end(),
                                                                           p4) != NeighboringPlanes[p1].end());
                                                if (isLateral && inNeighborhood) {
                                                    // Look into the shared planes of p4
                                                    for (unsigned int m = 0; m < SharedPlanes[p4].size(); m++) {
                                                        //int p5 = SharedPlanes[p4][m];
                                                        int p5 = OverlapWithPlanes(
                                                            {p1, p2, p3, p4}, SharedPlanes[p4], SharedPlanes);
                                                        if (p5 != p4 && p5 != p3 && p5 != p2 && p5 != p1) {
                                                            bool isLateral = (
                                                                SharedPlanes[p5].size() >= AllPlanes[p5].size());
                                                            bool inNeighborhood = (
                                                                std::find(NeighboringPlanes[p1].begin(),
                                                                          NeighboringPlanes[p1].end(),
                                                                          p5) != NeighboringPlanes[p1].end());
                                                            if (isLateral && inNeighborhood) {
                                                                // Look into the shared planes of p5
                                                                for (unsigned int n = 0; n < SharedPlanes[p5].size(); n
                                                                     ++) {
                                                                    //int p6 = SharedPlanes[p5][n];
                                                                    int p6 = OverlapWithPlanes(
                                                                        {p1, p2, p3, p4, p5}, SharedPlanes[p5],
                                                                        SharedPlanes);
                                                                    if (p6 != p5 && p6 != p4 && p6 != p3 && p6 != p2 &&
                                                                        p6 != p1) {
                                                                        bool isLateral = (
                                                                            SharedPlanes[p6].size() >= AllPlanes[p6].
                                                                            size());
                                                                        bool inNeighborhood = (
                                                                            std::find(NeighboringPlanes[p1].begin(),
                                                                                NeighboringPlanes[p1].end(),
                                                                                p6) != NeighboringPlanes[p1].end());
                                                                        if (isLateral && inNeighborhood) {
                                                                            // Look into the shared planes of p6
                                                                            for (unsigned int o = 0;
                                                                                o < SharedPlanes[p6].size(); o++) {
                                                                                //int p7 = SharedPlanes[p6][o];
                                                                                int p7 = OverlapWithPlanes(
                                                                                    {p1, p2, p3, p4, p5, p6},
                                                                                    SharedPlanes[p6], SharedPlanes);
                                                                                if (p7 != p6 && p7 != p5 && p7 != p4 &&
                                                                                    p7 != p3 && p7 != p2 && p7 != p1) {
                                                                                    bool isLateral = (
                                                                                        SharedPlanes[p7].size() >=
                                                                                        AllPlanes[p7].size());
                                                                                    bool inNeighborhood = (
                                                                                        std::find(
                                                                                            NeighboringPlanes[p1].
                                                                                            begin(),
                                                                                            NeighboringPlanes[p1].end(),
                                                                                            p7) != NeighboringPlanes[p1]
                                                                                        .end());
                                                                                    if (isLateral && inNeighborhood) {
                                                                                        // Look into the shared planes of p7
                                                                                        for (unsigned int p = 0;
                                                                                            p < SharedPlanes[p7].size();
                                                                                            p++) {
                                                                                            //int p8 = SharedPlanes[p7][p];
                                                                                            int p8 = OverlapWithPlanes(
                                                                                                {
                                                                                                    p1, p2, p3, p4, p5,
                                                                                                    p6, p7
                                                                                                }, SharedPlanes[p7],
                                                                                                SharedPlanes);
                                                                                            if (p8 != p7 && p8 != p6 &&
                                                                                                p8 != p5 && p8 != p4 &&
                                                                                                p8 != p3 && p8 != p2 &&
                                                                                                p8 != p1) {
                                                                                                bool isLateral = (
                                                                                                    SharedPlanes[p8].
                                                                                                    size() >= AllPlanes[
                                                                                                        p8].size());
                                                                                                bool inNeighborhood = (
                                                                                                    std::find(
                                                                                                        NeighboringPlanes
                                                                                                        [p1].begin(),
                                                                                                        NeighboringPlanes
                                                                                                        [p1].end(), p8)
                                                                                                    != NeighboringPlanes
                                                                                                    [p1].end());
                                                                                                if (isLateral &&
                                                                                                    inNeighborhood) {
                                                                                                    // Look into the shared planes of p8
                                                                                                    for (unsigned int q
                                                                                                                = 0;
                                                                                                        q < SharedPlanes
                                                                                                        [p8].size(); q
                                                                                                        ++) {
                                                                                                        //int p9 = SharedPlanes[p8][q];
                                                                                                        int p9 =
                                                                                                                OverlapWithPlanes(
                                                                                                                    {
                                                                                                                        p1,
                                                                                                                        p2,
                                                                                                                        p3,
                                                                                                                        p4,
                                                                                                                        p5,
                                                                                                                        p6,
                                                                                                                        p7,
                                                                                                                        p8
                                                                                                                    },
                                                                                                                    SharedPlanes
                                                                                                                    [p8],
                                                                                                                    SharedPlanes);
                                                                                                        if (p9 != p8 &&
                                                                                                            p9 != p7 &&
                                                                                                            p9 != p6 &&
                                                                                                            p9 != p5 &&
                                                                                                            p9 != p4 &&
                                                                                                            p9 != p3 &&
                                                                                                            p9 != p2 &&
                                                                                                            p9 != p1) {
                                                                                                            bool
                                                                                                                    isLateral
                                                                                                                            = (
                                                                                                                                SharedPlanes
                                                                                                                                [p9]
                                                                                                                                .size()
                                                                                                                                >=
                                                                                                                                AllPlanes
                                                                                                                                [p9]
                                                                                                                                .size());
                                                                                                            bool
                                                                                                                    inNeighborhood
                                                                                                                            = (
                                                                                                                                std::find(
                                                                                                                                    NeighboringPlanes
                                                                                                                                    [p1]
                                                                                                                                    .begin(),
                                                                                                                                    NeighboringPlanes
                                                                                                                                    [p1]
                                                                                                                                    .end(),
                                                                                                                                    p9)
                                                                                                                                !=
                                                                                                                                NeighboringPlanes
                                                                                                                                [p1]
                                                                                                                                .end());
                                                                                                            if (
                                                                                                                isLateral
                                                                                                                &&
                                                                                                                inNeighborhood) {
                                                                                                                // Look into the shared planes of p9
                                                                                                                for (
                                                                                                                    unsigned
                                                                                                                    int
                                                                                                                    r =
                                                                                                                            0;
                                                                                                                    r <
                                                                                                                    SharedPlanes
                                                                                                                    [p9]
                                                                                                                    .size()
                                                                                                                    ; r
                                                                                                                    ++) {
                                                                                                                    //int p10 = SharedPlanes[p9][r];
                                                                                                                    int
                                                                                                                            p10
                                                                                                                                    = OverlapWithPlanes(
                                                                                                                                        {
                                                                                                                                            p1,
                                                                                                                                            p2,
                                                                                                                                            p3,
                                                                                                                                            p4,
                                                                                                                                            p5,
                                                                                                                                            p6,
                                                                                                                                            p7,
                                                                                                                                            p8,
                                                                                                                                            p9
                                                                                                                                        },
                                                                                                                                        SharedPlanes
                                                                                                                                        [p9],
                                                                                                                                        SharedPlanes);
                                                                                                                    if (
                                                                                                                        p10
                                                                                                                        !=
                                                                                                                        p9
                                                                                                                        &&
                                                                                                                        p10
                                                                                                                        !=
                                                                                                                        p8
                                                                                                                        &&
                                                                                                                        p10
                                                                                                                        !=
                                                                                                                        p7
                                                                                                                        &&
                                                                                                                        p10
                                                                                                                        !=
                                                                                                                        p6
                                                                                                                        &&
                                                                                                                        p10
                                                                                                                        !=
                                                                                                                        p5
                                                                                                                        &&
                                                                                                                        p10
                                                                                                                        !=
                                                                                                                        p4
                                                                                                                        &&
                                                                                                                        p10
                                                                                                                        !=
                                                                                                                        p3
                                                                                                                        &&
                                                                                                                        p10
                                                                                                                        !=
                                                                                                                        p2
                                                                                                                        &&
                                                                                                                        p10
                                                                                                                        !=
                                                                                                                        p1) {
                                                                                                                        bool
                                                                                                                                isLateral
                                                                                                                                        = (
                                                                                                                                            SharedPlanes
                                                                                                                                            [p10]
                                                                                                                                            .size()
                                                                                                                                            >=
                                                                                                                                            AllPlanes
                                                                                                                                            [p10]
                                                                                                                                            .size());
                                                                                                                        bool
                                                                                                                                inNeighborhood
                                                                                                                                        = (
                                                                                                                                            std::find(
                                                                                                                                                NeighboringPlanes
                                                                                                                                                [p1]
                                                                                                                                                .begin(),
                                                                                                                                                NeighboringPlanes
                                                                                                                                                [p1]
                                                                                                                                                .end(),
                                                                                                                                                p10)
                                                                                                                                            !=
                                                                                                                                            NeighboringPlanes
                                                                                                                                            [p1]
                                                                                                                                            .end());
                                                                                                                        if
                                                                                                                        (isLateral
                                                                                                                            &&
                                                                                                                            inNeighborhood) {
                                                                                                                            // Look into the shared planes of p10
                                                                                                                            for
                                                                                                                            (unsigned
                                                                                                                                int
                                                                                                                                s =
                                                                                                                                        0;
                                                                                                                                s <
                                                                                                                                SharedPlanes
                                                                                                                                [p10]
                                                                                                                                .size()
                                                                                                                                ; s
                                                                                                                                ++) {
                                                                                                                                //int p11 = SharedPlanes[p10][s];
                                                                                                                                int
                                                                                                                                        p11
                                                                                                                                                = OverlapWithPlanes(
                                                                                                                                                    {
                                                                                                                                                        p1,
                                                                                                                                                        p2,
                                                                                                                                                        p3,
                                                                                                                                                        p4,
                                                                                                                                                        p5,
                                                                                                                                                        p6,
                                                                                                                                                        p7,
                                                                                                                                                        p8,
                                                                                                                                                        p9,
                                                                                                                                                        p10
                                                                                                                                                    },
                                                                                                                                                    SharedPlanes
                                                                                                                                                    [p10],
                                                                                                                                                    SharedPlanes);
                                                                                                                                if
                                                                                                                                (p11
                                                                                                                                    !=
                                                                                                                                    p10
                                                                                                                                    &&
                                                                                                                                    p11
                                                                                                                                    !=
                                                                                                                                    p9
                                                                                                                                    &&
                                                                                                                                    p11
                                                                                                                                    !=
                                                                                                                                    p8
                                                                                                                                    &&
                                                                                                                                    p11
                                                                                                                                    !=
                                                                                                                                    p7
                                                                                                                                    &&
                                                                                                                                    p11
                                                                                                                                    !=
                                                                                                                                    p6
                                                                                                                                    &&
                                                                                                                                    p11
                                                                                                                                    !=
                                                                                                                                    p5
                                                                                                                                    &&
                                                                                                                                    p11
                                                                                                                                    !=
                                                                                                                                    p4
                                                                                                                                    &&
                                                                                                                                    p11
                                                                                                                                    !=
                                                                                                                                    p3
                                                                                                                                    &&
                                                                                                                                    p11
                                                                                                                                    !=
                                                                                                                                    p2
                                                                                                                                    &&
                                                                                                                                    p11
                                                                                                                                    !=
                                                                                                                                    p1) {
                                                                                                                                    bool
                                                                                                                                            isLateral
                                                                                                                                                    = (
                                                                                                                                                        SharedPlanes
                                                                                                                                                        [p11]
                                                                                                                                                        .size()
                                                                                                                                                        >=
                                                                                                                                                        AllPlanes
                                                                                                                                                        [p11]
                                                                                                                                                        .size());
                                                                                                                                    bool
                                                                                                                                            inNeighborhood
                                                                                                                                                    = (
                                                                                                                                                        std::find(
                                                                                                                                                            NeighboringPlanes
                                                                                                                                                            [p1]
                                                                                                                                                            .begin(),
                                                                                                                                                            NeighboringPlanes
                                                                                                                                                            [p1]
                                                                                                                                                            .end(),
                                                                                                                                                            p11)
                                                                                                                                                        !=
                                                                                                                                                        NeighboringPlanes
                                                                                                                                                        [p1]
                                                                                                                                                        .end());
                                                                                                                                    if
                                                                                                                                    (isLateral
                                                                                                                                        &&
                                                                                                                                        inNeighborhood) {
                                                                                                                                        // Look into the shared planes of p11
                                                                                                                                        for
                                                                                                                                        (unsigned
                                                                                                                                            int
                                                                                                                                            t =
                                                                                                                                                    0;
                                                                                                                                            t <
                                                                                                                                            SharedPlanes
                                                                                                                                            [p11]
                                                                                                                                            .size()
                                                                                                                                            ; t
                                                                                                                                            ++) {
                                                                                                                                            //int p12 = SharedPlanes[p11][t];
                                                                                                                                            int
                                                                                                                                                    p12
                                                                                                                                                            = OverlapWithPlanes(
                                                                                                                                                                {
                                                                                                                                                                    p1,
                                                                                                                                                                    p2,
                                                                                                                                                                    p3,
                                                                                                                                                                    p4,
                                                                                                                                                                    p5,
                                                                                                                                                                    p6,
                                                                                                                                                                    p7,
                                                                                                                                                                    p8,
                                                                                                                                                                    p9,
                                                                                                                                                                    p10,
                                                                                                                                                                    p11
                                                                                                                                                                },
                                                                                                                                                                SharedPlanes
                                                                                                                                                                [p11],
                                                                                                                                                                SharedPlanes);
                                                                                                                                            if
                                                                                                                                            (p12
                                                                                                                                                !=
                                                                                                                                                p11
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p10
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p9
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p8
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p7
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p6
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p5
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p4
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p3
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p2
                                                                                                                                                &&
                                                                                                                                                p12
                                                                                                                                                !=
                                                                                                                                                p1) {
                                                                                                                                                bool
                                                                                                                                                        isLateral
                                                                                                                                                                = (
                                                                                                                                                                    SharedPlanes
                                                                                                                                                                    [p12]
                                                                                                                                                                    .size()
                                                                                                                                                                    >=
                                                                                                                                                                    AllPlanes
                                                                                                                                                                    [p12]
                                                                                                                                                                    .size());
                                                                                                                                                bool
                                                                                                                                                        inNeighborhood
                                                                                                                                                                = (
                                                                                                                                                                    std::find(
                                                                                                                                                                        NeighboringPlanes
                                                                                                                                                                        [p1]
                                                                                                                                                                        .begin(),
                                                                                                                                                                        NeighboringPlanes
                                                                                                                                                                        [p1]
                                                                                                                                                                        .end(),
                                                                                                                                                                        p12)
                                                                                                                                                                    !=
                                                                                                                                                                    NeighboringPlanes
                                                                                                                                                                    [p1]
                                                                                                                                                                    .end());
                                                                                                                                                if
                                                                                                                                                (isLateral
                                                                                                                                                    &&
                                                                                                                                                    inNeighborhood) {
                                                                                                                                                    // Look into the shared planes of p12
                                                                                                                                                    for
                                                                                                                                                    (unsigned
                                                                                                                                                        int
                                                                                                                                                        u =
                                                                                                                                                                0;
                                                                                                                                                        u <
                                                                                                                                                        SharedPlanes
                                                                                                                                                        [p12]
                                                                                                                                                        .size()
                                                                                                                                                        ; u
                                                                                                                                                        ++) {
                                                                                                                                                        //int p13 = SharedPlanes[p12][u];
                                                                                                                                                        int
                                                                                                                                                                p13
                                                                                                                                                                        = OverlapWithPlanes(
                                                                                                                                                                            {
                                                                                                                                                                                p1,
                                                                                                                                                                                p2,
                                                                                                                                                                                p3,
                                                                                                                                                                                p4,
                                                                                                                                                                                p5,
                                                                                                                                                                                p6,
                                                                                                                                                                                p7,
                                                                                                                                                                                p8,
                                                                                                                                                                                p9,
                                                                                                                                                                                p10,
                                                                                                                                                                                p11,
                                                                                                                                                                                p12
                                                                                                                                                                            },
                                                                                                                                                                            SharedPlanes
                                                                                                                                                                            [p12],
                                                                                                                                                                            SharedPlanes);
                                                                                                                                                        if
                                                                                                                                                        (p13
                                                                                                                                                            !=
                                                                                                                                                            p12
                                                                                                                                                            &&
                                                                                                                                                            p13
                                                                                                                                                            !=
                                                                                                                                                            p11
                                                                                                                                                            &&
                                                                                                                                                            p13
                                                                                                                                                            !=
                                                                                                                                                            p10
                                                                                                                                                            &&
                                                                                                                                                            p13
                                                                                                                                                            !=
                                                                                                                                                            p9
                                                                                                                                                            &&
                                                                                                                                                            p13
                                                                                                                                                            !=
                                                                                                                                                            p8
                                                                                                                                                            &&
                                                                                                                                                            p13
                                                                                                                                                            !=
                                                                                                                                                            p7
                                                                                                                                                            &&
                                                                                                                                                            p13
                                                                                                                                                            !=
                                                                                                                                                            p6
                                                                                                                                                            &&
                                                                                                                                                            p13
                                                                                                                                                            !=
                                                                                                                                                            p5
                                                                                                                                                            &&
                                                                                                                                                            p13
                                                                                                                                                            !=
                                                                                                                                                            p4
                                                                                                                                                            &&
                                                                                                                                                            p13
                                                                                                                                                            !=
                                                                                                                                                            p3
                                                                                                                                                            &&
                                                                                                                                                            p13
                                                                                                                                                            !=
                                                                                                                                                            p2
                                                                                                                                                            &&
                                                                                                                                                            p13
                                                                                                                                                            !=
                                                                                                                                                            p1) {
                                                                                                                                                            bool
                                                                                                                                                                    isLateral
                                                                                                                                                                            = (
                                                                                                                                                                                SharedPlanes
                                                                                                                                                                                [p13]
                                                                                                                                                                                .size()
                                                                                                                                                                                >=
                                                                                                                                                                                AllPlanes
                                                                                                                                                                                [p13]
                                                                                                                                                                                .size());
                                                                                                                                                            bool
                                                                                                                                                                    inNeighborhood
                                                                                                                                                                            = (
                                                                                                                                                                                std::find(
                                                                                                                                                                                    NeighboringPlanes
                                                                                                                                                                                    [p1]
                                                                                                                                                                                    .begin(),
                                                                                                                                                                                    NeighboringPlanes
                                                                                                                                                                                    [p1]
                                                                                                                                                                                    .end(),
                                                                                                                                                                                    p13)
                                                                                                                                                                                !=
                                                                                                                                                                                NeighboringPlanes
                                                                                                                                                                                [p1]
                                                                                                                                                                                .end());
                                                                                                                                                            if
                                                                                                                                                            (isLateral
                                                                                                                                                                &&
                                                                                                                                                                inNeighborhood) {
                                                                                                                                                                // Look into the shared planes of p13
                                                                                                                                                                for
                                                                                                                                                                (unsigned
                                                                                                                                                                    int
                                                                                                                                                                    v =
                                                                                                                                                                            0;
                                                                                                                                                                    v <
                                                                                                                                                                    SharedPlanes
                                                                                                                                                                    [p13]
                                                                                                                                                                    .size()
                                                                                                                                                                    ; v
                                                                                                                                                                    ++) {
                                                                                                                                                                    //int p14 = SharedPlanes[p13][v];
                                                                                                                                                                    int
                                                                                                                                                                            p14
                                                                                                                                                                                    = OverlapWithPlanes(
                                                                                                                                                                                        {
                                                                                                                                                                                            p1,
                                                                                                                                                                                            p2,
                                                                                                                                                                                            p3,
                                                                                                                                                                                            p4,
                                                                                                                                                                                            p5,
                                                                                                                                                                                            p6,
                                                                                                                                                                                            p7,
                                                                                                                                                                                            p8,
                                                                                                                                                                                            p9,
                                                                                                                                                                                            p10,
                                                                                                                                                                                            p11,
                                                                                                                                                                                            p12,
                                                                                                                                                                                            p13
                                                                                                                                                                                        },
                                                                                                                                                                                        SharedPlanes
                                                                                                                                                                                        [p13],
                                                                                                                                                                                        SharedPlanes);
                                                                                                                                                                    if
                                                                                                                                                                    (p14
                                                                                                                                                                        !=
                                                                                                                                                                        p13
                                                                                                                                                                        &&
                                                                                                                                                                        p14
                                                                                                                                                                        !=
                                                                                                                                                                        p12
                                                                                                                                                                        &&
                                                                                                                                                                        p14
                                                                                                                                                                        !=
                                                                                                                                                                        p11
                                                                                                                                                                        &&
                                                                                                                                                                        p14
                                                                                                                                                                        !=
                                                                                                                                                                        p10
                                                                                                                                                                        &&
                                                                                                                                                                        p14
                                                                                                                                                                        !=
                                                                                                                                                                        p9
                                                                                                                                                                        &&
                                                                                                                                                                        p14
                                                                                                                                                                        !=
                                                                                                                                                                        p8
                                                                                                                                                                        &&
                                                                                                                                                                        p14
                                                                                                                                                                        !=
                                                                                                                                                                        p7
                                                                                                                                                                        &&
                                                                                                                                                                        p14
                                                                                                                                                                        !=
                                                                                                                                                                        p6
                                                                                                                                                                        &&
                                                                                                                                                                        p14
                                                                                                                                                                        !=
                                                                                                                                                                        p5
                                                                                                                                                                        &&
                                                                                                                                                                        p14
                                                                                                                                                                        !=
                                                                                                                                                                        p4
                                                                                                                                                                        &&
                                                                                                                                                                        p14
                                                                                                                                                                        !=
                                                                                                                                                                        p3
                                                                                                                                                                        &&
                                                                                                                                                                        p14
                                                                                                                                                                        !=
                                                                                                                                                                        p2
                                                                                                                                                                        &&
                                                                                                                                                                        p14
                                                                                                                                                                        !=
                                                                                                                                                                        p1) {
                                                                                                                                                                        bool
                                                                                                                                                                                isLateral
                                                                                                                                                                                        = (
                                                                                                                                                                                            SharedPlanes
                                                                                                                                                                                            [p14]
                                                                                                                                                                                            .size()
                                                                                                                                                                                            >=
                                                                                                                                                                                            AllPlanes
                                                                                                                                                                                            [p14]
                                                                                                                                                                                            .size());
                                                                                                                                                                        bool
                                                                                                                                                                                inNeighborhood
                                                                                                                                                                                        = (
                                                                                                                                                                                            std::find(
                                                                                                                                                                                                NeighboringPlanes
                                                                                                                                                                                                [p1]
                                                                                                                                                                                                .begin(),
                                                                                                                                                                                                NeighboringPlanes
                                                                                                                                                                                                [p1]
                                                                                                                                                                                                .end(),
                                                                                                                                                                                                p14)
                                                                                                                                                                                            !=
                                                                                                                                                                                            NeighboringPlanes
                                                                                                                                                                                            [p1]
                                                                                                                                                                                            .end());
                                                                                                                                                                        if
                                                                                                                                                                        (isLateral
                                                                                                                                                                            &&
                                                                                                                                                                            inNeighborhood) {
                                                                                                                                                                            // This means that all the 14 planes are lateral planes
                                                                                                                                                                            std::vector
                                                                                                                                                                                    <int>
                                                                                                                                                                                    PlanesListTmp
                                                                                                                                                                                            = {
                                                                                                                                                                                                p1,
                                                                                                                                                                                                p2,
                                                                                                                                                                                                p3,
                                                                                                                                                                                                p4,
                                                                                                                                                                                                p5,
                                                                                                                                                                                                p6,
                                                                                                                                                                                                p7,
                                                                                                                                                                                                p8,
                                                                                                                                                                                                p9,
                                                                                                                                                                                                p10,
                                                                                                                                                                                                p11,
                                                                                                                                                                                                p12,
                                                                                                                                                                                                p13,
                                                                                                                                                                                                p14
                                                                                                                                                                                            };

                                                                                                                                                                            /* Now we need to check if these 14 planes form a close loop(i.e.a cage)
                                                                                                                                                                             They form a closed loop if the SharedPlanes of each plane contains five of all other planes */
                                                                                                                                                                            int
                                                                                                                                                                                    NumPlanesInCage
                                                                                                                                                                                            = 0;
                                                                                                                                                                            // Check how many planes in the list satisfy the cage constraint
                                                                                                                                                                            for
                                                                                                                                                                            (unsigned
                                                                                                                                                                                int
                                                                                                                                                                                w =
                                                                                                                                                                                        0;
                                                                                                                                                                                w <
                                                                                                                                                                                PlanesListTmp
                                                                                                                                                                                .size()
                                                                                                                                                                                ; w
                                                                                                                                                                                ++) {
                                                                                                                                                                                int
                                                                                                                                                                                        ptmp
                                                                                                                                                                                                = PlanesListTmp
                                                                                                                                                                                                [w];
                                                                                                                                                                                size_t
                                                                                                                                                                                        numSharedinList
                                                                                                                                                                                                = NumSharedPlanesInSequence(
                                                                                                                                                                                                    PlanesListTmp,
                                                                                                                                                                                                    SharedPlanes,
                                                                                                                                                                                                    ptmp);
                                                                                                                                                                                if
                                                                                                                                                                                (numSharedinList
                                                                                                                                                                                    ==
                                                                                                                                                                                    AllPlanes
                                                                                                                                                                                    [ptmp]
                                                                                                                                                                                    .size()) {
                                                                                                                                                                                    NumPlanesInCage
                                                                                                                                                                                            +=
                                                                                                                                                                                            1;
                                                                                                                                                                                }
                                                                                                                                                                            }
                                                                                                                                                                            // Check if the sequence of planes form a cage
                                                                                                                                                                            if
                                                                                                                                                                            (NumPlanesInCage
                                                                                                                                                                                ==
                                                                                                                                                                                PlanesListTmp
                                                                                                                                                                                .size()) {
                                                                                                                                                                                // This forms a closed loop so it is a cage
                                                                                                                                                                                //std::sort(PlanesListTmp.begin(), PlanesListTmp.end());
                                                                                                                                                                                // Check if the detected cage is already in the list
                                                                                                                                                                                if
                                                                                                                                                                                (!
                                                                                                                                                                                    isInVecofVec(
                                                                                                                                                                                        Cages14Plane,
                                                                                                                                                                                        PlanesListTmp)) {
                                                                                                                                                                                    Cages14Plane
                                                                                                                                                                                            .push_back(
                                                                                                                                                                                                PlanesListTmp);
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
    return Cages14Plane;
};

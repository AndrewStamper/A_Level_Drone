#pragma once


class MLCurve {
	/* this class holds data about how the drone is moving and as such can be used to alter the base values for how the drone spins up.
	This is implemented in 3 areas, roll,pitch and altitude. In roll and pitch these values set base values which can account for if the drone
	is not weighted evenly and as such the drone alters the default spinup values so that it can fly flat.In altitude this is used to record how the drone 
	responds to chnages to the spinup of the altimiter variable, as such this class with help the drone to tighten its tollerances so that the default value will
	hold the drone at set altitude.
	*/
public:
	MLCurve(double AdjustValue) {
		ArrayAdd(AdjustValue);
	};

	double MultiplierArray[50];//array wich saves the spinup values as drone flys
	int NextFreeMultiplierArray; //pointer to next free value in array
	boolean Direction; //variables to deonte when to start or stop recording
	boolean Recording;
	boolean NotMoving;
	int TicksSinceLastMove;


	void ArrayAdd(double AdjustValue) {
		if (NextFreeMultiplierArray <50) {
			MultiplierArray[NextFreeMultiplierArray] = AdjustValue;//add the new data point to the array
			NextFreeMultiplierArray += 1;
		}
	}

	double CalculateNewAdjuster() {
		double NewAdjuster;
		for (int Count = 0; Count<NextFreeMultiplierArray; Count++) {//add all spinup dat points together
			NewAdjuster += MultiplierArray[Count];
		}
		NewAdjuster = (NewAdjuster) / (NextFreeMultiplierArray - 1);// form an average
		return NewAdjuster;//this average can then be used with apropriate weighting to affect the set value
	}
	void IncrementTicksSinceMove() {
		TicksSinceLastMove += 1;
	}
};
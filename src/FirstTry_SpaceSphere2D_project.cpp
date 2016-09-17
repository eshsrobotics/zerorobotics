//Begin page main
//Declare any variables shared between functions here
float myZRState[12];	 // my SPHERE state
float currentItem[3];    // location of current targeted item
float distThreshold;     // distance threshold to determine if the SPHERE has reached its destination
int spsIndex;            // SPS item index

void init(){
	//This function is called once when your code is first loaded.

	//IMPORTANT: make sure to set any variables that need an initial value.
	//Do not assume variables will be set to 0 automatically!
	distThreshold = 0.1;  // distance between SPHERE and destination to be considered close enough
	spsIndex=0;                  //
}

void loop(){
	//This function is called once per second.  Use it to control the satellite.
	if (game.getNumSPSHeld() > 0) {
	    api.getMyZRState(myZRState); // read my SPHERE states to find my current location
	    game.getItemLoc(currentItem, spsIndex); // read the coordinates of the current target item
	    api.setPositionTarget(currentItem); // move to the current target item
	    if (areWeThereYet(myZRState,currentItem,distThreshold)) {  // is the SPHERE close enough to drop an SPS next to the current item
	        game.dropSPS();     // if so, drop an SPS
	        spsIndex += 1;      // change the target to the next item
	    }
	}
	else {
	    api.setPositionTarget(currentItem);  // stay at the last item to avoid out of bound
	}
}

//End page main
//Begin page myLib
// compute the distance between two points
float doPointToPointDistance(float fromPointLoc[3], float toPointLoc[3]) {
	float vecFT[3];
	
	mathVecSubtract(vecFT, fromPointLoc,toPointLoc, 3);
	return mathVecMagnitude(vecFT, 3);
}

// see if we have arrived at or near the destination coordinate off by not more that the threshold amount
bool areWeThereYet(float fromPointLoc[3], float toPointLoc[3], float threshold){
	if ( doPointToPointDistance(fromPointLoc, toPointLoc) < threshold) {
	    return true;
	}
	return false;
}

//End page myLib

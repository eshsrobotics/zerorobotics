//Begin page Definitions
// Begin page 
// The API containing all the important functions for Zero Robotics is here.
// http://static.zerorobotics.mit.edu/docs/tutorials/ZR_user_API.pdf
// 
// SPHERE specifications can be found here.
// https://en.wikipedia.org/wiki/SPHERES#cite_note-:2-1

// Yes, this is how you do constants in Zero Robotics.
#define epsilon 0.0001f
#define zeroVector Vector(0.0f, 0.0f, 0.0f)
#define invalidPoint Point(1000.0f, 1000.0f, -1000.0f)

#define ZR_MAX_FACING_ANGLE_RADIANS 0.968912f   //Cosine of the angle 0.25 radian (max. angle allowed) at which item can be docked.
#define ZR_MAX_SPEED_MS             0.01f       //0.01 m/s is the max speed the SPHERE can move while docking

// distance between SPHERE and destination to be considered close enough to dock
// (TODO: Where did this number come from?)
//#define DIST_THRESHOLD 0.17f

//Robot diameter is 0.2 found in page 7 of documentation
#define ROBOT_RADIUS 0.1f

enum GameState {
    // The state we're in if we're not doing anything else.  All we do it move to
    // target position and adjust to target attitude, nothing more.
    Default, 
    
    // We get to this mode if either:
    // (A) We were told to start the game while in default mode, or
    // (B) We have no more SPS objects to drop.
    //
    // All we do in this mode is move to our target item and attempt to acquire it.
    Hunting,
    
    // We get to this mode if either:
    // (A) We have hunted for and found our first item, or
    // (B) We have yet to drop off all of our SPS objects while already being
    //     in SPS dropoff mode.
    //
    // All we do here is move to fixed positions to drop off our remaining two SPS objects.
    SPS_Dropoff,
    
    // We get to this mode if we're in hunting mode AND we have no more SPS objects to drop off
    // AND we currently have an item.
    //
    // All we do here is move to the target zone and drop off our item.
    // * If there are still items to pick up, we go back into hunting mode.
    // * If there are no more items to pick up, we're done with the game and end the simulation.    
    Dumping,
    
    // Kill the simulation.
    //End,
};

// Identifies which color the machine is currently and positions if y is -0.15 it is red
// or if y = 0.15 then it is blue
enum RobotColor {
    Red,
    Blue,
};

struct Vector;

//class Satellite;
// End of Definitions Page
//End page Definitions
//Begin page Struct Point
struct Point {
    float x, y, z;
    
    // Same constructor as above, but using the member-initializer
    // syntax.
    //
    // Creates a point at the origin of 3-space.
    Point() : x(0.0f), y(0.0f), z(0.0f) { }
    
    // Creates a point at the given position
    Point(float x, float y, float z) : x(x), y(y), z(z) { }

    // Creates a point from a vector.
    Point(Vector v) : x(v.x), y(v.y), z(v.z) { };

    Vector operator- (Point p) {
        return Vector(x - p.x, y - p.y, z - p.z);
    }
    
    Point operator+ (Vector v) { 
        return Point(x + v.x, y + v.y, z + v.z);
    }
    
    friend float distance(Point a, Point b) {
        float dx = a.x - b.x;
        float dy = a.y - b.y;
        float dz = a.z - b.z;
        return sqrtf(dx*dx + dy*dy + dz*dz);
    }
    
}; // end (struct Point)
//End page Struct Point
//Begin page Struct Vector
//Class used for defining Vectors and there fcn's
struct Vector {
    
    float x, y, z;
    
    // Same constructor as above, but using the member-initializer
    // syntax.
    //
    // Creates a vector at the origin of 3-space.
    Vector(): x(0.0f), y(0.0f), z(0.0f) {}
    
    Vector(float x_, float y_, float z_) {
        x = x_;
        y = y_;
        z = z_;
    }
    
    friend Vector operator* (Vector v, float factor) { return Vector(factor*v.x, factor*v.y, factor*v.z); }
    friend Vector operator* (float factor, Vector v) { return Vector(factor*v.x, factor*v.y, factor*v.z); }   
    
    
    float magnitude() {
        return distance(Point(0.0f,0.0f,0.0f), Point(x,y,z));
    }
    
    void normalize() {
        float m = magnitude();
        if (m < epsilon) {
            // Now what?
        } else {
            x /= m;
            y /= m;
            z /= m;
        }
    }
    
    // A (dot) B, where A and B are vectors, is equal to a.magitude() * b.magnitude() * cos(theta),
    // where theta is the angle between the vectors.
    float dotProduct(Vector v) {
        return x * v.x + y * v.y + z * v.z;
    }
    
    // Normalizes A and B before calculating A (dot) B.  In other words,
    // this function just returns cos(theta), where theta is the angle between
    // *this and v.
    float normalizedDotProduct(Vector v) {
        float a[3], b[3];
        a[0] = x; a[1] = y; a[2] = z;
        b[0] = v.x; b[1] = v.y; b[2] = v.z;
        mathVecNormalize(a, 3);
        mathVecNormalize(b, 3);
        return mathVecInner(a, b, 3);
    }
    
    
}; // end (class Vector)
//End page Struct Vector
//Begin page main
// This is your robot.  Treat it well.

class Satellite {
    public:
        
        Satellite() :
            desiredPosition(), 
            desiredAttitude(), targetedItemIndex(-1), state(Default),
            robotColor(Red), SPSTargetLoc1(), SPSTargetLoc2() { }
        
        // This function signals the robot to start the game and enter our state machine.
        // By the time we get to the End state, all three SPS objects should be deposited,
        // and at least three (?) items will be dropped off.
        void startGame() {
            //Drop First SPS
            game.dropSPS();

            SPSTargetLoc1 = Point(0.32f, -0.32f, 0.0f);
            SPSTargetLoc2 = Point(0.32f, 0.32f, 0.0f);
            desiredPosition = invalidPoint;
            desiredAttitude = zeroVector;
            
            downloadSatelliteState();
	        if(position.y < 0.0f) {
	            // We're the red player.
	            robotColor = Red;
	        } else {
	            // We're the blue player.  Mirror on the x-y=0 plane (1*x - 1*y + 0*z + 0 = 0).
	            SPSTargetLoc1.x = -SPSTargetLoc1.x;
	            SPSTargetLoc1.y = -SPSTargetLoc1.y;
	            SPSTargetLoc2.x = -SPSTargetLoc2.x;
	            SPSTargetLoc2.y = -SPSTargetLoc2.y;
	            robotColor = Blue;
	        }
            
            // Select targets known to be on our side and then set desiredPosition to that so we can head toward it.
            selectClosestTarget(true);
            
            state = Hunting;
        }
        
        // Causes the satellite to select the closest target that is not in the assembly zone and then sets the
        // desiredPosition to just the right place that will allow us to dock with
        // that item.
        //
        // The closest target may be owned by the enemy!
        void selectClosestTarget(bool onlySelectItemsFromOurSide = false) {
            
            float winningDistance
            = 1000.0f;
            int winningIndex = 0;
            
            for (int i = 0; i < 6; ++i) {
                
                // Sometimes we only want to consider items that we own
                if (onlySelectItemsFromOurSide == true) {
                    // Item Number Item Type 2D Location (X, Y) 3D Location(X,Y,Z)
                    // 0 Large (.23, .23) (.23, .23, .23) Blue
                    // 1 Large (-.23, -.23) (-.23, -.23, -.23) Red
                    // 2 Medium (.36, -.36) (.36, -.36, .36) Blue
                    // 3 Medium (-.36, .36) (-.36, .36, -.36) Red
                    // 4 Small (-.50, .50) (-.50, .50, .50) Blue
                    // 5 Small (.50, -.50) (.50, -.50, -.50) Red
                    switch(robotColor) {
                        case Red:
                            if (i == 0 || i == 2 || i == 4) {
                                // Skip blue's items.
                                continue;
                            }
                            break;
                        case Blue:
                            if (i == 1 || i == 3 || i == 5) {
                                // Skip red's items.
                                continue;
                            }
                            break;
                    } // end (switch robotColor)
                }
                
                float coordinate[3];
                game.getItemLoc(coordinate, i);
                float d = distance(Point(coordinate[0], coordinate[1], coordinate[2]),
                                   this->position);
                DEBUG(("New distance away item %i at %f", i, d ));
                if (d < winningDistance && !game.itemInZone(i)) {
                    //DEBUG(("Winning Distance: %i", winningDistance));
                    winningDistance = d;
                    winningIndex = i;
                }
            }
       
            // We are now in "targeting mode"...at least until we dock.
            targetedItemIndex = winningIndex;
            
            // Point in a direction...any direction.
            desiredAttitude = Vector(1.0f, 0.0f, 0.0f);
        }
        
        // This function must be called once during loop().  Since loop() is itself
        // called at regular intervals, calling update() woll ensure that the robot
        // correctly maintains its desired states and decelerates at the appropriate
        // times.
        void update() {
            downloadSatelliteState();
            
            // Ensure that the robot is always located at its most recent desired
            // position.
            //
            // Believe it or not, calling setPositionTarget() during loop() sustains
            // the satellite position without burning significant amounts of fuel.
            // Our class is designed so that once a desired position is set, the robot
            // simply sits there until further notice.
            setDesiredSatellitePosition();
            
            // Ensure that the robot is always pointing toward its most recent desired
            // attitude.  Again, once this is reached, the robot will continue facing
            // in this direction until further notice, burining inconspicuous amounts
            // of fuel to accomplish this.
            setDesiredSatelliteAttitude();
            
            
            switch (state) {
                case Default: 
                    // If we're in this state, just move to the target attitude/position
                    // and await instructions from the outside.
                    break;
                
                case Hunting:
                    // If we get to here, we always have a targetedItemIndex, so we always know
                    // what item to hunt for.

                    if (!game.hasItem(targetedItemIndex)) {
                        
                        // Calculate an ideal position in which to rendezvous with the
                        // targeted item.
                        float targetPos[3];
                        game.getItemLoc(targetPos, targetedItemIndex);
                        Point targetPosition(targetPos[0], targetPos[1], targetPos[2]);
                        
                        desiredPosition = getDockingPosition(targetedItemIndex);
                        DEBUG(("item index %d", targetedItemIndex));
                        // Orient our attitude on the fly as well.
                        //            
                        // We desire to point toward the target by the time we dock with it.
                        desiredAttitude = targetPosition - position;
                        //DEBUG(("Distance to target: %0.2f", desiredAttitude.magnitude()));

                        // This gives me the current distance from the point of interest from
                        // sphere                        .
                        float distanceToTarget = desiredAttitude.magnitude();
                        
                        // Once we dock (if we dock), exit from hunting mode.
                        bool closeEnough = false;
                        bool orientedProperly = false;
                        bool slowEnough = false;
                        
                        // Since the desiredAttitude is already the vector pointing from
                        // here to there, we can "reuse" it to get our distance check done.
                        if (distanceToTarget <= dist_Threshold && distanceToTarget >= minRadius) {
                            DEBUG(("We are in Threshold %f aka close enough", dist_Threshold));
                            closeEnough = true;
                            // BIG assumption being made here: our robot will not be chasing
                            // targets that are held by other robots.  (I mean, we COULD, but
                            // we think this is a waste of fuel.)  So it's safe to stop once
                            // we're close enough to our target; the target, after all, is
                            // stationary!
                        } else {
                            if(distanceToTarget <= minRadius) {
                                DEBUG(("Back Up"));
                                //float v[3] = {0.01, 0.01, 0.01}; 
                               // api.setVelocityTarget(v);
                            }
                        }
                        
                        if (velocity.magnitude() < ZR_MAX_SPEED_MS) {
                            slowEnough = true;
                            DEBUG(("Velocity Ready: %f", velocity.magnitude()));
                        } else {
                            //DEBUG(("Velocity Not Yet: %f", velocity.magnitude()));
                        }
                        
                        // If we were actually oriented properly, our normalized dot product
                        // would be 1.  So we need to be pretty close to 1 to be confident of
                        // orientation.
                        float myCurrentAngle = attitude.normalizedDotProduct(desiredAttitude);
                        
                        if (myCurrentAngle > ZR_MAX_FACING_ANGLE_RADIANS) {
                            DEBUG(("From Update the Attitude in range %f aka accurate enough", attitude.normalizedDotProduct(desiredAttitude)));
                            orientedProperly = true;  
                            
                            // DEBUG(("Attitude dot product with desiredAttitude: %0.3f", attitude.normalizedDotProduct(desiredAttitude)));
                            // desiredAttitude = zeroVector;
                            // Point in a direction...any direction.
                        } else {
                            
                            //begins to move sphere and hope it hits the right thing
                            if (closeEnough && slowEnough) {
                                DEBUG(("From Update the Attitude is not in range %f", attitude.normalizedDotProduct(desiredAttitude)));
                                
                            }
                        }
                        
                        if (closeEnough && slowEnough && orientedProperly) {
                            DEBUG(("Reached target; attempting to dock."));
                            if (game.dockItem(targetedItemIndex)) {
                                DEBUG(("Docked successully with our current item at targedItemIndex."));
                                // Now game.hasItem(targedItemIndex) will be true, allowing us to move into "dump mode".
                                // TODO: Move into dumping mode: set a new target!
                            } else {
                                // This is a weird condition.  We don't expect to get here.
                                DEBUG(("Something went wrong; we failed to dock."));
                                
                            }
                        } else {

                            // We're still traveling towards the targeted item.
                            DEBUG(("Position: %.2f, %.2f, %.2f DesiredPosition: %.2f, %.2f, %.2f; Target Position: %.2f, %.2f, %.2f Targeted Item Index: %d, Distance to the target: %f", 
                                position.x, position.y, position.z, 
                                desiredPosition.x, desiredPosition.y, desiredPosition.z,  
                                targetPosition.x, targetPosition.y, targetPosition.z, 
                                targetedItemIndex,
                                distance(position, desiredPosition)));
                        }
                    } else {
                        // We have the item already!
                        if (game.getNumSPSHeld() > 0) {
                            // We can't actually drop off any items until we've dropped
                            // all three of our SPS objects so that we can calculate
                            // our target area.
                            state = SPS_Dropoff;
                        } else {
                            // We already dropped off all our SPS objects.  Time to dump the
                            // item we're holding.
                            state = Dumping;
                        }
                    }
                    break;
                    
                case SPS_Dropoff:
                    // By the time we get into this state, game.getNumSPSHeld() will be equal to 2 -- after all,
                    // we already dropped off the first SPS right when the game started.
                    
                    if (game.getNumSPSHeld() == 2) {
                        
                        desiredPosition = SPSTargetLoc1;
                        
                    } else if(game.getNumSPSHeld() == 1) {
                        
                        desiredPosition = SPSTargetLoc2;
                        
                    } else {
                    
                        // When we are at this state, we have dropped off all 3 SPSs.
                        // Putting the zoneInfo() call here ensures that it is only called once.
                        float zoneInfo[4];
                        game.getZone(zoneInfo);
                        assemblyZoneCenter.x = zoneInfo[0];
                        assemblyZoneCenter.y = zoneInfo[1];
                        assemblyZoneCenter.z = zoneInfo[2];
                        errorRadius = zoneInfo[3];
                        
                        // Drop off the item we've been holding all this time.
                        state = Dumping;                        
                    }
                    
                    if (game.getNumSPSHeld() > 0 && distance(position, desiredPosition) < 0.1f) {
                        DEBUG(("We have reached desired position (%f, %f, %f).  Number of SPSs held = %d", 
                                desiredPosition.x, desiredPosition.y, desiredPosition.z, game.getNumSPSHeld()));
                        // We've reached the drop-off point for whatever our SPS is
                        game.dropSPS();
                        
                    }
                    break;
                case Dumping:
                    {
                        // Get telemetry information from the item we are holding 
                        float itemZRState[12];
                        game.getItemZRState(itemZRState, targetedItemIndex);
                        Point itemPos = Point(itemZRState[0], itemZRState[1], itemZRState[2]);
                        DEBUG(("Assembly Zone Center %f, %f, %f and E-Radius: %f", assemblyZoneCenter.x, assemblyZoneCenter.y, assemblyZoneCenter.z, errorRadius));
                        
                        // Orient ourselves to point at where we need to go and find attitude to point at
                        
                        desiredAttitude = assemblyZoneCenter - position;
                        
                        // Certain conditions should be met for us to release the item.
                        //float assemblyZoneRadius = errorRadius;
                        //if (assemblyZoneRadius > 0.09f) {
                         //   assemblyZoneRadius = 0.09f;
                        //}  
                        if (distance(assemblyZoneCenter, itemPos) < errorRadius/3) {
                            // RELEASE THE ITEM.
                            game.dropItem(); 
                            
                            // Select new target to hunt for.
                            selectClosestTarget();
                            state = Hunting;                             

                        } else {
                            // We're not there yet.

                            // Move to a point just outside the assembly zone center using
                            // linterp for the computation.
                            //
                            // The variable 'u' is the parameter of interpolation.  If it's 1.0f,
                            // that means move straight into the center of the assembly zone.
                            // But we don't want that; instead, we calculate a value of u just less
                            // than 1.
                            
                            // How much percentage of the error radius to use for 
                            // determining the point to travel toward.
                            // 1.0f means use the full errorRadius; 0.0f means to pretend
                            // that the radius is 0: we travel to the center of the asssembly
                            // zone.
                            const float fudgeFactor = 0.0f;
                            
                            float distanceToAssemblyZone = distance(assemblyZoneCenter, position);
                            float u = (distanceToAssemblyZone - fudgeFactor * errorRadius) / distanceToAssemblyZone;
                            desiredPosition = position + u * (assemblyZoneCenter - desiredPosition);                          
                        }
                        
                        // We need to stop when our item is within the margin of error then begin to check for points
                        // because if we get points we should drop item and go to the next item
                    }
                    break;
                    
            } // end (switch state)
        } // end (void update())

    private:
        Point position;   // The last measured position of the robot.
        Vector velocity;  // The last measured velocity vector of the robot.
        Vector attitude;  // The direction the robot was pointing in last.
        
        // Target physical quantities.  All of these can be set independently.
        // Ideally, once we've reached our target, we reset the relevant target
        // quantity to its default, whatever that is.
        //
        // Why?  Because our robot is a state machine, that's why.  We neeed to
        // remember what we wanted to do the last time we were in loop(), and that
        // includes physical targets.
        Point desiredPosition;
        Vector desiredAttitude;
        
        // If we're tracking an item that we need to dock to, this will be set 
        // to that item's index.
        int targetedItemIndex;
        
        // What the satellite's internal state machine is currently doing.
        GameState state;
        RobotColor robotColor;
        Point SPSTargetLoc1, SPSTargetLoc2;
        // The target zone's center can only be determined after all 3 SPSs are dropped off
        Point assemblyZoneCenter;
        float errorRadius;
        float dist_Threshold;
        
        // Gets the "radius" of the items so that we know where to dock to them.
        // Much blood was spilled to obtain these numbers.  Change them at your peril
        // ... and test, always test.
        float minRadius;
        
        float getItemRadius(int itemType) {
            
            float maxRadius = 0.0f;
            
            switch(game.getItemType(itemType)) {
                case ITEM_TYPE_LARGE:
                    minRadius = 0.151f;
                    maxRadius = 0.173f; 
                    dist_Threshold = 0.172f;
                    break;
                case ITEM_TYPE_MEDIUM:
                    minRadius = 0.138f;
                    maxRadius = 0.160f;
                    dist_Threshold = 0.159f;
                    break;
                case ITEM_TYPE_SMALL:
                    minRadius = 0.124f;
                    maxRadius = 0.146f;
                    dist_Threshold = 0.145f;
                    break; 
            }
            
            const float radiusRatio = 0.57f; // How far between the radii the docking point should be.
            return minRadius + radiusRatio * (maxRadius - minRadius); 
        }
        
        Point getDockingPosition(int itemIndex) {
            // Need to know position of our target item and current position to use linear interpolation
            float itemPos[3];
            game.getItemLoc(itemPos, itemIndex);
            Point itemPosition(itemPos[0], itemPos[1], itemPos[2]);
            float itemDistance = distance(position, itemPosition);
            float itemRadius = getItemRadius(itemIndex);
            
            //assumption that robot radius is considered in setpositiontarget
            float u = (itemDistance - itemRadius)/itemDistance;
            float x = position.x + u * (itemPosition.x - position.x);
            float y = position.y + u * (itemPosition.y - position.y);
            float z = position.z + u * (itemPosition.z - position.z);
            
            return Point(x, y, z);
        }
        
        // In the Zero Robotics API, a robot's state is encapsulated
        // in an array of 12 floats. 
        //                                                                                 
        // This function downloads the current ZRAPI robot state
        // into our member variables.  That makes getting the official
        // ZRAPI robot state a one-liner, hiding the ugly
        // array-of-twelve-floats from the rest of our code.
        void downloadSatelliteState() {
            float myZRState[12];
            api.getMyZRState(myZRState);
            //float x[3] = {3,6,8};
            //print x[2]
            position.x = myZRState[0];
            position.y = myZRState[1];
            position.z = myZRState[2];
            velocity.x = myZRState[3];
            velocity.y = myZRState[4];
            velocity.z = myZRState[5];
            attitude.x = myZRState[6];
            attitude.y = myZRState[7];
            attitude.z = myZRState[8];
        }
        
        // These functions make the robot's actual state agree
        // with the values of our "desired" member variables.
        // IMPORTANT: These may cause the robot to move.
        //
        // As it so happens, there's no "setMyZRstate" function in the
        // ZR API.  Instead, there are corresponding "setWhateverTarget()"
        // methods that will actually cause the robot to move and turn
        // until it reaches the desired state.        
        
        void setDesiredSatellitePosition() {
            float positionTarget[3];
           // float ourPosition[3] = {position.x, position.y, position.z};
           // float displacement[3];
            positionTarget[0] = desiredPosition.x;
            positionTarget[1] = desiredPosition.y;
            positionTarget[2] = desiredPosition.z;
            api.setPositionTarget(positionTarget);
           // mathVecSubtract(displacement, positionTarget, ourPosition, 3);
            //mathVecNormalize(direction, 3)
           // api.setVelocityTarget(displacement);
            //float mag = mathVecMagnitude(displacement, 3);
            //if (mag < (mag/2)) {
             //   displacement[0] = -displacement[0];
             //   displacement[1] = -displacement[1];
              //  displacement[2] = -displacement[2];
              //  api.setVelocityTarget(displacement);
            //}
        }
        
        void setDesiredSatelliteAttitude() {
            Vector n = desiredAttitude;
            float attitudeTarget[3];
            attitudeTarget[0] = desiredAttitude.x;
            attitudeTarget[1] = desiredAttitude.y;
            attitudeTarget[2] = desiredAttitude.z;
            mathVecNormalize(attitudeTarget, 3);
            api.setAttitudeTarget(attitudeTarget);
        }
        
}; // end (class Satellite)

//This declaration by itself takes up 26%
Satellite mySatellite;

void init() {
	mySatellite.startGame();
}

void loop () {
    mySatellite.update();
} 


//End page main

//End page main

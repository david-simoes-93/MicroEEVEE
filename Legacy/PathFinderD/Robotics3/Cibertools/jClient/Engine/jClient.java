package Engine;
/*
 This file is part of ciberRatoToolsSrc.

 Copyright (C) 2001-2011 Universidade de Aveiro

 ciberRatoToolsSrc is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 ciberRatoToolsSrc is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Foobar; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

import static Engine.Comm.*;
import Engine.Comm.ExploreDir;
import Engine.Comm.TypeOfMsg;
import Engine.Map.Cell;
import static Engine.Sensors.*;
import Engine.Sensors.State;
import ciberIF.*;
import java.awt.Color;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

/**
 * example of a basic agent implemented using the java interface library.
 */
public class jClient {

    ciberIF cif;
    int helloCounter = 0, orderCounter = 0, cont = 0;
    Behaviour[] bevs = new Behaviour[4];
    EncodeMessage encMsg;
    DecodeMessage decMsg;
    static boolean debug = false;
    boolean showGui;

    int index;
    State prevState = null;
    int prevBehaviour;
    private String robName;
    private final int beaconToFollow;
    private Cell lastCrampingTarget = null;
    private int crampingTargetCounter = 0;

    public static void main(String[] args) {

        String host, robName;
        int pos;
        int arg;

        //default values
        host = "localhost";
        robName = "jClient";
        pos = 1;
        boolean showG = true;

        // parse command-line arguments
        try {
            arg = 0;
            while (arg < args.length) {
                if (args[arg].equals("-pos")) {
                    if (args.length > arg + 1) {
                        pos = Integer.valueOf(args[arg + 1]);
                        arg += 2;
                    }
                } else if (args[arg].equals("-robname")) {
                    if (args.length > arg + 1) {
                        robName = args[arg + 1];
                        arg += 2;
                    }
                } else if (args[arg].equals("-host")) {
                    if (args.length > arg + 1) {
                        host = args[arg + 1];
                        arg += 2;
                    }
                } else if (args[arg].equals("-debug")) {
                    debug = true;
                    arg++;
                } else if (args[arg].equals("-nogui")) {
                    showG = false;
                    arg++;
                } else {
                    throw new Exception();
                }
            }
        } catch (Exception e) {
            print_usage();
            return;
        }

        // create client
        jClient client = new jClient(showG);
        client.robName = robName;

        // register robot in simulator
        client.cif.InitRobot(robName, pos, host);
        // main loop
        client.mainLoop();

    }

    // Constructor
    jClient(boolean showGui) {
        cif = new ciberIF();
        Sensors.cif = cif;
        realBeacon = new beaconMeasure();
        beacon = new beaconMeasure();

        compass = myX = myY = lpow = rpow = 0;

        beaconToFollow = 0;
        ground = -1;

        bevs[0] = new FollowTheWall();
        bevs[1] = new FollowTheBeacon();
        bevs[2] = new DodgeObstacle();
        bevs[3] = new Wander();

        state = State.EXPLORING_QUADRANT;
        sendType = TypeOfMsg.MAP_UPDATE;
        rcvType = TypeOfMsg.MAP_UPDATE;

        //encode/decode message objects
        encMsg = new EncodeMessage();
        decMsg = new DecodeMessage();

        this.showGui = showGui;

        //expDir = ExploreDir.UP_RIGHT; // default
        //expDir random at start or if no message has been received
        Random rnd = new Random();

        for (int i = 2; i < robPos.length; i++) {
            toExploreDirs[i] = i - 1;
        }

        pathTaken = new ArrayList();
    }

    /**
     * reads a new message, decides what to do and sends action to simulator
     */
    public void mainLoop() {
        //initial sensor request and define map params
        map = Map.showMap(robName + "'s Map", 28 * 3 * 2 + 30, 14 * 3 * 2 + 30, showGui);

        //stall while map loads; even if -nogui, read once to read trash
        do {
            // initial request and reading
            cif.RequestIRSensor(0);
            cif.RequestCompassSensor();
            cif.RequestIRSensor(1);
            cif.RequestIRSensor(2);
            cif.ReadSensors();  // reads nothing
        } while (showGui && map.gui == null);

        // fill buffers
        for (int i = 0; i < numberOfMeasures; i++) {
            cif.RequestIRSensor(0);
            cif.RequestCompassSensor();
            cif.RequestIRSensor(1);
            cif.RequestIRSensor(2);

            cif.ReadSensors();
            sensors0[counter] = cif.GetObstacleSensor(0);
            oldMyCompass = compass = cif.GetCompassSensor();
            sensors1[counter] = cif.GetObstacleSensor(1);
            sensors2[counter] = cif.GetObstacleSensor(2);
            myX = cif.GetX();
            myY = cif.GetY();
            xs[counter] = myX;
            ys[counter] = myY;
            counter = (counter + 1) % numberOfMeasures;
        }

        // calculate averages and what-not
        cif.RequestIRSensor(0);
        cif.RequestCompassSensor();
        cif.RequestIRSensor(1);
        cif.RequestIRSensor(2);
        cif.ReadSensors();
        updateSensors();

        map.setOffsets((int) (28 * 3 + 12 - myX * 3), (int) (14 * 3 + 12 - myY * -3));

        pathTaken.add(new double[]{myX, myY});

        //Get robID in order to not process its own messages    
        String[] aux = robName.split("jClient");
        robID = Integer.parseInt(aux[1]);
        if (robID == 1) {
            setHome(myX, myY);
        }

        if (debug) {
            System.out.println("Im number " + robID);
        }

        prepareComm();

        while (true) {
            //System.out.println("Requesting sensors");
            reqSensors();
            //System.out.println("Reading sensors");
            cif.ReadSensors();
            //System.out.println("Updating sensors");
            updateSensors();
            //System.out.println("Making map functions");
            mapFunctions();
            //System.out.println("Deciding what to do");
            decide();
            //System.out.println("Filling values");

            robPos[robID][0] = myX;
            robPos[robID][1] = myY;
            if (true) {
                System.out.println("expDir=" + expDir.name());
            }

            //System.out.println("Processing messages");
            processMessages();
            //System.out.println("Sending messages");
            sendMessage();
            //System.out.println("Cycle done\n");
        }
    }

    private void updateSensors() {
        // all cycles
        if (cif.IsObstacleReady(0)) {
            sensors0[counter] = cif.GetObstacleSensor(0);
        } else {
            sensors0[counter] = -1;
        }
        if (cif.IsCompassReady()) {
            compass = cif.GetCompassSensor();
        }

        /* calc compass using kalman */
        //System.out.printf("justCompass= %5.1f ", compass);
        //myCompassKalman(oldMyCompass, compass);
        //save value to next cycle
        //oldMyCompass = compass;
        // 3/4 cycles
        if (cif.IsObstacleReady(1)) {
            sensors1[counter] = cif.GetObstacleSensor(1);
        } else {
            sensors1[counter] = -1;
        }
        if (cif.IsObstacleReady(2)) {
            sensors2[counter] = cif.GetObstacleSensor(2);
        } else {
            sensors2[counter] = -1;

        }

        // 1/4 cycles
        if (cif.IsGroundReady()) {
            ground = cif.GetGroundSensor();
        }
        if (cif.IsBeaconReady(beaconToFollow)) {
            realBeacon = cif.GetBeaconSensor(beaconToFollow);
        }

        /* Obtain index of last measure */
        if (counter == 0) {
            index = numberOfMeasures - 1;
        } else {
            index = counter - 1;
        }
        /* Calculate new position using last position and how much we ordered to the wheels */
        //calcMyGps(xs[index], ys[index], cif.GetX(), cif.GetY());
        calcMyGpsKalman(xs[index], ys[index], cif.GetX(), cif.GetY());

        /* Update position values*/
        xs[counter] = myX;
        ys[counter] = myY;

        // update counter
        counter = (counter + 1) % numberOfMeasures;
        index = counter;

        // GPS values always come
        //myX = myY = 0;
        //oldmyX = oldmyY = 0;
        // 4 cycle ago gps index

        // sensor values may not come due to lag or due to not being requested
        double tempOldIRSensor0 = 0, tempOldIRSensor1 = 0, tempOldIRSensor2 = 0, tempIRSensor0 = 0, tempIRSensor1 = 0, tempIRSensor2 = 0;
        int sensor12Counter = 0, oldSensor12Counter = 0, sensor0Counter = 0, oldSensor0Counter = 0;
        for (int i = 0; i < 3; i++) {
            int oldIndex = (i + index) % numberOfMeasures;
            int currIndex = (i + index + 3) % numberOfMeasures;

            if (i == 1) {
                oldmyX = xs[oldIndex];
                oldmyY = ys[oldIndex];
            }

            if (sensors0[oldIndex] != -1) {
                tempOldIRSensor0 += sensors0[oldIndex];
                oldSensor0Counter++;
            }
            if (sensors1[oldIndex] != -1) {
                tempOldIRSensor1 += sensors1[oldIndex];
                tempOldIRSensor2 += sensors2[oldIndex];
                oldSensor12Counter++;
            }

            //oldmyX += xs[oldIndex];
            //oldmyY += ys[oldIndex];
            if (sensors0[currIndex] != -1) {
                tempIRSensor0 += sensors0[currIndex];
                sensor0Counter++;
            }
            if (sensors1[currIndex] != -1) {
                tempIRSensor1 += sensors1[currIndex];
                tempIRSensor2 += sensors2[currIndex];
                sensor12Counter++;
            }

            //myX += xs[currIndex];
            //myY += ys[currIndex];
        }

        if (oldSensor0Counter != 0) {
            oldIrSensor0 = tempOldIRSensor0 / oldSensor0Counter;
        }
        if (oldSensor12Counter != 0) {
            oldIrSensor1 = tempOldIRSensor1 / oldSensor12Counter;
            oldIrSensor2 = tempOldIRSensor2 / oldSensor12Counter;
        }
        //oldmyX /= 3;
        //oldmyY /= 3;

        if (sensor0Counter != 0) {
            irSensor0 = tempIRSensor0 / sensor0Counter;
        }
        if (sensor12Counter != 0) {
            irSensor1 = tempIRSensor1 / sensor12Counter;
            irSensor2 = tempIRSensor2 / sensor12Counter;
        }

        //myX /= 3;
        //myY /= 3;
        
        /*if(state == State.WAIT || state == State.QUIT){
         myX = specialAvgX = (specialAvgX*specialAvgCounter + myX)/(specialAvgCounter+1);
         myY = specialAvgY = (specialAvgY*specialAvgCounter + myY)/(specialAvgCounter+1);
         specialAvgCounter++;
         }*/
        if (irSensor0 < 0.4) {
            irSensor0 = 0.4;
        }
        if (irSensor1 < 0.4) {
            irSensor1 = 0.4;
        }
        if (irSensor2 < 0.4) {
            irSensor2 = 0.4;
        }

        if (oldIrSensor0 < 0.4) {
            oldIrSensor0 = 0.4;
        }
        if (oldIrSensor1 < 0.4) {
            oldIrSensor1 = 0.4;
        }
        if (oldIrSensor2 < 0.4) {
            oldIrSensor2 = 0.4;
        }

    }

    public void mapFunctions() {
        double[] currPos = {myX, myY};
        if (getDistance(pathTaken.get(pathTaken.size() - 1), currPos) > 2) {
            pathTaken.add(currPos);
        }

        map.paintFrontObstacle(map.getCellfromGPS_x(oldmyX), map.getCellfromGPS_y(oldmyY), 1 + 3 / oldIrSensor0, compass);
        map.paintLeftObstacle(map.getCellfromGPS_x(oldmyX), map.getCellfromGPS_y(oldmyY), 2 + 3 / oldIrSensor1, compass);
        map.paintRightObstacle(map.getCellfromGPS_x(oldmyX), map.getCellfromGPS_y(oldmyY), 2 + 3 / oldIrSensor2, compass);
        map.paintClearRadius(map.getCellfromGPS_x(oldmyX), map.getCellfromGPS_y(oldmyY));
    }

    public void decide() {
        crampingTargetCounter++;
        if (crampingTargetCounter >= maxCrampingCounter) {
            crampingTargetCounter = 0;
            lastCrampingTarget = null;
        }
        map.resetGUI();
        checkState();
        Cell target = null;
        Cell beaconEstimation = null;
        int posX, posY;
        if (state != prevState) {
            System.out.println("\t" + state);
            prevState = state;
        }
        //System.out.println("state="+state.name());

        switch (state) {
            case CRAMPING_DOWN:

                // walk forwards until obstacle or out of zone
                if (ground == 1 && irSensor0 < 3 && irSensor1 < 3 && irSensor2 < 3) {
                    cif.DriveMotors(0.03, 0.03);
                } else {
                    //specialAvgX = myX;
                    //specialAvgY = myY;
                    //specialAvgCounter=1;

                    state = State.QUIT;
                    cif.DriveMotors(0, 0);
                }

                break;
            case QUIT:
                beaconSet(startingX, startingY);

                double beaconDistance = Sensors.getDistance(new double[]{myX, myY}, new double[]{startingX, startingY});
                if (ground != 1) {
                    bevs[1].execute(cif);
                    //cif.DriveMotors(-0.03, -0.03);
                } else if (beaconDistance < 1 && !(irSensor0 < 3 && irSensor1 < 3 && irSensor1 < 3)) {
                    cif.DriveMotors(0.06, -0.06);
                } else if (beaconDistance < 1) {
                    cif.DriveMotors(0.03, 0.03);
                } else if (beacon.beaconDir < 65 ) {
                    cif.DriveMotors(0.04, -0.04);
                }  else if (beacon.beaconDir > 85) {
                    cif.DriveMotors(-0.04, 0.04);
                } else if (irSensor0 < 3 && irSensor1 < 3 && irSensor2 < 3) {
                    cif.DriveMotors(0.03, 0.03);
                } else {
                    cif.DriveMotors(0.0, 0.0);
                }

                if (allReadyToQuit(robID)) {
                    cif.Finish();
                    System.exit(0);
                }
                break;
            case CRAMPING_UP:
                // define random beacon point to warn buddies
                if (!map.beaconPoints.isEmpty()) {
                    beaconEstimation = map.getRandomBeaconPoint();
                    beaconPts[0] = map.getGPSfromCell_x(beaconEstimation.x);
                    beaconPts[1] = map.getGPSfromCell_y(beaconEstimation.y);
                } else {
                    beaconPts[0] = myX;
                    beaconPts[1] = myY;
                }

                // mark beacon points
                if (ground == 0 && cif.IsGroundReady()) {
                    posX = map.getCellfromGPS_x(myX);
                    posY = map.getCellfromGPS_y(myY);
                    map.setUniqueBeaconPoint(posX, posY);
                    map.setUniqueBeaconPoint(posX + 1, posY);
                    map.setUniqueBeaconPoint(posX - 1, posY);
                    map.setUniqueBeaconPoint(posX, posY + 1);
                    map.setUniqueBeaconPoint(posX, posY - 1);
                }

                // walk forwards until obstacle or out of zone
                if (ground == 0 && irSensor0 < 3 && irSensor1 < 3 && irSensor2 < 3) {
                    cif.DriveMotors(0.03, 0.03);
                } else {
                    //specialAvgX = myX;
                    //specialAvgY = myY;
                    //specialAvgCounter=1;

                    state = State.WAIT;
                    cif.DriveMotors(0, 0);
                }
                break;
            case WAIT:
                // define random beacon point to warn buddies
                beaconEstimation = map.getRandomBeaconPoint();
                beaconPts[0] = map.getGPSfromCell_x(beaconEstimation.x);
                beaconPts[1] = map.getGPSfromCell_y(beaconEstimation.y);

                beaconEstimation = map.getBeaconEstimation();
                beaconDistance = Sensors.getDistance(new double[]{myX, myY}, new double[]{map.getGPSfromCell_x(beaconEstimation.x), map.getGPSfromCell_y(beaconEstimation.y)});
                beacon=realBeacon;
                
                if (ground != 0) {
                    bevs[1].execute(cif);
                } else if (beaconDistance < 1 && !(irSensor0 < 3 && irSensor1 < 3 && irSensor1 < 3)) {
                    cif.DriveMotors(0.06, -0.06);
                } else if (beaconDistance < 1) {
                    cif.DriveMotors(0.03, 0.03);
                } else if (realBeacon.beaconDir < 65 ) {
                    cif.DriveMotors(0.04, -0.04);
                }  else if (realBeacon.beaconDir > 85) {
                    cif.DriveMotors(-0.04, 0.04);
                } else if (irSensor0 < 3 && irSensor1 < 3 && irSensor2 < 3) {
                     cif.DriveMotors(0.03, 0.03);
                } else {
                    cif.DriveMotors(0.0, 0.0);
                }

                //check how many people know about the beacon
                int rwr = getRobotsWithinRange();
                if (rwr < 5) { // change to 5
                    System.out.println("\tOnly " + rwr + "/5 robots in range! Finding others");
                    state = State.FINDING_FRIENDS;
                }

                break;
            case FINDING_FRIENDS:
                //send a "go look for people" message to these assholes and decide where to look for (keep in mind the range of the robots; no need to explore all map)
                //go to place assigned spamming that message until place has been reached or all members have been contacted
                //turn back, wait until ready to go home
                target = map.goToRangeZone(new Map.Cell(map.getCellfromGPS_x(myX), map.getCellfromGPS_y(myY)), Comm.getDirFromLostRobot(robID));
                if (target != null) {
                    beaconSet(map.getGPSfromCell_x(target.x), map.getGPSfromCell_y(target.y));
                    move();
                } else {
                    System.out.println("\tCouldnt find friends");
                    cif.DriveMotors(0.0, 0.0);
                }
                break;
            case GOING_HOME:
                if (getDistance(new double[]{myX, myY}, new double[]{map.getGPSfromCell_x(startingCell.x), map.getGPSfromCell_y(startingCell.y)}) > 4) {
                    //System.out.println("\tGH: calculating new path. too far from target");
                    target = map.StarSearchExplorerZone(new Map.Cell(map.getCellfromGPS_x(myX), map.getCellfromGPS_y(myY)), startingCell, 1, 6);
                } else if (lastCrampingTarget == null) {
                    //((FollowTheWall) bevs[0]).setReactive(true);
                    //System.out.println("\tGH: calculating new path. prev path was null"); 
                    target = map.StarSearchExplorerZone(new Map.Cell(map.getCellfromGPS_x(myX), map.getCellfromGPS_y(myY)), startingCell, 1, 6);
                } else {
                    ((FollowTheWall) bevs[0]).setReactive(true);
                    System.out.println("\tGH: following old path  for " + crampingTargetCounter + " cycles");
                    target = map.getNextCell(new Cell(map.getCellfromGPS_x(myX), map.getCellfromGPS_y(myY)));
                }

                if (target == null) {
                    System.out.println("\tCant find way home! re-tracing footsteps");

                    double[] currPos = {myX, myY};
                    double minDist = Sensors.getDistance(pathTaken.get(0), currPos), tempDist;
                    int minIndex = 0;
                    for (int i = 1; i < pathTaken.size(); i++) {
                        if ((tempDist = Sensors.getDistance(pathTaken.get(i), currPos)) < minDist) {
                            minDist = tempDist;
                            minIndex = i;
                        }
                    }
                    while (pathTaken.size() > minIndex + 1) {
                        pathTaken.remove(pathTaken.size() - 1);
                    }

                    if (minDist < 3 && pathTaken.size() > 1) {
                        pathTaken.remove(pathTaken.size() - 1);
                    }

                    double[] tar = pathTaken.get(pathTaken.size() - 1);
                    System.out.printf("\tat (%3.1f,%3.1f) and going for (%3.1f,%3.1f), %d points left\n", myX, myY, tar[0], tar[1], pathTaken.size());
                    if (map.gui != null) {
                        map.gui.paintTemporaryPoint(map.getCellfromGPS_x(tar[0]), map.getCellfromGPS_y(tar[1]), Color.DARK_GRAY);
                    }
                    beaconSet(tar[0], tar[1]);
                } else {
                    beaconSet(map.getGPSfromCell_x(target.x), map.getGPSfromCell_y(target.y));
                }
                lastCrampingTarget = target;
                move();
                ((FollowTheWall) bevs[0]).setReactive(false);
                break;
            case GOING_TO_BEACON:
                // get beacon average to warn buddies. why not random?
                beaconEstimation = map.getBeaconEstimation();
                if (beaconEstimation != null) {
                    beaconPts[0] = map.getGPSfromCell_x(beaconEstimation.x);
                    beaconPts[1] = map.getGPSfromCell_y(beaconEstimation.y);
                } else {
                    beaconPts[0] = -1;
                    beaconPts[1] = -1;
                }

                if (lastCrampingTarget == null) {
                    if (cif.IsBeaconReady(beaconToFollow)) {
                        System.out.println("GTB: sensor was ready, getDirectionTarget()");
                        target = map.getDirectionTarget(new Map.Cell(map.getCellfromGPS_x(oldmyX), map.getCellfromGPS_y(oldmyY)), Math.toRadians(realBeacon.beaconDir + compass), new Map.Cell(map.getCellfromGPS_x(myX), map.getCellfromGPS_y(myY)));
                    } else if (!map.beaconPoints.isEmpty()) {
                        //target = map.getWayToBeacon(new Map.Cell(map.getCellfromGPS_x(myX), map.getCellfromGPS_y(myY)));
                        System.out.println("GTB: sensor was NOT ready, A*");
                        target = map.StarSearchExplorerZone(new Cell(map.getCellfromGPS_x(myX), map.getCellfromGPS_y(myY)), beaconEstimation, 1, 6);
                        if (target != null) {
                            maxCrampingCounter = (int) target.counter;
                        }
                    }
                } else {
                    //double lol = getDistance(new double[]{myX, myY}, new double[]{map.getGPSfromCell_x(lastCrampingTarget.x), map.getGPSfromCell_y(lastCrampingTarget.y)});
                    System.out.println("\tsticking to the same path for " + crampingTargetCounter + "/" + maxCrampingCounter + " cycles.");
                    target = map.getNextCell(new Cell(map.getCellfromGPS_x(myX), map.getCellfromGPS_y(myY)));
                }

                lastCrampingTarget = target;

                // move
                if (target != null) {
                    beaconSet(map.getGPSfromCell_x(target.x), map.getGPSfromCell_y(target.y));
                } else {
                    System.out.println("GTB: path NOT found. entering reactive mode");
                    beacon = realBeacon;
                    ((FollowTheWall) bevs[0]).setReactive(true);
                    //beacon.beaconVisible=false;
                }
                move();
                //((FollowTheWall) bevs[0]).setReactive(false);
                break;
            case GOING_TO_FAKE_BEACON:
                beaconEstimation = map.getBeaconEstimation();
                beaconPts[0] = map.getGPSfromCell_x(beaconEstimation.x);
                beaconPts[1] = map.getGPSfromCell_y(beaconEstimation.y);

                target = map.getWayToBeacon(new Map.Cell(map.getCellfromGPS_x(myX), map.getCellfromGPS_y(myY)));
                if (target != null) {
                    beaconSet(map.getGPSfromCell_x(target.x), map.getGPSfromCell_y(target.y));
                } else {
                    System.out.println("Couldn't find path to beacon guess");
                }
                move();
                break;
            case EXPLORING_QUADRANT:
                //System.out.println("ENTERING TargetOrder");
                target = map.getTargetByOrder(map.getCellfromGPS_x(myX), map.getCellfromGPS_y(myY), expDir);
                //System.out.println("LEAVING TargetOrder");
                if (target != null) {
                    beaconSet(map.getGPSfromCell_x(target.x), map.getGPSfromCell_y(target.y));
                } else {
                    System.out.println("Couldn't find path to unexplored target cell");
                }
                move();
                break;
        }
    }

    public void move() {
        for (int i = 0; i < bevs.length; i++) {
            if (bevs[i].isPossible()) {
                if (prevBehaviour != i) {
                    System.out.println("\tExecuting behaviour " + bevs[i].getName());
                    prevBehaviour = i;
                }
                bevs[i].execute(cif);
                break;
            }
        }
    }

    public void processMessages() {

        sendType = TypeOfMsg.MAP_UPDATE; //default
        coordOrd = false;
        //System.out.println("DECODE number "+cont++);

        //check if any msg received and decode it
        for (int i = 1; i < 6; i++) {
            //exclude own robID
            if (i == robID) {
                continue;
            }

            if (!cif.NewMessageFrom(i)) {
                map.setRobotPos(i, 0, 0);
                if (state != State.FINDING_FRIENDS) {
                    withinRangeRob[i] = false;
                }
                continue;
            }
            if (state != State.FINDING_FRIENDS || !alternatingMessage) {
                withinRangeRob[i] = true;
            }
            decMsg.decode(cif.GetMessageFrom(i), map, i);

            //get type and other params read from message received
            switch ((rcvType = decMsg.getTypeOfMsg())) {
                case HELLO:
                    //System.out.println("Received msg: " + rcvType.name());
                    //update positions from robots contained on msg, except mine
                    decMsg.decodeHello(i);

                    //send a message giving an order of expDir or repeat msg if im not the coordinator
                    if (robID == 1) {
                        //System.out.println("Im number "+robID+" sending ORDER msg");
                        sendType = TypeOfMsg.ORDER;
                        coordOrd = true;
                    } else if (receivedHello(i) && sendType == TypeOfMsg.MAP_UPDATE) {  //if i already received an hello from that robot
                        sendType = TypeOfMsg.MAP_UPDATE;
                    } else {
                        if (!(sentNonCoord[i]) && i != 1 && seqNum > 1) {
                            //System.out.println("Sending myOrder because of robot " + i);
                            orderToSend = EncodeMessage.encodeNonCoordOrder();
                            sentNonCoord[i] = true;
                            //System.out.println("Order: " + orderToSend);
                            sendType = TypeOfMsg.ORDER;
                        } else if (!receivedHello(i) && sendType != TypeOfMsg.ORDER) {
                            //System.out.println("First Hello");
                            firstHello = true;
                            sendType = TypeOfMsg.HELLO;
                        }
                    }

                    if (i == 1 && !receivedHello(i)) {
                        firstHello = true;
                        //System.out.println("FIRST HELLO FROM COORDINATOR, planning to send:"+sendType);
                    }

                    //mark who sent me a hello msg
                    helloRcvFrom[i] = true;
                    //System.out.println("Hello Message received from " + i);
                    break;

                case MAP_UPDATE:
                    //System.out.println("Received msg: " + rcvType.name() + " from robot "+i);

                    int[] mapLims = decMsg.getMapLimits();   //get lims
                    //merge map received with rob's own map
                    map.setActualMap(mapLims[0], mapLims[1], mapLims[2], mapLims[3], decMsg.getMapRead());
                    
                    //if(!receivedHello(i))
                    //    firstHello = true;
                    
                    break;
                case BEACONFOUND:
                    //System.out.println("Received msg: " + rcvType.name()+" from robot "+i);

                    double[] t = decMsg.getBeaconCenterPoint();
                    boolean added = map.setUniqueBeaconPoint(map.getCellfromGPS_x(t[0]), map.getCellfromGPS_y(t[1]));
                    while (map.beaconPoints.size() < 3 && added) {
                        map.setBeaconPoint(map.getCellfromGPS_x(t[0]), map.getCellfromGPS_y(t[1]));
                    }
                    break;

                case ORDER:
                    //System.out.println("Received msg: " + rcvType.name());

                    // if im coordinator ill ignore it
                    if (robID != 1) {
                        //send order if i received a fresher Order and seqNum > 1
                        if (decMsg.getDecodedFlag() && seqNum > 1) {
                            validOrder = true;
                            orderToSend = decMsg.getRcvMsg();
                            sendType = TypeOfMsg.ORDER;
                        }
                    }
                    break;
                case HOMEINFO:
                    break;
                case READYTOQUIT:
                    break;
                default:
                    //System.out.println("Another type of message received from "+i);
                    break;
            }
            System.out.println("Received msg: " + rcvType.name());
        }

        if (validOrder && !orderToSend.equals("")) {
            sendType = TypeOfMsg.ORDER;
            validOrder = false;
        } else if (firstHello) {
            sendType = TypeOfMsg.HELLO;
            firstHello = false;
        } else if (orderToSend.equals("") && !coordOrd) {
            sendType = TypeOfMsg.MAP_UPDATE;
        } else {
            sendType = TypeOfMsg.ORDER;
        }

        for (int i = 1; i < 6; i++) {
            if (withinIndirectRangeRob[i] && !withinRangeRob[i]) {
                // System.out.println("Within indirect range of " + i);
                withinRangeRob[i] = true;
            }
            withinIndirectRangeRob[i] = false;
        }
    }

    //method that decides which msg to be sent and defines it
    public void sendMessage() {
        if (sendType == TypeOfMsg.ORDER) {
            //sending message with expDir for all robs
            encMsg.setTypeOfMsg(sendType, map);
            //repeat message if im not coordinator
            if (robID != 1) {
                cif.Say(orderToSend);
                //else create a new msg with expDir order
            } else {
                cif.Say(encMsg.encodeOrder());
            }

        } else if (sendType == TypeOfMsg.HELLO) {
            //First hello in response to another hello
            encMsg.setTypeOfMsg(sendType, map);
            cif.Say(encMsg.encodeHello(robID));

        } else if (helloCounter < 10) {
            //send 10 beggining Hellos
            sendType = TypeOfMsg.HELLO;
            encMsg.setTypeOfMsg(sendType, map);
            cif.Say(encMsg.encodeHello(robID));

        } else if (state == State.WAIT || state == State.CRAMPING_UP || state == State.GOING_TO_BEACON || state == State.FINDING_FRIENDS) {
            // Send a BeaconFound or a MapUpdate, alternating
            if (!alternatingMessage) {

                sendType = TypeOfMsg.BEACONFOUND;
                encMsg.setTypeOfMsg(sendType, map);

                cif.Say(encMsg.encodeBeaconFound(beaconPts[0], beaconPts[1]));

                alternatingMessage = true;
            } else {
                sendType = TypeOfMsg.MAP_UPDATE;
                encMsg.setTypeOfMsg(sendType, map);
                cif.Say(encMsg.encodeMapUpdate());

                alternatingMessage = false;
            }
        } else if (state == State.GOING_HOME) {
            if (sentHomeInfo) {
                System.out.println("Sending information about home");

                sendType = TypeOfMsg.HOMEINFO;
                encMsg.setTypeOfMsg(sendType, map);

                cif.Say(encMsg.encodeBeaconFound(startingX, startingY));

                sentHomeInfo = false;
            } else {

                sendType = TypeOfMsg.MAP_UPDATE;
                encMsg.setTypeOfMsg(sendType, map);
                cif.Say(encMsg.encodeMapUpdate());
            }
        } else if (state == State.QUIT) {
            if (!alternatingMessage) {

                sendType = TypeOfMsg.READYTOQUIT;
                encMsg.setTypeOfMsg(sendType, map);

                cif.Say(encMsg.encodeRTQ());

                alternatingMessage = true;
            } else {
                sendType = TypeOfMsg.MAP_UPDATE;
                encMsg.setTypeOfMsg(sendType, map);
                cif.Say(encMsg.encodeMapUpdate());

                alternatingMessage = false;
            }
        } else {
            //send Map Update
            sendType = TypeOfMsg.MAP_UPDATE;
            encMsg.setTypeOfMsg(sendType, map);          //prepare msg header
            cif.Say(encMsg.encodeMapUpdate());
        }

        if (helloCounter <= 10) {
            helloCounter++;
        }

        if (debug) {
            System.out.println("Sent msg: " + sendType.name());
        }
        //reset orderToSend
        orderToSend = "";

    }

    //request sensors with a certain timing pre-defined
    public void reqSensors() {
        // Requesting front obst. sensor and compass every cycle
        // Every 4 cycles req beacon and ground (200ms) else obstacle sensors (left and right)
        cif.RequestIRSensor(0);
        cif.RequestCompassSensor();

        if (cif.GetTime() % 4 == 1) {
            cif.RequestGroundSensor();
            cif.RequestBeaconSensor(beaconToFollow);
        } else {
            cif.RequestIRSensor(1);
            cif.RequestIRSensor(2);
        }
    }

    public void prepareComm() {
        withinRangeRob[robID] = true;   //im always on range

        //initialize known positions of robs to -1 (unknown value)
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 2; j++) {
                robPos[i][j] = -1;
            }
        }
        //initialize hello array
        helloRcvFrom[0] = true;
        helloRcvFrom[robID] = true;
        //initialize seqNum
        seqNum = 1;
        if (robID == 1) {
            seqNum = 2;
        }

        if (robID == 1) {
            //initialize expDir array (its own expDir)
            if (compass >= -45 && compass < 45) {
                expDir = ExploreDir.RIGHT;
                toExploreDirs[1] = getDirID(ExploreDir.RIGHT);
            } else if (compass >= 45 && compass < 135) {
                expDir = ExploreDir.UP;
                toExploreDirs[1] = getDirID(ExploreDir.UP);
            } else if (compass >= 135 || compass < -135) {
                expDir = ExploreDir.LEFT;
                toExploreDirs[1] = getDirID(ExploreDir.LEFT);
            } else {
                expDir = ExploreDir.DOWN;
                toExploreDirs[1] = getDirID(ExploreDir.DOWN);
            }
        } else {
            expDir = getDirEnum(robID - 1);    //my own expDir
        }
        //make a copy of toExploDirs
        lastToExploreDirs = Arrays.copyOf(toExploreDirs, toExploreDirs.length);
    }

    static void print_usage() {
        System.out.println("Usage: java jClient [-robname <robname>] [-pos <pos>] [-host <hostname>[:<port>]]");
    }

};

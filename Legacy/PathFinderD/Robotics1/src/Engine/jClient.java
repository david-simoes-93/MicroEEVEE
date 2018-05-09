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
import static Engine.Repetitions.history;
import static Engine.Sensors.*;
import ciberIF.*;


/**
 * example of a basic agent implemented using the java interface library.
 */
public class jClient {

    ciberIF cif;
    Behaviour[] bevs = new Behaviour[4];
    boolean yolo;
    boolean startFollowingWithoutBeacon=true;
    

    public static void main(String[] args) {

        String host, robName;
        int pos;
        int arg;
        
        

        //default values
        host = "localhost";
        robName = "jClient";
        pos = 1;

        // parse command-line arguments
        try {
            arg = 0;
            while (arg < args.length) {
                if (args[arg].equals("-pos")) {
                    if (args.length > arg + 1) {
                        pos = Integer.parseInt(args[arg + 1]);
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
                } else {
                    throw new Exception();
                }
            }
        } catch (Exception e) {
            print_usage();
            return;
        }
        
        

        // create client
        jClient client = new jClient();

        client.robName = robName;

        // register robot in simulator
        client.cif.InitRobot(robName, pos, host);

        // main loop
        client.mainLoop();

    }

    // Constructor
    jClient() {
        cif = new ciberIF();
        beacon = new beaconMeasure();

        beaconToFollow = 0;
        ground = -1;

        bevs[0] = new FollowTheWall(startFollowingWithoutBeacon);
        bevs[1] = new FollowTheBeacon();
        bevs[2] = new DodgeObstacle();
        bevs[3] = new Wander();
        
        for(int i=0; i<history.length; i++)
            history[i]=-1;
    }

    /**
     * reads a new message, decides what to do and sends action to simulator
     */
    public void mainLoop() {

        while (true) {
            cif.ReadSensors();
            updateSensors();
            decide();
        }
    }

    private void updateSensors() {
        if (cif.IsObstacleReady(0)) {
            irSensor0 = cif.GetObstacleSensor(0);
        }
        if (cif.IsObstacleReady(1)) {
            irSensor1 = cif.GetObstacleSensor(1);
        }
        if (cif.IsObstacleReady(2)) {
            irSensor2 = cif.GetObstacleSensor(2);
        }

        if (cif.IsCompassReady()) {
            compass = cif.GetCompassSensor();
        }
        if (cif.IsGroundReady()) {
            ground = cif.GetGroundSensor();
        }

        if (cif.IsBeaconReady(beaconToFollow)) {
            beacon = cif.GetBeaconSensor(beaconToFollow);
        }

        //System.out.println("Measures: ir0=" + irSensor0 + " ir1=" + irSensor1 + " ir2=" + irSensor2 + "\n");
        //System.out.println("Measures: x=" + x + " y=" + y + " dir=" + dir);
        //System.out.println(robName);// + " state " + state);
    }

    /**
     * basic reactive decision algorithm, decides action based on current sensor
     * values
     */
    public void decide() {
        if (ground == 0) {         /* Visit Target */

            cif.SetVisitingLed(true);
            System.out.println(robName + " visited target at " + cif.GetTime() + "\n");
            cif.Finish();
            System.exit(0);
        } else {
            for (int i = 0; i < bevs.length; i++) {
                //if(i==avoidRepetitions())
                //    break;
                
                if (bevs[i].isPossible()) {
                    
                    if(!("Executing behaviour "+i).equals(beh)){
                        System.out.println("Executing behaviour "+bevs[i].getName());
                    }
                    beh = "Executing behaviour "+i;
                    bevs[i].execute(cif);
                    
                    //history[pointer]=i;
                    //pointer=(pointer+1)%16;
                    break;
                }
            }
        }
        
        //oldComp = cif.GetCompassSensor();
        
        cif.Say(robName);

        if (cif.GetTime() % 2 == 0) {
            cif.RequestIRSensor(0);
            if (cif.GetTime() % 8 == 0) {// || state == State.RETURN) {
                cif.RequestGroundSensor();
            } else {
                cif.RequestBeaconSensor(beaconToFollow);
            }
        } else {
            cif.RequestIRSensor(1);
            cif.RequestIRSensor(2);
        }
        cif.RequestCompassSensor();
        
    }
    
    

    static void print_usage() {
        System.out.println("Usage: java jClient [-robname <robname>] [-pos <pos>] [-host <hostname>[:<port>]]");
    }
    //private double oldComp;
    private String robName;
    private final int beaconToFollow;
    String beh="";
};

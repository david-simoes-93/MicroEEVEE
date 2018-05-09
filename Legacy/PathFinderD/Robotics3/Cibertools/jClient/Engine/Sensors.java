/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package Engine;

import static Engine.Comm.getRobotsWithinRange;
import static Engine.Comm.robID;
import Engine.Map.Cell;
import ciberIF.*;
import java.util.ArrayList;
import java.util.List;

public class Sensors {

    public static enum State {
        WAIT, GOING_HOME, GOING_TO_BEACON, GOING_TO_FAKE_BEACON, EXPLORING_QUADRANT, CRAMPING_UP, CRAMPING_DOWN, FINDING_FRIENDS, QUIT
    }
    public static boolean debug = false;
    public static final int numberOfMeasures = 6;
    public static final double gpsVariance = Math.pow(0.5, 2);
    //Kalman
    public static double lpow, rpow;
    public static final double motorVarianceFactor = 0.015;
    public static final double compassVariance = Math.pow(2.0, 2);
    public static double covOfProcessNoise, processNoise;
    public static double covOfProcessNoiseComp, processNoiseComp;
    public static double estCovariance = gpsVariance;
    public static double estCovarianceComp = compassVariance;
    public static double kalmanGain, kalmanGainComp;

    public static int counter = 0;
    public static double[] sensors0 = new double[numberOfMeasures];
    public static double[] sensors1 = new double[numberOfMeasures];
    public static double[] sensors2 = new double[numberOfMeasures];
    public static double[] xs = new double[numberOfMeasures];
    public static double[] ys = new double[numberOfMeasures];
    public static List<double[]> pathTaken;

    public static double radius = 10.0;
    public static State state;
    public static double irSensor0, irSensor1, irSensor2, compass, oldMyCompass;
    public static double oldIrSensor0, oldIrSensor1, oldIrSensor2;
    public static beaconMeasure realBeacon, beacon;
    public static int ground;
    public static boolean collision, alternatingMessage = false, sentHomeInfo=true;
    public static double x, y, startingX=-1, startingY=-1;
    public static double myX, myY, oldmyX, oldmyY;
    public static boolean goingHome = false;
    public static Map map;
    
    //public static double specialAvgX, specialAvgY;
    //public static int specialAvgCounter, 
    public static int maxCrampingCounter;

    public static ciberIF cif;

    public static ArrayList<Cell> wayHome = null;
    public static Cell startingCell;

    public static void setMotorsVal(double left, double right) {
        lpow = left;
        rpow = right;
    }

    public static boolean atStart() {
        if (lpow > 0.01 || rpow > 0.01) {
            return false;
        }
        return true;
    }

    /* Calculates the rotation at each moment updating the compass. Return in degrees*/
    public static void myCompass(double left, double right) {
        lpow = left;
        rpow = right;

        double rot = (rpow - lpow);    //rot = (r-l)/diam
        //compass+=Math.toDegrees(rot);
        compass = normalizeAngle(compass + Math.toDegrees(rot));
        
    }
    
    public static void myCompassKalman(double prevCompass, double currCompass) {
        double aux = 0.0;
        if (!cif.GetStartButton() || cif.GetStopButton()) {
            lpow = 0.000;
            rpow = 0.000;
        }
        //calculating angle using motor vals
        double rot = (rpow - lpow);    //rot = (r-l)/diam

        //current estimate
        //aux = normalizeAngle(prevCompass + Math.toDegrees(rot));
        aux = prevCompass + Math.toDegrees(rot);
        //calculate process noise
        processNoiseComp = (Math.pow(lpow * motorVarianceFactor, 2) + Math.pow(rpow * motorVarianceFactor, 2)) / 2;
        
        //Pt (propagated variance) FpFt+Q
        estCovarianceComp += processNoiseComp;
        aux+=processNoiseComp;
        
        //Calculate kalman gain
        kalmanGainComp = estCovarianceComp / (estCovarianceComp + compassVariance);
        
        //Calculate current compass (estimated value)
        compass = normalizeAngle( aux + kalmanGainComp * (currCompass - aux) );
        
        //update estimated covariance
        estCovarianceComp = (1 - kalmanGainComp) * estCovarianceComp;

        System.out.printf("KalmanGainCompass= %4.2f KalmanCompass= %5.1f\n", kalmanGainComp, compass);


    }

    public static double normalizeAngle(double angle) {
        while (angle < - 180) {
            angle += 360.0;
        }
        while (angle > 180) {
            angle -= 360.0;
        }
        return angle;
    }

    public static void calcMyGpsKalman(double pastX, double pastY, double presX, double presY) {
        
        if (!cif.GetStartButton() || cif.GetStopButton()) {
            lpow = 0.000;
            rpow = 0.000;
        }
        /* distance traveled in D */
        double dist = (lpow + rpow) / 2;
        double currX = presX, currY = presY;

        myX = pastX + Math.cos(Math.toRadians(compass)) * dist;     // distance travelled in x-axis
        myY = pastY + Math.sin(Math.toRadians(compass)) * dist;     // distance travelled in y-axis

        if(debug)
            System.out.println("lpow=" + lpow + " rpow=" + rpow);
        //Calculate processNoise
        processNoise = (Math.pow(lpow * motorVarianceFactor, 2) + Math.pow(rpow * motorVarianceFactor, 2)) / 2;
        //covOfProcessNoise = processNoise if 2 vars are identical
        //next state estimate X t+1
        //myX += processNoise;
        //myY +=processNoise;

        //Pt (propagated variance) FpFt+Q
        estCovariance += processNoise;

        if(debug)
            System.out.printf("processNoise=%f estCovariance=%f\n", processNoise, estCovariance);

        //Calculate kalman gain
        kalmanGain = estCovariance / (estCovariance + gpsVariance);

        //Calculate current position (estimated value)
        myX = myX + kalmanGain * (currX - myX);
        myY = myY + kalmanGain * (currY - myY);

        //update estimated covariance
        estCovariance = (1 - kalmanGain) * estCovariance;

        if(debug)
            System.out.printf("KalmanGain= %4.2f Current Pos: X= %5.1f Y= %5.1f Compass= %5.1f\n", kalmanGain, myX, myY, compass);

    }

    public static void calcMyGps(double pastX, double pastY, double presX, double presY) {
        /* distance traveled in D */
        double dist = (lpow - rpow) / 2;

        myX = pastX + Math.cos(Math.toRadians(compass)) * dist;     // distance travelled in x-axis
        myY = pastY + Math.sin(Math.toRadians(compass)) * dist;     // distance travelled in y-axis

        /* Do an average of calculated values and read values from GPS */
        myX = (myX * 3 + presX) / 4;
        myY = (myY * 3 + presY) / 4;

        //System.out.printf("Current Pos: X= %5.1f Y= %5.1f\n", myX, myY);
    }

    public static boolean isOnRadius(gpsMeasure[] before, double nowX, double nowY) {
        double dist = 0;
        for (int i = 0; i < 2; i++) {
            dist = Math.sqrt((Math.pow((nowX - before[i].x), 2)) + (Math.pow((nowY - before[i].y), 2)));
            if (dist <= radius) {
                return true;
            }
        }
        return false;
    }

    public static void setHome(double x, double y) {
        startingX = x;
        startingY = y;
    }

    public static void checkState() {
        if (state == State.GOING_HOME && ground == 1) {
            // returning home and found home area
            state = State.CRAMPING_DOWN;
        }else if(state == State.CRAMPING_DOWN || state== State.QUIT){
            // do nothing
        } else if (cif.GetReturningLed()) {
            if(startingX != -1 && startingY != -1){
                // return home after visiting beacon
                state = State.GOING_HOME;
                startingCell = new Cell(map.getCellfromGPS_x(startingX), map.getCellfromGPS_y(startingY));
            }
        } else if (state == State.FINDING_FRIENDS) {
            // if everyone in range OR close to assigned position, turn back
            int rwr = getRobotsWithinRange();
            if (rwr == 5) {
                System.out.println("Found everyone!");
                state = State.GOING_TO_FAKE_BEACON;
            }
            else if(getDistance(new double[]{myX, myY}, map.getRangeTarget(Comm.getDirFromLostRobot(robID)))<2){
                System.out.println("Got in range of target area!");
                state = State.GOING_TO_FAKE_BEACON;
            }
        } else if (ground == 0 || state == State.CRAMPING_UP || state == State.WAIT) {
            // if in any state (except WAiT), start cramping up
            if (state != State.WAIT) {
                // get to the end of the line, buddy
                state = State.CRAMPING_UP;
            } 
            // if waiting
            else if (ground == 0) {
                // at beacon, wait for others
                cif.SetVisitingLed(true);
            }
        } else if (realBeacon.beaconVisible) {
            // beacon is visible
            state = State.GOING_TO_BEACON;
        } else if (map.beaconPoints.size() >= 3) {
            // beacon isnt visible but we have a good idea of where it is
            state = State.GOING_TO_FAKE_BEACON;
        } else {
            // beacon is unknown
            state = State.EXPLORING_QUADRANT;
        }
    }

    public static void beaconSet(double targetX, double targetY) {
        double[] meToTargetVector = new double[2];
        double[] myDirectionVector = new double[2];

        meToTargetVector[0] = (targetX - myX);
        meToTargetVector[1] = (targetY - myY);
        myDirectionVector[0] = Math.cos(Math.toRadians(compass));
        myDirectionVector[1] = Math.sin(Math.toRadians(compass));

        double radianBeaconDir = Math.atan2(meToTargetVector[1], meToTargetVector[0]) - Math.atan2(myDirectionVector[1], myDirectionVector[0]);
        if (radianBeaconDir > Math.PI) {
            radianBeaconDir -= 2 * Math.PI;
        }
        if (radianBeaconDir < -Math.PI) {
            radianBeaconDir += 2 * Math.PI;
        }

        beacon.beaconDir = Math.toDegrees(radianBeaconDir);
        beacon.beaconVisible = true;
    }
    
    public static double getDistance(double[] a, double[] b) {
        return Math.sqrt(Math.pow(a[0] - b[0], 2) + Math.pow(a[1] - b[1], 2));
    }

}

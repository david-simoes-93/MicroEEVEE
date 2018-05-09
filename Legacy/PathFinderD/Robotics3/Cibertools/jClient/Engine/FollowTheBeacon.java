package Engine;

import static Engine.Sensors.*;
import ciberIF.ciberIF;

public class FollowTheBeacon implements Behaviour {

    double prevLeft = 0.0, prevRight = 0.0;
    boolean debug = false;

    @Override
    public void execute(ciberIF cif) {
        // if no obstacles and beacon to the left
        /*
         if (beacon.beaconDir > 60.0) {
         cif.DriveMotors(-0.1, 0.1);
         } // if no obstacles and beacon to the right
         else if (beacon.beaconDir < -60.0) {
         cif.DriveMotors(0.1, -0.1);
         } else if (beacon.beaconDir > 20.0) {
         cif.DriveMotors(0.0, 0.1);
         } // if no obstacles and beacon to the right
         else if (beacon.beaconDir < -20.0) {
         cif.DriveMotors(0.1, 0.0);
         } else if (beacon.beaconDir > 5.0) {
         cif.DriveMotors(0.05, 0.1);
         } // if no obstacles and beacon to the right
         else if (beacon.beaconDir < -5.0) {
         cif.DriveMotors(0.1, 0.05);
         } // if no obstacles and beacon "in front"
         else if(irSensor0<1 && irSensor1<2 && irSensor2<2 ){
         cif.DriveMotors(0.1, 0.1);
         } else{
         cif.DriveMotors(0.05, 0.05);
         }*/

        double std;
        double bDir = Math.abs(beacon.beaconDir);
        double baseVal = 0.06;

        if (bDir > 20) {
            std = bDir / 1200;  //0.02
        } else {
            std = bDir / 500;   //0.02
        }
        //System.out.printf("Bdir= %1.3f\n", beacon.beaconDir);

        if (Math.abs(beacon.beaconDir) > 60) {
            if (cif.IsCompassReady()) {
                //System.out.printf("\tspinning %3.1f\n", beacon.beaconDir);
                if (beacon.beaconDir >= 0) {
                    cif.DriveMotors(-0.08, 0.08);
                    prevLeft = -0.08;
                    prevRight = 0.08;
                } else {
                    cif.DriveMotors(0.08, -0.08);
                    prevLeft = 0.08;
                    prevRight = -0.08;
                }
            } else{
                System.out.println("Compass not ready, not spinning");
            }
            if (debug) {
                System.out.println("> |60| or obstacle too close");
            }

        } else if (beacon.beaconDir < -20.0) {
            cif.DriveMotors(baseVal + std / 2, baseVal - std);
            prevLeft = baseVal + std / 2;
            prevRight = baseVal - std;
            if (debug) {
                System.out.printf("NEG. 20-60 std= %1.3f\n", std);
            }
        } else if (beacon.beaconDir > 20.0) {
            cif.DriveMotors(baseVal - std, baseVal + std / 2);
            prevLeft = baseVal - std;
            prevRight = baseVal + std / 2;
            if (debug) {
                System.out.printf("20-60 std= %1.3f\n", std);
            }
        } else if (beacon.beaconDir < -10.0) {
            cif.DriveMotors(baseVal + std / 2, baseVal - std / 2);
            prevLeft = baseVal + std / 2;
            prevRight = baseVal - std / 2;
            if (debug) {
                System.out.printf("NEG. 10-20 PLUS std= %1.3f\n", std);
            }
        } else if (beacon.beaconDir > 10.0) {
            cif.DriveMotors(baseVal - std / 2, baseVal + std / 2);
            prevLeft = baseVal - std / 2;
            prevRight = baseVal + std / 2;
            if (debug) {
                System.out.printf("10-20 PLUS std= %1.3f\n", std);
            }
        } else if (beacon.beaconDir < 10 && beacon.beaconDir > -10) {
            cif.DriveMotors(0.1, 0.1);
            //cif.DriveMotors(prevLeft, prevRight);
            prevLeft = 0.1;
            prevRight = 0.1;
            //cif.DriveMotors(prevLeft, prevRight);
            if (debug) {
                System.out.printf("[-10,10]\n");
            }
        } // if no obstacles and beacon "in front"
        else if (irSensor0 < 1.0 && irSensor1 < 2.0 && irSensor2 < 2.0) {
            cif.DriveMotors(0.1, 0.1);
            prevLeft = 0.1;
            prevRight = 0.1;
            if (debug) {
                System.out.println("ELSEIF");
            }
        } else {
            cif.DriveMotors(0.05, 0.05);
            prevLeft = 0.05;
            prevRight = 0.05;
            if (debug) {
                System.out.println("ELSE");
            }
        }
        if (debug) {
            System.out.printf("Left= %5.3f Right= %5.3f Bdir= %1.3f\n", prevLeft, prevRight, beacon.beaconDir);
        }
    }

    @Override
    public boolean isPossible() {
        return (irSensor0 <= 2.0 && irSensor1 <= 2.0 && irSensor2 <= 2.0 && beacon.beaconVisible) || // if no obstacles at >=1 distance
                (beacon.beaconVisible && (beacon.beaconDir < -60 || beacon.beaconDir > 60));                       // beacon is visible
    }

    @Override
    public String getName() {
        return "FollowTheBeacon";
    }

}

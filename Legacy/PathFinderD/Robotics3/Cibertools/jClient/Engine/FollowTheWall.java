package Engine;

import static Engine.Sensors.*;
import ciberIF.ciberIF;

public class FollowTheWall implements Behaviour {

    boolean wallOnTheRight = false, wallOnTheLeft = false; //, facedBeacon = false, startFollowingWithoutBeacon = true, keepValues=false;
    // double lastKnownBeaconAngle = 0, lastKnownBeaconCompass = 0, grausRodados = 0.0, 
    double mainSide = 0.0;
    double wallSpeed = 0.0, opposSpeed = 0.0, prevSpeed = 0.0;
    int printID = 0, lastPrintID = 0, val;
    boolean debug = false;

   // public FollowTheWall(boolean fwa) {
    //     startFollowingWithoutBeacon = fwa;
    // }
    public void setReactive(boolean a){
        if(a) val=180;
        else val=90;
    }
    
    @Override
    public void execute(ciberIF cif) {
        //System.out.printf("\tFeeling middle=%2.1f left=%2.1f right=%2.1f\n", irSensor0, irSensor1, irSensor2);

        // double modder = 1.0;
        double weakSide = irSensor1, midSide = irSensor0;
        mainSide = irSensor2;

        if (!wallOnTheRight) {
            // modder = -1.0;
            mainSide = irSensor1;
            weakSide = irSensor2;
        }

        double far = 1 / mainSide / 20;
        double std = 1 / (mainSide * 10);
        double slow = 1 / (mainSide * 15);
        double slower = 1 / (mainSide * 20);
        double slowerpp = 1 / (mainSide * 25);

        //need to change 1 and 0 to modder influenced
        if ((mainSide > 4.0 || midSide > 4.0)) {
            wallSpeed = 0.08;
            opposSpeed = -0.08;
            //cif.DriveMotors(modder * -0.08, modder * 0.08); 
            printID = 1;
        } else if (weakSide > 4.0 && mainSide < 2.0) {
            wallSpeed = slower - 0.01;
            opposSpeed = std+0.01;           //max,min 0.05,0.025   0.33,0.16
            //cif.DriveMotors( modder * std  , modder * -slow );         
            printID = 2;
        } else if ((mainSide >= 3.0 && weakSide < 2.0) || midSide > 2.0) {  // if close wall on the right OR close wall in front
            std = 1 / (min(mainSide, midSide) * 10);
            slow = 1 / (min(mainSide, midSide) * 15);    // Para 2.0: 0.05 0.033 Para 4.0: 0.025 0.016
            wallSpeed = slow+0.01;
            opposSpeed = slow - 0.01;                             //lmax = 0.07 lmin = 0.045  rmin = 0.26 rmax = 0.43 
            //cif.DriveMotors( modder * -std, modder * std );   // sharp turn left
            prevSpeed = (mainSide - 3.0) / 20;                     //if prevSpeed > 0 means its close to wall else its away
            printID = 3;

        } else if (mainSide > 2.5) {                         // if far wall on the right and nothing in front
            //wallSpeed = 0.03 + std; opposSpeed =  0.05 - std;
            wallSpeed = 0.06 - std;
            opposSpeed = 0.04 + std;       //min 0.01 max 0.
            //cif.DriveMotors( 0.05 - modder * std, 0.03 + modder * std );    //  straight forward  min -0.01 max 0.07
            prevSpeed = (mainSide - 2.5) / 20;
            printID = 4;

        } else if (mainSide > 2.0) {                         // if far wall on the right and nothing in front
            wallSpeed = 0.08;
            opposSpeed = 0.08;
            //cif.DriveMotors(0.07, 0.07);    //  straight forward
            prevSpeed = (mainSide - 2.0) / 20;
            printID = 5;

        } else if (mainSide > 1.0) {                         // turn slightly right
            /*if(prevSpeed > 0){               
             wallSpeed = 0.02 + prevSpeed; opposSpeed = 0.06;
             prevSpeed-=0.005;
             }else{
             wallSpeed = 0.02 + (-prevSpeed); opposSpeed = 0.07;
             prevSpeed+=0.005;
             } */
            wallSpeed = 0.02 + slowerpp;
            opposSpeed = 0.03 + slow;        //0.06 0.08  Para 2.0: 0.025 0.033
            //cif.DriveMotors( 0.03 + modder * slow, 0.05 + modder * slower );       //max 0.075 min 0.04
            printID = 6;

        } else if (mainSide <= 1.0) {                        // if wall has vanished
            /*if(prevSpeed > 0){               
             wallSpeed = 0.01 + prevSpeed; opposSpeed = 0.06;
             prevSpeed-=0.005;
             }else{
             wallSpeed = 0.01 + (-prevSpeed); opposSpeed = 0.07;
             prevSpeed+=0.002;
             }*/

            if (mainSide == 1) {
                wallSpeed = 0.03;
                opposSpeed = 0.07;
            } else {
                wallSpeed = 0.03 - far;
                opposSpeed = 0.03 + far;
            }
            //cif.DriveMotors(0.04 + (modder * far), 0.04 - (modder * far) );    //  turn right
            printID = 7;

        } else {
            wallSpeed = 0.03;
            opposSpeed = 0.03;
            //cif.DriveMotors(0.01,0.01);
            printID = 8;

        }

        if (debug) {
            if (printID != lastPrintID) {
                switch (printID) {
                    case 1:
                        System.out.println("\tClose Obstacle on Main Side or front");
                        lastPrintID = 1;
                        break;
                    case 2:
                        System.out.println("\tGet closer to main wall");
                        lastPrintID = 2;
                        break;
                    case 3:
                        System.out.println("\tGet distance to main wall");
                        lastPrintID = 3;
                        break;
                    case 4:
                        System.out.println("\tFar wall on main side");
                        lastPrintID = 4;
                        break;
                    case 5:
                        System.out.println("\tFarther wall on main side");
                        lastPrintID = 5;
                        break;
                    case 6:
                        System.out.println("\tTurn slightly to main side");
                        lastPrintID = 6;
                        break;
                    case 7:
                        System.out.println("\tTurn to main side");
                        lastPrintID = 7;
                        break;
                    case 8:
                        System.out.println("\tElse, walk really slow");
                        lastPrintID = 8;
                        break;
                    default:
                        System.out.println("\tDefault");
                }
            }
        }

        if (wallSpeed == 0.0 && opposSpeed == 0.0) {
            System.out.println("HIT");
        }

        if (wallOnTheRight) {
            cif.DriveMotors(opposSpeed, wallSpeed);
            //prevLeft = opposSpeed; 
            //prevRight = wallSpeed;
        } else {
            cif.DriveMotors(wallSpeed, opposSpeed);
            //prevLeft = wallSpeed; 
            //prevRight = opposSpeed;
        }

    }

    @Override
    public boolean isPossible() {
        // wall on the right and beacon is to the right as well (or very closely in front)
        boolean rightWallOnBeaconSide = beacon.beaconVisible && irSensor2 > 1.5 && beacon.beaconDir >= -10 && beacon.beaconDir <= val;
        // wall on the left and beacon is to the left as well (or very closely in front)
        boolean leftWallOnBeaconSide = beacon.beaconVisible && irSensor1 > 1.5 && beacon.beaconDir <= 10 && beacon.beaconDir >= -val;
        // no beacon but wall on some side
        boolean beaconNotVisWithWall = !beacon.beaconVisible && (irSensor1 > 1.5 || irSensor2 > 1.5);

        // beacon visible and a wall appeared on its side
        if (rightWallOnBeaconSide || leftWallOnBeaconSide) {
            wallOnTheRight = rightWallOnBeaconSide;
            wallOnTheLeft = leftWallOnBeaconSide;
        } // beacon not visible and wall close by 
        else if (beaconNotVisWithWall) {
            wallOnTheLeft = irSensor1 > irSensor2;
            wallOnTheRight = !wallOnTheLeft;
        } // no walls
        else {
            wallOnTheRight = wallOnTheLeft = false;
        }
        //System.out.printf("Flags\trightWallOnBeaconSide: %b leftWallOnBeaconSide: %b beaconNotVisWithWall: %b\n", rightWallOnBeaconSide, leftWallOnBeaconSide, beaconNotVisWithWall);
        //System.out.printf("Sensors FTW\t Front: %1.1f Left: %1.1f Right: %1.1f\nBeacon Dir: %3.0f\n", irSensor0, irSensor1, irSensor2, beacon.beaconDir);
        return wallOnTheRight || wallOnTheLeft;
    }

    @Override
    public String getName() {
        return "FollowTheWall";
    }

    private double min(double a, double b) {
        if (a <= b) {
            return a;
        } else {
            return b;
        }
    }

}

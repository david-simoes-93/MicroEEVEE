package Engine;

import static Engine.Repetitions.*;
import static Engine.Sensors.*;
import ciberIF.ciberIF;

public class FollowTheWall implements Behaviour {

    boolean wallOnTheRight = false, wallOnTheLeft = false, facedBeacon = false, startFollowingWithoutBeacon = true;
    double lastKnownBeaconAngle = 0, lastKnownBeaconCompass = 0;
    double grausRodados = 0.0, prevComp = 0.0;

    public FollowTheWall(boolean fwa) {
        startFollowingWithoutBeacon = fwa;
    }

    @Override
    public void execute(ciberIF cif) {
        //System.out.printf("\tFeeling middle=%2.1f left=%2.1f right=%2.1f\n", irSensor0, irSensor1, irSensor2);

        double modder = 1.0;
        double mainSide = irSensor2, weakSide = irSensor1, midSide = irSensor0;
        if (!wallOnTheRight) {
            modder = -1.0;
            mainSide = irSensor1;
            weakSide = irSensor2;
        }

        /*boolean avoider = avoidRepetitions();
        if (avoider) {
            System.out.println("AVOID!");
        }*/

        //need to change 1 and 0 to modder influenced
        if ( (mainSide > 4.0 || midSide > 4.0)) { //avoider ||
            cif.DriveMotors(modder * -0.1, modder * 0.1);
            history[counter] = getMainSideTurning(modder);
        } else if (weakSide > 4.0 && mainSide < 2.0) {
            cif.DriveMotors(modder * 0.1, modder * -0.1);
            history[counter] = getWeakSideTurning(modder);
        } else if ((mainSide >= 3.0 && weakSide < 2.0) || midSide > 2.0) {  // if close wall on the right OR close wall in front
            cif.DriveMotors(modder * -0.1, modder * 0.1);   // sharp turn left
            history[counter] = getMainSideTurning(modder);
        } else if (mainSide > 2.5) {                         // if far wall on the right and nothing in front
            cif.DriveMotors(0.09 - modder * 0.01, 0.9 + modder * 0.01);    //  straight forward
            history[counter] = -1;
        } else if (mainSide > 2.0) {                         // if far wall on the right and nothing in front
            cif.DriveMotors(0.1, 0.1);    //  straight forward
            history[counter] = -1;
        } else if (mainSide > 1.0) {                         // turn slightly right
            cif.DriveMotors(0.06 + (modder * 0.04), 0.06 - (modder * 0.04));
            history[counter] = -1;
        } else if (mainSide <= 1.0) {                        // if wall has vanished
            cif.DriveMotors(0.05 + (modder * 0.05), 0.05 - (modder * 0.05));    //  turn right
            history[counter] = -1;
        }

        counter = (counter + 1) % history.length;
    }

    @Override
    public boolean isPossible() {
        double modder = 0;
        if (compass < -150 && prevComp > 150) {
            modder = 360 - Math.abs(compass) - Math.abs(prevComp);
        } else if (compass > 150 && prevComp < -150) {
            modder = -(360 - Math.abs(compass) - Math.abs(prevComp));
        } else {
            modder = compass - prevComp;
        }
        grausRodados += modder;

        if (beacon.beaconVisible) {
            lastKnownBeaconAngle = beacon.beaconDir;
            lastKnownBeaconCompass = compass - lastKnownBeaconAngle;
        } else {
            lastKnownBeaconAngle = compass - lastKnownBeaconCompass;
        }

        //System.out.printf("\t ------> Compass: %2.1f    Beacon: %2.1f     Graus: %2.1f\n", compass, lastKnownBeaconAngle, grausRodados);
        
        // IF FOLLOWING WALL 
        if (!facedBeacon && beacon.beaconVisible && (wallOnTheRight || wallOnTheLeft)) {
            wallOnTheRight = false;
            wallOnTheLeft = false;
        } else if (facedBeacon && (wallOnTheRight || wallOnTheLeft)) {
            // if beacon in front and no obstacles
            if ((lastKnownBeaconAngle < 60 && lastKnownBeaconAngle > -60) && irSensor0 < 1.0 && grausRodados < 180 && grausRodados > -180) {//distance < 100) {
                wallOnTheRight = false;
                wallOnTheLeft = false;
            }
            // IF NOT FOLLOWING WALL and facing beacon
        } else if (!wallOnTheRight && !wallOnTheLeft && (startFollowingWithoutBeacon || (beacon.beaconVisible && lastKnownBeaconAngle < 60 && lastKnownBeaconAngle > -60))) {

            if (irSensor2 > 2.0 || irSensor1 > 2.0) {     //|| irSensor0 > 1.0    // if beacon in front and obstacles as well
                //System.out.println("LeftSensor= "+irSensor1+ "    RightSensor="+irSensor2);
                wallOnTheLeft = irSensor1 > irSensor2;
                wallOnTheRight = !wallOnTheLeft;

                facedBeacon = beacon.beaconVisible;

                //distance = 100;
                //inertia = 0;
                //anguloInicial = compass;
                grausRodados = 0;
            }
        }

        prevComp = compass;

        return wallOnTheRight || wallOnTheLeft;
    }

    @Override
    public String getName() {
        return "FollowTheWall";
    }

}

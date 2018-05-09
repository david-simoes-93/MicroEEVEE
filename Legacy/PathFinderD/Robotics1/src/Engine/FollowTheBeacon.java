package Engine;

import static Engine.Sensors.*;
import ciberIF.ciberIF;
import static Engine.Repetitions.*;

public class FollowTheBeacon implements Behaviour {

    @Override
    public void execute(ciberIF cif) {
        // if no obstacles and beacon to the left
        if (beacon.beaconDir > 60.0) {
            cif.DriveMotors(-0.1, 0.1);
            history[counter] = LEFT;
        } // if no obstacles and beacon to the right
        else if (beacon.beaconDir < -60.0) {
            cif.DriveMotors(0.1, -0.1);
            history[counter] = RIGHT;
        } else if (beacon.beaconDir > 20.0) {
            cif.DriveMotors(0.0, 0.1);
            history[counter] = -1;
        } // if no obstacles and beacon to the right
        else if (beacon.beaconDir < -20.0) {
            cif.DriveMotors(0.1, 0.0);
            history[counter] = -1;
        } else if (beacon.beaconDir > 5.0) {
            cif.DriveMotors(0.05, 0.1);
            history[counter] = -1;
        } // if no obstacles and beacon to the right
        else if (beacon.beaconDir < -5.0) {
            cif.DriveMotors(0.1, 0.05);
            history[counter] = -1;
        } // if no obstacles and beacon "in front"
        else if(irSensor0<1 && irSensor1<2 && irSensor2<2 ){
            cif.DriveMotors(0.1, 0.1);
            history[counter] = -1;
        } else{
            cif.DriveMotors(0.05, 0.05);
            history[counter] = -1;
        }

        counter = (counter + 1) % history.length;

    }

    @Override
    public boolean isPossible() {
        return ((irSensor0 <= 2.0 && irSensor1 <= 2.0 && irSensor2 <= 2.0 && beacon.beaconVisible) || // if no obstacles at >=1 distance
                (beacon.beaconVisible && (beacon.beaconDir < -60 || beacon.beaconDir > 60)));                       // beacon is visible
    }

    @Override
    public String getName() {
        return "FollowTheBeacon";
    }

}

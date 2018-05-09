package Engine;

import static Engine.Sensors.*;
import ciberIF.ciberIF;

public class Wander implements Behaviour {

    @Override
    public void execute(ciberIF cif) {
        if (irSensor0 < 0.5 && irSensor1 < 1 && irSensor2 < 1) {
            cif.DriveMotors(0.15, 0.15);
        } else if (irSensor0 < 1 && irSensor1 < 2 && irSensor2 < 2) {
            cif.DriveMotors(0.1, 0.1);
        } else {
            cif.DriveMotors(0.05, 0.05);
        }
    }

    @Override
    public boolean isPossible() {
        return true;
    }

    @Override
    public String getName() {
        return "Wander";
    }
}

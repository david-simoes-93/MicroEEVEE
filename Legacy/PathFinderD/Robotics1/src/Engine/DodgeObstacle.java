package Engine;

import static Engine.Sensors.*;
import ciberIF.ciberIF;
import static Engine.Repetitions.*;

public class DodgeObstacle implements Behaviour {

    @Override
    public void execute(ciberIF cif) {
        /*boolean avoider = avoidRepetitions();
        if(avoider)
            System.out.println("AVOID!");
        */
        // Rotate right on obstacles close in front or close on the left
        if( irSensor1> irSensor2 ) { // avoider ||
    	    cif.DriveMotors(0.1,-0.1);
            history[counter]=RIGHT;
        }
        // Rotate left in obstacles close on the right
        else {
            cif.DriveMotors(-0.1,0.1);
            history[counter]=LEFT;
        }
        
        counter=(counter+1)%history.length;
    }

    @Override
    public boolean isPossible() {
        return (irSensor0 > 3.0 || irSensor1 > 3.0 || irSensor2 > 3.0);     //there's an obstacle somewhere at front/right/left
    }

    @Override
    public String getName() {
        return "DodgeObstacle";
    }
    
}

/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package Engine;

import ciberIF.*;

/**
 *
 * @author nuno
 */
public class Sensors {
    
    public static double radius = 10.0;
    //public static int currS, currN;
    //public static gpsMeasure south[]= new gpsMeasure[2];
    //public static gpsMeasure north[]= new gpsMeasure[2];
    public static double irSensor0, irSensor1, irSensor2, compass;
    public static beaconMeasure beacon;
    public static int ground;
    public static boolean collision;
    //public static double x, y, dir;
    
    public static boolean isOnRadius(gpsMeasure[] before, double nowX, double nowY){
  
        double dist=0;
        for(int i = 0; i < 2; i++){
            dist = Math.sqrt( (Math.pow((nowX - before[i].x) , 2))
                        + (Math.pow( (nowY - before[i].y) , 2)) );
            System.out.println("dist= "+dist+"\n");
            if(dist <= radius)
                return true;
        }
            return false;
    }
    
}

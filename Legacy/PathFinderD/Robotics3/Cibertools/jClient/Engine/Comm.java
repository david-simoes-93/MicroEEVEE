/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package Engine;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

/**
 *
 * @author nuno
 */
public class Comm {

    public static enum TypeOfMsg {

        HELLO, MAP_UPDATE, BEACONFOUND, ORDER, HOMEINFO, UNKNOWN, READYTOQUIT

    }

    public static enum ExploreDir {

        UP_RIGHT, DOWN_RIGHT, UP_LEFT, DOWN_LEFT, LEFT, RIGHT, UP, DOWN
        //1             2       3       4           5       6  7    8
    }
    public static boolean debug = false, coordOrd = false, firstHello = false;
    public static boolean validOrder = false;
    public static int robID, expDirID, seqNum;

    public static String orderToSend = new String();
    public static TypeOfMsg rcvType, sendType;
    public static ExploreDir expDir;

    public static final int MsgSize = 100;
    public static final int MaxHeaderSizeMU = 31;
    public static final int mapSize = MsgSize - MaxHeaderSizeMU;
    public static final double homeRadius = 2.0;

    public static boolean realBeaconFound = false;
    public static boolean[] withinRangeRob = new boolean[6];            // 0 ignored for direct mapping
    public static boolean[] withinIndirectRangeRob = new boolean[6];    // 0 ignored for direct mapping
    public static boolean[] readyToQuit = new boolean[6];

    public static int[] toExploreDirs = new int[6];
    public static int[] lastToExploreDirs = new int[6];

    public static double[] beaconPts = new double[2];

    public static double[][] robPos = new double[6][2];
    public static boolean[] helloRcvFrom = new boolean[6];
    public static boolean[] sentNonCoord = new boolean[6];
    public static boolean reallyReadyToQuit=false;
    
    public static boolean allReadyToQuit(int id){
        for(int i=1; i<6; i++){
            if(i==id) continue;
            if(!readyToQuit[i]) return false;
        }
        
        if(reallyReadyToQuit) return true;
        else{
            reallyReadyToQuit=true;
            return false;
        }
    }
    
    public static ExploreDir getDirFromLostRobot(int self) {
        int[] robsOnRange = new int[getRobotsWithinRange()];
        if (robsOnRange.length == 5) {
            System.out.println("trying to find lost robot with all robots withing range!");
            return ExploreDir.DOWN;
        }
        int[] robsOffRange = new int[5 - robsOnRange.length];

        for (int i = 1, j1 = 0, j2 = 0; i < 6; i++) {
            if (withinRangeRob[i]) {
                robsOnRange[j1] = i;
                j1++;
            } else {
                robsOffRange[j2] = i;
                j2++;
            }
        }

        for (int j1 = 0, j2 = 0; j1 < robsOnRange.length; j1++) {
            if (robsOnRange[j1] == self) {
                ExploreDir retVal=getDirEnum(toExploreDirs[robsOffRange[j2]]);

                
                //if(Sensors.getDistance(new double[]{myX, myY}, Sensors.map.getRangeTarget(retVal))<4){
                //    retVal=getDirEnumInt(new Random().nextInt(8) + 1);
               // }

                //System.out.println("robot" + self + " searching for robot" + robsOffRange[j2]+ " with dir "+retVal);
                return retVal;
            }
            j2 = (j2 + 1) % robsOffRange.length;
        }

        System.out.println("the target robot has unknown dir. randomizing");
        return getDirEnum(new Random().nextInt(8) + 1);
    }

    public static int getRobotsWithinRange() {
        int counter = 0;
        for (int i = 1; i < 6; i++) {
            if (withinRangeRob[i]) {
                counter++;
            }
        }
        return counter;
    }

    //algorithm to choose the exploreDir of each robot that is in our reach
    public static void coordinateRob() {

        int i, j, cont = 0;
        double radius = 4.0, tempX, tempY, meanX = 0.0, meanY = 0.0, sumX = 0.0, sumY = 0.0;
        boolean line = true, col = true;

        //UP_RIGHT(1), DOWN_RIGHT(2), UP_LEFT(3), DOWN_LEFT(4)          
        //just neighbour robs
        for (i = 2; i < 6; i++) {
            for (j = 0; j < 2; j++) {
                if (robPos[i][0] >= 0 && robPos[i][1] >= 0) {
                    meanX += robPos[i][0];
                    meanY += robPos[i][1];
                    cont++;
                }
            }
        }
        //calculate mean of X and Y
        meanX /= cont;
        meanY /= cont;

        //if we don't have pos of noone, stop
        if (meanX <= 0 || meanY <= 0) {
            System.out.println("EMPTY RETURN");
            return;
        }

        //verify if all are within radius from calculated mean
        //in order to check if robots are aligned horiz. or vertic.
        for (i = 2; i < 6; i++) {
            for (j = 0; j < 2; j++) {
                if (robPos[i][0] >= 0) {
                    sumX += robPos[i][0]-meanX;
                    sumY += robPos[i][1]-meanY;
                    if ((meanX + radius) < robPos[i][0] || (meanX - radius) > robPos[i][0]) {
                        col = false;
                    }
                    if ((meanY + radius) < robPos[i][1] || (meanY - radius) > robPos[i][1]) {
                        line = false;
                    }
                }
            }
        }
        if (col && line) {
            if (sumX < sumY) {
                line = false;
            } else {
                col = false;
            }
        }
        /* Start sending orders*/
        int[] orderedIndex;     //array with descendent order indexes to send orders accordingly
        boolean allThere = true;

        //if robots are vertically aligned
        if (col) {
            if (true) {
                System.out.println("Vertically Aligned");
            }
            int[] val = {1, 3, 2, 4};
            int colCnt = 0;
            //obter indexes dos robs ordenados (ordem decrescente)
            orderedIndex = decrescentOrder(robPos, 1);

            if (debug) {
                System.out.print("OrderedIndex=");
                for (i = 0; i < 4; i++) {
                    System.out.print(" " + orderedIndex[i]);
                }
                System.out.println();
            }
            //check if all chosen to switch
            for (i = 0; i < 4; i++) {
                if (orderedIndex[i] < 2 || orderedIndex[i] > 5) {
                    allThere = false;
                }
            }
            if (allThere) {
                //check which has X higher and switch index if first has lower X
                if (robPos[orderedIndex[0]][0] < robPos[orderedIndex[1]][0]) {
                    int a = orderedIndex[0];
                    orderedIndex[0] = orderedIndex[1];
                    orderedIndex[1] = a;
                }
                if (robPos[orderedIndex[2]][0] < robPos[orderedIndex[3]][0]) {
                    int a = orderedIndex[2];
                    orderedIndex[2] = orderedIndex[3];
                    orderedIndex[3] = a;
                }
            }
            //enviar os 2 + acima para os cantos de cima
            for (i = 0; i < orderedIndex.length; i++) {
                //os robs q tiverem valores sao mandados para algum sitio, os outros nao
                if (robPos[orderedIndex[i]][1] > 0) {
                    toExploreDirs[orderedIndex[i]] = val[colCnt];
                    if (debug) {
                        System.out.println("toExploreDirs[" + orderedIndex[i] + "] got dir: " + val[colCnt]);
                    }
                    colCnt++;
                }
            }
            //if robots are horizontally aligned    
        } else if (line) {
            if (true) {
                System.out.println("Horizontally Aligned");
            }
            int linCnt = 1;

            //obter indexes dos robs ordenados (ordem decrescente)
            orderedIndex = decrescentOrder(robPos, 0);

            //check if all chosen to switch
            for (i = 0; i < 4; i++) {
                if (orderedIndex[i] < 2 || orderedIndex[i] > 5) {
                    allThere = false;
                }
            }
            if (allThere) {
                //check which has Y higher and switch index if first has lower Y
                if (robPos[orderedIndex[0]][1] < robPos[orderedIndex[1]][1]) {
                    System.out.println("SWITCH");
                    int a = orderedIndex[0];
                    orderedIndex[0] = orderedIndex[1];
                    orderedIndex[1] = a;
                }
                if (robPos[orderedIndex[2]][1] < robPos[orderedIndex[3]][1]) {
                    System.out.println("SWITCH");
                    int a = orderedIndex[2];
                    orderedIndex[2] = orderedIndex[3];
                    orderedIndex[3] = a;
                }
            }
            //enviar os 2 + a direita para os cantos da direita
            for (i = 0; i < orderedIndex.length; i++) {
                if (robPos[orderedIndex[i]][0] > 0) {
                    toExploreDirs[orderedIndex[i]] = linCnt;
                    linCnt++;
                }
            }
        } else {
            //else random value
            Random rnd = new Random();
            for (i = 2; i < robPos.length; i++) {
                toExploreDirs[i] = rnd.nextInt(4) + 1;
            }

            if (debug) {
                System.out.println("Robots NOT on Line or Column");
            }
        }

    }

    //x = 0 ; y = 1
    private static int[] decrescentOrder(double[][] robPos, int coord) {
        int i;
        int[] temp = new int[4];
        double[] tempD = new double[4];

        //copy values of positions of other robs
        for (i = 2; i < robPos.length; i++) {
            tempD[i - 2] = robPos[i][coord];
            if (debug) {
                System.out.print(" " + tempD[i - 2]);
            }
        }
        if (debug) {
            System.out.println();
        }
        Arrays.sort(tempD); //ascendent order of array
        ArrayList<Double> ar = new ArrayList();

        //add to array list by descendent order
        int h = 0;
        for (i = tempD.length - 1; i >= 0; i--) {
            ar.add(tempD[i]);
            if (debug) {
                System.out.print(" " + ar.get(h++));
            }
        }
        if (debug) {
            System.out.println();
        }

        //get index of robot with highest values
        for (i = 0; i < 4; i++) {
            if (myIndexOf(ar.get(i), coord) != -1) {
                temp[i] = myIndexOf(ar.get(i), coord);
            }
        }
        return temp;
    }

    private static int myIndexOf(double elem, int coord) {
        for (int i = 0; i < robPos.length; i++) {
            if (robPos[i][coord] == elem) {
                return i;
            }
        }
        return -1;
    }

    public static int getDirID(ExploreDir e) {
        switch (e) {
            case UP_RIGHT:
                return 1;
            case DOWN_RIGHT:
                return 2;
            case DOWN_LEFT:
                return 3;
            case UP_LEFT:
                return 4;
            case LEFT:
                return 5;
            case RIGHT:
                return 6;
            case UP:
                return 7;
            case DOWN:
                return 8;
            default:
                return 0;
        }
    }

    public static ExploreDir getDirEnum(int e) {

        switch (e) {
            case '1':
                return ExploreDir.UP_RIGHT;
            case '2':
                return ExploreDir.DOWN_RIGHT;
            case '3':
                return ExploreDir.UP_LEFT;
            case '4':
                return ExploreDir.DOWN_LEFT;
            case '5':
                return ExploreDir.LEFT;
            case '6':
                return ExploreDir.RIGHT;
            case '7':
                return ExploreDir.UP;
            case '8':
                return ExploreDir.DOWN;
            case 1:
                return ExploreDir.UP_RIGHT;
            case 2:
                return ExploreDir.DOWN_RIGHT;
            case 3:
                return ExploreDir.UP_LEFT;
            case 4:
                return ExploreDir.DOWN_LEFT;
            case 5:
                return ExploreDir.LEFT;
            case 6:
                return ExploreDir.RIGHT;
            case 7:
                return ExploreDir.UP;
            case 8:
                return ExploreDir.DOWN;
            default:
                return ExploreDir.UP_RIGHT;
        }
    }

    public static boolean receivedHello(int helloFrom) {

        return helloRcvFrom[helloFrom];
    }

}

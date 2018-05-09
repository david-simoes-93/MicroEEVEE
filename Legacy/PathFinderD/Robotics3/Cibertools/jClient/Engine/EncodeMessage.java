package Engine;

import static Engine.Comm.*;
import static Engine.Sensors.*;
import Engine.Comm.TypeOfMsg;
import java.util.Arrays;

public class EncodeMessage {

    private TypeOfMsg TOM;
    private Map cMap;
    private int typeOfMsg, cont = 1;
    private String msg;
    private boolean debug = false;
    private int[] aux;

    public EncodeMessage() {

        //Initialize variables and structures 
        msg = new String();
        aux = new int[4];

    }

    public void setTypeOfMsg(TypeOfMsg tom, Map origMap) {
        cMap = origMap;
        //resetting important internal values
        clearInternalVars();

        //define TOM to write to msg to send
        switch (tom) {
            case HELLO:
                typeOfMsg = 1;
                break;
            case MAP_UPDATE:
                typeOfMsg = 2;
                break;
            case BEACONFOUND:
                typeOfMsg = 3;
                break;
            case ORDER:
                typeOfMsg = 5;
                break;
            case HOMEINFO:
                typeOfMsg = 6;
                break;
            case READYTOQUIT:
                typeOfMsg=7;
                break;
            default:
                System.out.println("Unknown type of message!");
                break;
        }
        //define enum val
        TOM = tom;
        // write TOM
        msg += typeOfMsg;

        if (debug) {
            System.out.println("Msg= " + msg);
        }
    }

    public String encodeHello(int myID) {

        //updating my Pos
        robPos[myID][0] = myX;
        robPos[myID][1] = myY;
        //saving it on my array and sending it to others
        for (int i = 1; i < robPos.length; i++) {
            for (int j = 0; j < 2; j++) {
                msg += String.format("%06.1f", robPos[i][j]);
            }
        }
        if (debug) {
            System.out.printf("Hello: myX=%06.1f myY=%06.1f\n", myX, myY);
        }
        return msg;
    }

    public String encodeOrder() {
        //our algorithm to choose the order given the positions we know of other robots
        //System.out.println("ENTERING coordinateRob");
        coordinateRob();
        //System.out.println("LEAVING coordinateRob");
        //[ rob1, rob2, rob3, rob4, rob5]
        if (debug) {
            System.out.printf("Order: exploreDir=");
        }

        for (int i = 1; i < toExploreDirs.length; i++) {
            msg += toExploreDirs[i];
            if (true) {
                System.out.print(" " + toExploreDirs[i]);
            }
        }
        if (debug) {
            System.out.println(" seqNum=" + seqNum);
        }

        //include order SeqNum
        msg += ";" + seqNum + ";";

        //update seqNum if values are different
        for (int i = 2; i < toExploreDirs.length; i++) {
            if (toExploreDirs[i] != lastToExploreDirs[i]) {
                //System.out.printf("%d = %d\n", toExploreDirs[i], lastToExploreDirs[i]);
                seqNum++;
                break;
            }
        }
        //update lastToExploreDirs
        lastToExploreDirs = Arrays.copyOf(toExploreDirs, toExploreDirs.length);

        return msg;
    }

    public String encodeMapUpdate() {
        //System.out.println("\nENCODE number "+cont);
        cont++;
        msg = cMap.getMessageMap(MsgSize, msg);

        if (debug) {
            System.out.println("Msg at end of MapUpdate= " + msg);
        }

        return msg;
    }

    public String encodeBeaconFound(double x, double y) {
        msg += ";" + String.format("%3.1f;%3.1f;", x, y);

        for (int i = 1; i < 6; i++) {
            if (withinRangeRob[i]) {
                msg += i;
            }
        }

        return msg;
    }
    
    public String encodeRTQ() {
        return msg;
    }

    public static String encodeNonCoordOrder() {
        String ret = "";
        int tom5 = 5;
        ret += tom5;
        for (int i = 1; i < toExploreDirs.length; i++) {
            ret += toExploreDirs[i];
        }
        ret += ";" + seqNum + ";";

        return ret;
    }

    private void clearInternalVars() {
        msg = "";
    }

}

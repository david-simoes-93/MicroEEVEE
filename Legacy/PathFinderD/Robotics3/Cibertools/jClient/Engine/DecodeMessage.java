package Engine;

import static Engine.Comm.*;
import Engine.Comm.TypeOfMsg;
import static Engine.Sensors.pathTaken;
import java.awt.Color;

public class DecodeMessage {

    private boolean debug = false, decoded = false;
    private TypeOfMsg TOM;
    private String rcvMsg;
    private int mapReadSize, typeOfMsg, msgSender, rcvSeqNum;
    private byte[] mapRead;
    private Map cMap;
    private int[] aux;
    private int[] mapLimits;
    private final double[] beaconCenterPoint;

    public DecodeMessage() {

        // Initialize variables and structures 
        mapReadSize = 0;
        aux = new int[4];
        mapLimits = new int[4];	// xmin xmax coff loff
        beaconCenterPoint = new double[2];
    }

    public void decode(String message, Map origMap, int msgSender) {

        cMap = origMap;
        //clean last msg read and other fields
        clearInternalVars();

        rcvMsg = message;
        this.msgSender = msgSender;

        switch (rcvMsg.charAt(0)) {
            case '1':
                typeOfMsg = rcvMsg.charAt(0);
                TOM = TypeOfMsg.HELLO;
                if (debug) {
                    System.out.println("Type of message received: " + TOM.name());
                }
                //decodeHello();
                break;
            case '2':
                typeOfMsg = rcvMsg.charAt(0);
                TOM = TypeOfMsg.MAP_UPDATE;
                if (debug) {
                    System.out.println("Type of message received: " + TOM.name());
                }
                decodeMapUpdate();
                break;
            case '3':
                typeOfMsg = rcvMsg.charAt(0);
                TOM = TypeOfMsg.BEACONFOUND;
                if (debug) {
                    System.out.println("Type of message received: " + TOM.name());
                }
                decodeBeaconFound();
                break;
            case '5':
                typeOfMsg = rcvMsg.charAt(0);
                TOM = TypeOfMsg.ORDER;
                if (debug) {
                    System.out.println("Type of message received: " + TOM.name());
                }

                if (robID != 1 && canDecode()) //if im not coordinator decodeOrder
                {
                    decodeOrder();
                }
                break;
            case '6':
                typeOfMsg = rcvMsg.charAt(0);
                TOM = TypeOfMsg.HOMEINFO;
                decodeHomeInfo();
                break;
            case '7':
                typeOfMsg = rcvMsg.charAt(0);
                TOM = TypeOfMsg.READYTOQUIT;
                decodeRTQ();
                break;
            default:
                System.out.println("Unknown type of message!\n");
                TOM = TypeOfMsg.UNKNOWN;
                if (debug) {
                    System.out.println("Type of message received: " + TOM.name());
                }
                break;
        }
    }

    private void decodeRTQ(){
        readyToQuit[msgSender]=true;
    }
    
    private boolean canDecode() {

        //read seq Num
        String aux[] = rcvMsg.split(";");
        rcvSeqNum = Integer.parseInt(aux[1]);

        if (debug) {
            System.out.println("rcvSeqNum=" + rcvSeqNum);
        }

        //check if order is fresher
        if (rcvSeqNum > seqNum) {
            seqNum = rcvSeqNum;
            decoded = true;
        } else {
            decoded = false;
            if (debug) {
                System.out.println("Ignoring Order");
            }
        }
        return decoded;
    }

    public void decodeHello(int sender) {
        int aux = 1;

        //saving it on my array and sending it to others
        for (int i = 1; i < robPos.length; i++) {
            if (i != robID && Double.parseDouble(rcvMsg.substring(1 + (aux - 1) * 6, 1 + (aux * 6))) >= 0.0) {     // doesnt read its own position
                robPos[i][0] = Double.parseDouble(rcvMsg.substring(1 + (aux - 1) * 6, 1 + (aux * 6)));
                robPos[i][1] = Double.parseDouble(rcvMsg.substring(1 + (aux * 6), 1 + ((aux + 1) * 6)));
            }
            aux += 2;
        }

        if (debug) {
            System.out.printf("decodeHello from %d:", sender);
            for (int i = 0; i < robPos.length; i++) {
                System.out.printf(" (%06.1f,%06.1f)", robPos[i][0], robPos[i][1]);
            }
            System.out.println();
        }

    }

    private void decodeHomeInfo() {
        String[] subs = rcvMsg.split(";");
        Sensors.startingX = Double.parseDouble(subs[1]);
        Sensors.startingY = Double.parseDouble(subs[2]);

        double[] start = new double[]{Sensors.startingX, Sensors.startingY};

        pathTaken.add(0, start);
        for (int i = 0; i < pathTaken.size(); i++) {
            double[] temp = pathTaken.get(i);
            if (cMap.gui != null) {
                cMap.gui.paintPermanentPoint(cMap.getCellfromGPS_x(temp[0]), cMap.getCellfromGPS_y(temp[1]), Color.red);
            }
        }
    }

    private void decodeOrder() {

        if (debug) {
            System.out.print("Im number " + robID + " ");
        }
        //save new order of expDir
        expDir = getDirEnum(rcvMsg.charAt(robID));

        //update toExploreDirs to know where everyone is going
        //System.out.print("Decoding toExploreDirs:");
        for (int i = 1; i < toExploreDirs.length; i++) {
            toExploreDirs[i] = rcvMsg.charAt(i);
            //System.out.print(" " + getDirEnum(toExploreDirs[i]).name());
        }
        //System.out.println();

        if (expDir.compareTo(getDirEnum(toExploreDirs[robID])) != 0) {
            System.out.println("\nexpDir and toExploreDirs[robID] DIFFERENT\n");
        }

        if (debug) {
            System.out.println("Msg Received:" + rcvMsg + " | Order received:" + expDir.name());
        }

    }

    private void decodeMapUpdate() {

        //xmin, xmax, yoff, xoff, myX, myY
        String temp[] = new String[6];
        //retrieve the GPS values and add point (4+"."+1)
        temp[0] = rcvMsg.substring(1, 5) + "." + rcvMsg.charAt(5);
        temp[1] = rcvMsg.substring(6, 10) + "." + rcvMsg.charAt(10);
        temp[2] = rcvMsg.substring(11, 15) + "." + rcvMsg.charAt(15);
        temp[3] = rcvMsg.substring(16, 20) + "." + rcvMsg.charAt(20);
        temp[4] = rcvMsg.substring(21, 25) + "." + rcvMsg.charAt(25);
        temp[5] = rcvMsg.substring(26, 30) + "." + rcvMsg.charAt(30);

        //convert from GPS to Cell coordinates
        mapLimits[0] = cMap.getCellfromGPS_x(Double.parseDouble(temp[0]));
        mapLimits[1] = cMap.getCellfromGPS_x(Double.parseDouble(temp[1]));
        mapLimits[2] = cMap.getCellfromGPS_y(Double.parseDouble(temp[2]));
        mapLimits[3] = cMap.getCellfromGPS_x(Double.parseDouble(temp[3]));
        robPos[msgSender][0] = Double.parseDouble(temp[4]);
        robPos[msgSender][1] = Double.parseDouble(temp[5]);

        cMap.setRobotPos(msgSender, cMap.getCellfromGPS_x(robPos[msgSender][0]),cMap.getCellfromGPS_y(robPos[msgSender][1]));

        if (debug) {
            //System.out.println("DECODE number "+cont);
            System.out.println("Received Msg: " + rcvMsg);
            //System.out.printf("GPS Limits xmin=%0.51f xmax=%05.1f yoffset=%05.1f xoffset=%05.1f\n", Double.parseDouble(temp[0]),
            //        Double.parseDouble(temp[1]), Double.parseDouble(temp[2]), Double.parseDouble(temp[3]));
            System.out.printf("From %d received position (%05.1f, %05.1f)\n", msgSender, Double.parseDouble(temp[4]), Double.parseDouble(temp[5]));
            //System.out.println("Map Limits: xmin=" + mapLimits[0] + " xmax=" + mapLimits[1] + " yoffset=" + mapLimits[2] + " Xoffset=" + mapLimits[3]);
        }

        //size of map and allocated memory
        String sub = rcvMsg.substring(MaxHeaderSizeMU);
        mapReadSize = sub.length();
        mapRead = new byte[mapReadSize];

        for (int j = 0; j < sub.length(); j++) {
            //fill map from received msg
            mapRead[j] = (byte) Character.getNumericValue(sub.charAt(j));

            if (debug) {
                System.out.print(sub.charAt(j) + " ");
            }

        }

    }

    private void decodeBeaconFound() {
        String[] subs = rcvMsg.split(";");
        beaconCenterPoint[0] = Double.parseDouble(subs[1]);
        beaconCenterPoint[1] = Double.parseDouble(subs[2]);

        for (int i = 0; i < subs[3].length(); i++) {
            withinIndirectRangeRob[Integer.parseInt(subs[3].charAt(i) + "")] = true;
        }
    }

    // reset values of important internal values
    private void clearInternalVars() {
        int i;

        /* Just to make sure */
        //clear fields
        for (i = 0; i < MsgSize - 1; i++) {
            if (i < 2) {
                beaconCenterPoint[i] = 0;
            }
            if (i < 3) {
                mapLimits[i] = 0; //clear map limits
            }
            if (i < 4) {
                aux[i] = 0;	//clear aux
            }
            if (i < mapReadSize) {
                mapRead[i] = 0;	//clear map
            }
        }

    }

    /* Getters */
    public TypeOfMsg getTypeOfMsg() {
        return TOM;
    }

    public int getTomID() {
        return typeOfMsg;
    }

    public double[] getBeaconCenterPoint() {
        return beaconCenterPoint;
    }

    public byte[] getMapRead() {
        return mapRead;
    }

    public int[] getMapLimits() {
        return mapLimits;
    }

    public String getRcvMsg() {
        return rcvMsg;
    }

    public int getRcvSeqNum() {
        return rcvSeqNum;
    }

    public boolean getDecodedFlag() {
        return decoded;
    }

}

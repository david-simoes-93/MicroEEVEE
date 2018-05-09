/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package Engine;

import Engine.Comm.ExploreDir;
import static Engine.Sensors.myX;
import static Engine.Sensors.myY;
import java.awt.Color;
import java.util.ArrayList;
import java.util.Random;

public class Map {

    /*
     // to do: robot cornering algorithm - line between robot and target; check for obstacles
     // to do: coordinate coming home one at a time ?
     */
    public static final byte blackCode = 1, greyCode = 0, clearCode = 2, externalClearCode = 5, externalBlackCode = 6;
    public byte[][] actualMap = new byte[mapWidthPixels][mapHeightPixels];
    public byte[][] mapReadings = new byte[mapWidthPixels][mapHeightPixels];
    ArrayList<Line> beaconLines = new ArrayList();
    ArrayList<Cell> beaconPoints = new ArrayList();

    private static int mapWidthPixels, mapHeightPixels;
    private int minX = mapWidthPixels, minY = mapHeightPixels, maxX, maxY;
    private int foreignminX = mapWidthPixels, foreignminY = mapHeightPixels, foreignmaxX, foreignmaxY;
    private Cell nextUnexploredTarget = null;
    private int offsetX, offsetY;
    public MapGUI gui;
    private int columnOffset, lineOffset;
    private int[][] robots = new int[5][2];

    public void setRobotPos(int id, int x, int y) {
        if (robots[id - 1][0] != 0 && robots[id - 1][1] != 0) {
            setRobot(robots[id - 1][0], robots[id - 1][1], (byte) 32, Color.LIGHT_GRAY, externalClearCode);
        }

        robots[id - 1][0] = x;
        robots[id - 1][1] = y;

        if (x != 0 && y != 0) {
            setRobot(x, y, (byte) 0, Color.GREEN, externalBlackCode);
        }
    }

    private void setRobot(int x, int y, byte val, Color col, byte code) {
        mapReadings[x][y] = val;
        mapReadings[x + 1][y] = val;
        mapReadings[x - 1][y] = val;
        mapReadings[x][y + 1] = val;
        mapReadings[x][y - 1] = val;

        mapReadings[x + 1][y + 1] = val;
        mapReadings[x - 1][y - 1] = val;
        mapReadings[x - 1][y + 1] = val;
        mapReadings[x + 1][y - 1] = val;

        actualMap[x][y] = code;
        actualMap[x + 1][y] = code;
        actualMap[x - 1][y] = code;
        actualMap[x][y + 1] = code;
        actualMap[x][y - 1] = code;

        actualMap[x + 1][y + 1] = code;
        actualMap[x - 1][y - 1] = code;
        actualMap[x - 1][y + 1] = code;
        actualMap[x + 1][y - 1] = code;

        if (gui != null) {
            gui.paintPoint(x, y, col);
            gui.paintPoint(x + 1, y, col);
            gui.paintPoint(x - 1, y, col);
            gui.paintPoint(x, y + 1, col);
            gui.paintPoint(x, y - 1, col);
            /*  gui.paintPoint(x + 1, y+1, col);
             gui.paintPoint(x - 1, y-1, col);
             gui.paintPoint(x-1, y + 1, col);
             gui.paintPoint(x+1, y - 1, col);*/
        }

    }

    // inserts an external map reading into the current map
    public void setActualMap(int minX2, int maxX2, int coff2, int loff2, byte[] rcvMap) {
        setNewExternalBounds(minX2, coff2);
        setNewExternalBounds(maxX2, coff2);

        for (int i = 0; i < rcvMap.length; i++) {
            byte actualMapCell = rcvMap[i];

            if (actualMapCell == clearCode && actualMap[loff2][coff2] == greyCode) {
                actualMap[loff2][coff2] = externalClearCode;
                if (gui != null) {
                    gui.paintPoint(loff2, coff2, getColor(externalClearCode));
                }
            } else if (actualMapCell == blackCode && actualMap[loff2][coff2] == greyCode) {
                actualMap[loff2][coff2] = externalBlackCode;
                if (gui != null) {
                    gui.paintPoint(loff2, coff2, getColor(externalBlackCode));
                }
            }

            //change column and (if needed) row
            loff2++;
            if (loff2 > maxX2) {
                loff2 = minX2;
                coff2++;
            }
        }
    }

    // returns a string with a part of the explored map
    public String getMessageMap(int sizeLeft, String t) {

        if (minX == mapWidthPixels || minY == mapHeightPixels) {
            t += "000.0200.0000.0000.000000000000";
            return t;
        }

        if (lineOffset == 0) {
            lineOffset = minX;
        }
        if (columnOffset == 0) {
            columnOffset = minY;
        }

        //System.out.println("reading from "+minX+" to "+maxX+", with currX="+loff+" and currentY="+coff);
        t += String.format("%06.1f%06.1f%06.1f%06.1f%06.1f%06.1f", getGPSfromCell_x(minX), getGPSfromCell_x(maxX), getGPSfromCell_y(columnOffset), getGPSfromCell_x(lineOffset), myX, myY);
        t = t.replace(".", "");

        //fill the rest of message with map values
        while (t.length() < sizeLeft) {
            byte actualMapCell = actualMap[lineOffset][columnOffset];

            // write to msg the map discovery (obstacle, unknown, clear)
            if (actualMapCell == externalBlackCode || actualMapCell == externalClearCode) {
                t += greyCode;
            } else {
                t += actualMapCell;
            }

            //change column and (if needed) row
            lineOffset++;

            if (lineOffset > maxX) {
                lineOffset = minX;
                columnOffset++;
            }

        }

        // if we got to the limit, restart
        if (columnOffset > maxY) {
            columnOffset = minY;
            lineOffset = minX;
        }

        return t;
    }

    // sets the offsets for the map, so that the map coordinates doesnt go over limits
    public void setOffsets(int x, int y) {
        offsetX = x;
        offsetY = y;
    }

    // converts the X value of a GPS coord into the cell coord
    public int getCellfromGPS_x(double gps) {
        return (int) Math.round(gps * 3 + offsetX);
    }

    // converts the Y value of a GPS coord into the cell coord
    public int getCellfromGPS_y(double gps) {
        return (int) Math.round(gps * -3 + offsetY);
    }

    // converts the X value of a Cell coord into the GPS coord
    public double getGPSfromCell_x(int cell) {
        return (cell * 1.0 - offsetX) / 3;
    }

    // converts the Y value of a Cell coord into the GPS coord
    public double getGPSfromCell_y(int cell) {
        return (cell * 1.0 - offsetY) / -3;
    }

    // sets a new reading at a given point
    // if the average of readings gives a new result for that point, it is updated
    public void setReading(int x, int y, byte val, int certainty) {
        byte value;
        if (val == blackCode) {
            value = (byte) (certainty * -1);
        } else {
            value = (byte) (certainty);
        }

        if (mapReadings[x][y] + value > 127) {
            mapReadings[x][y] = 127;
        } else if (mapReadings[x][y] + value < -127) {
            mapReadings[x][y] = -127;
        } else {
            mapReadings[x][y] += value;
        }

        if (mapReadings[x][y] > 32) {
            actualMap[x][y] = clearCode;
            if (gui != null) {
                gui.paintPoint(x, y, Color.WHITE);
            }
        } else if (mapReadings[x][y] < -32) {
            actualMap[x][y] = blackCode;
            if (gui != null) {
                gui.paintPoint(x, y, Color.BLACK);
            }
        }
    }

    // sets a "c" line between "x,y" and "x2,y2", with probability values according to the distance of this line
    private void addLine(int x, int y, int x2, int y2, byte c, double distMin, double distMax) {
        if (x < 0 || y < 0 || x2 < 0 || y2 < 0 || x >= mapWidthPixels || y >= mapHeightPixels || x2 >= mapWidthPixels || y2 >= mapHeightPixels) {
            return;
        }

        setNewBounds(x, y);
        setNewBounds(x2, y2);
        setNewExternalBounds(x, y);
        setNewExternalBounds(x2, y2);

        int wlol = x2 - x;
        int hlol = y2 - y;
        int dx1 = 0, dy1 = 0, dx2 = 0, dy2 = 0;
        if (wlol < 0) {
            dx1 = -1;
        } else if (wlol > 0) {
            dx1 = 1;
        }
        if (hlol < 0) {
            dy1 = -1;
        } else if (hlol > 0) {
            dy1 = 1;
        }
        if (wlol < 0) {
            dx2 = -1;
        } else if (wlol > 0) {
            dx2 = 1;
        }

        int longest = Math.abs(wlol);
        int shortest = Math.abs(hlol);
        if (!(longest > shortest)) {
            longest = Math.abs(hlol);
            shortest = Math.abs(wlol);
            if (hlol < 0) {
                dy2 = -1;
            } else if (hlol > 0) {
                dy2 = 1;
            }
            dx2 = 0;
        }
        int numerator = longest >> 1;

        for (int i = 0; i <= longest; i++) {
            setReading(x, y, c, getRangedVal(distMin, distMax, i + 1, longest + 1));

            numerator += shortest;
            if (!(numerator < longest)) {
                numerator -= longest;
                x += dx1;
                y += dy1;
            } else {
                x += dx2;
                y += dy2;
            }
        }
    }

    private boolean checkLineForObstacles(int x, int y, int x2, int y2) {
        int wlol = x2 - x;
        int hlol = y2 - y;
        int dx1 = 0, dy1 = 0, dx2 = 0, dy2 = 0;
        if (wlol < 0) {
            dx1 = -1;
        } else if (wlol > 0) {
            dx1 = 1;
        }
        if (hlol < 0) {
            dy1 = -1;
        } else if (hlol > 0) {
            dy1 = 1;
        }
        if (wlol < 0) {
            dx2 = -1;
        } else if (wlol > 0) {
            dx2 = 1;
        }

        int longest = Math.abs(wlol);
        int shortest = Math.abs(hlol);
        if (!(longest > shortest)) {
            longest = Math.abs(hlol);
            shortest = Math.abs(wlol);
            if (hlol < 0) {
                dy2 = -1;
            } else if (hlol > 0) {
                dy2 = 1;
            }
            dx2 = 0;
        }
        int numerator = longest >> 1;

        for (int i = 0; i <= longest; i++) {
            if (actualMap[x][y] == externalBlackCode || actualMap[x][y] == blackCode) {
                return true;
            }

            numerator += shortest;
            if (!(numerator < longest)) {
                numerator -= longest;
                x += dx1;
                y += dy1;
            } else {
                x += dx2;
                y += dy2;
            }
        }

        return false;
    }

    // return the probabily value for a given distance from the robot
    public int getRangedVal(double minDist, double maxDist, int curr, int max) {
        double ratio = curr * 1.0 / max;
        int val = (int) ((maxDist - minDist) * ratio + minDist);
        return (int) (10 - val);
    }

    // sets and paints an obstacle at distance "dist" from position"x,y", 60 to the left of "dir"
    public void paintLeftObstacle(int x, int y, double dist, double dir) {
        dir = (dir + 60) * Math.PI / 180;

        int newX = (int) (x + Math.cos(dir) * (dist - 1));
        int newY = (int) (y - Math.sin(dir) * (dist - 1));

        addLine(x, y, newX, newY, clearCode, 0, dist - 1);

        newX = (int) (x + Math.cos(dir) * dist);
        newY = (int) (y - Math.sin(dir) * dist);
        int nextX = (int) (x + Math.cos(dir) * dist + 1);
        int nextY = (int) (y - Math.sin(dir) * dist + 1);

        if (dist < 9.5) {
            addLine(newX, newY, nextX, nextY, blackCode, dist - 1, dist + 1);
        }

        if (x < 0 || y < 0 || nextX < 0 || nextY < 0 || x >= mapWidthPixels || y >= mapHeightPixels || nextX >= mapWidthPixels || nextY >= mapHeightPixels) {
            return;
        }
        setNewBounds(x, y);
        setNewBounds(nextX, nextY);
        setNewExternalBounds(x, y);
        setNewExternalBounds(nextX, nextY);
    }

    // sets and paints an obstacle at distance "dist" from position"x,y", 60 to the right of "dir"
    public void paintRightObstacle(int x, int y, double dist, double dir) {
        dir = (dir - 60) * Math.PI / 180;

        int newX = (int) (x + Math.cos(dir) * (dist - 1));
        int newY = (int) (y - Math.sin(dir) * (dist - 1));

        addLine(x, y, newX, newY, clearCode, 0, dist - 1);

        newX = (int) (x + Math.cos(dir) * dist);
        newY = (int) (y - Math.sin(dir) * dist);
        int nextX = (int) (x + Math.cos(dir) * dist + 1);
        int nextY = (int) (y - Math.sin(dir) * dist + 1);

        if (dist < 9.5) {
            addLine(newX, newY, nextX, nextY, blackCode, dist - 1, dist + 1);
        }

        if (x < 0 || y < 0 || nextX < 0 || nextY < 0 || x >= mapWidthPixels || y >= mapHeightPixels || nextX >= mapWidthPixels || nextY >= mapHeightPixels) {
            return;
        }
        setNewBounds(x, y);
        setNewBounds(nextX, nextY);
        setNewExternalBounds(x, y);
        setNewExternalBounds(nextX, nextY);
    }

    // sets and paints an obstacle at distance "dist" from position"x,y", in the direction of "dir"
    public void paintFrontObstacle(int x, int y, double dist, double dir) {
        dir = dir * Math.PI / 180;

        int newX = (int) (x + Math.cos(dir) * (dist - 1));
        int newY = (int) (y - Math.sin(dir) * (dist - 1));

        addLine(x, y, newX, newY, clearCode, 0, dist - 1);

        // int nextX = (int) (newX + Math.cos(dir) * 2);
        //  int nextY = (int) (newY - Math.sin(dir) * 2);
        newX = (int) (x + Math.cos(dir) * dist);
        newY = (int) (y - Math.sin(dir) * dist);
        int nextX = (int) (x + Math.cos(dir) * dist + 1);
        int nextY = (int) (y - Math.sin(dir) * dist + 1);

        if (dist < 8.5) {
            addLine(newX, newY, nextX, nextY, blackCode, dist - 1, dist + 1);
        }

        if (x < 0 || y < 0 || nextX < 0 || nextY < 0 || x >= mapWidthPixels || y >= mapHeightPixels || nextX >= mapWidthPixels || nextY >= mapHeightPixels) {
            return;
        }
        setNewBounds(x, y);
        setNewBounds(nextX, nextY);
        setNewExternalBounds(x, y);
        setNewExternalBounds(nextX, nextY);
    }

    // sets the minimum and maximum defined map coords
    public void setNewBounds(int x, int y) {
        if (x < minX) {
            minX = x;
        }
        if (x > maxX) {
            maxX = x;
        }
        if (y < minY) {
            minY = y;
        }
        if (y > maxY) {
            maxY = y;
        }
    }

    // sets the minimum and maximum defined map coords, by external information
    public void setNewExternalBounds(int x, int y) {
        if (x < foreignminX) {
            foreignminX = x;
        }
        if (x > foreignmaxX) {
            foreignmaxX = x;
        }
        if (y < foreignminY) {
            foreignminY = y;
        }
        if (y > foreignmaxY) {
            foreignmaxY = y;
        }
    }

    // resets temporary points in the GUI
    public void resetGUI() {
        if (gui != null) {
            gui.removeChangers();
        }
    }

    // sets and paints a clear cross on "x,y"
    public void paintClearRadius(int x, int y) {
        if (x <= 0 || x >= mapWidthPixels - 1 || y <= 0 || y >= mapHeightPixels - 1) {
            return;
        }

        setReading(x + 1, y, clearCode, 32);
        setReading(x - 1, y, clearCode, 32);
        setReading(x, y + 1, clearCode, 32);
        setReading(x, y - 1, clearCode, 32);
        setReading(x, y, clearCode, 32);

        setNewBounds(x - 1, y - 1);
        setNewBounds(x + 1, y + 1);
        setNewExternalBounds(x - 1, y - 1);
        setNewExternalBounds(x + 1, y + 1);

    }

    // add an unrepeatable beacon point
    public boolean setUniqueBeaconPoint(int x, int y) {
        if (x < 0 || y < 0 || x >= mapWidthPixels || y >= mapHeightPixels) {
            return false;
        }

        for (Cell c : beaconPoints) {
            if (c.x == x && c.y == y) {
                return true;
            }
        }

        //System.out.println("\tAdding unique beacon point! (" + x + "," + y + ")");
        beaconPoints.add(new Cell(x, y));
        if (gui != null) {
            //gui.paintPoint(x, y, Color.RED);
        }

        return true;
    }

    // add an repeatable beacon point
    public void setBeaconPoint(int x, int y) {
        if (x < 0 || y < 0 || x >= mapWidthPixels || y >= mapHeightPixels) {
            return;
        }

        //System.out.println("\tAdding foreign beacon point! (" + x + "," + y + ")");
        beaconPoints.add(new Cell(x, y));
        if (gui != null) {
            //gui.paintPoint(x, y, Color.RED);
        }
    }

    public double[] getRangeTarget(ExploreDir order) {
        int desiredX = 0, desiredY = 0;
        switch (order) {
            case UP_RIGHT:
                desiredX = maxX - 2 * 3;
                desiredY = minY + 2 * 3;
                break;
            case DOWN_RIGHT:
                desiredX = maxX - 2 * 3;
                desiredY = maxY - 2 * 3;
                break;
            case DOWN_LEFT:
                desiredX = minX + 2 * 3;
                desiredY = maxY - 2 * 3;
                break;
            case UP_LEFT:
                desiredX = minX + 2 * 3;
                desiredY = minY + 2 * 3;
                break;
            case UP:
                desiredX = (minX + maxX) / 2;
                desiredY = minY + 2 * 3;
                break;
            case DOWN:
                desiredX = (minX + maxX) / 2;
                desiredY = minY + 2 * 3;
                break;
            case LEFT:
                desiredX = minX + 2 * 3;
                desiredY = (minY + maxY) / 2;
                break;
            case RIGHT:
                desiredX = maxX - 2 * 3;
                desiredY = (minY + maxY) / 2;
                break;
            default:
                break;
        }

        return new double[]{getGPSfromCell_x(desiredX), getGPSfromCell_y(desiredY)};
    }

    // return a part of the map to be visited in the hopes of finding friends
    public Cell goToRangeZone(Cell curr, ExploreDir order) {
        int desiredX = 0, desiredY = 0;
        switch (order) {
            case UP_RIGHT:
                desiredX = foreignmaxX - 2 * 3;
                desiredY = foreignminY + 2 * 3;
                break;
            case DOWN_RIGHT:
                desiredX = foreignmaxX - 2 * 3;
                desiredY = foreignmaxY - 2 * 3;
                break;
            case DOWN_LEFT:
                desiredX = foreignminX + 2 * 3;
                desiredY = foreignmaxY - 2 * 3;
                break;
            case UP_LEFT:
                desiredX = foreignminX + 2 * 3;
                desiredY = foreignminY + 2 * 3;
                break;
            case UP:
                desiredX = (foreignminX + foreignmaxX) / 2;
                desiredY = foreignminY + 2 * 3;
                break;
            case DOWN:
                desiredX = (foreignminX + foreignmaxX) / 2;
                desiredY = foreignmaxY - 2 * 3;
                break;
            case LEFT:
                desiredX = foreignminX + 2 * 3;
                desiredY = (foreignminY + foreignmaxY) / 2;
                break;
            case RIGHT:
                desiredX = foreignmaxX - 2 * 3;
                desiredY = (foreignminY + foreignmaxY) / 2;
                break;
            default:
                break;
        }

        Cell target = null, temp;
        for (int i = 0; i < 4 * 3; i += 3) {
            temp = new Cell(desiredX + i, desiredY + i);
            if (!isBlocked(temp, 3)) {
                target = temp;
                break;
            }
            temp = new Cell(desiredX - i, desiredY + i);
            if (!isBlocked(temp, 3)) {
                target = temp;
                break;
            }
            temp = new Cell(desiredX + i, desiredY - i);
            if (!isBlocked(temp, 3)) {
                target = temp;
                break;
            }
            temp = new Cell(desiredX - i, desiredY - i);
            if (!isBlocked(temp, 3)) {
                target = temp;
                break;
            }
        }
        if (target == null) {
            System.out.println("couldnt find a suitable target zone");
            return null;
        }

        return StarSearchExplorer(curr, target, 3);
    }

    // added a new beacon line, mark a new beacon point, try and find a path there
    public Cell getDirectionTarget(Cell curr, double dir, Cell actualCurr) {
        boolean contains = false;
        // check if we have any beacon line pointing at the beacon and starting close to current position
        for (Line l : beaconLines) {
            if (getDistanceBetweenCells(l.a, curr) < 3) {
                contains = true;
                break;
            }
        }

        int dist = 5 * 3;
        int newX = (int) (curr.x + Math.cos(dir) * dist);
        int newY = (int) (curr.y - Math.sin(dir) * dist);
        Cell endLine = new Cell(newX, newY);

        // if we have no such line, add a new line and mark the point it intersects with other lines
        if (!contains) {
            Line newLine = new Line(curr, endLine);
            beaconLines.add(new Line(curr, endLine));

            double x3 = newLine.a.x;
            double y3 = newLine.a.y;
            double x4 = newLine.b.x;
            double y4 = newLine.b.y;

            for (int i = 0; i < beaconLines.size() - 1; i++) {
                double x1 = beaconLines.get(i).a.x;
                double y1 = beaconLines.get(i).a.y;
                double x2 = beaconLines.get(i).b.x;
                double y2 = beaconLines.get(i).b.y;

                //if parallel with other line, skip it
                if ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4) == 0) {
                    continue;
                }

                //intersect with other line
                double Px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4))
                        / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
                double Py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4))
                        / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));

                //if point between both lines' end points, keep it
                if (Px > Math.min(x3, x4) && Px < Math.max(x3, x4) && Py > Math.min(y3, y4) && Py < Math.max(y3, y4)) {
                    if (gui != null) {
                        gui.paintPoint((int) Px, (int) Py, Color.RED);
                    }
                    setUniqueBeaconPoint((int) Px, (int) Py);
                }
            }
        }

        if (!beaconPoints.isEmpty()) {
            return getWayToBeacon(actualCurr);
        } else {
            Cell tar = new Cell(newX, newY);
            tar = findClosestFreeCell(tar, 2);
            return StarSearchExplorerZone(actualCurr, tar, 2, 3);
        }
    }

    // return the target cell in the path to the beacon
    public Cell getWayToBeacon(Cell curr) {
        if (!beaconPoints.isEmpty()) {
            Cell tar = getBeaconEstimation();
            tar = findClosestFreeCell(tar, 2);
            return StarSearchExplorerZone(curr, tar, 2, 3);
            //return StarSearchExplorer(curr, tar, 2);
        }

        return null;
    }

    public Cell getRandomBeaconPoint() {
        return beaconPoints.get(new Random().nextInt(beaconPoints.size()));
    }

    // calculates the estimated beacon position
    public Cell getBeaconEstimation() {
        if (!beaconPoints.isEmpty()) {
            int avgX = 0, avgY = 0;
            for (Cell c : beaconPoints) {
                avgX += c.x;
                avgY += c.y;
            }
            avgX /= beaconPoints.size();
            avgY /= beaconPoints.size();

            if (gui != null) {
                gui.paintPoint(avgX, avgY, Color.red);
            }

            Cell b = new Cell(avgX, avgY);
            /*if (b.x >= 6 && b.y >= 6) {
                for (int i = -6; i < 7; i++) {
                    for (int j = -6; j < 7; j++) {
                        if (getDistanceBetweenCells(b, new Cell(avgX + i, avgY + j)) < 6) {
                            actualMap[avgX+i][avgY+j] = clearCode;
                        }
                    }
                }
            }*/

            return b;
        }
        return null;
    }

    // return the closest unexplored cell in radius of position "currX,currY" with a certain direction
    public Cell getTargetByOrder(int currX, int currY, ExploreDir order) {
        boolean flag = true;

        if (nextUnexploredTarget == null || !isUnexplored(nextUnexploredTarget, 3)) {
            int tempX = 6, tempY = 6;

            int limX = mapWidthPixels, limY = mapHeightPixels;
            int modX = -1, modY = -1;

            //System.out.println("Current Pos: X= "+currX+" Y="+currY);
            switch (order) {
                case UP_RIGHT:
                    //default case
                    tempY = -tempY;
                    modY = 1;
                    break;
                case DOWN_RIGHT:
                    //tempY = -tempY;
                    //modY = -1;
                    break;
                case DOWN_LEFT:
                    tempX = -tempX;
                    modX = 1;
                    break;
                case UP_LEFT:
                    tempX = -tempX;
                    tempY = -tempY;
                    modX = 1;
                    modY = 1;
                    break;
                case UP:
                    tempX = 0;
                    tempY = -tempY;
                    modY = 1;
                    break;
                case DOWN:
                    tempX = 0;
                    break;
                case LEFT:
                    tempX = -tempX;
                    modX = 1;
                    tempY = 0;
                    break;
                case RIGHT:
                    tempY = 0;
                    break;
                default:
                    break;
            }

            while (flag) {
                outerloop:
                for (int x = tempX; -modX * x >= 0; x += modX) {
                    for (int y = tempY; -modY * y >= 0; y += modY) {
                        if (currX + x >= 0 && currX + x < limX && currY + y >= 0 && currY + y < limY) {
                            Cell temp = new Cell(currX + x, currY + y);
                            if (isUnexplored(temp, 3)) {
                                nextUnexploredTarget = temp;
                                //System.out.println("nextUnexploredTarget ("+(currX+x)+","+(currY+y)+")");                                       
                                flag = false;
                                break outerloop;
                            }
                        }
                    }
                }
                //sum or subtract according to direction as it was a cartesian reference
                tempX += -3 * modX; //(3*modX);
                tempY += -3 * modY; //(3*modY);
            }
        }

        if (!flag) {
            System.out.println("next target is " + nextUnexploredTarget);
        }
        if (gui != null) {
            gui.paintTemporaryPoint(nextUnexploredTarget.x, nextUnexploredTarget.y, Color.PINK);
        }

        return StarSearchExplorer(new Cell(currX, currY), nextUnexploredTarget, 3);
    }

    // returns a color based on the byte code
    private Color getColor(byte a) {
        if (a == blackCode) {
            return Color.black;
        }
        if (a == greyCode) {
            return Color.gray;
        }
        if (a == externalBlackCode) {
            return Color.DARK_GRAY;
        }
        if (a == externalClearCode) {
            return Color.LIGHT_GRAY;
        }

        return Color.white;
    }

    // returns a free cell, closest to the "start" cell
    private Cell findClosestFreeCell(Cell start, int radius) {

        while (isBlocked(start, radius) && radius >= 1) {
            for (int i = 1; i <= 3; i++) {

                int[][] randomness = new int[4][2];
                randomness[0] = randomness[1] = randomness[2] = randomness[3] = null;

                int randomIndex = (int) (Math.random() * 4);
                while (randomness[randomIndex] != null) {
                    randomIndex = (int) (Math.random() * 4);
                }
                randomness[randomIndex] = new int[]{i, 0};

                while (randomness[randomIndex] != null) {
                    randomIndex = (int) (Math.random() * 4);
                }
                randomness[randomIndex] = new int[]{-i, 0};

                while (randomness[randomIndex] != null) {
                    randomIndex = (int) (Math.random() * 4);
                }
                randomness[randomIndex] = new int[]{0, i};

                while (randomness[randomIndex] != null) {
                    randomIndex = (int) (Math.random() * 4);
                }
                randomness[randomIndex] = new int[]{0, -i};

                if (!isBlocked(new Cell(start.x + randomness[0][0], start.y + randomness[0][1]), radius)) {
                    start = new Cell(start.x + randomness[0][0], start.y + randomness[0][1]);
                    break;
                } else if (!isBlocked(new Cell(start.x + randomness[1][0], start.y + randomness[1][1]), radius)) {
                    start = new Cell(start.x + randomness[1][0], start.y + randomness[1][1]);
                    break;
                } else if (!isBlocked(new Cell(start.x + randomness[2][0], start.y + randomness[2][1]), radius)) {
                    start = new Cell(start.x + randomness[2][0], start.y + randomness[2][1]);
                    break;
                } else if (!isBlocked(new Cell(start.x + randomness[3][0], start.y + randomness[3][1]), radius)) {
                    start = new Cell(start.x + randomness[3][0], start.y + randomness[3][1]);
                    break;
                }
            }
            radius--;
        }
        if (radius == 0) {
            System.out.println("couldnt find a free nearby cell " + start);
        }

        return start;
    }

    // uses StarSearch algorithm to get the best path from "start" to "goal" through clear and unknown cells
    // when a path isnt found, radius is decreases and path is searched again
    public Cell StarSearchExplorer(Cell start, Cell goal, int radius) {
        int mapLimitMinX = Math.max(0, Math.min(goal.x, foreignminX) - radius * 4), mapLimitMinY = Math.max(0, Math.min(goal.y, foreignminY) - radius * 4);
        int mapLimitMaxX = Math.min(mapWidthPixels, Math.max(goal.x, foreignmaxX) + radius * 4), mapLimitMaxY = Math.min(mapHeightPixels, Math.max(goal.y, foreignmaxY) + radius * 4);
        //System.out.println("\tA* on region ("+mapLimitMinX+","+mapLimitMinY+") and ("+mapLimitMaxX+","+mapLimitMaxY+")");

        Cell[][] origins = new Cell[mapLimitMaxX - mapLimitMinX][mapLimitMaxY - mapLimitMinY];

        // if first cell is blocked on a big radius, find a nearby cell that is not blocked
        start = findClosestFreeCell(start, radius);
        origins[start.x - mapLimitMinX][start.y - mapLimitMinY] = start;

        ArrayList<Cell> front = new ArrayList();
        for (Cell t : getNeighbors(start)) {
            front.add(t);
            origins[t.x - mapLimitMinX][t.y - mapLimitMinY] = start;
        }

        boolean haventFoundTheEnd = true;
        while (!front.isEmpty()) {
            Cell best = findBestOption(front, goal);
            front.remove(best);

            if (goal.x == best.x && goal.y == best.y) {
                haventFoundTheEnd = false;
                break;
            }

            for (Cell neighbour : getNeighbors(best)) {
                // if neighbour hasnt been visited yet and is not blocked 
                if (origins[neighbour.x - mapLimitMinX][neighbour.y - mapLimitMinY] == null && !isBlocked(neighbour, radius, mapLimitMinX, mapLimitMaxX, mapLimitMinY, mapLimitMaxY)) {
                    front.add(neighbour);
                    origins[neighbour.x - mapLimitMinX][neighbour.y - mapLimitMinY] = best;
                }
            }
        }

        if (haventFoundTheEnd) {
            if (radius > 1) {
                System.out.println("\tcould find path to target A*. adjusting radius to " + (radius - 1));
                return StarSearchExplorer(start, goal, radius - 1);
            }
            System.out.println("\tcould find path to target A*. giving up.");
            return null;
        }

        ArrayList<Cell> bestPath = new ArrayList();
        Cell currentPosition = goal;                                                    //Start at the endpoint.
        bestPath.add(goal);
        while (start.x != currentPosition.x || start.y != currentPosition.y) {          //Until we're back at the start.
            currentPosition = origins[currentPosition.x - mapLimitMinX][currentPosition.y - mapLimitMinY];
            bestPath.add(currentPosition);
            if (gui != null) {
                gui.paintTemporaryPoint(currentPosition.x, currentPosition.y, Color.BLUE);
            }
        }

        Cell temp;
        if (bestPath.size() >= 6) {
            temp = bestPath.get(bestPath.size() - 5);

        } else {
            temp = bestPath.get(0);
        }

        if (checkLineForObstacles(start.x, start.y, temp.x, temp.y)) {
            System.out.println("line has obstacles! returning a closer cell");
            temp = bestPath.get(bestPath.size() - 1);
        }

        if (gui != null) {
            gui.paintTemporaryPoint(temp.x, temp.y, Color.PINK);
        }

        //System.out.println("\tA* found a path with radius " + radius + " facing " + temp + " and current " + start);
        return temp;
    }

    // uses StarSearch algorithm to get the best path from "start" to "goal" through clear and unknown cells
    // when a path isnt found, radius is decreases and path is searched again
    public Cell StarSearchExplorerZone(Cell start, Cell goal, int radius, int zoneRadius) {
        int mapLimitMinX = Math.max(0, Math.min(goal.x, foreignminX) - radius * 4), mapLimitMinY = Math.max(0, Math.min(goal.y, foreignminY) - radius * 4);
        int mapLimitMaxX = Math.min(mapWidthPixels, Math.max(goal.x, foreignmaxX) + radius * 4), mapLimitMaxY = Math.min(mapHeightPixels, Math.max(goal.y, foreignmaxY) + radius * 4);
        //System.out.println("\tA* on region ("+mapLimitMinX+","+mapLimitMinY+") and ("+mapLimitMaxX+","+mapLimitMaxY+")");

        Cell[][] origins = new Cell[mapLimitMaxX - mapLimitMinX][mapLimitMaxY - mapLimitMinY];

        // if first cell is blocked on a big radius, find a nearby cell that is not blocked
        //System.out.println("checking if start cell is clear");
        start = findClosestFreeCell(start, radius);
        origins[start.x - mapLimitMinX][start.y - mapLimitMinY] = start;

        ArrayList<Cell> front = new ArrayList();
        for (Cell t : getNeighbors(start)) {
            front.add(t);
            origins[t.x - mapLimitMinX][t.y - mapLimitMinY] = start;
        }

        boolean haventFoundTheEnd = true;
        while (!front.isEmpty()) {
            Cell best = findBestOption(front, goal);
            front.remove(best);

            if (goal.x == best.x && goal.y == best.y) {
                haventFoundTheEnd = false;
                break;
            }

            if (getDistanceBetweenCells(best, goal) < zoneRadius) {
                if (origins[goal.x - mapLimitMinX][goal.y - mapLimitMinY] == null) {
                    origins[goal.x - mapLimitMinX][goal.y - mapLimitMinY] = best;
                }
                haventFoundTheEnd = false;
            }

            for (Cell neighbour : getNeighbors(best)) {
                // if neighbour hasnt been visited yet and is not blocked 
                if (origins[neighbour.x - mapLimitMinX][neighbour.y - mapLimitMinY] == null && !isBlocked(neighbour, radius, mapLimitMinX, mapLimitMaxX, mapLimitMinY, mapLimitMaxY)) {
                    front.add(neighbour);
                    origins[neighbour.x - mapLimitMinX][neighbour.y - mapLimitMinY] = best;
                }
            }
        }

        if (haventFoundTheEnd) {
            if (radius > 1) {
                System.out.println("\tcould find path to zone A*. adjusting radius to " + (radius - 1));
                return StarSearchExplorerZone(start, goal, radius - 1, zoneRadius);
            }
            System.out.println("\tcould find path to zone A*. giving up.");
            return null;
        }

        //System.out.println("about to crash; added goal "+goal+" with origins "+origins[goal.x-mapLimitMinX][goal.y-mapLimitMinY]);
        ArrayList<Cell> bestPath = new ArrayList();
        Cell currentPosition = goal;                                                    //Start at the endpoint.
        bestPath.add(goal);
        while (start.x != currentPosition.x || start.y != currentPosition.y) {          //Until we're back at the start.
            currentPosition = origins[currentPosition.x - mapLimitMinX][currentPosition.y - mapLimitMinY];
            bestPath.add(currentPosition);
            if (gui != null) {
                gui.paintTemporaryPoint(currentPosition.x, currentPosition.y, Color.BLUE);
            }
        }

        Cell temp;
        int index = bestPath.size() - 5;
        if (bestPath.size() >= 6) {
            temp = bestPath.get(index);
        } else {
            index = 0;
            temp = bestPath.get(0);
        }
        prevPath = bestPath;

        while (checkLineForObstacles(start.x, start.y, temp.x, temp.y) && index + 1 < bestPath.size()) {
            System.out.println("\nLine has obstacles! Going for index: " + index);
            temp = bestPath.get(index + 1);
            index++;
        }

        if (gui != null) {
            gui.paintTemporaryPoint(temp.x, temp.y, Color.PINK);
        }

        double dist = getDistanceBetweenCells(bestPath.get(Math.min(1, bestPath.size() - 1)), goal); // between 0 and 6
        temp.counter = (6.0 - dist) * 3.0 + 5;    // 23 and 5

        //System.out.println("\tA* found a path with radius " + radius + " facing " + temp + " from current " + start);
        return temp;
    }

    private ArrayList<Cell> prevPath;

    public Cell getNextCell(Cell curr) {
        System.out.println("\t\tPath size at beginning : " + prevPath.size());
        //run through path
        //find closest path cell to curr
        double minDist = getDistanceBetweenCells(curr, prevPath.get(0)), tDist;
        int minDistIndex = 0;
        for (int i = 1; i < prevPath.size(); i++) {
            if ((tDist = getDistanceBetweenCells(curr, prevPath.get(i))) < minDist) {
                minDistIndex = i;
                minDist = tDist;
            }
        }

        // only process if prevPath has at least 2 cells
        if (prevPath.size() <= 1) {
            System.out.println("returning null");
            return null;
        }

        //remove cells AFTER that one
        while (prevPath.size() > minDistIndex + 1) {
            prevPath.remove(minDistIndex + 1);
        }

        // if cell too close, remove it
        if (getDistanceBetweenCells(curr, prevPath.get(prevPath.size() - 1)) < 2 && prevPath.size() > 1) {
            prevPath.remove(prevPath.size() - 1);
        }

        System.out.println("\t\tpath left : " + prevPath.size());

        if (gui != null) {
            for (Cell c : prevPath) {
                gui.paintTemporaryPoint(c.x, c.y, Color.BLUE);
            }
        }

        Cell temp;
        int index = prevPath.size() - 5;
        if (prevPath.size() >= 6) {
            temp = prevPath.get(index);
        } else {
            index = 0;
            temp = prevPath.get(0);
        }

        while (checkLineForObstacles(curr.x, curr.y, temp.x, temp.y) && index + 1 < prevPath.size()) {
            System.out.println("\nLine has obstacles!");
            temp = prevPath.get(index + 1);
            index++;
        }

        if (gui != null) {
            gui.paintTemporaryPoint(temp.x, temp.y, Color.PINK);
        }

        return temp;
    }

    // gets the top, bottom, left and right neighbours of the cell "start"
    private static Cell[] getNeighbors(Cell start) {
        Cell[] neighbors = new Cell[8];
        neighbors[0] = new Cell(start.x, start.y - 1, start.counter + 1);
        neighbors[1] = new Cell(start.x, start.y + 1, start.counter + 1);
        neighbors[2] = new Cell(start.x + 1, start.y, start.counter + 1);
        neighbors[3] = new Cell(start.x - 1, start.y, start.counter + 1);
        neighbors[4] = new Cell(start.x - 1, start.y - 1, start.counter + 1.7);
        neighbors[5] = new Cell(start.x + 1, start.y + 1, start.counter + 1.7);
        neighbors[6] = new Cell(start.x + 1, start.y - 1, start.counter + 1.7);
        neighbors[7] = new Cell(start.x - 1, start.y + 1, start.counter + 1.7);

        return neighbors;
    }

    // returns the closest cell from "front" to the cell "goal"
    private static Cell findBestOption(ArrayList<Cell> front, Cell goal) {
        Cell bestPosition = null;
        double distanceOfBestPosition = Integer.MAX_VALUE; //Some very high number to start with.
        for (Cell cell : front) {
            if (cell.dist == -1) {
                cell.dist = getDistanceBetweenCells(cell, goal) + cell.counter; //Euclidean distance (Pythagoras Theorem)
            }
            if (cell.dist < distanceOfBestPosition) {
                distanceOfBestPosition = cell.dist;
                bestPosition = cell;
            }
        }

        return bestPosition;
    }

    // Cell class, simple "x,y" coordinate, with a counter field for distance walked
    public static class Cell {

        int x, y;
        double counter;
        double dist;

        public Cell(int a, int b) {
            x = a;
            y = b;
        }

        public Cell(int a, int b, double c) {
            x = a;
            y = b;
            counter = c;
            dist = -1;
        }

        @Override
        public String toString() {
            return "Cell " + x + ":" + y;
        }

    };

    // Line class, between 2 cells
    public static class Line {

        Cell a, b;

        public Line(Cell a2, Cell b2) {
            a = a2;
            b = b2;
        }

        @Override
        public String toString() {
            return "Line between " + a + " and " + b;
        }

    };

    // returns real distance between 2 cells
    public static double getDistanceBetweenCells(Cell a, Cell b) {
        return Math.sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    }

    // returns whether cross "a" has any cell blocked
    public boolean isBlocked(Cell a, int radius) {
        if (a.x < radius || a.y < radius || a.x >= mapWidthPixels - radius || a.y >= mapHeightPixels - radius) {
            return true;
        }

        boolean stat = actualMap[a.x][a.y] == blackCode || actualMap[a.x][a.y] == externalBlackCode;
        for (int i = 1; i <= radius && !stat; i++) {
            stat = stat || actualMap[a.x + i][a.y] == blackCode || actualMap[a.x - i][a.y] == blackCode
                    || actualMap[a.x][a.y + i] == blackCode || actualMap[a.x][a.y - i] == blackCode
                    || actualMap[a.x + i][a.y] == externalBlackCode || actualMap[a.x - i][a.y] == externalBlackCode
                    || actualMap[a.x][a.y + i] == externalBlackCode || actualMap[a.x][a.y - i] == externalBlackCode;
        }
        return stat;
    }

    // returns whether cross "a" has any cell blocked
    public boolean isBlocked(Cell a, int radius, int minX, int maxX, int minY, int maxY) {
        if (a.x < minX + radius || a.y < minY + radius || a.x >= maxX - radius || a.y >= maxY - radius) {
            return true;
        }

        boolean stat = actualMap[a.x][a.y] == blackCode || actualMap[a.x][a.y] == externalBlackCode;
        for (int i = 1; i <= radius && !stat; i++) {
            stat = stat || actualMap[a.x + i][a.y] == blackCode || actualMap[a.x - i][a.y] == blackCode
                    || actualMap[a.x][a.y + i] == blackCode || actualMap[a.x][a.y - i] == blackCode
                    || actualMap[a.x + i][a.y] == externalBlackCode || actualMap[a.x - i][a.y] == externalBlackCode
                    || actualMap[a.x][a.y + i] == externalBlackCode || actualMap[a.x][a.y - i] == externalBlackCode;
        }
        return stat;
    }

    // returns whether cross "a" has all cells unexplored
    public boolean isUnexplored(Cell a, int radius) {
        if (a.x < radius || a.y < radius || a.x >= mapWidthPixels - radius || a.y >= mapHeightPixels - radius) {
            return false;
        }

        boolean stat = actualMap[a.x][a.y] == greyCode;
        for (int i = 1; i <= radius && stat; i++) {
            stat = stat && actualMap[a.x + i][a.y] == greyCode && actualMap[a.x - i][a.y] == greyCode
                    && actualMap[a.x][a.y + i] == greyCode && actualMap[a.x][a.y - i] == greyCode;
        }
        return stat;
    }

    // creates windows and paints it grey
    public static Map showMap(String name, int w, int h, boolean guiB) {
        mapWidthPixels = w;
        mapHeightPixels = h;

        Map m = new Map();

        if (guiB) {
            MapLoaderThread thr = new MapLoaderThread(m, name, w, h);
            thr.start();
        }

        return m;
    }
}

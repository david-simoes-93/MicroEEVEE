/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package Engine;

/**
 *
 * @author bluemoon
 */
import java.awt.*;
import java.util.ArrayList;
import javax.swing.*;

public class MapGUI extends JFrame {

    JPanel[][] panels;
    int width, height, size;
    ArrayList<info> changers = new ArrayList();
    ArrayList<info> permas = new ArrayList();
    
    public MapGUI(String name) {
        super(name);
        setResizable(true);
    }

    public void addComponentsToPane(final Container pane) {
        pane.setLayout(new GridLayout(height, width));
        pane.setPreferredSize(new Dimension(size * width, size * height));

        panels = new JPanel[width][height];

        for (int j = 0; j < height; j++) {
            for (int i = 0; i < width; i++) {
                panels[i][j] = new JPanel();
                panels[i][j].setBackground(Color.GRAY);
                pane.add(panels[i][j]);
            }
        }
    }
    
    public void paintPermanentPoint(int x, int y, Color temp){
        paintPoint(x,y,temp);
        permas.add(new info(x,y,temp));
    }
    
    // removes temporary points
    public void removeChangers(){
        while(!changers.isEmpty()){
            info t = changers.get(changers.size()-1);
            panels[t.x][t.y].setBackground(t.keeper);
            changers.remove(changers.size()-1);
        }
    }

    // paints a point which will vanish when "removeChangers()" is called
    public void paintTemporaryPoint(int x, int y, Color temp){
        changers.add(new info(x,y,panels[x][y].getBackground()));
        panels[x][y].setBackground(temp);
    }
    
    // paints a point
    public void paintPoint(int x, int y, Color a) {
        for(info i : permas){
            if(i.x==x && i.y==y){
                return;
            }
        }
        //System.out.println("painting " + x + "." + y);
        for(info i : changers){
            if(i.x==x && i.y==y){
                i.keeper=a;
                return;
            }
        }
        panels[x][y].setBackground(a);
        //this.repaint();
    }

    public static MapGUI createAndShowGUI(String name, int w, int h, int s) {
        System.setProperty("java.util.Arrays.useLegacyMergeSort", "true");
        
        System.out.print("Loading..... ");
        MapGUI frame = new MapGUI(name);
        frame.width = w;
        frame.height = h;
        frame.size = s;

        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.addComponentsToPane(frame.getContentPane());

        //Display the window.
        frame.pack();
        frame.setVisible(true);

        System.out.println("done!");
        return frame;
    }
    
    private static class info{
        int x, y;
        Color keeper;
        
        public info(int x, int y, Color keeper){
            this.x=x;
            this.y=y;
            this.keeper=keeper;
        }
    };
}

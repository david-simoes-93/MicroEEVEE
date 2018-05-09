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
public class MapLoaderThread extends Thread{
    Map m;
    String name;
    int w, h;
    
    MapLoaderThread(Map m, String name, int w, int h) {
        this.m=m;
        this.name=name;
        this.w=w;
        this.h=h;
    }
    
    @Override
    public void run(){
        m.gui = MapGUI.createAndShowGUI(name, w, h, 4);
    }
}

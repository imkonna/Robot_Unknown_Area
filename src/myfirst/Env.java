package myfirst;

import javax.vecmath.*;
import simbad.sim.*;

public class Env extends EnvironmentDescription {
    public Env(){
        setupEnvironment();
    }

    public void setupEnvironment() {
        // Αφαιρούμε lines: δεν είναι μέρος της εκφώνησης
        addObstacles();
        addLights();
    }

    private void addObstacles() {
        // Μπορείς να κρατήσεις ό,τι obstacles θες για δοκιμές.
        // Ο ελεγκτής πρέπει να δουλεύει χωρίς να τα "ξέρει" από πριν (και δεν τα ξέρει).

        add(new Arch(new Vector3d(6,0,4),this));
        add(new Wall(new Vector3d(-0.5,0,4),10,1.5f,this));
        add(new Wall(new Vector3d(12.5,0,4),10,1.5f,this));
        add(new Wall(new Vector3d(6,0,8),6,1.5f,this));

        Wall w1 = new Wall(new Vector3d(3,0,6),4,1.5f,this);
        w1.rotate90(1);
        add(w1);

        Wall w2 = new Wall(new Vector3d(9,0,6),4,1.5f,this);
        w2.rotate90(1);
        add(w2);

        add(new Box(new Vector3d(2,0,2),new Vector3f(3f,1.5f,4f),this));
        add(new Box(new Vector3d(-6,0,-6),new Vector3f(2f,1.5f,2f),this));
        add(new Box(new Vector3d(-4,0,0.5),new Vector3f(2f,1.5f,7f),this));
    }

    private void addLights() {
        // Εκφώνηση: 1 λάμπα σε ύψος 2m.
        light1IsOn = true;
        light2IsOn = false;

        light1Color = new Color3f(1,0,0);
        light1Position = new Vector3d(6, 2, 6); // y = 2m
    }
}

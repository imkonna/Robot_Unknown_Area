package myfirst;

import simbad.gui.Simbad;
import simbad.sim.EnvironmentDescription;
import subsumption.BehaviorBasedAgent;

import javax.vecmath.Vector3d;

public class Myfirst {

    public static void main(String[] args) {

        EnvironmentDescription env = new Env();

        BehaviorBasedAgent robot =
                new MyRobot(new Vector3d(-9,0,-6), "Robot", 12, false); // ΧΩΡΙΣ line sensors

        env.add(robot);
        new Simbad(env, false);
    }
}

package subsumption;

import simbad.sim.Agent;
import simbad.sim.LineSensor;
import simbad.sim.RangeSensorBelt;
import simbad.sim.RobotFactory;

import javax.vecmath.Vector3d;

public class BehaviorBasedAgent extends Agent {

    private final Sensors sensors;

    private Behavior[] behaviors;

    public BehaviorBasedAgent(Vector3d position, String name, int sensorCount, boolean addLineSensors) {
        super(position, name);

        // Στο Simbad 1.7 παίρνουμε sonars μέσω RobotFactory
        RangeSensorBelt sonars = RobotFactory.addSonarBeltSensor(this, sensorCount);

        // Line sensors (αν τα θες). Εσύ τα έχεις false, άρα θα μείνει null.
        LineSensor line = null;
        if (addLineSensors) {
            line = RobotFactory.addLineSensor(this, sensorCount);
        }

        this.sensors = new Sensors(sonars, line);
    }

    public Sensors getSensors() {
        return sensors;
    }

    public void initBehaviors(Behavior[] behaviors, boolean[][] subsumes) {
        // για την ώρα αρκεί να κρατήσουμε μόνο τη λίστα behaviors
        // (το subsumes matrix δεν το χρησιμοποιούμε στο minimal scheduler)
        this.behaviors = behaviors;
    }

    @Override
    public void performBehavior() {
        if (behaviors == null || behaviors.length == 0) return;

        // priority order: behaviors[0] highest
        for (Behavior b : behaviors) {
            if (b.isActive()) {
                Velocities v = b.act();
                setTranslationalVelocity(v.translationalVelocity);
                setRotationalVelocity(v.rotationalVelocity);
                return;
            }
        }
    }
}

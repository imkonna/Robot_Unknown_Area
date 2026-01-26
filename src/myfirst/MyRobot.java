package myfirst;

import simbad.sim.*;
import subsumption.*;
import javax.vecmath.Vector3d;

public class MyRobot extends BehaviorBasedAgent {

    private final LightSensor lightL;
    private final LightSensor lightR;

    public MyRobot(Vector3d position, String name, int sensorCount, boolean addLineSensors) {
        super(position, name, sensorCount, addLineSensors);

        lightL = RobotFactory.addLightSensorLeft(this);
        lightR = RobotFactory.addLightSensorRight(this);

        Sensors sensors = this.getSensors();

        Behavior[] behaviors = {
                new ReachGoal(sensors, this),      // 0
                new DockInBox(sensors, this),      // 1
                new CorridorCenter(sensors, this), // 2
                new CircumNavigate(sensors, this), // 3
                new FollowLight(sensors, this)     // 4
        };

        boolean[][] subsumes = {
                { false, true,  true,  true,  true  },  // ReachGoal subsumes all
                { false, false, true,  true,  true  },  // DockInBox subsumes below
                { false, false, false, true,  true  },  // CorridorCenter subsumes below
                { false, false, false, false, true  },  // CircumNavigate subsumes FollowLight
                { false, false, false, false, false }
        };

        this.initBehaviors(behaviors, subsumes);
    }

    public LightSensor getLightLeftSensor()  { return lightL; }
    public LightSensor getLightRightSensor() { return lightR; }
}


/** Default: steering προς το φως */
class FollowLight extends Behavior {

    private final LightSensor lightR;
    private final LightSensor lightL;

    public FollowLight(Sensors sensors, MyRobot robot) {
        super(sensors);
        lightL = robot.getLightLeftSensor();
        lightR = robot.getLightRightSensor();
    }

    public Velocities act() {
        double luxL = lightL.getLux();
        double luxR = lightR.getLux();
        double diff = luxL - luxR;

        double v = (Math.abs(diff) < 0.03) ? 0.55 : 0.40;

        double rot = -0.9 * diff;
        if (rot > 0.35) rot = 0.35;
        if (rot < -0.35) rot = -0.35;
        if (Math.abs(diff) <= 0.01) rot = 0;

        return new Velocities(v, rot);
    }

    public boolean isActive() { return true; }
}


/** ✅ STOP μόνο όταν το Dock δηλώσει “docked” */
class ReachGoal extends Behavior {

    private final DockInBox dock;

    public ReachGoal(Sensors sensors, MyRobot robot) {
        super(sensors);
        this.dock = DockInBox.getInstance();
    }

    public Velocities act() {
        return new Velocities(0.0, 0.0);
    }

    public boolean isActive() {
        return dock != null && dock.isDocked();
    }
}


/**
 * ✅ DockInBox (STATE MACHINE)
 * Φάση 1: ALIGN στην πόρτα (με side sonars + light diff)
 * Φάση 2: PUSH μέσα
 * Φάση 3: STOP όταν είμαστε μέσα (lux ή geometry)
 */
class DockInBox extends Behavior {

    // Από το δικό σου screenshot: maxLux ~0.63 όταν είναι “στην πόρτα”
    private static final double ENTER_LUX = 0.55;     // ξεκινάμε docking νωρίτερα
    private static final double INSIDE_LUX = 0.62;    // θεωρούμε ότι είμαστε “μέσα/στόχος”
    private static final double EXIT_LUX = 0.50;      // για να μη μπαίνει/βγαίνει συνέχεια

    // Sonar gates για πόρτα/διάδρομο
    private static final double WALL_NEAR = 0.95;     // βλέπω τοίχο κοντά δεξιά/αριστερά => είμαι στο στενό
    private static final double FRONT_SAFE = 0.40;    // αν μπροστά είναι πιο κοντά, κόψε

    // Align tuning
    private static final double ALIGN_V = 0.18;
    private static final double ALIGN_ROT_MAX = 0.22;

    // Push tuning
    private static final int PUSH_TICKS = 80;
    private static final double PUSH_V = 0.26;

    // Inside detection by geometry (αν το lux δεν “γράφει” σωστά)
    private static final double INSIDE_SIDE_NEAR = 0.85;

    private enum Mode { IDLE, ALIGN, PUSH, STOPPED }

    private Mode mode = Mode.IDLE;
    private int pushLeft = 0;
    private boolean docked = false;

    private final LightSensor lightL;
    private final LightSensor lightR;

    private static DockInBox INSTANCE;

    public static DockInBox getInstance() { return INSTANCE; }

    public DockInBox(Sensors sensors, MyRobot robot) {
        super(sensors);
        this.lightL = robot.getLightLeftSensor();
        this.lightR = robot.getLightRightSensor();
        INSTANCE = this;
    }

    public boolean isDocked() { return docked; }

    public Velocities act() {
        if (mode == Mode.STOPPED) {
            docked = true;
            return new Velocities(0, 0);
        }

        RangeSensorBelt s = getSensors().getSonars();
        double luxL = lightL.getLux();
        double luxR = lightR.getLux();
        double maxLux = Math.max(luxL, luxR);

        double left = min(s, 2, 4);
        double right = min(s, 8, 10);
        double front = Math.min(Math.min(s.getMeasurement(0), s.getMeasurement(1)), s.getMeasurement(11));

        // ✅ αν είμαστε “μέσα”, σταμάτα
        if (isInside(maxLux, left, right)) {
            mode = Mode.STOPPED;
            docked = true;
            return new Velocities(0, 0);
        }

        // προστασία: αν είναι πολύ κοντά μπροστά, σταμάτα/μην σπρώχνεις
        if (front < FRONT_SAFE) {
            // μικρή στροφή για να ξεκολλήσει
            return new Velocities(0.0, 0.20);
        }

        if (mode == Mode.ALIGN) {
            // στόχος: κεντράρισμα στην πόρτα
            // 1) Χρησιμοποιούμε side sonar error (right-left)
            double err = (right - left); // αν right > left, είμαστε πιο κοντά αριστερά -> στρίψε λίγο δεξιά κλπ
            double rotSonar = 0.9 * err;

            // 2) Χρησιμοποιούμε light diff για “τελικό” κλείδωμα
            double diff = (luxL - luxR);
            double rotLight = -0.4 * diff;

            double rot = rotSonar + rotLight;
            if (rot > ALIGN_ROT_MAX) rot = ALIGN_ROT_MAX;
            if (rot < -ALIGN_ROT_MAX) rot = -ALIGN_ROT_MAX;

            // όταν έχουμε σχετικά “ίσια” (μικρό error), πάμε σε PUSH
            if (Math.abs(err) < 0.10 && Math.abs(diff) < 0.20) {
                mode = Mode.PUSH;
                pushLeft = PUSH_TICKS;
                return new Velocities(PUSH_V, 0.0);
            }

            return new Velocities(ALIGN_V, rot);
        }

        if (mode == Mode.PUSH) {
            pushLeft--;

            // Αν στη διάρκεια του push χάσουμε το φως πολύ (δηλ. βγήκαμε), γύρνα σε ALIGN
            if (maxLux < EXIT_LUX) {
                mode = Mode.ALIGN;
                return new Velocities(0.0, 0.25);
            }

            if (pushLeft <= 0) {
                // τελείωσε το push, αν δεν είμαστε μέσα ακόμη -> ξανά ALIGN (μην φύγει να κάνει γύρες!)
                mode = Mode.ALIGN;
                return new Velocities(0.0, 0.20);
            }

            return new Velocities(PUSH_V, 0.0);
        }

        // default safety
        return new Velocities(0.0, 0.0);
    }

    public boolean isActive() {
        if (mode == Mode.STOPPED) return false;

        RangeSensorBelt s = getSensors().getSonars();
        double luxL = lightL.getLux();
        double luxR = lightR.getLux();
        double maxLux = Math.max(luxL, luxR);

        double left = min(s, 2, 4);
        double right = min(s, 8, 10);

        boolean inCorridor = (left < WALL_NEAR) && (right < WALL_NEAR);

        // Ενεργοποίηση docking: κοντά στο φως ΚΑΙ σε “στενό/είσοδο”
        if (mode == Mode.IDLE) {
            if (maxLux >= ENTER_LUX && inCorridor) {
                mode = Mode.ALIGN;
                return true;
            }
            return false;
        }

        // Αν είμαστε ήδη σε docking mode, παραμένει active
        return (mode == Mode.ALIGN || mode == Mode.PUSH);
    }

    private boolean isInside(double maxLux, double left, double right) {
        // 1) με lux
        if (maxLux >= INSIDE_LUX) return true;

        // 2) με geometry: και οι δύο πλευρές “κοντά” => είμαστε μέσα στο πλαίσιο/κουτί
        return (left < INSIDE_SIDE_NEAR) && (right < INSIDE_SIDE_NEAR);
    }

    private double min(RangeSensorBelt s, int a, int b) {
        double m = s.getMeasurement(a);
        for (int i = a + 1; i <= b; i++) {
            if (s.getMeasurement(i) < m) m = s.getMeasurement(i);
        }
        return m;
    }
}


/**
 * CorridorCenter:
 * Κεντράρει στο στενό αλλά ΜΟΝΟ πριν το docking.
 */
class CorridorCenter extends Behavior {

    private static final double WALL_NEAR = 0.85;
    private static final double FRONT_CLEAR = 0.60;

    // Μόλις πλησιάσουμε αρκετά (dock zone), το αφήνουμε να αναλάβει το DockInBox
    private static final double CORRIDOR_GATE_LUX = 0.55;

    private final LightSensor lightL;
    private final LightSensor lightR;

    public CorridorCenter(Sensors sensors, MyRobot robot) {
        super(sensors);
        this.lightL = robot.getLightLeftSensor();
        this.lightR = robot.getLightRightSensor();
    }

    public Velocities act() {
        RangeSensorBelt s = getSensors().getSonars();

        double left = min(s, 2, 4);
        double right = min(s, 8, 10);
        double front = Math.min(min(s, 0, 1), s.getMeasurement(11));

        if (front < FRONT_CLEAR) {
            return new Velocities(0.0, 0.0);
        }

        double error = (right - left);
        double rot = 0.8 * error;
        if (rot > 0.30) rot = 0.30;
        if (rot < -0.30) rot = -0.30;

        return new Velocities(0.35, rot);
    }

    public boolean isActive() {
        double maxLux = Math.max(lightL.getLux(), lightR.getLux());
        if (maxLux >= CORRIDOR_GATE_LUX) return false;

        RangeSensorBelt s = getSensors().getSonars();
        double left = min(s, 2, 4);
        double right = min(s, 8, 10);

        return (left < WALL_NEAR) && (right < WALL_NEAR);
    }

    private double min(RangeSensorBelt s, int a, int b) {
        double m = s.getMeasurement(a);
        for (int i = a + 1; i <= b; i++) {
            if (s.getMeasurement(i) < m) m = s.getMeasurement(i);
        }
        return m;
    }
}


/** Αποφυγή εμποδίου (ήπια) */
class CircumNavigate extends Behavior {

    private static final double SAFETY = 0.45;
    private static final double GOAL_GATE_LUX = 0.60; // μην κόβεται πολύ νωρίς κοντά στο φως

    private boolean active = false;
    private boolean clockwise = true;

    private double entryLuxAvg = 0;

    private final LightSensor lightR;
    private final LightSensor lightL;

    public CircumNavigate(Sensors sensors, MyRobot robot) {
        super(sensors);
        lightL = robot.getLightLeftSensor();
        lightR = robot.getLightRightSensor();
    }

    public Velocities act() {
        RangeSensorBelt sonars = getSensors().getSonars();

        float tv = 0.25f;

        if (sonars.getFrontQuadrantHits() > 0
                && sonars.getMeasurement(minFrontIndex(sonars)) < SAFETY) {
            float rv = clockwise ? 0.35f : -0.35f;
            return new Velocities(0.0, rv);
        }

        float rv = 0.25f;

        if (clockwise) {
            if (sonars.getRightQuadrantHits() > 0) {
                if (sonars.getMeasurement(minRightIndex(sonars)) < SAFETY) {
                    return new Velocities(tv, +rv);
                }
                return new Velocities(tv, 0);
            } else {
                return new Velocities(tv, -rv);
            }
        } else {
            if (sonars.getLeftQuadrantHits() > 0) {
                if (sonars.getMeasurement(minLeftIndex(sonars)) < SAFETY) {
                    return new Velocities(tv, -rv);
                }
                return new Velocities(tv, 0);
            } else {
                return new Velocities(tv, +rv);
            }
        }
    }

    public boolean isActive() {
        RangeSensorBelt sonars = getSensors().getSonars();

        double luxL = lightL.getLux();
        double luxR = lightR.getLux();
        double luxAvg = 0.5 * (luxL + luxR);

        if (Math.max(luxL, luxR) >= GOAL_GATE_LUX) {
            active = false;
            return false;
        }

        if (!active) {
            if (sonars.getFrontQuadrantHits() > 0
                    && sonars.getMeasurement(minFrontIndex(sonars)) < SAFETY) {
                active = true;
                entryLuxAvg = luxAvg;
                clockwise = (luxL < luxR);
                return true;
            }
            return false;
        }

        if (sonars.getFrontQuadrantHits() == 0 && luxAvg > entryLuxAvg + 0.03) {
            active = false;
            return false;
        }

        return true;
    }

    private int minFrontIndex(RangeSensorBelt sonars) {
        int min = 0;
        for (int i = 0; i < 2; i++)
            if (sonars.getMeasurement(i) < sonars.getMeasurement(min)) min = i;
        if (sonars.getMeasurement(11) < sonars.getMeasurement(min)) min = 11;
        return min;
    }

    private int minLeftIndex(RangeSensorBelt sonars) {
        int min = 2;
        for (int i = 3; i < 5; i++)
            if (sonars.getMeasurement(i) < sonars.getMeasurement(min)) min = i;
        return min;
    }

    private int minRightIndex(RangeSensorBelt sonars) {
        int min = 8;
        for (int i = 9; i < 11; i++)
            if (sonars.getMeasurement(i) < sonars.getMeasurement(min)) min = i;
        return min;
    }
}

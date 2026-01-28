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

        Behavior[] behaviors =  {
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
        double avg = 0.5 * (luxL + luxR);

        double v = (Math.abs(diff) < 0.03) ? 0.55 : 0.40;
        double rot = -0.9 * diff;

        if (rot > 0.35) rot = 0.35;
        if (rot < -0.35) rot = -0.35;
        if (Math.abs(diff) <= 0.01) rot = 0;

        // 💡 Boost forward when strong light seen
        if (avg >= 0.66 && Math.abs(diff) < 0.02) {
            System.out.println("[FollowLight] Strong goal → driving straight in");
            return new Velocities(0.65, 0);
        }

        return new Velocities(v, rot);
    }


    public boolean isActive() {
        return true;
    }
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

    // Light thresholds
    private static final double ENTER_LUX = 0.68;
    private static final double INSIDE_LUX = 0.63;

    // Sonar thresholds
    private static final double WALL_NEAR = 0.4;

    // Alignment tuning
    private static final double ALIGN_V = 0.18;
    private static final double ALIGN_ROT_MAX = 0.25;

    // Push tuning
    private static final int PUSH_TICKS = 80;
    private static final double PUSH_V = 0.26;

    // Geometry inside check
    private static final double INSIDE_SIDE_NEAR = 0.65;

    private enum Mode { IDLE, ALIGN, PUSH, STOPPED }

    private Mode mode = Mode.IDLE;

    private int pushLeft = 0;
    private boolean docked = false;
    private int stableTicks = 0;


    private final LightSensor lightL;
    private final LightSensor lightR;

    private static DockInBox INSTANCE;

    public static DockInBox getInstance() {
        return INSTANCE;
    }

    public DockInBox(Sensors sensors, MyRobot robot) {
        super(sensors);

        this.lightL = robot.getLightLeftSensor();
        this.lightR = robot.getLightRightSensor();


        INSTANCE = this;
    }

    public boolean isDocked() {
        return docked;
    }

    // =====================================================
    // MAIN BEHAVIOR
    // =====================================================

    public Velocities act() {

        RangeSensorBelt s = getSensors().getSonars();

        double luxL = lightL.getLux();
        double luxR = lightR.getLux();
        double maxLux = Math.max(luxL, luxR);

        double left  = min(s, 2, 4);
        double right = min(s, 8, 10);
        double front = Math.min(Math.min(s.getMeasurement(0), s.getMeasurement(1)), s.getMeasurement(11));

        boolean inCorridor = (left < WALL_NEAR) || (right < WALL_NEAR);


        // 🔍 Log the sensors to verify state
        System.out.println("[Dock] IDLE mode check: maxLux=" + maxLux + " inCorridor=" + inCorridor + " front=" + front + " mode=" + mode);

        // Try to enter ALIGN mode when in corridor and near the light
        if (mode == Mode.IDLE && maxLux >= ENTER_LUX && (inCorridor || front > 0.5)) {
            mode = Mode.ALIGN;
            System.out.println("[Dock] Entering ALIGN mode. Lux=" + maxLux);
        }

        // ================================
        // ALIGN MODE
        // ================================
        if (mode == Mode.ALIGN) {
            double error = right - left;
            double rot = 1.8 * error;  // slightly stronger correction

            if (rot > ALIGN_ROT_MAX) rot = ALIGN_ROT_MAX;
            if (rot < -ALIGN_ROT_MAX) rot = -ALIGN_ROT_MAX;

            boolean wellAligned = Math.abs(error) < 0.025;   // tighter threshold
            boolean nearWall = front < 0.45;


            // Count stable alignment ticks
            if (wellAligned) {
                stableTicks++;
            } else {
                stableTicks = 0;
            }

            System.out.println("[Dock] ALIGN left=" + left +
                    " right=" + right +
                    " error=" + error +
                    " stableTicks=" + stableTicks);

            // Only push after multiple stable frames
            if (stableTicks >= 4 && nearWall) {
                mode = Mode.PUSH;
                pushLeft = PUSH_TICKS + 10;
                System.out.println("[Dock] ALIGN stable → PUSH");
                return new Velocities(PUSH_V, 0);
            }

            return new Velocities(ALIGN_V, rot);
        }


        // ================================
        // PUSH MODE
        // ================================
        if (mode == Mode.PUSH) {
            double error = right - left;
            double rot = 0.4 * error; // slight correction while pushing
            if (rot > 0.2) rot = 0.2;
            if (rot < -0.2) rot = -0.2;

            if (front < 0.1) {
                System.out.println("[Dock] 🚨 EMERGENCY STOP: front=" + front);
                mode = Mode.STOPPED;
                docked = true;
                return new Velocities(0, 0);
            }

            if (pushLeft-- > 0) {
                return new Velocities(PUSH_V, rot);  // <- not just straight!
            } else {
                mode = Mode.STOPPED;
                docked = true;
                return new Velocities(0, 0);
            }
        }


        // ================================
        // STOP IF FULLY INSIDE
        // ================================
        if (isInside(maxLux, left, right)) {
            mode = Mode.STOPPED;
            docked = true;
            System.out.println("[Dock] INSIDE → STOP");
            return new Velocities(0, 0);
        }

        // Otherwise: do nothing
        return new Velocities(0, 0);
    }


    // =====================================================
    // ACTIVATION LOGIC
    // =====================================================

    public boolean isActive() {
        if (mode == Mode.STOPPED) return false;

        RangeSensorBelt s = getSensors().getSonars();
        double luxL = lightL.getLux();
        double luxR = lightR.getLux();
        double maxLux = Math.max(luxL, luxR);

        double left  = min(s, 2, 4);
        double right = min(s, 8, 10);

        boolean inCorridor = (left < WALL_NEAR) && (right < WALL_NEAR);

        // Start docking
        if (mode == Mode.IDLE && maxLux >= ENTER_LUX && inCorridor) {
            mode = Mode.ALIGN;
            System.out.println("[Dock] IDLE → ALIGN (start docking)");
        }

        return (mode == Mode.ALIGN || mode == Mode.PUSH);
    }


    // =====================================================
    // HELPERS
    // =====================================================

    private boolean isInside(double maxLux, double left, double right) {

        if (maxLux >= INSIDE_LUX) return true;

        return (left < INSIDE_SIDE_NEAR) && (right < INSIDE_SIDE_NEAR);
    }

    private double min(RangeSensorBelt s, int a, int b) {

        double m = s.getMeasurement(a);

        for (int i = a + 1; i <= b; i++) {
            if (s.getMeasurement(i) < m)
                m = s.getMeasurement(i);
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
        double luxL = lightL.getLux();
        double luxR = lightR.getLux();
        double maxLux = Math.max(luxL, luxR);

        RangeSensorBelt s = getSensors().getSonars();
        double front = Math.min(min(s, 0, 1), s.getMeasurement(11));
        double left = min(s, 2, 4);
        double right = min(s, 8, 10);

        boolean inCorridor = (left < WALL_NEAR) && (right < WALL_NEAR);
        boolean frontBlocked = front < FRONT_CLEAR;

        // ✅ Enhancement: Stay active even near goal if obstacle still in front
        return inCorridor && (maxLux < CORRIDOR_GATE_LUX || frontBlocked);
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
    private boolean committedToGoal = false;



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

        // 🔒 Lock in goal if bright light once seen
        if (!committedToGoal && luxAvg >= 0.66) {
            committedToGoal = true;
            System.out.println("[iBug] COMMITTING TO GOAL → disabling avoidance");
            return false;
        }

        // Already committed? Don't block anymore
        if (committedToGoal) return false;

        if (!active) {
            if (sonars.getFrontQuadrantHits() > 0 &&
                    sonars.getMeasurement(minFrontIndex(sonars)) < SAFETY) {

                active = true;
                entryLuxAvg = luxAvg;
                clockwise = (luxL < luxR);
                return true;
            }
            return false;
        }

        // If already active, only stay if no improvement
        boolean frontClear = sonars.getFrontQuadrantHits() == 0;
        boolean lightImproved = luxAvg > entryLuxAvg + 0.02;
        boolean wellAligned = Math.abs(luxL - luxR) < 0.02;

        // Exit condition
        if (frontClear && lightImproved && wellAligned) {
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

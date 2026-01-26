package subsumption;

import simbad.sim.LineSensor;
import simbad.sim.RangeSensorBelt;

public class Sensors {
    private final RangeSensorBelt sonars;
    private final LineSensor lineSensors;

    public Sensors(RangeSensorBelt sonars, LineSensor lineSensors) {
        this.sonars = sonars;
        this.lineSensors = lineSensors;
    }

    public RangeSensorBelt getSonars() {
        return sonars;
    }

    public LineSensor getLineSensors() {
        return lineSensors;
    }
}

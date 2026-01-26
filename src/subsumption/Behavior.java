package subsumption;

public abstract class Behavior {
    private final Sensors sensors;

    protected Behavior(Sensors sensors) {
        this.sensors = sensors;
    }

    protected Sensors getSensors() {
        return sensors;
    }

    public abstract Velocities act();
    public abstract boolean isActive();
}

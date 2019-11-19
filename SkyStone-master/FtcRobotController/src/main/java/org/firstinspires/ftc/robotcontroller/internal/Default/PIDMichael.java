package org.firstinspires.ftc.robotcontroller.internal.Default;

public class PIDMichael {
    public double p, i, d, error, time, dt, de, integral = 0, differential = 0, target; //dt = delta time, de = delta error
    public PIDMichael(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }
    public void updatePID(double current, long time) {
        dt = time - this.time;
        this.time = time;
        de = (target - current) - this.error;
        this.error = target - current;
    }
    public void initPID(double target, long time) {
        this.target = target;
        this.time = time;
        this.error = target;
    }
    public void integrate() {
        if (p * error + i * integral + d * differential < 1) integral += (error + (error - de)) * dt / 2; //trapezoidal sum & windup check
    }
    public void differentiate() {
        differential = de / dt;
    }
    public double actuator(double current, long time) {
        updatePID(current, time);
        integrate();
        differentiate();
        return p * error + i * integral + d * differential;
    }
}

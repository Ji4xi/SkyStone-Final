package org.firstinspires.ftc.robotcontroller.internal.Default;

import android.icu.lang.UScript;

public class PIDMichael {
    public double p, i, d, error, time, dt, de, integral = 0, differential = 0, target, timeSnip; //dt = delta time, de = delta error
    public double actuator(double current, long time) {
        updatePID(current, time);
        integrate();
        differentiate();
        return p * error / target + i * integral + d * differential;
    }



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
        if (p * error / target + i * integral + d * differential < 0.4) integral += (error + (error + de))* dt / 2; //trapezoidal sum & windup check
    }
    public void differentiate() {
        differential = de / dt;
    }
    public double getPContrb() {
        return p * differential;
    }
    public double getIContrb() {
        return i * integral;
    }
    public double getDContrb() {
        return d * error / target;
    }
    //real
    public double actuator(double current) {
        updatePID(current);
        differentiate();
        integrate();
        return p * (target - error) / target + i * integral + d * differential;
    }
    //real
    public void updatePID(double current) {
        timeSnip = System.nanoTime();
        dt = timeSnip - this.time;
        time = timeSnip;
        de = current - error;
        error = current;
    }
    //real
    public void initPID(double target) {
        this.target = target;
        this.error = target;
    }
}


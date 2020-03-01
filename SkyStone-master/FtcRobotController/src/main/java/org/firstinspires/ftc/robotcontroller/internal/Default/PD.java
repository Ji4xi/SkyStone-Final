package org.firstinspires.ftc.robotcontroller.internal.Default;

import android.icu.lang.UScript;

public class PD {
    public double p, i, d, error, time, dt, de, integral = 0, differential = 0, target, timeSnip, threshold = 0; //dt = delta time, de = delta error

    public PD(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public void initPD(double target) {
        this.target = Math.abs(target);
        this.time = System.nanoTime();
        this.error = target;
    }


    public void differentiate() {
        differential = de / dt;
    }

    public double getPContrb() {
        return p * error;
    }
    public double getIContrb() {
        return i * integral;
    }
    public double getDContrb() {
        return d * differential;
    }
    //real
    public double actuator(double current) {
        updatePD(current);
        differentiate();
        if (outputPower() > 1) return 1;
        else return outputPower();
    }

    public double outputPower() {
        return p * error + d * differential + i * integral;
    }
    public void updatePD(double current) {
        current = Math.abs(current);
        de = (target - current) - error;
        error = target - current;



        timeSnip = System.nanoTime();
        dt = (timeSnip - time) * Math.pow(10, -9);
        time = timeSnip;

        integral += error * dt;
        if (integral > 0.1) integral = 0.1;
    }
}


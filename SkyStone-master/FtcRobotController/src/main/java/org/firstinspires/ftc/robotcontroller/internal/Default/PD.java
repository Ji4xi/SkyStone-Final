package org.firstinspires.ftc.robotcontroller.internal.Default;

import android.icu.lang.UScript;

public class PD {
    public double p, i, d, error, time, dt, de, integral = 0, differential = 0, target, timeSnip, threshold = 0.37; //dt = delta time, de = delta error

    public PD(double p, double d) {
        this.p = p;
        this.i = 1;
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
        return p * error / target;
    }
    public double getDContrb() {
        return d * differential;
    }
    //real
    public double actuator(double current) {
        updatePD(current);
        differentiate();
        if (outputPower() < threshold) return threshold;
        else if (outputPower() > 1) return 1;
        else return outputPower();
    }

    public double outputPower() {
        return p * error / target + d * differential;
    }
    public void updatePD(double current) {
        current = Math.abs(current);
        de = current - error;
        error = target - current;

        timeSnip = System.nanoTime();
        dt = timeSnip - time;
        time = timeSnip;
    }
}


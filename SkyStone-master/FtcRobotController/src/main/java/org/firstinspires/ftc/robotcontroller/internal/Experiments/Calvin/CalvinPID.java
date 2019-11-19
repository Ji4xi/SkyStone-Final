package org.firstinspires.ftc.robotcontroller.internal.Experiments.Calvin;

import com.qualcomm.robotcore.util.Range;

public class CalvinPID {

    double target;

    double p = 0;
    double i = 0;
    double d = 0;

    double minOutput = -1;
    double maxOutput = 1;

    double errorSum = 0;
    double maxIntegral = maxOutput - minOutput;

    double error;
    double lastValue;

    boolean firstRun = true;

    double min_pwr; //minimum power needed to move

    public CalvinPID() {
    }

    public CalvinPID(double p, double i, double d) {
        this.p = Math.abs(p);
        this.i = Math.abs(i);
        this.d = Math.abs(d);
    }

    public CalvinPID(double target) {
        this.target = target;
    }

    public CalvinPID(double p, double i, double d, double target) {
        this.p = Math.abs(p);
        this.i = Math.abs(i);
        this.d = Math.abs(d);
        this.target = target;
    }

    public void setP(double p) {
        this.p = Math.abs(p);
    }

    public void setI(double i) {
        if(this.i != 0){
            errorSum = errorSum * this.i / i;
        }
        this.i = Math.abs(i);
    }

    public void setD(double d) {
        this.d = Math.abs(d);
    }

    public void setPID(double p, double i, double d) {
        this.p = Math.abs(p);
        setI(i);
        this.d = Math.abs(d);
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public void setMaxIntegral(double maximum) {
        this.maxIntegral = maximum;
    }

    public void setOutputLimits(double minimum, double maximum){
        if (maximum < minimum) return;
        maxOutput = maximum;
        minOutput = minimum;

        if(maxIntegral == 0 || maxIntegral > (maximum - minimum) ){
            setMaxIntegral(maximum - minimum);
        }
    }

    public double getP() {
        return p;
    }

    public double getI() {
        return i;
    }

    public double getD() {
        return d;
    }

    public double getTarget() {
        return target;
    }

    public double getError() {
        return error;
    }

    public double getOutput(double currentValue) {

        double proportional;
        double integral;
        double derivative;

        error = target - currentValue;

        proportional = p * error;

        integral = Range.clip(i * errorSum, -maxIntegral, maxIntegral);

        derivative  = firstRun ? 0 : -d * (currentValue - lastValue);
        firstRun  = false;

        if ((Math.abs(errorSum + error) * i < maxOutput) && (Math.abs(errorSum + error) * i > minOutput)) {
            errorSum += error;
        }

        lastValue = currentValue;

        return Range.clip(proportional + integral + derivative, minOutput, maxOutput);

    }

    public void reset() {
        firstRun = true;
        errorSum = 0;
    }

}

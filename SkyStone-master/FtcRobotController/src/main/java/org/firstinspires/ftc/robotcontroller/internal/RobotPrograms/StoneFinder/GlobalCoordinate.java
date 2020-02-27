package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class GlobalCoordinate implements Runnable {
    private int sleepTime;
    private double globalX;
    private double globalY;
    private boolean isRunning;
    private DcMotor encoderLeft, encoderRight, encoderLeftBack, encoderRightBack;
    private double deltas1, deltas2, deltax1, deltax2, deltay1, deltay2, changeHorizontal, changeVertical, theta;
    private double deltas3, deltas4, deltax3, deltax4, deltay3, deltay4, changeHorizontalBack, changeVerticalBack;
    private double lastEncoderCountLeft, lastEncoderCountRight, currentEncoderCountLeft, currentEncoderCountRight;
    private double lastEncoderCountLeftBack, lastEncoderCountRightBack, currentEncoderCountLeftBack, currentEncoderCountRightBack;
    private double xMultiplier, yMultiplier;
    private BNO055IMU imu;
    /**
     * Constructor for GlobalCoordinate
     * @param encoderLeft
     * @param encoderRight
     * @param imu
     */
    public GlobalCoordinate(DcMotor encoderLeft, DcMotor encoderRight, DcMotor encoderLeftBack, DcMotor encoderRightBack, BNO055IMU imu) {
        globalX = 0;
        globalY = 0;
        isRunning = true;
        lastEncoderCountLeft = 0;
        lastEncoderCountRight = 0;
        lastEncoderCountLeftBack = 0;
        lastEncoderCountRightBack = 0;
        this.encoderLeft = encoderLeft;
        this.encoderRight = encoderRight;
        this.encoderLeftBack = encoderLeftBack;
        this.encoderRightBack = encoderRightBack;
        this.imu = imu;
        sleepTime = 12; //50-75 is recommended
        xMultiplier = 0.8;
        yMultiplier = 0.66 * 11 / 4.792;
    }

    public double getGlobalX() {
        return globalX;
    }


    public double getGlobalY() {
        return globalY;
    }


    public double getAbsoluteHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC , AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public double getChangeHorizontal() {
        return changeHorizontal;
    }

    public double getChangeVertical() {
        return changeVertical;
    }

    public double getNormalizedHeading() {
        return (getAbsoluteHeading() + 450) % 360;
    }

    public void stop() {
        isRunning = false;
    }
    public double getTheta() {
        return theta;
    }

    private void updateGlobalCoordinate() {
        theta = Math.toRadians(getNormalizedHeading());

        currentEncoderCountLeft = encoderLeft.getCurrentPosition();
        currentEncoderCountRight = encoderRight.getCurrentPosition();
        deltas1 = currentEncoderCountLeft - lastEncoderCountLeft;
        lastEncoderCountLeft = currentEncoderCountLeft;
        deltas2 = currentEncoderCountRight - lastEncoderCountRight;
        lastEncoderCountRight = currentEncoderCountRight;
        deltax1 = deltas1 * 1 / Math.sqrt(2);
        deltax2 = deltas2 * 1 / Math.sqrt(2);
        deltay1 = deltas1 * 1 / Math.sqrt(2);
        deltay2 = deltas2 * 1 / Math.sqrt(2);


        currentEncoderCountLeftBack = encoderLeftBack.getCurrentPosition();
        currentEncoderCountRightBack = encoderRightBack.getCurrentPosition();
        deltas3 = currentEncoderCountLeftBack - lastEncoderCountLeftBack;
        lastEncoderCountLeftBack = currentEncoderCountLeftBack;
        deltas4 = currentEncoderCountRightBack - lastEncoderCountRightBack;
        lastEncoderCountRightBack = currentEncoderCountRightBack;
        deltax3 = deltas3 * 1 / Math.sqrt(2);
        deltax4 = deltas4 * 1 / Math.sqrt(2);
        deltay3 = deltas3 * 1 / Math.sqrt(2);
        deltay4 = deltas4 * 1 / Math.sqrt(2);

        changeHorizontal = (deltax1 - deltax2 - deltax3 + deltax4) / 2;
        changeVertical = (deltay1 + deltay2 + deltay3 + deltay4) / 4;

        double tempChangeHorizontal = changeHorizontal;
        double tempChangeVertical = changeVertical;

        changeHorizontal = Math.cos(theta) * tempChangeVertical + Math.sin(theta) * tempChangeHorizontal;
        changeVertical = Math.sin(theta) * tempChangeVertical + Math.cos(theta) * tempChangeHorizontal;

        globalX += changeHorizontal * xMultiplier;
        globalY += changeVertical * yMultiplier;
    }

    @Override
    public void run() {
        while (isRunning) {
            updateGlobalCoordinate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e){
                e.printStackTrace();
            }
        }
    }

    public void setGlobalX(double globalX) {
        this.globalX = globalX;
    }

    public void setGlobalY(double globalY) {
        this.globalY = globalY;
    }
}

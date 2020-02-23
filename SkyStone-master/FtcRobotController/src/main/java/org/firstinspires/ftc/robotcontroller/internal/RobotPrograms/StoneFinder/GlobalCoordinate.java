package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class GlobalCoordinate implements Runnable {
    private double globalX;
    private double globalY;
    private boolean isRunning;
    private DcMotor encoderLeft, encoderRight;
    private double deltas1, deltas2, deltax1, deltax2, deltay1, deltay2, changeHorizontal, changeVertical, theta;
    private double lastEncoderCountLeft, lastEncoderCountRight;
    private BNO055IMU imu;
    /**
     * Constructor for GlobalCoordinate
     * @param encoderLeft
     * @param encoderRight
     * @param imu
     */
    public GlobalCoordinate(DcMotor encoderLeft, DcMotor encoderRight, BNO055IMU imu) {
        globalX = 0;
        globalY = 0;
        isRunning = true;
        lastEncoderCountLeft = 0;
        lastEncoderCountRight = 0;
        this.encoderLeft = encoderLeft;
        this.encoderRight = encoderRight;
        this.imu = imu;
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

    private void updateGlobalCoordinate() {
        theta = getNormalizedHeading();
        deltas1 = encoderLeft.getCurrentPosition() - lastEncoderCountLeft;
        lastEncoderCountLeft = deltas1;
        deltas2 = encoderRight.getCurrentPosition() - lastEncoderCountRight;
        lastEncoderCountRight = deltas2;
        deltax1 = deltas1 * Math.cos(1 / Math.sqrt(2)) * deltas1;
        deltax2 = deltas2 * Math.cos(1 / Math.sqrt(2)) * deltas2;
        deltay1 = deltas1 * Math.sin(1 / Math.sqrt(2)) * deltas1;
        deltay2 = deltas2 * Math.sin(1 / Math.sqrt(2)) * deltas2;

        changeHorizontal = deltax1 - deltax2;
        changeVertical = (deltay1 + deltay2) / 2;
        double tempChangeHorizontal = changeHorizontal;
        double tempChangeVertical = changeVertical;

        changeHorizontal = Math.cos(theta) * tempChangeVertical + Math.sin(theta) * tempChangeHorizontal;
        changeVertical = Math.sin(theta) * tempChangeVertical + Math.cos(theta) * tempChangeHorizontal;

        globalX += changeHorizontal;
        globalY += changeVertical;
    }

    @Override
    public void run() {
        while (isRunning) {
            updateGlobalCoordinate();
        }
    }
}

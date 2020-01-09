package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;
@Disabled
@TeleOp

public class Lift extends TeleOpMode {
    final double tpr = 288;

    final double drumDiameter = 2;
    double drumCirc = Math.PI * (drumDiameter);
    final double brickHeight = 5;
    final double oneBlockHeight = brickHeight / (drumCirc * tpr);
    double currentPower = 0.8;
    double maxPower = 0.8;
    double minPower = 0.75;

    //Define Motors
    DcMotor rightLiftMotor;
    DcMotor leftLiftMotor;

    @Override
    public void init() {
        rightLiftMotor = hardwareMap.dcMotor.get("rm");
        leftLiftMotor = hardwareMap.dcMotor.get("lm");
        rightLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void stop() {
        super.stop();
    }

    @Override
    public void telemetry() {
        telemetry.addData("RightDrumPosition", rightLiftMotor.getCurrentPosition());
        telemetry.addData("LeftDrumPosition", leftLiftMotor.getCurrentPosition());
        telemetry.addData("RightPower", rightLiftMotor.getPower());
        telemetry.addData("LeftPower", leftLiftMotor.getPower());
        telemetry.addData("AverageDrumPosition", (rightLiftMotor.getCurrentPosition() + leftLiftMotor.getCurrentPosition()) / 2);
    }

    @Override
    public void updateData() {
        //lift one up by each block level per press
        if (gamepad2.y) {
            leftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            rightLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while ((leftLiftMotor.getCurrentPosition() + rightLiftMotor.getCurrentPosition()) / 2 < oneBlockHeight) {
                double leftPower = currentPower + ((leftLiftMotor.getCurrentPosition() - rightLiftMotor.getCurrentPosition()));
                double rightPower = currentPower + ((rightLiftMotor.getCurrentPosition() - leftLiftMotor.getCurrentPosition()));
                if (leftPower > maxPower) {
                    leftPower = maxPower;
                }
                if (leftPower < minPower) {
                    leftPower = minPower;
                }
                if (rightPower > maxPower) {
                    rightPower = maxPower;
                }
                if (rightPower < minPower) {
                    rightPower = minPower;
                }
                leftLiftMotor.setPower(leftPower);
                rightLiftMotor.setPower(rightPower);
            }
            leftLiftMotor.setPower(0);
            rightLiftMotor.setPower(0);
            leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        else if (gamepad2.a) {
            leftLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            rightLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while ((leftLiftMotor.getCurrentPosition() + rightLiftMotor.getCurrentPosition()) / 2 < oneBlockHeight) {
                double leftPower = 0.2 + ((leftLiftMotor.getCurrentPosition() - rightLiftMotor.getCurrentPosition()));
                double rightPower = 0.2 + ((rightLiftMotor.getCurrentPosition() - leftLiftMotor.getCurrentPosition()) );
                if (leftPower > 0.2) {
                    leftPower = 0.2;
                }
                if (leftPower < 0.2) {
                    leftPower = 0.2;
                }
                if (rightPower > 0.1) {
                    rightPower = 0.1;
                }
                if (rightPower < 0.1) {
                    rightPower = 0.1;
                }
                leftLiftMotor.setPower(leftPower);
                rightLiftMotor.setPower(rightPower);
            }
            leftLiftMotor.setPower(0);
            rightLiftMotor.setPower(0);
            leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}

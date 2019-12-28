package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

@TeleOp

public class Lift extends TeleOpMode {
    final double drumDiameter = 2;
    double drumCirc = Math.PI * (drumDiameter);
    final double brickHeight = 5;
    final double oneBlockHeight = 0.75;
    double currentPower = 0.4;

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
        telemetry.addData("Average Drum Position", (rightLiftMotor.getCurrentPosition() + leftLiftMotor.getCurrentPosition()) / 2);
    }

    @Override
    public void updateData() {
        if (gamepad1.y) {
            leftLiftMotor.setTargetPosition((int) (leftLiftMotor.getCurrentPosition() + oneBlockHeight));
            rightLiftMotor.setTargetPosition((int) (rightLiftMotor.getCurrentPosition() + oneBlockHeight));
            leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (leftLiftMotor.isBusy() && rightLiftMotor.isBusy()) {
                leftLiftMotor.setPower(currentPower);
                rightLiftMotor.setPower(currentPower);
            }
            leftLiftMotor.setPower(0);
            rightLiftMotor.setPower(0);

            leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        else if (gamepad1.a) {
            leftLiftMotor.setTargetPosition((int) (leftLiftMotor.getCurrentPosition() - oneBlockHeight));
            rightLiftMotor.setTargetPosition((int) (rightLiftMotor.getCurrentPosition() - oneBlockHeight));
            leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (leftLiftMotor.isBusy() && rightLiftMotor.isBusy()) {
                leftLiftMotor.setPower(currentPower);
                rightLiftMotor.setPower(currentPower);
            }
            leftLiftMotor.setPower(0);
            rightLiftMotor.setPower(0);

            leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}

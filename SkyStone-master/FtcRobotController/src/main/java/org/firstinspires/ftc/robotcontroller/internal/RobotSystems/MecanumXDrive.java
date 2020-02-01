package org.firstinspires.ftc.robotcontroller.internal.RobotSystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.TestProps.MecanumX;

public class MecanumXDrive extends DriveTrain {
    public Boolean dir = true;
    public MecanumXDrive() {}
    public void telemetry(){
        telemetry.addData("MECANUM X DRIVE", "TELEMETRY");
        telemetry.addData(">>>Front Left Pwr", motors.get(0).getPower());
        telemetry.addData(">>>Front Right Pwr", motors.get(1).getPower());
        telemetry.addData(">>>Back Left Pwr", motors.get(2).getPower());
        telemetry.addData(">>>Back Right Pwr", motors.get(3).getPower());
    }

    public void update(){
        if (dir) {
            motors.get(0).setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x * turnPwrMax) * drivePwrMax); //-
            motors.get(1).setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x * turnPwrMax) * drivePwrMax); //+
            motors.get(2).setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x * turnPwrMax) * drivePwrMax); //+
            motors.get(3).setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x * turnPwrMax) * drivePwrMax); //-
        } else {
            motors.get(0).setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x * turnPwrMax) * drivePwrMax); //-
            motors.get(1).setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x * turnPwrMax) * drivePwrMax); //+
            motors.get(2).setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x * turnPwrMax) * drivePwrMax); //+
            motors.get(3).setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x * turnPwrMax) * drivePwrMax); //-
        }

    }

    @Override
    public void move(double... velocities) {

    }

    @Override
    public void moveForward(double velocity) {

    }

    @Override
    public void turnClockwise(double angularVelocity) {

    }

    public void flipMechanic(boolean bool) {
        if (bool) {
            for (int i = 0; i < numOfMotors; i++) {
                // set the direction of the motors
                if (i % 2 == 0) motors.get(i).setDirection(DcMotorSimple.Direction.REVERSE);
                else motors.get(i).setDirection(DcMotorSimple.Direction.FORWARD);
            }
        } else {
            for (int i = 0; i < numOfMotors; i++) {
                // set the direction of the motors
                if (i % 2 == 0) motors.get(i).setDirection(DcMotorSimple.Direction.FORWARD);
                else motors.get(i).setDirection(DcMotorSimple.Direction.REVERSE);
            }
            dir = false;
        }
    }
}

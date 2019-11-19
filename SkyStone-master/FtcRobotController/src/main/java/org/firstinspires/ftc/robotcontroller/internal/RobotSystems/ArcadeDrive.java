package org.firstinspires.ftc.robotcontroller.internal.RobotSystems;

import com.qualcomm.hardware.bosch.BNO055IMU;

public class ArcadeDrive extends DriveTrain {
    public double angle = 90;
    public double MOTOR_POWER_MAX = 1;
    public ArcadeDrive() {
        this.angle = angle; //ex: ArcadeDrive arcade = new ArcadeDrive(5.0);
        this.MOTOR_POWER_MAX = 0.5;
        //constructor function: activated by the new operator and will create an instance (object) ex: ArcadeDrive arcade = new ArcadeDrive();
    }
    @Override
    public void update() {
        double flPower, frPower;
        flPower = (-gamepad1.left_stick_y - gamepad1.right_stick_x) * drivePwrMax;
        frPower = (-gamepad1.left_stick_y + gamepad1.right_stick_x) * drivePwrMax;
        for (int i = 0; i < numOfMotors; i++) {
            if (i % 2 == 0) {
                motors.get(i).setPower(clipPower(flPower));
            } else {
                motors.get(i).setPower(clipPower(frPower));
            }
        }
    }

    @Override
    public void moveForward(double velocity) {

    }


    @Override
    public void telemetry() {
        telemetry.addData("DriveTrain","TankDrive");
        telemetry.addData("LeftPwr", motors.get(0).getPower());
        telemetry.addData("RightPwr", motors.get(1).getPower());
    }

    @Override
    public void move(double... velocities) {

    }

    @Override
    public void turnClockwise(double targetAngle) {

    }

    public void turnClockwiseZ(double angle) {
        motors.get(0).setPower(0.6);
        motors.get(1).setPower(-0.6);
        if (angle > 0) {
            angle--;
            turnClockwise(angle);
            telemetry.addData("Angles", angle);
        } else {
            motors.get(0).setPower(0);
            motors.get(1).setPower(0);
        }
    }
}

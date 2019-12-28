package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi.OldArchive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

@TeleOp
//@Disabled
public class NewIntake extends TeleOpMode {
    DcMotor rs;
    DcMotor ls;
//    Servo pusssh;
    final double maxPos = 1;
    final double minPos = 0;
    @Override
    public void init() {
        rs = hardwareMap.dcMotor.get("rs");
        rs.setDirection(DcMotorSimple.Direction.FORWARD);
        ls = hardwareMap.dcMotor.get("ls");
        ls.setDirection(DcMotorSimple.Direction.FORWARD);
//        pusssh = hardwareMap.servo.get("pusssh");
//        pusssh.setDirection(Servo.Direction.FORWARD);
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

    }

    @Override
    public void updateData() {
        //intake
        if (gamepad2.right_bumper) {
            rs.setPower(0.5);
            ls.setPower(0.5);
        }
        else if (gamepad2.left_bumper) {
            rs.setPower(-0.5);
            ls.setPower(-0.5);
        }
        else {
            rs.setPower(0);
            ls.setPower(0);
        }
        //servo
//        if (gamepad2.y) {
//            pusssh.setPosition(maxPos);
//        }
//        else if (gamepad2.x) {
//            pusssh.setPosition(0.9);
//        }
//        else if (gamepad2.a) {
//            pusssh.setPosition(minPos);
//        }
//
//        //bounds
//        if (pusssh.getPosition() > maxPos) {
//            pusssh.setPosition(maxPos);
//        }
//        if (pusssh.getPosition() < minPos) {
//            pusssh.setPosition(minPos);
//        }
    }
}

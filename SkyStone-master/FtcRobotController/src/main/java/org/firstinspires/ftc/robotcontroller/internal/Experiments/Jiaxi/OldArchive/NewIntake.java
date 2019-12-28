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
    DcMotor rs, ls;
    Servo pusssh;
    DcMotor fr, fl, bl, br;
    final double maxPos = 1;
    final double minPos = 0;
    double currentPower;
    @Override
    public void init() {
        rs = hardwareMap.dcMotor.get("rs");
        rs.setDirection(DcMotorSimple.Direction.FORWARD);
        ls = hardwareMap.dcMotor.get("ls");
        ls.setDirection(DcMotorSimple.Direction.FORWARD);
        pusssh = hardwareMap.servo.get("pusssh");
        pusssh.setDirection(Servo.Direction.FORWARD);

        fl = hardwareMap.dcMotor.get("fl");
        bl = hardwareMap.dcMotor.get("bl");
        fr = hardwareMap.dcMotor.get("fr");
        br = hardwareMap.dcMotor.get("br");
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
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
        telemetry.addData("left wheel", ls.getPower());
        telemetry.addData("right wheel", rs.getPower());
        telemetry.addData("Servo position", pusssh.getPosition());
    }

    @Override
    public void updateData() {
        //intake
        if (gamepad2.right_bumper) {
            rs.setPower(0.4);
            ls.setPower(0.4);
        }
        else if (gamepad2.left_bumper) {
            rs.setPower(-0.4);
            ls.setPower(-0.4 );
        }
        else {
            rs.setPower(0);
            ls.setPower(0);
        }
        //servo
        if (gamepad2.y) {
            pusssh.setPosition(minPos);
        }
        else if (gamepad2.a) {
            pusssh.setPosition(0.5);
        }

        //bounds
        if (pusssh.getPosition() > maxPos) {
            pusssh.setPosition(maxPos);
        }
        if (pusssh.getPosition() < minPos) {
            pusssh.setPosition(minPos);
        }

        //drive train
        currentPower = gamepad1.left_stick_y * currentPower;
        fr.setPower(currentPower);
        br.setPower(currentPower);
        bl.setPower(currentPower);
        fl.setPower(currentPower);
    }
}

package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi.OldArchive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

@TeleOp
@Disabled
public class MecanumTest extends TeleOpMode {
    DcMotor fr;
    DcMotor fl;
    DcMotor br;
    DcMotor bl;
    final double turnPwrMax = 0.6;
    final double drivePwrMax = 0.7;
    @Override
    public void init() {
        fr = hardwareMap.dcMotor.get("fr");
        fl = hardwareMap.dcMotor.get("fl");
        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get("bl");
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
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
//        fr.setPower(gamepad1.left_stick_y * drivePwrMax);
//        fl.setPower(gamepad1.left_stick_y * drivePwrMax);
//        br.setPower(gamepad1.left_stick_y * drivePwrMax);
//        bl.setPower(gamepad1.left_stick_y * drivePwrMax);
//
//        fr.setPower(gamepad1.left_stick_x * dr);
    }
}

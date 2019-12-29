package org.firstinspires.ftc.robotcontroller.internal.Experiments.Calvin;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

@TeleOp
public class Scoring extends TeleOpMode {

    DcMotor leftSlide;
    DcMotor rightSlide;

    final double slidePwr = 0.7;

    @Override
    public void init() {
        leftSlide = hardwareMap.dcMotor.get("leftSlide");
        rightSlide = hardwareMap.dcMotor.get("rightSlide");
        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void telemetry() {
        telemetry.addData("leftSlide_pwr", leftSlide.getPower());
        telemetry.addData("rightSlide_pwr", leftSlide.getPower());
    }

    @Override
    public void updateData() {
        updateSlides();
    }

    public void updateSlides() {
        double pwr = gamepad2.left_stick_y * slidePwr;
        leftSlide.setPower(pwr);
        rightSlide.setPower(pwr);
    }
}

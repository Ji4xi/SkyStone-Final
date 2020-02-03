package org.firstinspires.ftc.robotcontroller.internal.Experiments.Calvin;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

@Disabled
@TeleOp
public class Scoring extends TeleOpMode {

    DcMotor ls;
    DcMotor rs;

    final double slidePwr = 0.9;

    final double COUNTS_PER_REVOLUTION = 288;
    final double COUNTS_PER_INCH = COUNTS_PER_REVOLUTION / 6.69291;
    final double COUNTS_PER_BRICK = COUNTS_PER_INCH * 5;

    @Override
    public void init() {
        ls = hardwareMap.dcMotor.get("ls");
        rs = hardwareMap.dcMotor.get("rs");
        ls.setDirection(DcMotorSimple.Direction.FORWARD);
        rs.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void telemetry() {
        telemetry.addData("leftSlide_pwr", ls.getPower());
        telemetry.addData("rightSlide_pwr", ls.getPower());

    }

    @Override
    public void updateData() {
        updateSlides();
    }

    public void updateSlides() {
        double leftSlidePwr = gamepad2.left_stick_y * slidePwr;
        double rightSlidePwr = gamepad2.left_stick_y * slidePwr;

        leftSlidePwr = gamepad2.left_stick_y > 0 ? 0.2 : leftSlidePwr;
        rightSlidePwr = gamepad2.left_stick_y > 0 ? 0.2 : rightSlidePwr;


        if (gamepad2.x) {
            double nearest = COUNTS_PER_BRICK * Math.round((ls.getCurrentPosition() + rs.getCurrentPosition()) / 2 / COUNTS_PER_BRICK);

            leftSlidePwr =  slidePwr * (nearest - ls.getCurrentPosition())/COUNTS_PER_BRICK;
            rightSlidePwr = slidePwr * (nearest - rs.getCurrentPosition())/COUNTS_PER_BRICK;

        }

        ls.setPower(leftSlidePwr);
        rs.setPower(rightSlidePwr);
    }
}

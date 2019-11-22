package org.firstinspires.ftc.robotcontroller.internal.Experiments.Calvin.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

@Disabled
@TeleOp
public class Lift extends TeleOpMode {

    //motors
    DcMotor rotateMotor;
    DcMotor slideMotor;

    static final double ROTATE_PWR_MAX = 0.2;
    static final double SLIDE_PWR_MAX = 0.7;

    double rotatePower;
    double slidePower;

    static final double COUNTS_PER_REVOLUTION = 1680;

    //servos
    Servo gripServo;

    static final double MAX_POS = 0.5;
    static final double MIN_POS = 0;

    double gripPosition;
    double angle;

    @Override
    public void init() {
        rotateMotor = hardwareMap.get(DcMotor.class, "rotateMotor");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");

        rotateMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        rotateMotor.setPower(0);
        rotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        gripServo = hardwareMap.get(Servo.class, "gripServo");

        gripServo.setDirection(Servo.Direction.FORWARD);

        gripServo.setPosition(MIN_POS);
        gripPosition = MIN_POS;
    }

    @Override
    public void telemetry() {
        telemetry.addData("rotateMotor", rotateMotor.getPower());
        telemetry.addData("slideMotor", slideMotor.getPower());
    }

    @Override
    public void updateData() {

        if (gamepad1.dpad_right) gripPosition += 0.01;
        else if (gamepad1.dpad_left) gripPosition -= 0.01;

        //bounds
        if (gripPosition > MAX_POS) gripPosition = MAX_POS;
        else if (gripPosition < MIN_POS) gripPosition = MIN_POS;

        gripServo.setPosition(Range.clip(gripPosition, MIN_POS, MAX_POS));

    }

}

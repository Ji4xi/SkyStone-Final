package org.firstinspires.ftc.robotcontroller.internal.Experiments.Calvin;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

@Disabled
@TeleOp
public class LiftRedo extends TeleOpMode {
    //rotate motor
    DcMotor rotateMotor;
    static final double ROTATE_PWR_MAX = 0.2;
    static double rotatePower;
    static final double COUNTS_PER_REVOLUTION = 1680; //change to 60:1 (1680) motor for actual lift, 40:1 (1120) for testing

    //slide motor
    DcMotor slideMotor;
    static final double SLIDE_PWR_MAX = 0.7;
    static double slidePower;

    //pivot servo
    Servo pivotServo;
    static final double SERVO_MAX_POS = 0.5;
    static final double SERVO_MIN_POS = 0;
    static double pivotPosition;

    //grip servo
    Servo gripServo;
    static final double GRIP_MAX_POS = 0.5;
    static final double GRIP_MIN_POS = 0;
    static double gripPosition;

    static double angle;

    @Override
    public void init() {
        rotateMotor = hardwareMap.get(DcMotor.class, "rotateMotor");
        rotateMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        rotateMotor.setPower(0);
        rotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        pivotServo = hardwareMap.get(Servo.class, "pivotServo");
        pivotServo.setDirection(Servo.Direction.FORWARD);

    }

    @Override
    public void telemetry() {
        telemetry.addData("rotateMotor pwr", rotateMotor.getPower());
        telemetry.addData("rotateMotor position", rotateMotor.getCurrentPosition());
        telemetry.addData("angle", angle);

        telemetry.addData("slideMotor pwr", slideMotor.getPower());
    }

    @Override
    public void updateData() {

        //dpad up and down to rotate
        if (gamepad1.dpad_up) rotatePower = 1;
        else if (gamepad1.dpad_down) rotatePower = -1;
        else rotatePower = 0;
        rotateMotor.setPower(rotatePower * ROTATE_PWR_MAX);

        angle = rotateMotor.getCurrentPosition() / COUNTS_PER_REVOLUTION * 360;

        //right stick y to slide
        slidePower = gamepad1.right_stick_y;
        slideMotor.setPower(slidePower * SLIDE_PWR_MAX);

        //pivot servo follows angle of the lift
        pivotServo.setPosition(angle / 180);

        //x, y, a buttons to grip
        if (gamepad1.x) gripPosition = GRIP_MAX_POS;
        else if (gamepad1.y) gripPosition = GRIP_MIN_POS;

        //dpad left and right to adjust grip
        if (gamepad1.dpad_right) gripPosition += 0.01;
        else if (gamepad1.dpad_left) gripPosition -= 0.01;

        //bounds
        if (gripPosition > GRIP_MAX_POS) gripPosition = GRIP_MAX_POS;
        else if (gripPosition < GRIP_MIN_POS) gripPosition = GRIP_MIN_POS;

        gripServo.setPosition(Range.clip(gripPosition, GRIP_MIN_POS, GRIP_MAX_POS));

    }

}

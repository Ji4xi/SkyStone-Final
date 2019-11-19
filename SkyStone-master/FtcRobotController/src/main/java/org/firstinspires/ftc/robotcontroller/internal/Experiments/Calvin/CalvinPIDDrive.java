package org.firstinspires.ftc.robotcontroller.internal.Experiments.Calvin;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@Disabled
@Autonomous
public class CalvinPIDDrive extends LinearOpMode {

    DcMotor motor;
    static final double MOTOR_PWR_MAX = 0.8;
    static final double COUNTS_PER_REVOLUTION = 1120;

    CalvinPID PID = new CalvinPID(0.012, 0.001, 0.0022);

    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();


        rotatePID(5, 0);

    }

    public void rotatePID(double revolutions, double error) {
        PID.setTarget(revolutions * COUNTS_PER_REVOLUTION);

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition((int) (revolutions * COUNTS_PER_REVOLUTION));

        long startTime = System.nanoTime();

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && motor.isBusy()) {
            motor.setPower(PID.getOutput(motor.getCurrentPosition()) * MOTOR_PWR_MAX);

            telemetry.addData("position", motor.getCurrentPosition() / COUNTS_PER_REVOLUTION);
            telemetry.addData("power", motor.getPower());
            telemetry.addData("time", System.nanoTime() - startTime);
            telemetry.update();
        }

        stopMotorAndResetEncoder();
        PID.reset();

        sleep(5000);
    }

    public void stopMotorAndResetEncoder() {
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}

package org.firstinspires.ftc.robotcontroller.internal.Experiments.Michael;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class realVelocity extends LinearOpMode {
    DcMotor fr;
    @Override
    public void runOpMode() throws InterruptedException {
        fr = hardwareMap.dcMotor.get("fr");
        waitForStart();
        fr.setPower(1);
        sleep(2000);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double time = System.nanoTime();
        double lastCount = 0;
        while (537.6 - lastCount > 0) {
            lastCount = fr.getCurrentPosition();
        }
        time = (System.nanoTime() - time) * Math.pow(10, -9);
        telemetry.addData("time", time);
        telemetry.update();
        sleep(5000);
    }
}

package org.firstinspires.ftc.robotcontroller.internal.Experiments.Michael;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@Autonomous
public class PIDAutoMike extends LinearOpMode {
    DcMotor rm;
    DcMotor lm;
    @Override
    public void runOpMode() throws InterruptedException {
        rm = hardwareMap.dcMotor.get("rm");
        rm.setDirection(DcMotorSimple.Direction.FORWARD);
        lm = hardwareMap.dcMotor.get("lm");
        lm.setDirection(DcMotorSimple.Direction.REVERSE);
        rm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
    }
}

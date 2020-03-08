package org.firstinspires.ftc.robotcontroller.internal.Experiments.Michael;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

public class TestForPID extends TeleOpMode {
    DcMotorEx fr;

    @Override
    public void init() {
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void updateData() {

    }

    @Override
    public void telemetry() {

    }


}

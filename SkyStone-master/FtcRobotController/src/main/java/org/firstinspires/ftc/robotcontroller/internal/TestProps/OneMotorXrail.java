package org.firstinspires.ftc.robotcontroller.internal.TestProps;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

public class OneMotorXrail extends TeleOpMode {
    DcMotor xr;
    final double pwrMax = 0.8;
    double currentPwr;

    @Override
    public void init() {
        xr = hardwareMap.dcMotor.get("xr");
        xr.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void telemetry() {
        telemetry.addData("X-rail_Pwr", currentPwr);
    }

    @Override
    public void updateData() {
        updateDrawerSlide();
    }

    public void updateDrawerSlide() {
        currentPwr = pwrMax * gamepad2.left_stick_y;
        xr.setPower(Range.clip(currentPwr, -pwrMax, pwrMax));
    }
}

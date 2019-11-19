package org.firstinspires.ftc.robotcontroller.internal.TestProps;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

public class OneServoOneMotor extends TeleOpMode {
    DcMotor xr;
    final double pwrMax = 0.8;
    double currentPwr;

    Servo os;
    final double maxPos = 1;
    final double midPos = 0.625;
    final double minPos = 0.25;

    @Override
    public void init() {
        xr = hardwareMap.dcMotor.get("xrz");
        xr.setDirection(DcMotorSimple.Direction.FORWARD);
        os = hardwareMap.servo.get("osz");
        os.setDirection(Servo.Direction.REVERSE);

    }

    @Override
    public void telemetry() {
        telemetry.addData("X-rail_Pwr", currentPwr);
        telemetry.addData("OServo_Pos", os.getPosition());
    }

    @Override
    public void updateData() {
        updateDrawerSlide();
        updateServo();
    }

    public void updateDrawerSlide() {
        currentPwr = pwrMax * gamepad2.left_stick_y;
        xr.setPower(Range.clip(currentPwr, -pwrMax, pwrMax));
    }

    public void updateServo() {
        if (gamepad2.y) {
            os.setPosition(maxPos);
        }
        if (gamepad2.x) {
            os.setPosition(midPos);
        }
        if (gamepad2.a) {
            os.setPosition(minPos);
        }
    }
}

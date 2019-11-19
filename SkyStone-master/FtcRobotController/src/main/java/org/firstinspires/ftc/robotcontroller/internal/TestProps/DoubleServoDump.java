package org.firstinspires.ftc.robotcontroller.internal.TestProps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

public class DoubleServoDump extends TeleOpMode {
    DcMotor xr;
    final double pwrMax = 0.8;
    double currentPwr;
    Servo rs;
    Servo ls;
    final double maxPosDSD = 1;
    final double minPosDSD = 0.3;
    @Override
    public void init() {
        xr = hardwareMap.dcMotor.get("xr");
        xr.setDirection(DcMotorSimple.Direction.FORWARD);
        rs = hardwareMap.servo.get("rs");
        rs.setDirection(Servo.Direction.REVERSE);
        rs.setPosition(minPosDSD);
        ls = hardwareMap.servo.get("ls");
        ls.setDirection(Servo.Direction.FORWARD);
        ls.setPosition(minPosDSD);
    }

    @Override
    public void telemetry() {
        telemetry.addData("RDSDServo_Pos", rs.getPosition());
        telemetry.addData("LDSDServo_Pos", ls.getPosition());
    }

    @Override
    public void updateData() {
        if (gamepad2.y) {
            rs.setPosition(maxPosDSD);
            ls.setPosition(maxPosDSD);
        }
        if (gamepad2.a) {
            rs.setPosition(minPosDSD);
            ls.setPosition(minPosDSD);
        }
        currentPwr = pwrMax * gamepad2.left_stick_y;
        xr.setPower(Range.clip(currentPwr, -pwrMax, pwrMax));
    }
}

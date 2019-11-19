package org.firstinspires.ftc.robotcontroller.internal.TestProps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;
import org.firstinspires.ftc.robotcontroller.internal.RobotSystems.MecanumXDrive;

//@Disabled
@TeleOp
public class MecanumX extends TeleOpMode {
    MecanumXDrive mecanumXDrive = new MecanumXDrive();

    @Override
    public void init() {
        mecanumXDrive.syncOpMode(gamepad1, telemetry, hardwareMap);
        mecanumXDrive.init("fr","fl","br","bl"); //fr br fl bl
    }

    @Override
    public void telemetry() {
        mecanumXDrive.telemetry();
    }

    @Override
    public void updateData() {
        mecanumXDrive.update();
    }
}

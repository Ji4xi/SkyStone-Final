package org.firstinspires.ftc.robotcontroller.internal.TestProps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;
import org.firstinspires.ftc.robotcontroller.internal.RobotSystems.ArcadeDrive;
import org.firstinspires.ftc.robotcontroller.internal.RobotSystems.DriveTrain;

/**
 * two motor arcade drive for teleop
 */
//@Disabled
@TeleOp
@Disabled
public class Arcade extends TeleOpMode {
    ArcadeDrive arcade = new ArcadeDrive();

    @Override    public void init() {
        arcade.syncOpMode(gamepad1,telemetry,hardwareMap);
        arcade.init("fr","fl");
    }

    @Override
    public void telemetry() {
        arcade.telemetry();
    }

    @Override
    public void updateData() {
        arcade.update();
    }
}

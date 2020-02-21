package org.firstinspires.ftc.robotcontroller.internal.Experiments.Michael;

import android.database.sqlite.SQLiteException;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder.NewStoneFinder;

@Autonomous
public class RandomAuto extends NewStoneFinder {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        moveInches(40, 90, 90);
        sleep(4000);
        telemetry();
        telemetry.update();
        sleep(4000);
    }
}

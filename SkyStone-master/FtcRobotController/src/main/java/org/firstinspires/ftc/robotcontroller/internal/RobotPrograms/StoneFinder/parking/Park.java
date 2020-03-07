package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder.parking;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder.StoneFinder;


@Autonomous
@Disabled
public class Park extends StoneFinder {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        sleep(27000);
        //XLinearInchesGyro(9, Side.LEFT, 0.7, 0);
    }
}

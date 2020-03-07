package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class Left extends Right {
    @Override
    public void runOpMode() throws InterruptedException {
        mode = false;
        super.runOpMode();
    }
}

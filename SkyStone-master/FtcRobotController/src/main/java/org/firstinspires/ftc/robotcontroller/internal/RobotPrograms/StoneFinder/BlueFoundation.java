package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcontroller.internal.TestProps.DrawerSlides;

@Autonomous
@Disabled
public class BlueFoundation extends StoneFinder {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        foundation(Mode.OPEN,250); //set claw to open position (just in case)
        XLinearInchesGyro(8, Side.RIGHT, 0.45, 0); //move slightly to the right to line the robot with the foundation
        YLinearInchesGyro(28, GAGE.REVERSE, 0.45, 0); //move towards the foundation
        foundation(Mode.CLOSE, 500); //close the claws to grip onto the foundation
        YLinearInchesGyro(30, GAGE.FORWARD, 0.35, 0); //pull the foundation to the build site by moving backwards
        foundation(Mode.OPEN,1000); //open the claw to release the foundation from the robot
        XLinearInchesGyro(43, Side.LEFT, 0.30, -18);

    }
}
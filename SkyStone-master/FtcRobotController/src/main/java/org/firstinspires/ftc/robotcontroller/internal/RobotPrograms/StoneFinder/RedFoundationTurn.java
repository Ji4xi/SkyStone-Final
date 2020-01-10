package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedFoundationTurn extends StoneFinder {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        foundation(Mode.OPEN,250); //set claw to open position (just in case)
        moveInchesPID(51, 270, 1000); //move forward
        turnPID(-90, Direction.CLOCKWISE, 0.4); //turn so that the claws face the foundation
        foundation(Mode.CLOSE,500); //close the claws to grip onto the foundation
        moveInchesPID(45,180,1000); //drag the foundation to the build site by moving right
        foundation(Mode.OPEN,500); //open the claw to release the foundation from the robot
        moveInchesPID(24, 270, 1000); //park
    }
}

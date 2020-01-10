package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueFoundation extends StoneFinder {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        foundation(Mode.OPEN,250); //set claw to open position (just in case)
        moveInchesPID(8, 0, 1000); //move slightly to the right to line the robot with the foundation
        moveInchesPID(28, 270, 1000); //move towards the foundation
        foundation(Mode.CLOSE, 500); //close the claws to grip onto the foundation
        moveInchesPID(28, 90, 1000); //pull the foundation to the build site by moving backwards
        foundation(Mode.OPEN,500); //open the claw to release the foundation from the robot
        moveInchesPID(43, 180, 1000); //park

    }
}
package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class SkyScraperIntakeBlue extends SkyScraper{
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        long sleepTime = 700;

        Brick position;

        //move toward bricks
        moveForwardInchesGyro(25, 0, sleepTime);
        //turn toward bricks
        turnAbsolutePID(-90, Direction.CLOCKWISE, 0.3);
        sleep(sleepTime, "turn toward bricks");

        double distance = moveForwardInches(24, true);

        if (distance < 8) {
            position = Brick.LEFT;
        }
        else if (8 < distance && distance < 16) {
            position = Brick.MIDDLE;
        }
        else {
            position = Brick.RIGHT;
        }

        sleep(sleepTime, "move to first skystone");

        //turn toward skystone
        turnAbsolutePID(0, Direction.COUNTERCLOCKWISE, 0.3);

        // intake skystone

        // turn toward bridge
        turnAbsolutePID(90, Direction.COUNTERCLOCKWISE, 0.3);

        //move to skystone
        switch (position) {
            case LEFT:
                moveForwardInches(4, sleepTime);
            case MIDDLE:
                moveForwardInches(12, sleepTime);
            case RIGHT:
                moveForwardInches(20, sleepTime);
        }

//        // move toward bridge
//        moveForwardInches(25, sleepTime);
//
//        //finish turning toward bridge
//        turnAbsolutePID(-90, Direction.CLOCKWISE, 0.3);
//
//        // move under bridge
//        moveForwardInches(35);
//
//        //drop stone
//
//        //go back
//        moveForwardInches(-35);
//        turnAbsolutePID(-135, Direction.COUNTERCLOCKWISE, 0.3);
//        moveForwardInches(-25);
//        turnAbsolutePID(-90, Direction.CLOCKWISE, 0.3);
//
//        //move to other skystone
//        moveForwardInches(-24);
//
//        //turn toward skystone
//        turnAbsolutePID(0, Direction.COUNTERCLOCKWISE, 0.3);
//
//        // intake skystone
//
//        // turn toward bridge
//        turnAbsolutePID(-135, Direction.COUNTERCLOCKWISE, 0.3);
//
//        // move toward bridge
//        moveForwardInches(25);
//
//        //finish turning toward bridge
//        turnAbsolutePID(-90, Direction.CLOCKWISE, 0.3);
//
//        // move under bridge
//        moveForwardInches(35);

        //drop stone


//        telemetry.addData("SkyStone", "Looking");
//        telemetry.update();
//        FoundSkyStoneAngle();
//        telemetry.addData("SkyStone", "Found");
//        telemetry.update();
//        stopMotor();
    }

    enum Brick {
        LEFT, MIDDLE, RIGHT
    }

}

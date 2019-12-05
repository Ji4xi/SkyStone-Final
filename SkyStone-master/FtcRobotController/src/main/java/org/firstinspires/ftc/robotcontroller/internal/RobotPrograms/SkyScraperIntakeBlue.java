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

        //move to skystone
        double brickDistance = 0;
        switch (position) {
            case LEFT:
                brickDistance = 4;
                break;
            case MIDDLE:
                brickDistance = 12;
                break;
            case RIGHT:
                brickDistance = 20;
                break;
        }

        sleep(sleepTime, "move to first skystone");

        //turn toward skystone
        turnAbsolutePID(0, Direction.COUNTERCLOCKWISE, 0.3);
        sleep(sleepTime, "turn toward skystone");

        // intake skystone

        // turn toward bridge
        turnAbsolutePID(135, Direction.COUNTERCLOCKWISE, 0.3);
        sleep(sleepTime, "turn toward bridge");

        moveForwardInchesGyro(25, 135, sleepTime);

        turnAbsolutePID(90, Direction.CLOCKWISE, 0.3);
        sleep(sleepTime, "turn toward parking space");

        // move under bridge

        moveForwardInchesGyro(12 + brickDistance, 90, sleepTime);

        //drop stone

        //go back
        moveForwardInchesGyro(-12 - brickDistance, 90, sleepTime);
        turnAbsolutePID(135, Direction.COUNTERCLOCKWISE, 0.3);
        sleep(sleepTime);
        moveForwardInches(-25, sleepTime);
        turnAbsolutePID(90, Direction.CLOCKWISE, 0.3);
        sleep(sleepTime);

        //move to other skystone
        moveForwardInches(-24, sleepTime);

        //turn toward skystone
        turnAbsolutePID(0, Direction.CLOCKWISE, 0.3);
        sleep(sleepTime, "turn toward skystone");

        // intake skystone

        // turn toward bridge
        turnAbsolutePID(135, Direction.COUNTERCLOCKWISE, 0.3);
        sleep(sleepTime, "turn toward bridge");

        moveForwardInchesGyro(25, 135, sleepTime);

        turnAbsolutePID(90, Direction.CLOCKWISE, 0.3);
        sleep(sleepTime, "turn toward parking space");

        // move under bridge

        moveForwardInchesGyro(36 + brickDistance, 90, sleepTime);

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

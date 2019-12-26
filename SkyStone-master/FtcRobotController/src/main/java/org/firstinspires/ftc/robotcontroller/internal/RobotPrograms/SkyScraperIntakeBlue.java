package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class SkyScraperIntakeBlue extends SkyScraper{
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        long sleepTime = 400;

//        Brick position;

        //move toward bricks
        moveForwardInchesGyro(22.5, 0, sleepTime);
        //turn toward bricks
        turnAbsolutePID(-90, Direction.CLOCKWISE, 0.3);
        sleep(sleepTime, "turn toward bricks");
        int blockPlace = 0;

        for (int i = 0; i < 2; i++) {
            blockPlace = i + 1;
            if (FoundSkyStoneAngle(500)) {
                telemetry.addData("found", "found");
                telemetry.update();
                break;
            }
            moveForwardInchesGyro(8, -90, 0, 0.3);
        }

//        double distance = moveForwardInches(24, true);
//
//        if (distance < 8) {
//            position = Brick.LEFT;
//        }
//        else if (8 < distance && distance < 16) {
//            position = Brick.MIDDLE;
//        }
//        else {
//            position = Brick.RIGHT;
//        }
//
//        //move to skystone
//        double brickDistance = 0;
//        switch (position) {
//            case LEFT:
//                brickDistance = 4;
//                break;
//            case MIDDLE:
//                brickDistance = 12;
//                break;
//            case RIGHT:
//                brickDistance = 20;
//                break;
//        }
//
//        sleep(sleepTime, "move to first skystone");

        //turn toward skystone
        moveForwardInchesGyro(3, -90, sleepTime);
        turnAbsolutePID(25, Direction.COUNTERCLOCKWISE, 0.45);
        moveForwardInchesGyro(2, 25, 500);
        sleep(sleepTime, "turn toward skystone");

        // intake skystone
        rightIntake.setPower(0.8);
        leftIntake.setPower(0.8);

        moveForwardInchesGyro(15, 10);
        // turn toward bridge
        rightIntake.setPower(0);
        leftIntake.setPower(0);
        turnAbsolutePID(135, Direction.COUNTERCLOCKWISE, 0.3);

        sleep(sleepTime, "turn toward bridge");

        moveForwardInchesGyro(25, 135, sleepTime, 0.5);

        turnAbsolutePID(90, Direction.CLOCKWISE, 0.3);
        sleep(sleepTime, "turn toward parking space");

        // move under bridge

        moveForwardInchesGyro(18 + (12 * blockPlace), 90, sleepTime);
        rightIntake.setPower(-0.3);
        leftIntake.setPower(-0.3);
        sleep(500);
        moveForwardInchesGyro(-5, 90, sleepTime);

//        //drop stone
//        rightIntake.setPower(-0.8);
//        leftIntake.setPower(-0.8);
//        sleep(1000);
//        //go back and park avoid going backward
////        turnAbsolutePID(-90, Direction.COUNTERCLOCKWISE, 0.3);
////        moveForwardInchesGyro(5, -90);
////        moveForwardInchesGyro(-12 - brickDistance, 90, sleepTime);
//        rightIntake.setPower(0);
//        leftIntake.setPower(0);
////        turnAbsolutePID(134, Direction.COUNTERCLOCKWISE, 0.3);
//        sleep(sleepTime);
//        moveForwardInches(-25, sleepTime);
//        turnAbsolutePID(90, Direction.CLOCKWISE, 0.3);
//        sleep(sleepTime);

//        //move to other skystone
//        moveForwardInches(-24, sleepTime);
//
//        //turn toward skystone
//        turnAbsolutePID(0, Direction.CLOCKWISE, 0.3);
//        sleep(sleepTime, "turn toward skystone");
//
//        // intake skystone
//
//        // turn toward bridge
//        turnAbsolutePID(135, Direction.COUNTERCLOCKWISE, 0.3);
//        sleep(sleepTime, "turn toward bridge");
//
//        moveForwardInchesGyro(25, 135, sleepTime);
//
//        turnAbsolutePID(90, Direction.CLOCKWISE, 0.3);
//        sleep(sleepTime, "turn toward parking space");
//
//        // move under bridge
//
//        moveForwardInchesGyro(36 + brickDistance, 90, sleepTime);

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

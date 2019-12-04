package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class SkyScraperIntakeBlue extends SkyScraper{
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        Brick placeholder = Brick.LEFT;

        //move toward bricks
        moveForwardInches(39);
        //turn toward bricks
        turnAbsolutePID(-90, Direction.CLOCKWISE, 0.3);

        moveForwardInches(39, true);
        //move to skystone
        switch (placeholder) {
            case LEFT:
                moveForwardInches(4);
            case MIDDLE:
                moveForwardInches(12);
            case RIGHT:
                moveForwardInches(20);
        }

        //turn toward skystone
        turnAbsolutePID(0, Direction.COUNTERCLOCKWISE, 0.3);

        // intake skystone

        // turn toward bridge
        turnAbsolutePID(-135, Direction.COUNTERCLOCKWISE, 0.3);

        // move toward bridge
        moveForwardInches(25);

        //finish turning toward bridge
        turnAbsolutePID(-90, Direction.CLOCKWISE, 0.3);

        // move under bridge
        moveForwardInches(35);

        //drop stone

        //go back
        moveForwardInches(-35);
        turnAbsolutePID(-135, Direction.COUNTERCLOCKWISE, 0.3);
        moveForwardInches(-25);
        turnAbsolutePID(-90, Direction.CLOCKWISE, 0.3);

        //move to other skystone
        moveForwardInches(-24);

        //turn toward skystone
        turnAbsolutePID(0, Direction.COUNTERCLOCKWISE, 0.3);

        // intake skystone

        // turn toward bridge
        turnAbsolutePID(-135, Direction.COUNTERCLOCKWISE, 0.3);

        // move toward bridge
        moveForwardInches(25);

        //finish turning toward bridge
        turnAbsolutePID(-90, Direction.CLOCKWISE, 0.3);

        // move under bridge
        moveForwardInches(35);

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

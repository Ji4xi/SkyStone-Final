package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//@Autonomous
public class StoneFinderTest extends StoneFinder {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
//        moveInchesGyro(3 * WHEEL_PERIMETER_INCHES, 0, 2000);
//        moveInchesGyro(3 * WHEEL_PERIMETER_INCHES, 180, 2000);
//        for (int i = 60; i < 124; i+=2) {
//            moveInchesGyro(3 * WHEEL_PERIMETER_INCHES, 180, 2000, i, i);
//            moveInchesGyro(3 * WHEEL_PERIMETER_INCHES, 0, 2000, i, i);
//        }

        XLinearInchesGyro(43, Side.RIGHT, 0.30, 15);



    }
}

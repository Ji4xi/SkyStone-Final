package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.Odometry;

public class SkystoneBlue extends MyOdometryOpmode {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        //pick up the skystone


        //AFTER THIS POINT ASSUMes robot starts facing toward skystone wall (0 degrees) after picking up one stone
        //move to foundation (brick1)
//        goToPositionRelative(-22.75*3-18-8, 0, 0.3, 0);
//        goToPositionRelative(0, 4, 0.3, 0);


        //drop
        bottomClaw.setPosition(0.332);
        sleep(200);
        topClaw.setPosition(0.48);
        //raise
        moveClaw(0, 0.707);
        //move to second skystone
        goToPositionRelative(0, -4, 0.3, 0);
        goToPositionRelative(22.75*3+2, 0, 0.3, 0);
        //pick up second brick

        //move to foundation (brick 2)
        goToPositionRelative(-22.75*3-2, 0, 0.3, 0);
        goToPositionRelative(0, 4, 0.3, 0);
        //drop
        bottomClaw.setPosition(0.332);
        sleep(200);
        topClaw.setPosition(0.48);
        //raise
        moveClaw(0, 0.707);
        //

    }
}

package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class FoundationRedSide extends FoundationBlueSide {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
//        sleep(10000);
        moveFoundation();
        //park side
        goToPositionSupreme(-22.75/2 + 4 + 2.3 * TILE_INCH - 12 + 6, 0, maxPwr, 90, 1);
    }

    public void moveFoundation() throws InterruptedException {
        //line up with foundation
        goToPositionSupreme(-22.75/2, 0, maxPwr, 90, 1);
        //move to foundation
        goToPositionSupreme(-22.75/2, -39.25, maxPwr, 90, 1);
        //foundation
        ls.setPosition(0.5);
        rs.setPosition(0.5);
        sleep(100);
        //strafe toward corner, pulling foundation
        goToPositionSupreme(-22.75/2 + 4 - 7, 0, maxPwr, 90, 1);
        ls.setPosition(0);
        rs.setPosition(0);
        sleep(100);
    }
}

package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class StoneFinderTest extends StoneFinder {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        rs.setPosition(0);
        //ls.setPosition(1);
        ls.setPosition(0);
        telemetry.addData("rs_pos", rs.getPosition());
        telemetry.addData("ls_pos", ls.getPosition());
        telemetry.update();
        sleep(2000);
        rs.setPosition(1);
        //ls.setPosition(1);
        ls.setPosition(1);
        telemetry.addData("rs_pos", rs.getPosition());
        telemetry.addData("ls_pos", ls.getPosition());
        telemetry.update();
        sleep(2000);

//        moveInchesPID(3 * WHEEL_PERIMETER_INCHES, 0, 2000);
//        moveInchesPID(3 * WHEEL_PERIMETER_INCHES, 90, 2000);
//        moveInchesPID(3 * WHEEL_PERIMETER_INCHES, 180, 2000);
//        moveInchesPID(3 * WHEEL_PERIMETER_INCHES, 270, 2000);
//        turnPID(90, Direction.CLOCKWISE, 0.6);

    }
}

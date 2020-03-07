package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.Odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.Default.PD;
import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder.GlobalCoordinate;
import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder.NewStoneFinder;
import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder.StoneFinder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.awt.font.NumericShaper;

@Autonomous
public class MyOdometryOpmode extends NewStoneFinder {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        rightIntake.setPower(0.5);
        leftIntake.setPower(0.5);
        turnPID(135, Direction.CLOCKWISE,  maxPwr, 0.019, 0.01, 0.001, 4.1);
        goToPositionSupreme(-5.5, 9, maxPwr, 45,1.2,0.0021, 0.0, 0.000); //d:0.00155
        sleep(300);
        goToPositionSupreme(0, 0, maxPwr, 45,1.2,0.0021, 0.0, 0.000); //d:0.00155

    }

}

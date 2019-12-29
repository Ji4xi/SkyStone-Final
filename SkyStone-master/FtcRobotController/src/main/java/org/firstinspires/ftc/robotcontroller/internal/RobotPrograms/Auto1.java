package org.firstinspires.ftc.robotcontroller.internal.RobotPrograms;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcontroller.internal.Experiments.Michael.MecanumAuto;
import org.firstinspires.ftc.robotcontroller.internal.Experiments.Michael.OneMotorAuto;
import org.firstinspires.ftc.robotcontroller.internal.Experiments.Michael.SkyStoneVuforiaAuto;
import org.firstinspires.ftc.robotcontroller.internal.Experiments.Michael.VuforiaTest;

//@Disabled
@Autonomous
public class Auto1 extends SkyScraper {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        moveForward(3);
    }
}

package org.firstinspires.ftc.robotcontroller.internal.Experiments.Michael;

import android.database.sqlite.SQLiteException;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.RobotPrograms.StoneFinder.NewStoneFinder;

@Autonomous
@Disabled
public class RandomAuto extends NewStoneFinder {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo top;
        Servo bot;
        double topPos = 1, botPos = 1;
        top = hardwareMap.servo.get("top");
        bot = hardwareMap.servo.get("bot");
        top.setDirection(Servo.Direction.REVERSE);
        bot.setDirection(Servo.Direction.REVERSE);
        bot.setPosition(0.65); // 0.26 vertical for bot 0.65 for horizontal bump
        sleep(3000);
        top.setPosition(0.5); //1 ready to grab, 0.35 grab
        // 1 is the about to grab pos for top

        waitForStart();
        top.setPosition(1);
        sleep(2000);
        double count = 1;
//        while (count > 0) {
//            count -= 0.01;
//            sleep(300);
//            top.setPosition(count);
//            telemetry.addData("pos",count);
//            telemetry.update();
//        }
    }
}

package org.firstinspires.ftc.robotcontroller.internal.TestProps;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

public class DrawerSlides extends TeleOpMode {
    DcMotor rd;
    DcMotor ld;
    final double pwrMax = 0.8;
    double currentPwr;
    final int COUNTS_PER_REVOLUTION = 1120;

    @Override
    public void init() {
        rd = hardwareMap.dcMotor.get("rd");
        rd.setDirection(DcMotorSimple.Direction.FORWARD);
        ld = hardwareMap.dcMotor.get("ld");
        ld.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void telemetry() {
        telemetry.addData("Drawer_Pwr", currentPwr);
        telemetry.addData("Drawer_Auto", rd.getCurrentPosition());
    }

    @Override
    public void updateData() {
        updateDrawerSlide();
    }

    public void updateDrawerSlide() {
        currentPwr = pwrMax * gamepad1.right_trigger - pwrMax * gamepad1.left_trigger;
        rd.setPower(Range.clip(currentPwr, -pwrMax, pwrMax));
        ld.setPower(Range.clip(currentPwr, -pwrMax, pwrMax));
        if (gamepad2.left_bumper) {
            rd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rd.setTargetPosition((int) (3 * COUNTS_PER_REVOLUTION));
            ld.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ld.setTargetPosition((int) (3 * COUNTS_PER_REVOLUTION));

            rd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rd.setPower(0.5);
            ld.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ld.setPower(0.5);

            while(rd.isBusy() && ld.isBusy()){}
            rd.setPower(0);
            ld.setPower(0);
        }
    }
}

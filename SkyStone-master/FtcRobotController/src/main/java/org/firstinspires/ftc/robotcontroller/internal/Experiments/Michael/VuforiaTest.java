package org.firstinspires.ftc.robotcontroller.internal.Experiments.Michael;

public class VuforiaTest extends SkyStoneVuforiaAuto{
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        FoundSkyStoneAngle(1000);
        telemetry.addData("dsf","Re");
        telemetry.update();
        sleep(3000);
    }
}

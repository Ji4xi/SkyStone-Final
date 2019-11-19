package org.firstinspires.ftc.robotcontroller.internal.Experiments.Jiaxi;

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.Servo;

        import org.firstinspires.ftc.robotcontroller.internal.Default.TeleOpMode;

@Disabled
@TeleOp
public class ServoHW extends TeleOpMode {
    Servo rs;
    Servo ls;

    final double MAX_SERVO_POS = 1;
    final double MIN_SERVO_POS = 0;
    double currentPosition;

    boolean activate = true;

    @Override
    public void init() {
        rs = hardwareMap.servo.get("rs");
        rs.setDirection(Servo.Direction.REVERSE);
        ls = hardwareMap.servo.get("ls");
        ls.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void stop() {
        super.stop();
    }

    @Override
    public void telemetry() {
        telemetry.addData("rs", currentPosition);
        telemetry.addData("ls", currentPosition);
    }

    @Override
    public void updateData() {
        if (activate) {
            if (gamepad1.a) {
                currentPosition = 0.5;
                rs.setPosition(currentPosition);
                ls.setPosition(currentPosition);
            }
            else if (gamepad1.x) {
                currentPosition = 0.25;
                rs.setPosition(currentPosition);
                ls.setPosition(currentPosition);
            }
        }
        if (!activate) {
            if (gamepad1.dpad_up) {
                currentPosition += 0.01;
                rs.setPosition(currentPosition);
                ls.setPosition(currentPosition);
            }
            if (gamepad1.dpad_down) {
                currentPosition -= 0.01;
                rs.setPosition(currentPosition);
                ls.setPosition(currentPosition);
            }
        }
        //bounds
        if (rs.getPosition() > MAX_SERVO_POS) {
            rs.setPosition(MAX_SERVO_POS);
        }
        if (ls.getPosition() > MAX_SERVO_POS) {
            rs.setPosition(MAX_SERVO_POS);
        }
        if (rs.getPosition() < MIN_SERVO_POS) {
            rs.setPosition(MIN_SERVO_POS);
        }
        if (ls.getPosition() < MIN_SERVO_POS) {
            rs.setPosition(MIN_SERVO_POS);
        }
        if (gamepad1.start) {
            activate = true;
        }
        if (gamepad1.back) {
            activate = false;
        }
    }
}

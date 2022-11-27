package org.firstinspires.ftc.teamcode.PID;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Turn PID", group="")
public class ExampleTurnPID extends OpMode {
    private Drivetrain dt;
    @Override
    public void init() {
        dt = new Drivetrain(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            dt.turnLeft(90);
        }

        if (gamepad1.b) {
            dt.turnRight(90);
        }

        dt.update();
        dt.telemetry(telemetry);
    }

}

package org.firstinspires.ftc.teamcode.ExampleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.TestBot;

@TeleOp(name = "Example blocking opmode", group = "")
public class ExampleLinearTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Init a robot object
        TestBot robot = new TestBot(this);

        // Wait for opmode start
        robot.runUntil(this::opModeIsActive);

        while (opModeIsActive() && !isStopRequested()) {

            robot.drivetrain.fieldCentric(gamepad1);

            robot.update();
        }
    }
}

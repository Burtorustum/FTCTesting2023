package org.firstinspires.ftc.teamcode.ExampleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.TestBot;

@Autonomous(name = "Example blocking opmode", group = "")
public class ExampleLinearAuton extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Init a robot object
        TestBot robot = new TestBot(this);

        // Wait for opmode start
        robot.runUntil(this::opModeIsActive);

        // Turn 135d left
        robot.drivetrain.turn(135);
        robot.runUntil(robot.drivetrain::atTargetAngle);

        // Intake until cone picked up
        //robot.intake.intake();
        //robot.runUntil(robot.intake::coneDetected);

        // Turn 90d right
        //robot.intake.idle();
        robot.drivetrain.turn(-90);
        robot.runUntil(robot.drivetrain::atTargetAngle);

        // Do nothing for 3 seconds
        robot.runForTime(3000);

        // Turn 180d
        robot.drivetrain.turn(0);
        robot.runUntil(robot.drivetrain::atTargetAngle);

        // Wait for stop
        robot.runUntilStop();
    }
}

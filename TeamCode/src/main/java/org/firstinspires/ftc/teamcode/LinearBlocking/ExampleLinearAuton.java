package org.firstinspires.ftc.teamcode.LinearBlocking;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Example blocking opmode", group = "")
public class ExampleLinearAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Init a robot object
        Robot robot = new Robot(this);

        // Wait for opmode start
        robot.runUntil(this::opModeIsActive);

        // Turn 135d left
        robot.drivetrain.turnLeft(135);
        robot.runUntil(robot.drivetrain::atTargetAngle);

        // Intake until cone picked up
        robot.intake.intake();
        robot.runUntil(robot.intake::coneDetected);

        // Turn 90d right
        robot.intake.idle();
        robot.drivetrain.turnRight(90);
        robot.runUntil(robot.drivetrain::atTargetAngle);

        // Do nothing for 3 seconds
        robot.runForTime(3000);

        // Turn 180d
        robot.drivetrain.turnRight(180);
        robot.runUntil(robot.drivetrain::atTargetAngle);

        // Wait for stop
        robot.runUntilStop();
    }
}

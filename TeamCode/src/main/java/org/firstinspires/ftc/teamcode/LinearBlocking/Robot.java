package org.firstinspires.ftc.teamcode.LinearBlocking;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PID.Drivetrain;

public class Robot {

    public final LinearOpMode opMode;

    public final Drivetrain drivetrain;
    public final Intake intake;

    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        this.drivetrain = new Drivetrain(opMode.hardwareMap);
        this.intake = new Intake(opMode.hardwareMap);
    }

    public void update() {
        this.drivetrain.update();
        this.intake.update();
    }

    public interface Target {
        boolean reached();
    }

    public void runUntil(Target target) {
        while (!opMode.isStopRequested() && !target.reached()) {
            update();
        }
    }

    public void runUntilStop() {
        this.runUntil(() -> false);
    }

    public void runForTime(long millis) {
        long end = System.currentTimeMillis() + millis;
        runUntil(() -> System.currentTimeMillis() >= end);
    }
}

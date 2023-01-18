package org.firstinspires.ftc.teamcode.Robot.Structure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public abstract class Robot {

    protected final LinearOpMode opMode;
    protected final HardwareMap hwMap;
    protected final Telemetry telemetry;

    private final List<Subsystem> subsystems;

    protected Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hwMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;

        this.subsystems = new ArrayList<>();
    }

    public void update() {
        for (Subsystem sys : this.subsystems) {
            sys.update(telemetry);
        }
        telemetry.update();
    }

    protected void registerSubsystem(Subsystem subsystem) {
        if (!this.subsystems.contains(subsystem)) {
            this.subsystems.add(subsystem);
        }
    }

    public interface Target {
        boolean reached();
    }

    public void runUntil(Target target) {
        while (!opMode.isStopRequested() && !target.reached()) {
            this.update();
        }
    }

    public void runForTime(long millis) {
        long end = System.currentTimeMillis() + millis;
        runUntil(() -> System.currentTimeMillis() >= end);
    }

    public void runUntilStop() {
        this.runUntil(() -> false);
    }
}

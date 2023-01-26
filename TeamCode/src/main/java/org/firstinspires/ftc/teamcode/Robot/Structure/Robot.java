package org.firstinspires.ftc.teamcode.Robot.Structure;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Robot {

  protected final LinearOpMode opMode;
  protected final HardwareMap hwMap;
  protected final Telemetry telemetry;

  private final List<Subsystem> subsystems;

  protected Robot(LinearOpMode opMode) {
    this.opMode = opMode;
    this.hwMap = opMode.hardwareMap;
    this.telemetry = opMode.telemetry;

    // Setup bulk encoder reads using AUTO caching mode
    List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);
    for (LynxModule module : allHubs) {
      module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }

    this.subsystems = new ArrayList<>();
  }

  public void start() {
    for (Subsystem sys : this.subsystems) {
      sys.start();
    }
  }

  public void update() {
    for (Subsystem sys : this.subsystems) {
      sys.update(telemetry);
    }
    telemetry.update();
  }

  public void registerSubsystem(Subsystem subsystem) {
    if (!this.subsystems.contains(subsystem)) {
      this.subsystems.add(subsystem);
    }
  }

  public void deregisterSubsystem(Subsystem subsystem) {
    this.subsystems.remove(subsystem);
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

  public interface Target {

    boolean reached();
  }
}
package org.firstinspires.ftc.teamcode.Robot.Structure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Subsystem {

  protected final HardwareMap hwMap;

  protected Subsystem(LinearOpMode opMode) {
    hwMap = opMode.hardwareMap;
  }

  public void start() {
  }

  public abstract void update(Telemetry telemetry);
}
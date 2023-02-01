package org.firstinspires.ftc.teamcode.Robot.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class RGBSignalSleeveDetector extends Vision {

  private final RGBSignalSleevePipeline pipeline = new RGBSignalSleevePipeline();

  public RGBSignalSleeveDetector(LinearOpMode opMode) {
    super(opMode);
    setPipelineAndOpen(pipeline);
  }

  public PARK_LOCATION getDetermination() {
    return this.pipeline.determination;
  }
}
package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.Sensors.LVMaxSonarEZ;
import org.firstinspires.ftc.teamcode.Robot.Structure.Robot;
import org.firstinspires.ftc.teamcode.Robot.Structure.Subsystem;

public class TestBot extends Robot {

    // Sensors multiple subsystems require
    public final IMU imu;
    public final LVMaxSonarEZ ultrasonic;
    public final Rev2mDistanceSensor tof;

    // Subsystems
    public final MecanumDrivetrain drivetrain;

    public TestBot(LinearOpMode opMode) {
        super(opMode);

        this.imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));

        this.ultrasonic = new LVMaxSonarEZ(hwMap.analogInput.get("us"));
        this.tof = hwMap.get(Rev2mDistanceSensor.class, "tof");

        this.drivetrain = new MecanumDrivetrain(opMode, imu);
        this.registerSubsystem(this.drivetrain);

        this.registerSubsystem(new Subsystem(opMode) {
            @Override
            public void update(Telemetry telemetry) {
                telemetry.addData("ToF | Distance", tof.getDistance(DistanceUnit.INCH));
                telemetry.addData("MaxSonarEZ | Distance", ultrasonic.getDistance(DistanceUnit.INCH));
            }
        });
    }
}
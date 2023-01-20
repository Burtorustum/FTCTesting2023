package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Robot.Structure.Robot;

public class TestBot extends Robot {

    // Sensors multiple subsystems require
    public final IMU imu;

    // Subsystems
    public final MecanumDrivetrain drivetrain;
    //public final Intake intake;

    public TestBot(LinearOpMode opMode) {
        super(opMode);

        this.imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));

        this.drivetrain = new MecanumDrivetrain(opMode, imu);
        this.registerSubsystem(this.drivetrain);

        //this.intake = new Intake(opMode);
        //this.registerSubsystem(this.intake);
    }
}
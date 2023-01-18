package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Structure.Robot;
import org.firstinspires.ftc.teamcode.Robot.Vision.ConeDetector;

public class TestBot extends Robot {

    public final Drivetrain drivetrain;
    public final ConeDetector coneDetector;

    public TestBot(LinearOpMode opMode) {
        super(opMode);

        drivetrain = new Drivetrain(opMode);
        this.registerSubsystem(drivetrain);

        coneDetector = new ConeDetector(opMode);
        this.registerSubsystem(coneDetector);
    }
}

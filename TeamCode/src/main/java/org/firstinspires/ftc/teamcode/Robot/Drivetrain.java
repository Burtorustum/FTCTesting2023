package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxResetMotorEncoderCommand;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PID.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.PID.PIDFController;
import org.firstinspires.ftc.teamcode.Robot.Structure.Subsystem;

import java.util.List;

public class Drivetrain extends Subsystem {
    private final DcMotorEx fl, fr, bl, br;
    private double flPow, frPow, blPow, brPow;
    private final IMU imu;

    private final PIDFController turnController =
            new PIDFController(new PIDFCoefficients(2.2, 0.001, 0.07));
    private int angleTarget;

    private State mode = State.IDLE;
    enum State {
        IDLE, TURN, DIRECT_CONTROL
    }

    public Drivetrain(LinearOpMode opMode) {
        super(opMode);

        fl = hwMap.get(DcMotorEx.class, "fl");
        fr = hwMap.get(DcMotorEx.class, "fr");
        bl = hwMap.get(DcMotorEx.class, "bl");
        br = hwMap.get(DcMotorEx.class, "br");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));
    }

    public void update(Telemetry telemetry) {
        telemetry.addData("Drivetrain | Mode", this.mode.name());
        switch (mode) {
            case IDLE:
                flPow = 0;
                frPow = 0;
                blPow = 0;
                brPow = 0;
                break;

            case TURN:
                double botAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double error = -AngleUnit.normalizeRadians(AngleUnit.RADIANS.fromUnit(AngleUnit.DEGREES, this.angleTarget) - botAngle);

                double output = turnController.update(this.angleTarget, error);
                flPow = output;
                blPow = output;
                frPow = -output;
                brPow = -output;

                // TODO: is this the correct value?
                if (output < 0.01) {
                    this.mode = State.IDLE;
                }

                telemetry.addData("Drivetrain | Angle Error", Math.toDegrees(error));
                telemetry.addData("Drivetrain | Output", output);
                break;

            case DIRECT_CONTROL:
                break;
        }

        fl.setPower(flPow);
        fr.setPower(frPow);
        bl.setPower(blPow);
        br.setPower(brPow);
    }

    public void turn(int targetDegrees) {
        this.angleTarget = targetDegrees;
        this.mode = State.TURN;
    }

    public boolean atTargetAngle() {
        return this.mode == State.IDLE;
    }

    // TODO: test
    public void zeroMotorEncoders() throws LynxNackException, InterruptedException {
        List<LynxModule> hubs = this.hwMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            if (hub.isParent()) {
                new LynxResetMotorEncoderCommand(hubs.get(0), fl.getPortNumber()).send();
                new LynxResetMotorEncoderCommand(hubs.get(0), bl.getPortNumber()).send();
            } else {
                new LynxResetMotorEncoderCommand(hubs.get(0), fr.getPortNumber()).send();
                new LynxResetMotorEncoderCommand(hubs.get(0), br.getPortNumber()).send();
            }
        }
    }

    // TODO: test
    public boolean isEncoderResetFinished() {
        return fl.getCurrentPosition() == 0
                && fr.getCurrentPosition() == 0
                && bl.getCurrentPosition() == 0
                && br.getCurrentPosition() == 0;
    }

    public void directControl(double flPow, double frPow, double blPow, double brPow) {
        this.mode = State.DIRECT_CONTROL;
        this.flPow = flPow;
        this.frPow = frPow;
        this.blPow = blPow;
        this.brPow = brPow;
    }

    public void fieldCentric(Gamepad gamepad) {
        double y = -gamepad.left_stick_y; // Remember, this is reversed!
        double x = gamepad.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad.right_stick_x;

        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double flPow = (rotY + rotX + rx) / denominator;
        double frPow = (rotY - rotX - rx) / denominator;
        double blPow = (rotY - rotX + rx) / denominator;
        double brPow = (rotY + rotX - rx) / denominator;

        this.directControl(flPow, frPow, blPow, brPow);
    }
}

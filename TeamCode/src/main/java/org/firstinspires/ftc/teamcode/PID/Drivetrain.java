package org.firstinspires.ftc.teamcode.PID;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain {
    private final HardwareMap hwMap;

    private final DcMotor fl, fr, bl, br;
    private final BNO055IMU imu;

    private final PIDController turnPID;
    private double globalAngle;
    private double lastAngle = 0;
    private double errorAngle;
    private double angleTarget;
    private double correction;

    private State mode = State.STOP;

    enum State {
        STOP, TURN, STRAIGHT; // ....
    }

    public Drivetrain(HardwareMap hwMap) {
        this.hwMap = hwMap;

        this.fl = hwMap.dcMotor.get("fl");
        this.fr = hwMap.dcMotor.get("fr");
        this.bl = hwMap.dcMotor.get("bl");
        this.br = hwMap.dcMotor.get("br");

        this.fl.setDirection(DcMotorSimple.Direction.FORWARD);
        this.fr.setDirection(DcMotorSimple.Direction.FORWARD);
        this.bl.setDirection(DcMotorSimple.Direction.REVERSE);
        this.br.setDirection(DcMotorSimple.Direction.REVERSE);

        this.imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        this.turnPID = new PIDController(new PIDCoefficients(0.025, 0.0045, 0.003));
        this.turnPID.setOutputBounds(-1, 1);
    }

    public void update() {
        switch (mode) {
            case STOP:

                fl.setPower(0);
                fr.setPower(0);
                bl.setPower(0);
                br.setPower(0);
                break;

            case TURN:

                errorAngle = angleTarget - getAngle();

                if (atTargetAngle()) {
                    mode = State.STOP;
                    update();
                } else {
                    correction = turnPID.update(errorAngle);

                    fl.setPower(correction);
                    fr.setPower(correction);
                    bl.setPower(-correction);
                    br.setPower(-correction);
                }
                break;

            case STRAIGHT:
                // This example doesn't include driving to a distance, but that is implementable in
                // same way turns are above.
                // See https://github.com/acmerobotics/freight-frenzy/blob/main/src/com/acmerobotics/frieghtFrenzy/robot/Drive.java

                break;
        }
    }

    public void prepareMotors() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getAngle() {
        double currentAngle = imu.getAngularOrientation().firstAngle * 180 / Math.PI;
        double deltaAngle = currentAngle - lastAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngle = currentAngle;

        return globalAngle;
    }

    public void resetAngle() {
        globalAngle = 0;
    }

    public boolean atTargetAngle() {
        return (errorAngle > -0.5) && (errorAngle < 0.5);
    }

    public void turnLeft(double angleFromRobot) {
        turnPID.reset();
        resetAngle();

        this.mode = State.TURN;
        this.angleTarget = angleFromRobot;

        prepareMotors();
    }

    public void turnRight(double angleFromRobot) {
        turnPID.reset();
        resetAngle();

        this.mode = State.TURN;
        this.angleTarget = -angleFromRobot;

        prepareMotors();
    }

    public void telemetry(Telemetry telemetryData) {
        telemetryData.addData("Auto Mode", mode);
        telemetryData.addData("Angle Error", errorAngle);
        telemetryData.addData("Angle Target", angleTarget);
        telemetryData.addData("Correction", correction);
        telemetryData.addData("Last Angle", lastAngle);

        telemetryData.update();
    }
}

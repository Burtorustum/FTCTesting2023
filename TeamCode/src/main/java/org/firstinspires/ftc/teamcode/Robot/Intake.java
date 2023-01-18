package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Structure.Subsystem;

public class Intake extends Subsystem {
    private final DcMotor motor;
    private final RevColorSensorV3 color;
    private State mode = State.IDLE;

    public Intake(LinearOpMode opMode) {
        super(opMode);
        this.motor = hwMap.dcMotor.get("intake");
        this.color = hwMap.get(RevColorSensorV3.class, "color");
    }

    enum State {
        IDLE, INTAKE, OUTTAKE;
    }

    public void update(Telemetry telemetry) {
        switch (mode) {
            case IDLE:
                this.motor.setPower(0);
                break;

            case INTAKE:
                this.motor.setPower(1);
                break;

            case OUTTAKE:
                this.motor.setPower(-1);
                break;
        }
    }

    public void idle() {
        this.mode = State.IDLE;
    }

    public void intake() {
        this.mode = State.INTAKE;
    }

    public void outtake() {
        this.mode = State.OUTTAKE;
    }

    public boolean coneDetected() {
        NormalizedRGBA colorVals = color.getNormalizedColors();
        return colorVals.red > 100 || colorVals.blue > 100;
    }
}

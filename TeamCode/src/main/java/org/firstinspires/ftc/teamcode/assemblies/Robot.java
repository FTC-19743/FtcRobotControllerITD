package org.firstinspires.ftc.teamcode.assemblies;

import static org.firstinspires.ftc.teamcode.libs.teamUtil.Alliance.RED;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

public class Robot {
    public BNO055IMU imu;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public BasicDrive drive;
    public Output output;
    public Outtake outtake;
    public Intake intake;

    //public Intake intake;
    //public Output output;
    //public Lift lift;
    //public PixelRelease releaser;
    //public Launcher launcher;

    public double a,b,c,d;

    public Robot() {
        telemetry = teamUtil.theOpMode.telemetry;
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        drive = new BasicDrive();
        outtake = new Outtake();
        intake = new Intake();
        output = new Output();


    }

    public void initialize() {
        outtake.initalize();
        drive.initalize();
        output.initalize();
        intake.initialize();
        intake.initCV();


    }

    public void outputTelemetry() {
        drive.driveMotorTelemetry();
        intake.intakeTelemetry();
    }

    public void calibrate() {
        output.calibrate();
        outtake.calibrate();
        intake.calibrate();
    }

    public int fieldSide() { // helper method that returns heading out towards the field
        return teamUtil.alliance == RED ? 90 : 270;
    }

    public int driverSide() { // helper method that returns heading backs towards drivers
        return teamUtil.alliance == RED ? 270 : 90;
    }

    public int audienceSide() {
        return 180;
    }

    public int scoreSide() {
        return 0;
    }


}


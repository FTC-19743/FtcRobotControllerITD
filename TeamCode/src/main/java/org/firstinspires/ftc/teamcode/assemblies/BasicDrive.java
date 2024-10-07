package org.firstinspires.ftc.teamcode.assemblies;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

@Config // Makes Static data members available in Dashboard
public class BasicDrive {

////////////////////////////////////////////////////////////////////////////////////////////////
//
//    Drive class for mecanum wheels
//
////////////////////////////////////////////////////////////////////////////////////////////////

    // constants
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public BNO055IMU imu; //This variable is the imu
    public static double HEADING_OFFSET; // offset between IMU heading and field
    public double lastVelocity;
    public boolean holdingHeading = false;
    public double heldHeading = 0;
    public AtomicBoolean movingAutonomously = new AtomicBoolean(false); // true under autonomous operation in teleop
    public AtomicBoolean manualInterrupt = new AtomicBoolean(true); // used to interrupt autonomous operations with manual driving control

    public DcMotorEx fl = null;
    public DcMotorEx fr = null;
    public DcMotorEx bl = null;
    public DcMotorEx br = null;

    public DcMotorEx strafeEncoder; // TODO: Redo with Octoquad?
    public DcMotorEx forwardEncoder;


    static public double COUNTS_PER_MOTOR_REV = 537.7;    // GoBilda 5202 312 RPM
    static public double COUNTS_PER_CENTIMETER = 9.625; //Was 12.855 Used with 435 RPM
    static public double COUNTS_PER_CENTIMETER_312 = 17.923; // Used with 312 RPM
    static public double TICS_PER_CM_STRAFE_ENCODER = 130;
    static public double TICS_PER_CM_STRAIGHT_ENCODER = 735;

    public double TILE_CENTER_TO_CENTER = 60.325; // tile width in Cms

    static public double MIN_START_VELOCITY = 300; //calibrated with 435s
    static public double MIN_END_VELOCITY = 400; //calibrated with 435s
    static public double MAX_ACCELERATION = 22; //calibrated with 435s
    static public double MAX_DECELERATION = 1.0; //calibrated with 435s
    static public double MAX_STRAIGHT_ACCELERATION = 1.0; //calibrated with 435s
    static public double MAX_STRAIGHT_DECELERATION = 0.02; //calibrated with 435s
    static public double MIN_STRAFE_START_VELOCITY = 800; //calibrated with 435s
    static public double MIN_STRAFE_END_VELOCITY = 400; //calibrated with 435s
    static public double MAX_STRAFE_DECELERATION = .15; //calibrated with 435s
    static public double MAX_STRAFE_ACCELERATION = 2; // tentaive
    static public double MAX_VELOCITY = 2350; // Was 2680
    static public double MAX_VELOCITY_STRAFE = 2050; // Added with new motors
    static public double ROTATION_ADJUST_FACTOR = 0.04;
    static public double SIDE_VECTOR_COEFFICIENT = .92;
    static public double FORWARD_VECTOR_COEFFICIENT = 1.08;
    static public double SPIN_DECEL_THRESHOLD_DEGREES = 60;
    static public double SPIN_DRIFT_DEGREES = 1;
    static public double SPIN_CRAWL_SPEED = 200;
    static public double SPIN_CRAWL_DEGREES = 30;
    static public boolean details = true;
    public double CMS_PER_INCH = 2.54;
    static float STRAIGHT_HEADING_DECLINATION = .09f; // convert strafe encoder error into heading declination
    static float STRAIGHT_MAX_DECLINATION = 27f; // don't veer off of straight more than this number of degrees
    static float STRAFE_HEADING_DECLINATION = .09f; // convert strafe encoder error into heading declination
    static float STRAFE_MAX_DECLINATION = 27f; // don't veer off of straight more than this number of degrees

    /************************************************************************************************************/
    // Initialization
    public BasicDrive() {
        teamUtil.log("Constructing BasicDrive");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public interface ActionCallback{
        public void action();
        //must return immediatley or launch a thread
    }

    public void initalize() {
        teamUtil.log("Initializing BasicDrive");
        //Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //output=theOutput;
        fl = hardwareMap.get(DcMotorEx.class, "flm");
        fr = hardwareMap.get(DcMotorEx.class, "frm");
        bl = hardwareMap.get(DcMotorEx.class, "blm");
        br = hardwareMap.get(DcMotorEx.class, "brm");

        //forwardEncoder = hardwareMap.get(DcMotorEx.class, "leftForwardEncoder");
        //strafeEncoder = hardwareMap.get(DcMotorEx.class, "strafeEncoder");


        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);



        //forwardEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        //bl.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //These are the parameters that the imu uses in the code to name and keep track of the data
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
        setMotorsBrake();
        teamUtil.log("Initializing Drive - FINISHED");
    }

    /************************************************************************************************************/
    // Telemetry

    public void driveMotorTelemetry() {
        telemetry.addData("Motors ", "flm:%d frm:%d blm:%d brm:%d",
                fl.getCurrentPosition(), fr.getCurrentPosition(), bl.getCurrentPosition(), br.getCurrentPosition());
        //telemetry.addData("Odometry ", "strafe:%d strafe Cms:%f forward:%d forward Cms:%f heading:%f ",
                //strafeEncoder.getCurrentPosition(), strafeEncoder.getCurrentPosition()/TICS_PER_CM_STRAFE_ENCODER, forwardEncoder.getCurrentPosition(), forwardEncoder.getCurrentPosition()/TICS_PER_CM_STRAIGHT_ENCODER, getHeading());
        telemetry.addData("Heading ", "%f ",
                 getHeading());
    }



    public void logMotorPositions() {
        teamUtil.log("fr: " + fr.getCurrentPosition());
        teamUtil.log("fl: " + fl.getCurrentPosition());
        teamUtil.log("br: " + br.getCurrentPosition());
        teamUtil.log("bl: " + bl.getCurrentPosition());
    }

    /************************************************************************************************************/
    //   Basic Motor Operations

    public void setBulkReadOff() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        }
    }

    public void setBulkReadAuto() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void runMotors(double velocity) {
        lastVelocity = velocity;
        fl.setVelocity(velocity);
        fr.setVelocity(velocity);
        bl.setVelocity(velocity);
        br.setVelocity(velocity);
    }

    public void setMotorsBrake() {
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setMotorsFloat() {
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setMotorPower(double power) {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }

    public void setMotorsActiveBrake() {
        // hold a position using setPosition
        int flPosition = fl.getCurrentPosition();
        int frPosition = fr.getCurrentPosition();
        int blPosition = bl.getCurrentPosition();
        int brPosition = br.getCurrentPosition();
        fl.setTargetPosition(flPosition);
        fr.setTargetPosition(frPosition);
        bl.setTargetPosition(blPosition);
        br.setTargetPosition(brPosition);

        setMotorsRunToPosition();
        setMotorPower(0.5);
    }

    public void stopMotors() {
        teamUtil.log("Stopping Motors");
        lastVelocity = 0;
        fl.setVelocity(0);
        fr.setVelocity(0);
        bl.setVelocity(0);
        br.setVelocity(0);
    }

    public void setMotorVelocities(double flV, double frV, double blV, double brV) {
        fl.setVelocity(flV);
        fr.setVelocity(frV);
        bl.setVelocity(blV);
        br.setVelocity(brV);
    }

    public void setMotorsRunToPosition() {
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void setMotorsRunWithoutEncoder() {
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void resetAllDriveEncoders() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setMotorsWithEncoder() {
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /************************************************************************************************************/
    //   Holonomic Motor Operations

    public void driveMotorsHeadings(double driveHeading, double robotHeading, double velocity) {
        // move robot based on a heading to face and a heading to drive to
        double flV, frV, blV, brV;
        double x, y, scale;

        // Determine how much adjustment for rotational drift
        double headingError = getHeadingError(robotHeading); // Difference between desired and actual robot heading
        //double headingError = Math.max(-45.0, Math.min(getHeadingError(robotHeading), 45.0)); // clip this to 45 degrees in either direction to control rate of spin
        double rotationAdjust = ROTATION_ADJUST_FACTOR * velocity * headingError; // scale based on velocity AND amount of rotational error

        // Covert heading to cartesian on the unit circle and scale so largest value is 1
        // This is essentially creating joystick values from the heading
        // driveHeading is relative to robot at this point since the wheels are relative to robot!
        x = Math.cos(Math.toRadians(driveHeading + 90)); // + 90 cause forward is 0...
        y = Math.sin(Math.toRadians(driveHeading + 90));
        scale = 1 / Math.max(Math.abs(x), Math.abs(y));
        x = x * scale;
        y = y * scale;

        // Clip to motor power range
        flV = Math.max(-1.0, Math.min(x + y, 1.0)) * velocity;
        brV = flV;
        frV = Math.max(-1.0, Math.min(y - x, 1.0)) * velocity;
        blV = frV;

        // Adjust for rotational drift
        flV = flV - rotationAdjust;
        brV = brV + rotationAdjust;
        frV = frV + rotationAdjust;
        blV = blV - rotationAdjust;

        // Update the motors
        setMotorVelocities(flV, frV, blV, brV);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Set the velocity of all 4 motors based on a driveHeading RELATIVE TO FIELD and provided velocity
    // Will rotate robot as needed to achieve and hold robotHeading RELATIVE TO FIELD by moving to a set target
    public void driveMotorsHeadingsFR(double driveHeading, double robotHeading, double velocity) {
        double RRDriveHeading = getHeadingError(driveHeading);
        driveMotorsHeadings(RRDriveHeading, robotHeading, velocity);
    }

    public double getHeadingError(double targetAngle) {
        // distance from target
        double robotError;

        // calculate heading error in -179 to +180 range  (
        robotError = targetAngle - getHeading();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getRawHeading() {
        Orientation anglesCurrent = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (anglesCurrent.firstAngle);

    }

    public double getHeading() {
        // stows an offset to change the range and set the heading
        return adjustAngle(getRawHeading() - HEADING_OFFSET);
    }

    // Make the current heading 0.
    public void resetHeading() {
        HEADING_OFFSET = getRawHeading();
        heldHeading = getHeading();
    }

    //Make the current heading to specified number
    public void setHeading(int heading) {
        HEADING_OFFSET = getRawHeading() - heading;
        heldHeading = getHeading();
    }

    public double adjustAngle(double angle) {
        //assuming imu runs from [0, 360] and angle is added/substracted, adjust it to expected reading
        while (angle >= 360) {
            angle -= 360;
        }
        while (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    class MotorData { // a helper class to allow for faster access to hub data
        int eFL, eFR, eBL, eBR; // encoder values of each motor
    }

    public void getDriveMotorData(MotorData data) {
        // update current motor positions
        data.eFL = fl.getCurrentPosition();
        data.eFR = fr.getCurrentPosition();
        data.eBL = bl.getCurrentPosition();
        data.eBR = br.getCurrentPosition();
    }



    /************************************************************************************************************/
    // Methods to drive based on motor encoders

    public int getEncoderDistance(MotorData initialPositions) {
        // use trig to find the hypotenuse of the side and front distances
        MotorData currentPositions = new MotorData();
        getDriveMotorData(currentPositions);

        // Calculate the vector along the forward/backward axis
        int ForwardVector = (currentPositions.eFL - initialPositions.eFL)
                + (currentPositions.eFR - initialPositions.eFR)
                + (currentPositions.eBL - initialPositions.eBL)
                + (currentPositions.eBR - initialPositions.eBR);
        // Calculate the vector along the left/right axis
        int SideVector = (currentPositions.eFL - initialPositions.eFL)
                + (currentPositions.eBR - initialPositions.eBR)
                - (currentPositions.eFR - initialPositions.eFR)
                - (currentPositions.eBL - initialPositions.eBL);

        // Return the hypotenuse of the two vectors
        // divide by 4 to account for the math that adds all 4 motor encoders
        return (int) (Math.sqrt(Math.pow((ForwardVector * FORWARD_VECTOR_COEFFICIENT), 2) + (Math.pow((SideVector * SIDE_VECTOR_COEFFICIENT), 2))) / 4);

    }

    public void moveCm(double centimeters, double driveHeading) {
        // simplified parameters of moveCm
        moveCm(MAX_VELOCITY, centimeters, driveHeading, getHeading(), MIN_END_VELOCITY);
    }

    public void moveCm(double centimeters, double driveHeading, double endVelocity) {
        // simplified parameters of moveCm
        moveCm(MAX_VELOCITY, centimeters, driveHeading, getHeading(), endVelocity);
    }

    public void moveCm(double maxVelocity, double centimeters, double driveHeading, double endVelocity) {
        // simplified parameters of moveCm
        moveCm(maxVelocity, centimeters, driveHeading, getHeading(), endVelocity);
    }

    public void moveCm(double maxVelocity, double centimeters, double driveHeading, double robotHeading, double endVelocity) {
        // move based on a cruise, end, and max velocity, distance, and headings
        teamUtil.log("MoveCM cms:" + centimeters + " driveH:" + driveHeading + " robotH:" + robotHeading + " MaxV:" + maxVelocity + " EndV:" + endVelocity);

        details = false;
        MotorData data = new MotorData();
        getDriveMotorData(data);

        double velocityChangeNeededAccel;
        double velocityChangeNeededDecel;
        if (endVelocity < MIN_END_VELOCITY) {
            endVelocity = MIN_END_VELOCITY; // simplify by setting min end to 0
        }
        // tics^2/s
        if (lastVelocity == 0) { // at a stop
            velocityChangeNeededAccel = maxVelocity - MIN_START_VELOCITY;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        } else { // already moving
            velocityChangeNeededAccel = maxVelocity - lastVelocity;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        }
        setMotorsWithEncoder();
        // all are measured in tics
        double totalTics = centimeters * COUNTS_PER_CENTIMETER;
        double accelerationDistance = Math.abs(velocityChangeNeededAccel / MAX_ACCELERATION);
        double decelerationDistance = Math.abs(velocityChangeNeededDecel / MAX_DECELERATION);
        double postCruiseTargetDistance = totalTics - decelerationDistance;
        if (postCruiseTargetDistance < 0) { // need to cut off the curve
            double percentageToRemoveAccel = accelerationDistance / (accelerationDistance + decelerationDistance);
            accelerationDistance += postCruiseTargetDistance * percentageToRemoveAccel;
            decelerationDistance += postCruiseTargetDistance * percentageToRemoveAccel;
            postCruiseTargetDistance = 0;
        }
        double distance = 0;
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("Total tics: " + totalTics);
            teamUtil.log("Acceleration distance: " + accelerationDistance);
            teamUtil.log("Deceleration distance: " + decelerationDistance);
            teamUtil.log("Post Cruise distance: " + postCruiseTargetDistance);
        }
//acceleration
        while (distance < accelerationDistance) {
            distance = getEncoderDistance(data);
            if (lastVelocity == 0) {
                driveMotorsHeadingsFR(driveHeading, robotHeading, MAX_ACCELERATION * distance + MIN_START_VELOCITY); // velocity moves by distance
            } else {
                driveMotorsHeadingsFR(driveHeading, robotHeading, MAX_ACCELERATION * distance + lastVelocity);
            }
        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after acceleration: " + distance);
        }
//cruise
        while (distance < postCruiseTargetDistance) {

            distance = getEncoderDistance(data);
            driveMotorsHeadingsFR(driveHeading, robotHeading, maxVelocity); // constant
        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after cruise: " + distance);
        }


//deceleration
        double startDecelerationDistance = distance;
        double ticsUntilEnd = totalTics - distance;
        while (distance < totalTics) {
            distance = getEncoderDistance(data);
            ticsUntilEnd = totalTics - distance;
            driveMotorsHeadingsFR(driveHeading, robotHeading, MAX_DECELERATION * ticsUntilEnd + endVelocity); // lowers through tics to end

        }
        if (details) {
            teamUtil.log("distance after deceleration: " + distance);
        }
        if (endVelocity <= MIN_END_VELOCITY) {
            stopMotors();
            if (details) {
                teamUtil.log("Went below or was min end velocity");
            }
        }
        lastVelocity = endVelocity;
        teamUtil.log("MoveCM--Finished");

    }


    /************************************************************************************************************/
    // OLD Methods to drive based on odometry pods
    /*
    public boolean strafeToEncoder(double driveHeading, double robotHeading, double velocity, double targetEncoderValue, long timeout) {
        // strafe to a strafe encoder value
        long timeOutTime = System.currentTimeMillis() + timeout;
        teamUtil.log("strafeToEncoder: Current: " + strafeEncoder.getCurrentPosition() + " Target: " + targetEncoderValue);
        float driftCms = 1;
        double realTarget = targetEncoderValue + (driveHeading < 180? -1:1)*driftCms*TICS_PER_CM_STRAFE_ENCODER;
        if (driveHeading<180) {
            while (strafeEncoder.getCurrentPosition() < realTarget && teamUtil.keepGoing(timeOutTime)) {
                driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);
            }
        }
        else{
            while (strafeEncoder.getCurrentPosition() > realTarget && teamUtil.keepGoing(timeOutTime)) {
                driveMotorsHeadingsFR(driveHeading, robotHeading, velocity);
            }
        }
        lastVelocity=velocity;
        if (System.currentTimeMillis() > timeOutTime) {
            teamUtil.log("strafeToEncoder - TIMED OUT!");
            return false;
        } else {
            teamUtil.log("strafeToEncoder - FINISHED : Current: " + strafeEncoder.getCurrentPosition());
            return true;
        }
    }

     */

    public boolean strafeToEncoderWithDecel(double driveHeading, double robotHeading, double velocity, double targetEncoderValue, double endVelocity, double decelK, long timeout) {
        long timeOutTime = System.currentTimeMillis() + timeout;
        teamUtil.log("strafeToEncoder: Current: " + strafeEncoder.getCurrentPosition() + " Target: " + targetEncoderValue);
        float driftCms = 1;
        double realTarget = targetEncoderValue + (driveHeading < 180? -1:1)*driftCms*TICS_PER_CM_STRAFE_ENCODER;
        double strafeCmsToGo;
        double liveVelocity;
        if(endVelocity<MIN_END_VELOCITY){
            endVelocity = MIN_END_VELOCITY;
        }

        if (driveHeading<180) {
            while (strafeEncoder.getCurrentPosition() < realTarget && teamUtil.keepGoing(timeOutTime)) {
                strafeCmsToGo = Math.abs(targetEncoderValue-strafeEncoder.getCurrentPosition())/TICS_PER_CM_STRAFE_ENCODER;
                liveVelocity = Math.min(velocity, endVelocity+decelK* COUNTS_PER_CENTIMETER * strafeCmsToGo);
                driveMotorsHeadingsFR(driveHeading, robotHeading, liveVelocity);
            }
        }
        else{
            while (strafeEncoder.getCurrentPosition() > realTarget && teamUtil.keepGoing(timeOutTime)) {
                strafeCmsToGo = Math.abs(targetEncoderValue-strafeEncoder.getCurrentPosition())/TICS_PER_CM_STRAFE_ENCODER;
                liveVelocity = Math.min(velocity, endVelocity+decelK* COUNTS_PER_CENTIMETER * strafeCmsToGo);
                driveMotorsHeadingsFR(driveHeading, robotHeading, liveVelocity);
            }
        }
        lastVelocity=velocity;
        if (System.currentTimeMillis() > timeOutTime) {
            teamUtil.log("strafeToEncoder - TIMED OUT!");
            return false;
        } else {
            teamUtil.log("strafeToEncoder - FINISHED : Current: " + strafeEncoder.getCurrentPosition());
            return true;
        }
    }

    public void strafeToTarget(double maxVelocity, double strafeTarget, double driveHeading, double robotHeading, double endVelocity, long timeout) {
        teamUtil.log("strafeToTarget target: " + strafeTarget + " driveH: " + driveHeading + " robotH: " + robotHeading + " MaxV: " + maxVelocity + " EndV: " + endVelocity);
        details = false;
        long startTime = System.currentTimeMillis();
        long timeoutTime = startTime+timeout;

        if (details) teamUtil.log("Starting Strafe Encoder: "+ strafeEncoder.getCurrentPosition());

        double velocityChangeNeededAccel;
        double velocityChangeNeededDecel;
        if (endVelocity < MIN_STRAFE_END_VELOCITY) {
            endVelocity = MIN_STRAFE_END_VELOCITY;
        }
        // tics^2/s
        if (lastVelocity == 0) { // at stop
            velocityChangeNeededAccel = maxVelocity - MIN_STRAFE_START_VELOCITY;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        } else { // moving
            velocityChangeNeededAccel = maxVelocity - lastVelocity;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        }
        // all are measured in tics
        double startEncoder = strafeEncoder.getCurrentPosition();
        if ((driveHeading< 180 && strafeTarget-startEncoder <=0) || (driveHeading > 180 && startEncoder-strafeTarget <=0))
        {
            teamUtil.log("ALREADY PAST TARGET--Not Strafing");
            stopMotors();
            return;
        }
        double totalTics = Math.abs(startEncoder-strafeTarget);
        double accelerationDistance = Math.abs(velocityChangeNeededAccel / MAX_STRAFE_ACCELERATION);
        double decelerationDistance = Math.abs(velocityChangeNeededDecel / MAX_STRAFE_DECELERATION);
        if (accelerationDistance+decelerationDistance >= totalTics ) { // No room for cruise phase
            if (details) teamUtil.log("Adjusting distances to eliminate cruise phase");
            double accelPercentage = accelerationDistance / (accelerationDistance + decelerationDistance);
            double decelPercentage = 1-accelPercentage;
            accelerationDistance = totalTics * accelPercentage;
            decelerationDistance = totalTics * decelPercentage;
            if (details) teamUtil.log("Adjusting distances to eliminate cruise phase");

        }
        double distanceRemaining = 0;
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("Total tics: " + totalTics);
            teamUtil.log("Acceleration distance: " + accelerationDistance);
            teamUtil.log("Deceleration distance: " + decelerationDistance);
        }
        distanceRemaining = driveHeading<180 ? strafeTarget - strafeEncoder.getCurrentPosition() : strafeEncoder.getCurrentPosition()-strafeTarget;

        setBulkReadAuto();



//acceleration
        double currentVelocity = 0;
        while ((distanceRemaining > (totalTics-accelerationDistance))&&teamUtil.keepGoing(timeoutTime)) {
            distanceRemaining = driveHeading<180 ? strafeTarget - strafeEncoder.getCurrentPosition() : strafeEncoder.getCurrentPosition()-strafeTarget;
            if (lastVelocity == 0) {
                currentVelocity = MAX_STRAFE_ACCELERATION * (Math.max(0,totalTics-distanceRemaining)) + MIN_STRAFE_START_VELOCITY; // increases velocity
            } else {
                currentVelocity = MAX_STRAFE_ACCELERATION * (Math.max(0,totalTics-distanceRemaining)) + lastVelocity;
            }
            if (details) teamUtil.log("Accelerating at Velocity: "+ currentVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(driveHeading, robotHeading, currentVelocity);

        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after acceleration: " + distanceRemaining);
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered After Acceleration Phase");
            stopMotors();
            return;


        }
//cruise
        while ((distanceRemaining > decelerationDistance)&&teamUtil.keepGoing(timeoutTime)) {
            distanceRemaining = driveHeading<180 ? strafeTarget - strafeEncoder.getCurrentPosition() : strafeEncoder.getCurrentPosition()-strafeTarget;
            if (details) teamUtil.log("Cruising at Velocity: "+ maxVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(driveHeading, robotHeading, maxVelocity); // constant
        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after cruise: " + distanceRemaining);
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered After Cruise Phase");
            stopMotors();
            return;


        }


//deceleration
        while ((distanceRemaining > 0)&&teamUtil.keepGoing(timeoutTime)) {
            distanceRemaining = driveHeading<180 ? strafeTarget - strafeEncoder.getCurrentPosition() : strafeEncoder.getCurrentPosition()-strafeTarget;
            currentVelocity = MAX_STRAFE_DECELERATION * distanceRemaining + endVelocity;
            if (details) teamUtil.log("Decelerating at Velocity: "+ currentVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(driveHeading, robotHeading, currentVelocity);// decreases
        }
        if (details) {
            teamUtil.log("distance after deceleration: " + distanceRemaining);
        }
        if (endVelocity <= MIN_STRAFE_END_VELOCITY) {
            stopMotors();
            if (details) {
                teamUtil.log("Went below or was min end velocity");
            }
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered");
            stopMotors();
            return;


        }
        setBulkReadOff();
        lastVelocity = endVelocity;
        teamUtil.log("strafeToTarget--Finished.  Current Strafe Encoder:" + strafeEncoder.getCurrentPosition());

    }


    public void driveStraightToTarget(double maxVelocity, double forwardTarget, double driveHeading, double robotHeading, double endVelocity, long timeout) {
        // same as movecm but reads distance from dead wheels
        teamUtil.log("driveToTarget target: " + forwardTarget + " driveH: " + driveHeading + " robotH: " + robotHeading + " MaxV: " + maxVelocity + " EndV: " + endVelocity);
        details = false;
        long startTime = System.currentTimeMillis();
        long timeoutTime = startTime+timeout;



        if (details) teamUtil.log("Starting Forward Encoder: "+ forwardEncoder.getCurrentPosition());

        double velocityChangeNeededAccel;
        double velocityChangeNeededDecel;
        if (endVelocity < MIN_END_VELOCITY) {
            endVelocity = MIN_END_VELOCITY;
        }
        // tics^2/s
        if (lastVelocity == 0) {
            velocityChangeNeededAccel = maxVelocity - MIN_START_VELOCITY;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        } else {
            velocityChangeNeededAccel = maxVelocity - lastVelocity;
            velocityChangeNeededDecel = maxVelocity - endVelocity;
        }
        // all are measured in tics
        double startEncoder = forwardEncoder.getCurrentPosition();
        if (((driveHeading< 90 || driveHeading>270)&&forwardTarget-startEncoder >=0) || ((driveHeading> 90 && driveHeading<270) && startEncoder-forwardTarget >=0)){

            teamUtil.log("ALREADY PAST TARGET--Not Moving");
            stopMotors();
            return;
        }
        double totalTics = Math.abs(startEncoder-forwardTarget);
        double accelerationDistance = Math.abs(velocityChangeNeededAccel / MAX_STRAIGHT_ACCELERATION);
        double decelerationDistance = Math.abs(velocityChangeNeededDecel / MAX_STRAIGHT_DECELERATION);
        if (accelerationDistance+decelerationDistance >= totalTics ) { // No room for cruise phase
            if (details) teamUtil.log("Adjusting distances to eliminate cruise phase");
            double accelPercentage = accelerationDistance / (accelerationDistance + decelerationDistance);
            double decelPercentage = 1-accelPercentage;
            accelerationDistance = totalTics * accelPercentage;
            decelerationDistance = totalTics * decelPercentage;
            if (details) teamUtil.log("Adjusting distances to eliminate cruise phase");

        }
        double distanceRemaining = 0;
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("Total tics: " + totalTics);
            teamUtil.log("Acceleration distance: " + accelerationDistance);
            teamUtil.log("Deceleration distance: " + decelerationDistance);
        }
        distanceRemaining = (driveHeading< 90 || driveHeading>270) ? forwardEncoder.getCurrentPosition()-forwardTarget : forwardTarget - forwardEncoder.getCurrentPosition();

        setBulkReadAuto();


//acceleration
        double currentVelocity = 0;
        while ((distanceRemaining > (totalTics-accelerationDistance))&&teamUtil.keepGoing(timeoutTime)) {
            distanceRemaining = (driveHeading< 90 || driveHeading>270) ? forwardEncoder.getCurrentPosition()-forwardTarget : forwardTarget - forwardEncoder.getCurrentPosition();
            if (lastVelocity == 0) {
                currentVelocity = MAX_STRAIGHT_ACCELERATION * (totalTics-distanceRemaining) + MIN_START_VELOCITY;
            } else {
                currentVelocity = MAX_STRAIGHT_ACCELERATION * (totalTics-distanceRemaining) + lastVelocity;
            }
            if (details) teamUtil.log("Accelerating at Velocity: "+ currentVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(driveHeading, robotHeading, currentVelocity);

        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after acceleration: " + distanceRemaining);
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered After Acceleration Phase");
            stopMotors();
            return;


        }
//cruise
        while ((distanceRemaining > decelerationDistance)&&teamUtil.keepGoing(timeoutTime)) {
            distanceRemaining = (driveHeading< 90 || driveHeading>270) ? forwardEncoder.getCurrentPosition()-forwardTarget : forwardTarget - forwardEncoder.getCurrentPosition();
            if (details) teamUtil.log("Cruising at Velocity: "+ maxVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(driveHeading, robotHeading, maxVelocity);
        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after cruise: " + distanceRemaining);
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered After Cruise Phase");
            stopMotors();
            return;


        }


//deceleration
        while ((distanceRemaining > 0)&&teamUtil.keepGoing(timeoutTime)) {
            distanceRemaining = (driveHeading< 90 || driveHeading>270) ? forwardEncoder.getCurrentPosition()-forwardTarget : forwardTarget - forwardEncoder.getCurrentPosition();
            currentVelocity = MAX_STRAIGHT_DECELERATION * distanceRemaining + endVelocity;
            if (details) teamUtil.log("Decelerating at Velocity: "+ currentVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(driveHeading, robotHeading, currentVelocity);
        }
        if (details) {
            teamUtil.log("distance after deceleration: " + distanceRemaining);
        }
        if (endVelocity <= MIN_END_VELOCITY) {
            stopMotors();
            if (details) {
                teamUtil.log("Went below or was min end velocity");
            }
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered");
            stopMotors();
            return;


        }
        setBulkReadOff();
        lastVelocity = endVelocity;
        teamUtil.log("driveToTarget--Finished.  Current Forward Encoder:" + forwardEncoder.getCurrentPosition());

    }

    /************************************************************************************************************/
    // New Methods to drive based on odometry pods

    // Drive straight forward or backward while attempting to hold the strafe encoder at a specific value
    // Robot heading should be 0,90,180, or 270.  Drive Heading will be determined by the target
    public boolean straightHoldingStrafeEncoder(double maxVelocity, double straightTarget, double strafeTarget, int robotHeading, double endVelocity, ActionCallback action, double actionTarget, long timeout) {
        if(robotHeading!=90&&robotHeading!=270&&robotHeading!=0&&robotHeading!=180){
            teamUtil.log("straightHoldingStrafeEncoder - ERROR INCOMPATIBLE ROBOT HEADING");
            stopMotors();
            return false;
        }

        teamUtil.log("straightHoldingStrafeEncoder target: " + straightTarget +  " Strafe target: " + strafeTarget + " robotH: " + robotHeading + " MaxV: " + maxVelocity + " EndV: " + endVelocity);
        details = details;
        long startTime = System.currentTimeMillis();
        long timeoutTime = startTime+timeout;

        if (details) teamUtil.log("Starting Forward Encoder: "+ forwardEncoder.getCurrentPosition());
        if (details) teamUtil.log("Starting Strafe Encoder: "+ strafeEncoder.getCurrentPosition());

        double velocityChangeNeededAccel;
        double velocityChangeNeededDecel;
        double driveHeading;
        double startEncoder = forwardEncoder.getCurrentPosition();
        boolean goingUp;
        if(straightTarget-startEncoder >=0){
            driveHeading = robotHeading;
            goingUp = true;
        }else{
            driveHeading = adjustAngle(robotHeading+180);
            goingUp=false;
        }

        float headingFactor = goingUp? 1 : -1; // reverse correction for going backwards

        double requestedEndVelocity = endVelocity;
        if (endVelocity < MIN_END_VELOCITY) {
            endVelocity = MIN_END_VELOCITY;
        }

        if (lastVelocity == 0) {
            velocityChangeNeededAccel = maxVelocity - MIN_START_VELOCITY;
        } else {
            velocityChangeNeededAccel = maxVelocity - lastVelocity;
        }
        velocityChangeNeededDecel = maxVelocity - endVelocity;
        // all are measured in tics

        double totalTics = Math.abs(startEncoder-straightTarget);
        double accelerationDistance = Math.abs(velocityChangeNeededAccel / MAX_STRAIGHT_ACCELERATION);
        double decelerationDistance = Math.abs(velocityChangeNeededDecel / MAX_STRAIGHT_DECELERATION);
        if (accelerationDistance+decelerationDistance >= totalTics ) { // No room for cruise phase
            if (details) teamUtil.log("Adjusting distances to eliminate cruise phase");
            double accelPercentage = accelerationDistance / (accelerationDistance + decelerationDistance);
            double decelPercentage = 1-accelPercentage;
            accelerationDistance = totalTics * accelPercentage;
            decelerationDistance = totalTics * decelPercentage;
            if (details) teamUtil.log("Adjusting distances to eliminate cruise phase");

        }
        double distanceRemaining = 0;
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("Total tics: " + totalTics);
            teamUtil.log("Acceleration distance: " + accelerationDistance);
            teamUtil.log("Deceleration distance: " + decelerationDistance);
        }
        distanceRemaining = Math.abs(straightTarget - forwardEncoder.getCurrentPosition());

        setBulkReadAuto();
        boolean actionDone = false;
        int currentPos;

        //-------Acceleration Phase
        double currentVelocity;
        double adjustedDriveHeading;
        while ((distanceRemaining > (totalTics-accelerationDistance))&&teamUtil.keepGoing(timeoutTime)) {
            currentPos = forwardEncoder.getCurrentPosition();
            distanceRemaining = (!goingUp) ? currentPos-straightTarget : straightTarget - currentPos;
            if (lastVelocity == 0) {
                currentVelocity = MAX_STRAIGHT_ACCELERATION * (Math.max(0,totalTics-distanceRemaining)) + MIN_START_VELOCITY;
            } else {
                currentVelocity = MAX_STRAIGHT_ACCELERATION * (Math.max(0,totalTics-distanceRemaining)) + lastVelocity;
            }
            adjustedDriveHeading = driveHeading + MathUtils.clamp((strafeEncoder.getCurrentPosition() - strafeTarget)* STRAIGHT_HEADING_DECLINATION, -STRAIGHT_MAX_DECLINATION, STRAIGHT_MAX_DECLINATION) * headingFactor;
            if (details) {
                teamUtil.log("dh: " + adjustedDriveHeading);
            }

            if (details) teamUtil.log("Accelerating at Velocity: "+ currentVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(adjustedDriveHeading, robotHeading, currentVelocity);
            if(action!=null&&!actionDone&&((goingUp&&currentPos>=actionTarget)||(!goingUp&&currentPos<=actionTarget))){
                action.action();
                actionDone=true;
            }


        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after acceleration: " + distanceRemaining);
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered After Acceleration Phase");
            stopMotors();
            return false;


        }

        //-------Cruise Phase
        while ((distanceRemaining > decelerationDistance)&&teamUtil.keepGoing(timeoutTime)) {
            currentPos = forwardEncoder.getCurrentPosition();
            distanceRemaining = (!goingUp) ? currentPos-straightTarget : straightTarget - currentPos;
            adjustedDriveHeading = driveHeading + MathUtils.clamp((strafeEncoder.getCurrentPosition() - strafeTarget)* STRAIGHT_HEADING_DECLINATION, -STRAIGHT_MAX_DECLINATION, STRAIGHT_MAX_DECLINATION) * headingFactor;
            if (details) {
                teamUtil.log("dh: " + adjustedDriveHeading);
            }
            if (details) teamUtil.log("Cruising at Velocity: "+ maxVelocity + " Tics Remaining: " + distanceRemaining);

            driveMotorsHeadingsFR(adjustedDriveHeading, robotHeading, maxVelocity);
            if(action!=null&&!actionDone&&((goingUp&&currentPos>=actionTarget)||(!goingUp&&currentPos<=actionTarget))){
                action.action();
                actionDone=true;
            }
        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after cruise: " + distanceRemaining);
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered After Cruise Phase");
            stopMotors();
            return false;
        }

        //-------Deceleration Phase
        while ((distanceRemaining > 0)&&teamUtil.keepGoing(timeoutTime)) {
            currentPos = forwardEncoder.getCurrentPosition();
            distanceRemaining = (!goingUp) ? currentPos-straightTarget : straightTarget - currentPos;
            currentVelocity = MAX_STRAIGHT_DECELERATION * distanceRemaining + endVelocity;
            adjustedDriveHeading = driveHeading + MathUtils.clamp((strafeEncoder.getCurrentPosition() - strafeTarget)* STRAIGHT_HEADING_DECLINATION, -STRAIGHT_MAX_DECLINATION, STRAIGHT_MAX_DECLINATION) * headingFactor;
            if (details) {
                teamUtil.log("dh: " + adjustedDriveHeading);
            }

            if (details) teamUtil.log("Decelerating at Velocity: "+ currentVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(adjustedDriveHeading, robotHeading, currentVelocity);
            if(action!=null&&!actionDone&&((goingUp&&currentPos>=actionTarget)||(!goingUp&&currentPos<=actionTarget))){
                action.action();
                actionDone=true;
            }

        }
        if (details) {
            teamUtil.log("distance after deceleration: " + distanceRemaining);
        }
        if (requestedEndVelocity < 1) {
            stopMotors();
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered");
            stopMotors();
            return false;


        }
        setBulkReadOff();
        lastVelocity = endVelocity;
        teamUtil.log("driveStraightToTargetWithStrafeEncoderValue--Finished.  Current Forward Encoder:" + forwardEncoder.getCurrentPosition());
        return true;
    }

    // Strafe straight left or right while attempting to hold the forward encoder at a specific value
    // Robot heading should be 0,90,180, or 270.  Drive Heading will be determined by the target
    public boolean strafeHoldingStraightEncoder(double maxVelocity, double strafeTarget, double straightTarget, int robotHeading, double endVelocity, ActionCallback action, double actionTarget, long timeout) {
        if(robotHeading!=90&&robotHeading!=270&&robotHeading!=0&&robotHeading!=180){
            teamUtil.log("strafeHoldingStraightEncoder - ERROR INCOMPATIBLE ROBOT HEADING");
            stopMotors();
            return false;
        }

        teamUtil.log("strafeHoldingStraightEncoder target: " + strafeTarget +  " Straight target: " + straightTarget + " robotH: " + robotHeading + " MaxV: " + maxVelocity + " EndV: " + endVelocity);
        details = details;
        long startTime = System.currentTimeMillis();
        long timeoutTime = startTime+timeout;

        if (details) teamUtil.log("Starting Strafe Encoder: "+ strafeEncoder.getCurrentPosition());
        if (details) teamUtil.log("Starting Forward Encoder: "+ forwardEncoder.getCurrentPosition());

        double velocityChangeNeededAccel;
        double velocityChangeNeededDecel;
        double driveHeading;
        double startEncoder = strafeEncoder.getCurrentPosition();
        boolean goingUp;
        if(strafeTarget-startEncoder >=0){
            driveHeading = adjustAngle(robotHeading-90);
            goingUp = true;
        }else{
            driveHeading = adjustAngle(robotHeading+90);
            goingUp=false;
        }

        float headingFactor = goingUp? -1 : 1; // reverse correction for going backwards

        double requestedEndVelocity = endVelocity;
        if (endVelocity < MIN_END_VELOCITY) {
            endVelocity = MIN_END_VELOCITY;
        }

        if (lastVelocity == 0) {
            velocityChangeNeededAccel = maxVelocity - MIN_START_VELOCITY;
        } else {
            velocityChangeNeededAccel = maxVelocity - lastVelocity;
        }
        velocityChangeNeededDecel = maxVelocity - endVelocity;
        // all are measured in tics

        double totalTics = Math.abs(startEncoder-strafeTarget);
        double accelerationDistance = Math.abs(velocityChangeNeededAccel / MAX_STRAFE_ACCELERATION);
        double decelerationDistance = Math.abs(velocityChangeNeededDecel / MAX_STRAFE_DECELERATION);
        if (accelerationDistance+decelerationDistance >= totalTics ) { // No room for cruise phase
            if (details) teamUtil.log("Adjusting distances to eliminate cruise phase");
            double accelPercentage = accelerationDistance / (accelerationDistance + decelerationDistance);
            double decelPercentage = 1-accelPercentage;
            accelerationDistance = totalTics * accelPercentage;
            decelerationDistance = totalTics * decelPercentage;
            if (details) teamUtil.log("Adjusting distances to eliminate cruise phase");

        }
        double distanceRemaining = 0;
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("Total tics: " + totalTics);
            teamUtil.log("Acceleration distance: " + accelerationDistance);
            teamUtil.log("Deceleration distance: " + decelerationDistance);
        }
        distanceRemaining = Math.abs(strafeTarget - strafeEncoder.getCurrentPosition());

        setBulkReadAuto();
        boolean actionDone = false;
        int currentPos;

        //-------Acceleration Phase
        double currentVelocity;
        double adjustedDriveHeading;
        while ((distanceRemaining > (totalTics-accelerationDistance))&&teamUtil.keepGoing(timeoutTime)) {
            currentPos = strafeEncoder.getCurrentPosition();
            distanceRemaining = (!goingUp) ? currentPos-strafeTarget : strafeTarget - currentPos;
            if (lastVelocity == 0) {
                currentVelocity = MAX_STRAFE_ACCELERATION * (Math.max(0,totalTics-distanceRemaining)) + MIN_START_VELOCITY;
            } else {
                currentVelocity = MAX_STRAFE_ACCELERATION * (Math.max(0,totalTics-distanceRemaining)) + lastVelocity;
            }
            adjustedDriveHeading = driveHeading + MathUtils.clamp((forwardEncoder.getCurrentPosition() - straightTarget)* STRAFE_HEADING_DECLINATION, -STRAFE_MAX_DECLINATION, STRAFE_MAX_DECLINATION) * headingFactor;
            if (details) {
                teamUtil.log("dh: " + adjustedDriveHeading);
            }

            if (details) teamUtil.log("Accelerating at Velocity: "+ currentVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(adjustedDriveHeading, robotHeading, currentVelocity);
            if(action!=null&&!actionDone&&((goingUp&&currentPos>=actionTarget)||(!goingUp&&currentPos<=actionTarget))){
                action.action();
                actionDone=true;
            }


        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after acceleration: " + distanceRemaining);
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered After Acceleration Phase");
            stopMotors();
            return false;


        }

        //-------Cruise Phase
        while ((distanceRemaining > decelerationDistance)&&teamUtil.keepGoing(timeoutTime)) {
            currentPos = strafeEncoder.getCurrentPosition();
            distanceRemaining = (!goingUp) ? currentPos-strafeTarget : strafeTarget - currentPos;
            adjustedDriveHeading = driveHeading + MathUtils.clamp((forwardEncoder.getCurrentPosition() - straightTarget)* STRAFE_HEADING_DECLINATION, -STRAFE_MAX_DECLINATION, STRAFE_MAX_DECLINATION) * headingFactor;            if (details) {
                teamUtil.log("dh: " + adjustedDriveHeading);
            }
            if (details) teamUtil.log("Cruising at Velocity: "+ maxVelocity + " Tics Remaining: " + distanceRemaining);

            driveMotorsHeadingsFR(adjustedDriveHeading, robotHeading, maxVelocity);
            if(action!=null&&!actionDone&&((goingUp&&currentPos>=actionTarget)||(!goingUp&&currentPos<=actionTarget))){
                action.action();
                actionDone=true;
            }
        }
        if (details) {
            teamUtil.log("Heading:" + getHeading());
            teamUtil.log("distance after cruise: " + distanceRemaining);
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered After Cruise Phase");
            stopMotors();
            return false;
        }

        //-------Deceleration Phase
        while ((distanceRemaining > 0)&&teamUtil.keepGoing(timeoutTime)) {
            currentPos = strafeEncoder.getCurrentPosition();
            distanceRemaining = (!goingUp) ? currentPos-strafeTarget : strafeTarget - currentPos;
            currentVelocity = MAX_STRAIGHT_DECELERATION * distanceRemaining + endVelocity;
            adjustedDriveHeading = driveHeading + MathUtils.clamp((forwardEncoder.getCurrentPosition() - straightTarget)* STRAFE_HEADING_DECLINATION, -STRAFE_MAX_DECLINATION, STRAFE_MAX_DECLINATION) * headingFactor;            if (details) {
                teamUtil.log("dh: " + adjustedDriveHeading);
            }

            if (details) teamUtil.log("Decelerating at Velocity: "+ currentVelocity + " Tics Remaining: " + distanceRemaining);
            driveMotorsHeadingsFR(adjustedDriveHeading, robotHeading, currentVelocity);
            if(action!=null&&!actionDone&&((goingUp&&currentPos>=actionTarget)||(!goingUp&&currentPos<=actionTarget))){
                action.action();
                actionDone=true;
            }

        }
        if (details) {
            teamUtil.log("distance after deceleration: " + distanceRemaining);
        }
        if (requestedEndVelocity < 1) {
            stopMotors();
        }
        if(System.currentTimeMillis()>timeoutTime){
            teamUtil.log("TIMEOUT Triggered");
            stopMotors();
            return false;


        }
        setBulkReadOff();
        lastVelocity = endVelocity;
        teamUtil.log("strafeHoldingStraightEncoder--Finished.  Current Strafe Encoder:" + strafeEncoder.getCurrentPosition());
        return true;
    }

    public void moveTo(double maxVelocity, double strafeTarget, double straightTarget, double endVelocity,ActionCallback action, double actionTarget, long timeout){

        details = details;
        double straightChange = straightTarget - forwardEncoder.getCurrentPosition();
        double strafeChange = strafeTarget - strafeEncoder.getCurrentPosition();
        teamUtil.log("Straight Target: " + straightTarget);
        teamUtil.log("Strafe Target: " + strafeTarget);
        teamUtil.log("Straight Change: " + straightChange);
        teamUtil.log("Strafe Target: " + strafeTarget);

        double straightCm = Math.abs(straightChange)/TICS_PER_CM_STRAIGHT_ENCODER;
        double strafeCm = Math.abs(strafeChange)/TICS_PER_CM_STRAFE_ENCODER;
        double angle = Math.atan(straightCm/strafeCm);
        teamUtil.log("Angle Without Adjustment: " + angle);

        double driveHeading;


        double startHeading = getHeading();
        if (straightChange>0&&strafeChange>0){ //y change positive x change positive
            driveHeading=adjustAngle(getHeading()-(90-angle));
        }else if (straightChange>0&&strafeChange<0){ //y change positive x change negative
            driveHeading=adjustAngle(getHeading()+(90-angle));
        }else if (straightChange<0&&strafeChange>0){ //y change negative x change positive
            driveHeading=adjustAngle(getHeading()-(90+angle));
        }else{
            driveHeading=adjustAngle(getHeading()+(90+angle));
        }
        teamUtil.log("Drive Heading: " + driveHeading);

        double strafeTicsRemaining = Math.abs(strafeChange);
        double straightTicsRemaining = Math.abs(straightChange);

        double strafeTicsTravelled = 0;
        double straightTicsTravelled = 0;
        double startStrafe = strafeEncoder.getCurrentPosition();
        double startForward = forwardEncoder.getCurrentPosition();

        while(strafeTicsTravelled<strafeTicsRemaining&&straightTicsTravelled<straightTicsRemaining){
            strafeTicsTravelled = Math.abs(strafeEncoder.getCurrentPosition()-startStrafe);
            straightTicsTravelled = Math.abs(forwardEncoder.getCurrentPosition()-startForward);

            if(details){
                teamUtil.log("Strafe Tics Remaining: " + strafeTicsRemaining);
                teamUtil.log("Straight Tics Remaining: " + straightTicsRemaining);

            }

            driveMotorsHeadingsFR(driveHeading, startHeading,maxVelocity);
        }
        stopMotors();



    }

    // This was developed for CS April Tag Localization.  Might be some stuff here useful for the new "moveTo" odometry pod method
    public void backToPoint(double robotHeading, double x, double y, double endVelocity) { // assumes robot heading is 180
        // x positive means to the robots right
        // y positive means robot move backwards (not tested for anything else!)
        double heading, distance;
        teamUtil.log("Move to Point: x/y " + x + "/"+ y);
        distance = Math.sqrt(x*x+y*y);
        if (y == 0) {
            heading = x < 0 ? 270 : 90;
        } else if (y > 0) { // Using vertical (y-axis) to compute reference angles since 0 is at top
            heading = adjustAngle(Math.toDegrees(Math.atan(x / y)));
        } else {
            heading = 180 + Math.toDegrees(Math.atan(x / y));
        }
        moveCm(MAX_VELOCITY,distance,heading,180,endVelocity);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Stall detection using encoders

    public boolean waitForStall(long timeout){
        // wait for the robot to slow down on the wall
        // expects setPower
        boolean details = false;
        teamUtil.log("Waiting For Stall");
        long timeoutTime = System.currentTimeMillis()+timeout;
        int lastEncoder = forwardEncoder.getCurrentPosition();
        double startEncoderVelocity = forwardEncoder.getVelocity();
        while(teamUtil.keepGoing(timeoutTime)){
            teamUtil.pause(25);
            if(details){teamUtil.log("Forward Encoder Velocity: " + forwardEncoder.getVelocity());}
            int currentEncoder = forwardEncoder.getCurrentPosition();
            if (details) teamUtil.log("last: " + lastEncoder + " current: "+ currentEncoder);
            if(forwardEncoder.getVelocity()<startEncoderVelocity*0.5){
                teamUtil.log("Stalled");
                return true;
            }
            lastEncoder = currentEncoder;
        }
        teamUtil.log("Didn't Stall");
        return false; // didn't get a stall
    }



    /************************************************************************************************************/
    // Methods to turn the robot in place

    public void spinToHeading(double heading) {
        // moves at full speed then decelerates to spin
        double velocity = MAX_VELOCITY;
        boolean turningLeft;
        double startHeading = getHeading();
        double currentHeading = getHeading();
        double leftCoefficient = 1;
        double rightCoefficient = 1;
        setMotorsWithEncoder();
        if (heading > currentHeading) { // fix direction
            if (heading - currentHeading < 180) {
                leftCoefficient = -1;
            } else {
                rightCoefficient = -1;
            }
        } else {
            if (currentHeading - heading < 180) {
                rightCoefficient = -1;
            } else {
                leftCoefficient = -1;
            }
        }
        if (details) {
            teamUtil.log("turning left: " + rightCoefficient);
            teamUtil.log("current heading: " + currentHeading);
            teamUtil.log("heading goal: " + (heading + SPIN_DRIFT_DEGREES));
        }
        if (details) {
            teamUtil.log("crossing 0/360 barrier");
        }
        while (Math.abs(currentHeading - heading) > SPIN_DECEL_THRESHOLD_DEGREES) {
            setMotorVelocities(leftCoefficient * velocity, rightCoefficient * velocity, leftCoefficient * velocity, rightCoefficient * velocity);
            currentHeading = getHeading();
        }
        if (details) {
            teamUtil.log("current heading: " + currentHeading);
            teamUtil.log("heading cutoff (greater): " + adjustAngle(heading - SPIN_CRAWL_DEGREES));
            teamUtil.log("done with max velocity phase");
            teamUtil.log("heading: " + currentHeading);
        }
        while (Math.abs(currentHeading - heading) > SPIN_CRAWL_DEGREES) {
            currentHeading = getHeading();
            velocity = ((MAX_VELOCITY - SPIN_CRAWL_SPEED) / (SPIN_DECEL_THRESHOLD_DEGREES - SPIN_CRAWL_DEGREES)) * (Math.abs(currentHeading - heading) - SPIN_DECEL_THRESHOLD_DEGREES) + MAX_VELOCITY; // wrote an equasion
            if (velocity < SPIN_CRAWL_SPEED) {
                velocity = SPIN_CRAWL_SPEED;
            }
            setMotorVelocities(leftCoefficient * velocity, rightCoefficient * velocity, leftCoefficient * velocity, rightCoefficient * velocity);
        }

        if (details) {
            teamUtil.log("done with deceleration phase");
            teamUtil.log("heading: " + currentHeading);
        }
        while (Math.abs(currentHeading - heading) > SPIN_DRIFT_DEGREES) {
            currentHeading = getHeading();
            velocity = SPIN_CRAWL_SPEED;
            setMotorVelocities(leftCoefficient * velocity, rightCoefficient * velocity, leftCoefficient * velocity, rightCoefficient * velocity);
        }

        if (details) {
            teamUtil.log("done with crawl phase");
            teamUtil.log("heading: " + currentHeading);
        }

        setMotorsBrake();
        setMotorPower(0);
    }



    /************************************************************************************************************/
    //Methods to drive based on joystick values
    //TODO: Why are their multiple versions?

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Drive robot based on two joystick values
    // Implements a deadband where joystick position will be ignored (translation and rotation)
    // Uses a linear scale that starts at the edge of the dead band
    // Attempts to hold the last heading that was commanded via a turn
    public void driveJoyStick(float leftJoyStickX, float leftJoyStickY, float rightJoyStickX, boolean isFast) {
        // returns values to drive to the main loop
        boolean details = true;

        float DEADBAND = 0.1f;
        float SLOPE = 1.6f;
        float FASTSLOPE = 3.6f;
        float SLOWSPEED = .1f;
        float STRAFESLOWSPEED = 0.25f;
        //float SLOPE = 1 / (1 - DEADBAND); // linear from edge of dead band to full power  TODO: Should this be a curve?

        float POWERFACTOR = 1; // MAKE LESS THAN 0 to cap upperlimit of power
        float leftX;
        float leftY;
        float rotationAdjustment;

        float scaleAmount = isFast ? 1f : 0.5f; // full power is "fast", half power is "slow"

        float frontLeft, frontRight, backRight, backLeft;

        // Ensure nothing happens if we are inside the deadband
        if (Math.abs(leftJoyStickX) < DEADBAND) {
            leftJoyStickX = 0;
        }
        if (Math.abs(leftJoyStickY) < DEADBAND) {
            leftJoyStickY = 0;
        }
        if (Math.abs(rightJoyStickX) < DEADBAND) {
            rightJoyStickX = 0;
        }

        if (movingAutonomously.get() && (leftJoyStickX != 0 || rightJoyStickX != 0 || leftJoyStickY != 0)) { // Do we need to interrupt an autonomous operation?
            manualInterrupt.set(true);
        }
        if(leftJoyStickX == 0){ // NEW Power Curves
            leftX = 0;
        } else if(Math.abs(leftJoyStickX)<.75){
            leftX = leftJoyStickX>0? STRAFESLOWSPEED:-STRAFESLOWSPEED;
        } else{
            if (isFast) {
                leftX = leftJoyStickX * FASTSLOPE + (leftJoyStickX>0? -2.6f:2.6f);
            }
            else{
                leftX = leftJoyStickX * SLOPE + (leftJoyStickX>0? -1.1f:1.1f);
            }
        }

        if(leftJoyStickY == 0){
            leftY = 0;
        } else if(Math.abs(leftJoyStickY)<.75){
            leftY = leftJoyStickY>0? SLOWSPEED:-SLOWSPEED;
        } else{
            if (isFast) {
                leftY = leftJoyStickY * FASTSLOPE + (leftJoyStickY>0? -2.6f:2.6f);
            }
            else{
                leftY = leftJoyStickY *SLOPE + (leftJoyStickY>0? -1.1f:1.1f);
            }
        }

        final float MAXROTATIONFACTOR = 0.8f;
        if (Math.abs(rightJoyStickX) > DEADBAND) { // driver is turning the robot
            rotationAdjustment = (float) (rightJoyStickX * 0.525 * scaleAmount);
            holdingHeading = false;
        } else { // Need to automatically hold the current heading
            if (!holdingHeading) { // start to hold current heading
                heldHeading = getHeading();
                holdingHeading = true;
            }
            rotationAdjustment = (float) getHeadingError(heldHeading) * -1f * .05f; // auto rotate to held heading
            rotationAdjustment = rotationAdjustment * Math.min(Math.max(Math.abs(leftX), Math.abs(leftY)), 0.7f); // make it proportional to speed
            rotationAdjustment = MathUtils.clamp(rotationAdjustment, -MAXROTATIONFACTOR,MAXROTATIONFACTOR ); // clip rotation so it doesn't obliterate translation
        }

        frontLeft = -(leftY - leftX - rotationAdjustment);
        frontRight = (-leftY - leftX - rotationAdjustment);
        backRight = (-leftY + leftX - rotationAdjustment);
        backLeft = -(leftY + leftX - rotationAdjustment);

        if (details) {

            teamUtil.telemetry.addLine("Joy X/Y: "+ leftJoyStickX+ "/"+ leftJoyStickY+ " X/Y: "+ leftX+ "/"+leftY);
            if (holdingHeading) {
                teamUtil.telemetry.addData("HOLDING:", heldHeading);
            }
        }

        fl.setPower(frontLeft);
        fr.setPower(frontRight);
        br.setPower(backRight);
        bl.setPower(backLeft);

        //TODO: take out soon just for testing purposes
        telemetry.addLine("Left Joystick Y: " + leftJoyStickY);
        telemetry.addLine("Left Joystick X: " + leftJoyStickX);

        telemetry.addLine("fl power: " + frontLeft);
        telemetry.addLine("fr power: " + frontRight);
        telemetry.addLine("bl power: " + backLeft);
        telemetry.addLine("br power: " + backRight);


    }

    public void driveJoyStickV2(float leftJoyStickX, float leftJoyStickY, float rightJoyStickX, boolean isFast, boolean isSlow) {
        boolean details = true;

        float DEADBAND = 0.1f;
        float SLOWSLOPE =0.22f;
        float SLOWSLOPESTRAFE =0.35f;
        float SLOPE = 0.55f;
        float FASTSLOPE = 1f;
        float SLOWSPEED = .1f;
        //float STRAFESLOWSPEED = 0.25f;
        //float SLOPE = 1 / (1 - DEADBAND); // linear from edge of dead band to full power  TODO: Should this be a curve?

        float POWERFACTOR = 1; // MAKE LESS THAN 0 to cap upperlimit of power
        float leftX;
        float leftY;
        float rotationAdjustment;

        float scaleAmount = isFast ? 1f : 0.5f; // full power is "fast", half power is "slow"

        float frontLeft, frontRight, backRight, backLeft;

        // Ensure nothing happens if we are inside the deadband
        if (Math.abs(leftJoyStickX) < DEADBAND) {
            leftJoyStickX = 0;
        }
        if (Math.abs(leftJoyStickY) < DEADBAND) {
            leftJoyStickY = 0;
        }
        if (Math.abs(rightJoyStickX) < DEADBAND) {
            rightJoyStickX = 0;
        }

        if (movingAutonomously.get() && (leftJoyStickX != 0 || rightJoyStickX != 0 || leftJoyStickY != 0)) { // Do we need to interrupt an autonomous operation?
            manualInterrupt.set(true);
        }
        if(leftJoyStickX == 0){ // NEW Power Curves
            leftX = 0;
        } else if(isSlow){
            leftX = leftJoyStickX*SLOWSLOPESTRAFE+(leftJoyStickX>0? -0.078f:0.078f);
        } else if(isFast){
            leftX = leftJoyStickX * FASTSLOPE ;
        }
        //medium speed
        else{
            leftX = leftJoyStickX*SLOPE+(leftJoyStickX>0? -0.055f:0.055f);

        }

        if(leftJoyStickY == 0){
            leftY = 0;
        } else if(isSlow){
            leftY = leftJoyStickY*SLOWSLOPE+(leftJoyStickY>0? -0.078f:0.078f);
        } else if(isFast){
            leftY = leftJoyStickY * FASTSLOPE ;
        }
        //medium speed
        else{
            leftY = leftJoyStickY*SLOPE+(leftJoyStickY>0? -0.055f:0.055f);

        }

        final float MAXROTATIONFACTOR = 0.8f;
        if (Math.abs(rightJoyStickX) > DEADBAND) { // driver is turning the robot
            //old code
            //rotationAdjustment = (float) (rightJoyStickX * 0.525 * scaleAmount);
            rotationAdjustment = (float) (rightJoyStickX * 1 * scaleAmount);

            holdingHeading = false;
        } else { // Need to automatically hold the current heading
            if (!holdingHeading) { // start to hold current heading
                heldHeading = getHeading();
                holdingHeading = true;
            }
            // old code
            //rotationAdjustment = (float) getHeadingError(heldHeading) * -1f * .05f; // auto rotate to held heading
            rotationAdjustment = (float) getHeadingError(heldHeading) * -1f * .001f; // auto rotate to held heading
            rotationAdjustment = rotationAdjustment * Math.min(Math.max(Math.abs(leftX), Math.abs(leftY)), 0.7f); // make it proportional to speed
            rotationAdjustment = MathUtils.clamp(rotationAdjustment, -MAXROTATIONFACTOR,MAXROTATIONFACTOR ); // clip rotation so it doesn't obliterate translation
        }
        //old code from old motors

        frontLeft = -(leftY - leftX - rotationAdjustment);
        frontRight = (-leftY - leftX - rotationAdjustment);
        backRight = (-leftY + leftX - rotationAdjustment);
        backLeft = -(leftY + leftX - rotationAdjustment);






        if (details) {

            teamUtil.telemetry.addLine("Joy X/Y: "+ leftJoyStickX+ "/"+ leftJoyStickY+ " X/Y: "+ leftX+ "/"+leftY);
            if (holdingHeading) {
                teamUtil.telemetry.addData("HOLDING:", heldHeading);
            }
        }

        fl.setPower(frontLeft);
        fr.setPower(frontRight);
        br.setPower(backRight);
        bl.setPower(backLeft);

        String currentSpeedState;
        if(isSlow){
            currentSpeedState="Is Slow";
        }else if(isFast){
            currentSpeedState="Is Fast";
        }else{
            currentSpeedState="Is Medium";
        }
        telemetry.addLine("Current Speed State " + currentSpeedState);

        //TODO: take out soon just for testing purposes
        telemetry.addLine("Left Joystick Y: " + leftJoyStickY);
        telemetry.addLine("Left Joystick X: " + leftJoyStickX);

        telemetry.addLine("fl power: " + frontLeft);
        telemetry.addLine("fr power: " + frontRight);
        telemetry.addLine("bl power: " + backLeft);
        telemetry.addLine("br power: " + backRight);


    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Adds Field Relative driving to driveJoyStick  TODO: Why multiple versions?
    public void universalDriveJoystick(float leftJoyStickX, float leftJoyStickY, float rightJoyStickX, boolean isFast, double robotHeading) {
        double angleInRadians = robotHeading * Math.PI / 180;
        float leftX = leftJoyStickX;
        float leftY = leftJoyStickY;
        float rightX = rightJoyStickX;

        //rotate to obtain new coordinates
        float rotatedLeftX = (float) (Math.cos(angleInRadians) * leftX - Math.sin(angleInRadians) * leftY);
        float rotatedLeftY = (float) (Math.sin(angleInRadians) * leftX + Math.cos(angleInRadians) * leftY);

        driveJoyStick(rotatedLeftX, rotatedLeftY, rightX, isFast);
    }

    public void universalDriveJoystickV2(float leftJoyStickX, float leftJoyStickY, float rightJoyStickX, boolean isFast,boolean isSlow, double robotHeading) {
        double angleInRadians = robotHeading * Math.PI / 180;
        float leftX = leftJoyStickX;
        float leftY = leftJoyStickY;
        float rightX = rightJoyStickX;

        //rotate to obtain new coordinates
        float rotatedLeftX = (float) (Math.cos(angleInRadians) * leftX - Math.sin(angleInRadians) * leftY);
        float rotatedLeftY = (float) (Math.sin(angleInRadians) * leftX + Math.cos(angleInRadians) * leftY);

        driveJoyStickV2(rotatedLeftX, rotatedLeftY, rightX, isFast,isSlow);
    }

    public void setHeldHeading(double heading){
        holdingHeading = true;
        heldHeading = heading;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Testing Code

    public void findMaxVelocity(int cmDistance) {
        // set motors to 3000 (theoretical max) then see how fast they actually go
        long startTime = System.currentTimeMillis();
        teamUtil.log("Finding Forward Max Velocities...");
        resetAllDriveEncoders();
        double travelTics = COUNTS_PER_CENTIMETER * cmDistance;
        setMotorVelocities(3000, 3000, 3000, 3000);
        double flmax = 0, frmax = 0, blmax = 0, brmax = 0, v;
        double ticStartPosition = fr.getCurrentPosition();
        while (fr.getCurrentPosition() < travelTics) {
            flmax = (v = fl.getVelocity()) > flmax ? v : flmax;
            frmax = (v = fr.getVelocity()) > frmax ? v : frmax;
            blmax = (v = bl.getVelocity()) > blmax ? v : blmax;
            brmax = (v = br.getVelocity()) > brmax ? v : brmax;
            teamUtil.log("Looping FL:" + flmax + " FR:" + frmax + " BL:" + blmax + " BR:" + brmax);
        }
        stopMotors();
        long elapsedTime = System.currentTimeMillis()-startTime;
        double ticsTraveled = fr.getCurrentPosition()-ticStartPosition;
        double cmsTraveled = ticsTraveled/COUNTS_PER_CENTIMETER;
        double timeS = elapsedTime/1000.0;
        teamUtil.log("Elapsed Time: " + elapsedTime);
        teamUtil.log("Tics Traveled: " + ticsTraveled);
        teamUtil.log("Cms Traveled: " + cmsTraveled);
        teamUtil.log("Cms Per Second: " + cmsTraveled/timeS);

        teamUtil.log("Forward Max Velocities FL:" + flmax + " FR:" + frmax + " BL:" + blmax + " BR:" + brmax);
    }

    public void findMaxStrafeVelocity(double distance){
        // same as above
        setHeading(180);
        long startTime = System.currentTimeMillis();
        teamUtil.log("Finding Strafing Max Velocities...");
        resetAllDriveEncoders();
        double travelTics = distance*COUNTS_PER_CENTIMETER;
        teamUtil.log("Travel Tics: " + travelTics);
        double ticStartPosition = fr.getCurrentPosition();
        driveMotorsHeadingsFR(270,180,3000);
        double flmax = 0, frmax = 0, blmax = 0, brmax = 0, v;

        while(fr.getCurrentPosition()<travelTics){

            flmax = (v = fl.getVelocity()) > flmax ? v : flmax;
            frmax = (v = fr.getVelocity()) > frmax ? v : frmax;
            blmax = (v = bl.getVelocity()) > blmax ? v : blmax;
            brmax = (v = br.getVelocity()) > brmax ? v : brmax;
            teamUtil.log("Looping FL:" + flmax + " FR:" + frmax + " BL:" + blmax + " BR:" + brmax);


        }
        stopMotors();
        long elapsedTime = System.currentTimeMillis()-startTime;
        double ticsTraveled = fr.getCurrentPosition()-ticStartPosition;
        double cmsTraveled = ticsTraveled/COUNTS_PER_CENTIMETER;
        double timeS = elapsedTime/1000.0;
        teamUtil.log("Elapsed Time: " + elapsedTime);
        teamUtil.log("Tics Traveled: " + ticsTraveled);

        teamUtil.log("Cms Per Second: " + cmsTraveled/timeS);
        teamUtil.log("Cms Traveled: " + cmsTraveled);

        teamUtil.log("Tics Per Second: " + ticsTraveled/(elapsedTime/1000));

        teamUtil.log("Strafing Max Velocities FL:" + flmax + " FR:" + frmax + " BL:" + blmax + " BR:" + brmax);
    }
}


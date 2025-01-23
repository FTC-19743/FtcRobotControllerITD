package org.firstinspires.ftc.teamcode.assemblies;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.libs.teamUtil;

import java.util.concurrent.atomic.AtomicBoolean;

@Config
public class AxonSlider {
    public CRServo axon;
    AnalogInput axonPotentiometer;
    public OctoQuad octoquad;
    public final int ODO_SLIDER  = 0; // Intake Slider octoquad port

    AtomicBoolean moving = new AtomicBoolean(false);
    public AtomicBoolean timedOut = new AtomicBoolean(false);
    public static boolean details = false;

    public static int OUTSIDE_AVOIDANCE_THRESHOLD = 40; // for manual control
    public static int RIGHT_LIMIT = 0; //set during calibration
    public static int LEFT_LIMIT = -18380; //set during calibration
    static public float SLIDER_UNLOAD = -9420; //set during calibration
    static public float SLIDER_READY = -9420; //set during calibration
    public static int LEFT_LIMIT_OFFSET = -739; // TODO Recalibrate
    static public float SLIDER_UNLOAD_OFFSET = -361;// TODO Recalibrate
    static public float SLIDER_READY_OFFSET = -375; // TODO Recalibrate
    static public int RETRACT_LEFT_LIMIT = -11700;
    static public int RETRACT_RIGHT_LIMIT = -8700;

    public static float POWER_ADJUSTEMENT = -.01f;
    public static int DEGREE_NOISE_THRESHOLD = 20;

    public static float RTP_MAX_VELOCITY = .5f;
    public static int RTP_LEFT_DEADBAND_DEGREES = 490; // TODO Recalibrate
    public static int RTP_RIGHT_DEADBAND_DEGREES = 545; // TODO Recalibrate
    public static int RTP_SLOW_THRESHOLD = 1500; // TODO Recalibrate
    public static float RTP_SLOW_VELOCITY = .07f; // TODO Recalibrate
    public static int RTP_LEFT_DEADBAND_SLOW_DEGREES = 58; // TODO Recalibrate
    public static int RTP_RIGHT_DEADBAND_SLOW_DEGREES = 85; // TODO Recalibrate
    //public static float RTP_P_COEFFICIENT = .005f;
    //public static float RTP_MIN_VELOCITY = .1f;
    public static float MANUAL_SLIDER_INCREMENT;
    double SLIDER_X_DEADBAND = 0.5;
    public boolean CALIBRATED = false;

    //292 slider degrees to 10 cm
    // 1 mm = 51.2727 tics
    //LEFT IS NEGATIVE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    public static float SLIDER_TICS_PER_MM = 51.2f;
    public static float CM_PER_SLIDER_DEGREE = 0.03425f;

    public static int axonRotations = 0; // The number of full rotations postive or negative the servo has traveled from its center range
    private double lastDegrees360; // the rotational angle of the servo in degrees last time we checked

    public void init(HardwareMap hardwareMap, String servoName, String sensorName){
        teamUtil.log("Init AxonSlider");
        axon = hardwareMap.crservo.get(servoName);
        octoquad = hardwareMap.get(OctoQuad.class, "octoquad");
        octoquad.setSingleEncoderDirection(ODO_SLIDER,  OctoQuad.EncoderDirection.FORWARD); // RIGHT IS POSITIVE, LEFT IS NEGATIVE
        axonPotentiometer = hardwareMap.analogInput.get(sensorName);
        lastDegrees360 = getDegrees360();
        axonRotations=0; // presumes we are in the middle rotation.  Run Calibrate to be sure.
    }

    public void calibrateEncoder(float power) {
        // TODO: Create a new version of calibrate that uses the octoquad encoder instead of the potentiometer
        // TODO: Then comment out the old version and update all the calls to it
        CALIBRATED = false;
        teamUtil.log("Calibrating Intake Slider");
        axon.setPower(power);
        int lastPosition = getPositionEncoder();
        teamUtil.pause(250);
        while ((int)getPositionEncoder() != lastPosition) {
            lastPosition = getPositionEncoder();
            if (details) teamUtil.log("Calibrate Intake (With OctoQuad Encoder): Slider: " + getPositionEncoder());
            teamUtil.pause(50);
        }
        axon.setPower(0);
        teamUtil.log("Calibrate Intake Slider Done");

        teamUtil.log("Calibrate POS: " + getPositionEncoder());
        teamUtil.pause(250);

        octoquad.resetMultiplePositions(ODO_SLIDER);


        teamUtil.log("RIGHT LIMIT: " + RIGHT_LIMIT);
        teamUtil.log("LEFT LIMIT: " + LEFT_LIMIT);
        teamUtil.log("SLIDER READY: " + SLIDER_READY);
        teamUtil.log("SLIDER UNLOAD: " + SLIDER_UNLOAD);

        CALIBRATED = true;

    }
    /*
    // Flipper must be in a safe position for travel to the far right side
    // Leaves the slider on the far left
    public void calibrate (float power, int rotations) {
        CALIBRATED = false;
        teamUtil.log("Calibrating Intake Slider");
        axon.setPower(power);
        int lastPosition = getPosition();
        teamUtil.pause(250);
        while ((int)getPosition() != lastPosition) {
            lastPosition = getPosition();
            if (details) teamUtil.log("Calibrate Intake: Slider: " + getPosition());
            teamUtil.pause(50);
        }
        axon.setPower(0);
        teamUtil.log("Calibrate Intake Slider Done");

        axonRotations = rotations;
        teamUtil.log("Calibrate POS: " + getPosition());
        teamUtil.pause(250);
        lastDegrees360 = getDegrees360();

        RIGHT_LIMIT = getPosition();
        LEFT_LIMIT = getPosition()+ LEFT_LIMIT_OFFSET;
        SLIDER_READY = getPosition()+ SLIDER_READY_OFFSET;
        SLIDER_UNLOAD = getPosition()+ SLIDER_UNLOAD_OFFSET;
        teamUtil.log("RIGHT LIMIT: " + RIGHT_LIMIT);
        teamUtil.log("LEFT LIMIT: " + LEFT_LIMIT);
        teamUtil.log("SLIDER READY: " + SLIDER_READY);
        teamUtil.log("SLIDER UNLOAD: " + SLIDER_UNLOAD);

        CALIBRATED = true;
    }

     */


    public int getPositionEncoder() {
        return octoquad.readSinglePosition(ODO_SLIDER);
    }

    // Return the computed absolute position of the servo by using the number
    // of full rotations plus the current angle of the servo
    public int getPosition(){
        return (int) getDegrees360() + axonRotations*360;
    }

    // The servo is a bit stronger in one direction than the other.  This is evident at very low speeds
    // Call this method to adjust for this
    public void setAdjustedPower(float power){
        if (Math.abs(power-0) < .001f) {
            axon.setPower(0); // don't adjust 0 to non-zero
        } else {
            axon.setPower(power+POWER_ADJUSTEMENT);
        }
    }

    public void setPower(double power){ //-.5 to .5
        axon.setPower(power);
    }


    private void runToTargetEncoder (double target, float sliderVelocity, int leftDeadband, int rightDeadband, long timeOut) {
        // TODO: Create a new version of runToTarget that uses the octoquad encoder instead of the potentiometer (getPositionEncoder())
        // TODO: Then modify run to position to use runToTargetEncoder
        moving.set(true);
        teamUtil.log("Slider runToTargetEncoder: " + (int)target + " at power: "+ sliderVelocity);
        long timeoutTime = System.currentTimeMillis()+timeOut;
        if (target > RIGHT_LIMIT || target < LEFT_LIMIT) {
            teamUtil.log("ERROR: SLIDER TARGET OUTSIDE OF RANGE! -- Not Moving");
            moving.set(false);
            return;
        }

        double ticsFromTarget = target-getPositionEncoder();
        double lastTicsFromTarget = ticsFromTarget;
        double initialTicsFromTarget = ticsFromTarget;
        setAdjustedPower(ticsFromTarget > 0 ? sliderVelocity * -1 : sliderVelocity); // start moving
        while (teamUtil.keepGoing(timeoutTime) &&
                initialTicsFromTarget < 0 ? // while we haven't yet reached the drift threshold
                ticsFromTarget < -leftDeadband :
                ticsFromTarget > rightDeadband) {

            if (details)
                teamUtil.log("Tics from Target: " + ticsFromTarget + " Power: " + sliderVelocity);
            lastTicsFromTarget = ticsFromTarget;
            // teamutil.pause(10);
            ticsFromTarget = target - getPositionEncoder();

        }
        axon.setPower(0);
        moving.set(false);
        if (System.currentTimeMillis() > timeoutTime) {
            timedOut.set(true);
            teamUtil.log("Slider runToTarget TIMED OUT: " + (int)getPositionEncoder());
        } else {
            teamUtil.log("Slider runToTarget Finished at : " + (int)getPositionEncoder());
        }
    }

    /*
    // Run the servo to the specified position as quickly as possible
    // This method returns when the servo is in the new position
    private void runToTarget (double target, float sliderVelocity, int leftDeadband, int rightDeadband, long timeOut) {
        moving.set(true);
        teamUtil.log("Slider runToTarget: " + (int)target + " at power: "+ sliderVelocity);
        long timeoutTime = System.currentTimeMillis()+timeOut;
        if (target > RIGHT_LIMIT || target < LEFT_LIMIT) {
            teamUtil.log("ERROR: SLIDER TARGET OUTSIDE OF RANGE! -- Not Moving");
            moving.set(false);
            return;
        }
        loop();
        double degreesFromTarget = target-getPosition();
        double lastdegreesFromTarget = degreesFromTarget;
        double initialDegreesFromTarget = degreesFromTarget;
        setAdjustedPower(degreesFromTarget > 0 ? sliderVelocity * -1 : sliderVelocity); // start moving
        while (teamUtil.keepGoing(timeoutTime) &&
                initialDegreesFromTarget < 0 ? // while we haven't yet reached the drift threshold
                    degreesFromTarget < -leftDeadband :
                    degreesFromTarget > rightDeadband) {
            loop();
            if (details)
                teamUtil.log("Degrees from Target: " + degreesFromTarget + " Power: " + sliderVelocity);
            lastdegreesFromTarget = degreesFromTarget;
            // teamutil.pause(10);
            degreesFromTarget = target - getPosition();
            while (teamUtil.keepGoing(timeoutTime) && Math.abs(degreesFromTarget - lastdegreesFromTarget) > DEGREE_NOISE_THRESHOLD) {
                if (details) teamUtil.log("Ignoring Noise from Servo Potentiometer. Degrees: " + degreesFromTarget + "Last degrees: " + lastdegreesFromTarget);
                //lastdegreesFromTarget = degreesFromTarget;
                teamUtil.pause(10); // wait for a better reading
                loop();
                degreesFromTarget = target - getPosition();
            }
        }
        axon.setPower(0);
        moving.set(false);
        if (System.currentTimeMillis() > timeoutTime) {
            timedOut.set(true);
            teamUtil.log("Slider runToTarget TIMED OUT: " + (int)getPosition());
        } else {
            teamUtil.log("Slider runToTarget Finished at : " + (int)getPosition());
        }
    }

     */


/*
    // Run the servo to the specified position as quickly as possible
    // This method returns when the servo is in the new position
    public void runToPosition (double target, long timeOut) {
        timedOut.set(false);
        moving.set(true);
        loop(); // update position in case it hasn't happened recently
        teamUtil.log("Slider Run to Position Target: " + (int)target);
        if(Math.abs(target-getPosition())<RTP_SLOW_THRESHOLD){
            runToTarget(target, RTP_SLOW_VELOCITY, RTP_LEFT_DEADBAND_SLOW_DEGREES, RTP_RIGHT_DEADBAND_SLOW_DEGREES, timeOut);
        } else {
            runToTarget(target, RTP_MAX_VELOCITY, RTP_LEFT_DEADBAND_DEGREES, RTP_RIGHT_DEADBAND_DEGREES, timeOut);
        }
        teamUtil.log("Slider Run to Position Finished at : " + (int)getPosition());
    }

 */

    // Run the servo to the specified position as quickly as possible
    // This method returns when the servo is in the new position
    public void runToEncoderPosition (double target, boolean forceMaxSpeed, long timeOut) {
        timedOut.set(false);
        moving.set(true);
        teamUtil.log("Slider Run to Position Target Encoder : " + (int)target);
        if(Math.abs(target-getPositionEncoder())<RTP_SLOW_THRESHOLD && !forceMaxSpeed){
            runToTargetEncoder(target, RTP_SLOW_VELOCITY, RTP_LEFT_DEADBAND_SLOW_DEGREES, RTP_RIGHT_DEADBAND_SLOW_DEGREES, timeOut);
        } else {
            runToTargetEncoder(target, RTP_MAX_VELOCITY, RTP_LEFT_DEADBAND_DEGREES, RTP_RIGHT_DEADBAND_DEGREES, timeOut);
        }
        teamUtil.log("Slider Run to Position Target Encoder Finished at : " + (int)getPositionEncoder());
    }
/*
    public void runToPositionNoWait(double target, long timeOutTime) {
        if (moving.get()) { // Slider is already running in another thread
            teamUtil.log("WARNING: Attempt to AxonSlider.RunToPosition while slider is moving--ignored");
            return;
        } else {
            moving.set(true);
            teamUtil.log("Launching Thread to AxonSlider.RunToPosition");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    runToPosition(target, timeOutTime);
                }
            });
            thread.start();
        }
    }

 */

    public void runToEncoderPositionNoWait(double target, boolean forceMaxSpeed, long timeOutTime) {
        if (moving.get()) { // Slider is already running in another thread
            teamUtil.log("WARNING: Attempt to AxonSlider.RunToPosition while slider is moving--ignored");
            return;
        } else {
            moving.set(true);
            teamUtil.log("Launching Thread to AxonSlider.RunToPosition");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    runToEncoderPosition(target, forceMaxSpeed, timeOutTime);
                }
            });
            thread.start();
        }
    }
    /*
    public void manualSliderControl(double joystick) {
        if(Math.abs(joystick)>SLIDER_X_DEADBAND){
            double power = Math.abs(joystick)-0.5; //TODO MAke linear   (Coach: Isn't this linear already?  There might be something wierd here if the Deadband is less than .5?!)
            loop();

            if(joystick<0){
                if (Math.abs(getPosition()-LEFT_LIMIT)<40) { // TODO: What is this magic number?! Recalibrate for Encoder?
                    setAdjustedPower(0);
                }
                else{
                    setAdjustedPower((float)power);
                }
            }else if (joystick>0){
                if (Math.abs(getPosition()-RIGHT_LIMIT)<40){
                    setAdjustedPower(0);
                }else{
                    setAdjustedPower(-(float)power);
                }
            }else{
                setAdjustedPower(0);
            }

        }
        else{
            setAdjustedPower(0);
        }
    }

     */

    public void manualSliderControlWithEncoder(double joystick) {
        if(Math.abs(joystick)>SLIDER_X_DEADBAND){
            double power = Math.abs(joystick)-SLIDER_X_DEADBAND; //TODO MAke linear   (Coach: Isn't this linear already?  There might be something wierd here if the Deadband is less than .5?!)
            loop();

            if(joystick<0){
                if (Math.abs(getPositionEncoder()-LEFT_LIMIT)<OUTSIDE_AVOIDANCE_THRESHOLD) { // TODO: What is this magic number?! Recalibrate for Encoder?
                    setAdjustedPower(0);
                }
                else{
                    setAdjustedPower((float)power);
                }
            }else if (joystick>0){
                if (Math.abs(getPositionEncoder()-RIGHT_LIMIT)<OUTSIDE_AVOIDANCE_THRESHOLD){
                    setAdjustedPower(0);
                }else{
                    setAdjustedPower(-(float)power);
                }
            }else{
                setAdjustedPower(0);
            }

        }
        else{
            setAdjustedPower(0);
        }
    }

    // IMPORTANT:  This method must be called frequently whenever the servo is being moved.
    // It keep track of the servo position and notices when it wraps around between 0 and 360
    // So that the overall position can be calculated
    public void loop(){
        //teamUtil.log("LOOP CALLED");
        double degrees = getDegrees360();
        if(Math.abs(lastDegrees360-degrees)>180){
            if(degrees<180){
                axonRotations++;
                if (details) teamUtil.log("Axon Rotations Went UP.  Voltage: " + axonPotentiometer.getVoltage()+ "AXON Position: " + getPosition());
            }else{
                axonRotations--;
                if (details)teamUtil.log("Axon Rotations Went DOWN: Voltage: " + axonPotentiometer.getVoltage()+ "AXON Position: " + getPosition());
            }
        }
        lastDegrees360 = degrees;
    }

    // Convert from potentiometer reading to degrees
    public double getDegrees360(){
        return (axonPotentiometer.getVoltage()*360)/(3.274);
    }

}

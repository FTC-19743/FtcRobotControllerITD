package org.firstinspires.ftc.teamcode.assemblies;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.libs.teamUtil;

import java.util.concurrent.atomic.AtomicBoolean;

@Config
public class AxonSlider {
    public CRServo axon;
    AnalogInput axonPotentiometer;
    AtomicBoolean moving = new AtomicBoolean(false);
    public AtomicBoolean timedOut = new AtomicBoolean(false);
    public boolean details = false;

    public int RIGHT_LIMIT = 0; //tentative
    public int LEFT_LIMIT = 0; //tentative
    static public float SLIDER_UNLOAD = 0; // TODO Recalibrate
    static public float SLIDER_READY = 0;//TODO Recalibrate

    public static int RTP_DEADBAND_DEGREES = 3;
    public static float RTP_P_COEFFICIENT = .001f;
    public static float RTP_MIN_VELOCITY = .1f;
    public static float RTP_MAX_VELOCITY = .5f;
    public static float POWER_ADJUSTEMENT = -.01f;
    public static float MANUAL_SLIDER_INCREMENT;
    double SLIDER_X_DEADBAND = 0.5;
    public boolean CALIBRATED = false;

    public int axonRotations = 0; // The number of full rotations postive or negative the servo has traveled from its center range
    private double lastDegrees360; // the rotational angle of the servo in degrees last time we checked

    public void init(HardwareMap hardwareMap, String servoName, String sensorName){
        teamUtil.log("Init AxonSlider");
        axon = hardwareMap.crservo.get(servoName);
        axonPotentiometer = hardwareMap.analogInput.get(sensorName);
        lastDegrees360 = getDegrees360();
        axonRotations=0; // presumes we are in the middle rotation.  Run Calibrate to be sure.
    }

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
        RIGHT_LIMIT = getPosition();
        LEFT_LIMIT = getPosition()-739;
        SLIDER_READY = getPosition()-375;
        SLIDER_UNLOAD = getPosition()-321;
        teamUtil.log("RIGHT LIMIT: " + RIGHT_LIMIT);
        teamUtil.log("LEFT LIMIT: " + LEFT_LIMIT);
        teamUtil.log("SLIDER READY: " + SLIDER_READY);
        teamUtil.log("SLIDER UNLOAD: " + SLIDER_UNLOAD);

        CALIBRATED = true;
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

    // Run the servo to the specified position as quickly as possible
    // This method returns when the servo is in the new position
    public void runToPosition (double target, long timeOut) {
        moving.set(true);
        teamUtil.log("Slider Run to Position Target: " + (int)target);
        long timeoutTime = System.currentTimeMillis()+timeOut;
        if (target > RIGHT_LIMIT || target < LEFT_LIMIT) {
            teamUtil.log("ERROR: TARGET OUTSIDE OF RANGE! -- Not Moving");
            moving.set(false);
            return;
        }
        float sliderVelocity;
        double degreesFromTarget = target-getPosition();
        while (teamUtil.keepGoing(timeoutTime) && Math.abs(degreesFromTarget) > RTP_DEADBAND_DEGREES) {
            loop();
            sliderVelocity = (float) Math.min(RTP_P_COEFFICIENT * Math.abs(degreesFromTarget) + RTP_MIN_VELOCITY, RTP_MAX_VELOCITY);
            if (degreesFromTarget > 0) {
                sliderVelocity *= -1;
            }
            setAdjustedPower(sliderVelocity);
            if (details) teamUtil.log("Degrees from Target: " + degreesFromTarget + " Power: " + sliderVelocity);
            degreesFromTarget = target-getPosition();
        }
        axon.setPower(0);
        moving.set(false);
        if (System.currentTimeMillis() > timeoutTime) {
            timedOut.set(true);
            teamUtil.log("Slider Run to Position TIMED OUT: " + (int)getPosition());
        } else {
            teamUtil.log("Slider Run to Position Finished at : " + (int)getPosition());
        }
    }

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

    public void manualSliderControl(double joystick) {
        if(Math.abs(joystick)>SLIDER_X_DEADBAND){
            double power = Math.abs(joystick)-0.5; //TODO MAke linear
            loop();

            if(joystick<0){
                if (Math.abs(getPosition()-LEFT_LIMIT)<40) {
                    setAdjustedPower(0);
                }
                else{
                    setAdjustedPower((float)power); //TODO Know Signs
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

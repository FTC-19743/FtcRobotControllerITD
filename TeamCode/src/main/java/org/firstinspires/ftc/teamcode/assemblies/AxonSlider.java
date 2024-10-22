package org.firstinspires.ftc.teamcode.assemblies;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Config
public class AxonSlider {
    public int farRight = 661; //tentative
    public int farLeft = -89; //tentative

    public static int RTP_DEADBAND_DEGREES = 3;
    public static float RTP_P_COEFFICIENT = .001f;
    public static float RTP_MIN_VELOCITY = .1f;
    public static float RTP_MAX_VELOCITY = .5f;
    public static float POWER_ADJUSTEMENT = -.01f;

    public int axonRotations = 0;
    public CRServo axon;
    private double lastDegrees360;
    public static int sliderUnload = 355;


    //public double speedCoefficient = 0.5; //tentative needs to be tuned


    AnalogInput axonPotentiometer;




    public void init(HardwareMap hardwareMap, String servoName, String sensorName){
        axon = hardwareMap.crservo.get(servoName);
        axonPotentiometer = hardwareMap.analogInput.get(sensorName);
        lastDegrees360 = getDegrees360();
        axonRotations=0;
    }

    public void calibrate (float power, int rotations) {
        teamUtil.log("Calibrating Intake Slider");
        axon.setPower(power);
        int lastPosition = getPosition();
        teamUtil.pause(250);
        while ((int)getPosition() != lastPosition) {
            lastPosition = getPosition();
            teamUtil.log("Calibrate Intake: Slider: " + getPosition());
            teamUtil.pause(50);
        }
        axon.setPower(0);
        teamUtil.log("Calibrate Intake Slider Done");
        axonRotations = rotations;
    }

    public int getPosition(){
        return (int) getDegrees360() + axonRotations*360;
    }

    public void reset(){
        axonRotations=0;
    }

    public void setAdjustedPower(float power){
        axon.setPower(power+POWER_ADJUSTEMENT);
    }

    public void runToPosition (double target) {
        teamUtil.log("Run to Position Target: " + (int)target);
        if (target > farRight || target < farLeft) {
            teamUtil.log("ERROR: TARGET OUTSIDE OF RANGE! -- Not Moving");
            return;
        }
        float sliderVelocity;
        double degreesFromTarget = target-getPosition();
        while (Math.abs(degreesFromTarget) > RTP_DEADBAND_DEGREES) {
            loop();
            sliderVelocity = (float) Math.min(RTP_P_COEFFICIENT * Math.abs(degreesFromTarget) + RTP_MIN_VELOCITY, RTP_MAX_VELOCITY);
            if (degreesFromTarget > 0) {
                sliderVelocity *= -1;
            }
            axon.setPower(sliderVelocity);
            degreesFromTarget = target-getPosition();
        }
        axon.setPower(0);
    }
/*

   public void runToPostitionWithStallDetect(double targetDegrees, int timeout){
        axonPotentiometer = hardwareMap.analogInput.get("sensor");
        axon = hardwareMap.servo.get("servo");

        boolean positive;
        if(axon.getPosition()-DEGREES_TO_POSITION(targetDegrees)>0){positive = false;}
        else{positive = true;}

        axon.setPosition(DEGREES_TO_POSITION(targetDegrees));
        double targetVoltage = DEGREES_TO_VOLTAGE(targetDegrees);
        double lastVoltage = axonPotentiometer.getVoltage();
        double changeInVoltage = 0;
        double timeStalled = 0;
        double startStallTime = 0;

        while(Math.abs(axonPotentiometer.getVoltage() - targetVoltage)>.005&&timeStalled<timeout){
            double voltage = axonPotentiometer.getVoltage();

            changeInVoltage = voltage-lastVoltage;

            if(Math.abs(changeInVoltage)<.005){
                if(timeStalled == 0){
                    startStallTime = System.currentTimeMillis();
                    teamUtil.pause(1);
                    teamUtil.log("Start Stall Time" + startStallTime);
                }
                timeStalled = System.currentTimeMillis()-startStallTime;
                teamUtil.log("Time Stalled" + timeStalled);
            }
            else{
                timeStalled = 0;
            }
            lastVoltage = voltage;

        }
        if(timeStalled>timeout){
            teamUtil.log("SERVO STALLED");
            double safePosition = axonPotentiometer.getVoltage()*(-1/3.126)+1;
            if(positive){
                axon.setPosition(safePosition-0.1);
            }
            else{
                axon.setPosition(safePosition+0.1);
            }

        }


    }
    public boolean running(){
        return moving.get();
    }

    public boolean timeOut(){
        return timedOut.get();
    }

    //Speed is from 0-100
    public void setPosition(int targetPosition, int speed, long timeout, int deadband){
        teamUtil.log("Starting setPosition");
        startTime = System.currentTimeMillis();
        currentSpeed.set(speed);
        currentTargetPosition.set(targetPosition);
        if(targetPosition<farLeft||targetPosition>farRight){
            teamUtil.log("Required Position Extended Past Limit (Axon)");
        }
        currentTimeout.set(timeout);
        theDeadband = deadband;
        if(moving.get()){

        } else{
            moving.set(true);
            timedOut.set(false);
            teamUtil.log("Opening move loop");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() { moveLoop();}
            });
            thread.start();
        }
    }

 */

    public void setPower(double power){ //-1 to 1
        axon.setPower(power);
    }







    public void loop(){
        //teamUtil.log("LOOP CALLED");
        double degrees = getDegrees360();
        if(Math.abs(lastDegrees360-degrees)>180){
            if(degrees<180){
                axonRotations++;
                teamUtil.log("Axon Voltage: " + axonPotentiometer.getVoltage()+ "AXON Position: " + getPosition() + " Axon Rotations Went UP");


            }else{
                axonRotations--;
                teamUtil.log("Axon Voltage: " + axonPotentiometer.getVoltage()+ "AXON Position: " + getPosition() + " Axon Rotations Went DOWN");

            }
        }
        lastDegrees360 = degrees;

    }


    public double getDegrees360(){
        return (axonPotentiometer.getVoltage()*360)/(3.274);
    }


}

package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import org.firstinspires.ftc.teamcode.libs.teamUtil;


public class AxonSlider {
    public int farRight = 661; //tentative
    public int farLeft = -89; //tentative


    public int axonRotations = 0;
    //private AtomicBoolean moving = new AtomicBoolean(false);
    //private AtomicBoolean timedOut = new AtomicBoolean(false);
    //private AtomicInteger currentTargetPosition = new AtomicInteger(0);
    //private AtomicInteger currentSpeed = new AtomicInteger(0);
    //private AtomicLong currentTimeout = new AtomicLong(0);
    public CRServo axon;
    //private long startTime = 0;
    //private int theDeadband = 2;
    //private double decelslope = 0.0056;
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

    public int getPosition(){
        return (int) getDegrees360() + axonRotations*360;
    }

    public void reset(){
        axonRotations=0;
    }
/*
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

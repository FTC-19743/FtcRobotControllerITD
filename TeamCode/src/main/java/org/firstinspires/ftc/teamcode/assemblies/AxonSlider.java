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
    public double farRight = .5; //tentative
    public double farLeft = .5; //tentative


    public int axonRotations = 0;
    private AtomicBoolean moving = new AtomicBoolean(false);
    private AtomicBoolean timedOut = new AtomicBoolean(false);
    private AtomicInteger currentTargetPosition = new AtomicInteger(0);
    private AtomicInteger currentSpeed = new AtomicInteger(0);
    private AtomicLong currentTimeout = new AtomicLong(0);
    public CRServo axon;
    private long startTime = 0;
    private int theDeadband = 2;
    private double decelslope = 0.0056;
    private double lastDegrees360;

    public double speedCoefficient = 0.5; //tentative needs to be tuned


    AnalogInput axonPotentiometer;




    public void init(HardwareMap hardwareMap, String servoName, String sensorName){
        axon = hardwareMap.crservo.get(servoName);
        axonPotentiometer = hardwareMap.analogInput.get(sensorName);
        lastDegrees360 = getDegrees360();
    }

    public int getPosition(){
        return (int) getDegrees360() + axonRotations*360;
    }

    public void reset(){
        axonRotations=0;
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

    public void setVelocity(int velocity){
        currentSpeed.set(velocity);
    }

    public void runToOffset(int xOffset, long timeout,int deadband){
        double startPos = getPosition();
        double axonOffset = Math.abs(xOffset-startPos); //start pos has to be converted to pixels
        double targetPos = startPos+xOffset;
        double speedNeeded = xOffset*(speedCoefficient);

        setPosition((int)targetPos,(int)speedNeeded,timeout,deadband);
    }



    public void moveLoop(){
        double maxPower;
        float power;

        int remainingDistance = Math.abs(getPosition()-currentTargetPosition.get());
        while(remainingDistance>theDeadband){
            maxPower =  0.5*currentSpeed.get()/100f;

            if(remainingDistance>=(maxPower/decelslope)){
                power = (float) maxPower;
            }
            else{
                power = (float)((decelslope)*remainingDistance);
            }

            power = power*(currentTargetPosition.get()>getPosition()? -1:1);
            axon.setPower(power);
            RobotLog.d("power: " + power + " degrees360: " + getDegrees360() + " Remaining Distance: " + remainingDistance + " rotations: " + axonRotations);

            if(startTime+currentTimeout.get()<System.currentTimeMillis()){
                RobotLog.d("TIMED OUT");
                axon.setPower(0);
                moving.set(false);
                timedOut.set(true);
                return;
            }
            if(Math.abs(lastDegrees360-getDegrees360())>180){
                if(getDegrees360()<180){
                    axonRotations++;
                }else{
                    axonRotations--;
                }
            }
            lastDegrees360 = getDegrees360();
            remainingDistance = Math.abs(getPosition()-currentTargetPosition.get());
        }
        axon.setPower(0);
        moving.set(false);
        timedOut.set(false);
    }

    public void loop(){
        if(Math.abs(lastDegrees360-getDegrees360())>180){
            if(getDegrees360()<180){
                axonRotations++;
            }else{
                axonRotations--;
            }
        }
        lastDegrees360 = getDegrees360();

        moving.set(false);
        timedOut.set(false);
    }


    private double getDegrees360(){
        return (axonPotentiometer.getVoltage()*360)/(3.274);
    }


}

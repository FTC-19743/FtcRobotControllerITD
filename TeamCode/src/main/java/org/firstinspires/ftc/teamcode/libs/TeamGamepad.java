package org.firstinspires.ftc.teamcode.libs;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TeamGamepad {
    public Gamepad gamepad;
    Telemetry telemetry;

    boolean aWasPressed = false;
    boolean aWasPressedLastTime = false;
    boolean aBumpToDo = false;

    double rightJoystickWasRight = 0;
    double rightJoystickWasRightLastTime = 0;
    boolean rightJoystickWasRightToDo = false;

    double rightJoystickWasLeft = 0;
    double rightJoystickWasLeftLastTime = 0;
    boolean rightJoystickWasLeftToDo = false;

    double rightJoystickWasUp = 0;
    double rightJoystickWasUpLastTime = 0;
    boolean rightJoystickWasUpToDo = false;

    double rightJoystickWasDown = 0;
    double rightJoystickWasDownLastTime = 0;
    boolean rightJoystickWasDownToDo = false;

    boolean bWasPressed = false;
    boolean bWasPressedLastTime = false;
    boolean bBumpToDo = false;
    boolean xWasPressed = false;
    boolean xWasPressedLastTime = false;
    boolean xBumpToDo = false;
    boolean yWasPressed = false;
    boolean yWasPressedLastTime = false;
    boolean yBumpToDo = false;
    boolean upWasPressed = false;
    boolean upWasPressedLastTime = false;
    boolean upBumpToDo = false;
    boolean downWasPressed = false;
    boolean downWasPressedLastTime = false;
    boolean downBumpToDo = false;
    boolean rightWasPressed = false;
    boolean rightWasPressedLastTime = false;
    boolean rightBumpToDo = false;
    boolean leftWasPressed = false;
    boolean leftWasPressedLastTime = false;
    boolean leftBumpToDo = false;
    boolean rightBumperWasPressed = false;
    boolean rightBumperWasPressedLastTime = false;
    boolean rightBumperBumpToDo = false;
    boolean leftBumperWasPressed = false;
    boolean leftBumperWasPressedLastTime = false;
    boolean leftBumperBumpToDo = false;
    double leftTriggerWasPressedLastTime = 0;
    double leftTriggerWasPressed = 0;
    boolean leftTriggerBumpToDo = false;
    double rightTriggerWasPressedLastTime = 0;
    double rightTriggerWasPressed = 0;
    boolean rightTriggerBumpToDo = false;
    boolean optionsWasPressedLastTime = false;
    boolean optionsWasPressed = false;
    boolean optionsBumpToDo = false;
    boolean startWasPressedLastTime = false;
    boolean startWasPressed = false;
    boolean startBumpToDo = false;
    boolean backWasPressedLastTime = false;
    boolean backWasPressed = false;
    boolean backBumpToDo = false;
    boolean homeWasPressedLastTime = false;
    boolean homeWasPressed = false;
    boolean homeBumpToDo = false;


    public TeamGamepad(){

    }
    public void initilize(boolean gamepad1){
        if(gamepad1) {
            gamepad = teamUtil.theOpMode.gamepad1;
        }else{
            gamepad = teamUtil.theOpMode.gamepad2;
        }
        telemetry = teamUtil.telemetry;
    }

    public void reset(){
        aBumpToDo = false;
        bBumpToDo = false;
        xBumpToDo = false;
        yBumpToDo = false;
        upBumpToDo = false;
        downBumpToDo = false;
        leftBumpToDo = false;
        rightBumpToDo = false;
        rightBumperBumpToDo = false;
        leftBumperBumpToDo = false;
        leftTriggerBumpToDo = false;
        rightTriggerBumpToDo = false;
        rightJoystickWasLeftToDo = false;
        rightJoystickWasRightToDo = false;
        rightJoystickWasUpToDo = false;
        rightJoystickWasDownToDo = false;
        optionsBumpToDo = false;
        startBumpToDo = false;
        aWasPressedLastTime = false;
        aWasPressed = false;
        bWasPressedLastTime = false;
        bWasPressed = false;
        xWasPressedLastTime = false;
        xWasPressed = false;
        yWasPressedLastTime = false;
        yWasPressed = false;
        downWasPressedLastTime = false;
        downWasPressed = false;
        upWasPressedLastTime = false;
        upWasPressed = false;
        leftWasPressedLastTime = false;
        leftWasPressed =false;
        rightWasPressedLastTime = false;
        rightWasPressed = false;
        rightBumperWasPressedLastTime = false;
        rightBumperWasPressed = false;
        leftBumperWasPressedLastTime = false;
        leftBumperWasPressed = false;
        leftTriggerWasPressedLastTime = 0;
        leftTriggerWasPressed = 0;
        rightTriggerWasPressedLastTime = 0;
        rightTriggerWasPressed = 0;
        rightJoystickWasLeftLastTime = 0;
        rightJoystickWasLeft = 0;
        rightJoystickWasRightLastTime = 0;
        rightJoystickWasRight = 0;
        optionsWasPressedLastTime = false;
        optionsWasPressed = false;
        homeWasPressedLastTime = false;
        homeWasPressed = false;
        homeBumpToDo = false;
    }
    public void loop(){
        aWasPressedLastTime = aWasPressed;
        aWasPressed = gamepad.a;
        bWasPressedLastTime = bWasPressed;
        bWasPressed = gamepad.b;
        xWasPressedLastTime = xWasPressed;

        xWasPressed = gamepad.x;
        yWasPressedLastTime = yWasPressed;
        yWasPressed = gamepad.y;
        downWasPressedLastTime = downWasPressed;
        downWasPressed = gamepad.dpad_down;
        upWasPressedLastTime = upWasPressed;
        upWasPressed = gamepad.dpad_up;
        leftWasPressedLastTime = leftWasPressed;
        leftWasPressed = gamepad.dpad_left;
        rightWasPressedLastTime = rightWasPressed;
        rightWasPressed = gamepad.dpad_right;
        rightBumperWasPressedLastTime = rightBumperWasPressed;
        rightBumperWasPressed = gamepad.right_bumper;
        leftBumperWasPressedLastTime = leftBumperWasPressed;
        leftBumperWasPressed = gamepad.left_bumper;
        leftTriggerWasPressedLastTime = leftTriggerWasPressed;
        leftTriggerWasPressed = gamepad.left_trigger;
        rightTriggerWasPressedLastTime = rightTriggerWasPressed;
        rightTriggerWasPressed = gamepad.right_trigger;
        rightJoystickWasLeftLastTime = rightJoystickWasLeft;
        rightJoystickWasLeft = gamepad.right_stick_x;
        rightJoystickWasUpLastTime = rightJoystickWasUp;
        rightJoystickWasUp = gamepad.right_stick_y;
        rightJoystickWasDownLastTime = rightJoystickWasDown;
        rightJoystickWasDown = gamepad.right_stick_y;
        rightJoystickWasRightLastTime = rightJoystickWasRight;
        rightJoystickWasRight = gamepad.right_stick_x;
        optionsWasPressedLastTime = optionsWasPressed;
        optionsWasPressed = gamepad.options;
        startWasPressedLastTime = startWasPressed;
        startWasPressed = gamepad.start;
        backWasPressedLastTime = backWasPressed;
        backWasPressed = gamepad.back;
        homeWasPressedLastTime = homeWasPressed;
        homeWasPressed = gamepad.guide;

        if (aWasPressed == false && aWasPressedLastTime == true) {
            aBumpToDo = true;
        }
        if (bWasPressed == false && bWasPressedLastTime == true) {
            bBumpToDo = true;
        }
        if (xWasPressed == false && xWasPressedLastTime == true) {
            xBumpToDo = true;
        }
        if (yWasPressed == false && yWasPressedLastTime == true) {
            yBumpToDo = true;
        }
        if (upWasPressed == false && upWasPressedLastTime == true) {
            upBumpToDo = true;
        }
        if (downWasPressed == false && downWasPressedLastTime == true) {
            downBumpToDo = true;
        }
        if (leftWasPressed == false && leftWasPressedLastTime == true) {
            leftBumpToDo = true;
        }
        if (rightWasPressed == false && rightWasPressedLastTime == true) {
            rightBumpToDo = true;
        }
        if (rightBumperWasPressed == false && rightBumperWasPressedLastTime == true) {
            rightBumperBumpToDo = true;
        }
        if (leftBumperWasPressed == false && leftBumperWasPressedLastTime == true) {
            leftBumperBumpToDo = true;
        }
        if (leftTriggerWasPressed < 0.8 && leftTriggerWasPressedLastTime >= 0.8){
            leftTriggerBumpToDo = true;
        }
        if (rightTriggerWasPressed < 0.8 && rightTriggerWasPressedLastTime >= 0.8){
            rightTriggerBumpToDo = true;
        }
        if (rightJoystickWasLeft > -0.8 && rightJoystickWasLeftLastTime <= -0.8){
            rightJoystickWasLeftToDo = true;
        }
        if (rightJoystickWasRight < 0.8 && rightJoystickWasRightLastTime >= 0.8){
            rightJoystickWasRightToDo = true;
        }
        if (rightJoystickWasDown < 0.8 && rightJoystickWasDownLastTime >= 0.8){
            rightJoystickWasDownToDo = true;
        }
        if (rightJoystickWasUp > -0.8 && rightJoystickWasUpLastTime <= -0.8){
            rightJoystickWasUpToDo = true;
        }
        if (optionsWasPressed == false && optionsWasPressedLastTime == true) {
            optionsBumpToDo = true;
        }
        if (startWasPressed == false && startWasPressedLastTime == true) {
            startBumpToDo = true;
        }
        if (backWasPressed == false && backWasPressedLastTime == true) {
            backBumpToDo = true;
        }
        if (homeWasPressed == false && homeWasPressedLastTime == true) {
            homeBumpToDo = true;
        }
    }
    public boolean wasAPressed(){
        if(aBumpToDo) {
            aBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasBPressed(){
        if(bBumpToDo) {
            bBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasXPressed(){
        if(xBumpToDo) {
            xBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasYPressed(){
        if(yBumpToDo) {
            yBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasUpPressed(){
        if(upBumpToDo) {
            upBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasDownPressed(){
        if(downBumpToDo) {
            downBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasRightPressed(){
        if(rightBumpToDo) {
            rightBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasLeftPressed(){
        if(leftBumpToDo) {
            leftBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasRightBumperPressed(){
        if(rightBumperBumpToDo) {
            rightBumperBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasLeftBumperPressed(){
        if(leftBumperBumpToDo) {
            leftBumperBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasLeftTriggerPressed(){
        if(leftTriggerBumpToDo){
            leftTriggerBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasRightTriggerPressed(){
        if(rightTriggerBumpToDo){
            rightTriggerBumpToDo = false;
            return true;
        }
        return false;
    }

    public boolean wasRightJoystickFlickedRight(){
        if(rightJoystickWasRightToDo){
            rightJoystickWasRightToDo = false;
            return true;
        }
        return false;
    }

    public boolean wasRightJoystickFlickedUp(){
        if(rightJoystickWasUpToDo){
            rightJoystickWasUpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasRightJoystickFlickedDown(){
        if(rightJoystickWasDownToDo){
            rightJoystickWasDownToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasRightJoystickFlickedLeft(){
        if(rightJoystickWasLeftToDo){
            rightJoystickWasLeftToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasOptionsPressed(){
        if(optionsBumpToDo){
            optionsBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasStartPressed(){
        if(startBumpToDo){
            startBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasBackPressed(){
        if(backBumpToDo){
            backBumpToDo = false;
            return true;
        }
        return false;
    }

    public boolean wasHomePressed(){
        if(homeBumpToDo){
            homeBumpToDo = false;
            return true;
        }
        return false;
    }
}


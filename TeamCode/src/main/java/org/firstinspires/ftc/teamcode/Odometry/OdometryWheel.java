package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.hardware.DcMotor;

public class OdometryWheel {
    private double diameter;
    private int CPR;

    public DcMotor motor;

    public OdometryWheel(double d, int cpr, DcMotor motor){
//        The left and right odometry wheels must be equidistant with to the center of rotation and
//        perpendicular to the y axis with respect to the robot
        this.diameter = d;
        this.CPR = cpr;
        this.motor = motor;
    }

    public double getInches(int ticks){
        return (double)ticks / CPR * diameter * Math.PI;
    }

    public double getDiameter() {
        return diameter;
    }

    public double getCircumference(){
        return diameter * Math.PI;
    }

    public int getCPR() {
        return CPR;
    }
}

package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.io.File;

import RMath.Point;
import RMath.PolarPoint;
import RMath.Util;
import RMath.Vector;

public class OdometryLocalizer extends Thread {

    private double x;
    private double y;
    private double r;
    public boolean rightReversed = false;
    public boolean strafeReversed = false;
    public boolean leftReversed = false;
    private LinearOpMode opMode;
    private Vector velocity = new Vector(new Point(0, 0));

    private BNO055IMU imu;


    private OdometryWheel left, right, strafe;
    private ExpansionHubEx leftHub, rightHub;
    private String telemetry = "";

    private double ticksPerDegree;
    private double strafeTicksPerDegree;

    //    If the user wants to start at a specific location

    public OdometryLocalizer(OdometryWheel left,
                             OdometryWheel right,
                             OdometryWheel strafe,
                             ExpansionHubEx rightHub,
                             ExpansionHubEx leftHub,
                             LinearOpMode opMode) {
        set(0, 0, 0);
        this.left = left;
        this.right = right;
        this.strafe = strafe;
        this.opMode = opMode;
        this.rightHub = rightHub;
        this.leftHub = leftHub;

        start();
    }

    public OdometryLocalizer(OdometryWheel left,
                             OdometryWheel right,
                             OdometryWheel strafe,
                             ExpansionHubEx rightHub,
                             ExpansionHubEx leftHub,
                             LinearOpMode opMode,
                             BNO055IMU imu) {
        set(0, 0, 0);
        this.left = left;
        this.right = right;
        this.strafe = strafe;
        this.opMode = opMode;
        this.rightHub = rightHub;
        this.leftHub = leftHub;
        this.imu = imu;

        startIMU();

        start();
    }

    public OdometryWheel getLeftMount() {
        return left;
    }

    public OdometryWheel getRightMount() {
        return right;
    }

    public OdometryWheel getStrafeMount() {
        return strafe;
    }

    public void set(double x, double y, double r) {
        this.x = x;
        this.y = y;
        this.r = r;
    }

    public void set(Point p, double r) {
        set(p.x, p.y, r);
    }

    public void run() {


        ExpansionHubMotor rightHubMotor = (ExpansionHubMotor) left.motor;
        ExpansionHubMotor strafeHubMotor = (ExpansionHubMotor) strafe.motor;
        ExpansionHubMotor leftHubMotor = (ExpansionHubMotor) right.motor;

        boolean isLeftHubMotorRightHub = true;
        boolean isStrafeHubMotorRightHub = true;
        boolean isRightHubMotorRightHub = true;

        RevBulkData rightData = rightHub.getBulkInputData();
        RevBulkData leftData = leftHub.getBulkInputData();

        int ixl;
        int ixc;
        int ixr;

        try {
            ixl = rightData.getMotorCurrentPosition(leftHubMotor);
        } catch (RuntimeException e) {
            ixl = leftData.getMotorCurrentPosition(leftHubMotor);
            isLeftHubMotorRightHub = false;
        }

        try {
            ixc = rightData.getMotorCurrentPosition(strafeHubMotor);
        } catch (RuntimeException e) {
            ixc = leftData.getMotorCurrentPosition(strafeHubMotor);
            isStrafeHubMotorRightHub = false;
        }

        try {
            ixr = rightData.getMotorCurrentPosition(rightHubMotor);
        } catch (RuntimeException e) {
            ixr = leftData.getMotorCurrentPosition(rightHubMotor);
            isRightHubMotorRightHub = false;
        }

        int fxl = ixl;
        int fxc = ixc;
        int fxr = ixr;

        long initalTime = System.currentTimeMillis();
        long finalTime = initalTime;
        double deltaTime = 1;

        readFiles();

        while (!opMode.isStopRequested()) {

            finalTime = System.currentTimeMillis();
            deltaTime = (double) (finalTime - initalTime) / 1000d;

            rightData = rightHub.getBulkInputData();
            leftData = leftHub.getBulkInputData();

            if (isLeftHubMotorRightHub)
                fxl = rightData.getMotorCurrentPosition(leftHubMotor);
            else
                fxl = leftData.getMotorCurrentPosition(leftHubMotor);

            if (isStrafeHubMotorRightHub)
                fxc = rightData.getMotorCurrentPosition(strafeHubMotor);
            else
                fxc = leftData.getMotorCurrentPosition(strafeHubMotor);

            if (isRightHubMotorRightHub)
                fxr = rightData.getMotorCurrentPosition(rightHubMotor);
            else
                fxr = leftData.getMotorCurrentPosition(rightHubMotor);

            fxl = left.motor.getCurrentPosition();
            fxc = strafe.motor.getCurrentPosition();
            fxr = right.motor.getCurrentPosition();

            final int deltaPosLeft = fxl - ixl;
            final int deltaPosCenter = fxc - ixc;
            final int deltaPosRight = fxr - ixr;

            ixl = fxl;
            ixc = fxc;
            ixr = fxr;

            final double initialX = x;
            final double initialY = y;

            update(deltaPosRight, deltaPosLeft, deltaPosCenter);

            final double deltaX = x - initialX;
            final double deltaY = y - initialY;

            velocity = new Vector(new Point(deltaX / deltaTime, deltaY / deltaTime));
            initalTime = finalTime;
        }
    }

    private void update(int rightEncoderTranslate, int leftEncoderTranslate, int strafeEncoderTranslate) {
        String data = "";

        rightEncoderTranslate *= (rightReversed ? -1 : 1);
        leftEncoderTranslate *= (leftReversed ? -1 : 1);
        strafeEncoderTranslate *= (strafeReversed ? -1 : 1);

//        double rInches = right.getInches(rightEncoderTranslate) * (rightReversed ? -1 : 1);
//        double lInches = left.getInches(leftEncoderTranslate) * (leftReversed ? -1 : 1);
//        double sInches = strafe.getInches(strafeEncoderTranslate) * (strafeReversed ? -1 : 1);
//        data += "rInches: " + rInches;
//        data += "\nlInches: " + lInches;
//        data += "\nsInches: " + sInches;
//
//        double diameter = Util.dist(right.getPosition().x, right.getPosition().y) * 2;
//        data += "\ndiameter: " + diameter;
//
////        These are values that have been compensated for the offset of odometry wheels
//
////        The thetas are the angle offset that the wheels are from the tangent of the circle with
////        diameter of the robot.
//        final double leftTheta = Util.angle(new Point(0, 0), left.getPosition());
//        final double rightTheta = Util.angle(new Point(0, 0), right.getPosition());
//        final double strafeTheta = Util.angle(new Point(0, 0), strafe.getPosition());
//        data += "\nleftTheta: " + Math.toDegrees(leftTheta);
//        data += "\nrightTheta: " + Math.toDegrees(rightTheta);
//        data += "\nstrafeTheta: " + Math.toDegrees(strafeTheta);
//
////        This is solving for the hypotenuse vector relative to the wheel to find how much the robot
////        has rotated
//        final double lRot = -lInches / Math.cos(leftTheta);
//        final double rRot = rInches / Math.cos(rightTheta);
//        data += "\nlRot: " + lRot;
//        data += "\nrRot: " + rRot;
//
////        How much distance of the circle that the robot as traveled
//        final double distRot = (rRot - lRot) / 2;
//        data += "\ndistRot :" + distRot;
////        How many degrees the robot has rotated
//        final double deltaRot = distRot / (Math.PI * diameter) * 360;
//        data += "\ndeltaRot: " + deltaRot;
//
////        How much distance the wheels would have moved if the robot didn't change
////        in the forward direction
//        final double fRot = distRot * Math.abs(Math.cos(rightTheta));
//        final double strafeDiameter = Util.dist(strafe.getPosition().x, strafe.getPosition().y) * 2;
//        final double sRot = deltaRot / 360 * strafeDiameter * Math.cos(strafeTheta);
//        data += "\nfRot: " + fRot;
//        data += "\nstrafeDiameter: " + strafeDiameter;
//        data += "\nsRot: " + sRot;
//
////        Finding the leftover values from the rotation to find forward displacement
//        final double lf = lInches - fRot;
//        final double rf = rInches + fRot;
//        data += "\nlf: " + lf;
//        data += "\nrf: " + rf;
//
        double deltaRot = (rightEncoderTranslate - leftEncoderTranslate) / (2 * ticksPerDegree);

        double rightDeltaForward = right.getInches(rightEncoderTranslate);
        double leftDeltaForward = left.getInches(leftEncoderTranslate);
        double strafeDelta = strafe.getInches((int) (strafeEncoderTranslate - deltaRot * strafeTicksPerDegree));

        r += deltaRot;

//        This double modulo is to loop the negatives as well
        r = ((r % 360) + 360) % 360;

        final double forwardDelta = (rightDeltaForward + leftDeltaForward) / 2;
        data += "\nright forward: " + rightDeltaForward;
        data += "\nleft forward: " + leftDeltaForward;
        data += "\nforward: " + forwardDelta;
        data += "\nstrafe: " + strafeDelta;
        data += "\nheading: " + deltaRot;

        if (deltaRot != 0) {
//           This is a method to generate an arc to better replicate the movement of the robot
//           It plugs our initial heading, final heading, delta strafe, and delta forward values
//           into a polar equation, which gives us a polar point output which we then convert into a
//           rectangular output
            final double initialHeading = Math.toRadians(r - deltaRot);
            final double finalHeading = Math.toRadians(r);
            final double turnRadius = forwardDelta / Math.toRadians(deltaRot);
            PolarPoint destination = new PolarPoint(turnRadius + strafeDelta, finalHeading);
            Point start = new Point(new PolarPoint(turnRadius, initialHeading));
            Point relation = start.getRelation(new Point(destination));
            x += relation.x;
            y += relation.y;
            data += "\nhas rotated";
        } else {
            data += "\nhas not rotated";
            PolarPoint destination = new PolarPoint(new Point(strafeDelta, forwardDelta));
            destination.theta += r;
            data += "\ndestination: " + destination.toString();
            data += "\ndestination xy: " + new Point(destination).toString();
            x += new Point(destination).x;
            y += new Point(destination).y;
        }

        data += "\nspeed: " + velocity.getMagnitude();
//        Vector deltaPos = new Vector(new Point(s, f));
//        deltaPos.setTheta(deltaPos.getTheta() + r);
//
//        x += deltaPos.getX();
//        y += deltaPos.getY();

        telemetry = data + "\n";
    }

    public String getTelemetry() {
        return telemetry;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public Vector getVelocity() {
        return velocity;
    }

    public Point getPosition() {
        return new Point(x, y);
    }

    public double getHeading() {
        return r;
    }

    private void startIMU() {
        Thread t = new Thread(() -> {
            final AxesReference ref = AxesReference.EXTRINSIC;
            final AxesOrder order = AxesOrder.XYZ;
            final AngleUnit unit = AngleUnit.DEGREES;
            double imuHeading = imu.getAngularOrientation(ref, order, unit).thirdAngle;

            while (!opMode.isStopRequested()) {
                double newHeading = imu.getAngularOrientation(ref, order, unit).thirdAngle;
                if (imuHeading != newHeading) {
                    imuHeading = newHeading;
                    r = Util.loop(imuHeading, 0, 360);
                }
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    break;
                }
            }
        });

        t.setPriority(MIN_PRIORITY);

        t.start();

    }

    private void readFiles() {
        AppUtil instance = AppUtil.getInstance();
        File ticksPerDegreeFile = instance.getSettingsFile("odoTicksPerDegree.txt");
        File strafeTicksPerDegreeFile = instance.getSettingsFile("odoStrafeTicksPerDegree.txt");

        ticksPerDegree = Double.parseDouble(ReadWriteFile.readFile(ticksPerDegreeFile).trim());
        strafeTicksPerDegree = Double.parseDouble(ReadWriteFile.readFile(strafeTicksPerDegreeFile).trim());
    }

}

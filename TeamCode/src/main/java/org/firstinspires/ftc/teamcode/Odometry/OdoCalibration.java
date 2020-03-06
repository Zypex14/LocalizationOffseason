package org.firstinspires.ftc.teamcode.Odometry;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Components.OdometryBaseOpmode;

import java.io.File;

import RMath.Util;

@Autonomous(group="Odometry")
public class OdoCalibration extends OdometryBaseOpmode {

    private boolean LEFT_REVERSED = false;
    private boolean RIGHT_REVERSED = false;
    private boolean STRAFE_REVERSED = false;

    @Override

    public void run() {

        DcMotor left = fl;
        DcMotor strafe = br;
        DcMotor right = fr;

        File ticksPerDegreeFile = AppUtil.getInstance().getSettingsFile("odoTicksPerDegree.txt");
        File strafeTicksPerDegreeFile = AppUtil.getInstance().getSettingsFile("odoStrafeTicksPerDegree.txt");

        final int leftInitialPosition = left.getCurrentPosition();
        final int strafeInitialPosition = strafe.getCurrentPosition();
        final int rightInitialPosition = right.getCurrentPosition();

        final double initialHeading = getHeading();

//        Turn the robot approx. 90 degrees
        for(double h = 0; Math.abs(h - initialHeading) < 90; h = getHeading()) {

            double error = Util.absCap(getAngleDiff(90, h - initialHeading) / 45, 0.1, 0.3);
            drivetrain.move(-error, error, -error, error);

            telemetry.addData("Heading", h);
            telemetry.addData("Left wheel position", left.getCurrentPosition());
            telemetry.addData("Strafe wheel position", strafe.getCurrentPosition());
            telemetry.addData("Right wheel position", right.getCurrentPosition());
            telemetry.update();
        }

        drivetrain.stop();

        sleep(1000);

//        Finding the change in position of all the encoders
        final int leftDeltaPosition = (left.getCurrentPosition() - leftInitialPosition) * (LEFT_REVERSED? -1 : 1);
        final int strafeDeltaPosition = (strafe.getCurrentPosition() - strafeInitialPosition) * (STRAFE_REVERSED? -1 : 1);
        final int rightDeltaPosition = (right.getCurrentPosition() - rightInitialPosition) * (RIGHT_REVERSED? -1 : 1);

//        The change in heading
        final double deltaHeading = getHeading() - initialHeading;

//        We take the minimum value of each encoder because the robot might have moved forward a
//        a little bit, causing the encoders to read unwanted values. Taking the min will only be
//        the values that were because of turning
        final double turnDeltaPosition = Math.abs(rightDeltaPosition - leftDeltaPosition) / 2;

//        These are the ratios that we need to store in the files, so we can use them for calculating
//        our localized position
        double ticksPerDegree = Math.abs(turnDeltaPosition / deltaHeading);
        double strafeTicksPerDegree = Math.abs(strafeDeltaPosition / deltaHeading);

//        Writing to the files so they can be used
        ReadWriteFile.writeFile(ticksPerDegreeFile, Double.toString(ticksPerDegree));
        ReadWriteFile.writeFile(strafeTicksPerDegreeFile, Double.toString(strafeTicksPerDegree));

        ticksPerDegree = Double.parseDouble(ReadWriteFile.readFile(ticksPerDegreeFile).trim());
        strafeTicksPerDegree = Double.parseDouble(ReadWriteFile.readFile(strafeTicksPerDegreeFile).trim());

        while(opModeIsActive()){
            telemetry.addData("ticksPerDegree", ticksPerDegree);
            telemetry.addData("strafeTicksPerDegree", strafeTicksPerDegree);
            telemetry.addData("turnDeltaPosition", turnDeltaPosition);
            telemetry.addData("deltaHeading", deltaHeading);
            telemetry.update();
        }

    }

    @Override
    public void initialize() {
        mapHardware();
    }

    public double getHeading(){
        return imu.getAngularOrientation().firstAngle;
    }
}

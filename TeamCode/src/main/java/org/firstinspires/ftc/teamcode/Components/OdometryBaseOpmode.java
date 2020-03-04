package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Odometry.OdometryLocalizer;

public abstract class OdometryBaseOpmode extends LinearOpMode {

    protected DcMotor fl, fr, bl, br;
    protected DcMotor odoLeft, odoStrafe, odoRight;

    protected OdometryLocalizer localizer;

    public void initOdo(){
        localizer = new OdometryLocalizer();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");


    }
}

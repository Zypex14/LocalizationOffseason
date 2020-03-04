//TODO fix pos decay
//TODO make turn regulate for diagonal
package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Odometry.OdometryLocalizer;

import RMath.Point;
import RMath.Util;
import RMath.Vector;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;

public class Mecanum {
    public DcMotor fl;
    public DcMotor fr;
    public DcMotor bl;
    public DcMotor br;
    public BNO055IMU imu;

    private double currentPower = 1.0;
    public double wheelDiam = 4.0;
    public int CPR;
    public double strafeRatio = 71 + 3./7.;
    public double diagonalRatio = 1000. / 9.;

    public double angleTolerance = 10;
    public double turnDecay = 0.5;
    public double moveDecay = 0.5;
    public boolean decayMovement = false;
    public boolean decayTurn = true;
    public boolean regulateHeading = false;

    public double moveMin = 0.2;
    public double turnMin = 0.1;
    public double moveMax = 1;
    public double turnMax = 1;

    public int moveSleep = 100;

    public final double MAX_SPEED = 70.6;

    public Telemetry telemetry;

    public Mecanum(DcMotor fl,
                   DcMotor fr,
                   DcMotor bl,
                   DcMotor br,
                   int CPR,
                   BNO055IMU imu,
                   Telemetry t) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
        this.CPR = CPR;
        this.imu = imu;

        telemetry = t;

        brake(true);
        resetEncoders();
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stop();
    }

    public void brake(boolean b) {
        if (b) {
            fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }


    public void move(double fl, double fr, double bl, double br) {
        this.fl.setPower(fl);
        this.fr.setPower(fr);
        this.bl.setPower(bl);
        this.br.setPower(br);
    }

    public void stop() {
        move(0, 0, 0, 0);
    }


    public void setCurrentPower(double cp) {
        currentPower = Math.min(1.0, Math.abs(cp));
    }

    public double getCurrentPower() {
        return currentPower;
    }

    //    Standard drive method
    public void drive(double leftStickX, double leftStickY, double rightStickX) {

        final double direction = Math.atan2(leftStickY, -leftStickX) - Math.PI / 4.0;
        final double speed = Util.dist(leftStickX, leftStickY);

        final double fl = speed * Math.sin(direction) + rightStickX;
        final double fr = speed * Math.sin(direction) - rightStickX;
        final double bl = speed * Math.cos(direction) + rightStickX;
        final double br = speed * Math.cos(direction) - rightStickX;

        double maxWheelPower = Math.abs(fr);
        if(Math.abs(fl) > maxWheelPower) maxWheelPower = Math.abs(fl);
        if(Math.abs(bl) > maxWheelPower) maxWheelPower = Math.abs(bl);
        if(Math.abs(br) > maxWheelPower) maxWheelPower = Math.abs(br);

        maxWheelPower = Math.min(maxWheelPower, speed + Math.abs(rightStickX));

        final double multiplier = (speed + Math.abs(rightStickX)) / maxWheelPower;

//        final double multiplier = 1;
        move(fl * currentPower * multiplier,
                fr * currentPower * multiplier,
                bl * currentPower * multiplier,
                br * currentPower * multiplier
        );
    }

    //    Drive method to drive in absolute direction
    public void absoluteDrive(double x, double y, double rot, double heading) {

        rot = Util.cap(rot, -1, 1);

//        The + Math.PI / 4.0 is to account for the strafing wheels
        final double direction = Math.atan2(y, -x) + Math.PI / 4.0 + Math.PI / 2;

//        The local direction is to account for the direction the robot is pointing in relative to the field
        final double localDirection = direction - Math.toRadians(heading);
        final double requestedSpeed = Math.min(1.0, Math.sqrt(x * x + y * y));

//        motorMax is to have a max requested power value that the code can
//        use to divert power from moving to rotate as well without
//        going past the max limit for power
        final double motorMax = (requestedSpeed + Math.abs(rot));
        final double rotPower = (rot * Math.min(1.0, motorMax)) / motorMax;
        final double speed = (requestedSpeed * Math.min(1.0, motorMax)) / motorMax;

//        Setting moving values while regulating them to save power for rotation
        double fl = speed * Math.cos(localDirection);
        double fr = speed * Math.cos(localDirection);
        double bl = speed * Math.sin(localDirection);
        double br = speed * Math.sin(localDirection);

//        Adding rotation values while regulating them to save power for moving
        fl += rotPower;
        fr -= rotPower;
        bl += rotPower;
        br -= rotPower;

        fl *= currentPower;
        fr *= currentPower;
        bl *= currentPower;
        br *= currentPower;

//        Capping the values to keep them from going over max speed
        fl = fl > 1 ? 1 : fl < -1 ? -1 : fl;
        fr = fr > 1 ? 1 : fr < -1 ? -1 : fr;
        bl = bl > 1 ? 1 : bl < -1 ? -1 : bl;
        br = br > 1 ? 1 : br < -1 ? -1 : br;

        this.fl.setPower(-fl);
        this.fr.setPower(-fr);
        this.bl.setPower(-bl);
        this.br.setPower(-br);
    }

//    Uses velocity PID to maintain target speed, and travels in absolute direction, rather than
//    relative direction
    public void odoDrive(double x, double y, double rot, OdometryLocalizer tm){

        rot = Util.cap(rot, -1, 1);

        Vector targetVel = new Vector(new Point(x, y));
        targetVel.setMagnitude(targetVel.getMagnitude() * MAX_SPEED);

//        Finding the velocity needed to be applied to the current velocity to get the target velocity
        Vector appliedVel = new Vector(
                new Point(targetVel.getX() - tm.getVelocity().getX(),
                        targetVel.getY() - tm.getVelocity().getY()
                )
        );

        appliedVel.setMagnitude(appliedVel.getMagnitude() / MAX_SPEED);

//        The local direction is to account for the direction the robot is pointing in relative to the field
        final double localDirection = Math.toRadians(tm.getHeading()) - appliedVel.getTheta() + Math.PI / 4.0;
        final double requestedSpeed = Math.min(1.0, Math.sqrt(x * x + y * y));

//        motorMax is to have a max requested power value that the code can
//        use to divert power from moving to rotate as well without
//        going past the max limit for power
        final double motorMax = (requestedSpeed + Math.abs(rot));
        final double rotPower = (rot * Math.min(1.0, motorMax)) / motorMax;
        final double speed = (requestedSpeed * Math.min(1.0, motorMax)) / motorMax;

//        Setting moving values while regulating them to save power for rotation
        double fl = speed * Math.sin(localDirection);
        double fr = speed * Math.cos(localDirection);
        double bl = speed * Math.cos(localDirection);
        double br = speed * Math.sin(localDirection);

//        Adding rotation values while regulating them to save power for moving
        fl += rotPower;
        fr -= rotPower;
        bl += rotPower;
        br -= rotPower;

        fl *= currentPower;
        fr *= currentPower;
        bl *= currentPower;
        br *= currentPower;

//        Capping the values to keep them from going over max speed
        fl = fl > 1 ? 1 : fl < -1 ? -1 : fl;
        fr = fr > 1 ? 1 : fr < -1 ? -1 : fr;
        bl = bl > 1 ? 1 : bl < -1 ? -1 : bl;
        br = br > 1 ? 1 : br < -1 ? -1 : br;

        this.fl.setPower(-fl);
        this.fr.setPower(-fr);
        this.bl.setPower(-bl);
        this.br.setPower(-br);
    }

    private double getCumulativeHeading() {
        AxesReference reference = AxesReference.EXTRINSIC;
        AxesOrder order = AxesOrder.XYZ;
        AngleUnit unit = AngleUnit.DEGREES;

        double heading = imu.getAngularOrientation(reference, order, unit).thirdAngle;
        return heading;
    }

    private double getHeading() {
        return Util.loop(getCumulativeHeading(), -180, 180);
    }

    public void resetEncoders() {
        DcMotor.RunMode mode = fl.getMode();
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(mode);
    }

    public void setMode(DcMotor.RunMode mode) {
        fl.setMode(mode);
        fr.setMode(mode);
        bl.setMode(mode);
        br.setMode(mode);
    }

    public void turn(double degrees) {
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double current = getCumulativeHeading();
        double target = current - degrees;

        while (Math.abs(target - current) > angleTolerance) {
            double power = (target - current) / 360 * 1 / turnDecay;

            power = Util.absCap(power, turnMin, turnMax);

            move(-power, power, -power, power);

            current = getCumulativeHeading();

            telemetry.addData("heading", current);
            telemetry.addData("target", target);
            telemetry.addData("power", power);
            telemetry.update();
        }

        stop();
        sleep(moveSleep);
    }

    public void forward(double inches) {
        final double circ = Math.PI * wheelDiam;
        final double d = (inches / circ) * CPR * 2;

        resetEncoders();

        fl.setTargetPosition((int) d);
        fr.setTargetPosition((int) d);
        bl.setTargetPosition((int) d);
        br.setTargetPosition((int) d);

        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double initialHeading = getHeading();

        fl.setPower(currentPower);
        fr.setPower(currentPower);
        bl.setPower(currentPower);
        br.setPower(currentPower);


//        Waiting until the motors have moved and making adjustments based on user settings
        while (fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy()) {

            double power = currentPower;
            double turn = 0;
            if (decayMovement)
                power *= (initialHeading - getHeading()) / 360 * 1 / moveDecay;
            if (regulateHeading)
                turn = (initialHeading - getHeading()) / 360 * 1 / turnDecay;

            power = Util.absCap(power, moveMin, moveMax);

            fl.setPower(power + turn);
            fr.setPower(power - turn);
            bl.setPower(power + turn);
            br.setPower(power - turn);

            telemetry.addLine("forward");
            telemetry.update();
        }

        stop();
        sleep(moveSleep);
    }

    public void manualTurn(double sec){
        ElapsedTime timer = new ElapsedTime();
        fl.setMode(RUN_WITHOUT_ENCODER);
        bl.setMode(RUN_WITHOUT_ENCODER);
        fr.setMode(RUN_WITHOUT_ENCODER);
        br.setMode(RUN_WITHOUT_ENCODER);

        timer.startTime();

        while(timer.seconds() < sec){
            fl.setPower(-.65);
            bl.setPower(-.65);
            fr.setPower(.65);
            br.setPower(.65);
        }
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

        while (fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy()) {
            telemetry.addLine("brute Force");
            telemetry.update();
        }

        stop();


    }
    public void manualTurnRed(double sec){
        ElapsedTime timer = new ElapsedTime();
        fl.setMode(RUN_WITHOUT_ENCODER);
        bl.setMode(RUN_WITHOUT_ENCODER);
        fr.setMode(RUN_WITHOUT_ENCODER);
        br.setMode(RUN_WITHOUT_ENCODER);

        timer.startTime();

        while(timer.seconds() < sec){
            fl.setPower(.65);
            bl.setPower(.65);
            fr.setPower(-.65);
            br.setPower(-.65);
        }
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

        while (fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy()) {
            telemetry.addLine("brute Force");
            telemetry.update();
        }

        stop();


    }

    public void strafe(double inches) {
        final double d = (inches * strafeRatio);

        resetEncoders();

        fl.setTargetPosition((int) d);
        fr.setTargetPosition(-(int) d);
        bl.setTargetPosition(-(int) d);
        br.setTargetPosition((int) d);

        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setPower(currentPower);
        fr.setPower(currentPower);
        bl.setPower(currentPower);
        br.setPower(currentPower);

        double initialHeading = getHeading();

        //        Waiting until the motors have moved
        while (fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy()) {

            double power = currentPower;
            double turn = 0;
            if (decayMovement)
                power *= (initialHeading - getHeading()) / 360 * 1 / moveDecay;
            if (regulateHeading)
                turn = (initialHeading - getHeading()) / 360 * 1 / turnDecay;

            power = Util.absCap(power, moveMin, moveMax);

            fl.setPower(power + turn);
            fr.setPower(power - turn);
            bl.setPower(power + turn);
            br.setPower(power - turn);

            telemetry.addLine("strafe");
            telemetry.update();
        }

        stop();
        sleep(moveSleep);
    }

    public void diagonal(double inches, DiagonalAxis axis) {
        final double d = (inches * diagonalRatio);

        resetEncoders();

        DcMotor motor1;
        DcMotor motor2;
        switch(axis){
            case POSITIVE:
                motor1 = fl;
                motor2 = br;
                fr.setTargetPosition(0);
                bl.setTargetPosition(0);
                break;
            default:
                motor1 = fr;
                motor2 = bl;
                fl.setTargetPosition(0);
                br.setTargetPosition(0);
                break;
        }

        motor1.setTargetPosition((int) d);
        motor2.setTargetPosition((int) d);

        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fl.setPower(currentPower);
        fr.setPower(currentPower);
        br.setPower(currentPower);
        bl.setPower(currentPower);

        double initialHeading = getHeading();
        //        Waiting until the motors have moved
        while (motor1.isBusy() && motor2.isBusy()) {

            double power = currentPower;
            double turn = 0;
            if (decayMovement)
                power *= (initialHeading - getHeading()) / 360 * 1 / moveDecay;
            if (regulateHeading)
                turn = (initialHeading - getHeading()) / 360 * 1 / turnDecay;

            power = Util.absCap(power, moveMin, moveMax);

            motor1.setPower(power);
            motor2.setPower(power);

            telemetry.addLine("diagonal");
            telemetry.update();
        }

        stop();
        sleep(moveSleep);
    }

    private void sleep(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public enum DiagonalAxis {
        //        Along the y = x line
        POSITIVE,
        //        Along the y = -x line
        NEGATIVE
    }
}

package org.firstinspires.ftc.teamcode.Odometry;

import org.firstinspires.ftc.teamcode.Components.Mecanum;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.ArrayList;

import RMath.Circle;
import RMath.Point;
import RMath.Segment;
import RMath.Util;
import RMath.Vector;

public class PurePursuit extends Thread {

    private Mecanum mecanum;
    private ArrayList<PursuitPoint> points = new ArrayList<>();
    private ArrayList<PursuitPoint> input = new ArrayList<>();
    private boolean on;
    private OdometryLocalizer localizer;
    //    This is in inches
    private Circle lookahead;
    private String telemetry, telemetryGen;
    public double angleTolerance = 5, moveTolerance = 0.5;
    public double slowDownDistance = 10;
    private double rotVel = 0;

    private boolean busy = false;

    public PurePursuit(Mecanum m, OdometryLocalizer tm, double lookahead) {
        mecanum = m;
        this.localizer = tm;
        this.lookahead = new Circle(tm.getPosition(), lookahead);
        input.add(new PursuitPoint(tm.getX(), tm.getY()));
        cancel();
    }

    public void addPoint(double x, double y) {
        input.add(new PursuitPoint(x, y));
    }

    //    Adding a point with a custom lookahead
    public void addPointLookahead(double x, double y, double lookahead) {
        input.add(new PursuitPoint(x, y, lookahead, true));
    }

    //    Adding a point with a custom rotation
    public void addPointRotation(double x, double y, double rotation) {
        input.add(new PursuitPoint(x, y, rotation, false));
    }

    //    Adding a point with both lookahead and rotation
    public void addPoint(double x, double y, double rotation, double lookahead) {
        input.add(new PursuitPoint(x, y, rotation, lookahead));
    }

    public void addAction(Runnable r) {
        input.get(input.size() - 1).addAction(r, true);
    }

    public void addInterruptingAction(Runnable r) {
        input.get(input.size() - 1).addAction(r, false);
    }

    public void addInterruptingAction(Runnable r, double moveTolerance) {
        input.get(input.size() - 1).addAction(r, false, moveTolerance);
    }

    //    If the robot has finished traveling
    public boolean isBusy() {
        return busy;
    }

    public OdometryLocalizer getMounts() {
        return localizer;
    }

    public ArrayList<PursuitPoint> getPoints() {

        return points;

    }

    //    Take the input list and add it to the main path
    public void move() {
        points = new ArrayList<>(input);
        busy = true;
        input.clear();
        input.add(new PursuitPoint(localizer.getX(), localizer.getY()));
    }

    //    Deletes the path that the robot is traveling
    public void cancel() {
        points = new ArrayList<>();
        mecanum.stop();
        busy = false;
    }

    public void shutDown() {
        cancel();
        on = false;
        busy = false;
    }

    public String getTelemetry() {
        return telemetry;
    }

    public void setLookahead(double radius) {
        lookahead = new Circle(localizer.getPosition(), radius);
    }

    public double getLookahead() {
        return lookahead.r;
    }

    public void run() {
        on = true;

//        Our lookahead is basically a circle, and the lookahead point will be the intersection
//        of the circle and the line that the robot is following. Since there are two intersections usually,
//        the point closest to the next segment is going to be the one chosen as lookahead point


//        tr is our target rotation or target heading
        double tr = 0;

//        Variable for telemetry
        int movements = 0;
        while (on) {

            try {
                String telemetryGen = "";

                lookahead.h = localizer.getX();
                lookahead.k = localizer.getY();

                Point followPoint;
                double distance = 0;
//                If there is still one segment left for the robot to travel to
                if (points.size() > 1) {
                    busy = true;

                    if (points.get(1).hasLookahead()) lookahead.r = points.get(1).lookahead;
//                    If the end of the segment is already in the lookahead, remove the point and
//                    restart the loop
                    boolean withinMovement = Util.dist(points.get(1).position, localizer.getPosition()) < lookahead.r;
                    boolean actionsEmpty = points.get(0).getActions().isEmpty();

                    double angleDiff;
                    boolean withinAngle = true;
                    if(points.get(1).hasRotation()){
                        double targetRot = points.get(1).rotation;
                        angleDiff = Math.abs(getAngleDiff(localizer.getHeading(), targetRot));
                        withinAngle = angleDiff < angleTolerance;
                    }

                    if (withinMovement && actionsEmpty && withinAngle) {
                        points.remove(0);
                        movements++;
                        continue;
                    }

                    if(withinMovement && !withinAngle){
                        followPoint = points.get(1).position;
                        distance = Util.dist(localizer.getPosition(), followPoint);
                    }else {
                        followPoint = getPathIntersection();
//                        Find the distance of all of the paths left
                        for (int i = 0; i < points.size() - 1; i++) {
//                        If the point has an action that's not a thread, end distance there so the robot
//                        stops
                            boolean hasStop = false;
                            for (PursuitPoint.Action a : points.get(i).getActions()) {
                                if (!a.isThread()) {
                                    hasStop = true;
                                    break;
                                }
                            }
                            if (hasStop) break;

                            Point start = points.get(i).position;
                            Point end = points.get(i + 1).position;
                            distance += Util.dist(start, end);
                        }
                    }

//

//                    If there is only one point left
                } else if (points.size() == 1) {

//                    Follow the final point in the path
                    followPoint = points.get(0).position;
                    PursuitPoint nearestPoint = points.get(0);
                    if (points.get(0).hasLookahead()) lookahead.r = points.get(0).lookahead;

                    busy = calcBusy(nearestPoint, tr);

                    distance = Util.dist(localizer.getPosition(), nearestPoint.position);

                } else {
                    followPoint = localizer.getPosition();
                    busy = false;
                }


                if (!points.isEmpty()) {

//                    Run any actions that the current point might have
                    if (!points.get(0).getActions().isEmpty()) {

                        PursuitPoint.Action action = points.get(0).getActions().get(0);

                        if (action.isThread()) {

                            new Thread(action::run).start();
                            points.get(0).getActions().remove(0);
                            tr = calcTargetHeading(followPoint);

                        } else {
                            if (!action.hasStarted()) {
                                double dist = Util.dist(localizer.getPosition(), points.get(0).position);

                                if(Double.isNaN(action.moveTolerance)){
                                    new Thread(action::run).start();
                                }else if(dist < moveTolerance){
                                    new Thread(action::run).start();
                                }

                                followPoint = points.get(0).position;
                                distance = Util.dist(localizer.getPosition(), followPoint);

                            } else if (action.isRunning()) {

                                followPoint = points.get(0).position;
                                distance = Util.dist(localizer.getPosition(), followPoint);
                                tr = localizer.getHeading();

                            } else if (action.hasStarted() && !action.isRunning()) {
                                points.get(0).getActions().remove(0);
                            }
                        }
                    } else{
                        tr = calcTargetHeading(followPoint);
                    }


                    Vector movement = calcMovement(followPoint, distance);

                    PursuitPoint nearestPoint = points.get(points.size() > 1 ? 1 : 0);

//            Rotation
                    double r = Util.loop(localizer.getHeading(), 0, 360);
//            Target Rotation
                    if (nearestPoint.hasRotation())
                        tr = nearestPoint.rotation;

                    goTo(movement, tr);

                    telemetryGen += "Speed: " + localizer.getVelocity().getMagnitude() + "\n";
                    telemetryGen += "Movements: " + movements + "\n";
                    telemetryGen += "Nearest point: " + nearestPoint.position.toString() + "\n";
                    telemetryGen += "Follow point: " + followPoint.toString() + "\n";
                    telemetryGen += "Lookahead: " + lookahead.r + "\n";
                    telemetryGen += "Heading: " + localizer.getHeading() + "\n";
                    telemetryGen += "Target Heading: " + tr + "\n";
                    telemetryGen += "Position: " + localizer.getPosition().toString() + "\n";
                    telemetryGen += "Movement Power: " + movement.getMagnitude() + "\n";
                    telemetryGen += "Movement Direction: " + Math.toDegrees(movement.getTheta()) + "\n";
                    telemetryGen += "Movement components: " + movement.toPoint().toString() + "\n";
//                    telemetryGen += "Path:\n";
//                    for (PursuitPoint p : points)
//                        telemetryGen += p.position.toString() + "\n";


                } else {
                    telemetryGen += "Idle\n";
                }

                telemetryGen += "Path length: " + points.size() + "\n";
                telemetry = telemetryGen;

            } catch (Exception e) {
                StringWriter sw = new StringWriter();
                e.printStackTrace(new PrintWriter(sw));
                telemetry = "Exception occured:\n" + sw.toString();
            }
        }

    }

    /**
     * @param tr    the target heading that the robot needs to be when it reaches the point
     */
    public void goTo(Vector movement, double tr) {


//                    Setting the rotational velocity to the difference in angles to correct heading
        double r = Util.loop(localizer.getHeading(), 0, 360);
        tr = Util.loop(tr, 0, 360);
        double targetRotVel = 0;

        targetRotVel = getAngleDiff(tr, r);

        targetRotVel /= 50 * mecanum.getCurrentPower();

        targetRotVel = Util.cap(targetRotVel, -1, 1);


        mecanum.absoluteDrive(
                -movement.getX(),
                movement.getY(),
//                            0, 0,
                targetRotVel,
//                            0,
                localizer.getHeading()
        );

    }

    /**
     * A method to calculate the target heading that will make the robot move the fastest
     *
     * @param target the point that the robot is going to
     * @return the target heading
     */
    private double calcTargetHeading(Point target) {

        double forward;
        double backward;
        double tr;
//        Point towards the final point in the path if there is only one point left
        if (points.size() == 1) {
            Point end = points.get(0).position;
            if (Util.dist(localizer.getPosition(), end) < 4) {
                forward = localizer.getHeading();
                backward = localizer.getHeading() + 180;
            } else {
                forward = Math.toDegrees(Util.angle(localizer.getPosition(), end)) - 90;
                backward = Math.toDegrees(Util.angle(localizer.getPosition(), end)) + 90;
            }
        } else {
//            Point towards the next segment if the robot is very close to its target, so there are
//            no oscillations
            if (lookahead.r < 4) {
                Point end = points.get(1).position;
                forward = Math.toDegrees(Util.angle(localizer.getPosition(), end)) - 90;
                backward = Math.toDegrees(Util.angle(localizer.getPosition(), end)) + 90;
            } else {
//                Point towards the target
                forward = Math.toDegrees(Util.angle(localizer.getPosition(), target)) - 90;
                backward = Math.toDegrees(Util.angle(localizer.getPosition(), target)) + 90;

            }
        }

        tr = forward;
        if (Math.abs(getAngleDiff(localizer.getHeading(), forward)) > 90)
            tr = backward;


        return tr;
    }

    private Point getPathIntersection() {

        Point followPoint;

//                    We are using this segment as the current part of the path that thr robot is on
//                    in order to find the follow point, we are finding the intersection between the
//                    lookahead circle and this segment
        Segment currentSegment = new Segment(points.get(0).position, points.get(1).position);
        Point[] intersections = Util.getIntersection(lookahead, currentSegment);
        Point closestPoint = localizer.getPosition().closestPoint(currentSegment);

//                    If the circle is too far from the segment, then follow the closest point on
//                    the segment
        if (Util.dist(closestPoint, localizer.getPosition()) > lookahead.r) {
            followPoint = closestPoint;
            telemetryGen += "Path outside of lookahead\n";
        } else if (intersections.length == 1) {
//                        If there is only one intersection, then follow that one
            followPoint = intersections[0];
            telemetryGen += "One intersection\n";
        } else {
//                        Compare the two intersections and travel towards the one that is closer
//                        to the destination
            telemetryGen += "Two intersections\n";
            if (Util.dist(points.get(1).position, intersections[0]) <
                    Util.dist(points.get(1).position, intersections[1])) {
                followPoint = intersections[0];
            } else {
                followPoint = intersections[1];
            }
        }

        return followPoint;

    }

    private boolean calcBusy(PursuitPoint finalPoint, double tr) {
        //                    If the robot's position is within the movement tolerance
        if (Util.dist(localizer.getPosition(), finalPoint.position) < moveTolerance) {
//                        If the final point of the path has a requested rotation
            if (finalPoint.hasRotation()) {
                tr = Util.loop(tr, 0, 360);
                double r = Util.loop(localizer.getHeading(), 0, 360);
//                If the robot is within the angle tolerance then set busy to false,
//                vise versa
                return (Math.abs(getAngleDiff(tr, r)) > angleTolerance);
            }

            return false;
        }

        return true;
    }

    private Vector calcMovement(Point followPoint, double distance) {
        Vector movement = new Vector(localizer.getPosition().getRelation(followPoint));

//        When the robot is within slowDownDistance, the power will go below 1
        double speedCoefficient = 1 / slowDownDistance / mecanum.getCurrentPower();

//        If the robot is stopped, increase the min power to break static friction
        double minPower = (localizer.getVelocity().getMagnitude() > 3) ? 0.2 : 0.3 / mecanum.getCurrentPower();
        movement.setMagnitude(Util.absCap(distance * speedCoefficient, minPower, 1));

        return movement;
    }

    /**
     * @param a1 the first angle
     * @param a2 the second angle
     * @return the shortest interval between the two angles
     */
    public double getAngleDiff(double a1, double a2) {
        a1 = Util.loop(a1, 0, 360);
        a2 = Util.loop(a2, 0, 360);

        double dist = a1 - a2;
        double shortest;
        if (Math.abs(dist) < 180)
            shortest = dist;
        else {
            if (dist > 0) shortest = dist - 360;
            else shortest = dist + 360;
        }

        return shortest;
    }


}

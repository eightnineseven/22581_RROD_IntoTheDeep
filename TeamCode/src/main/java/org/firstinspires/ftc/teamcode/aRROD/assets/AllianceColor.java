package org.firstinspires.ftc.teamcode.aRROD.assets;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;


public enum AllianceColor {
    RED, BLUE;

    private final Pose startingPose = new Pose(7.575, 87, Math.toRadians(0));

    public Pose convertPose(Pose pose) {
        if (this == BLUE)
            return pose;
        else
            return new Pose(144 - pose.getX(), pose.getY(), Math.PI - pose.getHeading());
    }

    public Point convertPoint(Point point) {
        return this == BLUE ? point : new Point(144 - point.getX(), point.getY(), Point.CARTESIAN);
    }

    public Pose getStartingPose() {
        return this.convertPose(startingPose);
    }
}

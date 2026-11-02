package org.firstinspires.ftc.teamcode.bobot.autonomous.paths;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import org.firstinspires.ftc.teamcode.bobot.states.Alliance;

public class AutoPaths {
    public final Pose start;
    private final Pose path1, path1Control1, path1Control2;
    private final Pose path1Segment1Start, path1Segment1End, path1Segment2Start, path1Segment2End;
    private final Pose point2Start, point2;
    private final Pose point3Start, point3;
    private final Pose point4Start, point4;
    private final Pose point5, point5Segment1Heading, point5Segment2Start, point5Segment2End;
    private final Pose point6Start, point6;

    public AutoPaths(Alliance alliance) {
        PoseFactory poses = alliance == Alliance.RED
                ? PoseFactory.degrees()
                : PoseFactory.degrees().rotateAround(72, 72, 180);

        start = poses.of(56, 8, 90);
        path1 = poses.of(8, 8, 180);
        path1Control1 = poses.of(49, 21, 0);
        path1Control2 = poses.of(42, 8, 0);
        path1Segment1Start = poses.of(8, 8, 90);
        path1Segment1End = poses.of(8, 8, 180);
        path1Segment2Start = poses.of(8, 8, 180);
        path1Segment2End = poses.of(8, 8, 180);
        point2Start = poses.of(8, 8, 180);
        point2 = poses.of(42, 122, 300);
        point3Start = poses.of(42, 122, 300);
        point3 = poses.of(47.5, 126, 90);
        point4Start = poses.of(47.5, 126, 90);
        point4 = poses.of(47.5, 131, 90);
        point5 = poses.of(47.5, 126, -80);
        point5Segment1Heading = poses.of(47.5, 126, 90);
        point5Segment2Start = poses.of(47.5, 126, 90);
        point5Segment2End = poses.of(47.5, 126, 280);
        point6Start = poses.of(47.5, 126, 280);
        point6 = poses.of(9, 118, 360);
    }

    public Path path1() {
        return Paths.curve(start, path1Control1, path1Control2, path1).heading(Interpolator.piecewise()
                .until(0.75, Interpolator.linear(path1Segment1Start, path1Segment1End))
                .until(1, Interpolator.linear(path1Segment2Start, path1Segment2End)));
    }

    public Path path2() { return Paths.line(point2Start, point2).linear(point2Start, point2); }

    public Path path3() { return Paths.line(point3Start, point3).linear(point3Start, point3); }

    public Path path4() { return Paths.line(point4Start, point4).linear(point4Start, point4); }

    public Path path5() {
        return Paths.line(point4, point5).heading(Interpolator.piecewise()
                .until(0.64, Interpolator.constant(point5Segment1Heading))
                .until(1, Interpolator.linear(point5Segment2Start, point5Segment2End)));
    }

    public Path path6() { return Paths.line(point6Start, point6).linear(point6Start, point6); }
}

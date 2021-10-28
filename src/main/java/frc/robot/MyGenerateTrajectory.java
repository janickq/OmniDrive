package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

public class MyGenerateTrajectory {

    //Generate trajectory given a list of waypoints
    public Trajectory generateTrajectory( List<Translation2d> wp, TrajectoryConfig config, double R) {

        List<Translation2d> wp2 = new ArrayList<Translation2d>();
        List<Pose2d> waypoints = new ArrayList<Pose2d>();
        Pose2d p;
        Translation2d t;

        //Create intermediate way points as A* output with right angle path
        // are not suitable for trajectory generator
        //Intermediate way points are R distance from the original waypoint.
        //This will cause the path generated to be a small curve between the two intermediate waypoints
        int N = wp.size();
        wp2.add(wp.get(0)); //Copy start point
        if (N>2) {
            for (int i=1; i<N-1; i++){
                double x_1 = wp.get(i-1).getX();
                double y_1 = wp.get(i-1).getY();
                double x0 = wp.get(i).getX();
                double y0 = wp.get(i).getY();
                double x1 = wp.get(i+1).getX();
                double y1 = wp.get(i+1).getY();
                double dx = x0 - x_1;
                double dy = y0 - y_1;
                double a = Math.atan2(dy, dx);
                double len = Math.sqrt(dx*dx + dy*dy);
                t = new Translation2d(x0-R*Math.cos(a), y0-R*Math.sin(a));
                wp2.add(t);

                dx = x1 - x0;
                dy = y1 - y0;
                a = Math.atan2(dy, dx);
                len = Math.sqrt(dx*dx + dy*dy);
                t = new Translation2d(x0+R*Math.cos(a), y0+R*Math.sin(a));
                wp2.add(t);

            }
        }
        wp2.add(wp.get(N-1)); //Copy end point

        //Calculate intermediate points angle.
        //This allows us to control the spline curve
        //wp2 = wp;
        for (int i=0; i<wp2.size(); i+=2){
            double x0 = wp2.get(i).getX();
            double y0 = wp2.get(i).getY();
            double x1 = wp2.get(i+1).getX();
            double y1 = wp2.get(i+1).getY();
            double dx = x1 - x0;
            double dy = y1 - y0;
            Rotation2d angle = new Rotation2d(Math.atan2(dy, dx));

            p = new Pose2d(x0, y0, angle);
            waypoints.add(p);
            //System.out.println(p);    //Print for debug
            p = new Pose2d(x1, y1, angle);
            waypoints.add(p);
            //System.out.println(p);
            //System.out.printf("%d x:%5.2f, y:%5.2f, a:%5.2f", i, x0, y0, Math.atan2(dy,dx));
        }

        //quintic hermite splines
        //Can control waypoints angle
        Trajectory traj = TrajectoryGenerator.generateTrajectory(waypoints, config);

        //Debug
        for (double i=0.0; i<=traj.getTotalTimeSeconds(); i+=0.02) {
            p = traj.sample(i).poseMeters;
            System.out.println(p);
        }
        //Clamped cubic spline
        //Did not work well. In fact it's really bad !!!!!!!!
        //Generates car-like path
        // Trajectory traj = TrajectoryGenerator.generateTrajectory(
        //     // Start at the origin facing the +X direction
        //     start,
        //     // Pass through these waypoints
        //     waypoints,
        //     // End 
        //     end,
        //     config);

        return traj;    
    }

}

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

public class Utils {

    private static AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);



    public static AprilTag getClosestAprilTag(List<AprilTag> tags, Pose2d location) {
        AprilTag closestTag = null; // this is ok, because as long as there are tags, we'll pick one.
        double closestDistance = Double.MAX_VALUE;
        for (AprilTag t : tags ) {
            Pose2d tagPose = t.pose.toPose2d();
            double distance = location.getTranslation().getDistance(tagPose.getTranslation());

            if ( distance < closestDistance){
                closestDistance = distance;
                closestTag = t;
            }
        }
        return closestTag;

    }

    /**
     * Maybe this is the best generic way to get the nearest tag
     * from the reef.
     * @param location
     * @return
     */
    public static AprilTag getClosestAprilTag( Pose2d location ) {
        return getClosestAprilTag(field.getTags(), location);
    }

    /**
     * An ~O(24) implementation to find the closest reeftag
     * -- n = 12 algae locations looped over at most 2x.
     * @param location
     * @return
     */
    public static AprilTag getClosestReefAprilTag( Pose2d location ) {
        List<AprilTag> reefTags = new ArrayList<>();
        List<Integer> tagIds = Arrays.asList(17,18,19,20,21,22,6,7,8,9,10,11);

        for ( AprilTag t: field.getTags()){
            if ( tagIds.contains(t.ID) ){
                reefTags.add(t);
            }
        }
        return getClosestAprilTag(reefTags, location);
    }
    
}

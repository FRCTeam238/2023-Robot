package frc.core238;

import edu.wpi.first.math.geometry.Pose3d;

public class PoseHelper {
    public static double[] PoseToArray(Pose3d pose)
    {
      double[] retval = new double[7];
      retval[0] = pose.getX();
      retval[1] = pose.getY();
      retval[2] = pose.getZ();
      retval[3] = pose.getRotation().getQuaternion().getW();
      retval[4] = pose.getRotation().getQuaternion().getX();
      retval[5] = pose.getRotation().getQuaternion().getY();
      retval[6] = pose.getRotation().getQuaternion().getZ();
      return retval;
    }
}

package frc.robot;

/**
 * Utility Class Methods
 */
class Utilities {
    /**
     * Returns 0.0 if value is less than the tolerance threshold value
     * 
     * @param value Value to check for being under tolerance
     * 
     * @param tolerance threshold value
     * 
     * @return value if above the tolerance threshold
     */
    public static double deadband(double value, double tolerance) {
        return Math.abs(value) > tolerance ? value : 0.0;
    }

    /**
     * Normalizes a vector
     * 
     * @param values vector to normalize
     * 
     * @return normalized vector: [-1, 1]
     */
    public static double[] normalize(double[] values) {
        double max_val = values[0];
        boolean normFlag = max_val > 1;
        for(double val : values) {
            if(Math.abs(val) > max_val) {
                max_val = Math.abs(val);
                normFlag = max_val > 1;
            }
        }
        if(normFlag) {
            for(int x = 0; x < values.length; x++) 
                values[x] /= max_val;
        }
        return values;
    }

    /**
         * Thanks Dan :) 
		 * Gets rotational error on [-180, 180]
		 * 
		 * @param alpha
		 *            First angle
		 * @param beta
		 *            Second Angle
		 * @return Rotational error
		 */
    public static double rotationalError(double alpha, double beta) {
        double ret = alpha - beta;
        if (ret < -180) {
            ret += 360;
        }
        if (ret > 180) {
            ret -= 360;
        }
        return -ret;
    }

    /**
     * Normalizes Angle
     * 
     * @param theta angle to normalize
     * 
     * @return normalized angle from [-180, 180]
     */
    public static double normalizeAngle(double theta) {
        if(theta > 180) {
            theta -= 360;
        } else if(theta < -180) {
            theta += 360;
        }

        return theta;
    }

    /**
     * Checks if value is within epsilon of targetValue
     * 
     * @param targetValue desired value
     * @param value current value
     * @param epsilon the range value can be within from targetValue
     * @return true if value is within epsilon range of targetValue
     */
    public static boolean withinEpsilon(double targetValue, double value, double epsilon) {
        return Math.abs(targetValue - value) < epsilon;
    }
}

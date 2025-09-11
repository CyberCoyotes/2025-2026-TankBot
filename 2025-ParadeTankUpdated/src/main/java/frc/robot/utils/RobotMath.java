package frc.robot.utils;

import edu.wpi.first.math.MathUtil;

/**
 * Utility class for common robot mathematical operations.
 */
public class RobotMath {
    
    /**
     * Applies deadband and exponential scaling to an input value.
     * @param input Raw input value
     * @param deadband Deadband threshold (0 to 1)
     * @param exponent Exponential scaling factor (typically 1-3)
     * @return Processed input value
     */
    public static double processInput(double input, double deadband, double exponent) {
        // Apply deadband
        input = MathUtil.applyDeadband(input, deadband);
        
        // Apply exponential scaling while preserving sign
        return Math.copySign(Math.pow(Math.abs(input), exponent), input);
    }
    
    /**
     * Limits the rate of change of a value.
     * @param current Current value
     * @param target Target value
     * @param maxChange Maximum change per call
     * @return Rate-limited value
     */
    public static double rateLimit(double current, double target, double maxChange) {
        double difference = target - current;
        
        if (Math.abs(difference) <= maxChange) {
            return target;
        }
        
        return current + Math.copySign(maxChange, difference);
    }
    
    /**
     * Normalizes an angle to be between -180 and 180 degrees.
     * @param angleDegrees Angle in degrees
     * @return Normalized angle
     */
    public static double normalizeAngle(double angleDegrees) {
        while (angleDegrees > 180) {
            angleDegrees -= 360;
        }
        while (angleDegrees < -180) {
            angleDegrees += 360;
        }
        return angleDegrees;
    }
    
    /**
     * Calculates the shortest angular distance between two angles.
     * @param fromAngle Starting angle in degrees
     * @param toAngle Target angle in degrees
     * @return Shortest angular distance in degrees
     */
    public static double angleDifference(double fromAngle, double toAngle) {
        double difference = toAngle - fromAngle;
        return normalizeAngle(difference);
    }
    
    /**
     * Converts encoder ticks to distance based on wheel parameters.
     * @param ticks Encoder ticks
     * @param ticksPerRevolution Ticks per wheel revolution
     * @param wheelDiameterMeters Wheel diameter in meters
     * @return Distance in meters
     */
    public static double ticksToDistance(double ticks, double ticksPerRevolution, double wheelDiameterMeters) {
        double revolutions = ticks / ticksPerRevolution;
        double circumference = wheelDiameterMeters * Math.PI;
        return revolutions * circumference;
    }
    
    /**
     * Converts distance to encoder ticks based on wheel parameters.
     * @param distanceMeters Distance in meters
     * @param ticksPerRevolution Ticks per wheel revolution
     * @param wheelDiameterMeters Wheel diameter in meters
     * @return Encoder ticks
     */
    public static double distanceToTicks(double distanceMeters, double ticksPerRevolution, double wheelDiameterMeters) {
        double circumference = wheelDiameterMeters * Math.PI;
        double revolutions = distanceMeters / circumference;
        return revolutions * ticksPerRevolution;
    }
    
    /**
     * Interpolates between two values.
     * @param start Starting value
     * @param end Ending value
     * @param t Interpolation factor (0 to 1)
     * @return Interpolated value
     */
    public static double lerp(double start, double end, double t) {
        return start + t * (end - start);
    }
    
    /**
     * Clamps a value between minimum and maximum bounds.
     * @param value Value to clamp
     * @param min Minimum bound
     * @param max Maximum bound
     * @return Clamped value
     */
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
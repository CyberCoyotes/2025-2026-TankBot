package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

/**
 * Test for DriveSubsystem deadband functionality
 */
public class DriveSubsystemTest {

    @Test
    public void testDeadbandFunctionality() {
        // Test that inputs within deadband are set to zero
        double deadband = Constants.Drive.JOYSTICK_DEADBAND;
        
        // Test small positive input (should be zeroed)
        double smallInput = 0.05;
        double result = MathUtil.applyDeadband(smallInput, deadband);
        assertEquals(0.0, result, 0.001, "Small positive input should be zeroed by deadband");
        
        // Test small negative input (should be zeroed)
        double smallNegativeInput = -0.05;
        result = MathUtil.applyDeadband(smallNegativeInput, deadband);
        assertEquals(0.0, result, 0.001, "Small negative input should be zeroed by deadband");
        
        // Test input at deadband threshold (should be zeroed)
        double thresholdInput = deadband;
        result = MathUtil.applyDeadband(thresholdInput, deadband);
        assertEquals(0.0, result, 0.001, "Input at deadband threshold should be zeroed");
        
        // Test input above deadband threshold (should not be zeroed)
        double largeInput = 0.5;
        result = MathUtil.applyDeadband(largeInput, deadband);
        assertTrue(result > 0, "Large input should not be zeroed by deadband");
        
        // Test input below negative deadband threshold (should not be zeroed)
        double largeNegativeInput = -0.5;
        result = MathUtil.applyDeadband(largeNegativeInput, deadband);
        assertTrue(result < 0, "Large negative input should not be zeroed by deadband");
    }
    
    @Test
    public void testDeadbandConstants() {
        // Verify deadband constant is reasonable
        double deadband = Constants.Drive.JOYSTICK_DEADBAND;
        assertTrue(deadband > 0, "Deadband should be positive");
        assertTrue(deadband < 0.5, "Deadband should be less than 50%");
        assertEquals(0.1, deadband, 0.001, "Deadband should be 10%");
    }
    
    @Test
    public void testDeadbandScaling() {
        // Test that deadband properly scales values above threshold
        double deadband = Constants.Drive.JOYSTICK_DEADBAND;
        double fullInput = 1.0;
        double result = MathUtil.applyDeadband(fullInput, deadband);
        
        // Full input should remain full after deadband
        assertEquals(1.0, result, 0.001, "Full scale input should remain full scale");
        
        // Input just above deadband should be scaled to start from 0
        double justAboveDeadband = deadband + 0.01;
        result = MathUtil.applyDeadband(justAboveDeadband, deadband);
        assertTrue(result > 0, "Input just above deadband should be positive");
        assertTrue(result < justAboveDeadband, "Input should be scaled down due to deadband");
    }
}
package frc.robot.subsystems.hood;

/** Utility conversions for migrating hood control from actuator pulse-width commands to motor rotations. */
public final class HoodConversions {
    private static final double MIN_PULSE_WIDTH_US = 1000.0;
    private static final double MAX_PULSE_WIDTH_US = 2000.0;
    private static final double ACTUATOR_STROKE_MM = 50.0;

    private HoodConversions() {}

    /**
     * Converts the legacy hood setpoint (microseconds) into equivalent linear rack travel in millimeters.
     */
    public static double pulseWidthToTravelMillimeters(double pulseWidthUs) {
        double clampedPulse = Math.max(MIN_PULSE_WIDTH_US, Math.min(MAX_PULSE_WIDTH_US, pulseWidthUs));
        double normalized = (clampedPulse - MIN_PULSE_WIDTH_US) / (MAX_PULSE_WIDTH_US - MIN_PULSE_WIDTH_US);
        return normalized * ACTUATOR_STROKE_MM;
    }

    /**
     * Converts linear rack travel to motor rotations.
     *
     * @param travelMillimeters required rack travel in mm
     * @param pinionPitchDiameterMillimeters pitch diameter of the pinion in mm
     * @param motorToPinionRatio motor rotations / pinion rotations (1.0 for direct drive)
     */
    public static double travelMillimetersToMotorRotations(
            double travelMillimeters,
            double pinionPitchDiameterMillimeters,
            double motorToPinionRatio) {
        if (pinionPitchDiameterMillimeters <= 0.0) {
            throw new IllegalArgumentException("Pinion pitch diameter must be > 0");
        }
        if (motorToPinionRatio <= 0.0) {
            throw new IllegalArgumentException("Motor-to-pinion ratio must be > 0");
        }

        double pinionCircumferenceMillimeters = Math.PI * pinionPitchDiameterMillimeters;
        double pinionRotations = travelMillimeters / pinionCircumferenceMillimeters;
        return pinionRotations * motorToPinionRatio;
    }

    /**
     * Convenience method for converting legacy hood pulse-width setpoints directly into motor rotations.
     */
    public static double pulseWidthToMotorRotations(
            double pulseWidthUs,
            double pinionPitchDiameterMillimeters,
            double motorToPinionRatio) {
        return travelMillimetersToMotorRotations(
                pulseWidthToTravelMillimeters(pulseWidthUs),
                pinionPitchDiameterMillimeters,
                motorToPinionRatio);
    }
}

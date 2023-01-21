package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final ControlAreaNetworkDevice driveMotor;
    public final ControlAreaNetworkDevice angleMotor;
    public final ControlAreaNetworkDevice encoder;
    public final Rotation2d angleOffset;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     */
    public SwerveModuleConstants(ControlAreaNetworkDevice driveMotor, ControlAreaNetworkDevice angleMotor, ControlAreaNetworkDevice encoder, Rotation2d angleOffset) {
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.encoder = encoder;
        this.angleOffset = angleOffset;
    }
}

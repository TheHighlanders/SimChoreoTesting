// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.SwerveModuleConfig;

public class SwerveSim extends SwerveBase {

    public class Constants {

        static boolean diagnosticMode = false;
        static SwerveModuleConfig FL = new SwerveModuleConfig(1, 2, new Rotation2d());
        static SwerveModuleConfig FR = new SwerveModuleConfig(2, 3, new Rotation2d());
        static SwerveModuleConfig BL = new SwerveModuleConfig(4, 5, new Rotation2d());
        static SwerveModuleConfig BR = new SwerveModuleConfig(6, 7, new Rotation2d());
        static double kWheelBase = Units.inchesToMeters(24);
        static double kTrackWidth = Units.inchesToMeters(24);
        static double kMaxSpeed = 5.0;
        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
                new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
                new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
                new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));
    }

    /* Array of Modules */
    public SwerveModuleSim[] modules;
    private AHRS gyro;
    public ChassisSpeeds chassisSpeeds;
    private StructArrayPublisher<SwerveModuleState> statePublisher;
    private DoublePublisher anglePublisher;
    private DoublePublisher anglespPublisher;
    private DoublePublisher drivePublisher;
    private DoublePublisher drivespPublisher;

    public SwerveDrivePoseEstimator swervePoseEstimator;
    private SwerveModulePosition[] lastModulePositions;
    private Rotation2d rawGyroRotation = new Rotation2d();
    public Field2d field;

    public SwerveSim() {
        /* Initializes modules from Constants */
        modules = new SwerveModuleSim[] {
                new SwerveModuleSim(0),
                new SwerveModuleSim(1),
                new SwerveModuleSim(2),
                new SwerveModuleSim(3),
        };

        swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.kinematics, new Rotation2d(), getModulePositions(),
                new Pose2d());
        lastModulePositions = getModulePositions();
        
        field = new Field2d();
        SmartDashboard.putData(field);
        statePublisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
        anglePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/SwerveAngle").publish();
        anglespPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/SwerveAngleSetpoints").publish();
        drivePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/SwerveDrive").publish();
        drivespPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/SwerveDriveSetpoints").publish();

    }

    @Override
    public void periodic() {
        updateOdometry();
        field.setRobotPose(swervePoseEstimator.getEstimatedPosition());
        statePublisher.set(getStates());
        anglePublisher.set(modules[0].getAnglePosition().getDegrees());
        anglespPublisher.set(modules[0].getAngleSetpoint().getDegrees());
        drivePublisher.set(modules[0].getDriveVelocity());
        drivespPublisher.set(modules[0].getDriveSetpoint());

        updateAllModules();
    }

    /**
     * Runs all IK and sets modules states
     *
     * @param translate     Desired translations speeds m/s
     * @param rotate        Desired rotation rate Rotation2d
     * @param fieldRelative Driving mode
     * @param isOpenLoop    Drive controller mode
     */
    public void drive(Translation2d translate, Rotation2d rotate, boolean fieldRelative, boolean isOpenLoop) {
        chassisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(-translate.getX(), -translate.getY(), rotate.getRadians(),
                        getYaw())
                : new ChassisSpeeds(translate.getX(), translate.getY(), rotate.getRadians());

        SwerveModuleState[] swerveModuleStates = Constants.kinematics.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kMaxSpeed);

        for (SwerveModuleSim m : modules) {
            m.setModuleState(SwerveModuleState.optimize(swerveModuleStates[m.moduleNumber], m.getAnglePosition())); // WHY
                                                                                                                    // WHY
                                                                                                                    // WHY
        }
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

        SwerveModuleState[] swerveModuleStates = Constants.kinematics.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kMaxSpeed);

        for (SwerveModuleSim m : modules) {
            m.setModuleState(swerveModuleStates[m.moduleNumber]);
        }
    }

    /**
     * Zeros the NavX
     */
    public void zeroGyro() {
        gyro.zeroYaw();
    }

    // For PP
    public void resetPose(Pose2d pose) {
        swervePoseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
        // SmartDashboard.putNumber("ResetPoseX", pose.getX());
        // SmartDashboard.putNumber("ResetPoseY", pose.getY());
    }

    public Rotation2d getYaw() {
        return swervePoseEstimator.getEstimatedPosition().getRotation();
    }

    public void updateOdometry(){
        double sampleTimestamp = Timer.getFPGATimestamp();
        // Read wheel positions and deltas from each module
        SwerveModulePosition[] modulePositions = getModulePositions();
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
            moduleDeltas[moduleIndex] =
             new SwerveModulePosition(
                 modulePositions[moduleIndex].distanceMeters
                      - lastModulePositions[moduleIndex].distanceMeters,
                    modulePositions[moduleIndex].angle);
            lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
                // 
        }
        // Use the angle delta from the kinematics and module delt
        // s
        Twist2d twist = Constants.kinematics.toTwist2d(moduleDeltas);
        //
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    

    // Apply update
        swervePoseEstimator.updateWithTime(sampleTimestamp,rawGyroRotation,modulePositions);
    }

    /**
     * Gets swerve modules positions for all modules
     *
     * @return Array of modules positions, in modules ID order
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModuleSim mod : modules) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        // return new Pose2d(odometer.getPoseMeters().getTranslation(), new Rotation2d());
        // return odometer.getPoseMeters();
        // return swervePoseEstimator.getEstimatedPosition();
        return swervePoseEstimator.getEstimatedPosition();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        // SmartDashboard.putNumber("getRobotRelativeSpeedsX", Constants.SwerveConst.kinematics.toChassisSpeeds(getStates()).vxMetersPerSecond);
        // SmartDashboard.putNumber("getRobotRelativeSpeedsY", Constants.SwerveConst.kinematics.toChassisSpeeds(getStates()).vyMetersPerSecond);
        // SmartDashboard.putNumber("getRobotRelativeSpeedsO", Constants.SwerveConst.kinematics.toChassisSpeeds(getStates()).omegaRadiansPerSecond);

        return Constants.kinematics.toChassisSpeeds(getStates());
    }

    /**
     * MUST be called every loop
     */
    public void updateAllModules(){
        for(SwerveModuleSim mod : modules){
            mod.updateModule();
        }
    }

    /**
     * Gets swerve modules states for all modules
     *
     * @return Array of modules states, in modules ID order
     */
    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModuleSim mod : modules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    /**
     *  Sends actual angle encoder data to SmartDashboard, paired with module number (drive motor ID), for use in debuging w/ sendAngleTargetDiagnostic()
     */
    public void sendAngleDiagnostic() {
        for (SwerveModuleSim m : modules) {
            SmartDashboard.putNumber("Module " + m.moduleNumber + " Angle Actual", m.getAnglePosition().getDegrees());
        }
    }

    /**
     *  Sends angle PID target data to SmartDashboard, paired with module number (drive motor ID), for use in debuging w/ sendAngleDiagnostic()
     */
    public void sendAngleTargetDiagnostic() {
        for (SwerveModuleSim m : modules) {
            SmartDashboard.putNumber("Module " + m.moduleNumber + " Angle Target", m.anglePID.getSetpoint());
        }
    }

    /**
     *  Sends actual drive encoder data to SmartDashboard, paired with module number (drive motor ID), for use in debuging w/ sendDriveTargetDiagnostic()
     */
    public void sendDriveDiagnostic() {
        double[] wheelSpeeds = new double[4];
        for (SwerveModuleSim m : modules) {
            SmartDashboard.putNumber("Module " + m.moduleNumber + " Position Drive Actual", m.getDrivePosition());
            SmartDashboard.putNumber("Module " + m.moduleNumber + " Velocity Actual", m.getDriveVelocity());
            wheelSpeeds[m.moduleNumber] = m.getDriveVelocity();
        }

        SmartDashboard.putNumber(
            "Velocity Range",
            Math.abs(Arrays.stream(wheelSpeeds).max().getAsDouble()) - Math.abs(Arrays.stream(wheelSpeeds).min().getAsDouble())
        );
    }

    /**
     *  Sends drive PID target data to SmartDashboard, paired with module number (drive motor ID), for use in debuging w/ sendDriveDiagnostic()
     */
    public void sendDriveTargetDiagnostic() {
        for (SwerveModuleSim m : modules) {
            SmartDashboard.putNumber("Module " + m.moduleNumber + " Velocity Target", m.drivePID.getSetpoint());
        }
    }
    public void sendSmartDashboardDiagnostics() {
        sendAngleDiagnostic();
        sendAngleTargetDiagnostic();

        sendDriveDiagnostic();
        // sendDriveTargetDiagnostic();

        SmartDashboard.putNumber("NavX Angle", getYaw().getDegrees());
        SmartDashboard.putNumber("Pose X", getPose().getX());
        SmartDashboard.putNumber("Pose Y", getPose().getY());
        SmartDashboard.putNumber("Pose Theta", getPose().getRotation().getDegrees());
    }

    public void jogSingleModule(int moduleNumber, double input, boolean drive) {
        if (drive) {
            modules[moduleNumber].setDriveState(new SwerveModuleState(input, new Rotation2d(0)));
            DriverStation.reportWarning(moduleNumber+"", false);
        } else {
            modules[moduleNumber].setAngleState(new SwerveModuleState(0, new Rotation2d(Math.toRadians(input))));
            DriverStation.reportWarning(moduleNumber +"", false);
        }
    }

    public void jogAllModuleDrive(double v) {
        drive(new Translation2d(0, v), new Rotation2d(0), true, false);
    }
}

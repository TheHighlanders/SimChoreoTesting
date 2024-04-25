// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.choreo.lib.*;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.SwerveModuleConfig;
import java.util.Arrays;
import java.util.Optional;

public class Swerve extends SubsystemBase {

    public class Constants {

        static boolean diagnosticMode = false;
        static SwerveModuleConfig FL = new SwerveModuleConfig(1, 2, new Rotation2d());
        static SwerveModuleConfig FR = new SwerveModuleConfig(2, 3, new Rotation2d());
        static SwerveModuleConfig BL = new SwerveModuleConfig(4, 5, new Rotation2d());
        static SwerveModuleConfig BR = new SwerveModuleConfig(6, 7, new Rotation2d());
        static double kWheelBase = Units.inchesToMeters(24);
        static double kTrackWidth = Units.inchesToMeters(24);
        static double kMaxSpeed = 3.0;
        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
            new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
            new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
            new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0)
        );

        private class AutoConstants {

            static double kPXController = 1;
            static double kPYController = kPXController;
            static double kPThetaController = 1;
        }
    }

    /* Array of Modules */
    public SwerveModule[] modules;
    private AHRS gyro;
    public ChassisSpeeds chassisSpeeds;

    public SwerveDriveOdometry odometer;

    public Swerve() {
        /* Initializes modules from Constants */
        if (Robot.isReal()) {
            modules =
                new SwerveModule[] {
                    new SwerveModule(0, Constants.FL),
                    new SwerveModule(1, Constants.FR),
                    new SwerveModule(2, Constants.BL),
                    new SwerveModule(3, Constants.BR),
                };
            gyro = new AHRS(SPI.Port.kMXP);
            zeroGyro();
        } else {
          modules = new SwerveModuleSim[]{
            new SwerveModuleSim(0),
            new SwerveModuleSim(1),
            new SwerveModuleSim(2),
            new SwerveModuleSim(3),
          };
        }

        // SmartDashboard.putNumber("getRobotRelativeSpeedsX", 1);
        // SmartDashboard.putNumber("getRobotRelativeSpeedsY", 2);
        // SmartDashboard.putNumber("getRobotRelativeSpeedsO", 3);

        // SmartDashboard.putNumber("PP X", 4);
        // SmartDashboard.putNumber("PP Y", 5);
        // SmartDashboard.putNumber("PP O", 6);
        odometer = new SwerveDriveOdometry(Constants.kinematics, getYaw(), getModulePositions());

        ChoreoTrajectory traj = Choreo.getTrajectory("Trajectory"); //

        Command trajectory = Choreo.choreoSwerveCommand(
            traj, //
            this::getPose, //
            new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0), //
            new PIDController(Constants.AutoConstants.kPYController, 0.0, 0.0), //
            new PIDController(Constants.AutoConstants.kPThetaController, 0.0, 0.0), //
            (ChassisSpeeds speeds) -> //
                this.drive(
                        new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                        new Rotation2d(speeds.omegaRadiansPerSecond),
                        false,
                        true
                    ),
            () -> {
                Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                return alliance.isPresent() && alliance.get() == Alliance.Red;
            }, //
            this //
        );

        chassisSpeeds = new ChassisSpeeds();
        // SmartDashboard.putData(field);
    }

    @Override
    public void periodic() {
        odometer.update(gyro.getRotation2d(), getModulePositions());

        if (Constants.diagnosticMode) {
            sendSmartDashboardDiagnostics();
        }
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
        chassisSpeeds =
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(-translate.getX(), -translate.getY(), rotate.getRadians(), getYaw())
                : new ChassisSpeeds(translate.getX(), translate.getY(), rotate.getRadians());

        SwerveModuleState[] swerveModuleStates = Constants.kinematics.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kMaxSpeed);

        for (SwerveModule m : modules) {
            m.setModuleState(swerveModuleStates[m.moduleNumber], isOpenLoop); //WHY WHY WHY
        }
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

        SwerveModuleState[] swerveModuleStates = Constants.kinematics.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kMaxSpeed);

        for (SwerveModule m : modules) {
            m.setModuleState(swerveModuleStates[m.moduleNumber], true);
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
        odometer.resetPosition(getYaw(), getModulePositions(), pose);
        // SmartDashboard.putNumber("ResetPoseX", pose.getX());
        // SmartDashboard.putNumber("ResetPoseY", pose.getY());
    }

    /**
     * Returns the gyro's yaw
     *
     * @return Yaw of gyro, includes zeroing
     */
    public Rotation2d getYaw() {
        if(Robot.isReal()){
            return Rotation2d.fromDegrees(-1 * gyro.getYaw());
        } else return new Rotation2d(); //TODO: FIX THIS
    }

    /**
     * Gets swerve modules positions for all modules
     *
     * @return Array of modules positions, in modules ID order
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : modules) {
            DriverStation.reportWarning("" + mod.moduleNumber, false);
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        // return new Pose2d(odometer.getPoseMeters().getTranslation(), new Rotation2d());
        // return odometer.getPoseMeters();
        // return swervePoseEstimator.getEstimatedPosition();
        return odometer.getPoseMeters();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        // SmartDashboard.putNumber("getRobotRelativeSpeedsX", Constants.SwerveConst.kinematics.toChassisSpeeds(getStates()).vxMetersPerSecond);
        // SmartDashboard.putNumber("getRobotRelativeSpeedsY", Constants.SwerveConst.kinematics.toChassisSpeeds(getStates()).vyMetersPerSecond);
        // SmartDashboard.putNumber("getRobotRelativeSpeedsO", Constants.SwerveConst.kinematics.toChassisSpeeds(getStates()).omegaRadiansPerSecond);

        return Constants.kinematics.toChassisSpeeds(getStates());
    }

    /**
     * Gets swerve modules states for all modules
     *
     * @return Array of modules states, in modules ID order
     */
    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : modules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void resetAllModulestoAbsol() {
        for (SwerveModule m : modules) {
            m.setIntegratedAngleToAbsolute();
        }
    }

    public void resetAllModules() {
        for (SwerveModule m : modules) {
            m.configureAngleMotor();
            m.configureDriveMotor();
        }
    }

    /**
     *  Sends actual angle encoder data to SmartDashboard, paired with module number (drive motor ID), for use in debuging w/ sendAngleTargetDiagnostic()
     */
    public void sendAngleDiagnostic() {
        for (SwerveModule m : modules) {
            SmartDashboard.putNumber("Module " + m.driveMotor.getDeviceId() / 10 + " Angle Actual", m.angleEncoder.getPosition());
        }
    }

    /**
     *  Sends angle PID target data to SmartDashboard, paired with module number (drive motor ID), for use in debuging w/ sendAngleDiagnostic()
     */
    public void sendAngleTargetDiagnostic() {
        for (SwerveModule m : modules) {
            SmartDashboard.putNumber("Module " + m.driveMotor.getDeviceId() / 10 + " Angle Target", m.angleReference);
        }
    }

    /**
     *  Sends actual drive encoder data to SmartDashboard, paired with module number (drive motor ID), for use in debuging w/ sendDriveTargetDiagnostic()
     */
    public void sendDriveDiagnostic() {
        double[] wheelSpeeds = new double[4];
        for (SwerveModule m : modules) {
            SmartDashboard.putNumber("Module " + m.driveMotor.getDeviceId() / 10 + " Position Drive Actual", m.driveEncoder.getPosition());
            SmartDashboard.putNumber("Module " + m.driveMotor.getDeviceId() / 10 + " Velocity Actual", m.driveEncoder.getVelocity());
            wheelSpeeds[m.driveMotor.getDeviceId() / 10] = m.driveEncoder.getVelocity();
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
        for (SwerveModule m : modules) {
            SmartDashboard.putNumber("Module " + m.driveMotor.getDeviceId() / 10 + " Velocity Target", m.driveReference);
        }
    }

    public void sendAbsoluteDiagnostic() {
        for (SwerveModule m : modules) {
            SmartDashboard.putNumber("Module " + m.driveMotor.getDeviceId() / 10 + " Absolute", m.getAbsolutePositionNoOffset().getDegrees());
            SmartDashboard.putNumber("Module " + m.driveMotor.getDeviceId() / 10 + " Absolute Offsetted", m.getAbsolutePosition().getDegrees());
        }
    }

    public void sendSmartDashboardDiagnostics() {
        sendAngleDiagnostic();
        sendAngleTargetDiagnostic();

        sendDriveDiagnostic();
        // sendDriveTargetDiagnostic();

        sendAbsoluteDiagnostic();

        SmartDashboard.putNumber("NavX Angle", getYaw().getDegrees());
        SmartDashboard.putNumber("Pose X", getPose().getX());
        SmartDashboard.putNumber("Pose Y", getPose().getY());
        SmartDashboard.putNumber("Pose Theta", getPose().getRotation().getDegrees());
    }

    public void jogSingleModule(int moduleNumber, double input, boolean drive) {
        if (drive) {
            modules[moduleNumber].setDriveState(new SwerveModuleState(input, new Rotation2d(0)), false);
            DriverStation.reportWarning(modules[moduleNumber].driveMotor.getDeviceId() + "", false);
        } else {
            modules[moduleNumber].setAngleState(new SwerveModuleState(0, new Rotation2d(Math.toRadians(input))));
            DriverStation.reportWarning(modules[moduleNumber].angleMotor.getDeviceId() + "", false);
        }
    }

    public void jogAllModuleDrive(double v) {
        drive(new Translation2d(0, v), new Rotation2d(0), true, false);
    }
}

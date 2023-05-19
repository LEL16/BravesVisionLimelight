package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;

public class TargetTrackingWithLimelight extends CommandBase {
    private final LimelightSubsystem m_limelightSubsystem;
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private PIDController m_horizontalPIDController;
    private PIDController m_verticalPIDController;
    private PIDController m_angController;

    private double m_horizontalVelocity;
    private double m_verticalVelocity;
    private double m_angularVelocity;
    private double distanceToPlayer;
    private double deadband;
    private double kP;
    private double isFinishedTolerance;

    private boolean fieldRelative;

    private ShuffleboardTab limelightTab;

    /**
     * @param drivetrainSubsystem The robot's drivetrain subsystem
     * @param limelightSubsystem  The robot's Limelight subsystem
     * @param distanceToPlayer    The desired distance to maintain from the player
     * @param isFinishedTolerance The tolerance in which the command is considered
     *                            finished
     * @param kP                  The proportional constant for the PID controller
     * @param deadband            The range in which small values will be ignored to
     *                            avoid jitter
     * @param fieldRelative       Whether to use field-relative control
     */
    public TargetTrackingWithLimelight(DrivetrainSubsystem drivetrainSubsystem, LimelightSubsystem limelightSubsystem,
            double distanceToPlayer, double isFinishedTolerance, double kP, double deadband, boolean fieldRelative) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_limelightSubsystem = limelightSubsystem;

        this.distanceToPlayer = distanceToPlayer;
        this.isFinishedTolerance = isFinishedTolerance;
        this.kP = kP;
        this.deadband = deadband;
        this.fieldRelative = fieldRelative;

        limelightTab = Shuffleboard.getTab("Limelight Tracking Command");
        ShuffleboardLayout limelightLayout = limelightTab.getLayout("Limelight Values", BuiltInLayouts.kList)
                .withSize(2, 3).withPosition(0, 0);

        limelightLayout.addNumber("Horizontal Velocity", () -> m_horizontalVelocity);
        limelightLayout.addNumber("Vertical Velocity", () -> m_verticalVelocity);
        limelightLayout.addNumber("Angular Velocity", () -> m_angularVelocity);
        limelightLayout.addNumber("IsFinished() Tolerance", () -> this.isFinishedTolerance);
        limelightLayout.addBoolean("Field Relative", () -> this.fieldRelative);

        addRequirements(m_drivetrainSubsystem);
    }

    /**
     * Initialize the command and create PID controllers for horizontal, vertical,
     * and angular movements.
     */
    public void initialize() {
        m_horizontalPIDController = new PIDController(kP, 0, 0);
        m_verticalPIDController = new PIDController(kP, 0, 0);
        m_angController = new PIDController(kP, 0, 0);
    }

    /**
     * Execute the command. Calculate velocities using PID controllers based on the
     * Limelight data, apply deadbands,
     * and drive the drivetrain.
     */
    public void execute() {
        if (fieldRelative) {
            m_horizontalVelocity = m_horizontalPIDController.calculate(m_limelightSubsystem.getHorizontalTargetAngle());
        } else {
            m_angularVelocity = m_angController.calculate(m_limelightSubsystem.getHorizontalTargetAngle());
        }

        m_verticalVelocity = -m_verticalPIDController.calculate((m_limelightSubsystem.getDistance() -
                distanceToPlayer) * 100);

        m_verticalVelocity = (Math.abs(m_verticalVelocity) < deadband) ? 0 : m_verticalVelocity;
        m_horizontalVelocity = (Math.abs(m_horizontalVelocity) < deadband) ? 0 : m_horizontalVelocity;
        m_angularVelocity = (Math.abs(m_angularVelocity) < deadband) ? 0 : m_horizontalVelocity;

        m_drivetrainSubsystem.drive(
                m_verticalVelocity,
                m_horizontalVelocity,
                m_angularVelocity,
                fieldRelative);

        limelightTab.addNumber("Limelight Distance", () -> m_limelightSubsystem.getDistance());
        limelightTab.addNumber("Limelight Horizontal Target Angle",
                () -> m_limelightSubsystem.getHorizontalTargetAngle());
        limelightTab.addBoolean("Is Finished", () -> isFinished());
    }

    /**
     * @return True if the command is considered finished (when the robot is within
     *         a certain tolerance of the target),
     *         false otherwise.
     */

    public boolean isFinished() {
        if (Math.abs(m_limelightSubsystem.getHorizontalTargetAngle()) > isFinishedTolerance
                || (Math.abs(m_limelightSubsystem.getDistance() - distanceToPlayer) > isFinishedTolerance / 5)) {
            return false;
        }
        return true;
    }

    /**
     * @param interrupted If the command was interrupted or not
     *                    Stops the drivetrain when the command ends, regardless of
     *                    reason (successful completion or interruption).
     */
    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(0, 0, 0, true);
    }
}
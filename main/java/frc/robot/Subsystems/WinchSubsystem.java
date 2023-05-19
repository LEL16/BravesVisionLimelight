package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WinchSubsystem extends SubsystemBase {
    private CANSparkMax m_winch1;
    private CANSparkMax m_winch2;

    private double m_winchSpeed = 0;

    private DutyCycleEncoder m_winchEncoder;

    DigitalInput m_topSwitch;
    DigitalInput m_bottomSwitch;

    public WinchSubsystem() {
        m_winch1 = new CANSparkMax(61, MotorType.kBrushless);
        m_winch1.setIdleMode(IdleMode.kBrake);

        m_winch2 = new CANSparkMax(7, MotorType.kBrushless);
        m_winch2.setIdleMode(IdleMode.kBrake);
        m_winch2.follow(m_winch1, true);

        m_winchEncoder = new DutyCycleEncoder(0);
        m_winchEncoder.setDistancePerRotation(-360); // Convert from encoder rotations to degrees

        m_topSwitch = new DigitalInput(3);
        m_bottomSwitch = new DigitalInput(4);
    }

    public void rotate(double winchSpeed) {
        m_winchSpeed = winchSpeed;
    }

    public double getWinchAbsPosition() {
        return 90 - m_winchEncoder.getDistance();
    }

    // Only resets when a match starts
    public void resetEncoders() {
        m_winchEncoder.reset();
    }

    @Override
    public void periodic() {
        if (!m_topSwitch.get() && m_winchSpeed > 0) {
            m_winch1.set(0);
            /*
            if (this.getCurrentCommand() != null) {
                this.getCurrentCommand().cancel();
            }
            */
        }
        else if (!m_bottomSwitch.get() && m_winchSpeed < 0) {
            m_winch1.set(m_winchSpeed / 4.0);
            /*
            if (this.getCurrentCommand() != null) {
                this.getCurrentCommand().cancel();
            }
            */
        }
        else {
            m_winch1.set(m_winchSpeed);
        }

        SmartDashboard.putNumber("Winch Angle", getWinchAbsPosition());
    }

    /*
    public double calculateTheta(double distance) {
        double theta = Math.pow(0.76, 2) + Math.pow(0.96, 2) - Math.pow(distance, 2);
        theta /= (2 * 0.83 * 0.91);
        if (theta > 1) {
            theta = 1;
        }
        theta = Math.acos(theta);
        return Math.toDegrees(theta);
    }

    public double calculateDistance() {
        return (-244.505 * Math.pow(1.05753, getWinchAbsPosition()) + 266.168) / 100;
    }
    */
}
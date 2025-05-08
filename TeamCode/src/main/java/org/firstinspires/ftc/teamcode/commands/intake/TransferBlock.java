package org.firstinspires.ftc.teamcode.commands.intake;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeExtensionWithTimeout;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeShoulderByTime;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeSpinByTime;

/**
 * A command that handles the complete sequence of transferring a block from the intake to the basket.
 * This includes:
 * 1. Leveling the basket while extending the intake
 * 2. Lifting the shoulder
 * 3. Transferring the block (spinning the intake)
 * 4. Retracting the intake while it's still upright
 * 5. Final shoulder adjustment
 */
public class TransferBlock extends SequentialCommandGroup {
    private final Callisto robot;

    // ------- MAGIC NUMBERS -----------
    // How long we run the intake spin to transfer the block
    private static final double SPIN_DURATION_MS = 2000;
    // How far we extend the intake (0 = fully extended, 1 = fully retracted)
    private static final double INTAKE_EXTENSION = 0.5;
    // ------- MAGIC NUMBERS -----------

    /**
     * Creates a new TransferBlock command.
     * @param robot The robot container instance
     */
    public TransferBlock(Callisto robot) {
        this.robot = robot;

        // Add all the sequential commands for the transfer process
        addCommands(
                // Step 1: Level basket while extending intake (parallel actions)
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.lift.levelBasket()),
                        new IntakeExtensionWithTimeout(robot, INTAKE_EXTENSION, 2000)
                ),

                // Step 2: Lift shoulder up
                new IntakeShoulderByTime(robot, 0.6, 1000),

                // Step 3: Transfer block by spinning intake
                new IntakeSpinByTime(robot, (long)SPIN_DURATION_MS, 0.3),

                // Step 4: Retract intake while still upright
                new IntakeExtensionWithTimeout(robot, 1, 2000),

                // Step 5: Final shoulder adjustment
                new IntakeShoulderByTime(robot, 0.25, 1500)
        );
    }

    @Override
    public boolean isFinished() {
        // The command is finished when all sub-commands are complete
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        // Ensure everything stops properly if interrupted
        if (interrupted) {
            robot.intake.stopSpin();
            robot.intake.stopShoulder();
        }
        super.end(interrupted);
    }
}
package frc.robot.commands;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.logging.ConsoleLogger;

public class AsyncDeferredCommand extends Command {
  // Shared thread pool:
  private static final ExecutorService THREAD_POOL = Executors.newCachedThreadPool();
  
  private final Supplier<Command> supplier;
  private final String name;

  private CompletableFuture<Command> futureCommand;
  private Command generatedCommand;
  private boolean hasInitialized;

  /**
   * Like a {@link edu.wpi.first.wpilibj2.command.DeferredCommand DeferredCommand}, but the supplier
   * is run asynchronously.
   * @param name
   * @param supplier
   * @param requirements
   */
  public AsyncDeferredCommand(String name, Supplier<Command> supplier, Subsystem... requirements) {
    this.supplier = supplier;
    this.name = name;

    addRequirements(requirements);
  }

  /** Runs the supplier to get the new command */
  private Command generate() {
    double start = System.currentTimeMillis();

    Command generated;
    
    // Try to generate command
    try {
      generated = supplier.get();
    } catch (Exception e) {
      ConsoleLogger.reportError(String.format("AsyncDeferredCommand %s supplier failed: %s", name, e.getMessage()));
      generated = null;
    }

    // Report if failed
    if (generated == null) {
      generated = new PrintCommand(String.format("AsyncDeferredCommand %s supplier returned null!", name));
    }

    // Output total time
    double delta = (System.currentTimeMillis() - start) / 1000.0; // seconds
    System.out.printf("AsyncDeferredCommand %s generated %s in %f seconds\n", name, generated.getName(), delta);
    return generated;
  }

  @Override
  public void initialize() {
    generatedCommand = null;
    hasInitialized = false;

    // Cancel if already running
    if (futureCommand != null && !futureCommand.isDone()) {
      futureCommand.cancel(true);
    }

    // Generate new command
    futureCommand = CompletableFuture.supplyAsync(this::generate, THREAD_POOL);
  }

  @Override
  @SuppressWarnings("UseSpecificCatch")
  public void execute() {
    // Check that the command has been generated yet
    if (generatedCommand == null && futureCommand.isDone()) {
      try {
        generatedCommand = futureCommand.get();
      } catch (Exception e) {
        ConsoleLogger.reportError(String.format("AsyncDeferredCommand %s .get() failed: %s", name, e.getMessage()));
        generatedCommand = new PrintCommand("Failed to run AsyncDeferredCommand " + name);
      }
    }

    // Run the command
    if (generatedCommand != null) {
      if (!hasInitialized) {
        CommandScheduler.getInstance().registerComposedCommands(generatedCommand);

        // Sometimes, .initialize() throws
        try {
          generatedCommand.initialize();
          hasInitialized = true;
        } catch (Exception e) {
          ConsoleLogger.reportError(String.format(
            "AsyncDeferredCommand %s failed to initialize %s: %s\n",
            name,
            generatedCommand.getName(),
            e.getMessage()
          ));
          generatedCommand = new PrintCommand("Failed to initialize " + name);
        }
      } else {
        generatedCommand.execute();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (generatedCommand != null) {
      generatedCommand.end(interrupted);
    }

    futureCommand.cancel(true);
  }

  @Override
  public boolean isFinished() {
    return (generatedCommand != null && hasInitialized && generatedCommand.isFinished());
  }
}

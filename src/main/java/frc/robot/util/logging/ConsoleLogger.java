package frc.robot.util.logging;

import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintStream;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.BuildConstants;

/**
 * Captures Java's Console messages to WPILogs, because the built-in logger sometimes garbles
 * messages.
 */
public class ConsoleLogger {
  private static final StringLogEntry outLog = 
    new StringLogEntry(DataLogManager.getLog(), "messages");

  private static final StringPublisher outPub;

  static {
    // Init string publisher conditionally
    if (BuildConstants.PUBLISH_EVERYTHING) {
      NetworkTableInstance nt = NetworkTableInstance.getDefault();
      outPub = nt.getStringTopic("ConsoleOut").publish();
    } else {
      outPub = null;
    }
  }

  /**
   * OutputStream that captures to DataLog.
   */
  private static class DataLogOutputStream extends OutputStream {
    private final PrintStream originalStream;
    private final StringLogEntry dataLogOut;

    private final StringBuilder buffer = new StringBuilder();

    /**
     * @param originalStream The original stream that this is replacing.
     * @param entryName The name of the DataLog entry to log to.
     */
    public DataLogOutputStream(PrintStream originalStream, StringLogEntry dataLogOut) {
      this.originalStream = originalStream;
      this.dataLogOut = dataLogOut;
    }

    @Override
    public void write(int b) throws IOException {
      if (b == '\n' || buffer.length() > 800) {
        // Flush buffer
        dataLogOut.append(buffer.toString());

        if (BuildConstants.PUBLISH_EVERYTHING) {
          outPub.set(buffer.toString());
        }
        
        buffer.delete(0, buffer.length());
      } else {
        // Append to buffer
        buffer.append((char)b);
      }

      originalStream.append((char)b);
    }
  }

  /**
   * Starts logging the console to a WPILog file.
   */
  public static void init() {
    System.setOut(
      new PrintStream(new DataLogOutputStream(System.out, outLog), true));

    System.setErr(
      new PrintStream(new DataLogOutputStream(System.err, outLog), true));
  }

  /**
   * @param msg Warning to report to the driver station and log.
   */
  public static void reportWarning(String msg) {
    outLog.append("[WARNING] " + msg);
    DriverStation.reportWarning(msg, false);

    if (BuildConstants.PUBLISH_EVERYTHING) {
      outPub.set("[WARNING] " + msg);
    }
  }

  /**
   * @param msg Error to report to the driver station and log.
   */
  public static void reportError(String msg) {
    outLog.append("[ERROR] " + msg);
    DriverStation.reportError(msg, false);

    if (BuildConstants.PUBLISH_EVERYTHING) {
      outPub.set("[ERROR] " + msg);
    }
  }

  /** Utility class, prevent instantiation */
  private ConsoleLogger() {}
}
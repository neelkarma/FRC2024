package frc.robot.utils.logger;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.concurrent.locks.ReentrantLock;

import frc.robot.constants.FileConstants;
import frc.robot.constants.LoggerConstants;

public class Logger {

  private ArrayList<LogRow> cache = new ArrayList<LogRow>();
  private final ReentrantLock cacheLock = new ReentrantLock();

  private final Instant dateCutoff = Instant.parse("2000-01-01T00:00:00.00Z");
  private final int millisPerCSVLine = 20;

  private Thread thread;

  private boolean paused = false;
  private boolean stopped = false;

  private File file;
  private String logFolderPath;
  private String fileName;

  public Logger(String fileName) {
    this.fileName = fileName;
    findFilePath();
    makeFile();
    launchThread();
  }

  public void log(String line) {
    addLine(line);
  }

  /** Adds a new entry to the current row in the log file */
  private void addLine(String line) {

    if (paused || stopped) {
      return;
    }

    cacheLock.lock();

    var index = getCurrentGroupIndex();
    var newGroup = cache.get(index);
    newGroup.addLine(line);
    cache.set(index, newGroup);

    cacheLock.unlock();
  }

  private int getCurrentGroupIndex() {

    // create a new LogRow if the last one is out-of-date
    var lastGroup = cache.get(cache.size()-1);
    if (lastGroup.getTime().isBefore(Calendar.getInstance().getTime().toInstant().minusMillis(millisPerCSVLine))) {
      cache.add(new LogRow());
    }

    return cache.size()-1;
  }

  private void findFilePath() {
    if (paused || stopped) {
      return;
    }

    try {
      for (String tmpPath : FileConstants.PATH_USB) {
        if (new File(tmpPath).exists()) {
          logFolderPath = tmpPath;
          return;
        }
      }
      stopped = true;
      System.out.println("[" + fileName + " Logger] Log Folder Path Not Found : Killing Logger");
    } catch (SecurityException e) {
      stopped = true;
      System.out.println("[" + fileName + " Logger] Log Folder Path Security Exception : Killing Logger");
    }
  }

  private void makeFile() {
    if (paused || stopped) {
      return;
    }

    file = new File(logFolderPath + fileName + ".csv");


    try {
      String filePath = "";
      for (int i = 0; !file.createNewFile(); i++) {
        if (i > LoggerConstants.REPEAT_LIMIT_LOGGER_CREATION) {
          stopped = true;
          System.out.println("[" + fileName + " Logger] File Creation Attempt Limit Exceeded : Killing Logger");
          return;
        }
        filePath = logFolderPath + fileName + "_(" + i + ").csv";
        file = new File(filePath);

      }
      
      if (!stopped) {
        System.out.println("[" + fileName + " Logger] Log File Path Found : " + filePath);
      }

    } catch (IOException e) {
      stopped = true;
      System.out.println("[" + fileName + " Logger] File Creation Failed : IOException : " + e + " : Killing Logger");
    } catch (SecurityException e) {
      stopped = true;
      System.out.println("[" + fileName + " Logger] File Creation Failed : Security Exception : " + e + " : Killing Logger");
    }

  }

  public void save() {
    cacheLock.lock();

    try {
      FileWriter writer = new FileWriter(file, true);
      while (!cache.isEmpty()) {
        
        var group = cache.remove(0);

        var timestamp = group.getTime();
        if (timestamp.isBefore(dateCutoff)) {
          timestamp = dateCutoff;
        }
        writer.write(timestamp.toString());
        
        for (String line : group.getLines()) {
          writer.write(", " + line.toString());
        }
        
        writer.write(System.lineSeparator());
      }
      writer.close();
    } catch (IOException e) {
      System.out.println("[" + fileName + " Logger] File Save Failed : IOExeption : " + e);
    } catch (SecurityException e) {
      System.out.println("[" + fileName + " Logger] File Save Failed : Security Exeption : " + e);
    } finally {
      if (!cache.isEmpty()) {
        System.out.println("[" + fileName + " Logger] Save Error : Killing Logger");
      }
      stopped = true;
      
      cacheLock.unlock();
    }
    cacheLock.unlock();
  }

  /** reversibly stops operation of the logger */
  public void pause() {
    if (!stopped) {
      paused = true;
    }
  }

  /** restarts the logger when paused */
  public void unpause() {
    if (!stopped) {
      paused = false;
    }
  }

  /** irreversably kills the logger */
  public void stop() {
    if (!stopped) {
      pause();
      stopped = true;
    }
  }

  private void launchThread() {
    if (stopped) {
      return;
    }
    thread = new Thread(() -> {
      try {
        while (!stopped) {
          if (!cache.isEmpty() && !paused && !stopped) {
            save();
          }

          Thread.sleep(1000 / LoggerConstants.SAVE_RATE);
        }
        cache.clear();
      } catch (InterruptedException e) {
        System.out.println("[" + fileName + " Logger] Save Thread Interrupted : " + e);
      }
    });
    thread.start();
  }
}

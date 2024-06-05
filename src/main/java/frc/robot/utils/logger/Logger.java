package frc.robot.utils.logger;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.concurrent.locks.ReentrantLock;

import frc.robot.constants.FileConstants;
import frc.robot.constants.LoggerConstants;

public class Logger {

  private ArrayList<double[]> cache = new ArrayList<double[]>();
  private final ReentrantLock cacheLock = new ReentrantLock();

  private final long creationTime = Calendar.getInstance().getTime().toInstant().toEpochMilli();

  private Thread thread;

  private boolean paused = false;
  private boolean stopped = false;

  private File file;
  private String logFolderPath;
  private String fileName;

  public Logger(String fileName, String[] columns) {
    this.fileName = fileName;
    findFilePath();
    makeFile();

    if(!stopped){
      save(columns);

      launchThread();
    }
  }


  public void log(double[] values) {

    var row = new double[values.length + 1];
    row[0] = (double)(Calendar.getInstance().getTime().toInstant().toEpochMilli() - creationTime) / 1000d;
    for (int i = 0; i < values.length; i++) {
      row[i+1] = values[i];
    }

    cacheLock.lock();
    cache.add(row);
    cacheLock.unlock();
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
    if (file == null) {
      stopped = true;
      return;
    }


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
    save(new String[0]);
  }

  public void save(String[] override) {
    cacheLock.lock();

    try {
      FileWriter writer = new FileWriter(file, true);

      if (override.length != 0) {
        writer.write("Time");
        for (int i = 0; i < override.length; i++) {
          writer.write(", " + override[i]);
        }
        writer.write(System.lineSeparator());
      }

      while (!cache.isEmpty()) {
        
        var row = cache.remove(0);
        
        writer.write(String.valueOf(row[0])); // timestamp
        for (int i = 1; i < row.length; i++) {
          writer.write(", " + row[i]);
        }
        
        writer.write(System.lineSeparator());
      }
      writer.close();
    } catch (IOException e) {
      System.out.println("[" + fileName + " Logger] File Save Failed : IOExeption : " + e);
      stopped = true;
    } catch (SecurityException e) {
      System.out.println("[" + fileName + " Logger] File Save Failed : Security Exeption : " + e);
      stopped = true;
    } finally {
      if (!cache.isEmpty()) {
        System.out.println("[" + fileName + " Logger] Save Error : Killing Logger");
      }
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

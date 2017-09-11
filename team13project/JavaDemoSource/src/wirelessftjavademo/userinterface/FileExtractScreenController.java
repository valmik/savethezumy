/*
 * Copyright 2013
 * ATI Industrial Automation
 */
package wirelessftjavademo.userinterface;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.net.URL;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.ResourceBundle;
import java.util.prefs.Preferences;
import javafx.application.Platform;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.concurrent.Task;
import javafx.concurrent.Worker;
import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.scene.Node;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.ProgressBar;
import javafx.scene.control.TextField;
import javafx.stage.DirectoryChooser;
import javafx.stage.FileChooser;
import javafx.stage.FileChooser.ExtensionFilter;
import wirelessftjavademo.WirelessFTDemoModel;
import static wirelessftjavademo.userinterface.WirelessFTDemoMainScreenController.PREF_USER_ROOT_NODE_PATHNAME;
import wirelessftsensor.WirelessFTSample;

/**
 * FXML Controller class
 * 
 * @author Chris Collins
 */
public class FileExtractScreenController implements Initializable {
    
    /**
     * Used to access preferred extraction folder.
     */
    private static final String PREF_FILE_EXTRACT_DIRECTORY = "fileExtractDefault";
    
    /**
     * Used to access preferred destination folder.
     */
    private static final String PREF_FILE_STORAGE_DIRECTORY = "fileStorageDefault";
    
    /**
     * Text field for the .DAT files.
     */
    @FXML
    TextField m_txtFiles;
    
    /**
     * Opens a file chooser for .DAT files.
     */
    @FXML
    Button m_btnChooseFiles; 
    
    /**
     * Destination folder name.
     */
    @FXML
    TextField m_txtDest;
    
    /**
     * Opens a directory chooser to the destination folder.
     */
    @FXML
    Button m_btnChooseDest; 
    
    /**
     * Begins the extraction/file parsing process.
     */
    @FXML
    Button m_btnExtract;
    
    /**
     * Displays job progress.
     */
    @FXML
    ProgressBar m_progressBar;
    
    /**
     * Progress as a % number.
     */
    @FXML
    Label m_lblProgress;
    
    /**
     * The files to be converted from the MicroSD.
     */
    List<File> m_files;
    
    /**
     * The progress of the file extract.
     */
    private final SimpleObjectProperty<Double> m_fileExtractProgress = 
            new SimpleObjectProperty<>(0.0);
    
    /**
     * Is the file extract done?
     */
    private final SimpleObjectProperty<Boolean> m_extractDone = 
            new SimpleObjectProperty<>(false);
    
    /**
     * Initializes the controller class.
     * 
     * @param url Not used.
     * @param rb  Not used.
     */
    @Override
    public void initialize(URL url, ResourceBundle rb) {
    }
    
    /**
     * Enables/disables controls based
     * on file-extract job status.
     * 
     * @param disabled Should the UI be disabled?
     */
    private void enableUI(boolean enabled) 
    {
        m_txtFiles      .setDisable(!enabled);
        m_txtDest       .setDisable(!enabled);
        m_btnExtract    .setDisable(!enabled);
        m_btnChooseFiles.setDisable(!enabled);
        m_btnChooseDest .setDisable(!enabled);
    }
    
    /**
     * Opens a file chooser window to select one or more
     * .dat files from the MicroSD card to convert into
     * .csv files on the local hard drive.
     */
    @FXML
    protected void chooseFiles() 
    {
        Preferences prefs    = WirelessFTDemoMainScreenController.m_prefsRoot.node(PREF_USER_ROOT_NODE_PATHNAME);
        String      pathname = prefs.get(PREF_FILE_EXTRACT_DIRECTORY, null);
        
        try 
        {
            FileChooser chooser = new FileChooser();
            chooser.setTitle("Choose one or more files to convert:");
            chooser.getExtensionFilters().add(new ExtensionFilter(".DAT Files", "*.dat"));
            if(pathname != null) 
            {
                try 
                {
                    chooser.setInitialDirectory(new File(pathname));
                    m_files = chooser.showOpenMultipleDialog(((Node)m_btnChooseFiles).getScene().getWindow());
                } 
                catch(Exception e) 
                {
                    // User's last preferred directory was deleted.
                    chooser.setInitialDirectory(null);
                    m_files = chooser.showOpenMultipleDialog(((Node)m_btnChooseFiles).getScene().getWindow());
                }
            } 
            else 
            {
                // No preferred directory (first use).
                chooser.setInitialDirectory(null);
                m_files = chooser.showOpenMultipleDialog(((Node)m_btnChooseFiles).getScene().getWindow());
            }
            if(m_files != null) 
            {
                boolean first = true;
                for(File f : m_files) 
                {
                    if (!f.getPath().isEmpty()) 
                    {
                        if(!first) 
                        {
                            m_txtFiles.appendText(File.pathSeparator);
                        } 
                        else 
                        {
                            if (f.getParentFile().exists()) 
                            {
                                chooser.setInitialDirectory(f.getParentFile());
                                prefs.put(PREF_FILE_EXTRACT_DIRECTORY, f.getParentFile().getAbsolutePath());
                            }
                            first = false;
                        }
                        m_txtFiles.appendText(f.getAbsolutePath());
                    }
                }
            }
        } 
        catch (NullPointerException npe) 
        {
            // Do nothing, they didn't pick a valid file.
        }
    }
    
    /**
     * Opens a file chooser window to select the
     * destination folder for the parsed .csv files.
     */
    @FXML
    protected void chooseDestinationFolder() 
    {
        Preferences prefs    = WirelessFTDemoMainScreenController.m_prefsRoot.node(PREF_USER_ROOT_NODE_PATHNAME);
        String      pathname = prefs.get(PREF_FILE_STORAGE_DIRECTORY, null);
        
        //m_txtDest.setText("");
        File directory;
        DirectoryChooser chooser = new DirectoryChooser();
        chooser.setTitle("Choose destination folder");
        if (pathname != null) 
        {
            try 
            {
                chooser.setInitialDirectory(new File(pathname));
                directory = chooser.showDialog(((Node)m_btnChooseDest).getScene().getWindow());
            } 
            catch(Exception iae) 
            {
                // User's last preferred directory was deleted.
                chooser.setInitialDirectory(null);
                directory = chooser.showDialog(((Node)m_btnChooseDest).getScene().getWindow());
            }
        } 
        else 
        {
            // No preferred directory (first use).
            chooser.setInitialDirectory(null);
            directory = chooser.showDialog(((Node)m_btnChooseDest).getScene().getWindow());
        }
        if (directory != null) 
        {
            m_txtDest.setText(directory.getAbsolutePath());
            prefs.put(PREF_FILE_STORAGE_DIRECTORY, directory.getAbsolutePath());
        }
    }
    
    /**
     * Extracts files from the SD Card and converts them
     * to .CSV files as the specified directory.
     */
    @FXML
    protected void extractFiles() 
    {
        if (!m_txtFiles.getText().isEmpty() && !m_txtDest.getText().isEmpty()) 
        {
            // Check the text field again in case they changed it.
            m_files = new ArrayList<>();
            String[] filenames = m_txtFiles.getText().split("" + File.pathSeparator);
            for(String s : filenames) 
            {
                m_files.add(new File(s));
            }
           
            enableUI(false); // Disable controls.
            m_fileExtractProgress.set(0.0); // Start the progress bar.
            m_progressBar.progressProperty().bind(fileExtractProgress());
            ExtractFileTask eTask = new ExtractFileTask(m_files);
            new Thread(eTask).start();
            
            eTask.stateProperty().addListener(new ChangeListener<Worker.State>() 
            {
                @Override
                public void changed(ObservableValue<? extends Worker.State> ov,
                        Worker.State t, Worker.State t1) 
                {
                    if (t1 == Task.State.SUCCEEDED) 
                    {
                        m_extractDone.set(true);
                    }
                }
            });
        } 
        else 
        {
            m_lblProgress.setText("Please select at least one .dat file and"
                                + " a destination folder.");
        }
    }
    
    /**
     * The percentage progress of the latest file extract process.
     * 
     * @return The percentage 0-100 of the file extract process.
     */
    private ObservableValue<Double> fileExtractProgress() {
        return m_fileExtractProgress;
    }
    
    /**
     * Background threading of the file extraction.
     */
    private class ExtractFileTask extends Task<Void> {
        /** Sensor headers for the .CSV file. */
        private final String[] CHANNEL_NAMES = new String[]{"ForceX/G0", "ForceY/G1", "ForceZ/G2", "TorqueX/G3", "TorqueY/G4", "TorqueZ/G5"};
        
        /** The name of the file(s). */
        private final ArrayList<String> m_fileNames = new ArrayList<>();
        
        /** The file(s) data. */
        private final ArrayList<byte[]> m_data      = new ArrayList<>();
        
        /** Storage for parsed files. */
        private final ArrayList<ArrayList<WirelessFTSample>> m_samples = new ArrayList<>();
        
        /** The total size of the job. */
        private double m_totalSize = 0.0;
        
        /** Writes the CSV file from WirlessFTSample data. */
        private BufferedWriter m_bufferedWriter;
        
        /**
         * Gets the size (in bytes) of the specified file,
         * if it is in the list of files to be extracted.
         * 
         * @param filename The name of the file.
         * @return The total size, in bytes, of a file to
         * send to the Wireless F/T (-1 if FileNotFound).
         */
        public int getFileSize(String filename) {
            int i = m_fileNames.indexOf(filename);
            if (i != -1)
                return m_data.get(i).length;
            return -1; // File not found.
        }
        
        /**
         * Creates new ExtractFileThread to convert files
         * from the MicroSD.
         *
         * @param files The file(s) that will be converted from
         * .dat to .csv and stored locally.
         */
        public ExtractFileTask(List<File> files) {
            try {
                m_lblProgress.setText("Reading .dat file(s)...");
                
                /* Files must be read from one place,
                 * then written to another. Runtime is
                 * 2*n (see call() function).
                 */
                m_totalSize = files.size() * 2.0;
                
                // Retrieve names and byte-arrays for each file.
                for (File f : files) {
                    byte[] fileToWrite;
                    try (FileInputStream input = new FileInputStream(f)) {
                        fileToWrite = new byte[(int) f.length()];
                        input.read(fileToWrite);
                    }
                    m_data.add(fileToWrite);

                    String fname = f.getName();
                    m_fileNames.add(fname);
                }
            } catch (IOException e) {
                m_lblProgress.setText("Error reading files.");
                enableUI(true);
            }
        }
        
        /**
         * Interprets the bytes of each .dat file, then writes the
         * WirelessFTSamples from them as ASCII into .csv files with
         * the same names in the specified directory.
         */
        @Override 
        public Void call() {
            String destination = m_txtDest.getText();
            int steps = 0; // Number of steps (out of 2*n) completed.
            
            try {
                Platform.runLater(new Runnable() {
                    /**
                     * runLater to make UI interaction
                     * thread-safe.
                     */
                    @Override public void run() {
                      m_lblProgress.setText("Parsing .dat file(s)...");                       
                    }
                });
                for(byte[] b : m_data) {
                    m_samples.add(WirelessFTSample.listOfSamplesFromPacket(b, b.length));
                    m_fileExtractProgress.set(++steps / m_totalSize);
                }
                
                Platform.runLater(new Runnable() {
                    /**
                     * runLater to make UI interaction
                     * thread-safe.
                     */
                    @Override public void run() {
                      m_lblProgress.setText("Writing .csv file(s)...");
                    }
                });
                
                int i = 0;
                for (ArrayList<WirelessFTSample> parsedData : m_samples) {
                    String csvFile = destination + File.separator + m_fileNames.get(i).toUpperCase().replace(".DAT", ".CSV");
                    m_bufferedWriter = Files.newBufferedWriter(Paths.get(csvFile), Charset.forName("US-ASCII"));
                    /**
                     * Write column headers.
                     */
                    m_bufferedWriter.write("Wireless F/T Time Stamp, Sequence Number, Status Code 1, Status Code 2, Battery Level, Sensor Mask");
                    boolean first = true;
                    
                    for (WirelessFTSample wfts : parsedData) 
                    {
                        if (!first) 
                        {
                            m_bufferedWriter.write(String.format("%d, %d, %8x, %8x, %2x, %2x",
                                    wfts.getTimeStamp(),    wfts.getSequence(),
                                    wfts.getStatusCode1(),  wfts.getStatusCode2(),
                                    wfts.getBatteryLevel(), wfts.getSensorMask()));
                            
                            for (int transducer = 0; transducer < WirelessFTDemoModel.MAX_SENSORS; transducer++)
                            {
                                if (wfts.getActiveSensors()[transducer]) 
                                {
                                    for (int axis = 0; axis < WirelessFTDemoModel.NUM_AXES; axis++) 
                                    {
                                        m_bufferedWriter.write(", " + Double.toString(wfts.getFtOrGageData()[transducer][axis]));
                                    }
                                }
                            }
                        } 
                        else 
                        {
                            for (int transducer = 0; transducer < WirelessFTDemoModel.MAX_SENSORS; transducer++) 
                            {
                                if (wfts.getActiveSensors()[transducer])
                                {
                                    for (int axis = 0; axis < WirelessFTDemoModel.NUM_AXES; axis++)
                                    {
                                        m_bufferedWriter.write(", Transducer " + (transducer + 1) + " " + CHANNEL_NAMES[axis]); // Transducer is origin 1 to the user
                                    }
                                }
                            }
                            
                            first = false;
                        }
                        
                        m_bufferedWriter.write("\n");
                    }
                    
                    m_bufferedWriter.close();
                    m_fileExtractProgress.set(++steps / m_totalSize);
                    i++;
                }
            } catch(IOException e) {
                Platform.runLater(new Runnable() {
                    /**
                     * runLater to make UI interaction
                     * thread-safe.
                     */
                    @Override public void run() {
                      m_lblProgress.setText("Error " + "parsing/writing" + " files.");
                      enableUI(true);
                    }
                });
                return null;
            }
            Platform.runLater(new Runnable() {
                /**
                 * runLater to make UI interaction
                 * thread-safe.
                 */
                @Override public void run() {
                    m_lblProgress.setText("Done.");
                    enableUI(true);
                }
            });
            return null;
        }
    }
}
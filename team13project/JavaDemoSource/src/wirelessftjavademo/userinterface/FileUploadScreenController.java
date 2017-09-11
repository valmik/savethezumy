/*
 * Copyright 2013
 * ATI Industrial Automation
 */
package wirelessftjavademo.userinterface;

import java.io.File;
import java.io.IOException;
import java.net.URL;
import java.util.List;
import java.util.ResourceBundle;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.prefs.Preferences;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.scene.Node;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.ProgressBar;
import javafx.scene.control.TextField;
import javafx.stage.FileChooser;
import wirelessftjavademo.WirelessFTDemoModel;
import static wirelessftjavademo.userinterface.WirelessFTDemoMainScreenController.PREF_USER_ROOT_NODE_PATHNAME;

/**
 * FXML Controller class
 *
 * @author Sam Skuce, Chris Collins
 */
public class FileUploadScreenController implements Initializable 
{
    /**
     * The pathnames for any files
     * to upload to the digital board.
     */
    @FXML
    TextField m_txtFilePath;
    
    /**
     * Opens a multiple file chooser
     * to set the upload files.
     */
    @FXML
    Button m_btnChooseFile; 
    
    /**
     * Displays upload progress.
     */
    @FXML
    ProgressBar m_progressUpload;
    
    /**
     * Displays status of the upload job.
     */
    @FXML
    Label m_lblProgress;
    
    /**
     * Begins uploading files to the board.
     */
    @FXML
    Button m_btnUpload;
    
    /**
     * The WNet to which files will be written.
     */
    WirelessFTDemoModel m_model = null;
    
    /**
     * The default upload file directory.
     */
    private static final String PREF_FILE_UPLOAD_DIRECTORY = "fileUploadDefault";
    
    /**
     * The list of files to upload.
     */
    List<File> m_files;
    
    /**
     * Logs any errors that occur.
     */
    static final Logger m_logger = Logger.getLogger("wirelessft");
    
    /**
     * Sets the demo model to use for communicating with the WNet.
     * 
     * @param model The model to use.
     */
    public void setDemoModel(WirelessFTDemoModel model) 
    {
        m_model = model;
        m_progressUpload.setProgress(0);
        m_lblProgress.setText("Choose file(s) to upload");
        
        m_model.checkFileUploadStatus().addListener(new ChangeListener<Boolean>() 
        {
            /**
             * Monitors the status of the file upload
             * job. Once the job is done, the user is
             * notified with "job complete" label.
             * 
             * @param ov Whether or not the job is done.
             * @param t  Was the job finished before?
             * @param t1 Is it now?
             */
            @Override
            public void changed(ObservableValue<? extends Boolean> ov, Boolean t, Boolean t1) 
            {
                if (t1) 
                {
                    m_lblProgress.setText   ("File upload complete"); // (3)
                    m_logger.log(Level.INFO, "File upload complete"); // (3)
                    m_btnUpload.setDisable(false);
                    m_model.m_flagFileUploadComplete.set(false);

                    if (m_model.getNeedWnetReset()) // If one of the files uploaded was appl.bin, the user needs to reset the Wnet manually.
                    {
                        try 
                        {
                            PopupDialogController.PopUpDialogBox("File upload complete", "Press the WNet power button to complete the bootloading process.");
                        } 
                        catch (IOException ex) 
                        {
                            Logger.getLogger(WirelessFTDemoModel.class.getName()).log(Level.SEVERE, null, ex);
                        }
                    }
                }
            }
        });
        
        m_progressUpload.progressProperty().addListener(new ChangeListener<Number>() 
        {
            /**
             * Listens for progress indication from the upload task.
             * Once the upload task is done (or if it hasn't
             * started yet), the Upload button will be
             * available.
             * 
             * @param ov The percent value (0-100) of the upload job.
             * @param t  The previous value.
             * @param t1 The current value.
             */
            @Override
            public void changed(ObservableValue<? extends Number> ov, Number t, Number t1) 
            {
                boolean uploadButtonOn = t1.doubleValue() == 0;
                
                try
                {
                    m_btnUpload.setDisable(!uploadButtonOn); // This function really should be called from the FX application thread
                }
                catch (IllegalStateException e) // kludge
                {
                }
            }
        });
    }
    
    /**
     * Initializes the controller class.
     * 
     * @param url Not used.
     * @param rb  Not used.
     */
    @Override
    public void initialize(URL url, ResourceBundle rb) 
    {
        m_btnUpload.setPrefWidth(m_txtFilePath.getPrefWidth() +  m_btnChooseFile.getPrefWidth());
    }  
    
    /**
     * Opens a file chooser to pick one or more files
     * that will be written to the digital board once
     * the Upload button is pressed.
     */
    @FXML
    protected void chooseFiles() 
    {
        Preferences prefs    = WirelessFTDemoMainScreenController.m_prefsRoot.node(PREF_USER_ROOT_NODE_PATHNAME);
        String      pathname = prefs.get(PREF_FILE_UPLOAD_DIRECTORY, null);
        
        try 
        {
            FileChooser chooser = new FileChooser();
            chooser.setTitle("Choose one or more files to upload:");
            
            if (pathname != null) 
            {
                try 
                {
                    chooser.setInitialDirectory(new File(pathname));
                    m_files = chooser.showOpenMultipleDialog(((Node)m_btnChooseFile).getScene().getWindow());
                } 
                catch (Exception e) // User's last preferred directory was deleted.
                {
                    chooser.setInitialDirectory(null);
                    m_files = chooser.showOpenMultipleDialog(((Node)m_btnChooseFile).getScene().getWindow());
                }
            } 
            else  // No preferred directory (first use).
            {
                chooser.setInitialDirectory(null);
                m_files = chooser.showOpenMultipleDialog(((Node)m_btnChooseFile).getScene().getWindow());
            }
            
            if (m_files != null) 
            {
                boolean first = true;
                for (File f : m_files) 
                {
                    if (!f.getPath().isEmpty()) 
                    {
                        if (!first) 
                        {
                            m_txtFilePath.appendText(" ... ");
                        } 
                        else 
                        {
                            if (f.getParentFile().exists()) 
                            {
                                chooser.setInitialDirectory(f.getParentFile());
                                prefs.put(PREF_FILE_UPLOAD_DIRECTORY, f.getParentFile().getAbsolutePath());
                            }
                            first = false;
                        }
                        m_txtFilePath.appendText(f.getAbsolutePath());
                    }
                }
            }
        } 
        catch (NullPointerException npe)  // Do nothing, they didn't pick a valid file.
        {
        }
    }
    
    /**
     * Starts a thread to write each file to the WNet
     * and links UI elements of the FileUploadScreen
     * to the progress of that thread's task.
     */
    @FXML
    protected void startFileUpload() 
    {
        if (m_files != null)
        {
            if (!m_files.isEmpty()) 
            {
                try 
                {
                    m_progressUpload.progressProperty().bind(m_model.fileUploadProgress());
                    m_lblProgress.setText("File(s) uploading");
                    m_model.startFileUpload(m_files, m_model);
                } 
                catch (IOException ioexc) 
                {
                    m_logger.log(Level.WARNING, ioexc.getMessage());
                }
            }
        }
    }
}
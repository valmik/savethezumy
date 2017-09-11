package wirelessftjavademo;

import java.io.File;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import org.w3c.dom.DOMException;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

/**
 * Represents the XML profiles used by
 * the Java Demo to set the state of a
 * connected WNet.
 * 
 * @author colch
 */
public class WirelessFTDemoProfile 
{
    // ---------- Default Settings ---------- \\
    public static final String DEFAULT_RATE               = "125";
    public static final String DEFAULT_OVERSAMPLING       = "32";
    public static final String DEFAULT_SD                 = "OFF";
    public static final String DEFAULT_FORCE_UNITS        = "Default";
    public static final String DEFAULT_TORQUE_UNITS       = "Default";
    public static final String DEFAULT_FILTER_TYPE        = "Running Mean";
    public static final String DEFAULT_FILTER_VALUE       = "8";
    public static final String DEFAULT_DISPLACEMENT_UNITS = "m";
    public static final String DEFAULT_ROTATION_UNITS     = "Degrees";
    
    public static final String[]   DEFAULT_XPWR    = {"ON","OFF","OFF","OFF","OFF","OFF",};
    
    public static final String[][] DEFAULT_FILTERS = {{DEFAULT_FILTER_TYPE, DEFAULT_FILTER_VALUE},
                                                      {DEFAULT_FILTER_TYPE, DEFAULT_FILTER_VALUE},
                                                      {DEFAULT_FILTER_TYPE, DEFAULT_FILTER_VALUE},
                                                      {DEFAULT_FILTER_TYPE, DEFAULT_FILTER_VALUE},
                                                      {DEFAULT_FILTER_TYPE, DEFAULT_FILTER_VALUE},
                                                      {DEFAULT_FILTER_TYPE, DEFAULT_FILTER_VALUE}};
    
    public static final String[] DEFAULT_CALS = {"0","0","0","0","0","0"}; // Calibration defaults = "Default"
    
    public static final String[][][] DEFAULT_XFORMS = 
    {
        {{DEFAULT_DISPLACEMENT_UNITS, "0.0","0.0","0.0"}, {DEFAULT_ROTATION_UNITS, "0.0","0.0","0.0"}},
        {{DEFAULT_DISPLACEMENT_UNITS, "0.0","0.0","0.0"}, {DEFAULT_ROTATION_UNITS, "0.0","0.0","0.0"}},
        {{DEFAULT_DISPLACEMENT_UNITS, "0.0","0.0","0.0"}, {DEFAULT_ROTATION_UNITS, "0.0","0.0","0.0"}},
        {{DEFAULT_DISPLACEMENT_UNITS, "0.0","0.0","0.0"}, {DEFAULT_ROTATION_UNITS, "0.0","0.0","0.0"}},
        {{DEFAULT_DISPLACEMENT_UNITS, "0.0","0.0","0.0"}, {DEFAULT_ROTATION_UNITS, "0.0","0.0","0.0"}},
        {{DEFAULT_DISPLACEMENT_UNITS, "0.0","0.0","0.0"}, {DEFAULT_ROTATION_UNITS, "0.0","0.0","0.0"}},
    };
    
    /**
     * Conversion constants.
     */
    private final double CONVERT_DISP_INCHES_IN      =  1;
    private final double CONVERT_DISP_FEET_FT        = 12;
    private final double CONVERT_DISP_METERS_M       = 39.3701;
    private final double CONVERT_DISP_CENTIMETERS_CM =  0.393701;
    private final double CONVERT_DISP_MILLIMETERS_MM =  0.0393701;
    
    private final double[] m_displacementConversionFactors = {
        CONVERT_DISP_INCHES_IN, CONVERT_DISP_FEET_FT, CONVERT_DISP_METERS_M,
        CONVERT_DISP_CENTIMETERS_CM, CONVERT_DISP_MILLIMETERS_MM};
    
    public static final int NUM_SENSORS = 6;
    
    /**
     * 6x6 Identity matrices for each sensor. 
     */
    public static final double[][][] IDENTITY_MATRICES = {
        {
            {1, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0},
            {0, 0, 0, 1, 0, 0}, {0, 0, 0, 0, 1, 0}, {0, 0, 0, 0, 0, 1}
        },
        {
            {1, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0},
            {0, 0, 0, 1, 0, 0}, {0, 0, 0, 0, 1, 0}, {0, 0, 0, 0, 0, 1}
        },
        {
            {1, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0},
            {0, 0, 0, 1, 0, 0}, {0, 0, 0, 0, 1, 0}, {0, 0, 0, 0, 0, 1}
        },
        {
            {1, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0},
            {0, 0, 0, 1, 0, 0}, {0, 0, 0, 0, 1, 0}, {0, 0, 0, 0, 0, 1}
        },
        {
            {1, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0},
            {0, 0, 0, 1, 0, 0}, {0, 0, 0, 0, 1, 0}, {0, 0, 0, 0, 0, 1}
        },
        {
            {1, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0},
            {0, 0, 0, 1, 0, 0}, {0, 0, 0, 0, 1, 0}, {0, 0, 0, 0, 0, 1}
        }
    };
    
    // ----------      ^END^      ---------- \\
    
    /**
     * The rate at which to send UDP packets,
     * initially the default profile's value.
     */
    public String m_rate = DEFAULT_RATE;
    
    /**
     * The rate at which to sample transducer data,
     * initially the default profile's value.
     */
    public String m_oversampling = DEFAULT_OVERSAMPLING;
    
    /**
     * Whether or not the MicroSD is recording,
     * initially the default profile's value.
     */
    public String m_sd = DEFAULT_SD;
    
    /**
     * The units used to measure force,
     * initially the default profile's value.
     */
    public String m_force = DEFAULT_FORCE_UNITS;
    
    /**
     * The units used to measure torque,
     * initially the default profile's value.
     */
    public String m_torque = DEFAULT_TORQUE_UNITS;
    
    /**
     * The active transducers,
     * initially the default profile's value.
     */
    public String[] m_xpwr = DEFAULT_XPWR;
    
    /**
     * The filters,
     * initially the default profile's value.
     */
    public String[][] m_filters = DEFAULT_FILTERS;
    
    /**
     * The active calibrations,
     * initially the default profile's value.
     * 
     * Format: [command to set transducer]|[calibration for that transducer]
     */
    public String[] m_cals = DEFAULT_CALS;
    
    /**
     * The rotations and displacements as strings,
     * initially the default profile's value.
     */
    public String[][][] m_xforms = DEFAULT_XFORMS;
    
    /**
     * The rotation and displacement values
     * initially the identity.
     */
    private double[][][] m_xformValues = IDENTITY_MATRICES;
    
    public String m_Notes = "";
    
    public int m_Wnet = 3; // WNet-3 or WNet-6
    
    public boolean m_ntpUse           = false;
    public String  m_ntpServer        = "";
    public int     m_ntpOffsetHours   = 0;
    public int     m_ntpOffsetMinutes = 0;
    public boolean m_ntpDst           = true;
    
    /**
     * Constructs a new profile with the
     * default settings for a new WNet.
     */
    public WirelessFTDemoProfile() 
    {
        m_rate         = DEFAULT_RATE;
        m_oversampling = DEFAULT_OVERSAMPLING;
        m_sd           = DEFAULT_SD;
        m_force        = DEFAULT_FORCE_UNITS;
        m_torque       = DEFAULT_TORQUE_UNITS;
        m_xpwr         = DEFAULT_XPWR;
        m_filters      = DEFAULT_FILTERS;
        m_cals         = DEFAULT_CALS;
        m_xforms       = DEFAULT_XFORMS;
        m_Notes        = "";
        m_Wnet         = 3;

        m_ntpUse           = false;
        m_ntpServer        = "";
        m_ntpOffsetHours   = 0;
        m_ntpOffsetMinutes = 0;
        m_ntpDst           = true;
    }
    
    /**
     * Constructs a custom WNet profile
     * based on a given XML file.
     * 
     * @param profileXML The file to parse.
     * @throws Exception If file-parse failed.
     */
    public WirelessFTDemoProfile(File profileXML) throws Exception 
    {
        if (    profileXML == null 
            || !profileXML.exists() 
            || !profileXML.getAbsolutePath().toUpperCase().endsWith("XML")) 
        {
            throw new IllegalArgumentException("No XML file provided.");
        }
        
        parseXMLToWNetProfile(profileXML); // Read the xml file
    }
    
    /**
     * Populates the fields of this demo profile with
     * settings from a valid-format XML profile.
     * 
     * @param profileXML The file to parse.
     * @throws Exception If the file cannot be parsed for any reason.
     */
    private void parseXMLToWNetProfile(File profileXML) throws Exception // Read the xml file
    {
        // Read a settings profile.
        DocumentBuilder docBuilder = DocumentBuilderFactory.newInstance().newDocumentBuilder();
        
        // Root element
        Document doc = docBuilder.parse(profileXML != null ? profileXML :
                (new File("src/wirelessftjavademo/DefaultProfile.xml")));
        Element  root    = doc.getDocumentElement();
        NodeList profile = root.getChildNodes();
        
        /**
         * General settings
         */
        {
            NodeList general   = profile.item(0).getChildNodes();

            { // Transmit rate
                m_rate         = general.item(0).getTextContent();
            }
            { // Oversampling rate
                m_oversampling = general.item(1).getTextContent();
            }
            { // MicroSD Recording
                m_sd           = general.item(2).getTextContent();
            }
            { // Force units
                m_force        = general.item(3).getTextContent();
            }
            { // Torque units
                m_torque       = general.item(4).getTextContent();
            }
            { // Active transducers
                NodeList transducers = general.item(5).getChildNodes();
                
                for (int i = 0; i < NUM_SENSORS; i++) 
                {
                    m_xpwr[i] = transducers.item(i).getTextContent();
                }
            }
            
            try // Try to read the Notes field from the XML file.
            {
                m_Notes = general.item(6).getTextContent();
            }
            catch (Exception e) // If the Notes field is not there,
            {
                m_Notes = "";   // put in the default.
            }
            
            try // Try to read the WNet field from the XML file.
            {
                String wnet = general.item(7).getTextContent();
                m_Wnet = Integer.parseInt(wnet);
            }
            catch (DOMException | NumberFormatException e) // If the WNet field is not there,
            {
                m_Wnet = 3;     // put in the default.
            }
            
            try // Try to read the NTP on/off field from the XML file.
            {
                m_ntpUse = general.item(8).getTextContent().equals("ON");
            }
            catch (Exception e) // If the NTP on/off field is not there,
            {
                m_ntpUse = false;                          // put in the default.
            }
           
            try // Try to read the NTP server field from the XML file.
            {
                m_ntpServer = general.item(9).getTextContent();
            }
            catch (Exception e) // If the NTP server field is not there,
            {
                m_ntpServer = "";                          // put in the default.
            }
            
            try // Try to read the NTP hours field from the XML file.
            {
                m_ntpOffsetHours = Integer.parseInt(general.item(10).getTextContent());
            }
            catch (DOMException | NumberFormatException e) // If the NTP hours field is not there,
            {
                m_ntpOffsetHours = 0;                      // put in the default.
            }
            
            try // Try to read the NTP minutes field from the XML file.
            {
                m_ntpOffsetMinutes = Integer.parseInt(general.item(11).getTextContent());
            }
            catch (DOMException | NumberFormatException e) // If the NTP minutes field is not there,
            {
                m_ntpOffsetMinutes = 0;                    // put in the default.
            }
            
            try // Try to read the NTP DST field from the XML file.
            {
                m_ntpDst = general.item(12).getTextContent().equals("ON");
            }
            catch (Exception e) // If the NTP DST field is not there,
            {
                m_ntpDst = false;                          // put in the default.
            }
        }

        /**
         * Filters (averaging) for each transducer
         */
        {
            NodeList filters = profile.item(1).getChildNodes();

            for (int i = 0; i < NUM_SENSORS; i++) 
            {
                NodeList trans  = filters.item(i).getChildNodes();
                m_filters[i][0] = trans.item(0).getTextContent();
                m_filters[i][1] = trans.item(1).getTextContent();
            }
        }

        /**
         * Calibration index for each transducer
         */
        {
            NodeList calibration = profile.item(2).getChildNodes();

            for (int i = 0; i < NUM_SENSORS; i++) 
            {
                m_cals[i] = calibration.item(i).getTextContent();
            }
        }

        /**
         * Transformations for each transducer
         */
        {
            NodeList transformation = profile.item(3).getChildNodes();
            
            final int UNITS_AND_VALUES = 4;
            
            for (int i = 0; i < NUM_SENSORS; i++) 
            {
                NodeList trans = transformation.item(i).getChildNodes();
                { // Displacement
                    NodeList disp = trans.item(0).getChildNodes();
                    for(int j = 0; j < UNITS_AND_VALUES; j++) 
                    {
                        m_xforms[i][0][j] = disp.item(j).getTextContent();
                    }
                }
                { // Rotation
                    NodeList rot = trans.item(1).getChildNodes();
                    for(int j = 0; j < UNITS_AND_VALUES; j++) 
                    {
                        m_xforms[i][1][j] = rot.item(j).getTextContent();
                    }
                }
            }
        }
    }
    
    public void setForceUnits(String units) {
        m_force = units;
    }
    
    public void setTorqueUnits(String units) {
        m_torque = units;
    }
    
    public void setRate(int rate) {
        m_rate = "" + rate;
    }
    
    public void setOversampling(int ovrsmp) {
        m_oversampling = "" + ovrsmp;
    }
    
    public void setSD(boolean on) {
        m_sd = (on ? "ON" : "OFF");
    }
    
    public void setXPWR(int transducer, boolean on) 
    {
        m_xpwr[transducer] = (on ? "ON" : "OFF");
    }
    
    public void setFilterType(int transducer, String type) 
    {
        m_filters[transducer][0] = type;
    }
    
    public void setFilterValue(int transducer, int val) 
    {
        m_filters[transducer][1] = "" + val;
    }
    
    public void setCal(int transducer, int cal) 
    {
        m_cals[transducer] = "" + cal;
    }
    
    public void setDisplacementUnits(int transducer, String units) 
    {
        m_xforms[transducer][0][0] = units;
    }
    
    public void setDisplacementValue(int transducer, int val) 
    {
        m_xforms[transducer][0][1] = "" + val;
    }
    
    public void setRotationUnits(int transducer, String units) 
    {
        m_xforms[transducer][1][0] = units;
    }
    
    public void setRotationValue(int transducer, int val) 
    {
        m_xforms[transducer][1][1] = "" + val;
    }
    
    /**
     * Gets the command that should be written to
     * the firmware to set the UDP packet transmit
     * rate.
     * 
     * @return The rate command.
     */
    public String getRateCommand() 
    {
        return "RATE " + m_rate + " " + m_oversampling;
    }
    
    /**
     * Gets the demo force units.
     * 
     * @return The force units.
     */
    public String getForceUnits() {
        return m_force;
    }
    
    /**
     * Gets the demo torque units.
     * 
     * @return The torque units.
     */
    public String getTorqueUnits() {
        return m_torque;
    }
    
    /**
     * Gets the firmware command to set
     * whether or not data records will
     * be written to the WNet's MicroSD
     * card.
     * 
     * @return 
     */
    public String getSDCommand() 
    {
        return "SDREC " + m_sd;
    }
    
    /**
     * Gets the firmware command to set power
     * for a particular transducer index.
     * 
     * @param transducer The index of the transducer.
     * @return The xpwr command for that transducer.
     */
    public String getXPWRCommand(int transducer) 
    {
        return "XPWR " + (transducer + 1) + " " + m_xpwr[transducer]; // Transducer is origin 1 to the user
    }
    
    /**
     * Gets the filter type (MEAN, MEADIAN, or IIR)
     * for a particular transducer.
     * 
     * @param transducer The index of the transducer.
     * @return The filter type for that transducer.
     */
    public String getFilterType(int transducer) 
    {
        return m_filters[transducer][0]; // Zero-based in array.
    }
    
    /**
     * Gets the filter samples (or time constant)
     * for a particular transducer.
     * 
     * @param transducer The index of the transducer.
     * @return The filter value for that transducer.
     */
    public String getFilterValue(int transducer) 
    {
        return m_filters[transducer][1]; // Zero-based in array.
    }
    
    /**
     * Gets the firmware command to set filter
     * for a particular transducer index.
     * 
     * @param transducer The index of the transducer.
     * @return The filter command for that transducer.
     */
    public String getFilterCommand(int transducer)
    {
        String filter;
        String value;
        
        filter = m_filters[transducer][0]; // Transducer number is zero-based in array.
        value  = m_filters[transducer][1];
        
        switch (filter)
        {
            case "Running Mean":
                filter = "MEAN";
                break;
                
            case "Running Median":
                filter = "MEDIAN";
                break;
                
            case "IIR":
                filter = "IIR";
                break;
                
            case "No Filtering":
            default:
                filter = "MEAN";
                value  = "1";
                break;
        }
        
        return "FILTER " + (transducer + 1) + " " + filter + " " + value; // Transducer is origin 1 to the user
    }
    
    /**
     * Gets the active calibration
     * for a particular transducer.
     * 
     * @param transducer The index of the transducer.
     * @return The calibration index for that transducer.
     */
    public String getActiveCalibration(int transducer) 
    {
        return m_cals[transducer]; // Zero-based in array.
    }
    
    /**
     * Gets the firmware command to set calibration
     * for a particular transducer index.
     * 
     * @param transducer The transducer number, 0 to 5.
     * @return The TRANS command for that transducer.
     */
    public String getTransducerCommand(int transducer) 
    {
        return "TRANS " + (transducer + 1); // Transducer is origin 1 to the user
    }
    
    /**
     * Gets the firmware command to set calibration
     * for a particular transducer index.
     * 
     * @param transducer The transducer number, 0 to 5.
     * @return The CALIB command for that transducer.
     */
    public String getCalibrationCommand(int transducer) 
    {
        String cal = m_cals[transducer];
        String text;
        
        if (cal.contains("0"))
        {
            text = "";
        }
        else
        {
            text = "CALIB " + cal;
        }
        
        return text;
    }
    
    /**
     * Gets the displacement units for a particular
     * transducer.
     * 
     * @param transducer The index of the transducer.
     * @return The displacement units for the transducer.
     */
    public String getDisplacementUnits(int transducer) 
    {
        return m_xforms[transducer][0][0];// Zero-based in array.
    }
    
    /**
     * Gets the applied displacement for a
     * particular transducer index.
     * 
     * @param transducer The index of the transducer.
     * @return The displacement values for the transducer.
     * Format: [dx]|[dy]|[dz]
     */
    public String getDisplacementValues(int transducer) 
    {
        String xforms = "";
        
        for (int i = 1; i < m_xforms[transducer][0].length; i++) 
        {
            xforms += m_xforms[transducer][0][i] + "|";
        }
        
        return xforms.substring(0, xforms.length() - 1); // Remove trailing |
    }
    
    /**
     * Gets the rotation units for a particular
     * transducer.
     * 
     * @param transducer The index of the transducer.
     * @return The rotation units for the transducer.
     */
    public String getRotationUnits(int transducer) 
    {
        return m_xforms[transducer][1][0]; // Zero-based in array.
    }
    
    /**
     * Gets the applied rotation for a
     * particular transducer index.
     * 
     * @param transducer The index of the transducer.
     * @return The rotation values for the transducer.
     * Format: [rx]|[ry]|[rz]
     */
    public String getRotationValues(int transducer) 
    {
        String xforms = "";
        
        for (int i = 1; i < m_xforms[transducer][1].length; i++) 
        {
            xforms += m_xforms[transducer][1][i] + "|";
        }
        
        return xforms.substring(0, xforms.length() - 1); // Remove trailing |
    }
    
    /**
     * Gets the transformation matrix for a
     * particular transducer index.
     * 
     * @param transducer The zero-based index of the transducer.
     * 
     * @return The transformation matrix.
     */
    public double[][] getTransformationMatrix(int transducer) 
    {
        return m_xformValues[transducer];
    }
    
    /**
     * Applies tool transformations for each transducer,
     * setting the matrix locally to prevent doing the
     * transformation math each time a sample comes in.
     */
    public void applyTransformations() 
    {
        double[][][] rotations     = IDENTITY_MATRICES; // Rotations are done from scratch.
        double[][][] displacements = IDENTITY_MATRICES; // Displacements are a modified identity matrix (see below).
       
        boolean rChanged[] = new boolean[NUM_SENSORS]; // Did we apply a rotation/displacement to this transducer?
        boolean dChanged[] = new boolean[NUM_SENSORS];
        
        // Setup the rotation matrix.
        for (int i = 0; i < NUM_SENSORS; i++) 
        {
            String rotationUnits = getRotationUnits(i);
           
            if (!rotationUnits.isEmpty())  // Set the units for this rotation.
            {
                boolean radians;
                switch (rotationUnits) 
                {
                    case "Radians":
                        radians = true;
                        break;
                    case "Degrees":
                        radians = false;
                        break;
                    default: // No units selected, no rotation.
                        rChanged[i] = false;
                        continue;
                }
/**
 * Setup the rotation matrix.
 * 
 * ROTATION MATRIX FORMAT (c = cosine, s = sine):
 *          [0]            [1]                [2]             [3]             [4]               [5]
 * [0]   | cy*cz   (sx*sy*cz)+(cx*sz)  (sx*sz)-(cx*sy*cz)  |   0               0                 0         |
 * [1]   |-cy*sz   (-sx*sy*sz)+(cx*cz) (sx*cz)+(cx*sy*sz)  |   0               0                 0         |
 * [2]   |__sy___________-sx*cy______________cx*cy_________|___0_______________0_________________0_________|
 * [3]   |   0              0                  0           | cy*cz   (sx*sy*cz)+(cx*sz)  (sx*sz)-(cx*sy*cz)|
 * [4]   |   0              0                  0           |-cy*sz   (-sx*sy*sz)+(cx*cz) (sx*cz)+(cx*sy*sz)|
 * [5]   |   0              0                  0           |  sy           -sx*cy              cx*cy       |
 * 
 */
                try 
                {
                    double rX = Double.parseDouble(getRotationValues(i).split("|")[0]); // Get x, y, and z rotations.
                    double rY = Double.parseDouble(getRotationValues(i).split("|")[1]);
                    double rZ = Double.parseDouble(getRotationValues(i).split("|")[2]);
                   
                    if(radians)  // Convert radians to degrees, if applicable.
                    {
                        rX = Math.toDegrees(rX);
                        rY = Math.toDegrees(rY);
                        rZ = Math.toDegrees(rZ);
                    }
                    
                    rotations[i][0][0] =   Math.cos(rY) * Math.cos(rZ);                                           //  cy*cz
                    rotations[i][3][3] = rotations[i][0][0];
                    
                    rotations[i][0][1] =  (Math.sin(rX) * Math.sin(rY)*Math.cos(rZ))+(Math.cos(rX)*Math.sin(rZ)); // (sx*sy*cz)+(cx*sz)
                    rotations[i][3][4] = rotations[i][0][1];
                   
                    rotations[i][0][2] =  (Math.sin(rX) * Math.sin(rZ))-(Math.cos(rX)*Math.sin(rY)*Math.cos(rZ)); // (sx*sz)-(cx*sy*cz)
                    rotations[i][3][5] = rotations[i][0][2];
                   
                    rotations[i][1][0] =  -Math.cos(rY) * Math.sin(rZ);                                           // -cy*sz
                    rotations[i][4][3] = rotations[i][1][0];
                  
                    rotations[i][1][1] = (-Math.sin(rX) * Math.sin(rY)*Math.sin(rZ))+(Math.cos(rX)*Math.cos(rZ)); //(-sx*sy*sz)+(cx*cz)
                    rotations[i][4][4] = rotations[i][1][1];
                  
                    rotations[i][1][2] =  (Math.sin(rX) * Math.cos(rZ))+(Math.cos(rX)*Math.sin(rY)*Math.sin(rZ)); // (sx*cz)+(cx*sy*sz)
                    rotations[i][4][5] = rotations[i][1][2];
                   
                    rotations[i][2][0] =   Math.sin(rY);                                                          //  sy
                    rotations[i][5][3] = rotations[i][2][0];
                   
                    rotations[i][2][1] =  -Math.sin(rX) * Math.cos(rY);                                           // -sx*cy
                    rotations[i][5][4] = rotations[i][2][1];
                   
                    rotations[i][2][2] =   Math.cos(rX) * Math.cos(rY);                                           //  cx*cy
                    rotations[i][5][5] = rotations[i][2][2];
                    
                    rChanged[i] = true; // A valid rotation will be applied.
                } 
                catch(NumberFormatException nfe) 
                {
                    rChanged[i] = false; // Something was invalid, do not apply rotation.
                }
            } 
            else 
            {
                rChanged[i] = false; // No units, no rotation.
            }
        }
        
        for (int i = 0; i < NUM_SENSORS; i++)  // Setup the displacement matrix.
        {
            String displacementUnits = getDisplacementUnits(i);
           
            if(!displacementUnits.isEmpty())  // Set the units for this displacement.
            {
                double conversion;
                
                switch (displacementUnits) 
                {
                    case "in":
                        conversion = m_displacementConversionFactors[0];
                        break;
                    case "ft":
                        conversion = m_displacementConversionFactors[1];
                        break;
                    case "m":
                        conversion = m_displacementConversionFactors[2];
                        break;
                    case "cm":
                        conversion = m_displacementConversionFactors[3];
                        break;
                    case "mm":
                        conversion = m_displacementConversionFactors[4];
                        break;
                    default:
                        dChanged[i] = false; // No units selected, no displacement.
                        continue;
                }
                
                /**
                 * Setup the 3rd quadrant of the displacement matrix.
                 * 
                 * DISPLACMENT MATRIX FORMAT:
                 *         [0]  [1]  [2]   [3]  [4]  [5]
                 * [0]   |  1    0    0  |  0    0    0  |
                 * [1]   |  0    1    0  |  0    0    0  |
                 * [2]   |__0____0____1__|__0____0____0__|
                 * [3]   |  0    dz  -dy |  1    0    0  |
                 * [4]   | -dz   0    dx |  0    1    0  |
                 * [5]   |  dy  -dx   0  |  0    0    1  |
                 * 
                 */
                try 
                {
                    double dX = Double.parseDouble(getDisplacementValues(i).split("|")[0]); // Get x, y, and z displacements.
                    double dY = Double.parseDouble(getDisplacementValues(i).split("|")[1]);
                    double dZ = Double.parseDouble(getDisplacementValues(i).split("|")[2]);
                    
                    displacements[i][3][1] =        dZ * conversion; //  dz
                    displacements[i][3][2] = -1.0 * dY * conversion; // -dy
                    displacements[i][4][0] = -1.0 * dZ * conversion; // -dz
                    displacements[i][4][2] =        dX * conversion; //  dx
                    displacements[i][5][0] =        dY * conversion; //  dy
                    displacements[i][5][1] = -1.0 * dZ * conversion; // -dx
                    dChanged     [i]       = true;                   // A valid displacement will be applied.
                } 
                catch(NumberFormatException nfe) 
                {
                    dChanged[i] = false; // Something was invalid, do not apply displacement.
                }
            } 
            else 
            {
                dChanged[i] = false; // No units, no displacement.
            }
        }
       
        m_xformValues = matrixMult(rotations, displacements, rChanged, dChanged); // [Transformation] = [Rotations]*[Displacements]
        
        if (m_xformValues == null)             // If they somehow managed to give us invalid dimensions,
        {
            m_xformValues = IDENTITY_MATRICES; // reset to the identity matrix.
        }
    }
    
    /**
     * Steps through the 6 rotation/displacement matrices
     * and multiplies them together. If a matrix was not
     * populated via the Tool Transform settings, it is
     * treated as an identity matrix (no effect).
     * 
     * @param r The matrix of all rotations, valid or otherwise.
     * @param d The matrix of all displacements, valid or otherwise.
     * @param validR Which transducers are being rotated?
     * @param validD Which transducers are being displaced?
     * @return The resulting transformation matrices for the 6 transducers.
     */
    private double[][][] matrixMult(double[][][] r, double[][][] d, boolean[] validR, boolean[] validD) {
        if(r.length != d.length) return null; //invalid dims
        
        double ans[][][] = new double[NUM_SENSORS][NUM_SENSORS][NUM_SENSORS];

        for (int x = 0; x < NUM_SENSORS; x++) 
        {// 6 Matrices.
            // Only do math for valid transformations.
            if (validR[x] || validD[x]) 
            {
                for (int i = 0; i < NUM_SENSORS; i++)  
                { // 6 Rows each
                    for (int j = 0; j < NUM_SENSORS; j++) 
                    { // 6 Cols in d
                        for (int k = 0; k < NUM_SENSORS; k++) 
                        { // 6 Cols in r
                           ans[x][i][j] += r[x][i][k] * d[x][k][j];
                        }
                    }
                }
            } 
            else 
            { // Don't change this transducer (use the identity matrix).
                for (int i = 0; i < NUM_SENSORS; i++)
                {
                    ans[x][i][i] = 1.0;
                }
            }
        }
        
        return ans;
    }
}
/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package wirelessftjavademo;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

/**
 *
 * @author skusa
 */
public class CalibrationTest {
    
    public CalibrationTest() {
    }
    
    @BeforeClass
    public static void setUpClass() {
    }
    
    @AfterClass
    public static void tearDownClass() {
    }
    
    @Before
    public void setUp() {
    }
    
    @After
    public void tearDown() {
    }
    
    @Test
    /**
     * Test of getForceTorqueConversionFactors()
     */
    public void testgetForceTorqueConversionFactors()
    {
        int transducer = 0; // Transducer 1
        
        Calibration calTest = new Calibration();
        calTest.setForceUnits("A broken force");
        calTest.setTorqueUnits("Nm");
        try{
            calTest.getForceTorqueConversionFactors("lbf", "Nm", transducer);            
            assertTrue("No exception with bad calibration force units.", false);
        }catch(IllegalArgumentException i)
        {
            /* Expected. */
            System.out.println(i.getMessage());
        }
        calTest.setForceUnits("lbf");
        calTest.setTorqueUnits("One moment please");
        try{
            calTest.getForceTorqueConversionFactors("lbf", "Nm", transducer);
            assertTrue("No exception with bad calibration torque units.", false );
        }catch(IllegalArgumentException i )
        {
            /* Expected. */
            System.out.println(i.getMessage());                    
        }
        calTest.setTorqueUnits("N-m");
        try{
            calTest.getForceTorqueConversionFactors("A force for evil.", "Nm", transducer);
            assertTrue("No exception with bad requested force units.", false );
        }catch(IllegalArgumentException i )
        {
            /* Expected. */
            System.out.println(i.getMessage());            
        }
        try
        {
            calTest.getForceTorqueConversionFactors("lbf", "turtle", transducer);
            assertTrue("No exception with bad requested torque units.", false );
        }catch(IllegalArgumentException i )
        {
            /* Expected */
            System.out.println(i.getMessage());
        }
        
        /* calTest force units are lbf, torque units are Nm. */
        double[] results;
        
        String[] forceUnits    = { "lbf",    "klbf",   "n",   "kn",   "g",     "kg"   }; /* The supported force  units. */
        String[] torqueUnits   = { "lbf-in", "lbf-ft", "n-m", "n-mm", "kg-cm", "kn-m" }; /* The supported torque units. */
        
        double[] expForceConv  = { 1.0,     0.001,    4.44822,    0.004448222, 453.5924, 0.4535924 };
        double[] expTorqueConv = { 8.85075, 0.737561, 1.0,     1000,            10.1972, 0.001     };
        
        int i;
        
        for (i = 0; i < forceUnits.length; i++)
        {
            results = calTest.getForceTorqueConversionFactors(forceUnits[i], "n-m", transducer);
            assertTrue("Wrong force conversion to " + forceUnits[i], Math.abs( results[0] - expForceConv[i] ) < 0.001 );
        }
        
        for (i = 0; i < torqueUnits.length; i++)
        {
            results = calTest.getForceTorqueConversionFactors("lbf", torqueUnits[i], transducer);
            double diff = Math.abs(results[3] - expTorqueConv[i]);
            assertTrue("Wrong torque conversion to " + torqueUnits[i], diff < 0.001 );
        }
    }

    /**
     * Test of parseCalibrationFromTelnetResponse method, of class Calibration.
     */
    @Test
    public void testParseCalibrationFromTelnetResponse() {
        System.out.println("parseCalibrationFromTelnetResponse");
        /* Calibration text that caused a parse error due to units not being set. */
        String calCommandResponse = "CAL\r\n" +
"Tr Cal   Gain Offset Row           G0        G1        G2        G3        G4        G5  Properties\r\n" +
"-- ---   ---- ------ ---           --        --        --        --        --        --  ----------\r\n" +
" 2   0      0  32768   0 Fx         1         0         0         0         0         0  Serial: Serial-4\r\n" +
" 2   0      0  32768   1 Fy         0         1         0         0         0         0  Date:   1970/01/04\r\n" +
" 2   0      0  32768   2 Fz         0         0         1         0         0         0  Part:   Part-4\r\n" +
" 2   0      0  32768   3 Tx         0         0         0         1         0         0  Force:  0 counts/\r\n" +
" 2   0      0  32768   4 Ty         0         0         0         0         1         0  Torque: 0 counts/\r\n" +
" 2   0      0  32768   5 Tz         0         0         0         0         0         1  Mult:   ON\r\n" +
" 2                 0 MaxRatings:\r\n" +
" 2                 0                0         0         0         0         0         0\r\n" +
">";
        
        Calibration expResult = new Calibration();  
        expResult.setCalibrationDate("1970/01/04");
        expResult.setSerialNumber("Serial-4");
        expResult.setCountsPerUnitForce(0);
        expResult.setCountsPerUnitTorque(0);
        expResult.setForceUnits("");
        expResult.setTorqueUnits("");
        expResult.setPartNumber("Part-4");
        
        /* Create identity matrix. */
        float[][] identityMatrix = new float[6][6];
        for( int i = 0; i < 6; i++ )
        {
            for( int j = 0; j < 6; j++ )
            {
                if ( i == j )
                {
                    identityMatrix[i][j] = 1;
                }else
                {
                    identityMatrix[i][j] = 0;
                }
            }
        }               
        expResult.setMatrix(identityMatrix);      
        Calibration result = Calibration.parseCalibrationFromTelnetResponse(calCommandResponse);
        assertCalibrationEquals(expResult, result);        
    }
    
    private void assertCalibrationEquals(Calibration cal1, Calibration cal2)
    {
        assertEquals(cal1.getCalibrationDate(), cal2.getCalibrationDate());
        assertEquals(cal1.getCountsPerUnitForce(), cal2.getCountsPerUnitForce());
        assertEquals(cal1.getCountsPerUnitTorque(), cal2.getCountsPerUnitTorque());
        assertEquals(cal1.getForceUnits(), cal2.getForceUnits());
        assertEquals(cal1.getPartNumber(), cal2.getPartNumber());
        assertEquals(cal1.getSerialNumber(), cal2.getSerialNumber());
        assertEquals(cal1.getTorqueUnits(), cal2.getTorqueUnits());
        assertArrayEquals(flattenCalibrationMatrix(cal1.getMatrix()), flattenCalibrationMatrix(cal2.getMatrix()), 0.0001);
    }
    
    private double[] flattenCalibrationMatrix(float[][] matrix)
    {        
        double[] flattened = new double[matrix.length * matrix[0].length];
        for( int i = 0; i < matrix.length; i++ )
        {
            for( int j = 0; j < matrix[i].length; j++ )
            {
                flattened[i * matrix[i].length + j] = matrix[i][j];
            }                
        }
        return flattened;
    }
}
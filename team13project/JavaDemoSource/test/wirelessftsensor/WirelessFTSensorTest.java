/*
 * Copyright 2013
 * ATI Industrial Automation
 */
package wirelessftsensor;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.ByteBuffer;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;
import wirelessftjavademo.WirelessFTDemoModel;


/**
 *
 * @author Sam Skuce
 */
public class WirelessFTSensorTest {   
    
    private ServerSocket m_serverSocket = null;
    
    public WirelessFTSensorTest() {        
    }
    
    @BeforeClass
    public static void setUpClass() {
        
    }
    
    @AfterClass
    public static void tearDownClass(){
        
    }
    
    @Before
    public void setUp() throws IOException{
       m_serverSocket = new ServerSocket(23);
        
    }
    
    @After
    public void tearDown() throws IOException {
        m_serverSocket.close();
    }

    /**
     * Test of getSensorAddress method, of class WirelessFTSensor.
     */
    @Test    
    public void testGetSensorAddress() {
        System.out.println("getSensorAddress");
        WirelessFTSensor instance = new WirelessFTSensor();  
        String expResult = "";      /* Should default to an empty string. */
        String result = instance.getSensorAddress();   
        assertEquals(expResult, result); 
        expResult = "localhost";
        try{
            instance.setSensorAddress(expResult);
        }catch(IOException ioexc)
        {
            fail("Exception setting sensor address: " + ioexc.getMessage());
        }
        result = instance.getSensorAddress();
        assertEquals(expResult, result);        
        instance.endCommunication();
    }

    /**
     * Test of setSensorAddress method, of class WirelessFTSensor.
     */
    @Test
    public void testSetSensorAddress() throws Exception {
        System.out.println("setSensorAddress");
        String val = "this.is.not.a.valid.address.and.if.it.is.your.network.admin.is.crazy";
        WirelessFTSensor instance = new WirelessFTSensor();
        try{
            instance.setSensorAddress(val);
            fail("No IOException when passed invalid address.");            
        }catch(IOException ioexc)
        {
            /* Do nothing.  We hope to have this exception when we pass a bad address. */
        }
        val = "localhost";
        instance.setSensorAddress(val);
        assertEquals("setSensorAddress did not set localhost.", val, instance.getSensorAddress());

    }

    /**
     * Test of readStreamingSample method, of class WirelessFTSensor.
     * @throws java.lang.Exception
     */
    @Test
    public void testReadStreamingSample() throws Exception 
    {
        System.out.println("readStreamingSample");
        WirelessFTSensor instance = new WirelessFTSensor();
        instance.setSensorAddress("localhost");
        DatagramSocket socket = new DatagramSocket(49152);
        DatagramPacket packet = new DatagramPacket(new byte[1024], 1024);
        instance.startStreamingData();
        socket.receive(packet); /* Dummy receive just to get the address and port the instance is listening on. */
        byte[] samplePacket = new byte[90];
        ByteBuffer packetFiller = ByteBuffer.wrap(samplePacket);
        final int timeStamp = 0x12345678;        
        packetFiller.putInt(timeStamp);
        final int sequence = 0xaabbccdd;
        packetFiller.putInt(sequence);
        final int firstStatus = 0x552233aa;
        packetFiller.putInt(firstStatus);
        final int secondStatus = 0x83927561;
        packetFiller.putInt(secondStatus);
        final byte batteryLevel = (byte)0x80;
        packetFiller.put(batteryLevel);
        final byte transMask = (byte)0x07;
        packetFiller.put(transMask);
        int transducer, axis;
        int sample_data[][] = new int[3][6];
        for( transducer = 0; transducer < 3; transducer++)
        {
            for( axis = 0; axis < WirelessFTDemoModel.NUM_AXES; axis++ )
            {
                sample_data[transducer][axis] = (transducer + 1) * (axis + 2);
                packetFiller.putInt(sample_data[transducer][axis]);
            }
        }
        packet.setData(samplePacket);      
        socket.send(packet);
        WirelessFTSample result;
        result = instance.readStreamingSample();
        socket.close();
        
        assertEquals("Wrong timestamp",    timeStamp,    result.getTimeStamp());
        assertEquals("Wrong sequence",     sequence,     result.getSequence());
        assertEquals("Wrong status (1-3)", firstStatus,  result.getStatusCode1());
        assertEquals("Wrong status (4-6)", secondStatus, result.getStatusCode2());
        assertEquals("Wrong battery",      batteryLevel, result.getBatteryLevel());
        assertEquals("Wrong xdcr mask",    transMask,    result.getSensorMask());
        for( transducer = 0; transducer < 3; transducer++)
        {
            for( axis = 0; axis < WirelessFTDemoModel.NUM_AXES; axis++)
            {
                assertEquals(String.format("Wrong value for transducer %d axis %d", (transducer + 1), axis), // Transducer is origin 1 to the user
                        sample_data             [transducer][axis], 
                        result.getFtOrGageData()[transducer][axis]);
            }
        }        
    }

    

    /**
     * Test of readTelnetData method, of class WirelessFTSensor.
     */
    @Test
    public void testReadTelnetData() throws Exception 
    {
        System.out.println("readTelnetData");       
        WirelessFTSensor instance = new WirelessFTSensor();
        instance.setSensorAddress("localhost");
        final Socket instanceSocket = m_serverSocket.accept();
        final String expectedVal    = "hello\r\n";
        instanceSocket.getOutputStream().write(expectedVal.getBytes("US-ASCII"));
        String result = instance.readTelnetData(false);
        assertEquals("Wrong message received.", expectedVal, result);
        
        /* To test that it will block and wait for input data, send the text after a delay. */
        new Thread(new Runnable() 
        {
            @Override
            public void run() 
            {
                try
                {
                    Thread.sleep(2000);
                    instanceSocket.getOutputStream().write(expectedVal.getBytes("US-ASCII"));
                }
                catch(Exception exc)
                {
                    System.out.println("Exception sending delayed value: " + exc.getMessage());
                }
            }
        }).start();
        
        result = instance.readTelnetData(true);
        assertEquals("Wrong message received with blocking.", expectedVal, result);
        
        result = instance.readTelnetData(false);
        assertEquals("Didn't get empty value when not blocking and no input data", "", result);
       
    }
}
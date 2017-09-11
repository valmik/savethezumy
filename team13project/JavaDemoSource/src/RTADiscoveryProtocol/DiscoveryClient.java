/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package RTADiscoveryProtocol;

import java.io.IOException;
import java.net.*;
import java.util.Arrays;

/**
 *
 * @author fleda
 * Ported from RTADiscoveryProtocolClient.cs
 */
public class DiscoveryClient 
{
    private static final int    RECEIVE_PORT     = 28250; // The port to which discovery protocol responses are sent.
    private static final int    SEND_PORT        = 51000; // The port to which discovery protocol requests are sent.
    private static final String MULTICAST_IP     = "224.0.5.128"; // The Multicast IP address we use to listen for responses.
    private static final byte   DELAY_MULTIPLIER = 7;  // Delay multiplier sent with discovery request of 10 ms. Delay multipliers are used to spread out the responses from the devices and avoid collisions.
    private static final String DISCOVERY_REQUEST_HEADER  = "RTA Device DiscoveryRTAD"; // Beginning of every discovery request message. Text lets the device know that this is a discovery request
    private static final String DISCOVERY_RESPONSE_HEADER = "RTAD"; // Beginning of every RTA response message.
    private static final int    IP_FIELD_LENGTH  = 4; // At this time, all IP fields are of length 4.

    /* The following definitions were taken from the RTA Discovery Protocol Guide. */
    private static final int RTA_DISC_TAG_MAC  = 1;    // MAC,  same as Digi
    private static final int RTA_DISC_TAG_IP   = 2;    // IP,   same as Digi
    private static final int RTA_DISC_TAG_MASK = 3;    // Mask, same as Digi
    private static final int RTA_DISC_TAG_GW   = 0x0b; // Gateway, same as Digi
    private static final int RTA_DISC_TAG_HW   = 0x81; // Hardware platform
    private static final int RTA_DISC_TAG_APP  = 0x0d; // Application, same as Digi
    private static final int RTA_DISC_TAG_VER  = 8;    // Version, same as Digi
    private static final int RTA_DISC_TAG_SEQ  = 0x82; // Sequence number
    private static final int RTA_DISC_TAG_CRCS = 0x96; // CRC of selected parts of message
    private static final int RTA_DISC_TAG_CRC  = 0xf0; // CRC of entire message
    private static final int RTA_DISC_TAG_TICK = 0x83; // Clock tick when response message sent
    private static final int RTA_DISC_TAG_RND2 = 0x93; // Random number (for encryption)
    private static final int RTA_DISC_TAG_RND1 = 0x84; // Random number (for encryption)
    private static final int RTA_DISC_TAG_RND  = 0x94; // Random number (for encryption)
    private static final int RTA_DISC_TAG_PSWD = 0x85; // Encrypted password
    private static final int RTA_DISC_TAG_LOC  = (0x86 - 256); // Location of the unit // Compensate for the lack of an unsigned char type.
    private static final int RTA_DISC_TAG_DISC = 0x95; // Discovery SW revision
    private static final int RTA_DISC_TAG_MULT = 0xf2; // Response-delay multiplier for broadcast
    
    public static RTADevice[] discoverRTADevicesLocalBroadcast() throws UnknownHostException, IOException
    {
        return discoverRTADevicesLocalBroadcast(null);
    }

    /// <summary>
    /// Broadcasts a discovery request to your local network and listens for responses
    /// from RTA devices.  Even devices which aren't configured properly can be found
    /// with this method, because it requests the devices to respond to a multicast
    /// address, which allows the devices to ignore their default gateway, netmask,
    /// etc., and "just send" the response.
    /// </summary>
    /// <returns>The list of discovered devices.</returns>
    /// <param name="ipaLocal">The local interface to search from.  Setting this parameter
    /// to null causes the function to search from any available interface.</param>
    public static RTADevice[] discoverRTADevicesLocalBroadcast(InetAddress ipaLocal) throws UnknownHostException, IOException
    {
        int i;

        InetAddress    multicastAddress   = InetAddress.getByName(MULTICAST_IP); // Set up the multicast address
        byte           discoveryMessage[] = createMulticastDiscoveryRequest();   // Create the discovery request message
        DatagramPacket sendPacket         = new DatagramPacket(discoveryMessage, 0, discoveryMessage.length, InetAddress.getByName("255.255.255.255"), SEND_PORT); //create packet to send discovery message
        RTADevice[]    rtaList            = new RTADevice[0];                    // list of RTA Devices to be built
        
        MulticastSocket socket;                     // create multicast socket, used for both sending and receiving
        socket = new MulticastSocket(RECEIVE_PORT);
        socket.joinGroup(multicastAddress);         // joing multicast group
        socket.setBroadcast(true);                  // set broadcast settings
        socket.setSoTimeout(257 * DELAY_MULTIPLIER); // Set socket timeout.
        socket.send(sendPacket);                    // send discovery packet
            
        while (true)                                // Read one discovery response (or timeout)
        {
            try
            {
                byte           receiveMessage[] = new byte[205]; // create receive buffer
                DatagramPacket receivePacket    = new DatagramPacket(receiveMessage, receiveMessage.length); // create response packet
                socket.receive(receivePacket); // Get UDP discovery response packet. BLOCKS until packet received or timeout
                RTADevice      tempRTA          = parseDiscoveryResponse(receiveMessage); // Parse received discovery response packet
                
                if (tempRTA != null)                         // If we got a discovery response packet,
                {
                    int ip1 = tempRTA.m_ipa.getAddress()[3]; // Get last byte of IP address that we just read
                    if (ip1 < 0) ip1 += 256;                 // Compensate for the lack of an unsigned char type.
                    socket.setSoTimeout((257 - ip1) * DELAY_MULTIPLIER); // Set new socket timeout based on IP address
                       
                    RTADevice[] tempRTAList = new RTADevice[rtaList.length + 1]; // add new device to list of devices
                    for (i = 0; i < rtaList.length; i++) 
                    {
                        tempRTAList[i] = rtaList[i];
                    }
                    tempRTAList[i] = tempRTA;
                    rtaList        = tempRTAList;
                }
            }
            catch (SocketTimeoutException ste) // This is the normal loop exit
            {
                break;
            }
            catch (IOException e)
            {
                System.out.println("Exception: " + e.getMessage() + " | " + Arrays.toString(e.getStackTrace()));
                break;
            }
        }
        
        socket.leaveGroup(multicastAddress); // close multicast socket
        socket.close();
        return rtaList;
    }

    /// <summary>
    /// Creates a discovery request message with a multicast reply-to address.
    /// </summary>
    /// <returns>The discovery request message.</returns>
    private static byte[] createMulticastDiscoveryRequest()
    {
        //array to be returned
        byte discoveryRequest[] = new byte[DISCOVERY_REQUEST_HEADER.length() + 9];
        int i;

        //Place DISCOVERY_REQUEST_HEADER in bytes into discoveryRequest[]
        for (i = 0; i < DISCOVERY_REQUEST_HEADER.length(); i++)
        {
            discoveryRequest[i] = (byte)DISCOVERY_REQUEST_HEADER.charAt(i);
        }

        discoveryRequest[i++] = (byte)RTA_DISC_TAG_IP;
        discoveryRequest[i++] = (byte)4; // length of an IP address

        // Add in the IP address
        String[] ipValues = MULTICAST_IP.split("[.]+"); // split operand is a regular expression

        for (int index = 0; index < 4; index++)
        {
            discoveryRequest[i++] = (byte)Integer.parseInt(ipValues[index]);
        }
        
        discoveryRequest[i++] = (byte)RTA_DISC_TAG_MULT;
        discoveryRequest[i++] = 1; // length of delay multiplier
        discoveryRequest[i  ] = (byte)DELAY_MULTIPLIER;
        
        return discoveryRequest;
    }

    /// <summary>
    /// Parses an RTADevice structure from a discovery response from an RTA device.
    /// </summary>
    /// <param name="abResponse">The response data received from the network.</param>
    /// <returns>The RTADeviceStructure represented by the response, or null
    /// if the response does not contain a valid discovery protocol response.</returns>
    private static RTADevice parseDiscoveryResponse(byte[] abResponse) throws UnknownHostException
    {
        RTADevice rtad = new RTADevice(); // the new RTADevice to be returned
        int i;
       
        for (i = 0; i < DISCOVERY_RESPONSE_HEADER.length(); i++) //first check response for correct header
        {
            if(abResponse[i] != (char)DISCOVERY_RESPONSE_HEADER.charAt(i)) return null;
        }

        //the message had the correct header, ready to start parsing message
        while (i < abResponse.length - 1)
        {
            int j;
            int iFieldLength = abResponse[i + 1];//byte after tag specifier is the field length

            if ((i + 2 + iFieldLength) > abResponse.length)
            {
                return null; //the field length specified goes beyond the end of the packet, packet is invalid
            }

            switch (abResponse[i])
            {
                case RTA_DISC_TAG_MAC: // Mac address field
                    rtad.m_macstring = "";
                    
                    for (int k = 0; k < iFieldLength; k++)
                    {
                        //note:  I am conditionally adding 256 to the bytes from abResponse because java falsely thinks they
                        //represent negative numbers instead of unsigned positive numbers.  Java does not support
                        //unsigned variables, so I took a roundabout way to accomplish the same task.
                        int value =   (abResponse[i + 2 + k] >= 0) 
                                    ? (abResponse[i + 2 + k])
                                    : (abResponse[i + 2 + k] + 256);
                        
                        if (k > 0)
                        {
                            rtad.m_macstring += "-";
                        }
                        rtad.m_macstring += String.format("%02x", value); //generate two characters for mac address fields
                    }
                    break;
                    
                case RTA_DISC_TAG_IP://IP Address field
                    if (iFieldLength != IP_FIELD_LENGTH) return null; //IP fields must be of length 4
                    rtad.m_ipa = IPFromSubArray(abResponse, i + 2);
                    break;
                    
                case RTA_DISC_TAG_MASK://Netmask Field
                    if (iFieldLength != IP_FIELD_LENGTH) return null; //Mask is an IP, so it has length 4
                    rtad.m_ipaNetmask = IPFromSubArray(abResponse, i + 2);
                    break;
                    
                case RTA_DISC_TAG_APP: //Application Description field
                    rtad.m_strApplication = "";
                    /* Precondition: iFieldLength = length of description field in
                    * response, abResponse = response from RTA device, i = position
                    * of application description tag in response, rtad.m_strApplication
                    * = "".
                    * Postcondition: rtad.m_strApplication = The application
                    * description from the response, j = length of the description.*/
                    for (j = 0; j < iFieldLength; j++)
                    {
                        rtad.m_strApplication += (char)abResponse[i + 2 + j];
                    }
                    break;
                    
                case RTA_DISC_TAG_LOC:
                    rtad.m_strLocation = "";
                    for (j = 0; j < iFieldLength; j++)
                    {
                        rtad.m_strLocation += (char)abResponse[i + 2 + j];
                    }
                    break;
                    
                default:
                    break;
            }
           
            i += iFieldLength + 2; //Move i to next tag
        }
        return rtad;
    }

    private static InetAddress IPFromSubArray(byte abPacket[], int iStartPos) throws UnknownHostException
    {
        byte abSubArray[] = new byte[IP_FIELD_LENGTH];//the subarray containing ip address
        int i;

        /* Precondition: abSubArrray has IP_FIELD_LENGTH slots, abPacket = the response
        * packet, iStartPos = the position of the IP address in the response packet.
        * Postcondition: abSubArray has the elements of the IP address, i ==
        * IP_FIELD_LENGTH.*/
        for (i = 0; i < IP_FIELD_LENGTH; i++)
        {
            abSubArray[i] = abPacket[iStartPos + i];
        }
       
        return InetAddress.getByAddress(abSubArray); //return new InetAddress(abSubArray);
    }
}

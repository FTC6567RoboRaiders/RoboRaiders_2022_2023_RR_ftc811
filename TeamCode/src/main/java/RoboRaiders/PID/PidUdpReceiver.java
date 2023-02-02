package RoboRaiders.PID;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.ByteBuffer;


/**
 * PidIpdReceiver - will facilitate the sending and receiving of PID values
 *
 */
public class PidUdpReceiver
{
    private int port;
    private double p, i, d, degrees, direction;
    private Thread backgroundThread;

    public PidUdpReceiver(int port)
    {
        this.port = port;
    }

    public PidUdpReceiver()
    {
        this(8087);
    }

    public void beginListening()
    {
        backgroundThread = new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                listen();
            }
        });

        backgroundThread.start();
    }

    public void shutdown()
    {
        backgroundThread.interrupt();
    }

    public synchronized double getP()
    {
        return p;
    }

    public synchronized double getI()
    {
        return i;
    }

    public synchronized double getD()
    {
        return d;
    }

    public synchronized double getDegrees()
    {
        return degrees;
    }

    public synchronized double getDirection(){ return direction; }

    private void listen()
    {
        try
        {
            DatagramSocket serverSocket = new DatagramSocket(port);
            byte[] packet = new byte[40];

            System.out.printf("Listening on udp:%s:%d%n", InetAddress.getLocalHost().getHostAddress(), port);
            DatagramPacket receivePacket = new DatagramPacket(packet, packet.length);

            while (!Thread.currentThread().isInterrupted())
            {
                serverSocket.receive(receivePacket);

                byte[] pVal = new byte[8];
                byte[] iVal = new byte[8];
                byte[] dVal = new byte[8];
                byte[] degVal = new byte[8];
                byte[] dirVal = new byte[8];

                System.arraycopy(packet, 0, pVal, 0, 8);
                System.arraycopy(packet, 8, iVal, 0, 8);
                System.arraycopy(packet, 16, dVal, 0, 8);
                System.arraycopy(packet, 24, degVal, 0, 8);
                System.arraycopy(packet, 32, dirVal, 0, 8);

                p = toDouble(pVal);
                i = toDouble(iVal);
                d = toDouble(dVal);
                degrees = toDouble(degVal);
                direction = toDouble(dirVal);
            }

            serverSocket.close();
        }

        catch (IOException e)
        {
            e.printStackTrace();
        }
    }

    private double toDouble(byte[] bytes)
    {
        return ByteBuffer.wrap(bytes).getDouble();
    }
}
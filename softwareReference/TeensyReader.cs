using UnityEngine;
using System.IO.Ports;
using System;
using System.Threading;
using System.Collections.Concurrent;
using UnityEngine.Rendering.Universal;

public class TeensyReceiver : MonoBehaviour
{
    // Adjust this to match your Teensy port
    public string portName = ""; // You can set this from the TeensyDetector script or hardcode it
    private SerialPort _serialPort;
    private Thread _readThread;
    private bool _isRunning = false;
    private ConcurrentQueue<byte[]> _dataQueue = new ConcurrentQueue<byte[]>();

    void Start()
    {
        SerialScanner scanner = new SerialScanner();
        portName = scanner.portName; // Get the detected port from the scanner
        // Initialize the port
        _serialPort = new SerialPort(portName, 9600);
        Debug.Log($"Attempting to connect to Teensy on {portName}...");
        try
        {
            _serialPort.ReadTimeout = 10; // Low timeout to keep Unity smooth
            _serialPort.Open();
            Debug.Log($"Connected to Teensy on {portName}");
            _readThread = new Thread(ReadSerialLoop);
            _readThread.IsBackground = true;
            _isRunning = true;
            _readThread.Start();
            Debug.Log("Started serial read thread.");
        }
        catch (Exception e)
        {
            Debug.LogError($"Could not open serial port: {e.Message}");
        }
    }

    void ReadSerialLoop()
    {
        while (_isRunning)
        {
            if (_serialPort != null && _serialPort.IsOpen)
            {
                Debug.Log("Serial port is open, checking for data...");
                try
                {
                    // Check if there is data waiting
                    int bytesToRead = _serialPort.BytesToRead;
                    if (bytesToRead > 0)
                    {
                        byte[] buffer = new byte[bytesToRead];
                        _serialPort.Read(buffer, 0, bytesToRead);

                        // Log the raw bytes to the Unity Console
                        string hex = BitConverter.ToString(buffer).Replace("-", " ");
                        string decimalList = string.Join(", ", buffer);

                        Debug.Log($"<color=cyan>Received Bytes:</color> {decimalList} (Hex: {hex})");
                        _dataQueue.Enqueue(buffer); // Store the data for processing in the main thread
                    }
                    // else {
                    //     Debug.Log("No data available to read.");
                    // }
                }
                // catch (TimeoutException) {
                //     Debug.LogWarning("Serial read timed out. No data received.");
                // } // Ignore timeouts
                catch (Exception e)
                {
                    Debug.LogError($"Error reading serial: {e.Message}");
                }
            }
            else
            {
                Debug.LogWarning("Serial port is not open. Cannot read data.");
            }
        }
    }

    void Update()
    {
        // Process any received data in the main thread
        while (_dataQueue.TryDequeue(out byte[] data))
        {
            ProcessData(data);
        }
    }

    void ProcessData(byte[] data)
    {
        // Example: Log the first byte as an integer
        if (data.Length > 0)
        {
            int value = data[0]; // Assuming the first byte is the value you want
            Debug.Log($"<color=yellow>Processed Value:</color> {value}");
            // Here you can add logic to update your game based on the received data
        }
    }

    // Crucial: Close the port when the game stops or Unity will lock the COM port
    void OnApplicationQuit()
    {
        _isRunning = false;
        if (_readThread != null && _readThread.IsAlive)
        {
            _readThread.Join(); // Wait for the thread to finish
        }
        if (_serialPort != null && _serialPort.IsOpen)
        {
            _serialPort.Close();
            Debug.Log("Serial Port Closed Safely.");
        }
    }
}
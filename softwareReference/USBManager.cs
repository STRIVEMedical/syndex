using UnityEngine;
using System.IO.Ports;
using System;
using System.Threading;
using System.Threading.Channels;
using System.Threading.Tasks;
using System.Collections.Generic;
using UnityEditor;
using System.IO;

namespace USB
{
    /// <summary>
    /// USBManager manages the serial connection to the Teensy (or other USB-serial device).
    /// Responsibilities:
    ///  - Establish and close SerialPort connections.
    ///  - Spawn background tasks to read incoming bytes and write outgoing packets.
    ///  - Parse the incoming byte stream into framed packets and push them to PacketBus.ReceiveWriter.
    ///  - Read outgoing packets from PacketBus.SendReader and write them to the serial port.
    ///
    /// Events:
    ///  - OnUSBConnected: raised when a serial port is opened successfully.
    ///  - OnUSBDisconnected: raised when the device disconnects or the port is closed.
    /// </summary>
    public class USBManager : MonoBehaviour
    {
        public static event Action OnUSBConnected;
        public static event Action OnUSBDisconnected;

        // Public flag that other code can check to see whether hardware is currently connected.
        public static bool IsConnected { get; private set; } = false;

        // If an IOException occurs on a background thread, we set this flag and call Disconnect() on the main thread in Update.
        private volatile bool _hardwareDisconnected = false;

        public string portName = "";
        private SerialPort _serialPort;
        private CancellationTokenSource _cts;

        // Constants used by packet framing code. SYNC_BYTES matches the two sync bytes at packet start (0x7F, 0xFE).
        private const uint SYNC_BYTES = 0x7FFE;
        private const int HEADER_SIZE = 4; 

        void Start()
        {
            Debug.Log("USB Manager ready. Attempting automatic connection to Teensy device...");
            // Automatically detect and connect to Teensy on startup
            ConnectToTeensy();
        }

        private void Update()
        {
            // If a background thread detected hardware disconnection via IOException, it sets _hardwareDisconnected.
            // We respond on the main thread by calling Disconnect to ensure proper Unity lifecycle behavior.
            if (_hardwareDisconnected)
            {
                _hardwareDisconnected = false; 
                Disconnect();
            }
        }

        /// <summary>
        /// Open the serial port and start reader/writer background tasks.
        /// Tasks are started with Task.Run and run on thread pool threads so they don't block the Unity main thread.
        /// </summary>
        public void ConnectToTeensy()
        {
            if (IsConnected || (_serialPort != null && _serialPort.IsOpen))
            {
                Debug.Log("Already connected / attempted connection!");
                return;
            }

            USBDetector detector = new USBDetector();
            detector.ReadCOMs();
            portName = detector.portName;

            if (string.IsNullOrEmpty(portName))
            {
                Debug.LogError("No COM port detected. Please connect the device and try again.");
                return;
            }

            _serialPort = new SerialPort(portName, 115200);
            Debug.Log($"Attempting to connect to device on {portName}...");

            try
            {
                // Short read timeout allows the reader loop to keep running and check cancellation frequently.
                _serialPort.ReadTimeout = 10;
                _serialPort.Open();
                Debug.Log($"Connected to device on {portName}");

                _cts = new CancellationTokenSource();

                IsConnected = true;
                OnUSBConnected?.Invoke(); // Notify subscribers (PacketProcessor, etc.) that the device is connected.

                // Start background loops for reading and writing serial data. They will run until cancelled.
                Task.Run(() => ReadSerialData(_cts.Token));
                Task.Run(() => WriteSerialData(_cts.Token));

                Debug.Log("Serial Reader started.");
            }
            catch (Exception e)
            {
                Debug.LogError($"Could not open serial port: {e.Message}");
            }
        }

        /// <summary>
        /// Gracefully disconnect: cancel tasks, close and dispose serial port, and raise disconnect event.
        /// </summary>
        public void Disconnect()
        {
            if (!IsConnected)
            {
                Debug.Log("USB already Disconnected");
                return;
            }
            
            _cts.Cancel();

            if (_serialPort != null)
            {
                try { _serialPort.Close(); }
                catch { Debug.Log("Port already closed"); }
                _serialPort.Dispose();
                _serialPort = null;
            }

            IsConnected = false;
            OnUSBDisconnected?.Invoke();
        }

        /// <summary>
        /// Background read loop that reads raw bytes from the serial port and frames them into packets.
        /// It accumulates bytes into streamBuffer until there are enough bytes for a full packet, then extracts it.
        ///
        /// Packet framing rules (as used here):
        ///  - Two sync bytes at the start: 0x7F, 0xFE
        ///  - Header size is 4 bytes; the payload length is located in byte index 3 of the header.
        ///  - Total packet size = HEADER_SIZE + payloadLength + 2 (CRC size).
        ///
        /// After extracting a complete packet, it writes the framed packet (minus the two leading sync bytes)
        /// into PacketBus.ReceiveWriter for the PacketProcessor to parse and handle.
        /// </summary>
        private async Task ReadSerialData(CancellationToken token)
        {
            byte[] readBuffer = new byte[1024];
            List<byte> streamBuffer = new List<byte>();
            int droppedPacketCount = 0;
            DateTime lastDropLogUtc = DateTime.UtcNow;
            while (!token.IsCancellationRequested)
            {
                try
                {
                    if (_serialPort == null || !_serialPort.IsOpen)
                    {
                        break;
                    }
                    int count = _serialPort.Read(readBuffer, 0, readBuffer.Length);
                    if (count > 0)
                    {
                        // Append read bytes to the stream buffer.
                        for (int i = 0; i < count; i++)
                        {
                            streamBuffer.Add(readBuffer[i]);
                        }

                        // Attempt to parse one or more packets from the buffer.
                        while (streamBuffer.Count >= HEADER_SIZE)
                        {
                            // Check for sync bytes at the start of the buffer. If they don't match, drop leading byte and retry.
                            if (streamBuffer[0] == 0x7F && streamBuffer[1] == 0xFE)
                            {
                                // payloadLength is stored at header byte index 3 (0-based).
                                byte payloadLength = streamBuffer[3];
                                int totalPacketSize = HEADER_SIZE + payloadLength + 2; // +2 for CRC

                                if (streamBuffer.Count >= totalPacketSize)
                                {
                                    // Extract the packet data. The existing code slices starting at index 2,
                                    // meaning the two sync bytes are removed before sending to PacketBus.
                                    byte[] packetData = streamBuffer.GetRange(2, totalPacketSize - 2).ToArray();

                                    bool success = PacketBus.ReceiveWriter.TryWrite(packetData);

                                    if (!success)
                                    {
                                        droppedPacketCount++;
                                        DateTime nowUtc = DateTime.UtcNow;
                                        if ((nowUtc - lastDropLogUtc).TotalSeconds >= 1.0)
                                        {
                                            Debug.LogWarning($"Receive packet bus full: dropped {droppedPacketCount} packets in the last second.");
                                            droppedPacketCount = 0;
                                            lastDropLogUtc = nowUtc;
                                        }
                                    }

                                    // Remove the bytes that comprise this packet from the buffer.
                                    streamBuffer.RemoveRange(0, totalPacketSize);
                                }
                                else
                                {
                                    // Not enough bytes yet for a full packet; wait for more data.
                                    break;
                                }
                            }
                            else
                            {
                                // Sync mismatch: drop first byte and continue scanning.
                                streamBuffer.RemoveAt(0); 
                            }
                        }
                    }
                }
                catch (TimeoutException)
                {
                    // Read timeout is expected as we set a short timeout; ignore and continue the loop.
                }
                catch (IOException)
                {
                    // Serial port disconnected unexpectedly. Set a flag so the main thread can handle cleanup.
                    //Debug.LogWarning("Hardware USB Disconnected during read!");
                    _hardwareDisconnected = true; 
                    break;
                }
                catch (Exception e)
                {
                    Debug.LogError($"Error reading serial data: {e.Message}");
                }

                // Small delay to avoid a tight spin loop; yields to other tasks.
                await Task.Delay(1);
            }
        }

        /// <summary>
        /// Background writer loop that awaits packets from PacketBus.SendReader and writes them to the serial port.
        /// Using "await foreach" will asynchronously wait for items and support cancellation via the token.
        /// </summary>
        private async Task WriteSerialData(CancellationToken token)
        {
            try
            {
                await foreach (var packet in PacketBus.SendReader.ReadAllAsync(token))
                {
                    if (_serialPort != null && _serialPort.IsOpen)
                    {
                        _serialPort.Write(packet, 0, packet.Length);
                    }
                }
            }
            catch (IOException)
            {
                // If a write fails because the device disconnected, set the flag to trigger Disconnect on main thread.
                //Debug.LogWarning("Hardware USB Disconnected during write!");
                _hardwareDisconnected = true;
            }
            catch (OperationCanceledException)
            {
                // Normal shutdown path when cancellation is requested.
                Debug.Log("Writer thread shutting down...");
            }
            catch (Exception e)
            {
                Debug.LogError($"Writer thread error: {e.Message}");
            }
        }

        void OnApplicationQuit()
        {
            Disconnect();
        }
        
        /// <summary>
        /// Simple CRC-16 calculation (polynomial 0xA001) used for packet integrity checks.
        /// </summary>
        public static ushort CalculateCRC(byte[] data)
        {
            ushort crc = 0x0000;
            foreach (byte b in data)
            {
                crc ^= b;
                for (int i = 0; i < 8; i++)
                {
                    if ((crc & 1) != 0)
                        crc = (ushort)((crc >> 1) ^ 0xA001);
                    else
                        crc >>= 1;
                }
            }
            return crc;
        }
    }
}

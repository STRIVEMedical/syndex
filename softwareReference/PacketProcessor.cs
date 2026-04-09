using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Runtime.Remoting.Messaging;
using System.Text;
using System.Threading;
using System.Threading.Channels;
using System.Threading.Tasks;
using UnityEngine;

/// <summary>
/// PacketBus is a small, process-wide message bus implemented with System.Threading.Channels.
/// It provides two bounded channels:
///  - Receive channel (Teensy -> Unity): USBManager writes incoming raw packets here.
///  - Send channel (Unity -> Teensy): PacketProcessor (or any producer) writes outgoing packets here.
///
/// Channels are used because they provide a thread-safe, efficient producer/consumer queue that
/// supports both synchronous and asynchronous producers/consumers. The channels are bounded to
/// 100 items to avoid unbounded memory growth if the writer produces data faster than readers consume it.
/// </summary>
public static class PacketBus
{
    // Teensy -> Unity (incoming)
    private static readonly Channel<byte[]> _receiveChannel = Channel.CreateBounded<byte[]>(1024);
    public static ChannelWriter<byte[]> ReceiveWriter => _receiveChannel.Writer; // Used by USBManager to push received packets into Unity.
    public static ChannelReader<byte[]> ReceiveReader => _receiveChannel.Reader; // Used by PacketProcessor to read packets as they arrive.

    // Unity -> Teensy (outgoing)
    private static readonly Channel<byte[]> _sendChannel = Channel.CreateBounded<byte[]>(100);
    public static ChannelWriter<byte[]> SendWriter => _sendChannel.Writer; // Used by PacketProcessor and other senders to queue packets for USBManager to transmit.
    public static ChannelReader<byte[]> SendReader => _sendChannel.Reader; // Used by USBManager to read outgoing packets and write them to the serial port.
}

namespace PacketProcessor
{
    /// <summary>
    /// PacketProcessor owns the logic for consuming packets from PacketBus.ReceiveReader,
    /// parsing them into higher-level Packet objects, and dispatching them to registered handlers.
    ///
    /// It also provides helper methods to create and send command packets via PacketBus.SendWriter.
    /// </summary>
    public class PacketProcessor : MonoBehaviour
    {
        // Singleton instance so other parts of the code (UI, InputHandler) can easily access processor methods.
        public static PacketProcessor Instance { get; private set; }

        public float LatestJ0Angle { get; private set; }

        // CancellationTokenSource used to cancel the background packet processing loop(s) when USB disconnects
        // or when the object is destroyed. Canceling this token signals the Task/loops to stop cleanly.
        private CancellationTokenSource _cts;

        // Dictionary mapping packet types to handler delegates.
        // The key is a PacketType enum value and the value is an Action<Packet> to call when a packet of that type arrives.
        // This is a simple dispatch table: O(1) lookup for handler based on packet type.
        private Dictionary<PacketType, Action<Packet>> _packetHandlers;
        private int _jointTelemetryLogDivider = 0;
        private int _statusTelemetryLogDivider = 0;
        // Lower values print more often; keep >1 to avoid console bottlenecks.
        private const int JointTelemetryLogEveryN = 20;
        private const int StatusTelemetryLogEveryN = 20;
        private static readonly string[] JointLabels =
        {
            "EXT_CH0",
            "EXT_CH1",
            "EXT_CH2",
            "EXT_CH3",
            "ROTATE",
            "REACH",
            "LIFT"
        };

        private void Awake()
        {
            // Ensure only one PacketProcessor exists (singleton pattern). If duplicates exist (e.g. scene reload),
            // destroy the extra GameObject to avoid multiple loops and duplicate handlers.
            if (Instance != null && Instance != this)
            {
                Destroy(this.gameObject);
            }
            else
            {
                Instance = this;
            }

            // Initialize the dispatch dictionary with the default handlers.
            // You can subscribe additional handlers later with Subscribe(...)
            _packetHandlers = new Dictionary<PacketType, Action<Packet>>
            {
                { PacketType.RESP_PONG, HandlePongResponse },
                { PacketType.RESP_ACK, HandleAckResponse },
                { PacketType.RESP_NACK, HandleNackResponse },
                { PacketType.TELEM_JOINT_DATA, HandleJointData },
                { PacketType.TELEM_STATUS, HandleStatus },
                { PacketType.LOG_MESSAGE, HandleLogMessage },
                { PacketType.ERROR_MESSAGE, HandleErrorMessage }
            };
        }

        /// <summary>
        /// Register an additional handler for a packet type.
        /// If a handler already exists for the type, the provided handler is added (multicast delegate),
        /// so multiple subscribers can receive the same packet type.
        ///
        /// Example: _packetHandlers[type] += handler adds another invocation to the multicast delegate.
        /// </summary>
        public void Subscribe(PacketType type, Action<Packet> handler)
        {
            if (_packetHandlers.ContainsKey(type))
            {
                _packetHandlers[type] += handler;
            }
            else
            {
                _packetHandlers[type] = handler;
            }
        }

        private void Start()
        {
            // Subscribe to USB connection events. These events are static on USBManager so they can be raised
            // by the USB code and observed by this instance.
            //
            // Events are multicast delegates in C#: when OnUSBConnected is invoked, all subscribers will be called.
            USB.USBManager.OnUSBConnected += HandleUSBConnected;
            USB.USBManager.OnUSBDisconnected += HandleUSBDisconnected;
        }

        private void OnDestroy()
        {
            // Unsubscribe to avoid memory leaks or invoking methods on a destroyed object.
            USB.USBManager.OnUSBConnected -= HandleUSBConnected;
            USB.USBManager.OnUSBDisconnected -= HandleUSBDisconnected;
        }

        /// <summary>
        /// Called when USBManager signals a successful connection.
        /// This cancels any previous processing loops, creates a fresh CancellationTokenSource,
        /// sends an initial ping and starts the packet processing loop in the background.
        ///
        /// Note: "_ = RunPacketLoopAsync(_cts.Token);" starts the async method but intentionally
        /// discards the returned Task so Start does not await completion. This launches a background task.
        /// </summary>
        private volatile bool _startupPongReceived = false;
        private async void HandleUSBConnected()
        {
            Debug.Log("PacketProcessor detected USB connection. Starting loops.");

            // If there was an existing CTS, cancel and dispose it to stop previous loops before creating a new one.
            _cts?.Cancel();
            _cts?.Dispose();
            _cts = new CancellationTokenSource();
            
            _ = RunPacketLoopAsync(_cts.Token);

            // 2. RESET THE FLAG & SEND PING
            _startupPongReceived = false;

            // Optionally send a ping immediately to validate comms.
            if (!SendPing())
            {
                Debug.LogWarning("Connection Warning: Initial ping enqueue failed.");
            }

            // 3. WAIT FOR THE PONG (Non-blocking)
            int timeoutMs = 2000; // Wait up to 2 seconds
            int waitedMs = 0;
            int checkInterval = 50; // Check every 50 milliseconds
            
            while (!_startupPongReceived && waitedMs < timeoutMs)
            {
                await Task.Delay(checkInterval); // Pause here, let Unity keep running, then check again
                waitedMs += checkInterval;
            }

            // 4. EVALUATE THE RESULT
            if (_startupPongReceived)
            {
                Debug.Log("<color=green>Connection Validated: Initial Pong Received!</color>");
                // (Optional) You can trigger other startup logic here now that you know it's safe
            }
            else
            {
                Debug.LogWarning("<color=red>Connection Warning: Timed out waiting for initial Pong.</color>");
                // (Optional) You could force a disconnect here if you require a strict handshake
                // USB.USBManager.Disconnect(); 
            }
            // Start the background loop that reads packets. The returned Task is intentionally not awaited
            // so the loop runs concurrently on the thread pool / synchronization context as appropriate.

        }

        /// <summary>
        /// Called when USBManager signals a disconnection.
        /// Cancel the cancellation token to stop background tasks.
        /// </summary>
        private void HandleUSBDisconnected()
        {
            Debug.Log("PacketProcessor detected USB disconnection. Stopping loops.");
            _cts?.Cancel();
        }

        private void OnApplicationQuit()
        {
            // Ensure loops stop when the application exits.
            _cts?.Cancel();
        }

        /// <summary>
        /// Wrapper to run the packet loop on a thread pool task. Running the loop with Task.Run
        /// ensures it doesn't block the Unity main thread. Exceptions are caught and logged.
        ///
        /// Task.Run schedules the provided delegate to run on the CLR thread pool.
        /// The method awaits the Task so exceptions thrown inside will be observed here.
        /// </summary>
        private async Task RunPacketLoopAsync(CancellationToken token)
        {
            try
            {
                // Run the synchronous-like packet loop on a thread pool thread.
                await Task.Run(() => PacketProcessorLoop(token));
            }
            catch (OperationCanceledException)
            {
                Debug.Log("PacketProcessor loop was cancelled.");
            }
            catch (Exception ex)
            {
                Debug.LogError($"Unexpected error in PacketProcessor loop: {ex}");
            }
        }

        /// <summary>
        /// Main packet processing loop.
        /// Uses C# 8 async enumerable support: ReadAllAsync(token) yields each packet as it becomes available.
        /// The loop will await when there are no packets (non-blocking) and resume when new items arrive.
        ///
        /// For each raw packet, construct a Packet object, validate it, and dispatch to the handler mapped in _packetHandlers.
        /// If a handler exists, it is invoked on the current thread (thread pool thread).
        /// </summary>
        private async Task PacketProcessorLoop(CancellationToken token)
        {
            while (await PacketBus.ReceiveReader.WaitToReadAsync(token))
            {
                while (PacketBus.ReceiveReader.TryRead(out var recPacket))
                {
                    // Parse raw bytes into a Packet instance (parsing, CRC checks, header extraction, etc).
                    Packet parser = new Packet(recPacket);
                    if (parser.IsValid == false)
                    {
                        // Invalid packet (bad CRC, malformed), ignore and continue.
                        continue;
                    }

                    // Attempt to locate a handler in the dispatch dictionary.
                    if (_packetHandlers.TryGetValue(parser.Header.Type, out Action<Packet> handler))
                    {
                        // Invoke the handler. Because we allowed multicast delegates in Subscribe,
                        // this may call multiple registered subscribers.
                        handler?.Invoke(parser);
                    }
                }
            }
        }

        // The following methods are simple packet handlers that currently log receipt.
        // They can be expanded to update game state, telemetry, UI, etc.

        private void HandlePongResponse(Packet packet)
        {
            _startupPongReceived = true; // Tell the startup sequence we got it!
            Debug.Log("Received Pong Response");
            // Additional processing can be done here if needed
        }

        private void HandleAckResponse(Packet packet)
        {
            Debug.Log("Received ACK Response");
            // Additional processing can be done here if needed
        }

        private void HandleNackResponse(Packet packet)
        {
            Debug.LogWarning("Received NACK Response");
            // Additional processing can be done here if needed
        }

        private void HandleJointData(Packet packet)
        {
            if (packet.JointData.HasValue)
            {
                var payload = packet.JointData.Value;

                float j0 = Packet.DecodeUnsigned16Telemetry(payload.J0_AngleRaw);
                float j1 = Packet.DecodeUnsigned16Telemetry(payload.J1_AngleRaw);
                float j2 = Packet.DecodeUnsigned16Telemetry(payload.J2_AngleRaw);
                float j3 = Packet.DecodeUnsigned16Telemetry(payload.J3_AngleRaw);
                float j4 = Packet.DecodeUnsigned16Telemetry(payload.J4_AngleRaw);
                float j5 = Packet.DecodeUnsigned16Telemetry(payload.J5_AngleRaw);
                float j6 = Packet.DecodeUnsigned16Telemetry(payload.J6_AngleRaw);

                LatestJ0Angle = j0;

                // Throttle telemetry logs to avoid starving packet processing.
                if ((++_jointTelemetryLogDivider % JointTelemetryLogEveryN) == 0)
                {
                    Debug.Log(
                        $"Joint telemetry sample | " +
                        $"J0:{JointLabels[0]}={j0:F2}, " +
                        $"J1:{JointLabels[1]}={j1:F2}, " +
                        $"J2:{JointLabels[2]}={j2:F2}, " +
                        $"J3:{JointLabels[3]}={j3:F2}, " +
                        $"J4:{JointLabels[4]}={j4:F2}, " +
                        $"J5:{JointLabels[5]}={j5:F2}, " +
                        $"J6:{JointLabels[6]}={j6:F2}");
                }
            }
            // Additional processing can be done here if needed
        }

        private void HandleStatus(Packet packet)
        {
            if ((++_statusTelemetryLogDivider % StatusTelemetryLogEveryN) == 0)
            {
                Debug.Log("Received Status Telemetry");
            }
            // Additional processing can be done here if needed
        }

        private void HandleLogMessage(Packet packet)
        {
            // Convert payload bytes to ASCII string and log.
            string logMsg = Encoding.ASCII.GetString(packet.PayloadBytes);
            Debug.Log($"Received Log Message: {logMsg}");
        }

        private void HandleErrorMessage(Packet packet)
        {
            // Convert payload bytes to ASCII string and log as error.
            string errorMsg = Encoding.ASCII.GetString(packet.PayloadBytes);
            Debug.LogError($"Received Error Message: {errorMsg}");
        }

        // The following methods construct command packets and write them to PacketBus.SendWriter.
        // These are non-blocking uses of TryWrite; if the channel is full, TryWrite returns false and the packet is dropped.
        // If stronger delivery guarantees are required, consider using WriteAsync which can await until space is available.

        public bool SendPing()
        {
            if (!USB.USBManager.IsConnected)
            {
                Debug.LogWarning("Ignored Ping command: USB is disconnected.");
                return false; // Exit early when hardware is not available.
            }
            PacketType Type = PacketType.CMD_PING;
            byte[] Payload = new byte[0];
            Packet packet = new Packet(Type, Payload);

            // TryWrite places the packet into the bounded channel immediately if there is capacity.
            bool queued = PacketBus.SendWriter.TryWrite(packet.ToBytes());
            if (!queued)
            {
                Debug.LogWarning("Failed to enqueue Ping packet: send channel is full.");
                return false;
            }
            Debug.Log("<color=blue>Sent Ping Packet</color>");
            return true;
        }
        
        public void SendReset()
        {
            if (!USB.USBManager.IsConnected)
            {
                Debug.LogWarning("Ignored Reset command: USB is disconnected.");
                return;
            }
            PacketType Type = PacketType.CMD_RESET_DEVICE;
            byte[] Payload = new byte[0];
            Packet packet = new Packet(Type, Payload);
            PacketBus.SendWriter.TryWrite(packet.ToBytes());
            Debug.Log("<color=blue>Sent Reset Packet</color>");
        }

        public void SendRequestTelem()
        {
            if (!USB.USBManager.IsConnected)
            {
                Debug.LogWarning("Ignored Request Telemetry command: USB is disconnected.");
                return;
            }
            PacketType Type = PacketType.CMD_REQUEST_TELEM;
            byte[] Payload = new byte[0];
            Packet packet = new Packet(Type, Payload);
            PacketBus.SendWriter.TryWrite(packet.ToBytes());
            Debug.Log("<color=blue>Sent Request Telemetry Packet</color>");
        }

        public void SendStartHoming()
        {
            if (!USB.USBManager.IsConnected)
            {
                Debug.LogWarning("Ignored Start Homing command: USB is disconnected.");
                return;
            }
            PacketType Type = PacketType.CMD_START_HOMING;
            byte[] Payload = new byte[0];
            Packet packet = new Packet(Type, Payload);
            PacketBus.SendWriter.TryWrite(packet.ToBytes());
            Debug.Log("<color=blue>Sent Start Homing Packet<color>");
        }

        public void SendEStop()
        {
            if (!USB.USBManager.IsConnected)
            {
                Debug.LogWarning("Ignored Emergency Stop command: USB is disconnected.");
                return;
            }
            PacketType Type = PacketType.CMD_ESTOP;
            byte[] Payload = new byte[0];
            Packet packet = new Packet(Type, Payload);
            PacketBus.SendWriter.TryWrite(packet.ToBytes());
            Debug.Log("<color=blue>Sent E-Stop Packet</color>");
        }

        public void SendSetODState(Payload_SetODriveState States)
        {
            if (!USB.USBManager.IsConnected)
            {
                Debug.LogWarning("Ignored Set ODrive State command: USB is disconnected.");
                return;
            }
            PacketType Type = PacketType.CMD_SET_ODRIVE_STATE;
            byte[] Payload = Packet.StructToBytes(States); // Helper that marshals struct into byte[].
            Packet packet = new Packet(Type, Payload);
            PacketBus.SendWriter.TryWrite(packet.ToBytes());
            Debug.Log("<color=blue>Sent Set ODrive State Packet</color>");
        }

        public void SendSetJointTargets(Payload_SetJointTargets Targets)
        {
            if (!USB.USBManager.IsConnected)
            {
                Debug.LogWarning("Ignored Set Joint Targets command: USB is disconnected.");
                return;
            }
            PacketType Type = PacketType.CMD_SET_JOINT_TARGETS;
            byte[] Payload = Packet.StructToBytes(Targets);
            Packet packet = new Packet(Type, Payload);
            PacketBus.SendWriter.TryWrite(packet.ToBytes());
            Debug.Log("<color=blue>Sent Set Joint Targets Packet</color>");
        }

        public void SendSetJointParams(Payload_SetJointParameter Params)
        {
            if (!USB.USBManager.IsConnected)
            {
                Debug.LogWarning("Ignored Set Joint Parameter command: USB is disconnected.");
                return;
            }
            PacketType Type = PacketType.CMD_SET_JOINT_PARAMETER;
            byte[] Payload = Packet.StructToBytes(Params);
            Packet packet = new Packet(Type, Payload);
            PacketBus.SendWriter.TryWrite(packet.ToBytes());
            Debug.Log("<color=blue>Sent Set Joint Params Packet<color>");
        }
    }
}

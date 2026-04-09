using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.IO.Ports;
using UnityEngine;

// Data types
public enum PacketType : byte
{
    // Device -> Host (Telemetry)
    RESP_PONG = 0x81,
    RESP_ACK = 0x82,
    RESP_NACK = 0x83,
    TELEM_JOINT_DATA = 0x84,
    TELEM_STATUS = 0x85,
    LOG_MESSAGE = 0x86,
    ERROR_MESSAGE = 0xF0,

    // Host -> Device (Commands)
    CMD_PING = 0x01,
    CMD_RESET_DEVICE = 0x02,
    CMD_SET_ODRIVE_STATE = 0x03,
    CMD_SET_JOINT_TARGETS = 0x04,
    CMD_REQUEST_TELEM = 0x05,
    CMD_START_HOMING = 0x06,
    CMD_SET_JOINT_PARAMETER = 0x07,
    CMD_ESTOP = 0x08
}

// TELEM_JOINT_DATA (Type: 0x84, Length: 28)
[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct Payload_TelemJointData
{
    public ushort J0_AngleRaw; public ushort J0_VelRaw;
    public ushort J1_AngleRaw; public ushort J1_VelRaw;
    public ushort J2_AngleRaw; public ushort J2_VelRaw;
    public ushort J3_AngleRaw; public ushort J3_VelRaw;
    public ushort J4_AngleRaw; public ushort J4_VelRaw;
    public ushort J5_AngleRaw; public ushort J5_VelRaw;
    public ushort J6_AngleRaw; public ushort J6_VelRaw;
}

// TELEM_STATUS (Type: 0x85, Length: 10)
[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct Payload_TelemStatus
{
    public byte ArmStatus;
    public byte ODriveFaults;

    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
    public byte[] Reserved;
}

// CMD_SET_ODRIVE_STATE (Type: 0x03, Length: 2)
[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct Payload_SetODriveState
{
    public byte JointMask;
    public byte AxisState;
}

// CMD_SET_JOINT_TARGETS (Type: 0x04, Length: 28)
[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct Payload_SetJointTargets
{
    public float J0_Vel; public float J0_Torque;
    public float J1_Vel; public float J1_Torque;
    public float J2_Vel; public float J2_Torque;
    public float J3_Vel; public float J3_Torque;
    public float J4_Vel; public float J4_Torque;
    public float J5_Vel; public float J5_Torque;
    public float J6_Vel; public float J6_Torque;
}

// CMD_SET_JOINT_PARAMETER (Type: 0x07, Length: 6)
[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct Payload_SetJointParameter
{
    public byte JointMask;
    public byte ParameterID;
    public float Value;
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct PacketHeader
{
    public PacketType Type;
    public byte Length;
}

// Represents a single Packet (either incoming or outgoing)
public class Packet
{
    public PacketHeader Header;
    public byte[] PayloadBytes;
    public bool IsValid { get; private set; } // Tracks if parsing/CRC was successful
    public Payload_TelemJointData? JointData { get; private set; }
    public Payload_TelemStatus? StatusData { get; private set; }
    public string TextData { get; private set; } // Strings are already nullable

    // ------------------------------------------------------------------------
    // CONSTRUCTOR 1: Outgoing Packet 
    // Usage: var p = new PacketParser(); p.Header.Type = 0x03; p.SetPayload(myStruct);
    // ------------------------------------------------------------------------
    public Packet(PacketType Type, byte[] Payload)
    {
        Header.Type = Type;
        switch(Type)
        {
            case PacketType.CMD_PING:
            case PacketType.CMD_RESET_DEVICE:
            case PacketType.CMD_REQUEST_TELEM:
            case PacketType.CMD_START_HOMING:
            case PacketType.CMD_ESTOP:
                PayloadBytes = null;
                Header.Length = 0;
                break;
            case PacketType.CMD_SET_ODRIVE_STATE:
                Header.Length = 0x02;
                PayloadBytes = new byte[2];
                PayloadBytes = Payload;
                break;
            case PacketType.CMD_SET_JOINT_TARGETS:
                Header.Length = 0x1C;
                PayloadBytes = new byte[28];
                PayloadBytes = Payload;
                break;
            case PacketType.CMD_SET_JOINT_PARAMETER:
                Header.Length = 0x06;
                PayloadBytes = new byte[6];
                PayloadBytes = Payload;
                break;
            default:
                Debug.LogError("Invalid Packet Type for this constructor!");
                break;
        }

        IsValid = true;
    }

    // ------------------------------------------------------------------------
    // CONSTRUCTOR 2: Incoming Packet from USB
    // Usage: var incomingPacket = new PacketParser(serialDataArray);
    // ------------------------------------------------------------------------
    public Packet(byte[] packet)
    {
        // Minimum incoming shape is [type, len, crc_lo, crc_hi].
        if (packet == null)
        {
            Debug.LogError($"Packet error: Packet = null." );
            IsValid = false;
            return;
        }

        if (packet.Length < 4)
        {
            Debug.LogError("Packet error: Packet too short.");
            IsValid = false;
            return;
        }

        // 3. Extract Header
        Header.Type = (PacketType)packet[0];
        Header.Length = packet[1];

        if (packet.Length != Header.Length + 4)
        {
            Debug.LogError("Packet error: Length mismatch.");
            IsValid = false;
            return;
        }

        // 4. Extract Payload
        PayloadBytes = new byte[Header.Length];
        if (Header.Length > 0)
        {
            // Incoming packet is [type, len, payload..., crc_lo, crc_hi] (sync already stripped).
            Buffer.BlockCopy(packet, 2, PayloadBytes, 0, Header.Length);
        }
        // 5. Verify CRC
        IsValid = VerifyCRC(packet);

        // 6. Automatically process payload if valid
        if (IsValid)
        {
            ProcessPayload();
        }
    }

    // ------------------------------------------------------------------------
    // PAYLOAD HELPERS
    // ------------------------------------------------------------------------

    // Easily load a struct into the payload for sending
    public void SetPayload<T>(T payloadStruct) where T : struct
    {
        PayloadBytes = StructToBytes(payloadStruct);
        Header.Length = (byte)PayloadBytes.Length;
    }

    // Easily extract a struct from the payload for reading
    public T GetPayload<T>() where T : struct
    {
        if (PayloadBytes == null || PayloadBytes.Length < Marshal.SizeOf(typeof(T)))
        {
            throw new InvalidOperationException("Payload size mismatch for requested struct.");
        }
        return BytesToStruct<T>(PayloadBytes);
    }

    // ------------------------------------------------------------------------
    // EXPORT TO BYTES (For sending out via Serial)
    // ------------------------------------------------------------------------
    public byte[] ToBytes()
    {
        byte len = (byte)(PayloadBytes?.Length ?? 0);
        Header.Length = len; // Ensure length is up to date

        byte[] packet = new byte[4 + len + 2];

        // Apply Preamble & Header
        packet[0] = 0x7F;
        packet[1] = 0xFE;
        packet[2] = (byte)Header.Type;
        packet[3] = Header.Length;

        // Apply Payload
        if (PayloadBytes != null && len > 0)
        {
            Buffer.BlockCopy(PayloadBytes, 0, packet, 4, len);
        }

        // Calculate CRC
        ushort crc = 0;
        crc = UpdateCRC(crc, packet[2]); // type
        crc = UpdateCRC(crc, packet[3]); // len
        for (int i = 0; i < len; i++)
        {
            crc = UpdateCRC(crc, packet[4 + i]);
        }

        // Append CRC (Little Endian)
        packet[packet.Length - 2] = (byte)(crc & 0xFF);
        packet[packet.Length - 1] = (byte)(crc >> 8);

        return packet;
    }

    // ------------------------------------------------------------------------
    // PACKET PROCESSING (Automatically called inside constructor)
    // ------------------------------------------------------------------------
    private void ProcessPayload()
    {
        PacketType type = (PacketType)Header.Type;

        switch (type)
        {
            case PacketType.RESP_PONG:
            case PacketType.RESP_ACK:
            case PacketType.RESP_NACK:
                break;

            case PacketType.TELEM_JOINT_DATA:
                if (PayloadBytes.Length == Marshal.SizeOf(typeof(Payload_TelemJointData)))
                {
                    JointData = GetPayload<Payload_TelemJointData>();
                }
                break;

            case PacketType.TELEM_STATUS:
                if (PayloadBytes.Length == Marshal.SizeOf(typeof(Payload_TelemStatus)))
                {
                    StatusData = GetPayload<Payload_TelemStatus>();
                }
                break;

            case PacketType.LOG_MESSAGE:
            case PacketType.ERROR_MESSAGE:
                TextData = System.Text.Encoding.ASCII.GetString(PayloadBytes);
                break;

            default:
                Debug.Log($"Unknown Packet Type: 0x{Header.Type:X2}");
                break;
        }
    }

    // ------------------------------------------------------------------------
    // UTILITY / MATH FUNCTIONS
    // ------------------------------------------------------------------------

    private bool VerifyCRC(byte[] packet)
    {
        ushort crc = 0;
        if (packet != null)
        {
            foreach (byte b in packet)
            {
                crc = UpdateCRC(crc, b);
            }
        }
        return crc == 0;
    }

    private static ushort UpdateCRC(ushort crc, byte data)
    {
        crc ^= data;
        for (int i = 0; i < 8; i++)
        {
            if ((crc & 0x0001) != 0)
                crc = (ushort)((crc >> 1) ^ 0xA001);
            else
                crc = (ushort)(crc >> 1);
        }
        return crc;
    }

    // Matches Teensy float16ToUnsigned16 mapping used by firmware telemetry.
    public static float DecodeUnsigned16Telemetry(ushort bits)
    {
        return bits / 181.84f - 180.0f;
    }

    public static byte[] StructToBytes<T>(T str) where T : struct
    {
        int size = Marshal.SizeOf(str);
        byte[] arr = new byte[size];
        IntPtr ptr = Marshal.AllocHGlobal(size);
        try
        {
            Marshal.StructureToPtr(str, ptr, true);
            Marshal.Copy(ptr, arr, 0, size);
            return arr;
        }
        finally
        {
            Marshal.FreeHGlobal(ptr);
        }
    }

    public static T BytesToStruct<T>(byte[] bytes) where T : struct
    {
        GCHandle handle = GCHandle.Alloc(bytes, GCHandleType.Pinned);
        try
        {
            return Marshal.PtrToStructure<T>(handle.AddrOfPinnedObject());
        }
        finally
        {
            handle.Free();
        }
    }
}

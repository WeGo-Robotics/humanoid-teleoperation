// XrLink transport for the Quest app.
//
// Uses System.Net.WebSockets.ClientWebSocket, which ships with Unity's .NET
// Standard profile and works under IL2CPP on Android -- no third-party
// dependency, deliberately, to keep the build simple.
//
// The binary framing here must match teleop/xr/codec.py byte for byte. If you
// change one, change the other and bump PROTO_VERSION on both sides. Run
// tools/fake_quest.py against the host to see what correct looks like.

using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Net.WebSockets;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

namespace WeGo.Teleop
{
    public class XrLinkClient : IDisposable
    {
        public const byte ProtoVersion = 2;
        private const ushort Magic = 0x5852;   // 'XR'

        [Flags]
        private enum Flags : byte
        {
            None = 0,
            Worn = 1 << 0,
            LeftTracked = 1 << 1,
            RightTracked = 1 << 2,
            HandMode = 1 << 3,
        }

        public bool IsConnected => _socket != null &&
                                   _socket.State == WebSocketState.Open;

        /// <summary>Frames skipped because a send was still in flight. Surfaced
        /// on the HUD: a number that climbs means the link cannot keep up with
        /// the capture rate, which is worth seeing before it becomes latency.</summary>
        public int SkippedFrames { get; private set; }

        /// <summary>Control messages from the host, drained on the main thread.</summary>
        public readonly ConcurrentQueue<string> Inbound = new ConcurrentQueue<string>();

        private readonly string _url;
        private ClientWebSocket _socket;
        private CancellationTokenSource _cts;
        private Task _receiveLoop;
        private uint _seq;

        // Reused so the 72Hz send path allocates nothing. Only ever written
        // while _sendLock is held -- see SendTrackingAsync.
        private readonly byte[] _controllerBuffer = new byte[240];
        private readonly byte[] _handBuffer = new byte[840];
        private readonly SemaphoreSlim _sendLock = new SemaphoreSlim(1, 1);
        private readonly StringBuilder _json = new StringBuilder(160);

        public XrLinkClient(string url) { _url = url; }

        // ------------------------------------------------------------------
        // connection
        // ------------------------------------------------------------------
        public async Task RunAsync(CancellationToken external)
        {
            var backoffMs = 250;
            while (!external.IsCancellationRequested)
            {
                try
                {
                    _cts = CancellationTokenSource.CreateLinkedTokenSource(external);
                    _socket = new ClientWebSocket();
                    _socket.Options.KeepAliveInterval = TimeSpan.FromSeconds(5);
                    await _socket.ConnectAsync(new Uri(_url), _cts.Token);
                    Debug.Log($"[XrLink] connected to {_url}");
                    backoffMs = 250;

                    await SendControlAsync(
                        $"{{\"t\":\"hello\",\"proto\":{ProtoVersion}," +
                        $"\"dev\":\"quest3\",\"session\":\"{Guid.NewGuid()}\"}}");

                    _receiveLoop = ReceiveLoopAsync(_cts.Token);
                    await _receiveLoop;
                }
                catch (OperationCanceledException) { break; }
                catch (Exception e)
                {
                    Debug.LogWarning($"[XrLink] link error: {e.Message}");
                }
                finally
                {
                    Cleanup();
                }

                if (external.IsCancellationRequested) break;

                // Exponential backoff with jitter. The host treats any reconnect
                // while following as a safe stop, so reconnecting promptly is
                // safe -- it can never silently resume motion.
                var wait = backoffMs + UnityEngine.Random.Range(0, 100);
                Debug.Log($"[XrLink] reconnecting in {wait}ms");
                try { await Task.Delay(wait, external); }
                catch (OperationCanceledException) { break; }
                backoffMs = Math.Min(backoffMs * 2, 2000);
            }
        }

        private async Task ReceiveLoopAsync(CancellationToken token)
        {
            var buffer = new byte[8192];
            while (_socket.State == WebSocketState.Open && !token.IsCancellationRequested)
            {
                var result = await _socket.ReceiveAsync(
                    new ArraySegment<byte>(buffer), token);
                if (result.MessageType == WebSocketMessageType.Close) return;
                if (result.MessageType != WebSocketMessageType.Text) continue;

                if (!result.EndOfMessage)
                {
                    // Host control messages are a few hundred bytes; anything
                    // that fragments an 8KB buffer is not a message we know how
                    // to read. Drain it and drop it rather than enqueue a
                    // truncated fragment that JsonUtility would parse into a
                    // plausible-looking object with missing fields.
                    Debug.LogWarning("[XrLink] oversized host message, dropping");
                    while (!result.EndOfMessage && !token.IsCancellationRequested)
                        result = await _socket.ReceiveAsync(
                            new ArraySegment<byte>(buffer), token);
                    continue;
                }
                Inbound.Enqueue(Encoding.UTF8.GetString(buffer, 0, result.Count));
            }
        }

        private void Cleanup()
        {
            try { _socket?.Dispose(); } catch { }
            _socket = null;
            try { _cts?.Dispose(); } catch { }
            _cts = null;
        }

        public void Dispose() { Cleanup(); }

        // ------------------------------------------------------------------
        // sending
        // ------------------------------------------------------------------
        public async Task SendControlAsync(string json)
        {
            if (!IsConnected) return;
            // Control messages queue rather than skip: presence, buttons and
            // estop are events, and a dropped event is not recoverable the way
            // a dropped tracking frame is.
            await _sendLock.WaitAsync();
            try
            {
                await _socket.SendAsync(
                    new ArraySegment<byte>(Encoding.UTF8.GetBytes(json)),
                    WebSocketMessageType.Text, true, _cts.Token);
            }
            catch (Exception e) { Debug.LogWarning($"[XrLink] control send: {e.Message}"); }
            finally { _sendLock.Release(); }
        }

        /// <summary>Fire-and-forget presence. Used from HMDUnmounted, where the
        /// OS may suspend us at any moment -- see docs section 10.3. Never
        /// awaited, so it cannot be blocked behind a slow tracking send.</summary>
        public void SendPresence(bool worn)
        {
            var json = $"{{\"t\":\"presence\",\"worn\":{(worn ? "true" : "false")}," +
                       $"\"focus\":true,\"ts\":{DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0}}}";
            _ = SendControlAsync(json);
        }

        public void SendEstop() { _ = SendControlAsync("{\"t\":\"estop\"}"); }

        /// <summary>The full set of buttons currently held. Level-triggered:
        /// the host replaces its whole set with this one, so a dropped message
        /// self-corrects on the next send rather than sticking a button down.
        /// Names must come from BUTTON_NAMES in codec.py.</summary>
        public void SendButtons(List<string> pressed)
        {
            _json.Clear();
            _json.Append("{\"t\":\"buttons\",\"pressed\":[");
            for (var i = 0; i < pressed.Count; i++)
            {
                if (i > 0) _json.Append(',');
                _json.Append('"').Append(pressed[i]).Append('"');
            }
            _json.Append("]}");
            _ = SendControlAsync(_json.ToString());
        }

        /// <summary>Skips the frame if a send is already in flight rather than
        /// queueing behind it. At 72Hz a queued frame is stale by the time it
        /// leaves, and a backlog turns a slow link into growing latency --
        /// which the host cannot distinguish from the operator moving slowly.
        /// Dropping is visible (SkippedFrames, and the host's staleness
        /// watchdog); queueing is not.</summary>
        public async Task SendTrackingAsync(TrackingSample s)
        {
            if (!IsConnected) return;
            if (!await _sendLock.WaitAsync(0)) { SkippedFrames++; return; }
            try
            {
                // Packed inside the lock: the buffers are reused, so packing
                // outside it would let one frame rewrite bytes that SendAsync
                // is still reading from the previous one.
                var buffer = s.HandMode ? _handBuffer : _controllerBuffer;
                var n = Pack(buffer, in s);
                await _socket.SendAsync(new ArraySegment<byte>(buffer, 0, n),
                                        WebSocketMessageType.Binary, true, _cts.Token);
            }
            catch (Exception e) { Debug.LogWarning($"[XrLink] tracking send: {e.Message}"); }
            finally { _sendLock.Release(); }
        }

        // ------------------------------------------------------------------
        // framing -- must mirror teleop/xr/codec.py exactly
        // ------------------------------------------------------------------
        private int Pack(byte[] b, in TrackingSample s)
        {
            var flags = Flags.None;
            if (s.Worn) flags |= Flags.Worn;
            if (s.LeftTracked) flags |= Flags.LeftTracked;
            if (s.RightTracked) flags |= Flags.RightTracked;
            if (s.HandMode) flags |= Flags.HandMode;

            var o = 0;
            WriteU16(b, ref o, Magic);
            b[o++] = ProtoVersion;
            b[o++] = (byte)flags;
            WriteU32(b, ref o, unchecked(++_seq));
            WriteF64(b, ref o, s.DeviceTime);

            WriteMatrix(b, ref o, s.Head);
            WriteMatrix(b, ref o, s.LeftWrist);
            WriteMatrix(b, ref o, s.RightWrist);

            // INPUT_FIELDS order is part of the contract; do not reorder.
            WriteF32(b, ref o, s.LeftPinch);
            WriteF32(b, ref o, s.RightPinch);
            WriteF32(b, ref o, s.LeftTrigger);
            WriteF32(b, ref o, s.RightTrigger);
            WriteF32(b, ref o, s.LeftThumb.x);
            WriteF32(b, ref o, s.LeftThumb.y);
            WriteF32(b, ref o, s.RightThumb.x);
            WriteF32(b, ref o, s.RightThumb.y);

            if (s.HandMode)
            {
                WriteJoints(b, ref o, s.LeftJoints);
                WriteJoints(b, ref o, s.RightJoints);
            }
            return o;
        }

        /// <summary>4x4 row-major float32, matching numpy's default layout.</summary>
        private static void WriteMatrix(byte[] b, ref int o, Matrix4x4 m)
        {
            for (var row = 0; row < 4; row++)
                for (var col = 0; col < 4; col++)
                    WriteF32(b, ref o, m[row, col]);
        }

        private static void WriteJoints(byte[] b, ref int o, Vector3[] joints)
        {
            for (var i = 0; i < 25; i++)
            {
                var v = (joints != null && i < joints.Length) ? joints[i] : Vector3.zero;
                WriteF32(b, ref o, v.x);
                WriteF32(b, ref o, v.y);
                WriteF32(b, ref o, v.z);
            }
        }

        // Bytes are emitted explicitly rather than via BitConverter.GetBytes:
        // that allocates a throwaway array per value (56 per frame at 72Hz, in
        // VR, where GC spikes are visible as judder), and it assumes the host
        // is little-endian. Writing the bytes by hand costs nothing and makes
        // the wire format independent of the CPU it is produced on.
        [StructLayout(LayoutKind.Explicit)]
        private struct F32Bits
        {
            [FieldOffset(0)] public float F;
            [FieldOffset(0)] public uint U;
        }

        [StructLayout(LayoutKind.Explicit)]
        private struct F64Bits
        {
            [FieldOffset(0)] public double D;
            [FieldOffset(0)] public ulong U;
        }

        private static void WriteU16(byte[] b, ref int o, ushort v)
        { b[o++] = (byte)v; b[o++] = (byte)(v >> 8); }

        private static void WriteU32(byte[] b, ref int o, uint v)
        { for (var i = 0; i < 4; i++) b[o++] = (byte)(v >> (8 * i)); }

        private static void WriteF32(byte[] b, ref int o, float v)
        {
            var bits = new F32Bits { F = v }.U;
            for (var i = 0; i < 4; i++) b[o++] = (byte)(bits >> (8 * i));
        }

        private static void WriteF64(byte[] b, ref int o, double v)
        {
            var bits = new F64Bits { D = v }.U;
            for (var i = 0; i < 8; i++) b[o++] = (byte)(bits >> (8 * i));
        }
    }

    public struct TrackingSample
    {
        public double DeviceTime;
        public bool Worn, LeftTracked, RightTracked, HandMode;
        public Matrix4x4 Head, LeftWrist, RightWrist;
        public float LeftPinch, RightPinch, LeftTrigger, RightTrigger;
        public Vector2 LeftThumb, RightThumb;
        public Vector3[] LeftJoints, RightJoints;   // 25 each, hand mode only
    }
}

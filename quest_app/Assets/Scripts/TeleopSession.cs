// The Quest app's session behaviour: presence, tracking, and host state.
//
// Attach to one GameObject in the scene. Everything safety-relevant is here;
// the UI is expected to read SessionState/AlignReason and render them, and
// nothing else.
//
// Three things in this file are easy to get wrong and expensive to debug on
// device. In order of how much they will hurt:
//
//  1. HANDEDNESS. Unity is left-handed (+z forward); OpenXR is right-handed
//     (+z backward). The wire format is OpenXR. ToOpenXR() below is the entire
//     conversion -- if arms mirror left/right on the robot, look here first.
//  2. PRESENCE ORDERING. HMDUnmounted must transmit before anything else in
//     that frame. The OS may suspend us immediately afterwards.
//  3. NO LOCAL SAFETY LOGIC. This app reports; it does not decide. The host
//     owns every stop decision. Do not add "helpful" client-side gating -- two
//     half-implementations of a safety rule is worse than one.

using System;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;

namespace WeGo.Teleop
{
    public class TeleopSession : MonoBehaviour
    {
        [Header("Host")]
        public string HostAddress = "192.168.123.2";
        public int Port = 8443;
        public bool UseTls = true;

        [Header("Input")]
        public bool HandTracking = false;   // controllers first; see docs §12

        [Header("Read-only state for the UI")]
        public string SessionState = "DISCONNECTED";
        public string AlignReason = "";
        public float AlignProgress;
        public bool IsWorn;

        private XrLinkClient _link;
        private CancellationTokenSource _cts;
        private Transform _head, _leftHand, _rightHand;

        // ------------------------------------------------------------------
        // lifecycle
        // ------------------------------------------------------------------
        private void Awake()
        {
            if (!BitConverter.IsLittleEndian)
                Debug.LogError("[Teleop] big-endian host: the wire format is " +
                               "little-endian and will be misread by the host.");

            // Keep running when the headset is doffed, so HMDUnmounted has a
            // chance to transmit. Also see the Android manifest notes in the
            // runbook -- this flag alone is not always sufficient.
            Application.runInBackground = true;

            _head = Camera.main != null ? Camera.main.transform : transform;
        }

        private void OnEnable()
        {
            OVRManager.HMDMounted += HandleMounted;
            OVRManager.HMDUnmounted += HandleUnmounted;

            var scheme = UseTls ? "wss" : "ws";
            _link = new XrLinkClient($"{scheme}://{HostAddress}:{Port}");
            _cts = new CancellationTokenSource();
            _ = _link.RunAsync(_cts.Token);
        }

        private void OnDisable()
        {
            OVRManager.HMDMounted -= HandleMounted;
            OVRManager.HMDUnmounted -= HandleUnmounted;

            // Best effort: tell the host we are gone before we stop existing.
            // If it does not arrive the watchdog still catches it -- that is the
            // whole point of not making this load-bearing.
            _link?.SendPresence(false);
            _cts?.Cancel();
            _link?.Dispose();
        }

        // ------------------------------------------------------------------
        // presence -- the fast doff path
        // ------------------------------------------------------------------
        private void HandleUnmounted()
        {
            // FIRST. Before logging, before UI, before anything that could be
            // scheduled after an OS suspend. Fire-and-forget by design.
            _link?.SendPresence(false);
            IsWorn = false;
            SessionState = "DOFFED";
        }

        private void HandleMounted()
        {
            _link?.SendPresence(true);
            IsWorn = true;
            // Never resume on our own authority: the host re-runs alignment.
            SessionState = "WAITING";
        }

        // ------------------------------------------------------------------
        // per-frame
        // ------------------------------------------------------------------
        private void Update()
        {
            DrainHostMessages();

            if (_link == null || !_link.IsConnected)
            {
                SessionState = "DISCONNECTED";
                return;
            }
            _ = _link.SendTrackingAsync(BuildSample());
        }

        private TrackingSample BuildSample()
        {
            IsWorn = OVRManager.isUserPresent;

            var leftOk = OVRInput.IsControllerConnected(OVRInput.Controller.LTouch);
            var rightOk = OVRInput.IsControllerConnected(OVRInput.Controller.RTouch);

            var leftPose = PoseOf(OVRInput.Controller.LTouch);
            var rightPose = PoseOf(OVRInput.Controller.RTouch);
            var headPose = Matrix4x4.TRS(_head.position, _head.rotation, Vector3.one);

            // Analog inputs follow the host's inverted convention: 10.0 fully
            // open, 0.0 fully pressed. Matching it here means the gripper code
            // needs no per-source special-casing.
            float Inverted(float t) => Mathf.Clamp(10.0f - t * 10.0f, 0.0f, 10.0f);
            var lTrig = Inverted(OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger,
                                              OVRInput.Controller.LTouch));
            var rTrig = Inverted(OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger,
                                              OVRInput.Controller.RTouch));

            return new TrackingSample
            {
                DeviceTime = Time.realtimeSinceStartupAsDouble,
                Worn = IsWorn,
                LeftTracked = leftOk,
                RightTracked = rightOk,
                HandMode = HandTracking,
                Head = ToOpenXR(headPose),
                LeftWrist = ToOpenXR(leftPose),
                RightWrist = ToOpenXR(rightPose),
                LeftTrigger = lTrig,
                RightTrigger = rTrig,
                LeftPinch = lTrig,     // controllers: pinch mirrors trigger
                RightPinch = rTrig,
                LeftThumb = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick,
                                         OVRInput.Controller.LTouch),
                RightThumb = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick,
                                          OVRInput.Controller.RTouch),
            };
        }

        private static Matrix4x4 PoseOf(OVRInput.Controller c)
        {
            return Matrix4x4.TRS(OVRInput.GetLocalControllerPosition(c),
                                 OVRInput.GetLocalControllerRotation(c),
                                 Vector3.one);
        }

        /// <summary>Unity (left-handed, +z forward) -> OpenXR (right-handed,
        /// +z backward).
        ///
        /// Negating z on both the position and the basis is the whole
        /// conversion. This is the *only* convention change the device makes --
        /// the OpenXR-to-robot chain lives on the host in transforms.py, so it
        /// exists once rather than once per client.</summary>
        private static Matrix4x4 ToOpenXR(Matrix4x4 m)
        {
            var pos = m.GetColumn(3);
            var rot = m.rotation;
            var flippedPos = new Vector3(pos.x, pos.y, -pos.z);
            var flippedRot = new Quaternion(-rot.x, -rot.y, rot.z, rot.w);
            return Matrix4x4.TRS(flippedPos, flippedRot, Vector3.one);
        }

        // ------------------------------------------------------------------
        // host -> device
        // ------------------------------------------------------------------
        private void DrainHostMessages()
        {
            while (_link != null && _link.Inbound.TryDequeue(out var raw))
            {
                try
                {
                    var msg = JsonUtility.FromJson<HostMessage>(raw);
                    switch (msg.t)
                    {
                        case "state":
                            SessionState = msg.session ?? SessionState;
                            AlignReason = msg.reason ?? "";
                            AlignProgress = msg.align != null ? msg.align.progress : 0f;
                            break;
                        case "abort":
                            SessionState = "ABORTED";
                            AlignReason = msg.reason ?? "";
                            break;
                    }
                }
                catch (Exception e)
                {
                    Debug.LogWarning($"[Teleop] bad host message: {e.Message}");
                }
            }
        }

        [Serializable] private class AlignPayload { public float progress; public string reason; }
        [Serializable] private class HostMessage
        {
            public string t, session, reason;
            public AlignPayload align;
        }
    }
}

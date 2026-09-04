// The Quest app's session behaviour: presence, tracking, buttons, host state.
//
// Attach to one GameObject in the scene. Everything safety-relevant is here;
// the UI is expected to read the public read-only fields and render them, and
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
//     half-implementations of a safety rule is worse than one. The estop below
//     is not an exception: it *requests* a stop, it does not perform one.

using System;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;
using UnityEngine.XR;

namespace WeGo.Teleop
{
    public class TeleopSession : MonoBehaviour
    {
        [Header("Host")]
        public string HostAddress = "192.168.123.2";
        public int Port = 8443;
        public bool UseTls = false;   // see the TLS note in docs section 12.7

        [Header("Input")]
        public bool HandTracking = false;   // controllers first; see docs 12.2

        [Header("Read-only state for the UI")]
        public string SessionState = "DISCONNECTED";
        public string AlignReason = "";
        public float AlignProgress;
        public bool IsWorn;
        public bool LinkConnected;
        public int SkippedFrames;
        public string HostUrl = "";

        /// <summary>Where the operator's wrists have to be for the gate to
        /// pass, in Unity world space. The host sends these head-relative in
        /// robot axes; <see cref="ToUnityOffset"/> turns them back into this
        /// app's frame and <see cref="HeadPosition"/> anchors them.
        ///
        /// Valid only while <see cref="HasAlignTargets"/> is true -- forward
        /// kinematics can be unavailable, and a marker drawn from a fabricated
        /// pose would send the operator to a place the robot never asked for.</summary>
        public Vector3 LeftAlignTarget => HeadPosition + _leftTargetOffset;
        public Vector3 RightAlignTarget => HeadPosition + _rightTargetOffset;
        public bool HasAlignTargets;
        public bool AlignWithinTolerance;

        /// <summary>The host's verdict on each wrist, not the device's.
        ///
        /// The checklist used to test LeftPosError against a 0.10f constant of
        /// its own. That constant was the absolute gate's tolerance, and once
        /// the host moved to a scale-free check (docs 16.3) the two no longer
        /// agreed -- the console would have ticked, or refused to tick, on a
        /// rule the host was not applying. A readout that decides for itself
        /// what the host means is the defect that cost builds 12 and 13.</summary>
        public bool LeftInPosition, RightInPosition;

        /// <summary>Radius each ring should be drawn at, metres, from the
        /// host. Zero until a report has arrived, which the guide reads as
        /// "fall back to your own default" rather than drawing a dot.
        ///
        /// This used to be a 0.10f constant on the device, commented as
        /// matching the gate's position tolerance. The gate has no position
        /// tolerance any more (docs 16.3), and a ring sized to a rule nobody
        /// is applying tells the operator to be more precise, or less, than
        /// they actually need to be.</summary>
        public float LeftRingRadius, RightRingRadius;

        /// <summary>Both triggers held; and X and A held together. Read locally
        /// rather than echoed back from the host, because the checklist has to
        /// respond the instant the operator squeezes and a round trip would
        /// make it lag by a message. Nothing gates on these -- the host still
        /// decides.
        ///
        /// SkipHeld requires BOTH face buttons and is not latched, because
        /// that is exactly what the host tests:
        ///
        ///     skip_requested = frame.left_ctrl_aButton and frame.right_ctrl_aButton
        ///
        /// evaluated fresh every cycle. An earlier version latched on either
        /// button, so the checklist reported the position check waived while
        /// the host was still refusing to skip -- the feature looked broken
        /// when it was the readout that was wrong.</summary>
        public bool ConfirmHeld;
        public bool SkipHeld;

        /// <summary>Grip buttons, client-side only. Never sent to the host --
        /// see PollButtons for why that is deliberate and why it makes the
        /// grip the safe control for the console's own gestures.</summary>
        public bool LeftGrip, RightGrip;
        public float LeftPosError = float.PositiveInfinity;
        public float RightPosError = float.PositiveInfinity;

        /// <summary>Live centre-eye position, the same read the tracking frame
        /// is built from, so the guide and the wire cannot disagree about where
        /// the operator's head is.</summary>
        public Vector3 HeadPosition => Poses != null ? Poses.Head
            : InputTracking.GetLocalPosition(XRNode.CenterEye);

        /// <summary>Read through InputTracking rather than off
        /// OVRCameraRig.centerEyeAnchor.
        ///
        /// The rig's anchors are driven by OVRPlugin, which on this headset
        /// reports empty for the controller state and, measured on device,
        /// leaves centerEyeAnchor at identity. Anything that hung its
        /// orientation off that transform -- the console's follow behaviour and
        /// its facing/pitch readout -- simply never moved. InputTracking is a
        /// separate pipe through the XR input subsystem and is the one that
        /// works here; see docs section 14.</summary>
        public Quaternion HeadRotation => Poses != null ? Poses.HeadRotation
            : InputTracking.GetLocalRotation(XRNode.CenterEye);

        public Vector3 LeftWristPosition => Poses != null ? Poses.LeftWrist
            : InputTracking.GetLocalPosition(XRNode.LeftHand);
        public Vector3 RightWristPosition => Poses != null ? Poses.RightWrist
            : InputTracking.GetLocalPosition(XRNode.RightHand);

        public Quaternion LeftWristRotation => Poses != null ? Poses.LeftWristRotation
            : InputTracking.GetLocalRotation(XRNode.LeftHand);
        public Quaternion RightWristRotation => Poses != null ? Poses.RightWristRotation
            : InputTracking.GetLocalRotation(XRNode.RightHand);

        /// <summary>Non-null only in the desktop preview build, where there is
        /// no headset to read. Left null on device, so the properties above
        /// compile to exactly the tracking reads they always were -- the
        /// preview cannot change what ships.</summary>
        [NonSerialized] public ITeleopPoses Poses;

        /// <summary>Preview builds run the real HUD against synthetic state
        /// with no host and no headset. Everything this gates is I/O:
        /// the websocket, the OVR presence events, and the per-frame tracking
        /// send. None of the display code is aware of it, which is the point --
        /// what you see on the monitor is the same code that runs on device.</summary>
        [NonSerialized] public bool Offline;

        /// <summary>Both secondary face buttons (Y on the left controller, B on
        /// the right). Chosen because it is symmetric, reachable without
        /// looking, and not bound to anything else -- A/X already quit the
        /// session and the thumbstick clicks already damp the robot.</summary>
        private const string EstopBinding = "Y + B";
        public string EstopHint => $"{EstopBinding}  =  EMERGENCY STOP";

        private const float ButtonResendInterval = 0.1f;

        private XrLinkClient _link;
        private CancellationTokenSource _cts;

        private readonly List<string> _pressed = new List<string>(6);
        private string _lastButtonKey = null;
        private float _lastButtonSend;
        private bool _estopLatched;

        private const float DiagLogInterval = 1.0f;
        private float _lastDiagLog;

        // ------------------------------------------------------------------
        // lifecycle
        // ------------------------------------------------------------------
        private void Awake()
        {
            if (HandTracking)
            {
                // The 26->25 joint mapping is not implemented (docs 12.2). With
                // the flag on we would send all-zero joints, which the host
                // would retarget into a real finger pose. Refuse rather than
                // send something that looks like data.
                Debug.LogError("[Teleop] hand tracking is not implemented in " +
                               "this build; falling back to controllers.");
                HandTracking = false;
            }

            // Keep running when the headset is doffed, so HMDUnmounted has a
            // chance to transmit. Also see the Android manifest notes in the
            // runbook -- this flag alone is not always sufficient.
            Application.runInBackground = true;
        }

        private void OnEnable()
        {
            HostUrl = $"{(UseTls ? "wss" : "ws")}://{HostAddress}:{Port}";
            if (Offline) return;

            OVRManager.HMDMounted += HandleMounted;
            OVRManager.HMDUnmounted += HandleUnmounted;

            _link = new XrLinkClient(HostUrl);
            _cts = new CancellationTokenSource();
            _ = _link.RunAsync(_cts.Token);
        }

        private void OnDisable()
        {
            if (Offline) return;

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
        /// <summary>Latest head-camera frame, or null when the host is not
        /// sending one. TeleopHud puts this on the console's stage in place of
        /// the G1 model.</summary>
        public Texture2D CameraTexture { get; private set; }

        public int CameraFrames { get; private set; }

        private float _lastFrameAt;

        /// <summary>How long a frame stays on screen after the stream stops.
        /// A frozen picture of where the robot used to be looking is worse
        /// than no picture, because it does not announce itself -- so the
        /// stage falls back to the model rather than holding the last
        /// frame.</summary>
        private const float FrameStaleAfter = 2.0f;

        private void PollCameraFrames()
        {
            var jpeg = _link?.TakeFrame();
            if (jpeg != null)
            {
                if (CameraTexture == null)
                {
                    // Mipmaps off: this is displayed at roughly 1:1 on a flat
                    // panel and never minified, so they would cost memory and
                    // an upload every frame for nothing.
                    CameraTexture = new Texture2D(2, 2, TextureFormat.RGB24, false)
                    {
                        wrapMode = TextureWrapMode.Clamp,
                        filterMode = FilterMode.Bilinear,
                    };
                }
                // LoadImage resizes the texture to the JPEG's own dimensions.
                if (CameraTexture.LoadImage(jpeg, false))
                {
                    CameraFrames++;
                    _lastFrameAt = Time.unscaledTime;
                }
            }

            if (CameraTexture != null && Time.unscaledTime - _lastFrameAt > FrameStaleAfter)
            {
                Destroy(CameraTexture);
                CameraTexture = null;
            }
        }

        private void Update()
        {
            // The preview driver owns every field the UI reads. Falling through
            // here would immediately stamp DISCONNECTED over it.
            if (Offline) return;

            PollCameraFrames();
            DrainHostMessages();

            // Before the link gate below, deliberately. The console's grab
            // (TeleopHudGrab) is the operator tidying their own display: it is
            // not a session concern, and DISCONNECTED is exactly when they are
            // most likely to want it and exactly when the early return below
            // would otherwise have left it dead. Everything read here is
            // client-side and never reaches the wire.
            PollLocalControls();

            LinkConnected = _link != null && _link.IsConnected;
            SkippedFrames = _link?.SkippedFrames ?? 0;
            if (!LinkConnected)
            {
                SessionState = "DISCONNECTED";
                return;
            }

            PollEstop();
            PollButtons();
            _ = _link.SendTrackingAsync(BuildSample());
        }

        /// <summary>Edge-triggered and latched until release, so holding the
        /// binding sends one request rather than one per frame. The host
        /// decides what a stop means; this only asks.</summary>
        private void PollEstop()
        {
            var held = GetButton(LeftDevice(), CommonUsages.secondaryButton) &&
                       GetButton(RightDevice(), CommonUsages.secondaryButton);
            if (held && !_estopLatched)
            {
                _estopLatched = true;
                _link.SendEstop();
                SessionState = "ESTOP REQUESTED";
                Debug.LogWarning("[Teleop] operator requested emergency stop");
            }
            else if (!held)
            {
                _estopLatched = false;
            }
        }

        /// <summary>Sends on change and then at ButtonResendInterval while
        /// anything is held. The host's set is replaced wholesale by each
        /// message, so the repeat is what makes a dropped message harmless --
        /// it matters because two of these buttons stop the robot.</summary>
        /// <summary>Controls the app answers for itself, with no host involved.
        ///
        /// The grips, and deliberately NOT via Collect() into _pressed: the
        /// wire protocol has no grip -- codec.py's BUTTON_NAMES does not list
        /// one and no INPUT_FIELD could carry it -- so sending it would be a
        /// name the host silently discards. That absence is exactly what makes
        /// the grip the right control for moving and collapsing the console
        /// (TeleopHudGrab): it is the one input on the pad that cannot collide
        /// with the align gate, which owns both triggers and X + A for the
        /// whole of its duration.</summary>
        private void PollLocalControls()
        {
            LeftGrip = GetButton(LeftDevice(), CommonUsages.gripButton);
            RightGrip = GetButton(RightDevice(), CommonUsages.gripButton);

            if (Time.unscaledTime - _lastGripLog < DiagLogInterval) return;
            _lastGripLog = Time.unscaledTime;
            Debug.Log($"[Teleop] grips left={LeftGrip} right={RightGrip} " +
                      $"(devices valid L={LeftDevice().isValid} R={RightDevice().isValid})");
            LogControllerFeaturesOnce();
        }

        private float _lastGripLog;

        private void PollButtons()
        {
            _pressed.Clear();
            Collect(LeftDevice(), CommonUsages.primaryButton, "left_a");
            Collect(RightDevice(), CommonUsages.primaryButton, "right_a");
            Collect(LeftDevice(), CommonUsages.secondaryButton, "left_b");
            Collect(RightDevice(), CommonUsages.secondaryButton, "right_b");
            Collect(LeftDevice(), CommonUsages.primary2DAxisClick, "left_thumb");
            Collect(RightDevice(), CommonUsages.primary2DAxisClick, "right_thumb");

            // Local mirrors for the checklist, matching what the host tests.
            ConfirmHeld = ConfirmFrom(TriggerValue(LeftDevice()),
                                      TriggerValue(RightDevice()));
            SkipHeld = _pressed.Contains("left_a") && _pressed.Contains("right_a");

            var key = string.Join(",", _pressed);
            var due = Time.unscaledTime - _lastButtonSend >= ButtonResendInterval;
            if (key == _lastButtonKey && !(due && _pressed.Count > 0)) return;

            _lastButtonKey = key;
            _lastButtonSend = Time.unscaledTime;
            _link.SendButtons(_pressed);
        }

        // ------------------------------------------------------------------
        // input devices
        //
        // Deliberately NOT OVRInput.Get(Button/Axis, Controller) below. That
        // path (and OVRInput.IsControllerConnected/GetConnectedControllers)
        // all resolve through OVRPlugin.GetControllerState6, and on-device
        // measurement (docs 14) showed that call's ConnectedControllers and
        // Buttons bits both come back empty for real hardware even while the
        // node-tracking pipe (GetLocalControllerPosition/PositionTracked,
        // used above) works. UnityEngine.XR's InputDevice/CommonUsages read
        // through the XR input subsystem instead -- a separate pipe from
        // GetControllerState6, same family as the InputTracking head-pose fix.
        // ------------------------------------------------------------------
        private InputDevice _leftDevice;
        private InputDevice _rightDevice;

        private InputDevice LeftDevice()
        {
            if (!_leftDevice.isValid) _leftDevice = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
            return _leftDevice;
        }

        private InputDevice RightDevice()
        {
            if (!_rightDevice.isValid) _rightDevice = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
            return _rightDevice;
        }

        /// <summary>Raw trigger pull that counts as confirming, both hands.
        ///
        /// The triggers, not the grips, because the grips do not exist as far
        /// as the host is concerned: they are not in codec.py's BUTTON_NAMES,
        /// not in its INPUT_FIELDS, and there is no field on the wire that
        /// could carry them. The host's XRFrame.confirm_gesture is
        ///
        ///     (left_hand_pinch and right_hand_pinch)
        ///       or (left_ctrl_trigger and right_ctrl_trigger)
        ///
        /// and nothing else can satisfy it. Build 13 asked the operator for
        /// the grips and lit its own HOLD CONFIRM bar when they squeezed, so
        /// the console said the gate was half-passed while the host had never
        /// seen a confirm at all -- which is also why X + A could not skip:
        /// the skip path still requires the confirm gesture, and waives only
        /// the position check.
        ///
        /// 0.9 raw is the host's threshold restated. It sends the trigger
        /// inverted, 10.0 open to 0.0 pressed (see BuildSample), and the host
        /// tests `value < 1.0`. Both halves are derived from this one constant
        /// so they cannot drift apart again. Strictly greater, because the
        /// host's test is strictly less-than.</summary>
        private const float ConfirmTriggerRaw = 0.9f;

        private static bool ConfirmFrom(float leftRaw, float rightRaw)
        {
            return leftRaw > ConfirmTriggerRaw && rightRaw > ConfirmTriggerRaw;
        }

        private static bool GetButton(InputDevice device, InputFeatureUsage<bool> usage)
        {
            return device.TryGetFeatureValue(usage, out var pressed) && pressed;
        }

        /// <summary>Every feature the controllers actually expose, once.
        ///
        /// This app has been bitten twice by an input that exists in the API
        /// and returns nothing on this hardware (OVRInput's buttons, then its
        /// connected-controller list -- docs 14). Guessing a third time is not
        /// worth a build cycle: TeleopHudGrab needs the grip, so the question
        /// of whether "GripButton" is in this list is worth answering out
        /// loud rather than inferring from a control that appears not to
        /// work.</summary>
        private bool _featuresLogged;

        private void LogControllerFeaturesOnce()
        {
            if (_featuresLogged) return;
            var left = LeftDevice();
            var right = RightDevice();
            if (!left.isValid && !right.isValid) return;
            _featuresLogged = true;

            var usages = new List<InputFeatureUsage>();
            if (left.isValid && left.TryGetFeatureUsages(usages))
                Debug.Log($"[Teleop] left controller features: " +
                          $"{string.Join(", ", usages.ConvertAll(u => u.name))}");
            usages.Clear();
            if (right.isValid && right.TryGetFeatureUsages(usages))
                Debug.Log($"[Teleop] right controller features: " +
                          $"{string.Join(", ", usages.ConvertAll(u => u.name))}");
        }

        private void Collect(InputDevice device, InputFeatureUsage<bool> usage, string name)
        {
            if (GetButton(device, usage)) _pressed.Add(name);
        }

        private static float TriggerValue(InputDevice device)
        {
            return device.TryGetFeatureValue(CommonUsages.trigger, out var v) ? v : 0f;
        }

        private static Vector2 ThumbstickValue(InputDevice device)
        {
            return device.TryGetFeatureValue(CommonUsages.primary2DAxis, out var v) ? v : Vector2.zero;
        }

        private TrackingSample BuildSample()
        {
            // Instance property, and the instance can be null before the rig
            // finishes waking. Absent evidence of presence is reported as
            // absence, never as presence -- the host latches on worn=false,
            // which is the safe direction to be wrong in.
            var ovr = OVRManager.instance;
            IsWorn = ovr != null && ovr.isUserPresent;

            // IsControllerConnected() reports pairing state from the legacy
            // GetControllerState6 pipe, which came back false for both real
            // Touch controllers on device even while they streamed live poses
            // (docs 14.2 defect 2/3). GetControllerPositionTracked() reads the
            // node-tracking pipe directly -- the same one PoseOf() below already
            // relies on -- so this is what "the controller is actually usable"
            // means here, not just "paired".
            var leftOk = OVRInput.GetControllerPositionTracked(OVRInput.Controller.LTouch);
            var rightOk = OVRInput.GetControllerPositionTracked(OVRInput.Controller.RTouch);

            // Do not resolve the head through OVRCameraRig's anchor: on device
            // that anchor only updates when OVRNodeStateProperties.IsHmdPresent()
            // is true, and it measured false for the whole session, freezing
            // centerEyeAnchor at the identity pose (docs 14.2 defect 1, 14.3).
            // InputTracking reads the CenterEye node the same unconditional way
            // PoseOf() reads controller nodes, with no such gate.
            var headPose = Matrix4x4.TRS(InputTracking.GetLocalPosition(XRNode.CenterEye),
                                         InputTracking.GetLocalRotation(XRNode.CenterEye),
                                         Vector3.one);
            var leftPose = PoseOf(OVRInput.Controller.LTouch);
            var rightPose = PoseOf(OVRInput.Controller.RTouch);

            // Analog inputs follow the host's inverted convention: 10.0 fully
            // open, 0.0 fully pressed. Matching it here means the gripper code
            // needs no per-source special-casing. Read via the same InputDevice
            // path as buttons above, not OVRInput.Get(Axis1D/Axis2D, Controller)
            // -- that also resolves through the GetControllerState6 struct that
            // measured empty on device (docs 14.2 defect 2/3).
            float Inverted(float t) => Mathf.Clamp(10.0f - t * 10.0f, 0.0f, 10.0f);
            var lTrig = Inverted(TriggerValue(LeftDevice()));
            var rTrig = Inverted(TriggerValue(RightDevice()));
            var lThumb = ThumbstickValue(LeftDevice());
            var rThumb = ThumbstickValue(RightDevice());

            // Once-a-second, not per-frame: the head-pose and tracked-state
            // fixes above were only trustworthy once this line could actually
            // be read on device (docs 14.2 defect 4 -- Debug.Log is stripped
            // from a non-Development build). Buttons stayed empty for the
            // whole first session; this is what settles whether they still do.
            if (Time.unscaledTime - _lastDiagLog >= DiagLogInterval)
            {
                _lastDiagLog = Time.unscaledTime;
                Debug.Log($"[Teleop] diag leftDeviceValid={LeftDevice().isValid} " +
                          $"rightDeviceValid={RightDevice().isValid} " +
                          $"leftTracked={leftOk} rightTracked={rightOk} " +
                          $"head=({headPose.GetColumn(3).x:F3},{headPose.GetColumn(3).y:F3},{headPose.GetColumn(3).z:F3}) " +
                          $"trig=({lTrig:F2},{rTrig:F2}) grip=({LeftGrip},{RightGrip}) " +
                          $"pressed=({string.Join(",", _pressed)})");
            }

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
                LeftThumb = lThumb,
                RightThumb = rThumb,
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

        /// <summary>Robot axes (+x front, +y left, +z up) -> Unity
        /// (+x right, +y up, +z forward), for a translation only.
        ///
        /// This is the inverse of the two hops an outgoing pose makes:
        /// Unity -> OpenXR is the z-negation in <see cref="ToOpenXR"/>, and
        /// OpenXR -> robot is T_ROBOT_OPENXR in the host's transforms.py.
        /// Composing the inverses collapses to a relabelling:
        ///
        ///     unity.x = -robot.y     (robot's left is the operator's -x)
        ///     unity.y =  robot.z     (both call up "up")
        ///     unity.z =  robot.x     (robot's front is the operator's forward)
        ///
        /// Translation only, deliberately: transforms.py subtracts the head
        /// position without rotating, so these offsets are in world axes and
        /// must not be re-oriented by head yaw here.</summary>
        internal static Vector3 ToUnityOffset(float[] robot)
        {
            if (robot == null || robot.Length < 3) return Vector3.zero;
            return new Vector3(-robot[1], robot[2], robot[0]);
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
                            ApplyAlignTargets(msg.align);
                            break;
                        case "abort":
                            SessionState = "ABORTED";
                            AlignReason = msg.reason ?? "";
                            AlignProgress = 0f;
                            HasAlignTargets = false;
                            break;
                    }
                }
                catch (Exception e)
                {
                    Debug.LogWarning($"[Teleop] bad host message: {e.Message}");
                }
            }
        }

        /// <summary>Store the host's targets as head-relative offsets.
        ///
        /// Deliberately not baked into world points here. The gate's condition
        /// is on `op_wrist - op_head`, so the required wrist position moves
        /// with the head: step forward and your hand has to come with you or
        /// the offset changes and the hold resets. Freezing a world point at
        /// message-arrival time would leave the marker behind and quietly ask
        /// the operator for a pose the host is not checking. The world position
        /// is therefore derived per frame in the properties below, which also
        /// makes the markers update at frame rate rather than message rate.</summary>
        private void ApplyAlignTargets(AlignPayload align)
        {
            var ok = align != null
                     && align.left_target != null && align.left_target.Length >= 3
                     && align.right_target != null && align.right_target.Length >= 3;
            if (!ok)
            {
                HasAlignTargets = false;
                return;
            }
            _leftTargetOffset = ToUnityOffset(align.left_target);
            _rightTargetOffset = ToUnityOffset(align.right_target);
            AlignWithinTolerance = align.within_tolerance;
            LeftPosError = align.left_pos_err;
            RightPosError = align.right_pos_err;
            LeftInPosition = align.left_ok;
            RightInPosition = align.right_ok;
            LeftRingRadius = align.left_radius;
            RightRingRadius = align.right_radius;
            HasAlignTargets = true;
        }

        private Vector3 _leftTargetOffset, _rightTargetOffset;

        /// <summary>Preview-only door onto the same two offsets the host sets.
        /// Deliberately takes robot-frame arrays rather than Unity vectors, so
        /// the preview exercises <see cref="ToUnityOffset"/> too -- the
        /// conversion is the part that has never been checked against a real
        /// headset, and a preview that bypassed it would prove nothing.</summary>
        internal void SetAlignTargetsForPreview(float[] leftRobot, float[] rightRobot)
        {
            _leftTargetOffset = ToUnityOffset(leftRobot);
            _rightTargetOffset = ToUnityOffset(rightRobot);
            HasAlignTargets = leftRobot != null && rightRobot != null;
        }

        [Serializable] private class AlignPayload
        {
            public float progress;
            public string reason;
            public bool within_tolerance;
            public float left_pos_err, right_pos_err;
            // The host's per-wrist verdict. Rendered as-is; the console does
            // not re-derive it from the errors above.
            public bool left_ok, right_ok;
            // Ring size, metres. The gate's tolerance is an angle, and an
            // angle has no size until it is put at a distance -- so the host
            // sends the size along with the place.
            public float left_radius, right_radius;
            // Head-relative, robot axes (+x front, +y left, +z up). Null on the
            // wire when the host has no forward kinematics; JsonUtility gives a
            // zero-length array for that, which AlignGuide reads as "no target"
            // rather than placing a marker on the operator's own head.
            public float[] left_target, right_target;
        }
        [Serializable] private class HostMessage
        {
            public string t, session, reason;
            public AlignPayload align;
        }
    }
}

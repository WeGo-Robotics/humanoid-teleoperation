// Desktop stand-in for the headset and the host.
//
// Excluded from the Android player by the guard below, so nothing here can
// reach the APK. It exists to answer one question without anyone putting a
// Quest on: does the in-headset UI actually look and behave the way it is
// supposed to?
//
// It replaces exactly two things and no more:
//
//   * the headset -- ITeleopPoses served from mouse and keyboard
//   * the host    -- the align fields TeleopSession would otherwise fill from
//                    a LinkSnapshot, computed here with the same rules
//
// TeleopHud and TeleopAlignGuide are the shipping components, unmodified and
// unaware. That is the whole design: if the preview and the device disagree,
// the bug is in the tracking or the transport, never in the drawing.
//
// Controls are drawn on screen at runtime; see HelpText.

#if UNITY_STANDALONE || UNITY_EDITOR

using UnityEngine;

namespace WeGo.Teleop
{
    public class TeleopPreviewDriver : MonoBehaviour, ITeleopPoses
    {
        public TeleopSession Session;
        public Camera Head;

        // The gate's own numbers, so the preview passes and fails where the
        // real thing does. Mirrors AlignConfig in teleop/safety/align.py.
        private const float PosTolerance = 0.10f;
        private const float GuidanceRange = 0.75f;
        private const float HoldSeconds = 2.0f;

        // Head-relative wrist targets in robot axes (x front, y left, z up),
        // from CONST_LEFT_ARM_POSE minus WAIST_OFFSET -- the same figure the
        // host derives. Right is the mirror. Documented in docs 15.x; the
        // short version is that the robot's height never enters into it, so
        // the operator is matching a posture and not a stature.
        private static readonly float[] LeftTargetRobot = { 0.0998f, 0.1487f, -0.3548f };
        private static readonly float[] RightTargetRobot = { 0.0998f, -0.1487f, -0.3548f };

        // ------------------------------------------------------------------
        // ITeleopPoses
        // ------------------------------------------------------------------
        Vector3 ITeleopPoses.Head => _headPos;
        Quaternion ITeleopPoses.HeadRotation => Quaternion.Euler(_pitch, _yaw, 0f);
        Vector3 ITeleopPoses.LeftWrist => _leftWrist;
        Vector3 ITeleopPoses.RightWrist => _rightWrist;

        // Controllers are held roughly level and pointing where the operator
        // faces. The preview has no way to know a real wrist's roll, and
        // pretending otherwise would make the marker's roll stub a lie.
        Quaternion ITeleopPoses.LeftWristRotation => Quaternion.Euler(_pitch * 0.5f, _yaw, 0f);
        Quaternion ITeleopPoses.RightWristRotation => Quaternion.Euler(_pitch * 0.5f, _yaw, 0f);

        private Vector3 _headPos = new Vector3(0f, 1.62f, 0f);
        private Vector3 _leftWrist, _rightWrist;
        private float _yaw, _pitch;

        // Wrist positions are held head-relative and resolved each frame, so
        // walking around does not drag the hands off the body.
        private Vector3 _leftLocal = new Vector3(-0.34f, -0.16f, 0.16f);
        private Vector3 _rightLocal = new Vector3(0.34f, -0.16f, 0.16f);

        private float _held;
        private bool _skipLatched;
        private GUIStyle _style;

        private void Start()
        {
            if (Session == null) Session = GetComponent<TeleopSession>();
            Session.Offline = true;
            Session.Poses = this;
            Session.LinkConnected = true;
            Session.HostUrl = "preview (no host)";
            Session.SessionState = "ALIGN";
            Session.SetAlignTargetsForPreview(LeftTargetRobot, RightTargetRobot);
            ApplyPoses();
        }

        private void Update()
        {
            // Runs its own alignment on a loop until someone touches a control,
            // then hands over and stays handed over. Without this, opening the
            // preview shows a frozen ALIGN screen and you have to read the key
            // bindings before you can see the thing you opened it to look at.
            if (_manual || Input.anyKeyDown || Input.GetMouseButton(1))
            {
                _manual = true;
                ReadInput();
            }
            else
            {
                RunDemo();
            }

            ApplyPoses();
            DriveAlign();
        }

        private bool _manual;
        private float _demoT;

        private const float DemoApproach = 7f;   // hands drift onto the targets
        private const float DemoSettle = 2.5f;   // in tolerance, not yet held
        private const float DemoHold = 3.5f;     // triggers held, gate closes
        private const float DemoFollow = 4f;     // FOLLOWING, then round again

        private void RunDemo()
        {
            var dt = Time.unscaledDeltaTime;
            _demoT += dt;

            var total = DemoApproach + DemoSettle + DemoHold + DemoFollow;
            if (_demoT > total) { _demoT = 0f; Reset(); return; }

            if (_demoT < DemoApproach)
            {
                // Ease onto the targets rather than lerping at a fixed rate, so
                // the gauge sweeps the whole range instead of racing the last
                // 20% and sitting there.
                var t = Mathf.SmoothStep(0f, 1f, _demoT / DemoApproach);
                _leftLocal = Vector3.Lerp(StartLeft, TargetLocal(LeftTargetRobot), t);
                _rightLocal = Vector3.Lerp(StartRight, TargetLocal(RightTargetRobot), t);

                // A slow look around, so the console's follow behaviour and the
                // guide's edge chevrons are both visible without touching
                // anything.
                // Enough downward pitch to bring the operator's own hands into
                // view, since that is where the rings and the controller
                // markers are and a preview that never shows them is not
                // previewing the thing that matters.
                _yaw = Mathf.Sin(_demoT * 0.55f) * 14f;
                _pitch = 20f + Mathf.Sin(_demoT * 0.4f) * 10f;
                return;
            }

            _leftLocal = TargetLocal(LeftTargetRobot);
            _rightLocal = TargetLocal(RightTargetRobot);
            _yaw = Mathf.Lerp(_yaw, 0f, 3f * dt);
            _pitch = Mathf.Lerp(_pitch, 10f, 3f * dt);

            _demoConfirming = _demoT >= DemoApproach + DemoSettle;
        }

        private bool _demoConfirming;

        private static readonly Vector3 StartLeft = new Vector3(-0.34f, -0.16f, 0.16f);
        private static readonly Vector3 StartRight = new Vector3(0.34f, -0.16f, 0.16f);

        // ------------------------------------------------------------------
        // input
        // ------------------------------------------------------------------
        private void ReadInput()
        {
            var dt = Time.unscaledDeltaTime;

            // Look with the right mouse button down, so the cursor stays free
            // for dragging a hand.
            if (Input.GetMouseButton(1))
            {
                _yaw += Input.GetAxisRaw("Mouse X") * 3f;
                _pitch = Mathf.Clamp(_pitch - Input.GetAxisRaw("Mouse Y") * 3f, -80f, 80f);
            }

            var rot = Quaternion.Euler(0f, _yaw, 0f);
            var move = new Vector3(Axis(KeyCode.D, KeyCode.A), 0f, Axis(KeyCode.W, KeyCode.S));
            _headPos += rot * move * (1.2f * dt);
            _headPos.y = Mathf.Clamp(_headPos.y + Axis(KeyCode.E, KeyCode.Q) * 0.6f * dt, 0.8f, 2.2f);

            // Hand nudging. Held keys move a wrist in head-local axes -- the
            // frame the gate measures in, so the numbers on screen move the way
            // the operator's would.
            NudgeHand(ref _leftLocal, KeyCode.Alpha1, dt);
            NudgeHand(ref _rightLocal, KeyCode.Alpha2, dt);

            // One key that puts both hands exactly where the host wants them,
            // for checking the passing case without fighting the mouse.
            if (Input.GetKey(KeyCode.G))
            {
                _leftLocal = Vector3.Lerp(_leftLocal, TargetLocal(LeftTargetRobot), 6f * dt);
                _rightLocal = Vector3.Lerp(_rightLocal, TargetLocal(RightTargetRobot), 6f * dt);
            }

            if (Input.GetKeyDown(KeyCode.R)) Reset();
            if (Input.GetKeyDown(KeyCode.Escape)) Application.Quit();
        }

        private void NudgeHand(ref Vector3 local, KeyCode modifier, float dt)
        {
            if (!Input.GetKey(modifier)) return;
            var speed = 0.55f * dt;
            local += new Vector3(Input.GetAxisRaw("Mouse X") * speed * 2f,
                                 Input.GetAxisRaw("Mouse Y") * speed * 2f,
                                 Axis(KeyCode.UpArrow, KeyCode.DownArrow) * speed);
        }

        private static float Axis(KeyCode plus, KeyCode minus)
        {
            return (Input.GetKey(plus) ? 1f : 0f) - (Input.GetKey(minus) ? 1f : 0f);
        }

        private static Vector3 TargetLocal(float[] robot)
        {
            return TeleopSession.ToUnityOffset(robot);
        }

        private void ApplyPoses()
        {
            var rot = Quaternion.Euler(_pitch, _yaw, 0f);
            _leftWrist = _headPos + rot * _leftLocal;
            _rightWrist = _headPos + rot * _rightLocal;

            if (Head == null) return;
            Head.transform.position = _headPos;
            Head.transform.rotation = rot;
        }

        // ------------------------------------------------------------------
        // the host's half
        // ------------------------------------------------------------------
        private void DriveAlign()
        {
            var le = Vector3.Distance(_leftWrist, Session.LeftAlignTarget);
            var re = Vector3.Distance(_rightWrist, Session.RightAlignTarget);
            Session.LeftPosError = le;
            Session.RightPosError = re;

            // The driver stands in for the host here, so it is the one
            // entitled to hold a tolerance and decide each hand's verdict --
            // the console reads the verdict and never re-derives it.
            Session.LeftInPosition = le <= PosTolerance;
            Session.RightInPosition = re <= PosTolerance;
            var within = Session.LeftInPosition && Session.RightInPosition;
            Session.AlignWithinTolerance = within;

            var confirming = _manual ? Input.GetKey(KeyCode.Space) : _demoConfirming;
            var skip = _manual && Input.GetKey(KeyCode.Tab);

            // The device reads these off the triggers and X+A in PollButtons;
            // offline that path never runs, so the driver supplies them.
            Session.ConfirmHeld = confirming;
            Session.SkipHeld = _skipLatched;
            if (skip && !_skipLatched) _skipLatched = true;

            var gate = confirming && (within || _skipLatched);
            _held = gate ? _held + Time.unscaledDeltaTime : 0f;

            // Progress is the worse hand's distance mapped across the guidance
            // range, then handed over to the hold timer once both are inside --
            // same two-part shape the host reports, so the bar does not jump
            // when the operator crosses into tolerance.
            var worst = Mathf.Max(le, re);
            var closeness = 1f - Mathf.Clamp01((worst - PosTolerance) /
                                               Mathf.Max(GuidanceRange - PosTolerance, 1e-4f));
            Session.AlignProgress = gate
                ? Mathf.Lerp(0.85f, 1f, Mathf.Clamp01(_held / HoldSeconds))
                : closeness * 0.85f;

            if (_held >= HoldSeconds)
            {
                Session.SessionState = "FOLLOWING";
                Session.AlignReason = _skipLatched
                    ? "position check waived by operator"
                    : "";
                return;
            }

            Session.SessionState = "ALIGN";
            Session.AlignReason = Reason(le, re, within, confirming);
        }

        private string Reason(float le, float re, bool within, bool confirming)
        {
            if (_skipLatched && !confirming) return "position waived — hold both triggers to confirm";
            if (within) return confirming ? "holding…" : "in position — hold both triggers to confirm";
            var worst = le >= re ? "left" : "right";
            var d = Mathf.Max(le, re);
            return $"move your {worst} hand {d * 100f:F0}cm to its ring";
        }

        private void Reset()
        {
            _leftLocal = StartLeft;
            _rightLocal = StartRight;
            _headPos = new Vector3(0f, 1.62f, 0f);
            _yaw = 0f;
            _pitch = 8f;
            _held = 0f;
            _skipLatched = false;
            _demoConfirming = false;
            Session.SessionState = "ALIGN";
        }

        // ------------------------------------------------------------------
        private const string HelpText =
            "PREVIEW — not the device build\n" +
            "right-drag: look    W A S D / Q E: move\n" +
            "hold 1 + move mouse: left hand    hold 2: right hand\n" +
            "G: snap both hands to target    SPACE: hold confirm\n" +
            "TAB: request skip    R: reset    ESC: quit";

        private void OnGUI()
        {
            if (_style == null)
                _style = new GUIStyle(GUI.skin.label)
                {
                    fontSize = 13,
                    normal = { textColor = new Color(0.75f, 0.79f, 0.85f) },
                };
            GUI.Label(new Rect(14f, 12f, 620f, 130f),
                      _manual ? HelpText
                              : "PREVIEW — not the device build\n" +
                                "running the alignment on a loop; " +
                                "press any key to take over",
                      _style);
            GUI.Label(new Rect(14f, Screen.height - 34f, 900f, 24f),
                      $"left {Session.LeftPosError * 100f:F0}cm   " +
                      $"right {Session.RightPosError * 100f:F0}cm   " +
                      $"progress {Session.AlignProgress * 100f:F0}%", _style);
        }
    }
}

#endif

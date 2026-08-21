// The in-headset align console.
//
// This is the layout from hud_console_preview, reproduced rather than
// reinterpreted: corner brackets, facing line, posture line, then three
// columns -- Overall Alignment / Alignment Checklist / Tip on the left, the
// stage with its align bar and hold prompts in the middle, Joint Guide / Voice
// Guide / E-Stop on the right.
//
// The centre stage is the important one. It is a RawImage fed by TeleopStage,
// and it exists to be replaced: when the robot's camera stream arrives, call
// SetStageTexture with the video texture and the rest of the console is
// unchanged. An earlier version of this file dropped the stage on the grounds
// that a viewport of the operator duplicated the room. That was wrong -- the
// panel is the stream's home, and the mirror figure is a placeholder in it.
//
// Two things from the mock are prompts here rather than controls: HOLD CONFIRM
// and HOLD SKIP. They are the grips and A/X. There is no pointer in this app,
// so a button you cannot press would be worse than a label that tells you
// which control to hold -- but they still show their hold progress, because
// that is the part the operator actually needs.
//
// Built in code rather than as a prefab for the same reason the scene is; see
// the header of TeleopBootstrap. Every element uses a point anchor and an
// explicit size, so positions read as plain numbers in a 1780x1240 panel.

using UnityEngine;
using UnityEngine.UI;

namespace WeGo.Teleop
{
    public class TeleopHud : MonoBehaviour
    {
        public TeleopSession Session;
        public Transform Anchor;
        public TeleopStage Stage;

        [Header("Placement")]
        [Tooltip("Metres in front of the head.")]
        public float Distance = 1.45f;
        [Tooltip("Degrees below eye level for the TOP edge of the console. " +
                 "The drop is derived from this and the panel's own height, " +
                 "rather than being a fixed distance, so the console stays " +
                 "hung off the horizon whatever size the layout grows to.")]
        public float TopEdgeBelowHorizon = 9f;
        [Tooltip("Seconds for the console to catch up. Non-zero so it does not " +
                 "feel welded to the face, which is a reliable way to make " +
                 "someone motion sick.")]
        public float FollowLag = 0.25f;

        // ------------------------------------------------------------------
        // geometry, in panel units
        // ------------------------------------------------------------------
        private const float W = 1780f, H = 1240f;
        private const float Pad = 30f;
        private const float ColTop = -126f;
        private const float SideW = 400f;
        private const float StageX = 452f, StageW = 876f;
        private const float RightX = 1348f;
        private const float Gap = 16f;

        // 1780 units at 0.00095 m/unit is a 1.69m console; at 1.45m that
        // subtends about 62 degrees across and 45 down.
        //
        // That is a large HUD, and it is sized from legibility rather than
        // taste. A Quest 3 resolves roughly 20 pixels per degree, so a panel
        // unit is worth 62 * 20.6 / 1780 = 0.72 device pixels and the smallest
        // text here lands at about 17 px. Going smaller means cutting content,
        // not shrinking type -- this layout is already at the floor. The
        // console only occupies the view during alignment; FOLLOWING is when
        // the operator needs the room, and by then it is the only thing up.
        private const float MetresPerUnit = 0.00095f;

        private const float PosTolerance = 0.10f;
        private const float HalfFovH = 44f, HalfFovV = 38f;

        // ------------------------------------------------------------------
        // palette -- from the design mock
        // ------------------------------------------------------------------
        private static readonly Color ConsoleBg = new Color(0.024f, 0.051f, 0.039f, 0.94f);
        private static readonly Color PanelBg = new Color(0.035f, 0.094f, 0.067f, 0.62f);
        private static readonly Color Green = new Color(0.235f, 0.878f, 0.490f);
        private static readonly Color GreenHi = new Color(0.553f, 1.000f, 0.741f);
        private static readonly Color GreenLo = new Color(0.431f, 0.882f, 0.647f, 0.55f);
        private static readonly Color Border = new Color(0.275f, 0.863f, 0.549f, 0.30f);
        private static readonly Color BorderSoft = new Color(0.275f, 0.863f, 0.549f, 0.16f);
        private static readonly Color Amber = new Color(1.000f, 0.788f, 0.247f);
        private static readonly Color Red = new Color(1.000f, 0.325f, 0.314f);
        private static readonly Color White = new Color(0.918f, 1.000f, 0.953f);
        private static readonly Color Dim = new Color(0.647f, 0.863f, 0.753f, 0.55f);
        private static readonly Color Track = new Color(1f, 1f, 1f, 0.10f);

        // ------------------------------------------------------------------
        private Transform _panel;
        private Font _font;

        private Text _facing, _posture, _pill, _message, _sub, _reason, _link;
        private Text _gaugePct, _gaugeTier, _gaugeMsg, _voice;
        private Image _pillBorder, _gaugeFill;
        private RawImage _stageImage;
        private Text _stageCaption;
        private Image _confirmFill, _skipFill;
        private Text _confirmLabel, _skipLabel;
        private RectTransform _confirmFillRect, _skipFillRect;
        private Image[] _wave;
        private Check[] _checks;

        private struct Check
        {
            public Text Label;
            public Image Marker;
            public Text Tick;
        }

        private void Start()
        {
            Build();
            Debug.Log($"[Teleop] console built: graphics=" +
                      $"{_panel.GetComponentsInChildren<Graphic>(true).Length} " +
                      $"stage={(Stage != null ? "yes" : "no")}");
        }

        /// <summary>Point the stage panel at a different texture. This is the
        /// entire integration surface for the robot's camera stream.</summary>
        public void SetStageTexture(Texture texture)
        {
            if (_stageImage != null) _stageImage.texture = texture;
        }

        private void LateUpdate()
        {
            if (_panel == null || Session == null) return;
            Follow();
            Render();
        }

        // ------------------------------------------------------------------
        // placement
        // ------------------------------------------------------------------
        private void Follow()
        {
            // Head pose comes from the session, not from Anchor.transform.
            // Anchor is an OVRCameraRig anchor, and on this headset OVRPlugin
            // leaves it at identity -- which froze both the follow and the
            // facing readout.
            var headPos = Session.HeadPosition;
            var headRot = Session.HeadRotation;

            // Yaw only. Following pitch means the console climbs out of view
            // when the operator looks down at their hands, which is exactly
            // when they most want to read it.
            var forward = headRot * Vector3.forward;
            forward.y = 0f;
            if (forward.sqrMagnitude < 1e-4f) forward = Vector3.forward;
            forward.Normalize();

            // Hung from the horizon rather than parked at a fixed depth: the
            // top edge sits TopEdgeBelowHorizon degrees below eye level and the
            // rest of the panel hangs beneath it. A constant drop put the top
            // of a 1.18 m console fifteen degrees ABOVE the eye line, which is
            // what made it read as too high -- and it would drift again every
            // time the layout changed height.
            var halfHeight = H * MetresPerUnit * 0.5f;
            var drop = Distance * Mathf.Tan(TopEdgeBelowHorizon * Mathf.Deg2Rad)
                     + halfHeight;

            var target = headPos + forward * Distance + Vector3.down * drop;

            // Upright, not aimed at the eye. LookRotation(target - headPos)
            // points the panel's normal along the head-to-panel vector, and
            // once the panel hangs below eye level that vector slopes
            // downward -- so the top leaned away from the operator and the
            // bottom leaned toward them. Facing along the horizontal forward
            // keeps the panel vertical and the text square.
            var look = Quaternion.LookRotation(forward, Vector3.up);

            // Framerate-independent smoothing; a plain Lerp factor would make
            // the console lag differently at 72Hz and 90Hz.
            var t = FollowLag <= 0f ? 1f
                  : 1f - Mathf.Exp(-Time.unscaledDeltaTime / FollowLag);
            _panel.position = Vector3.Lerp(_panel.position, target, t);
            _panel.rotation = Quaternion.Slerp(_panel.rotation, look, t);
        }

        // ------------------------------------------------------------------
        // rendering
        // ------------------------------------------------------------------
        private void Render()
        {
            var state = string.IsNullOrEmpty(Session.SessionState) ? "—" : Session.SessionState;
            var colour = ColourFor(state, Session.LinkConnected);
            var aligning = state == "ALIGN";

            var e = Session.HeadRotation.eulerAngles;
            _facing.text = $"facing {Mathf.RoundToInt(Mathf.DeltaAngle(0f, e.y))}°   ·   " +
                           $"pitch {-Mathf.RoundToInt(Mathf.DeltaAngle(0f, e.x))}°";

            _posture.text = aligning ? "HUMANOID POSTURE: STANDING G1"
                                     : $"HUMANOID POSTURE: {state}";

            _pill.text = state;
            _pill.color = colour;
            _pillBorder.color = new Color(colour.r, colour.g, colour.b, 0.45f);

            _message.text = string.IsNullOrEmpty(Session.AlignReason)
                ? (aligning ? "hold both grips to confirm" : DefaultMessage(state))
                : Session.AlignReason;

            _sub.text = aligning
                ? "Make sure your hands are inside the targets"
                : "";

            // The camera stream owns the stage only once alignment has
            // passed. During ALIGN the operator is matching a posture, and the
            // thing that helps them do that is the G1 in its reference pose --
            // a view from the robot's head shows them neither their own hands
            // nor the pose they are copying. The stream is what they need
            // afterwards, when they are actually driving.
            var camera = ShowsCamera(state) ? Session.CameraTexture : null;
            var wanted = camera != null ? camera
                       : (Stage != null ? Stage.Output : null);
            if (wanted != null && _stageImage.texture != wanted)
            {
                _stageImage.texture = wanted;
                _stageCaption.text = camera != null
                    ? "head camera · live"
                    : "reference pose · rings are your wrist targets";
            }

            var p = Mathf.Clamp01(Session.AlignProgress);
            RenderGauge(aligning ? p : 0f, aligning);
            RenderChecks(aligning);
            RenderHolds(aligning, p);
            RenderWave(aligning);

            _voice.text = aligning
                ? (string.IsNullOrEmpty(Session.AlignReason)
                    ? "Bring both hands forward to the rings."
                    : Session.AlignReason)
                : DefaultMessage(state);

            _reason.text = $"align.reason = \"{Session.AlignReason}\"";

            _link.text = Session.LinkConnected
                ? (Session.SkippedFrames > 0
                    ? $"link up   ·   {Session.HostUrl}   ·   {Session.SkippedFrames} skipped"
                    : $"link up   ·   {Session.HostUrl}")
                : $"no link   ·   retrying   ·   {Session.HostUrl}";
            _link.color = Session.LinkConnected ? Dim : Red;
        }

        private void RenderGauge(float p, bool aligning)
        {
            _gaugeFill.fillAmount = p;
            _gaugeFill.color = p >= 0.999f ? Green : (p >= 0.5f ? Amber : Red);

            if (!aligning)
            {
                // Not "0%". A gauge reading zero says the alignment is failing;
                // what is true is that none is running.
                _gaugePct.text = "—";
                _gaugePct.color = Dim;
                _gaugeTier.text = "IDLE";
                _gaugeTier.color = Dim;
                _gaugeMsg.text = "Alignment runs before every session.";
                _gaugeMsg.color = Dim;
                return;
            }

            _gaugePct.text = $"{Mathf.RoundToInt(p * 100f)}%";
            _gaugePct.color = White;

            if (p >= 0.999f) { _gaugeTier.text = "READY"; _gaugeTier.color = Green; }
            else if (p >= 0.75f) { _gaugeTier.text = "CLOSE"; _gaugeTier.color = Amber; }
            else if (p >= 0.35f) { _gaugeTier.text = "ADJUST"; _gaugeTier.color = Amber; }
            else { _gaugeTier.text = "FAR"; _gaugeTier.color = Red; }

            _gaugeMsg.text = Session.HasAlignTargets
                ? "Bring both hands to the rings."
                : "Waiting for the robot's pose.";
            _gaugeMsg.color = Session.HasAlignTargets ? Green : Amber;
        }

        private void RenderChecks(bool aligning)
        {
            var tracked = Session.HeadPosition.sqrMagnitude > 1e-6f;
            var left = Session.LeftPosError <= PosTolerance;
            var right = Session.RightPosError <= PosTolerance;

            SetCheck(0, tracked);
            SetCheck(1, Session.HasAlignTargets && InView(Session.LeftAlignTarget));
            SetCheck(2, Session.HasAlignTargets && InView(Session.RightAlignTarget));
            SetCheck(3, left);
            SetCheck(4, right);
            SetCheck(5, Session.ConfirmHeld);
            SetCheck(6, Session.AlignWithinTolerance || Session.SkipLatched);
        }

        /// <summary>Angle-based, matching TeleopAlignGuide, so "in view" on the
        /// checklist means the same thing as a ring rather than a chevron.</summary>
        private bool InView(Vector3 world)
        {
            var local = Quaternion.Inverse(Session.HeadRotation) * (world - Session.HeadPosition);
            if (local.z <= 0.01f) return false;
            return Mathf.Abs(Mathf.Atan2(local.x, local.z) * Mathf.Rad2Deg) < HalfFovH
                && Mathf.Abs(Mathf.Atan2(local.y, local.z) * Mathf.Rad2Deg) < HalfFovV;
        }

        private void SetCheck(int i, bool on)
        {
            var c = _checks[i];
            c.Label.color = on ? White : Dim;
            c.Marker.color = on ? Green : new Color(0.471f, 0.784f, 0.647f, 0.30f);
            c.Tick.color = on ? new Color(0.016f, 0.094f, 0.051f) : Color.clear;
        }

        private void RenderHolds(bool aligning, float p)
        {
            // The confirm prompt fills as the hold accumulates. Progress past
            // the gate's 85% mark is the hold timer, so that is the part shown
            // here -- below it the bar would be reporting how far the hands
            // are, which the gauge already does.
            var holding = aligning && Session.ConfirmHeld;
            var frac = holding ? Mathf.Clamp01((p - 0.85f) / 0.15f) : 0f;
            _confirmFillRect.sizeDelta = new Vector2(HoldW * frac, HoldH);
            _confirmFill.color = frac >= 0.999f ? Green : Amber;
            _confirmLabel.color = holding ? White : GreenHi;

            _skipFillRect.sizeDelta = new Vector2(Session.SkipLatched ? HoldW : 0f, HoldH);
            _skipLabel.color = Session.SkipLatched ? Amber : GreenHi;
        }

        private void RenderWave(bool aligning)
        {
            for (var i = 0; i < _wave.Length; i++)
            {
                var t = Time.unscaledTime * 3.2f + i * 0.45f;
                var a = aligning ? (0.20f + 0.80f * Mathf.Abs(Mathf.Sin(t))) : 0.16f;
                var rt = _wave[i].rectTransform;
                rt.sizeDelta = new Vector2(WaveBarW, WaveH * a);
            }
        }

        /// <summary>What to say when the host sends a state but no reason. The
        /// host owns the wording whenever it has something to say; this only
        /// covers the silence.</summary>
        private static string DefaultMessage(string state)
        {
            switch (state)
            {
                case "FOLLOWING": return "the robot is following you";
                case "IDLE": return "waiting for the operator";
                case "WAITING": return "waiting for the host";
                case "HOLD": return "holding position";
                case "SAFE_STOP": return "stopped safely — the host must resume";
                case "ABORTED": return "session aborted";
                case "DOFFED": return "headset removed";
                case "ESTOP REQUESTED": return "emergency stop sent";
                case "DISCONNECTED": return "no link to the host";
                default: return "—";
            }
        }

        /// <summary>Which states put the head camera on the stage. Only the
        /// two in which the operator is driving the robot: before alignment
        /// passes the stage belongs to the reference model, and in the stopped
        /// and disconnected states a live view would suggest a control the
        /// operator does not have.</summary>
        private static bool ShowsCamera(string state)
        {
            return state == "FOLLOWING" || state == "HOLD";
        }

        private static Color ColourFor(string session, bool linked)
        {
            if (!linked) return Red;
            // Every state the host can send is named here, including the ones
            // that want the neutral colour. A state that falls through to the
            // default is one nobody decided how to display -- see the contract
            // test in teleop/tests/test_contracts.py, which enforces it.
            switch (session)
            {
                case "FOLLOWING": return Green;
                case "IDLE": return Dim;
                case "ALIGN":
                case "HOLD":
                case "WAITING": return Amber;
                case "SAFE_STOP":
                case "ABORTED":
                case "DOFFED":
                case "ESTOP REQUESTED":
                case "DISCONNECTED": return Red;
                default: return Dim;
            }
        }

        // ------------------------------------------------------------------
        // construction
        // ------------------------------------------------------------------
        private void Build()
        {
            _font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf")
                    ?? Resources.GetBuiltinResource<Font>("Arial.ttf");

            var canvasGo = new GameObject("TeleopHud");
            var canvas = canvasGo.AddComponent<Canvas>();
            canvas.renderMode = RenderMode.WorldSpace;
            canvasGo.AddComponent<CanvasScaler>().dynamicPixelsPerUnit = 3f;

            var root = canvas.GetComponent<RectTransform>();
            root.sizeDelta = new Vector2(W, H);
            root.localScale = Vector3.one * MetresPerUnit;
            _panel = canvasGo.transform;

            BuildShell(root);
            BuildHeader(root);
            BuildLeftColumn(root);
            BuildStageColumn(root);
            BuildRightColumn(root);
        }

        private void BuildShell(RectTransform root)
        {
            Sprite9(root, TeleopHudTextures.RoundedRect(26), ConsoleBg,
                    Anchor05, Vector2.zero, new Vector2(W, H));
            Sprite9(root, TeleopHudTextures.RoundedRect(26, 2f), BorderSoft,
                    Anchor05, Vector2.zero, new Vector2(W, H));

            var scan = Sprite9(root, TeleopHudTextures.Scanline(0.035f), GreenLo,
                               Anchor05, Vector2.zero, new Vector2(W - 8f, H - 8f));
            scan.type = Image.Type.Tiled;

            // Corner brackets. Place() sets pivot == anchor, so anchoring each
            // bracket to its own corner mirrors the same two positive-size
            // rectangles into all four without sign juggling.
            const float bs = 46f, bt = 4f, bi = 18f;
            foreach (var corner in Corners)
            {
                var o = new Vector2(corner.x < 0.5f ? bi : -bi, corner.y > 0.5f ? -bi : bi);
                Solid(root, Green, corner, o, new Vector2(bs, bt));
                Solid(root, Green, corner, o, new Vector2(bt, bs));
            }
        }

        private void BuildHeader(RectTransform root)
        {
            _facing = Label(root, 30, FontStyle.Normal, GreenHi, TextAnchor.UpperCenter,
                            new Vector2(0f, -24f), new Vector2(W, 40f));

            // The mock's RECENTER button, as a prompt: there is no pointer in
            // this app, so it names the control instead of pretending to be one.
            Label(root, 24, FontStyle.Normal, Dim, TextAnchor.UpperRight,
                  new Vector2(-Pad - 40f, -26f), new Vector2(360f, 34f))
                .text = "RECENTER — hold ☰";

            _posture = Label(root, 38, FontStyle.Bold, White, TextAnchor.UpperCenter,
                             new Vector2(0f, -70f), new Vector2(W, 46f));
        }

        // ------------------------------------------------------------------
        private void BuildLeftColumn(RectTransform root)
        {
            var gauge = Panel(root, new Vector2(Pad, ColTop), new Vector2(SideW, 360f),
                              "Overall Alignment");

            const float ring = 196f;
            var centre = new Vector2(SideW * 0.5f - ring * 0.5f, -70f);

            Sprite9(gauge, TeleopHudTextures.Ring(192, 0.16f), Track,
                    AnchorTopLeft, centre, new Vector2(ring, ring));

            _gaugeFill = Sprite9(gauge, TeleopHudTextures.Ring(192, 0.16f), Amber,
                                 AnchorTopLeft, centre, new Vector2(ring, ring));
            // Radial fill from the top, clockwise -- the direction a gauge is
            // read, and the direction the mock's SVG stroke ran.
            _gaugeFill.type = Image.Type.Filled;
            _gaugeFill.fillMethod = Image.FillMethod.Radial360;
            _gaugeFill.fillOrigin = (int)Image.Origin360.Top;
            _gaugeFill.fillClockwise = true;
            _gaugeFill.fillAmount = 0f;

            _gaugePct = Label(gauge, 62, FontStyle.Bold, White, TextAnchor.MiddleCenter,
                              new Vector2(SideW * 0.5f - 90f, -128f), new Vector2(180f, 74f));
            _gaugeTier = Label(gauge, 24, FontStyle.Bold, Amber, TextAnchor.MiddleCenter,
                               new Vector2(SideW * 0.5f - 90f, -198f), new Vector2(180f, 30f));
            _gaugeMsg = Label(gauge, 25, FontStyle.Normal, Green, TextAnchor.UpperCenter,
                              new Vector2(18f, -272f), new Vector2(SideW - 36f, 76f));

            BuildChecklist(root, ColTop - 360f - Gap);

            // Tip. The figure is deliberately large: at the previous 120 units
            // the G1 was a smudge, and a reference image nobody can make out is
            // just a dark rectangle taking up a panel.
            var tipTop = ColTop - 360f - Gap - 420f - Gap;
            var tip = Panel(root, new Vector2(Pad, tipTop), new Vector2(SideW, 272f), "Tip");

            var figure = Resources.Load<Texture2D>("g1_reference");
            var textX = 24f;
            if (figure != null)
            {
                const float figH = 210f;
                var w = figH * figure.width / figure.height;
                var img = new GameObject("Figure", typeof(RawImage));
                var raw = img.GetComponent<RawImage>();
                raw.texture = figure;
                // Tinted above white: the render is mid-grey on a near-black
                // panel and reads as a smudge at 1:1. The mock used a CSS
                // brightness filter for the same reason.
                raw.color = new Color(1.55f, 1.70f, 1.60f, 1f);
                raw.raycastTarget = false;
                Place(img, tip, AnchorTopLeft, new Vector2(18f, -52f), new Vector2(w, figH));
                textX = 18f + w + 16f;
            }

            Label(tip, 24, FontStyle.Normal, Dim, TextAnchor.UpperLeft,
                  new Vector2(textX, -52f), new Vector2(SideW - textX - 18f, 210f))
                .text = "Copy the pose, don't chase the numbers. Relaxed stance, " +
                        "elbows bent, hands at belly height — the 10 cm tolerance " +
                        "does the rest.";
        }

        private static readonly string[] CheckNames =
        {
            "Head Tracked",
            "Left Target In View",
            "Right Target In View",
            "Left Hand On Target",
            "Right Hand On Target",
            "Both Hands Held",
            "Position Check",
        };

        private void BuildChecklist(RectTransform root, float top)
        {
            var panel = Panel(root, new Vector2(Pad, top), new Vector2(SideW, 420f),
                              "Alignment Checklist");

            _checks = new Check[CheckNames.Length];
            for (var i = 0; i < CheckNames.Length; i++)
            {
                var y = -60f - i * 50f;
                var marker = Sprite9(panel, TeleopHudTextures.Disc(40),
                                     new Color(0.471f, 0.784f, 0.647f, 0.30f),
                                     AnchorTopLeft, new Vector2(18f, y),
                                     new Vector2(34f, 34f));
                var tick = Label(panel, 25, FontStyle.Bold, Color.clear,
                                 TextAnchor.MiddleCenter, new Vector2(18f, y),
                                 new Vector2(34f, 34f));
                tick.text = "✓";

                var label = Label(panel, 24, FontStyle.Normal, Dim, TextAnchor.MiddleLeft,
                                  new Vector2(66f, y), new Vector2(SideW - 86f, 34f));
                label.text = CheckNames[i];

                _checks[i] = new Check { Label = label, Marker = marker, Tick = tick };
            }
        }

        // ------------------------------------------------------------------
        private const float StageH = 780f;
        private const float HoldW = 300f, HoldH = 54f;

        private void BuildStageColumn(RectTransform root)
        {
            // The stage itself. Bordered like a panel but with no header, so
            // the image is the whole of it -- which is what it has to be when
            // the camera stream takes it over.
            var frame = Sprite9(root, TeleopHudTextures.RoundedRect(14), new Color(0f, 0f, 0f, 0.85f),
                                AnchorTopLeft, new Vector2(StageX, ColTop),
                                new Vector2(StageW, StageH)).rectTransform;
            Sprite9(frame, TeleopHudTextures.RoundedRect(14, 2f), Border,
                    Anchor05, Vector2.zero, new Vector2(StageW, StageH));

            var imgGo = new GameObject("StageImage", typeof(RawImage));
            _stageImage = imgGo.GetComponent<RawImage>();
            _stageImage.texture = Stage != null ? Stage.Output : Texture2D.blackTexture;
            _stageImage.raycastTarget = false;
            Place(imgGo, frame, AnchorTopLeft, new Vector2(4f, -4f),
                  new Vector2(StageW - 8f, StageH - 8f));

            // REFERENCE badge, bottom-right of the stage, as in the mock.
            var badge = Resources.Load<Texture2D>("g1_reference");
            if (badge != null)
            {
                const float bh = 190f;
                var bw = bh * badge.width / badge.height;
                var caption = Label(frame, 22, FontStyle.Bold, Green, TextAnchor.UpperCenter,
                                    new Vector2(StageW - 250f, -StageH + bh + 58f),
                                    new Vector2(230f, 30f));
                caption.horizontalOverflow = HorizontalWrapMode.Overflow;
                caption.text = "REFERENCE";

                var go = new GameObject("RefBadge", typeof(RawImage));
                var raw = go.GetComponent<RawImage>();
                raw.texture = badge;
                raw.color = new Color(1.55f, 1.70f, 1.60f, 1f);
                raw.raycastTarget = false;
                Place(go, frame, AnchorTopLeft,
                      new Vector2(StageW - bw - 30f, -StageH + bh + 24f),
                      new Vector2(bw, bh));
            }

            _stageCaption = Label(frame, 20, FontStyle.Normal,
                                  new Color(0.59f, 0.84f, 0.71f, 0.34f),
                                  TextAnchor.LowerLeft, new Vector2(14f, -StageH + 30f),
                                  new Vector2(520f, 26f));
            _stageCaption.text = "reference pose · rings are your wrist targets";

            BuildAlignBar(root, ColTop - StageH - Gap);
        }

        private void BuildAlignBar(RectTransform root, float top)
        {
            const float barH = 150f, rowH = 96f, pillW = 300f;

            var bar = Sprite9(root, TeleopHudTextures.RoundedRect(12), new Color(0.024f, 0.063f, 0.043f, 0.72f),
                              AnchorTopLeft, new Vector2(StageX, top),
                              new Vector2(StageW, barH)).rectTransform;
            Sprite9(bar, TeleopHudTextures.RoundedRect(12, 2f), Border,
                    Anchor05, Vector2.zero, new Vector2(StageW, barH));

            _pillBorder = Sprite9(bar, TeleopHudTextures.RoundedRect(10, 2f), Border,
                                  AnchorTopLeft, new Vector2(14f, -12f),
                                  new Vector2(pillW, rowH - 12f));
            _pill = Label(bar, 42, FontStyle.Bold, Green, TextAnchor.MiddleCenter,
                          new Vector2(14f, -12f), new Vector2(pillW, rowH - 12f));
            // Shrink-to-fit rather than wrap: a wrapped state name splits
            // "FOLLOWING" across two lines inside the pill, and the pill is the
            // one element that has to be readable at a glance.
            _pill.horizontalOverflow = HorizontalWrapMode.Overflow;
            _pill.resizeTextForBestFit = true;
            _pill.resizeTextMinSize = 22;
            _pill.resizeTextMaxSize = 42;

            _message = Label(bar, 30, FontStyle.Normal, White, TextAnchor.MiddleLeft,
                             new Vector2(pillW + 36f, -12f),
                             new Vector2(StageW - pillW - 56f, rowH - 12f));

            Solid(bar, BorderSoft, AnchorTopLeft, new Vector2(0f, -rowH),
                  new Vector2(StageW, 2f));

            _sub = Label(bar, 24, FontStyle.Normal, Dim, TextAnchor.UpperCenter,
                         new Vector2(20f, -rowH - 14f), new Vector2(StageW - 40f, 36f));

            // Hold prompts.
            var holdsTop = top - barH - 12f;
            _confirmFill = BuildHold(root, new Vector2(StageX, holdsTop),
                                     "HOLD CONFIRM  (grips)", out _confirmLabel,
                                     out _confirmFillRect);
            _skipFill = BuildHold(root, new Vector2(StageX + HoldW + 16f, holdsTop),
                                  "HOLD SKIP  (A / X)", out _skipLabel, out _skipFillRect);

            _reason = Label(root, 22, FontStyle.Normal, GreenLo, TextAnchor.MiddleLeft,
                            new Vector2(StageX + 2f * HoldW + 44f, holdsTop),
                            new Vector2(StageW - 2f * HoldW - 46f, HoldH));

            _link = Label(root, 22, FontStyle.Normal, Dim, TextAnchor.UpperLeft,
                          new Vector2(StageX, holdsTop - HoldH - 12f),
                          new Vector2(StageW, 32f));
        }

        private Image BuildHold(RectTransform root, Vector2 pos, string label,
                                out Text text, out RectTransform fillRect)
        {
            var box = Sprite9(root, TeleopHudTextures.RoundedRect(10), new Color(0.027f, 0.071f, 0.047f, 0.80f),
                              AnchorTopLeft, pos, new Vector2(HoldW, HoldH)).rectTransform;
            var fill = Sprite9(box, TeleopHudTextures.RoundedRect(10), Amber,
                               AnchorTopLeft, Vector2.zero, new Vector2(0f, HoldH));
            fill.color = new Color(1f, 0.79f, 0.25f, 0.28f);
            Sprite9(box, TeleopHudTextures.RoundedRect(10, 2f), Border,
                    Anchor05, Vector2.zero, new Vector2(HoldW, HoldH));
            text = Label(box, 23, FontStyle.Bold, GreenHi, TextAnchor.MiddleCenter,
                         Vector2.zero, new Vector2(HoldW, HoldH));
            text.text = label;
            fillRect = fill.rectTransform;
            return fill;
        }

        // ------------------------------------------------------------------
        private const float WaveH = 46f, WaveBarW = 5f;

        private void BuildRightColumn(RectTransform root)
        {
            var joint = Panel(root, new Vector2(RightX, ColTop),
                              new Vector2(SideW, 220f), "Joint Guide");
            var legend = new[]
            {
                ("Good", Green), ("Adjust", Amber), ("Poor", Red),
            };
            for (var i = 0; i < legend.Length; i++)
            {
                var y = -64f - i * 48f;
                Sprite9(joint, TeleopHudTextures.Disc(32), legend[i].Item2,
                        AnchorTopLeft, new Vector2(20f, y), new Vector2(28f, 28f));
                Label(joint, 25, FontStyle.Normal, White, TextAnchor.MiddleLeft,
                      new Vector2(62f, y), new Vector2(SideW - 82f, 28f))
                    .text = legend[i].Item1;
            }

            var voiceTop = ColTop - 220f - Gap;
            var voice = Panel(root, new Vector2(RightX, voiceTop),
                              new Vector2(SideW, 260f), "Voice Guide");
            _voice = Label(voice, 25, FontStyle.Normal, White, TextAnchor.UpperLeft,
                           new Vector2(20f, -58f), new Vector2(SideW - 40f, 120f));

            _wave = new Image[22];
            for (var i = 0; i < _wave.Length; i++)
            {
                _wave[i] = Sprite9(voice, TeleopHudTextures.RoundedRect(3), Green,
                                   new Vector2(0f, 0f), new Vector2(22f + i * 9f, 26f),
                                   new Vector2(WaveBarW, WaveH * 0.4f));
            }

            var estopTop = voiceTop - 260f - Gap;
            var estop = Panel(root, new Vector2(RightX, estopTop),
                              new Vector2(SideW, 572f), null);

            Label(estop, 30, FontStyle.Bold, Red, TextAnchor.UpperCenter,
                  new Vector2(20f, -34f), new Vector2(SideW - 40f, 38f))
                .text = Session != null ? "Y+B = E-STOP" : "Y+B = E-STOP";

            // The button is a prompt, like the holds -- but drawn as the real
            // thing, because an e-stop that looks like a label is a safety
            // problem. Two discs make the dome read as a dome.
            const float dome = 190f;
            var cx = (SideW - dome) * 0.5f;
            Sprite9(estop, TeleopHudTextures.Disc(200), new Color(0.56f, 0.08f, 0.07f),
                    AnchorTopLeft, new Vector2(cx, -110f), new Vector2(dome, dome));
            Sprite9(estop, TeleopHudTextures.Disc(200), new Color(0.94f, 0.25f, 0.24f),
                    AnchorTopLeft, new Vector2(cx + 16f, -122f),
                    new Vector2(dome - 42f, dome - 42f));
            Sprite9(estop, TeleopHudTextures.Disc(200), new Color(1f, 0.62f, 0.60f, 0.55f),
                    AnchorTopLeft, new Vector2(cx + 44f, -140f),
                    new Vector2(dome * 0.36f, dome * 0.36f));

            Label(estop, 30, FontStyle.Bold, Green, TextAnchor.UpperCenter,
                  new Vector2(20f, -324f), new Vector2(SideW - 40f, 38f))
                .text = "EMERGENCY STOP";
            Label(estop, 22, FontStyle.Normal, Dim, TextAnchor.UpperCenter,
                  new Vector2(20f, -368f), new Vector2(SideW - 40f, 90f))
                .text = "Hold both face buttons for two seconds. The robot damps to a stop.";
        }

        // ------------------------------------------------------------------
        // widgets
        // ------------------------------------------------------------------
        private RectTransform Panel(RectTransform parent, Vector2 position,
                                    Vector2 size, string header)
        {
            var card = Sprite9(parent, TeleopHudTextures.RoundedRect(18), PanelBg,
                               AnchorTopLeft, position, size).rectTransform;
            Sprite9(card, TeleopHudTextures.RoundedRect(18, 2f), Border,
                    Anchor05, Vector2.zero, size);

            if (!string.IsNullOrEmpty(header))
                Label(card, 23, FontStyle.Bold, Green, TextAnchor.UpperLeft,
                      new Vector2(20f, -16f), new Vector2(size.x - 40f, 30f))
                    .text = Spaced(header);

            return card;
        }

        /// <summary>Legacy Text has no letter-spacing and the mock's section
        /// headers lean on it. Interleaving spaces is the only way to get it
        /// without pulling in TextMeshPro for six labels -- but it roughly
        /// doubles the width, and "Alignment Checklist" spaced out wraps onto
        /// two lines in a 400-unit panel. Long headers therefore get the
        /// uppercase without the spacing.</summary>
        private static string Spaced(string s)
        {
            var upper = s.ToUpperInvariant();
            return upper.Length > 12 ? upper : string.Join(" ", upper.ToCharArray());
        }

        private static readonly Vector2 Anchor05 = new Vector2(0.5f, 0.5f);
        private static readonly Vector2 AnchorTopLeft = new Vector2(0f, 1f);

        private static readonly Vector2[] Corners =
        {
            new Vector2(0f, 1f), new Vector2(1f, 1f),
            new Vector2(0f, 0f), new Vector2(1f, 0f),
        };

        private Image Sprite9(RectTransform parent, Sprite sprite, Color colour,
                              Vector2 anchor, Vector2 position, Vector2 size)
        {
            var go = new GameObject("Shape", typeof(Image));
            var image = go.GetComponent<Image>();
            image.sprite = sprite;
            image.type = Image.Type.Sliced;
            image.color = colour;
            image.raycastTarget = false;
            Place(go, parent, anchor, position, size);
            return image;
        }

        private Image Solid(RectTransform parent, Color colour, Vector2 anchor,
                            Vector2 position, Vector2 size)
        {
            var go = new GameObject("Rule", typeof(Image));
            var image = go.GetComponent<Image>();
            image.color = colour;
            image.raycastTarget = false;
            Place(go, parent, anchor, position, size);
            return image;
        }

        private Text Label(RectTransform parent, int size, FontStyle style,
                           Color colour, TextAnchor align, Vector2 position,
                           Vector2 rectSize)
        {
            var go = new GameObject("Label", typeof(Text));
            var text = go.GetComponent<Text>();
            text.font = _font;
            text.fontSize = size;
            text.fontStyle = style;
            text.color = colour;
            text.alignment = align;
            text.horizontalOverflow = HorizontalWrapMode.Wrap;
            text.verticalOverflow = VerticalWrapMode.Overflow;
            text.raycastTarget = false;
            Place(go, parent, AnchorTopLeft, position, rectSize);
            return text;
        }

        /// <summary>Anchored top-left by default: position is an offset from
        /// the parent's top-left corner, so y values are negative going
        /// down.</summary>
        private static void Place(GameObject go, RectTransform parent,
                                  Vector2 anchor, Vector2 position, Vector2 size)
        {
            var rt = go.GetComponent<RectTransform>();
            rt.SetParent(parent, false);
            // Point anchor: anchorMin == anchorMax, so sizeDelta is the literal
            // size in panel units and anchoredPosition is a literal offset.
            rt.anchorMin = anchor;
            rt.anchorMax = anchor;
            rt.pivot = anchor;
            rt.sizeDelta = size;
            rt.anchoredPosition = position;
            rt.localScale = Vector3.one;
            rt.localRotation = Quaternion.identity;
        }
    }
}

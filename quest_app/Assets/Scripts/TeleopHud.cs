// The in-headset align console.
//
// Renders what TeleopSession has been told by the host, and nothing else. It
// holds no state of its own and makes no decisions -- if you find yourself
// wanting to add a condition here that changes behaviour rather than colour,
// it belongs on the host.
//
// Layout is the console from the design mock, adapted rather than copied. Two
// things in that mock do not survive the move into a headset:
//
//   * the 3D stage. The mock had to draw the operator's body and the wrist
//     targets in a viewport, because it was a web page. In here the operator
//     IS in the scene and TeleopAlignGuide draws the rings in the actual room.
//     Reproducing the stage would be a picture of where you are, next to
//     where you are.
//   * the buttons. HOLD CONFIRM and HOLD SKIP are the grips and A/X. A panel
//     you cannot press is worse than no panel, so they read as prompts.
//
// Everything else is here: the gauge, the readiness checklist, the align bar,
// the reference figure, the posture line and the e-stop reminder.
//
// Built in code rather than as a prefab for the same reason the scene is; see
// the header of TeleopBootstrap. Every element uses a point anchor and an
// explicit size, so positions read as plain numbers in a 1780x780 panel rather
// than as interacting anchor/pivot/offset rules.

using UnityEngine;
using UnityEngine.UI;

namespace WeGo.Teleop
{
    public class TeleopHud : MonoBehaviour
    {
        public TeleopSession Session;
        public Transform Anchor;

        [Header("Placement")]
        [Tooltip("Metres in front of the head.")]
        public float Distance = 1.15f;
        [Tooltip("Metres below eye level. Kept low so the console is not in " +
                 "the way of the arms the operator is watching, but not so low " +
                 "that reading it means looking away from them.")]
        public float Drop = 0.34f;
        [Tooltip("Seconds for the console to catch up. Non-zero so it does not " +
                 "feel welded to the face, which is a reliable way to make " +
                 "someone motion sick.")]
        public float FollowLag = 0.25f;

        // ------------------------------------------------------------------
        // geometry, in panel units
        // ------------------------------------------------------------------
        private const float W = 1780f, H = 780f;
        private const float Pad = 28f;
        private const float ColTop = -118f;
        private const float SideW = 400f;
        private const float CentreX = 452f, CentreW = 876f;
        private const float RightX = 1348f;

        // 1780 units at 0.00075 m/unit is a 1.34m console; at 1.15m that
        // subtends about 60 degrees across and 28 down.
        //
        // That is large, and it is sized from legibility rather than taste. A
        // Quest 3 resolves roughly 20 pixels per degree, so a panel unit is
        // worth 60 * 20.6 / 1780 = 0.7 device pixels. Body text at 27 units
        // therefore lands at about 19 pixels tall, which is the floor for
        // comfortable reading through the optics. The first version of this
        // console was half the size, which put body text at 10 pixels: it
        // looked correct on a monitor and was unreadable in the headset. If
        // you shrink this, shrink the content first.
        private const float MetresPerUnit = 0.00075f;

        // The gate's tolerance, for the checklist's per-hand rows. Display
        // only: what actually gates is AlignWithinTolerance off the host.
        private const float PosTolerance = 0.10f;

        // ------------------------------------------------------------------
        // palette -- from the design mock
        // ------------------------------------------------------------------
        private static readonly Color ConsoleBg = new Color(0.024f, 0.051f, 0.039f, 0.93f);
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

        private Text _facing, _posture, _pill, _message, _sub, _link, _reason;
        private Text _gaugePct, _gaugeTier, _gaugeMsg;
        private Image _pillBorder, _gaugeFill, _barFill, _barTrack;
        private RectTransform _barFillRect;
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
            Debug.Log($"[Teleop] hud built: panel={_panel.position} " +
                      $"rect={((RectTransform)_panel).sizeDelta} " +
                      $"scale={_panel.localScale.x} " +
                      $"graphics={_panel.GetComponentsInChildren<Graphic>(true).Length} " +
                      $"anchor={(Anchor != null ? Anchor.name : "null")} " +
                      $"mainCam={(Camera.main != null ? Camera.main.name : "null")}");
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
            var head = Anchor != null ? Anchor : transform;

            // Yaw only. Following pitch means the console climbs out of view
            // when the operator looks down at their hands, which is exactly
            // when they most want to read it.
            var forward = head.forward;
            forward.y = 0f;
            if (forward.sqrMagnitude < 1e-4f) forward = Vector3.forward;
            forward.Normalize();

            var target = head.position + forward * Distance + Vector3.down * Drop;
            var look = Quaternion.LookRotation(target - head.position, Vector3.up);

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
            var state = string.IsNullOrEmpty(Session.SessionState)
                ? "—" : Session.SessionState;
            var colour = ColourFor(state, Session.LinkConnected);
            var aligning = state == "ALIGN";

            var head = Anchor != null ? Anchor : transform;
            var e = head.rotation.eulerAngles;
            _facing.text = $"FACING {Mathf.RoundToInt(Mathf.DeltaAngle(0f, e.y))}°   ·   " +
                           $"PITCH {-Mathf.RoundToInt(Mathf.DeltaAngle(0f, e.x))}°";

            _posture.text = aligning ? "HUMANOID POSTURE: STANDING G1"
                                     : "HUMANOID POSTURE: LIVE";

            _pill.text = state;
            _pill.color = colour;
            _pillBorder.color = new Color(colour.r, colour.g, colour.b, 0.45f);

            _message.text = string.IsNullOrEmpty(Session.AlignReason)
                ? (aligning ? "hold both grips to confirm" : DefaultMessage(state))
                : Session.AlignReason;

            _sub.text = aligning
                ? "A / X waives the position check   ·   both grips to confirm"
                : "";

            // The bar is only meaningful during alignment. At any other time a
            // progress bar sitting at zero reads as "stuck", not "not running",
            // so the track goes away entirely rather than sitting there empty.
            var p = Mathf.Clamp01(Session.AlignProgress);
            _barTrack.gameObject.SetActive(aligning);
            if (aligning)
            {
                _barFillRect.sizeDelta = new Vector2((CentreW - 2f * Pad) * p, 22f);
                _barFill.color = p >= 0.999f ? Green : Amber;
            }

            RenderGauge(aligning ? p : 0f, aligning);
            RenderChecks(aligning);

            _link.text = Session.LinkConnected
                ? (Session.SkippedFrames > 0
                    ? $"link up   ·   {Session.HostUrl}   ·   {Session.SkippedFrames} frames skipped"
                    : $"link up   ·   {Session.HostUrl}")
                : $"no link   ·   retrying   ·   {Session.HostUrl}";
            _link.color = Session.LinkConnected ? Dim : Red;

            _reason.text = $"align.reason = \"{Session.AlignReason}\"";
        }

        private void RenderGauge(float p, bool aligning)
        {
            _gaugeFill.fillAmount = p;
            _gaugeFill.color = p >= 0.999f ? Green : (p >= 0.5f ? Amber : Red);

            if (!aligning)
            {
                // Not "0%". A gauge reading zero says the alignment is failing;
                // what is true is that no alignment is running, and a dash says
                // that without inventing a number.
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
                ? "Put both hands through the rings."
                : "Waiting for the robot's pose.";
            _gaugeMsg.color = Session.HasAlignTargets ? Green : Amber;
        }

        private void RenderChecks(bool aligning)
        {
            var tracked = Session.HeadPosition.sqrMagnitude > 1e-6f;
            var left = Session.LeftPosError <= PosTolerance;
            var right = Session.RightPosError <= PosTolerance;

            SetCheck(0, Session.LinkConnected);
            SetCheck(1, tracked);
            SetCheck(2, Session.HasAlignTargets);
            SetCheck(3, left);
            SetCheck(4, right);
            SetCheck(5, Session.AlignWithinTolerance);
            SetCheck(6, !aligning && Session.LinkConnected);
        }

        private void SetCheck(int i, bool on)
        {
            var c = _checks[i];
            c.Label.color = on ? White : Dim;
            c.Marker.color = on ? Green : new Color(0.471f, 0.784f, 0.647f, 0.30f);
            c.Tick.color = on ? new Color(0.016f, 0.094f, 0.051f) : Color.clear;
        }

        /// <summary>What to say when the host sends a state but no reason. The
        /// host owns the wording whenever it has something to say; this only
        /// covers the silence, and it says what the operator should do rather
        /// than restating the state name already in the pill.</summary>
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
            BuildCentreColumn(root);
            BuildRightColumn(root);
        }

        private void BuildShell(RectTransform root)
        {
            Sprite9(root, TeleopHudTextures.RoundedRect(26), ConsoleBg,
                    Anchor05, Vector2.zero, new Vector2(W, H));
            Sprite9(root, TeleopHudTextures.RoundedRect(26, 2f), BorderSoft,
                    Anchor05, Vector2.zero, new Vector2(W, H));

            // Scanlines. Tiled, so the period is in panel units and does not
            // change with the console's size.
            var scan = Sprite9(root, TeleopHudTextures.Scanline(0.035f), GreenLo,
                               Anchor05, Vector2.zero, new Vector2(W - 8f, H - 8f));
            scan.type = Image.Type.Tiled;

            // Corner brackets: the mock's strongest single cue that this is an
            // instrument and not a notification.
            //
            // Place() sets pivot == anchor, so anchoring each bracket to its
            // own corner makes the same two positive-size rectangles mirror
            // into all four without any sign juggling.
            const float bs = 46f, bt = 4f, bi = 18f;
            foreach (var corner in Corners)
            {
                var ox = corner.x < 0.5f ? bi : -bi;
                var oy = corner.y > 0.5f ? -bi : bi;
                var o = new Vector2(ox, oy);
                Solid(root, Green, corner, o, new Vector2(bs, bt));
                Solid(root, Green, corner, o, new Vector2(bt, bs));
            }
        }

        private void BuildHeader(RectTransform root)
        {
            _facing = Label(root, 32, FontStyle.Normal, GreenHi, TextAnchor.UpperCenter,
                            new Vector2(0f, -24f), new Vector2(W, 40f));
            _posture = Label(root, 38, FontStyle.Bold, White, TextAnchor.UpperCenter,
                             new Vector2(0f, -68f), new Vector2(W, 46f));
        }

        // ------------------------------------------------------------------
        private void BuildLeftColumn(RectTransform root)
        {
            var gauge = Panel(root, new Vector2(Pad + 4f, ColTop), new Vector2(SideW, 380f),
                              "ALIGNMENT");

            const float ring = 196f;
            var centre = new Vector2(SideW * 0.5f, -196f);

            Sprite9(gauge, TeleopHudTextures.Ring(192, 0.16f), Track,
                    AnchorTopLeft, centre - new Vector2(ring * 0.5f, -ring * 0.5f),
                    new Vector2(ring, ring));

            _gaugeFill = Sprite9(gauge, TeleopHudTextures.Ring(192, 0.16f), Amber,
                                 AnchorTopLeft, centre - new Vector2(ring * 0.5f, -ring * 0.5f),
                                 new Vector2(ring, ring));
            // Radial fill from the top, clockwise -- the direction a gauge is
            // read, and the direction the mock's SVG stroke ran.
            _gaugeFill.type = Image.Type.Filled;
            _gaugeFill.fillMethod = Image.FillMethod.Radial360;
            _gaugeFill.fillOrigin = (int)Image.Origin360.Top;
            _gaugeFill.fillClockwise = true;
            _gaugeFill.fillAmount = 0f;

            _gaugePct = Label(gauge, 64, FontStyle.Bold, White, TextAnchor.MiddleCenter,
                              new Vector2(SideW * 0.5f - 90f, -158f), new Vector2(180f, 76f));
            _gaugeTier = Label(gauge, 26, FontStyle.Bold, Amber, TextAnchor.MiddleCenter,
                               new Vector2(SideW * 0.5f - 90f, -232f), new Vector2(180f, 32f));
            _gaugeMsg = Label(gauge, 26, FontStyle.Normal, Green, TextAnchor.UpperCenter,
                              new Vector2(20f, -294f), new Vector2(SideW - 40f, 76f));

            // Reference figure: the simulator's own front-on render of the G1,
            // keyed to transparency by tools/make_reference_figure.py. It is
            // here to answer "what am I matching?" without words, which is the
            // one question the old text panel could not answer at all.
            var reference = Panel(root, new Vector2(Pad + 4f, ColTop - 392f),
                                  new Vector2(SideW, 242f), "REFERENCE");
            var figure = Resources.Load<Texture2D>("g1_reference");
            if (figure != null)
            {
                const float figH = 168f;
                var w = figH * figure.width / figure.height;
                var img = new GameObject("Figure", typeof(RawImage));
                var raw = img.GetComponent<RawImage>();
                raw.texture = figure;
                // Tinted above white. The render is mid-grey on a panel that is
                // nearly black, and at 1:1 the robot reads as a smudge; the
                // mock compensated with a CSS brightness filter and this is the
                // same move. The faint green bias ties it to the console rather
                // than leaving a grey cut-out floating on it.
                raw.color = new Color(1.55f, 1.70f, 1.60f, 1f);
                raw.raycastTarget = false;
                Place(img, reference, AnchorTopLeft,
                      new Vector2(24f, -62f), new Vector2(w, figH));
            }
            else
            {
                Debug.LogWarning("[Teleop] no Resources/g1_reference; the console " +
                                 "will show the reference panel without a figure. " +
                                 "Run tools/make_reference_figure.py.");
            }

            Label(reference, 26, FontStyle.Normal, Dim, TextAnchor.UpperLeft,
                  new Vector2(174f, -62f), new Vector2(SideW - 198f, 180f))
                .text = "Stand as the robot stands. You are matching its posture, " +
                        "not its height.";
        }

        // ------------------------------------------------------------------
        private void BuildCentreColumn(RectTransform root)
        {
            var bar = Panel(root, new Vector2(CentreX, ColTop),
                            new Vector2(CentreW, 250f), null);

            // State pill and message share a row, split by a rule -- the mock's
            // row1. The pill is the one thing on the console readable from the
            // very corner of the eye.
            // Wide enough for "ESTOP REQUESTED", the longest state the host can
            // send. The pill also best-fits its text (see Label below), but
            // sizing the box for the worst case keeps the short states from
            // being rendered at a different size to the long ones.
            const float pillW = 360f, rowH = 104f;
            _pillBorder = Sprite9(bar, TeleopHudTextures.RoundedRect(14, 2f), Border,
                                  AnchorTopLeft, new Vector2(18f, -16f),
                                  new Vector2(pillW, rowH - 12f));
            _pill = Label(bar, 46, FontStyle.Bold, Green, TextAnchor.MiddleCenter,
                          new Vector2(18f, -16f), new Vector2(pillW, rowH - 12f));
            // Shrink-to-fit rather than wrap. A wrapped state name splits
            // "FOLLOWING" across two lines inside the pill, which is the one
            // element on the console that has to be readable at a glance.
            _pill.horizontalOverflow = HorizontalWrapMode.Overflow;
            _pill.resizeTextForBestFit = true;
            _pill.resizeTextMinSize = 24;
            _pill.resizeTextMaxSize = 46;

            _message = Label(bar, 33, FontStyle.Normal, White, TextAnchor.MiddleLeft,
                             new Vector2(pillW + 40f, -16f),
                             new Vector2(CentreW - pillW - 60f, rowH - 12f));

            Solid(bar, BorderSoft, AnchorTopLeft, new Vector2(18f, -rowH - 8f),
                  new Vector2(CentreW - 36f, 2f));

            _barTrack = Sprite9(bar, TeleopHudTextures.RoundedRect(11), Track,
                                AnchorTopLeft, new Vector2(Pad, -rowH - 34f),
                                new Vector2(CentreW - 2f * Pad, 22f));
            _barFill = Sprite9(_barTrack.rectTransform, TeleopHudTextures.RoundedRect(11), Amber,
                               AnchorTopLeft, Vector2.zero,
                               new Vector2(CentreW - 2f * Pad, 22f));
            _barFillRect = _barFill.rectTransform;

            _sub = Label(bar, 26, FontStyle.Normal, Dim, TextAnchor.UpperCenter,
                         new Vector2(Pad, -rowH - 78f),
                         new Vector2(CentreW - 2f * Pad, 40f));

            // Status block. The e-stop reminder is the only red thing on a
            // healthy console, so it never has to compete for attention.
            var status = Panel(root, new Vector2(CentreX, ColTop - 268f),
                               new Vector2(CentreW, 366f), "SESSION");

            Sprite9(status, TeleopHudTextures.Disc(48), Red, AnchorTopLeft,
                    new Vector2(28f, -66f), new Vector2(38f, 38f));
            Label(status, 33, FontStyle.Bold, Red, TextAnchor.MiddleLeft,
                  new Vector2(82f, -66f), new Vector2(CentreW - 106f, 38f))
                .text = Session != null ? Session.EstopHint : "Y + B  =  EMERGENCY STOP";

            Label(status, 26, FontStyle.Normal, Dim, TextAnchor.UpperLeft,
                  new Vector2(28f, -122f), new Vector2(CentreW - 56f, 40f))
                .text = "Hold both for two seconds. The robot damps to a stop.";

            Solid(status, BorderSoft, AnchorTopLeft, new Vector2(28f, -178f),
                  new Vector2(CentreW - 56f, 2f));

            _link = Label(status, 26, FontStyle.Normal, Dim, TextAnchor.UpperLeft,
                          new Vector2(28f, -200f), new Vector2(CentreW - 56f, 40f));
            _reason = Label(status, 24, FontStyle.Normal, GreenLo, TextAnchor.UpperLeft,
                            new Vector2(28f, -250f), new Vector2(CentreW - 56f, 110f));
        }

        // ------------------------------------------------------------------
        private static readonly string[] CheckNames =
        {
            "Link to host",
            "Head tracked",
            "Robot pose received",
            "Left hand on target",
            "Right hand on target",
            "Both hands in tolerance",
            "Alignment complete",
        };

        private void BuildRightColumn(RectTransform root)
        {
            var panel = Panel(root, new Vector2(RightX, ColTop),
                              new Vector2(SideW, 634f), "READINESS");

            _checks = new Check[CheckNames.Length];
            for (var i = 0; i < CheckNames.Length; i++)
            {
                var y = -66f - i * 78f;
                var marker = Sprite9(panel, TeleopHudTextures.Disc(40),
                                     new Color(0.471f, 0.784f, 0.647f, 0.30f),
                                     AnchorTopLeft, new Vector2(20f, y),
                                     new Vector2(38f, 38f));
                var tick = Label(panel, 28, FontStyle.Bold, Color.clear,
                                 TextAnchor.MiddleCenter, new Vector2(20f, y),
                                 new Vector2(38f, 38f));
                tick.text = "✓";

                var label = Label(panel, 28, FontStyle.Normal, Dim, TextAnchor.MiddleLeft,
                                  new Vector2(74f, y), new Vector2(SideW - 94f, 38f));
                label.text = CheckNames[i];

                _checks[i] = new Check { Label = label, Marker = marker, Tick = tick };
            }
        }

        // ------------------------------------------------------------------
        // widgets
        // ------------------------------------------------------------------
        /// <summary>A bordered card with an optional uppercase section header.
        /// Returns the card's rect, which children anchor inside.</summary>
        private RectTransform Panel(RectTransform parent, Vector2 position,
                                    Vector2 size, string header)
        {
            var card = Sprite9(parent, TeleopHudTextures.RoundedRect(18), PanelBg,
                               AnchorTopLeft, position, size).rectTransform;
            Sprite9(card, TeleopHudTextures.RoundedRect(18, 2f), Border,
                    Anchor05, Vector2.zero, size);

            if (!string.IsNullOrEmpty(header))
                Label(card, 25, FontStyle.Bold, Green, TextAnchor.UpperLeft,
                      new Vector2(20f, -16f), new Vector2(size.x - 40f, 32f))
                    .text = Spaced(header);

            return card;
        }

        /// <summary>Legacy Text has no letter-spacing, and the mock's section
        /// headers lean on it heavily. Interleaving thin spaces is the only way
        /// to get it without pulling in TextMeshPro for six labels.</summary>
        private static string Spaced(string s)
        {
            var chars = s.ToUpperInvariant().ToCharArray();
            return string.Join(" ", chars);
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

        /// <summary>A plain filled rectangle: rules, dividers, brackets.
        /// Anchored by <paramref name="anchor"/>, which for the corner
        /// brackets is the corner itself, so the same offsets mirror.</summary>
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

        /// <summary>Anchored top-left: position is an offset from the parent's
        /// top-left corner, so y values are negative going down.</summary>
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

// The in-headset HUD.
//
// Renders what TeleopSession has been told by the host, and nothing else. It
// holds no state of its own and makes no decisions -- if you find yourself
// wanting to add a condition here that changes behaviour rather than colour,
// it belongs on the host.
//
// Built in code rather than as a prefab for the same reason the scene is; see
// the header of TeleopBootstrap. Every element uses a point anchor and an
// explicit size, so positions read as plain numbers in a 1000x380 panel rather
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
        public float Distance = 1.1f;
        [Tooltip("Metres below eye level. Kept low so the panel is not in the " +
                 "way of the arms the operator is watching.")]
        public float Drop = 0.42f;
        [Tooltip("Seconds for the panel to catch up. Non-zero so it does not " +
                 "feel welded to the face, which is a reliable way to make " +
                 "someone motion sick.")]
        public float FollowLag = 0.25f;

        private const float PanelWidth = 1000f;
        private const float PanelHeight = 380f;
        private const float Margin = 40f;
        private const float BarWidth = PanelWidth - 2f * Margin;

        private static readonly Color Good = new Color(0.30f, 0.85f, 0.42f);
        private static readonly Color Warn = new Color(0.98f, 0.72f, 0.18f);
        private static readonly Color Bad = new Color(0.95f, 0.31f, 0.29f);
        private static readonly Color Dim = new Color(0.62f, 0.66f, 0.72f);

        private Transform _panel;
        private Text _state, _reason, _link, _hint;
        private RawImage _accent, _barBg, _barFill;
        private RectTransform _barFillRect;

        private void Start()
        {
            Build();
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

            // Yaw only. Following pitch means the panel climbs out of view when
            // the operator looks down at the robot's hands, which is exactly
            // when they most want to read it.
            var forward = head.forward;
            forward.y = 0f;
            if (forward.sqrMagnitude < 1e-4f) forward = Vector3.forward;
            forward.Normalize();

            var target = head.position + forward * Distance + Vector3.down * Drop;
            var look = Quaternion.LookRotation(target - head.position, Vector3.up);

            // Framerate-independent smoothing; a plain Lerp factor would make
            // the panel lag differently at 72Hz and 90Hz.
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
            var session = string.IsNullOrEmpty(Session.SessionState)
                ? "—" : Session.SessionState;
            var colour = ColourFor(session, Session.LinkConnected);

            _state.text = session;
            _state.color = colour;
            _accent.color = colour;

            _reason.text = Session.AlignReason ?? "";

            _link.text = Session.LinkConnected
                ? (Session.SkippedFrames > 0
                    ? $"link up   ·   {Session.HostUrl}   ·   {Session.SkippedFrames} frames skipped"
                    : $"link up   ·   {Session.HostUrl}")
                : $"no link   ·   retrying   ·   {Session.HostUrl}";
            _link.color = Session.LinkConnected ? Dim : Bad;

            // The bar is only meaningful during alignment. At any other time a
            // progress bar sitting at zero reads as "stuck", not "not running".
            var aligning = session == "ALIGN";
            _barBg.gameObject.SetActive(aligning);
            if (aligning)
            {
                var p = Mathf.Clamp01(Session.AlignProgress);
                _barFillRect.sizeDelta = new Vector2(BarWidth * p, 16f);
                _barFill.color = p >= 0.999f ? Good : Warn;
            }

            _hint.text = Session.EstopHint;
            _hint.color = Bad;
        }

        private static Color ColourFor(string session, bool linked)
        {
            if (!linked) return Bad;
            // Every state the host can send is named here, including the ones
            // that want the neutral colour. A state that falls through to the
            // default is one nobody decided how to display -- see the contract
            // test in teleop/tests/test_contracts.py, which enforces it.
            switch (session)
            {
                case "FOLLOWING": return Good;
                case "IDLE": return Dim;
                case "ALIGN":
                case "HOLD":
                case "WAITING": return Warn;
                case "SAFE_STOP":
                case "ABORTED":
                case "DOFFED":
                case "ESTOP REQUESTED":
                case "DISCONNECTED": return Bad;
                default: return Dim;
            }
        }

        // ------------------------------------------------------------------
        // construction
        // ------------------------------------------------------------------
        private void Build()
        {
            var font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf")
                       ?? Resources.GetBuiltinResource<Font>("Arial.ttf");

            var canvasGo = new GameObject("TeleopHud");
            var canvas = canvasGo.AddComponent<Canvas>();
            canvas.renderMode = RenderMode.WorldSpace;
            canvasGo.AddComponent<CanvasScaler>().dynamicPixelsPerUnit = 3f;

            var root = canvas.GetComponent<RectTransform>();
            root.sizeDelta = new Vector2(PanelWidth, PanelHeight);
            // 1000 units wide at 0.00065 m/unit is a 65cm panel; at 1.1m that
            // subtends roughly 33 degrees -- readable without being a wall.
            root.localScale = Vector3.one * 0.00065f;
            _panel = canvasGo.transform;

            // Background, centred and filling the panel.
            Image(root, new Color(0.04f, 0.05f, 0.07f, 0.82f),
                  Anchor05, Vector2.zero, new Vector2(PanelWidth, PanelHeight));

            // A colour bar down the left edge: readable at a glance in
            // peripheral vision without parsing any text.
            _accent = Image(root, Dim, AnchorLeft, Vector2.zero,
                            new Vector2(14f, PanelHeight));

            _state = Label(root, font, 64, FontStyle.Bold,
                           new Vector2(Margin, -24f), new Vector2(BarWidth, 78f));
            _reason = Label(root, font, 34, FontStyle.Normal,
                            new Vector2(Margin, -106f), new Vector2(BarWidth, 110f));

            _barBg = Image(root, new Color(1f, 1f, 1f, 0.13f), AnchorBottomLeft,
                           new Vector2(Margin, 96f), new Vector2(BarWidth, 16f));
            _barFill = Image(_barBg.rectTransform, Warn, AnchorBottomLeft,
                             Vector2.zero, new Vector2(BarWidth, 16f));
            _barFillRect = _barFill.rectTransform;

            _link = Label(root, font, 26, FontStyle.Normal,
                          new Vector2(Margin, -PanelHeight + 88f),
                          new Vector2(BarWidth, 34f));
            _hint = Label(root, font, 28, FontStyle.Bold,
                          new Vector2(Margin, -PanelHeight + 48f),
                          new Vector2(BarWidth, 34f));
        }

        private static readonly Vector2 Anchor05 = new Vector2(0.5f, 0.5f);
        private static readonly Vector2 AnchorLeft = new Vector2(0f, 0.5f);
        private static readonly Vector2 AnchorTopLeft = new Vector2(0f, 1f);
        private static readonly Vector2 AnchorBottomLeft = new Vector2(0f, 0f);

        private static RawImage Image(RectTransform parent, Color colour,
                                      Vector2 anchor, Vector2 position,
                                      Vector2 size)
        {
            var go = new GameObject("Panel", typeof(RawImage));
            var image = go.GetComponent<RawImage>();
            image.texture = Texture2D.whiteTexture;
            image.color = colour;
            image.raycastTarget = false;
            Place(go, parent, anchor, position, size);
            return image;
        }

        /// <summary>Anchored top-left: position is an offset from the panel's
        /// top-left corner, so y values are negative going down.</summary>
        private static Text Label(RectTransform parent, Font font, int size,
                                  FontStyle style, Vector2 position,
                                  Vector2 rectSize)
        {
            var go = new GameObject("Label", typeof(Text));
            var text = go.GetComponent<Text>();
            text.font = font;
            text.fontSize = size;
            text.fontStyle = style;
            text.alignment = TextAnchor.UpperLeft;
            text.horizontalOverflow = HorizontalWrapMode.Wrap;
            text.verticalOverflow = VerticalWrapMode.Truncate;
            text.raycastTarget = false;
            Place(go, parent, AnchorTopLeft, position, rectSize);
            return text;
        }

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

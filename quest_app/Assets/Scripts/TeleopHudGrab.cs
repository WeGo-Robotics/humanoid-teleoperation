// Moving and collapsing the console.
//
// The console is head-following by default, which is right while the operator
// is aligning and wrong the moment they want to look at the real robot: a
// 1.7 m panel hung in front of your face does not stop being in front of your
// face when you turn your head. So it can be pushed aside, and it can be shut.
//
// Ported from XRoboToolkit-Unity-Client's Assets/Scripts/UI/UISurroundDrag.cs,
// keeping the two decisions in it that matter:
//
//   * The panel ORBITS the operator rather than moving freely. A free panel
//     can be shoved out of reach, edge-on, or behind you, and there is no way
//     back from any of those without taking the headset off. Orbiting keeps
//     the distance and the facing fixed by construction, so every reachable
//     position is a usable one.
//
//   * A grab that does not rotate is a click. One control does both jobs, and
//     the operator never has to find a close button on a panel they are
//     already touching.
//
// What is deliberately different:
//
//   * The grip, not the trigger. Theirs uses the trigger through XR
//     Interaction Toolkit's XRUIInputModule; ours cannot, because the align
//     gate owns both triggers for its whole duration (the host's
//     confirm_gesture is both triggers held) and X + A on top of them for the
//     skip. Grabbing the console with a trigger would mean fighting the gate
//     at exactly the moment the operator is waving a ray around. The grip is
//     the one control on the pad that the wire protocol has no field for --
//     see TeleopSession.PollButtons -- so it can be spent locally for free.
//
//   * The ray is cast and intersected here rather than going through
//     EventSystem/XRUIInputModule. This app has no EventSystem and its input
//     comes from UnityEngine.XR.InputDevice directly, for the reasons in
//     TeleopSession's device section. A rectangle intersection is a dozen
//     lines and needs no colliders on a canvas built at runtime.
//
//   * Travel is clamped. Theirs is unbounded, which is fine for a settings
//     panel and not fine for the display an operator reads to decide whether a
//     humanoid is safe to drive. It cannot be put behind you.
//
// The clamps are measured against the rest position, not against the horizon,
// and the rest position is not where it looks. TeleopHud hangs the console
// from the horizon by an angle derived from the panel's own height, which
// works out at 44 degrees down. So an up-limit has to clear 45 before the
// operator can raise the console to eye level at all -- anything less reads
// on device as "it only drags left, right and down", because that is exactly
// what it does.

using UnityEngine;

namespace WeGo.Teleop
{
    [DisallowMultipleComponent]
    public class TeleopHudGrab : MonoBehaviour
    {
        public TeleopSession Session;
        public TeleopHud Hud;

        [Header("Feel")]
        [Tooltip("How quickly the console chases the drag. XRoboToolkit's " +
                 "ROTATION_SPEED, same units: a per-second Lerp rate.")]
        public float RotationSpeed = 10f;

        [Tooltip("Total travel below which a grab counts as a click rather " +
                 "than a drag, and toggles the console instead of moving it.")]
        public float ClickAngleDeg = 1.5f;

        [Header("Limits")]
        [Tooltip("Left/right travel from the rest position. Wide enough to put " +
                 "the console fully out of the way, short of losing it behind " +
                 "the operator.")]
        public float YawLimitDeg = 110f;

        [Tooltip("Travel above the rest position. Has to be large, because the " +
                 "rest position is a long way down: TeleopHud hangs the console " +
                 "44 degrees below the horizon, so anything under about 45 here " +
                 "means it can never actually be raised to eye level.")]
        public float PitchUpLimitDeg = 55f;

        [Tooltip("Travel below the rest position, for pushing it down out of " +
                 "the way without collapsing it.")]
        public float PitchDownLimitDeg = 32f;

        /// <summary>Drawn only while a ray is actually on the console. A
        /// permanent laser would add two more lines to a view that already has
        /// rings, markers and a panel in it; an appearing one is also the
        /// feedback that says "this is grabbable", which nothing else says.</summary>
        private static readonly Color HoverColour = new Color(0.55f, 1.00f, 0.74f, 0.55f);
        private static readonly Color HeldColour = new Color(1.00f, 0.79f, 0.25f, 0.85f);

        private LineRenderer _ray;
        private Material _material;

        private bool _dragging;
        private bool _isRight;
        private bool _prevLeftGrip, _prevRightGrip;
        private Quaternion _grabRot;
        private float _grabYaw, _grabPitch;
        private float _targetYaw, _targetPitch;

        private void Start()
        {
            var shader = Shader.Find("Sprites/Default") ?? Shader.Find("UI/Default");
            if (shader == null)
            {
                Debug.LogWarning("[Teleop] no unlit shader; the console ray cannot draw.");
                enabled = false;
                return;
            }
            _material = new Material(shader);

            var go = new GameObject("ConsoleRay");
            go.transform.SetParent(transform, false);
            _ray = go.AddComponent<LineRenderer>();
            _ray.material = _material;
            _ray.useWorldSpace = true;
            _ray.positionCount = 2;
            _ray.startWidth = _ray.endWidth = 0.004f;
            _ray.numCapVertices = 2;
            _ray.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            _ray.receiveShadows = false;
            _ray.enabled = false;
        }

        // LateUpdate, after TeleopHud.Follow has put the panel where it goes
        // this frame. Hit-testing against last frame's position would make the
        // console feel like it was dodging the ray while the operator turned.
        private void LateUpdate()
        {
            if (Session == null || Hud == null || Hud.PanelTransform == null) return;

            var leftGrip = Session.LeftGrip;
            var rightGrip = Session.RightGrip;

            if (_dragging)
            {
                if (_isRight ? rightGrip : leftGrip) Drag();
                else Release();
            }
            else
            {
                // Whichever hand is pointing at the console when its grip goes
                // down. Edge-triggered: a grip already held when the ray
                // wanders across the panel must not snatch it.
                if (rightGrip && !_prevRightGrip && Pointing(true, out _)) Grab(true);
                else if (leftGrip && !_prevLeftGrip && Pointing(false, out _)) Grab(false);
            }

            _prevLeftGrip = leftGrip;
            _prevRightGrip = rightGrip;

            Apply();
            DrawRay(leftGrip, rightGrip);
            Hud.SetGrabHighlight(_dragging || _ray.enabled);
        }

        // ------------------------------------------------------------------
        private void Grab(bool right)
        {
            _dragging = true;
            _isRight = right;
            _grabRot = Rotation(right);
            _grabYaw = _targetYaw = Hud.YawOffsetDeg;
            _grabPitch = _targetPitch = Hud.PitchOffsetDeg;
        }

        private void Drag()
        {
            // The controller's rotation since the grab, in its own frame, with
            // roll discarded. Roll is how you hold the pad, not where you want
            // the panel; leaving it in makes the console tilt whenever the
            // operator's wrist does.
            var delta = Quaternion.Inverse(_grabRot) * Rotation(_isRight);
            var e = delta.eulerAngles;

            _targetYaw = Mathf.Clamp(_grabYaw + Wrap(e.y), -YawLimitDeg, YawLimitDeg);
            _targetPitch = Mathf.Clamp(_grabPitch + Wrap(e.x),
                                       -PitchUpLimitDeg, PitchDownLimitDeg);
        }

        private void Release()
        {
            _dragging = false;

            // A grab that did not move it was a click. Measured against where
            // the drag started, not against the smoothed current value, so a
            // quick tap during the ease-out still reads as a tap.
            var travel = Mathf.Abs(_targetYaw - _grabYaw) + Mathf.Abs(_targetPitch - _grabPitch);
            if (travel < ClickAngleDeg) Hud.SetCollapsed(!Hud.Collapsed);
        }

        /// <summary>Framerate-independent ease, so the console does not chase
        /// the hand at a different rate at 72Hz and 90Hz.</summary>
        private void Apply()
        {
            var t = 1f - Mathf.Exp(-RotationSpeed * Time.unscaledDeltaTime);
            Hud.YawOffsetDeg = Mathf.Lerp(Hud.YawOffsetDeg, _targetYaw, t);
            Hud.PitchOffsetDeg = Mathf.Lerp(Hud.PitchOffsetDeg, _targetPitch, t);
        }

        // ------------------------------------------------------------------
        // ray
        // ------------------------------------------------------------------

        /// <summary>Ray from a controller against the console's own rectangle.
        /// The panel's local units are panel units and its size is whatever is
        /// actually on screen, so a collapsed console is only grabbable by its
        /// handle -- not by the space the full console used to fill.</summary>
        private bool Pointing(bool right, out Vector3 hit)
        {
            hit = Vector3.zero;

            var panel = Hud.PanelTransform;
            var origin = Position(right);
            var dir = Rotation(right) * Vector3.forward;

            var normal = panel.forward;
            var denom = Vector3.Dot(normal, dir);
            if (Mathf.Abs(denom) < 1e-5f) return false;          // parallel

            var distance = Vector3.Dot(normal, panel.position - origin) / denom;
            if (distance <= 0f || distance > MaxRayLength) return false;

            hit = origin + dir * distance;
            var local = panel.InverseTransformPoint(hit);
            var half = Hud.ActiveSizeUnits * 0.5f;
            return Mathf.Abs(local.x) <= half.x && Mathf.Abs(local.y) <= half.y;
        }

        private const float MaxRayLength = 6f;

        private void DrawRay(bool leftGrip, bool rightGrip)
        {
            // While dragging, the ray stays on the grabbing hand even once it
            // has swung off the panel -- otherwise it vanishes exactly when the
            // operator is using it.
            bool show;
            Vector3 hit;
            bool right;

            if (_dragging)
            {
                right = _isRight;
                show = true;
                if (!Pointing(right, out hit))
                    hit = Position(right) + Rotation(right) * Vector3.forward * 1.2f;
            }
            else
            {
                right = true;
                show = Pointing(true, out hit);
                if (!show) { right = false; show = Pointing(false, out hit); }
            }

            // Deliberately not gated on Session.LinkConnected, the way
            // TeleopPostureGuide's markers are. Those describe the host's view
            // of the operator and mean nothing without a session; this is the
            // operator tidying their own display, which they are most likely to
            // want while DISCONNECTED and waiting. Gating the ray but not the
            // grab would also have left the console grabbable with no visible
            // pointer, which is worse than either.
            _ray.enabled = show;
            if (!show) return;

            _ray.SetPosition(0, Position(right));
            _ray.SetPosition(1, hit);
            var colour = _dragging || (right ? rightGrip : leftGrip) ? HeldColour : HoverColour;
            _ray.startColor = _ray.endColor = colour;
        }

        // ------------------------------------------------------------------
        private Vector3 Position(bool right)
            => right ? Session.RightWristPosition : Session.LeftWristPosition;

        private Quaternion Rotation(bool right)
            => right ? Session.RightWristRotation : Session.LeftWristRotation;

        private static float Wrap(float degrees)
            => degrees > 180f ? degrees - 360f : degrees;

        private void OnDestroy()
        {
            if (_material != null) Destroy(_material);
        }
    }
}

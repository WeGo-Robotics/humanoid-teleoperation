// Passthrough, and keeping it that way.
//
// The operator stands in a room with a humanoid and, in the experience centre,
// an instructor beside them. They have to be able to see both. Everything this
// app draws -- the console, the wrist rings, the controller markers -- is meant
// to float over the real room, and the room comes from the headset's cameras.
//
// Why this is its own component rather than four lines in TeleopBootstrap.
//
// Passthrough initialisation is ASYNCHRONOUS. OVRManager's own path returns
// Result.Success_Pending and parks the state at PassthroughInitializationState
// .Pending; only a later frame moves it to Initialized (OVRManager.cs:3660,
// SDK 78.0.0). The bootstrap used to set isInsightPassthroughEnabled, create
// the OVRPassthroughLayer, and set the camera to clear transparent -- all in
// the same frame, and then treat "OVRManager exists" as "passthrough is on".
//
// A camera clearing to transparent black with nothing composited behind it is
// not transparent. It is BLACK. So every way passthrough can fail to come up
// -- still pending, unsupported, initialisation refused -- produced the same
// symptom: a console panel floating in a void, with the room and the person
// standing in it invisible. The fallback branch that exists for exactly this
// case could never run, because its condition was the wrong question.
//
// So: ask the SDK what actually happened, wait for it, and never clear to
// transparent until there is something behind it to see. If passthrough is not
// up the camera clears to a dim solid instead -- legible, and obviously a
// fallback rather than a dead display.
//
// It also has to keep asking. Passthrough is shut down and restarted around
// app pause and resume, which for this app means every doff and don (docs
// section 10). Deciding once at startup would leave the operator in the void
// after the first time they took the headset off.

using System.Collections;
using UnityEngine;

namespace WeGo.Teleop
{
    [DisallowMultipleComponent]
    public class TeleopPassthrough : MonoBehaviour
    {
        /// <summary>The centre-eye camera whose clear mode is being driven.</summary>
        public Camera Head;

        /// <summary>How long to wait for initialisation before reporting it as
        /// failed. Generous: this costs nothing but a dim background while it
        /// runs, and giving up early on a slow cold start would be worse than
        /// waiting.</summary>
        public float InitTimeout = 8f;

        /// <summary>Not black. A black clear is indistinguishable from a
        /// display that has died, and that is the one reading the operator
        /// must never have to guess at.</summary>
        private static readonly Color Fallback = new Color(0.05f, 0.06f, 0.08f, 1f);

        private static readonly Color Composited = new Color(0f, 0f, 0f, 0f);

        /// <summary>Whether the room is actually showing through right now.</summary>
        public bool Active { get; private set; }

        private OVRPassthroughLayer _layer;
        private bool _warned;

        private IEnumerator Start()
        {
            Apply(false);

            // OVRManager first: the support query goes through OVRPlugin, and
            // asking it anything before the manager exists is a question about
            // an uninitialised plugin.
            var mgr = OVRManager.instance;
            if (mgr == null)
            {
                Debug.LogWarning("[Teleop] no OVRManager, so passthrough cannot start.");
                yield break;
            }

            if (!OVRManager.IsInsightPassthroughSupported())
            {
                Debug.LogWarning("[Teleop] this headset reports no passthrough support. " +
                                 "The operator will not see the room, the robot, or " +
                                 "anyone standing next to them.");
                yield break;
            }

            // OVRManager acts on this in its own Update, which has not run yet
            // for a rig built this frame -- hence the wait below rather than
            // reading the state straight back.
            mgr.isInsightPassthroughEnabled = true;

            var waited = 0f;
            while (!OVRManager.IsInsightPassthroughInitialized() && waited < InitTimeout)
            {
                if (OVRManager.HasInsightPassthroughInitFailed())
                {
                    Debug.LogError("[Teleop] passthrough initialisation failed. Check that " +
                                   "com.oculus.feature.PASSTHROUGH is in the manifest " +
                                   "(QuestManifest.cs) and that the headset has granted " +
                                   "spatial data permission.");
                    yield break;
                }
                waited += Time.unscaledDeltaTime;
                yield return null;
            }

            if (!OVRManager.IsInsightPassthroughInitialized())
            {
                Debug.LogError($"[Teleop] passthrough still not initialised after {waited:F1}s " +
                               $"(pending={OVRManager.IsInsightPassthroughInitPending()}). " +
                               "Running on the fallback background.");
                yield break;
            }

            Debug.Log($"[Teleop] passthrough initialised after {waited:F2}s");
            Apply(true);
        }

        /// <summary>Passthrough goes down and comes back around pause/resume,
        /// which for this app is every doff and don. Whatever the state is now
        /// is what the camera is set for now.</summary>
        private void LateUpdate()
        {
            var up = OVRManager.IsInsightPassthroughInitialized();
            if (up != Active) Apply(up);
        }

        private void Apply(bool up)
        {
            Active = up;

            if (up && _layer == null)
            {
                // Created only once passthrough is genuinely initialised. A
                // layer built before that has nothing to attach to.
                var go = new GameObject("PassthroughLayer");
                go.transform.SetParent(transform, false);
                _layer = go.AddComponent<OVRPassthroughLayer>();
                _layer.overlayType = OVROverlay.OverlayType.Underlay;
            }
            if (_layer != null) _layer.enabled = up;

            if (Head == null)
            {
                if (!_warned)
                {
                    _warned = true;
                    Debug.LogWarning("[Teleop] passthrough has no camera to drive; the " +
                                     "background will be whatever the scene camera clears to.");
                }
                return;
            }

            Head.clearFlags = CameraClearFlags.SolidColor;
            Head.backgroundColor = up ? Composited : Fallback;
        }
    }
}

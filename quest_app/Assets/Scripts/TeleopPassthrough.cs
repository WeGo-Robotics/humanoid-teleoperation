// Passthrough, and keeping it that way.
//
// The operator stands in a room with a humanoid and, in the experience centre,
// an instructor beside them. They have to be able to see both. Everything this
// app draws -- the console, the wrist rings, the controller markers -- is meant
// to float over the real room, and the room comes from the headset's cameras.
//
// There are two ways to get it, and they are not equivalent.
//
// Insight Passthrough (OVRPassthroughLayer, Underlay) hands the job to the OS
// compositor. The room is composited behind the app's own layer by the same
// stage that reprojects it, so it is stereo, covers the full display FOV, and
// is re-warped against head motion at display rate no matter what frame rate
// this app is managing. It needs no camera permission and costs this app
// nothing to draw. This is what a well-behaved Quest MR app does, and it is
// why other robotics clients on this hardware look right.
//
// PassthroughCameraAccess hands back raw camera frames and leaves the drawing
// to us: one quad, one eye's feed, uploaded at camera rate. That means mono
// (both eyes see one flat image, so the room has no depth), a physical camera
// FOV narrower than the display's (hence CoverageFactor below, trading
// magnification against a black border -- there is no setting that removes
// both), latency and swim on head motion, and the HEADSET_CAMERA runtime
// permission.
//
// So Insight is strictly preferred and the camera quad is the fallback.
//
// Insight was unavailable on this headset from 2026-08-25 to 2026-09-04, which
// is why the camera quad exists at all. The cause was never passthrough: it was
// OculusSettings never reaching the built player, so OculusLoader.GetSettings()
// returned null and OVRManager.InitOVRManager() threw a NullReferenceException
// before it could set OVRManagerinitialized -- see QuestBuild.cs,
// EnsureOculusSettingsRegistered, for the full chain and the fix. With that
// registered, Insight has something functioning underneath it again.
//
// Both paths stay in the tree. The fallback is not dead weight: passthrough
// initialisation can genuinely fail (permission refused, unsupported device),
// and a camera quad is a great deal better than a void.

using System.Collections;
using UnityEngine;
using Meta.XR;

namespace WeGo.Teleop
{
    [DisallowMultipleComponent]
    public class TeleopPassthrough : MonoBehaviour
    {
        /// <summary>Which way the room is reaching the operator's eyes.</summary>
        public enum Source
        {
            /// <summary>Neither path came up. Fallback colour.</summary>
            None,
            /// <summary>OS compositor underlay. Stereo, full FOV, reprojected.</summary>
            Insight,
            /// <summary>Camera feed on a head-parented quad. Mono, narrower FOV.</summary>
            CameraQuad,
        }

        /// <summary>The centre-eye camera the backdrop is parented to and whose
        /// fallback clear colour is being driven.</summary>
        public Camera Head;

        [Tooltip("Metres in front of the head. Behind the HUD panel (1.45m) so " +
                 "the console reads as floating over the room rather than " +
                 "cutting into it. Camera-quad fallback only.")]
        public float Distance = 6f;

        [Tooltip("World-space size of the backdrop quad at Distance, before " +
                 "intrinsics arrive. Generous on purpose: it is head-parented, " +
                 "so any size that clears the Quest 3's ~110x96 degree FOV at " +
                 "this distance never shows an edge. Camera-quad fallback only.")]
        public Vector2 Size = new Vector2(24f, 20f);

        [Tooltip("Seconds to wait for passthrough to come up, each path, " +
                 "before falling through to the next one.")]
        public float InitTimeout = 8f;

        [Tooltip("Multiplier applied to the backdrop's true, camera-accurate " +
                 "size. The passthrough camera's physical FOV is narrower " +
                 "than the headset's display FOV, so at 1.0 the backdrop is " +
                 "geometrically correct but does not fill the view -- there " +
                 "is a Fallback-coloured border. There is no way to fill the " +
                 "view without either that border or manufactured " +
                 "magnification; this trades a bit of the latter for less " +
                 "of the former. Camera-quad fallback only; Insight has no " +
                 "such problem because the compositor owns the whole display.")]
        public float CoverageFactor = 1.3f;

        /// <summary>Not black. A black clear is indistinguishable from a
        /// display that has died, and that is the one reading the operator
        /// must never have to guess at.</summary>
        private static readonly Color Fallback = new Color(0.05f, 0.06f, 0.08f, 1f);

        /// <summary>Transparent, so the compositor's underlay shows through.
        /// Only ever set once Insight is confirmed up -- clearing to
        /// transparent with nothing behind it does not read as transparent, it
        /// reads as black.</summary>
        private static readonly Color Composited = new Color(0f, 0f, 0f, 0f);

        /// <summary>Which path won, if any.</summary>
        public Source Mode { get; private set; } = Source.None;

        /// <summary>Whether the room is actually showing through right now.</summary>
        public bool Active =>
            Mode == Source.Insight ? OVRManager.IsInsightPassthroughInitialized()
          : Mode == Source.CameraQuad ? _pca != null && _pca.IsPlaying
          : false;

        private OVRPassthroughLayer _layer;
        private PassthroughCameraAccess _pca;
        private GameObject _backdrop;
        private bool _warned;

        private IEnumerator Start()
        {
            if (Head != null)
            {
                Head.clearFlags = CameraClearFlags.SolidColor;
                Head.backgroundColor = Fallback;
            }
            else if (!_warned)
            {
                _warned = true;
                Debug.LogWarning("[Teleop] passthrough has no camera to drive; the " +
                                 "background will be whatever the scene camera clears to.");
            }

            yield return TryInsight();
            if (Mode == Source.Insight) yield break;

            Debug.LogWarning("[Teleop] falling back to the PassthroughCameraAccess quad: " +
                             "mono, narrower FOV, and it swims on head motion. Check the " +
                             "Insight diagnosis above -- this is a degraded mode, not the " +
                             "intended one.");
            TryCameraQuad();
        }

        // ------------------------------------------------------------------
        // preferred: OS compositor underlay
        // ------------------------------------------------------------------
        private IEnumerator TryInsight()
        {
            // OVRManager first: the support query goes through OVRPlugin, and
            // asking it anything before the manager exists is a question about
            // an uninitialised plugin.
            var mgr = OVRManager.instance;
            if (mgr == null)
            {
                Debug.LogWarning("[Teleop] no OVRManager, so Insight passthrough cannot start.");
                yield break;
            }

            // The tell for the OculusSettings failure described in the header.
            // OVRManager.InitOVRManager() sets this on its last line, so a false
            // here means the method threw on the way there and half of OVRManager
            // is unconfigured -- worth naming explicitly, because every symptom
            // downstream of it points somewhere else.
            if (!OVRManager.OVRManagerinitialized)
                Debug.LogWarning("[Teleop] OVRManager did not finish initialising. If a " +
                                 "NullReferenceException from InitOVRManager is above this " +
                                 "line, OculusSettings is missing from the player -- see " +
                                 "QuestBuild.EnsureOculusSettingsRegistered.");

            if (!OVRManager.IsInsightPassthroughSupported())
            {
                Debug.LogWarning("[Teleop] this headset reports no Insight passthrough support.");
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
                    Debug.LogError("[Teleop] Insight passthrough initialisation failed. Check " +
                                   "that com.oculus.feature.PASSTHROUGH is in the manifest " +
                                   "(QuestManifest.cs) and that the headset has granted " +
                                   "spatial data permission.");
                    yield break;
                }
                waited += Time.unscaledDeltaTime;
                yield return null;
            }

            if (!OVRManager.IsInsightPassthroughInitialized())
            {
                Debug.LogError($"[Teleop] Insight passthrough still not initialised after " +
                               $"{waited:F1}s (pending={OVRManager.IsInsightPassthroughInitPending()}).");
                yield break;
            }

            var go = new GameObject("PassthroughLayer");
            go.transform.SetParent(transform, false);
            _layer = go.AddComponent<OVRPassthroughLayer>();
            _layer.overlayType = OVROverlay.OverlayType.Underlay;

            Mode = Source.Insight;
            ApplyInsight(true);
            Debug.Log($"[Teleop] Insight passthrough up after {waited:F2}s (stereo, full FOV)");
        }

        /// <summary>Insight goes down and comes back around pause/resume, which
        /// for this app is every doff and don (docs section 10). Deciding once
        /// at startup would leave the operator in the void after the first time
        /// they took the headset off. Only runs on the Insight path -- the
        /// camera quad has its own lifecycle inside PassthroughCameraAccess.</summary>
        private void LateUpdate()
        {
            if (Mode != Source.Insight || Head == null) return;

            var up = OVRManager.IsInsightPassthroughInitialized();
            var wantColour = up ? Composited : Fallback;
            if (Head.backgroundColor != wantColour) ApplyInsight(up);
        }

        private void ApplyInsight(bool up)
        {
            if (_layer != null) _layer.enabled = up;
            if (Head == null) return;
            Head.clearFlags = CameraClearFlags.SolidColor;
            Head.backgroundColor = up ? Composited : Fallback;
        }

        // ------------------------------------------------------------------
        // fallback: raw camera frames on a head-parented quad
        // ------------------------------------------------------------------
        private void TryCameraQuad()
        {
            if (!PassthroughCameraAccess.IsSupported)
            {
                Debug.LogError("[Teleop] no Passthrough Camera Access either. The operator " +
                               "will not see the room, the robot, or anyone standing next " +
                               "to them.");
                return;
            }

            // Granted from a previous install persists across `adb install -r`,
            // but request anyway for a genuinely fresh install -- a no-op if
            // it is already held. PassthroughCameraAccess also waits on this
            // permission itself (WaitForPermissionsAndPlay), so this is a
            // belt-and-suspenders speed-up, not a hard dependency.
            if (!OVRPermissionsRequester.IsPermissionGranted(OVRPermissionsRequester.Permission.PassthroughCameraAccess))
                OVRPermissionsRequester.Request(new[] { OVRPermissionsRequester.Permission.PassthroughCameraAccess });

            _backdrop = GameObject.CreatePrimitive(PrimitiveType.Quad);
            _backdrop.name = "PassthroughBackdrop";
            Destroy(_backdrop.GetComponent<Collider>());

            var parent = Head != null ? Head.transform : transform;
            _backdrop.transform.SetParent(parent, false);
            _backdrop.transform.localPosition = new Vector3(0f, 0f, Distance);
            // A default Quad's front face looks down local +Z; parented at
            // +Z in front of the camera, that faces away from it. Turn it to
            // face back toward the parent origin.
            _backdrop.transform.localRotation = Quaternion.Euler(0f, 180f, 0f);
            // Negative X mirrors the displayed image without depending on the
            // shader respecting a UV transform -- see the material comment
            // below for why that route does not work with Sprites/Default.
            _backdrop.transform.localScale = new Vector3(-Size.x, Size.y, 1f);

            // Sprites/Default is unlit and double-sided (no backface culling),
            // so the 180-degree flip above is a belt-and-suspenders correction
            // rather than a hard requirement -- either facing renders. It also
            // does not respect Material.mainTextureScale/Offset the way a
            // standard Unlit shader would (it is built for sprite-atlas UVs),
            // so that is not a working way to flip the image -- flip the
            // quad's own geometry instead, in SizeBackdropToFov, which is
            // shader-agnostic and never silently no-ops.
            var material = new Material(Shader.Find("Sprites/Default"));
            _backdrop.GetComponent<MeshRenderer>().sharedMaterial = material;

            var pcaGo = new GameObject("PassthroughCameraAccess");
            pcaGo.transform.SetParent(transform, false);
            _pca = pcaGo.AddComponent<PassthroughCameraAccess>();
            _pca.CameraPosition = PassthroughCameraAccess.CameraPositionType.Left;
            _pca.RequestedResolution = new Vector2Int(1280, 960);
            _pca.TargetMaterial = material;

            Mode = Source.CameraQuad;
            Debug.Log("[Teleop] passthrough backdrop built, PassthroughCameraAccess attached");
            StartCoroutine(SizeBackdropToFov());
        }

        /// <summary>The quad's initial size (<see cref="Size"/>) is a guess
        /// generous enough to cover the Quest 3's display FOV. The camera's
        /// own FOV is narrower than that, so a guessed size makes the image
        /// look zoomed in -- stretched across more angle than it actually
        /// covers. Once Intrinsics are available (set synchronously inside
        /// Play(), so usually the same frame Play succeeds), resize the quad
        /// to the camera's true angular size at Distance so 1 real metre
        /// reads as 1 apparent metre.
        ///
        /// FocalLength/PrincipalPoint are calibrated against the sensor's own
        /// native pixel grid (SensorResolution), which is not necessarily
        /// CurrentResolution even at a matching aspect ratio -- using
        /// CurrentResolution directly under-measured the true FOV and left
        /// the image looking zoomed even once correctly *positioned*. This
        /// reproduces the SDK's own CalcSensorCropRegion (private, used
        /// internally by ViewportPointToLocalRay) to get the crop in sensor
        /// pixel units first, matching FocalLength's calibration.</summary>
        private IEnumerator SizeBackdropToFov()
        {
            var waited = 0f;
            while (_pca.Intrinsics.FocalLength == Vector2.zero && waited < InitTimeout)
            {
                waited += Time.unscaledDeltaTime;
                yield return null;
            }

            var intrinsics = _pca.Intrinsics;
            if (intrinsics.FocalLength == Vector2.zero)
            {
                Debug.LogWarning($"[Teleop] no camera intrinsics after {waited:F1}s; " +
                                 "backdrop keeps its guessed size and may look zoomed.");
                yield break;
            }

            var sensorResolution = (Vector2)intrinsics.SensorResolution;
            var currentResolution = (Vector2)_pca.CurrentResolution;
            var scaleFactor = new Vector2(
                currentResolution.x / sensorResolution.x,
                currentResolution.y / sensorResolution.y);
            scaleFactor /= Mathf.Max(scaleFactor.x, scaleFactor.y);
            var cropSize = new Vector2(sensorResolution.x * scaleFactor.x, sensorResolution.y * scaleFactor.y);

            var worldSize = new Vector2(
                Distance * cropSize.x / intrinsics.FocalLength.x,
                Distance * cropSize.y / intrinsics.FocalLength.y) * CoverageFactor;
            _backdrop.transform.localScale = new Vector3(-worldSize.x, worldSize.y, 1f);
            Debug.Log($"[Teleop] backdrop resized to {worldSize} (coverage x{CoverageFactor}) " +
                     $"from camera intrinsics (focal={intrinsics.FocalLength}, " +
                     $"sensorRes={sensorResolution}, streamRes={currentResolution}, cropSize={cropSize})");
        }
    }
}

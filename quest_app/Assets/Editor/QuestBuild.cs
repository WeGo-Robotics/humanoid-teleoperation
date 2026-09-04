// Batchmode APK build for the Quest teleoperation client.
//
// Every setting that matters is applied here rather than trusted to
// ProjectSettings/*.asset. Those files are generated YAML that nobody reviews;
// a wrong scripting backend or a missing ARM64 flag is invisible in a diff and
// produces an APK that installs and then fails on device. Applying them from
// code means the build is reproducible from a fresh clone and the settings are
// readable in a pull request.
//
// Usage (see tools/build_quest_apk.ps1, which wraps this):
//
//   Unity.exe -quit -batchmode -nographics -projectPath quest_app \
//             -executeMethod WeGo.Teleop.Editor.QuestBuild.Build \
//             -logFile build.log -- -host 192.168.123.2 -port 8443
//
// Arguments after the bare "--" are ours; Unity ignores them.

using System;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEditor.XR.Management;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.XR.Management;

namespace WeGo.Teleop.Editor
{
    public static class QuestBuild
    {
        private const string ScenePath = "Assets/Scenes/Teleop.unity";
        private const string DefaultApk = "Build/G1Teleop.apk";
        private const string PackageId = "com.wegorobotics.g1teleop";
        private const string OculusLoader = "Unity.XR.Oculus.OculusLoader";

        // Quest 3 runs Android 12L. Meta requires target 32 or newer for
        // anything distributed through their channels, including MDM.
        private const AndroidSdkVersions MinSdk = AndroidSdkVersions.AndroidApiLevel32;
        private const AndroidSdkVersions TargetSdk = AndroidSdkVersions.AndroidApiLevel34;

        [MenuItem("WeGo/Build Quest APK")]
        public static void BuildFromMenu() => Run(new Options());

        public static void Build() => Run(Options.FromCommandLine());

        // ------------------------------------------------------------------
        private static void Run(Options opt)
        {
            try
            {
                Log($"host={opt.Host}:{opt.Port} tls={opt.UseTls} " +
                    $"output={opt.Output} development={opt.Development}");

                SwitchToAndroid();
                ConfigurePlayer(opt);
                ConfigureXr();
                GenerateScene(opt);

                var report = BuildPipeline.BuildPlayer(new BuildPlayerOptions
                {
                    scenes = new[] { ScenePath },
                    locationPathName = opt.Output,
                    target = BuildTarget.Android,
                    targetGroup = BuildTargetGroup.Android,
                    options = opt.Development
                        ? BuildOptions.Development | BuildOptions.AllowDebugging
                        : BuildOptions.None,
                });

                var summary = report.summary;
                if (summary.result != BuildResult.Succeeded)
                {
                    Fail($"build {summary.result} with {summary.totalErrors} error(s)");
                    return;
                }

                Log($"OK  {opt.Output}  {summary.totalSize / (1024 * 1024)} MB  " +
                    $"in {summary.totalTime.TotalSeconds:F0}s");
                if (opt.BatchMode) EditorApplication.Exit(0);
            }
            catch (Exception e)
            {
                Fail($"{e.GetType().Name}: {e.Message}\n{e.StackTrace}");
            }
        }

        private static void SwitchToAndroid()
        {
            if (EditorUserBuildSettings.activeBuildTarget == BuildTarget.Android) return;
            Log("switching active build target to Android");
            if (!EditorUserBuildSettings.SwitchActiveBuildTarget(
                    BuildTargetGroup.Android, BuildTarget.Android))
                throw new Exception(
                    "could not switch to Android. Is Android Build Support " +
                    "installed for this editor version?");
        }

        // ------------------------------------------------------------------
        // player settings
        // ------------------------------------------------------------------
        private static void ConfigurePlayer(Options opt)
        {
            var android = NamedBuildTarget.Android;

            PlayerSettings.companyName = "WeGo Robotics";
            PlayerSettings.productName = "G1 Teleop";
            PlayerSettings.SetApplicationIdentifier(android, PackageId);
            PlayerSettings.bundleVersion = "0.1.0";
            PlayerSettings.Android.bundleVersionCode = opt.VersionCode;

            // IL2CPP + ARM64 is not a preference: Meta rejects 32-bit-only and
            // Mono builds, and the Quest 3 will not run them.
            PlayerSettings.SetScriptingBackend(android, ScriptingImplementation.IL2CPP);
            PlayerSettings.Android.targetArchitectures = AndroidArchitecture.ARM64;

            // .NET Standard 2.1 in 2022.3. ClientWebSocket lives here -- drop to
            // a smaller profile and the transport stops compiling.
            PlayerSettings.SetApiCompatibilityLevel(android,
                                                    ApiCompatibilityLevel.NET_Standard);

            PlayerSettings.Android.minSdkVersion = MinSdk;
            PlayerSettings.Android.targetSdkVersion = TargetSdk;

            // The app must keep running with the headset off the head, or the
            // doff message never leaves. See docs section 10.3.
            PlayerSettings.runInBackground = true;

            // The whole app is a network client; an APK without this permission
            // fails at connect time with an error that looks like a bad address.
            PlayerSettings.Android.forceInternetPermission = true;

            PlayerSettings.colorSpace = ColorSpace.Linear;
            PlayerSettings.SetUseDefaultGraphicsAPIs(BuildTarget.Android, false);
            PlayerSettings.SetGraphicsAPIs(BuildTarget.Android,
                                           new[] { GraphicsDeviceType.Vulkan });
            PlayerSettings.MTRendering = true;
            PlayerSettings.gpuSkinning = true;

            // Cleartext is on because XrLink has no TLS yet (docs 12.7). It is
            // the single reason this build is lab-only, so it is loud here as
            // well as in the manifest.
            if (!opt.UseTls)
                Warn("cleartext ws:// is enabled -- isolated lab network only");

            EnsureLineShaderIncluded();

            AssetDatabase.SaveAssets();
        }

        /// <summary>Keep Sprites/Default in the build.
        ///
        /// TeleopAlignGuide builds its LineRenderer material with
        /// Shader.Find at runtime, and a shader referenced only by Shader.Find
        /// is invisible to the build's dependency walk -- it gets stripped, the
        /// material comes back magenta or blank, and the alignment rings simply
        /// do not appear. That failure looks exactly like a tracking problem
        /// from inside the headset, which is the worst way to spend a hardware
        /// session, so it is pinned here rather than left to chance.</summary>
        private static void EnsureLineShaderIncluded()
        {
            var shader = Shader.Find("Sprites/Default");
            if (shader == null) { Warn("Sprites/Default not found; align rings may not render"); return; }

            var settings = AssetDatabase
                .LoadAllAssetsAtPath("ProjectSettings/GraphicsSettings.asset")
                .FirstOrDefault();
            if (settings == null) { Warn("no GraphicsSettings; cannot pin the line shader"); return; }

            var so = new SerializedObject(settings);
            var list = so.FindProperty("m_AlwaysIncludedShaders");
            if (list == null) { Warn("no m_AlwaysIncludedShaders property"); return; }

            for (var i = 0; i < list.arraySize; i++)
                if (list.GetArrayElementAtIndex(i).objectReferenceValue == shader) return;

            list.InsertArrayElementAtIndex(list.arraySize);
            list.GetArrayElementAtIndex(list.arraySize - 1).objectReferenceValue = shader;
            so.ApplyModifiedProperties();
            Log("pinned Sprites/Default into Always Included Shaders");
        }

        // ------------------------------------------------------------------
        // XR plugin management
        // ------------------------------------------------------------------
        private static void ConfigureXr()
        {
            var perTarget = GetOrCreateXrSettings();
            // Guarded, not unconditional. CreateDefaultManagerSettingsForBuildTarget
            // always news up an XRManagerSettings, names it "Android Providers",
            // makes it the Manager and AddObjectToAsset's it -- so calling it on
            // every build orphans the previous one inside
            // Assets/XR/XRGeneralSettings.asset and never removes it. That file
            // had accumulated thirty dead "Android Providers" sub-assets by
            // 2026-09-04, and every build was also discarding a configured
            // manager to rebuild it from empty.
            if (!perTarget.HasManagerSettingsForBuildTarget(BuildTargetGroup.Android))
                perTarget.CreateDefaultManagerSettingsForBuildTarget(BuildTargetGroup.Android);
            var settings = perTarget.SettingsForBuildTarget(BuildTargetGroup.Android);
            if (settings == null || settings.Manager == null)
                throw new Exception("XR Management produced no manager for Android");

            var already = settings.Manager.activeLoaders
                .Any(l => l != null && l.GetType().FullName == OculusLoader);
            if (already)
            {
                Log("Oculus XR loader already active");
            }
            else
            {
                if (!UnityEditor.XR.Management.Metadata.XRPackageMetadataStore.AssignLoader(
                        settings.Manager, OculusLoader, BuildTargetGroup.Android))
                    throw new Exception(
                        $"could not assign {OculusLoader}. Is com.unity.xr.oculus " +
                        "resolved in Packages/manifest.json?");

                EditorUtility.SetDirty(settings.Manager);
                AssetDatabase.SaveAssets();
                Log("assigned the Oculus XR loader for Android");
            }

            EnsureOculusSettingsRegistered();
        }

        internal const string OculusSettingsAssetPath = "Assets/XR/Settings/OculusSettings.asset";

        /// <summary>The key OculusBuildProcessor reads its settings from --
        /// declared there as OculusBuildProcessor.BuildSettingsKey.</summary>
        private const string OculusSettingsKey = "Unity.XR.Oculus.Settings";

        // Registers OculusSettings.asset as an EditorBuildSettings config
        // object. Without this the built player carries no OculusSettings at
        // all, and that one fact caused both of the failures this app has been
        // chasing since 2026-08-25.
        //
        // The chain. OculusBuildProcessor derives from
        // XRBuildHelper<OculusSettings>, whose OnPreprocessBuild calls
        // SetSettingsForRuntime(SettingsForBuildTargetGroup(..)), and
        // SettingsForBuildTargetGroup is nothing but
        // EditorBuildSettings.TryGetConfigObject("Unity.XR.Oculus.Settings", ..).
        // That key is only ever written by the XR Plug-in Management *window*,
        // which a headless batchmode build never opens; XRPackageMetadataStore
        // .AssignLoader does not write it either. So the key was absent,
        // TryGetConfigObject returned null, and SetSettingsForRuntime ran its
        // unconditional CleanOldSettings<OculusSettings>() -- which strips every
        // OculusSettings out of Preloaded Assets -- and then returned early
        // without putting one back.
        //
        // That is also why the previous fix here logged success and changed
        // nothing on device: it added the asset to Preloaded Assets before
        // BuildPlayer, and the provider's own preprocess hook deleted the entry
        // again on the way in. Ordering, not the wrong asset.
        //
        // What a null OculusSettings then broke, on every launch:
        //
        //   * OculusLoader.Initialize() wraps its entire native handoff in
        //     `if (settings != null)`, so NativeMethods.SetUserDefinedSettings
        //     was never called: the native display was never told the stereo
        //     rendering mode (Multiview), the colour space (Linear), or the
        //     shared depth buffer. Initialize() still returns true, because the
        //     subsystems themselves create fine -- so the app "runs" while the
        //     compositor holds a swapchain nobody configured. That is the
        //     intermittent black screen, and the Unity splash goes missing with
        //     it because the splash renders through the same display subsystem.
        //
        //   * OVRManager.InitOVRManager() (OVRManager.cs:2408-2414) null-checks
        //     the loader but not GetSettings(), then reads
        //     oculusSettings.DepthSubmission -- NullReferenceException, thrown
        //     before OVRManagerinitialized is ever set. That is what has kept
        //     Insight Passthrough dead and pushed TeleopPassthrough onto raw
        //     PassthroughCameraAccess instead.
        //
        // Registering the key lets Unity's own build processor do the Preloaded
        // Assets work, every build, at the right point in the pipeline.
        // OculusSettingsPreloadGuard below then checks that it actually did.
        private static void EnsureOculusSettingsRegistered()
        {
            var oculusSettings =
                AssetDatabase.LoadAssetAtPath<UnityEngine.Object>(OculusSettingsAssetPath)
                ?? CreateOculusSettings();
            if (oculusSettings == null) return;

            if (EditorBuildSettings.TryGetConfigObject(OculusSettingsKey,
                                                       out UnityEngine.Object current)
                && current == oculusSettings)
            {
                Log($"OculusSettings already registered under '{OculusSettingsKey}'");
            }
            else
            {
                EditorBuildSettings.AddConfigObject(OculusSettingsKey, oculusSettings, true);
                Log($"registered {OculusSettingsAssetPath} under '{OculusSettingsKey}'");
            }

            EnsureQuestTargets(oculusSettings);
        }

        /// <summary>Assets/XR is gitignored, so a fresh clone has no
        /// OculusSettings asset at all -- normally it is created by the XR
        /// Plug-in Management window, which a batchmode build never opens.
        /// Without this, the first build on a new machine or in CI would trip
        /// OculusSettingsPreloadGuard with nothing the operator could do about
        /// it. Created by reflection so this file keeps its independence from
        /// the Meta and Oculus assemblies.</summary>
        private static UnityEngine.Object CreateOculusSettings()
        {
            var type = Type.GetType("Unity.XR.Oculus.OculusSettings, Unity.XR.Oculus");
            if (type == null)
            {
                Warn("Unity.XR.Oculus.OculusSettings is not loaded, so no settings asset " +
                     "can be created. Is com.unity.xr.oculus resolved?");
                return null;
            }

            var created = ScriptableObject.CreateInstance(type);
            Directory.CreateDirectory(Path.GetDirectoryName(OculusSettingsAssetPath));
            AssetDatabase.CreateAsset(created, OculusSettingsAssetPath);
            AssetDatabase.SaveAssets();
            Log($"created {OculusSettingsAssetPath} (none existed; Assets/XR is gitignored)");
            return created;
        }

        /// <summary>The Target* flags in OculusSettings are the plugin's record
        /// of which headsets the build is for. They shipped as Quest 2 only,
        /// which matches neither the com.oculus.supportedDevices this app writes
        /// in QuestManifest.cs nor the Quest 3 on the bench. Set from code for
        /// the same reason everything else here is: a wrong checkbox in
        /// generated YAML is invisible in review.</summary>
        private static void EnsureQuestTargets(UnityEngine.Object oculusSettings)
        {
            var so = new SerializedObject(oculusSettings);
            var changed = false;
            foreach (var flag in new[] { "TargetQuest2", "TargetQuestPro",
                                         "TargetQuest3", "TargetQuest3S" })
            {
                var prop = so.FindProperty(flag);
                if (prop == null) { Warn($"OculusSettings has no {flag}"); continue; }
                if (prop.boolValue) continue;
                prop.boolValue = true;
                changed = true;
            }
            if (!changed) return;

            so.ApplyModifiedPropertiesWithoutUndo();
            EditorUtility.SetDirty(oculusSettings);
            AssetDatabase.SaveAssets();
            Log("enabled Quest 2 / Pro / 3 / 3S in OculusSettings, matching the " +
                "supportedDevices QuestManifest writes");
        }

        private static XRGeneralSettingsPerBuildTarget GetOrCreateXrSettings()
        {
            if (EditorBuildSettings.TryGetConfigObject(
                    XRGeneralSettings.k_SettingsKey,
                    out XRGeneralSettingsPerBuildTarget existing) && existing != null)
                return existing;

            Directory.CreateDirectory("Assets/XR");
            var created = ScriptableObject.CreateInstance<XRGeneralSettingsPerBuildTarget>();
            AssetDatabase.CreateAsset(created, "Assets/XR/XRGeneralSettings.asset");
            AssetDatabase.SaveAssets();
            EditorBuildSettings.AddConfigObject(XRGeneralSettings.k_SettingsKey,
                                                created, true);
            Log("created XR general settings");
            return created;
        }

        // ------------------------------------------------------------------
        // scene
        // ------------------------------------------------------------------
        /// <summary>One empty object with TeleopBootstrap on it. Everything
        /// else is built at runtime -- see the header of TeleopBootstrap for
        /// why the scene is not a committed .unity file full of GUIDs.</summary>
        private static void GenerateScene(Options opt)
        {
            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene,
                                                    NewSceneMode.Single);
            var go = new GameObject("Teleop");
            var boot = go.AddComponent<TeleopBootstrap>();
            boot.HostAddress = opt.Host;
            boot.Port = opt.Port;
            boot.UseTls = opt.UseTls;

            Directory.CreateDirectory("Assets/Scenes");
            if (!EditorSceneManager.SaveScene(scene, ScenePath))
                throw new Exception($"could not save {ScenePath}");
            Log($"generated {ScenePath} for {boot.HostAddress}:{boot.Port}");
        }

        // ------------------------------------------------------------------
        private sealed class Options
        {
            public string Host = "192.168.123.2";
            public int Port = 8443;
            public bool UseTls = false;
            public string Output = DefaultApk;
            public bool Development;
            public int VersionCode = 1;
            public bool BatchMode = Application.isBatchMode;

            public static Options FromCommandLine()
            {
                var o = new Options();
                var args = Environment.GetCommandLineArgs();
                for (var i = 0; i < args.Length; i++)
                {
                    switch (args[i])
                    {
                        case "-host": o.Host = Next(args, i); break;
                        case "-port": o.Port = int.Parse(Next(args, i)); break;
                        case "-output": o.Output = Next(args, i); break;
                        case "-versionCode": o.VersionCode = int.Parse(Next(args, i)); break;
                        case "-tls": o.UseTls = true; break;
                        case "-development": o.Development = true; break;
                    }
                }
                o.Output = Path.GetFullPath(o.Output);
                Directory.CreateDirectory(Path.GetDirectoryName(o.Output) ?? ".");
                return o;
            }

            private static string Next(string[] args, int i)
            {
                if (i + 1 >= args.Length)
                    throw new ArgumentException($"{args[i]} needs a value");
                return args[i + 1];
            }
        }

        private static void Log(string m) => Debug.Log($"[QuestBuild] {m}");
        private static void Warn(string m) => Debug.LogWarning($"[QuestBuild] {m}");

        private static void Fail(string m)
        {
            Debug.LogError($"[QuestBuild] {m}");
            // Batchmode must exit non-zero, or CI and the wrapper script read a
            // failed build as a successful one.
            if (Application.isBatchMode) EditorApplication.Exit(1);
        }
    }

    /// <summary>Fails the build if OculusSettings did not reach Preloaded Assets.
    ///
    /// callbackOrder 100 puts this after XRBuildHelper's own preprocess hook,
    /// which runs at 0 and is the thing that both strips and re-adds the entry.
    /// Whatever is in Preloaded Assets once this runs is what the player ships
    /// with.
    ///
    /// A hard failure rather than a warning, on purpose. The symptom of getting
    /// this wrong is not a build error: it is an APK that installs, opens to a
    /// black void with no Unity splash, and throws a NullReferenceException
    /// somewhere that looks unrelated -- see
    /// QuestBuild.EnsureOculusSettingsRegistered for the full chain. That cost
    /// eleven days. A red build costs a minute.</summary>
    internal class OculusSettingsPreloadGuard : IPreprocessBuildWithReport
    {
        public int callbackOrder => 100;

        public void OnPreprocessBuild(BuildReport report)
        {
            if (report.summary.platformGroup != BuildTargetGroup.Android) return;

            var preloaded = PlayerSettings.GetPreloadedAssets() ?? new UnityEngine.Object[0];
            var named = preloaded.Where(a => a != null).Select(a => a.name).ToArray();
            if (preloaded.Any(a => a != null && a.GetType().Name == "OculusSettings"))
            {
                Debug.Log("[QuestBuild] OculusSettings is in Preloaded Assets " +
                          $"({string.Join(", ", named)})");
                return;
            }

            throw new BuildFailedException(
                "OculusSettings is not in Preloaded Assets, so OculusSettings.s_Settings " +
                "would be null in the player: the Oculus XR plugin would start without " +
                "ever calling SetUserDefinedSettings (black screen, no Unity splash) and " +
                "OVRManager.InitOVRManager() would throw a NullReferenceException (no " +
                "Insight Passthrough). Preloaded Assets currently holds: " +
                $"{(named.Length == 0 ? "nothing" : string.Join(", ", named))}. Check that " +
                "QuestBuild.EnsureOculusSettingsRegistered registered " +
                $"{QuestBuild.OculusSettingsAssetPath} under \"Unity.XR.Oculus.Settings\".");
        }
    }
}

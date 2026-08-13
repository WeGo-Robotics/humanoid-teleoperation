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

            AssetDatabase.SaveAssets();
        }

        // ------------------------------------------------------------------
        // XR plugin management
        // ------------------------------------------------------------------
        private static void ConfigureXr()
        {
            var perTarget = GetOrCreateXrSettings();
            perTarget.CreateDefaultManagerSettingsForBuildTarget(BuildTargetGroup.Android);
            var settings = perTarget.SettingsForBuildTarget(BuildTargetGroup.Android);
            if (settings == null || settings.Manager == null)
                throw new Exception("XR Management produced no manager for Android");

            var already = settings.Manager.activeLoaders
                .Any(l => l != null && l.GetType().FullName == OculusLoader);
            if (already)
            {
                Log("Oculus XR loader already active");
                return;
            }

            if (!UnityEditor.XR.Management.Metadata.XRPackageMetadataStore.AssignLoader(
                    settings.Manager, OculusLoader, BuildTargetGroup.Android))
                throw new Exception(
                    $"could not assign {OculusLoader}. Is com.unity.xr.oculus " +
                    "resolved in Packages/manifest.json?");

            EditorUtility.SetDirty(settings.Manager);
            AssetDatabase.SaveAssets();
            Log("assigned the Oculus XR loader for Android");
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
}

// Patches the generated Android manifest instead of replacing it.
//
// Dropping a hand-written Assets/Plugins/Android/AndroidManifest.xml into the
// project overrides Unity's *and* the Meta SDK's, so every entry either of them
// injects has to be reproduced by hand and re-checked on every package upgrade.
// A missing one usually shows up as an app that installs and then will not
// launch. Patching after generation adds the four things this app needs and
// inherits everything else.
//
// Every edit here is idempotent: the build runs this on a freshly generated
// manifest each time, but re-running it on a patched one changes nothing.

using System.IO;
using System.Xml;
using UnityEditor.Android;
using UnityEngine;

namespace WeGo.Teleop.Editor
{
    public class QuestManifest : IPostGenerateGradleAndroidProject
    {
        // Runs after the Meta SDK's own manifest work, so our entries win where
        // they overlap and are additive where they do not.
        public int callbackOrder => 10000;

        private const string AndroidNs = "http://schemas.android.com/apk/res/android";

        public void OnPostGenerateGradleAndroidProject(string path)
        {
            // `path` is the unityLibrary module. In Unity 2020+ the launcher
            // activity is declared in the sibling launcher module, so both are
            // candidates and neither is guaranteed to hold everything.
            var root = Directory.GetParent(path)?.FullName;
            Patch(Path.Combine(path, "src/main/AndroidManifest.xml"));
            if (root != null)
                Patch(Path.Combine(root, "launcher/src/main/AndroidManifest.xml"));
        }

        private static void Patch(string file)
        {
            if (!File.Exists(file)) return;

            var doc = new XmlDocument();
            doc.Load(file);
            var manifest = doc.DocumentElement;
            if (manifest == null || manifest.Name != "manifest") return;

            var changed = false;
            changed |= EnsureInternetPermission(doc, manifest);
            changed |= EnsureFeature(doc, manifest, "android.hardware.vr.headtracking",
                                     required: true, version: "1");

            // Alignment happens in passthrough so the operator can see the real
            // robot they are matching against. Normally the Meta SDK emits this
            // from OVRProjectConfig while generating its own manifest, which we
            // do not use -- so without this line passthrough silently fails to
            // start and the align gate runs against a black screen.
            //
            // required=false so the APK still installs on a headset without
            // passthrough; the runtime request degrades instead of the install
            // being refused.
            changed |= EnsureFeature(doc, manifest, "com.oculus.feature.PASSTHROUGH",
                                     required: false);

            var application = manifest.SelectSingleNode("application") as XmlElement;
            if (application != null)
            {
                // XrLink has no TLS yet (docs 12.7). Without this the OS may
                // block the plain ws:// connection, and the failure looks like
                // a network problem rather than a policy one.
                changed |= SetAttribute(application, "usesCleartextTraffic", "true");
                changed |= EnsureMeta(doc, application, "com.oculus.supportedDevices",
                                      "quest2|questpro|quest3");
                changed |= PatchActivities(doc, application);
            }

            if (!changed) return;
            doc.Save(file);
            Debug.Log($"[QuestManifest] patched {file}");
        }

        private static bool PatchActivities(XmlDocument doc, XmlElement application)
        {
            var changed = false;
            foreach (XmlNode node in application.SelectNodes("activity"))
            {
                if (!(node is XmlElement activity)) continue;
                if (activity.SelectSingleNode(
                        "intent-filter/category[@android:name='android.intent.category.LAUNCHER']",
                        NsManager(doc)) == null)
                    continue;

                // Focus-aware: without it the OS pauses the app whenever the
                // system menu takes focus, which stops tracking mid-session and
                // reads on the host as a link that went stale for no reason.
                changed |= EnsureMeta(doc, activity, "com.oculus.vr.focusaware", "true");

                foreach (XmlNode filterNode in activity.SelectNodes("intent-filter"))
                {
                    if (!(filterNode is XmlElement filter)) continue;
                    if (filter.SelectSingleNode(
                            "category[@android:name='android.intent.category.LAUNCHER']",
                            NsManager(doc)) == null)
                        continue;
                    // Without the VR category the app is filed under 2D apps in
                    // the Quest launcher and never appears where the operator
                    // looks for it.
                    changed |= EnsureCategory(doc, filter, "com.oculus.intent.category.VR");
                }
            }
            return changed;
        }

        // ------------------------------------------------------------------
        private static bool EnsureInternetPermission(XmlDocument doc, XmlElement manifest)
        {
            if (manifest.SelectSingleNode(
                    "uses-permission[@android:name='android.permission.INTERNET']",
                    NsManager(doc)) != null)
                return false;
            var e = doc.CreateElement("uses-permission");
            e.SetAttribute("name", AndroidNs, "android.permission.INTERNET");
            manifest.AppendChild(e);
            return true;
        }

        private static bool EnsureFeature(XmlDocument doc, XmlElement manifest,
                                          string feature, bool required,
                                          string version = null)
        {
            if (manifest.SelectSingleNode($"uses-feature[@android:name='{feature}']",
                                          NsManager(doc)) != null)
                return false;
            var e = doc.CreateElement("uses-feature");
            e.SetAttribute("name", AndroidNs, feature);
            if (version != null) e.SetAttribute("version", AndroidNs, version);
            e.SetAttribute("required", AndroidNs, required ? "true" : "false");
            manifest.AppendChild(e);
            return true;
        }

        private static bool EnsureMeta(XmlDocument doc, XmlElement parent,
                                       string name, string value)
        {
            var existing = parent.SelectSingleNode(
                $"meta-data[@android:name='{name}']", NsManager(doc)) as XmlElement;
            if (existing != null)
                return SetAttribute(existing, "value", value);

            var e = doc.CreateElement("meta-data");
            e.SetAttribute("name", AndroidNs, name);
            e.SetAttribute("value", AndroidNs, value);
            parent.AppendChild(e);
            return true;
        }

        private static bool EnsureCategory(XmlDocument doc, XmlElement filter, string name)
        {
            if (filter.SelectSingleNode($"category[@android:name='{name}']",
                                        NsManager(doc)) != null)
                return false;
            var e = doc.CreateElement("category");
            e.SetAttribute("name", AndroidNs, name);
            filter.AppendChild(e);
            return true;
        }

        private static bool SetAttribute(XmlElement element, string name, string value)
        {
            if (element.GetAttribute(name, AndroidNs) == value) return false;
            element.SetAttribute(name, AndroidNs, value);
            return true;
        }

        private static XmlNamespaceManager NsManager(XmlDocument doc)
        {
            var ns = new XmlNamespaceManager(doc.NameTable);
            ns.AddNamespace("android", AndroidNs);
            return ns;
        }
    }
}

// The three poses the UI needs, behind an interface so a desktop build can
// supply them.
//
// On device TeleopSession reads these straight off the XR subsystem and this
// interface is never instantiated. It exists so that the preview build can run
// the real TeleopHud and TeleopAlignGuide against a mouse instead of a headset:
// the display code cannot tell the difference, which is the only way a preview
// is worth anything. A preview that reimplemented the HUD would be a drawing of
// the app, not the app.

using UnityEngine;

namespace WeGo.Teleop
{
    public interface ITeleopPoses
    {
        /// <summary>Centre-eye, floor-relative -- same convention as
        /// OVRManager.TrackingOrigin.FloorLevel.</summary>
        Vector3 Head { get; }
        Quaternion HeadRotation { get; }
        Vector3 LeftWrist { get; }
        Vector3 RightWrist { get; }
        Quaternion LeftWristRotation { get; }
        Quaternion RightWristRotation { get; }
    }
}

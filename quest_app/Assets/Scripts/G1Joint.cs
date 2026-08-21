// Marks a revolute joint on the baked G1 prefab.
//
// The importer writes one of these onto every revolute link so the runtime can
// pose the model without re-reading the URDF: the joint's name, its axis in
// the child's own frame, and the rest rotation the URDF origin gave it.
//
// Setting Angle rotates about the axis relative to that rest rotation, which
// is what a URDF joint value means. Composing onto whatever rotation happens
// to be there instead would drift a little further from the robot's real pose
// on every frame.

using UnityEngine;

namespace WeGo.Teleop
{
    public class G1Joint : MonoBehaviour
    {
        public string JointName;
        public Vector3 LocalAxis = Vector3.forward;
        public Quaternion RestRotation = Quaternion.identity;

        [SerializeField] private float _angle;

        /// <summary>Radians, matching the URDF and the host's joint vector.</summary>
        public float Angle
        {
            get => _angle;
            set
            {
                _angle = value;
                transform.localRotation =
                    RestRotation * Quaternion.AngleAxis(value * Mathf.Rad2Deg, LocalAxis);
            }
        }
    }
}

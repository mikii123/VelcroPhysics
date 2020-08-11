using FixedMath.Net;

namespace VelcroPhysics.Shared
{
    /// <summary>
    /// A transform contains translation and rotation. It is used to represent
    /// the position and orientation of rigid frames.
    /// </summary>
    public struct Transform
    {
        public Vec2 p;
        public Rot q;

        /// <summary>
        /// Initialize using a position vector and a rotation matrix.
        /// </summary>
        /// <param name="position">The position.</param>
        /// <param name="rotation">The r.</param>
        public Transform(ref Vec2 position, ref Rot rotation)
        {
            p = position;
            q = rotation;
        }

        /// <summary>
        /// Set this to the identity transform.
        /// </summary>
        public void SetIdentity()
        {
            p = Vec2.Zero;
            q.SetIdentity();
        }

        /// <summary>
        /// Set this based on the position and angle.
        /// </summary>
        /// <param name="position">The position.</param>
        /// <param name="angle">The angle.</param>
        public void Set(Vec2 position, Fix64 angle)
        {
            p = position;
            q.Set(angle);
        }
    }
}
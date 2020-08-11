using FixedMath.Net;

namespace VelcroPhysics.Shared
{
    /// <summary>
    /// Rotation
    /// </summary>
    public struct Rot
    {
        /// Sine and cosine
        public Fix64 s,
                     c;

        /// <summary>
        /// Initialize from an angle in radians
        /// </summary>
        /// <param name="angle">Angle in radians</param>
        public Rot(Fix64 angle)
        {
            // TODO_ERIN optimize
            s = Fix64.Sin(angle);
            c = Fix64.Cos(angle);
        }

        /// <summary>
        /// Set using an angle in radians.
        /// </summary>
        /// <param name="angle"></param>
        public void Set(Fix64 angle)
        {
            //Velcro: Optimization
            if (angle == Fix64.Zero)
            {
                s = Fix64.Zero;
                c = Fix64.One;
            }
            else
            {
                // TODO_ERIN optimize
                s = Fix64.Sin(angle);
                c = Fix64.Cos(angle);
            }
        }

        /// <summary>
        /// Set to the identity rotation
        /// </summary>
        public void SetIdentity()
        {
            s = Fix64.Zero;
            c = Fix64.One;
        }

        /// <summary>
        /// Get the angle in radians
        /// </summary>
        public Fix64 GetAngle()
        {
            return Fix64.Atan2(s, c);
        }

        /// <summary>
        /// Get the x-axis
        /// </summary>
        public Vec2 GetXAxis()
        {
            return new Vec2(c, s);
        }

        /// <summary>
        /// Get the y-axis
        /// </summary>
        public Vec2 GetYAxis()
        {
            return new Vec2(-s, c);
        }
    }
}
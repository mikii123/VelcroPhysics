using FixedMath.Net;

namespace VelcroPhysics.Collision.Narrowphase
{
    /// <summary>
    /// This structure is used to keep track of the best separating axis.
    /// </summary>
    public struct EPAxis
    {
        public int Index;
        public Fix64 Separation;
        public EPAxisType Type;
    }
}
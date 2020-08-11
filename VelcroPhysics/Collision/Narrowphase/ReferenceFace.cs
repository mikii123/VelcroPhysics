using FixedMath.Net;

namespace VelcroPhysics.Collision.Narrowphase
{
    /// <summary>
    /// Reference face used for clipping
    /// </summary>
    public struct ReferenceFace
    {
        public int i1, i2;

        public Vec2 v1, v2;

        public Vec2 Normal;

        public Vec2 SideNormal1;
        public Fix64 SideOffset1;

        public Vec2 SideNormal2;
        public Fix64 SideOffset2;
    }
}
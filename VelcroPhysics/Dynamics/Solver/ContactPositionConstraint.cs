using FixedMath.Net;
using VelcroPhysics.Collision.Narrowphase;

namespace VelcroPhysics.Dynamics.Solver
{
    public sealed class ContactPositionConstraint
    {
        public int IndexA;
        public int IndexB;
        public Fix64 InvIA, InvIB;
        public Fix64 InvMassA, InvMassB;
        public Vec2 LocalCenterA, LocalCenterB;
        public Vec2 LocalNormal;
        public Vec2 LocalPoint;
        public Vec2[] LocalPoints = new Vec2[Settings.MaxManifoldPoints];
        public int PointCount;
        public Fix64 RadiusA, RadiusB;
        public ManifoldType Type;
    }
}
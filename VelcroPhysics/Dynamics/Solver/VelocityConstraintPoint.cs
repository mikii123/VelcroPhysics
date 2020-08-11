using FixedMath.Net;
using Microsoft.Xna.Framework;

namespace VelcroPhysics.Dynamics.Solver
{
    public sealed class VelocityConstraintPoint
    {
        public Fix64 NormalImpulse;
        public Fix64 NormalMass;
        public Vec2 rA;
        public Vec2 rB;
        public Fix64 TangentImpulse;
        public Fix64 TangentMass;
        public Fix64 VelocityBias;
    }
}
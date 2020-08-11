using System.Collections.Generic;
using FixedMath.Net;
using VelcroPhysics.Dynamics;
using VelcroPhysics.Extensions.PhysicsLogics.PhysicsLogicBase;
using VelcroPhysics.Shared;

namespace VelcroPhysics.Extensions.PhysicsLogics.Explosion
{
    /// <summary>
    /// Creates a simple explosion that ignores other bodies hiding behind static bodies.
    /// </summary>
    public sealed class SimpleExplosion : PhysicsLogic
    {
        public SimpleExplosion(World world)
            : base(world, PhysicsLogicType.Explosion)
        {
            Power = Fix64.One; //linear
        }

        /// <summary>
        /// This is the power used in the power function. A value of 1 means the force
        /// applied to bodies in the explosion is linear. A value of 2 means it is exponential.
        /// </summary>
        public Fix64 Power { get; set; }

        /// <summary>
        /// Activate the explosion at the specified position.
        /// </summary>
        /// <param name="pos">The position (center) of the explosion.</param>
        /// <param name="radius">The radius of the explosion.</param>
        /// <param name="force">The force applied</param>
        /// <param name="maxForce">A maximum amount of force. When force gets over this value, it will be equal to maxForce</param>
        /// <returns>A list of bodies and the amount of force that was applied to them.</returns>
        public Dictionary<Body, Vec2> Activate(Vec2 pos, Fix64 radius, Fix64 force, Fix64 maxForce)
        {
            HashSet<Body> affectedBodies = new HashSet<Body>();

            AABB aabb;
            aabb.LowerBound = pos - new Vec2(radius);
            aabb.UpperBound = pos + new Vec2(radius);

            // Query the world for bodies within the radius.
            World.QueryAABB(fixture =>
            {
                if (Vec2.Distance(fixture.Body.Position, pos) <= radius)
                {
                    if (!affectedBodies.Contains(fixture.Body))
                        affectedBodies.Add(fixture.Body);
                }

                return true;
            }, ref aabb);

            return ApplyImpulse(pos, radius, force, maxForce, affectedBodies);
        }

        private Dictionary<Body, Vec2> ApplyImpulse(Vec2 pos, Fix64 radius, Fix64 force, Fix64 maxForce, HashSet<Body> overlappingBodies)
        {
            Dictionary<Body, Vec2> forces = new Dictionary<Body, Vec2>(overlappingBodies.Count);

            foreach (Body overlappingBody in overlappingBodies)
            {
                if (IsActiveOn(overlappingBody))
                {
                    Fix64 distance = Vec2.Distance(pos, overlappingBody.Position);
                    Fix64 forcePercent = GetPercent(distance, radius);

                    Vec2 forceVector = pos - overlappingBody.Position;
                    forceVector *= Fix64.One / Fix64.Sqrt(forceVector.X * forceVector.X + forceVector.Y * forceVector.Y);
                    forceVector *= MathHelper.Min(force * forcePercent, maxForce);
                    forceVector *= -Fix64.One;

                    overlappingBody.ApplyLinearImpulse(forceVector);
                    forces.Add(overlappingBody, forceVector);
                }
            }

            return forces;
        }

        private Fix64 GetPercent(Fix64 distance, Fix64 radius)
        {
            //(1-(distance/radius))^power-1
            Fix64 percent = Fix64.Pow(Fix64.One - ((distance - radius) / radius), Power) - Fix64.One;
            
            return MathHelper.Clamp(percent, Fix64.Zero, Fix64.One);
        }
    }
}
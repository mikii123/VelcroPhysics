using FixedMath.Net;
using VelcroPhysics.Collision.Filtering;
using VelcroPhysics.Collision.Shapes;

namespace VelcroPhysics.Templates
{
    public class FixtureTemplate : IDefaults
    {
        public FixtureTemplate()
        {
            SetDefaults();
        }

        /// <summary>
        /// The density, usually in kg/m^2.
        /// </summary>
        public Fix64 Density { get; set; }

        /// <summary>
        /// Contact filtering data.
        /// </summary>
        public Filter Filter { get; set; }

        /// <summary>
        /// The friction coefficient, usually in the range [0,1].
        /// </summary>
        public Fix64 Friction { get; set; }

        /// <summary>
        /// A sensor shape collects contact information but never generates a collision response.
        /// </summary>
        public bool IsSensor { get; set; }

        /// <summary>
        /// The restitution (elasticity) usually in the range [0,1].
        /// </summary>
        public Fix64 Restitution { get; set; }

        /// <summary>
        /// The shape, this must be set. The shape will be cloned, so you can create the shape on the stack.
        /// </summary>
        public Shape Shape { get; set; }

        /// <summary>
        /// Use this to store application specific fixture data.
        /// </summary>
        public object UserData { get; set; }

        public void SetDefaults()
        {
            Friction = (Fix64.One / Fix64.Five);
        }
    }
}
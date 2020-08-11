using FixedMath.Net;
using VelcroPhysics.Collision.Shapes;

namespace VelcroPhysics.Templates.Shapes
{
    public class ShapeTemplate
    {
        public ShapeTemplate(ShapeType type)
        {
            ShapeType = type;
        }

        /// <summary>
        /// Gets or sets the density.
        /// </summary>
        public Fix64 Density { get; set; }

        /// <summary>
        /// Radius of the Shape
        /// </summary>
        public Fix64 Radius { get; set; }

        /// <summary>
        /// Get the type of this shape.
        /// </summary>
        public ShapeType ShapeType { get; }
    }
}
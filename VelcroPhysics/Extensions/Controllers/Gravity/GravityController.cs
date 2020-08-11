using System.Collections.Generic;
using FixedMath.Net;
using VelcroPhysics.Dynamics;
using VelcroPhysics.Extensions.Controllers.ControllerBase;

namespace VelcroPhysics.Extensions.Controllers.Gravity
{
    public class GravityController : Controller
    {
        public GravityController(Fix64 strength)
            : base(ControllerType.GravityController)
        {
            Strength = strength;
            MaxRadius = Fix64.MaxValue;
            GravityType = GravityType.DistanceSquared;
            Points = new List<Vec2>();
            Bodies = new List<Body>();
        }

        public GravityController(Fix64 strength, Fix64 maxRadius, Fix64 minRadius)
            : base(ControllerType.GravityController)
        {
            MinRadius = minRadius;
            MaxRadius = maxRadius;
            Strength = strength;
            GravityType = GravityType.DistanceSquared;
            Points = new List<Vec2>();
            Bodies = new List<Body>();
        }

        public Fix64 MinRadius { get; set; }
        public Fix64 MaxRadius { get; set; }
        public Fix64 Strength { get; set; }
        public GravityType GravityType { get; set; }
        public List<Body> Bodies { get; set; }
        public List<Vec2> Points { get; set; }

        public override void Update(Fix64 dt)
        {
            Vec2 f = Vec2.Zero;

            foreach (Body worldBody in World.BodyList)
            {
                if (!IsActiveOn(worldBody))
                    continue;

                foreach (Body controllerBody in Bodies)
                {
                    if (worldBody == controllerBody || (worldBody.IsStatic && controllerBody.IsStatic) || !controllerBody.Enabled)
                        continue;

                    Vec2 d = controllerBody.Position - worldBody.Position;
                    Fix64 r2 = d.MagnitudeSquared;

                    if (r2 <= Fix64.Epsilon || r2 > MaxRadius * MaxRadius || r2 < MinRadius * MinRadius)
                        continue;

                    switch (GravityType)
                    {
                        case GravityType.DistanceSquared:
                            f = Strength / r2 * worldBody.Mass * controllerBody.Mass * d;
                            break;
                        case GravityType.Linear:
                            f = Strength / Fix64.Sqrt(r2) * worldBody.Mass * controllerBody.Mass * d;
                            break;
                    }

                    worldBody.ApplyForce(ref f);
                }

                foreach (Vec2 point in Points)
                {
                    Vec2 d = point - worldBody.Position;
                    Fix64 r2 = d.MagnitudeSquared;

                    if (r2 <= Fix64.Epsilon || r2 > MaxRadius * MaxRadius || r2 < MinRadius * MinRadius)
                        continue;

                    switch (GravityType)
                    {
                        case GravityType.DistanceSquared:
                            f = Strength / r2 * worldBody.Mass * d;
                            break;
                        case GravityType.Linear:
                            f = Strength / Fix64.Sqrt(r2) * worldBody.Mass * d;
                            break;
                    }

                    worldBody.ApplyForce(ref f);
                }
            }
        }

        public void AddBody(Body body)
        {
            Bodies.Add(body);
        }

        public void AddPoint(Vec2 point)
        {
            Points.Add(point);
        }
    }
}
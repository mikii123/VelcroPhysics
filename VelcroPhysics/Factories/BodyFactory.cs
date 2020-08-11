using System;
using System.Collections.Generic;
using FixedMath.Net;
using VelcroPhysics.Collision.Shapes;
using VelcroPhysics.Dynamics;
using VelcroPhysics.Shared;
using VelcroPhysics.Templates;
using VelcroPhysics.Tools.Triangulation.TriangulationBase;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Factories
{
    public static class BodyFactory
    {
        public static Body CreateBody(World world, Vec2 position = new Vec2(), Fix64 rotation = new Fix64(), BodyType bodyType = BodyType.Static, object userData = null)
        {
	        BodyTemplate template = new BodyTemplate { Position = position, Angle = rotation, Type = bodyType, UserData = userData };

	        return world.CreateBody(template);
        }

        public static Body CreateEdge(World world, Vec2 start, Vec2 end, object userData = null)
        {
            Body body = CreateBody(world);
            body.UserData = userData;

            FixtureFactory.AttachEdge(start, end, body);
            return body;
        }

        public static Body CreateChainShape(World world, Vertices vertices, Vec2 position = new Vec2(), object userData = null)
        {
            Body body = CreateBody(world, position);
            body.UserData = userData;

            FixtureFactory.AttachChainShape(vertices, body);
            return body;
        }

        public static Body CreateLoopShape(World world, Vertices vertices, Vec2 position = new Vec2(), object userData = null)
        {
            Body body = CreateBody(world, position);
            body.UserData = userData;

            FixtureFactory.AttachLoopShape(vertices, body);
            return body;
        }

        public static Body CreateRectangle(World world, Fix64 width, Fix64 height, Fix64 density, Vec2 position = new Vec2(), Fix64 rotation = new Fix64(), BodyType bodyType = BodyType.Static, object userData = null)
        {
            if (width <= Fix64.Zero)
                throw new ArgumentOutOfRangeException(nameof(width), "Width must be more than 0 meters");

            if (height <= Fix64.Zero)
                throw new ArgumentOutOfRangeException(nameof(height), "Height must be more than 0 meters");

            Body body = CreateBody(world, position, rotation, bodyType, userData);

            Vertices rectangleVertices = PolygonUtils.CreateRectangle(width / Fix64.Two, height / Fix64.Two);
            FixtureFactory.AttachPolygon(rectangleVertices, density, body);

            return body;
        }

        public static Body CreateCircle(World world, Fix64 radius, Fix64 density, Vec2 position = new Vec2(), BodyType bodyType = BodyType.Static, object userData = null)
        {
            Body body = CreateBody(world, position, Fix64.Zero, bodyType, userData);
            FixtureFactory.AttachCircle(radius, density, body);
            return body;
        }

        public static Body CreateEllipse(World world, Fix64 xRadius, Fix64 yRadius, int edges, Fix64 density, Vec2 position = new Vec2(), Fix64 rotation = new Fix64(), BodyType bodyType = BodyType.Static, object userData = null)
        {
            Body body = CreateBody(world, position, rotation, bodyType, userData);
            FixtureFactory.AttachEllipse(xRadius, yRadius, edges, density, body);
            return body;
        }

        public static Body CreatePolygon(World world, Vertices vertices, Fix64 density, Vec2 position = new Vec2(), Fix64 rotation = new Fix64(), BodyType bodyType = BodyType.Static, object userData = null)
        {
            Body body = CreateBody(world, position, rotation, bodyType, userData);
            FixtureFactory.AttachPolygon(vertices, density, body);
            return body;
        }

        public static Body CreateCompoundPolygon(World world, List<Vertices> list, Fix64 density, Vec2 position = new Vec2(), Fix64 rotation = new Fix64(), BodyType bodyType = BodyType.Static, object userData = null)
        {
            //We create a single body
            Body body = CreateBody(world, position, rotation, bodyType, userData);
            FixtureFactory.AttachCompoundPolygon(list, density, body);
            return body;
        }

        public static Body CreateGear(World world, Fix64 radius, int numberOfTeeth, Fix64 tipPercentage, Fix64 toothHeight, Fix64 density, Vec2 position = new Vec2(), Fix64 rotation = new Fix64(), BodyType bodyType = BodyType.Static, object userData = null)
        {
            Vertices gearPolygon = PolygonUtils.CreateGear(radius, numberOfTeeth, tipPercentage, toothHeight);

            //Gears can in some cases be convex
            if (!gearPolygon.IsConvex())
            {
                //Decompose the gear:
                List<Vertices> list = Triangulate.ConvexPartition(gearPolygon, TriangulationAlgorithm.Earclip);

                return CreateCompoundPolygon(world, list, density, position, rotation, bodyType, userData);
            }

            return CreatePolygon(world, gearPolygon, density, position, rotation, bodyType, userData);
        }

        public static Body CreateCapsule(World world, Fix64 height, Fix64 topRadius, int topEdges, Fix64 bottomRadius, int bottomEdges, Fix64 density, Vec2 position = new Vec2(), Fix64 rotation = new Fix64(), BodyType bodyType = BodyType.Static, object userData = null)
        {
            Vertices verts = PolygonUtils.CreateCapsule(height, topRadius, topEdges, bottomRadius, bottomEdges);

            //There are too many vertices in the capsule. We decompose it.
            if (verts.Count >= Settings.MaxPolygonVertices)
            {
                List<Vertices> vertList = Triangulate.ConvexPartition(verts, TriangulationAlgorithm.Earclip);
                return CreateCompoundPolygon(world, vertList, density, position, rotation, bodyType, userData);
            }

            return CreatePolygon(world, verts, density, position, rotation, bodyType, userData);
        }

        public static Body CreateCapsule(World world, Fix64 height, Fix64 endRadius, Fix64 density, Vec2 position = new Vec2(), Fix64 rotation = new Fix64(), BodyType bodyType = BodyType.Static, object userData = null)
        {
            //Create the middle rectangle
            Vertices rectangle = PolygonUtils.CreateRectangle(endRadius, height / Fix64.Two);

            List<Vertices> list = new List<Vertices>();
            list.Add(rectangle);

            Body body = CreateCompoundPolygon(world, list, density, position, rotation, bodyType, userData);
            FixtureFactory.AttachCircle(endRadius, density, body, new Vec2(Fix64.Zero, height / Fix64.Two));
            FixtureFactory.AttachCircle(endRadius, density, body, new Vec2(Fix64.Zero, -(height / Fix64.Two)));

            //Create the two circles
            //CircleShape topCircle = new CircleShape(endRadius, density);
            //topCircle.Position = new Vector2(0, height / 2);
            //body.CreateFixture(topCircle);

            //CircleShape bottomCircle = new CircleShape(endRadius, density);
            //bottomCircle.Position = new Vector2(0, -(height / 2));
            //body.CreateFixture(bottomCircle);
            return body;
        }

        public static Body CreateRoundedRectangle(World world, Fix64 width, Fix64 height, Fix64 xRadius, Fix64 yRadius, int segments, Fix64 density, Vec2 position = new Vec2(), Fix64 rotation = new Fix64(), BodyType bodyType = BodyType.Static, object userData = null)
        {
            Vertices verts = PolygonUtils.CreateRoundedRectangle(width, height, xRadius, yRadius, segments);

            //There are too many vertices in the capsule. We decompose it.
            if (verts.Count >= Settings.MaxPolygonVertices)
            {
                List<Vertices> vertList = Triangulate.ConvexPartition(verts, TriangulationAlgorithm.Earclip);
                return CreateCompoundPolygon(world, vertList, density, position, rotation, bodyType, userData);
            }

            return CreatePolygon(world, verts, density, position, rotation, bodyType, userData);
        }

        public static Body CreateLineArc(World world, Fix64 radians, int sides, Fix64 radius, bool closed = false, Vec2 position = new Vec2(), Fix64 rotation = new Fix64(), BodyType bodyType = BodyType.Static, object userData = null)
        {
            Body body = CreateBody(world, position, rotation, bodyType, userData);
            FixtureFactory.AttachLineArc(radians, sides, radius, closed, body);
            return body;
        }

        public static Body CreateSolidArc(World world, Fix64 density, Fix64 radians, int sides, Fix64 radius, Vec2 position = new Vec2(), Fix64 rotation = new Fix64(), BodyType bodyType = BodyType.Static, object userData = null)
        {
            Body body = CreateBody(world, position, rotation, bodyType, userData);
            FixtureFactory.AttachSolidArc(density, radians, sides, radius, body);

            return body;
        }

        public static BreakableBody CreateBreakableBody(World world, Vertices vertices, Fix64 density, Vec2 position = new Vec2(), Fix64 rotation = new Fix64())
        {
            //TODO: Implement a Voronoi diagram algorithm to split up the vertices
            List<Vertices> triangles = Triangulate.ConvexPartition(vertices, TriangulationAlgorithm.Earclip);

            BreakableBody breakableBody = new BreakableBody(world, triangles, density, position, rotation);
            breakableBody.MainBody.Position = position;
            world.AddBreakableBody(breakableBody);
            return breakableBody;
        }

        public static BreakableBody CreateBreakableBody(World world, IEnumerable<Shape> shapes, Vec2 position = new Vec2(), Fix64 rotation = new Fix64())
        {
            BreakableBody breakableBody = new BreakableBody(world, shapes, position, rotation);
            breakableBody.MainBody.Position = position;
            world.AddBreakableBody(breakableBody);
            return breakableBody;
        }

        public static Body CreateFromTemplate(World world, BodyTemplate bodyTemplate)
        {
            return world.CreateBody(bodyTemplate);
        }
    }
}
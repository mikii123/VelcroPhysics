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
    /// <summary>
    /// An easy to use factory for creating bodies
    /// </summary>
    public static class FixtureFactory
    {
        public static Fixture AttachEdge(Vec2 start, Vec2 end, Body body, object userData = null)
        {
            EdgeShape edgeShape = new EdgeShape(start, end);
            return body.CreateFixture(edgeShape, userData);
        }

        public static Fixture AttachChainShape(Vertices vertices, Body body, object userData = null)
        {
            ChainShape shape = new ChainShape(vertices);
            return body.CreateFixture(shape, userData);
        }

        public static Fixture AttachLoopShape(Vertices vertices, Body body, object userData = null)
        {
            ChainShape shape = new ChainShape(vertices, true);
            return body.CreateFixture(shape, userData);
        }

        public static Fixture AttachRectangle(Fix64 width, Fix64 height, Fix64 density, Vec2 offset, Body body, object userData = null)
        {
            Vertices rectangleVertices = PolygonUtils.CreateRectangle(width / Fix64.Two, height / Fix64.Two);
            rectangleVertices.Translate(ref offset);
            PolygonShape rectangleShape = new PolygonShape(rectangleVertices, density);
            return body.CreateFixture(rectangleShape, userData);
        }

        public static Fixture AttachCircle(Fix64 radius, Fix64 density, Body body, object userData = null)
        {
            if (radius <= 0)
                throw new ArgumentOutOfRangeException(nameof(radius), "Radius must be more than 0 meters");

            CircleShape circleShape = new CircleShape(radius, density);
            return body.CreateFixture(circleShape, userData);
        }

        public static Fixture AttachCircle(Fix64 radius, Fix64 density, Body body, Vec2 offset, object userData = null)
        {
            if (radius <= 0)
                throw new ArgumentOutOfRangeException(nameof(radius), "Radius must be more than 0 meters");

            CircleShape circleShape = new CircleShape(radius, density);
            circleShape.Position = offset;
            return body.CreateFixture(circleShape, userData);
        }

        public static Fixture AttachPolygon(Vertices vertices, Fix64 density, Body body, object userData = null)
        {
            if (vertices.Count <= 1)
                throw new ArgumentOutOfRangeException(nameof(vertices), "Too few points to be a polygon");

            PolygonShape polygon = new PolygonShape(vertices, density);
            return body.CreateFixture(polygon, userData);
        }

        public static Fixture AttachEllipse(Fix64 xRadius, Fix64 yRadius, int edges, Fix64 density, Body body, object userData = null)
        {
            if (xRadius <= 0)
                throw new ArgumentOutOfRangeException(nameof(xRadius), "X-radius must be more than 0");

            if (yRadius <= 0)
                throw new ArgumentOutOfRangeException(nameof(yRadius), "Y-radius must be more than 0");

            Vertices ellipseVertices = PolygonUtils.CreateEllipse(xRadius, yRadius, edges);
            PolygonShape polygonShape = new PolygonShape(ellipseVertices, density);
            return body.CreateFixture(polygonShape, userData);
        }

        public static List<Fixture> AttachCompoundPolygon(List<Vertices> list, Fix64 density, Body body, object userData = null)
        {
            List<Fixture> res = new List<Fixture>(list.Count);

            //Then we create several fixtures using the body
            foreach (Vertices vertices in list)
            {
                if (vertices.Count == 2)
                {
                    EdgeShape shape = new EdgeShape(vertices[0], vertices[1]);
                    res.Add(body.CreateFixture(shape, userData));
                }
                else
                {
                    PolygonShape shape = new PolygonShape(vertices, density);
                    res.Add(body.CreateFixture(shape, userData));
                }
            }

            return res;
        }

        public static Fixture AttachLineArc(Fix64 radians, int sides, Fix64 radius, bool closed, Body body)
        {
            Vertices arc = PolygonUtils.CreateArc(radians, sides, radius);
            arc.Rotate((Fix64.Pi - radians) / Fix64.Two);
            return closed ? AttachLoopShape(arc, body) : AttachChainShape(arc, body);
        }

        public static List<Fixture> AttachSolidArc(Fix64 density, Fix64 radians, int sides, Fix64 radius, Body body)
        {
            Vertices arc = PolygonUtils.CreateArc(radians, sides, radius);
            arc.Rotate((Fix64.Pi - radians) / Fix64.Two);

            //Close the arc
            arc.Add(arc[0]);

            List<Vertices> triangles = Triangulate.ConvexPartition(arc, TriangulationAlgorithm.Earclip);

            return AttachCompoundPolygon(triangles, density, body);
        }

        public static Fixture CreateFromTemplate(Body body, FixtureTemplate f1)
        {
            return body.CreateFixture(f1);
        }
    }
}
using System;
using System.Collections.Generic;
using FixedMath.Net;
using VelcroPhysics.Collision.Shapes;
using VelcroPhysics.Dynamics;
using VelcroPhysics.Factories;
using VelcroPhysics.Shared;
using VelcroPhysics.Tools.Triangulation.TriangulationBase;

namespace VelcroPhysics.Tools.PathGenerator
{
    /// <summary>
    /// An easy to use manager for creating paths.
    /// </summary>
    public static class PathManager
    {
        #region LinkType enum

        public enum LinkType
        {
            Revolute,
            Slider
        }

        #endregion

        //Contributed by Matthew Bettcher

        /// <summary>
        /// Convert a path into a set of edges and attaches them to the specified body.
        /// Note: use only for static edges.
        /// </summary>
        /// <param name="path">The path.</param>
        /// <param name="body">The body.</param>
        /// <param name="subdivisions">The subdivisions.</param>
        public static void ConvertPathToEdges(Path path, Body body, int subdivisions)
        {
            Vertices verts = path.GetVertices(subdivisions);

            if (path.Closed)
            {
                ChainShape chain = new ChainShape(verts, true);
                body.CreateFixture(chain);
            }
            else
            {
                for (int i = 1; i < verts.Count; i++)
                {
                    body.CreateFixture(new EdgeShape(verts[i], verts[i - 1]));
                }
            }
        }

        /// <summary>
        /// Convert a closed path into a polygon.
        /// Convex decomposition is automatically performed.
        /// </summary>
        /// <param name="path">The path.</param>
        /// <param name="body">The body.</param>
        /// <param name="density">The density.</param>
        /// <param name="subdivisions">The subdivisions.</param>
        public static void ConvertPathToPolygon(Path path, Body body, Fix64 density, int subdivisions)
        {
            if (!path.Closed)
                throw new Exception("The path must be closed to convert to a polygon.");

            List<Vec2> verts = path.GetVertices(subdivisions);

            List<Vertices> decomposedVerts = Triangulate.ConvexPartition(new Vertices(verts), TriangulationAlgorithm.Bayazit);

            foreach (Vertices item in decomposedVerts)
            {
                body.CreateFixture(new PolygonShape(item, density));
            }
        }

        /// <summary>
        /// Duplicates the given Body along the given path for approximately the given copies.
        /// </summary>
        /// <param name="world">The world.</param>
        /// <param name="path">The path.</param>
        /// <param name="shapes">The shapes.</param>
        /// <param name="type">The type.</param>
        /// <param name="copies">The copies.</param>
        /// <param name="userData"></param>
        /// <returns></returns>
        public static List<Body> EvenlyDistributeShapesAlongPath(World world, Path path, IEnumerable<Shape> shapes, BodyType type, int copies, object userData = null)
        {
            List<Vec3> centers = path.SubdivideEvenly(copies);
            List<Body> bodyList = new List<Body>();

            for (int i = 0; i < centers.Count; i++)
            {
                // copy the type from original body
                Body b = BodyFactory.CreateBody(world, new Vec2(centers[i].X, centers[i].Y), centers[i].Z, type, userData);

                foreach (Shape shape in shapes)
                {
                    b.CreateFixture(shape);
                }

                bodyList.Add(b);
            }

            return bodyList;
        }

        /// <summary>
        /// Duplicates the given Body along the given path for approximately the given copies.
        /// </summary>
        /// <param name="world">The world.</param>
        /// <param name="path">The path.</param>
        /// <param name="shape">The shape.</param>
        /// <param name="type">The type.</param>
        /// <param name="copies">The copies.</param>
        /// <param name="userData">The user data.</param>
        /// <returns></returns>
        public static List<Body> EvenlyDistributeShapesAlongPath(World world, Path path, Shape shape, BodyType type, int copies, object userData)
        {
            List<Shape> shapes = new List<Shape>(1);
            shapes.Add(shape);

            return EvenlyDistributeShapesAlongPath(world, path, shapes, type, copies, userData);
        }

        public static List<Body> EvenlyDistributeShapesAlongPath(World world, Path path, Shape shape, BodyType type, int copies)
        {
            return EvenlyDistributeShapesAlongPath(world, path, shape, type, copies, null);
        }

        /// <summary>
        /// Moves the given body along the defined path.
        /// </summary>
        /// <param name="path">The path.</param>
        /// <param name="body">The body.</param>
        /// <param name="time">The time.</param>
        /// <param name="strength">The strength.</param>
        /// <param name="timeStep">The time step.</param>
        public static void MoveBodyOnPath(Path path, Body body, Fix64 time, Fix64 strength, Fix64 timeStep)
        {
            Vec2 destination = path.GetPosition(time);
            Vec2 positionDelta = body.Position - destination;
            Vec2 velocity = (positionDelta / timeStep) * strength;

            body.LinearVelocity = -velocity;
        }
    }
}
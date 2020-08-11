﻿using System;
using System.Collections.Generic;
using System.Text;
using FixedMath.Net;
using Microsoft.Xna.Framework;
using VelcroPhysics.Shared;

namespace VelcroPhysics.Tools.PathGenerator
{
    //Contributed by Matthew Bettcher

    /// <summary>
    /// Path:
    /// Very similar to Vertices, but this
    /// class contains vectors describing
    /// control points on a Catmull-Rom
    /// curve.
    /// </summary>
    public class Path
    {
        private Fix64 _deltaT;

        /// <summary>
        /// All the points that makes up the curve
        /// </summary>
        public List<Vec2> ControlPoints;

        /// <summary>
        /// Initializes a new instance of the <see cref="Path" /> class.
        /// </summary>
        public Path()
        {
            ControlPoints = new List<Vec2>();
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="Path" /> class.
        /// </summary>
        /// <param name="vertices">The vertices to created the path from.</param>
        public Path(Vec2[] vertices)
        {
            ControlPoints = new List<Vec2>(vertices.Length);

            for (int i = 0; i < vertices.Length; i++)
            {
                Add(vertices[i]);
            }
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="Path" /> class.
        /// </summary>
        /// <param name="vertices">The vertices to created the path from.</param>
        public Path(IList<Vec2> vertices)
        {
            ControlPoints = new List<Vec2>(vertices.Count);
            for (int i = 0; i < vertices.Count; i++)
            {
                Add(vertices[i]);
            }
        }

        /// <summary>
        /// True if the curve is closed.
        /// </summary>
        /// <value><c>true</c> if closed; otherwise, <c>false</c>.</value>
        public bool Closed { get; set; }

        /// <summary>
        /// Gets the next index of a controlpoint
        /// </summary>
        /// <param name="index">The index.</param>
        /// <returns></returns>
        public int NextIndex(int index)
        {
            if (index == ControlPoints.Count - 1)
            {
                return 0;
            }
            return index + 1;
        }

        /// <summary>
        /// Gets the previous index of a controlpoint
        /// </summary>
        /// <param name="index">The index.</param>
        /// <returns></returns>
        public int PreviousIndex(int index)
        {
            if (index == 0)
            {
                return ControlPoints.Count - 1;
            }
            return index - 1;
        }

        /// <summary>
        /// Translates the control points by the specified vector.
        /// </summary>
        /// <param name="vector">The vector.</param>
        public void Translate(ref Vec2 vector)
        {
            for (int i = 0; i < ControlPoints.Count; i++)
                ControlPoints[i] = Vec2.Add(ControlPoints[i], vector);
        }

        /// <summary>
        /// Scales the control points by the specified vector.
        /// </summary>
        /// <param name="value">The Value.</param>
        public void Scale(ref Vec2 value)
        {
            for (int i = 0; i < ControlPoints.Count; i++)
                ControlPoints[i] = Vec2.Multiply(ControlPoints[i], value);
        }

        /// <summary>
        /// Rotate the control points by the defined value in radians.
        /// </summary>
        /// <param name="value">The amount to rotate by in radians.</param>
        public void Rotate(Fix64 value)
        {
            Matrix rotationMatrix;
            Matrix.CreateRotationZ(value, out rotationMatrix);

            for (int i = 0; i < ControlPoints.Count; i++)
                ControlPoints[i] = Vec2.Transform(ControlPoints[i], rotationMatrix);
        }

        public override string ToString()
        {
            StringBuilder builder = new StringBuilder();
            for (int i = 0; i < ControlPoints.Count; i++)
            {
                builder.Append(ControlPoints[i]);
                if (i < ControlPoints.Count - 1)
                {
                    builder.Append(" ");
                }
            }
            return builder.ToString();
        }

        /// <summary>
        /// Returns a set of points defining the
        /// curve with the specifed number of divisions
        /// between each control point.
        /// </summary>
        /// <param name="divisions">Number of divisions between each control point.</param>
        /// <returns></returns>
        public Vertices GetVertices(int divisions)
        {
            Vertices verts = new Vertices();

            Fix64 timeStep = Fix64.One / divisions;

            for (Fix64 i = new Fix64(); i < Fix64.One; i += timeStep)
            {
                verts.Add(GetPosition(i));
            }

            return verts;
        }

        public Vec2 GetPosition(Fix64 time)
        {
            Vec2 temp;

            if (ControlPoints.Count < 2)
                throw new Exception("You need at least 2 control points to calculate a position.");

            if (Closed)
            {
                Add(ControlPoints[0]);

                _deltaT = Fix64.One / (ControlPoints.Count - 1);

                int p = (int)(time / _deltaT);

                // use a circular indexing system
                int p0 = p - 1;
                if (p0 < 0) p0 = p0 + (ControlPoints.Count - 1);
                else if (p0 >= ControlPoints.Count - 1) p0 = p0 - (ControlPoints.Count - 1);
                int p1 = p;
                if (p1 < 0) p1 = p1 + (ControlPoints.Count - 1);
                else if (p1 >= ControlPoints.Count - 1) p1 = p1 - (ControlPoints.Count - 1);
                int p2 = p + 1;
                if (p2 < 0) p2 = p2 + (ControlPoints.Count - 1);
                else if (p2 >= ControlPoints.Count - 1) p2 = p2 - (ControlPoints.Count - 1);
                int p3 = p + 2;
                if (p3 < 0) p3 = p3 + (ControlPoints.Count - 1);
                else if (p3 >= ControlPoints.Count - 1) p3 = p3 - (ControlPoints.Count - 1);

                // relative time
                Fix64 lt = (time - _deltaT * p) / _deltaT;

                temp = Vec2.CatmullRom(ControlPoints[p0], ControlPoints[p1], ControlPoints[p2], ControlPoints[p3], lt);

                RemoveAt(ControlPoints.Count - 1);
            }
            else
            {
                int p = (int)(time / _deltaT);

                // 
                int p0 = p - 1;
                if (p0 < 0) p0 = 0;
                else if (p0 >= ControlPoints.Count - 1) p0 = ControlPoints.Count - 1;
                int p1 = p;
                if (p1 < 0) p1 = 0;
                else if (p1 >= ControlPoints.Count - 1) p1 = ControlPoints.Count - 1;
                int p2 = p + 1;
                if (p2 < 0) p2 = 0;
                else if (p2 >= ControlPoints.Count - 1) p2 = ControlPoints.Count - 1;
                int p3 = p + 2;
                if (p3 < 0) p3 = 0;
                else if (p3 >= ControlPoints.Count - 1) p3 = ControlPoints.Count - 1;

                // relative time
                Fix64 lt = (time - _deltaT * p) / _deltaT;

                temp = Vec2.CatmullRom(ControlPoints[p0], ControlPoints[p1], ControlPoints[p2], ControlPoints[p3], lt);
            }

            return temp;
        }

        /// <summary>
        /// Gets the normal for the given time.
        /// </summary>
        /// <param name="time">The time</param>
        /// <returns>The normal.</returns>
        public Vec2 GetPositionNormal(Fix64 time)
        {
	        Fix64 offsetTime = time + (Fix64.One / new Fix64(10000));

            Vec2 a = GetPosition(time);
            Vec2 b = GetPosition(offsetTime);

            Vec2 output = new Vec2(); 
            Vec2 temp = new Vec2();

            Vec2.Subtract(ref a, ref b, out temp);

#if (XBOX360 || WINDOWS_PHONE)
output = new Vector2();
#endif
            output.X = -temp.Y;
            output.Y = temp.X;

            output.Normalize();

            return output;
        }

        public void Add(Vec2 point)
        {
            ControlPoints.Add(point);
            _deltaT = Fix64.One / (ControlPoints.Count - 1);
        }

        public void Remove(Vec2 point)
        {
            ControlPoints.Remove(point);
            _deltaT = Fix64.One / (ControlPoints.Count - 1);
        }

        public void RemoveAt(int index)
        {
            ControlPoints.RemoveAt(index);
            _deltaT = Fix64.One / (ControlPoints.Count - 1);
        }

        public Fix64 GetLength()
        {
            List<Vec2> verts = GetVertices(ControlPoints.Count * 25);
            Fix64 length = new Fix64();

            for (int i = 1; i < verts.Count; i++)
            {
                length += Vec2.Distance(verts[i - 1], verts[i]);
            }

            if (Closed)
                length += Vec2.Distance(verts[ControlPoints.Count - 1], verts[0]);

            return length;
        }

        public List<Vec3> SubdivideEvenly(int divisions)
        {
            List<Vec3> verts = new List<Vec3>();

            Fix64 length = GetLength();

            Fix64 deltaLength = length / divisions + (Fix64.One / new Fix64(1000));
            Fix64 t = Fix64.Zero;

            // we always start at the first control point
            Vec2 start = ControlPoints[0];
            Vec2 end = GetPosition(t);

            // increment t until we are at half the distance
            while (deltaLength * Fix64.Half >= Vec2.Distance(start, end))
            {
                end = GetPosition(t);
                t += (Fix64.One / new Fix64(10000));

                if (t >= Fix64.One)
                    break;
            }

            start = end;

            // for each box
            for (int i = 1; i < divisions; i++)
            {
                Vec2 normal = GetPositionNormal(t);
                Fix64 angle = Fix64.Atan2(normal.Y, normal.X);

                verts.Add(new Vec3(end, angle));

                // until we reach the correct distance down the curve
                while (deltaLength >= Vec2.Distance(start, end))
                {
                    end = GetPosition(t);
                    t += (Fix64.One / new Fix64(100000));

                    if (t >= 1)
                        break;
                }
                if (t >= 1)
                    break;

                start = end;
            }
            return verts;
        }
    }
}
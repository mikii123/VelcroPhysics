using FixedMath.Net;
using Microsoft.Xna.Framework;

namespace VelcroPhysics.Shared
{
    /// <summary>
    /// A 2-by-2 matrix. Stored in column-major order.
    /// </summary>
    public struct Mat22
    {
        public Vec2 ex, ey;

        /// <summary>
        /// Construct this matrix using columns.
        /// </summary>
        /// <param name="c1">The c1.</param>
        /// <param name="c2">The c2.</param>
        public Mat22(Vec2 c1, Vec2 c2)
        {
            ex = c1;
            ey = c2;
        }

        /// <summary>
        /// Construct this matrix using scalars.
        /// </summary>
        /// <param name="a11">The a11.</param>
        /// <param name="a12">The a12.</param>
        /// <param name="a21">The a21.</param>
        /// <param name="a22">The a22.</param>
        public Mat22(Fix64 a11, Fix64 a12, Fix64 a21, Fix64 a22)
        {
            ex = new Vec2(a11, a21);
            ey = new Vec2(a12, a22);
        }

        public Mat22 Inverse
        {
            get
            {
                Fix64 a = ex.X, b = ey.X, c = ex.Y, d = ey.Y;
                Fix64 det = a * d - b * c;
                if (det != Fix64.Zero)
                {
                    det = Fix64.One / det;
                }

                Mat22 result = new Mat22();
                result.ex.X = det * d;
                result.ex.Y = -det * c;

                result.ey.X = -det * b;
                result.ey.Y = det * a;

                return result;
            }
        }

        /// <summary>
        /// Initialize this matrix using columns.
        /// </summary>
        /// <param name="c1">The c1.</param>
        /// <param name="c2">The c2.</param>
        public void Set(Vec2 c1, Vec2 c2)
        {
            ex = c1;
            ey = c2;
        }

        /// <summary>
        /// Set this to the identity matrix.
        /// </summary>
        public void SetIdentity()
        {
            ex.X = Fix64.One;
            ey.X = Fix64.Zero;
            ex.Y = Fix64.Zero;
            ey.Y = Fix64.One;
        }

        /// <summary>
        /// Set this matrix to all zeros.
        /// </summary>
        public void SetZero()
        {
            ex.X = Fix64.Zero;
            ey.X = Fix64.Zero;
            ex.Y = Fix64.Zero;
            ey.Y = Fix64.Zero;
        }

        /// <summary>
        /// Solve A * x = b, where b is a column vector. This is more efficient
        /// than computing the inverse in one-shot cases.
        /// </summary>
        /// <param name="b">The b.</param>
        /// <returns></returns>
        public Vec2 Solve(Vec2 b)
        {
            Fix64 a11 = ex.X, a12 = ey.X, a21 = ex.Y, a22 = ey.Y;
            Fix64 det = a11 * a22 - a12 * a21;
            if (det != Fix64.Zero)
            {
                det = Fix64.One / det;
            }

            return new Vec2(det * (a22 * b.X - a12 * b.Y), det * (a11 * b.Y - a21 * b.X));
        }

        public static void Add(ref Mat22 A, ref Mat22 B, out Mat22 R)
        {
            R.ex = A.ex + B.ex;
            R.ey = A.ey + B.ey;
        }
    }
}
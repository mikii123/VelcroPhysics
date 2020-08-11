using FixedMath.Net;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Shared
{
    /// <summary>
    /// A 3-by-3 matrix. Stored in column-major order.
    /// </summary>
    public struct Mat33
    {
        public Vec3 ex, ey, ez;

        /// <summary>
        /// Construct this matrix using columns.
        /// </summary>
        /// <param name="c1">The c1.</param>
        /// <param name="c2">The c2.</param>
        /// <param name="c3">The c3.</param>
        public Mat33(Vec3 c1, Vec3 c2, Vec3 c3)
        {
            ex = c1;
            ey = c2;
            ez = c3;
        }

        /// <summary>
        /// Set this matrix to all zeros.
        /// </summary>
        public void SetZero()
        {
            ex = Vec3.Zero;
            ey = Vec3.Zero;
            ez = Vec3.Zero;
        }

        /// <summary>
        /// Solve A * x = b, where b is a column vector. This is more efficient
        /// than computing the inverse in one-shot cases.
        /// </summary>
        /// <param name="b">The b.</param>
        /// <returns></returns>
        public Vec3 Solve33(Vec3 b)
        {
            Fix64 det = Vec3.Dot(ex, Vec3.Cross(ey, ez));
            if (det != Fix64.Zero)
            {
                det = Fix64.One / det;
            }

            return new Vec3(det * Vec3.Dot(b, Vec3.Cross(ey, ez)), det * Vec3.Dot(ex, Vec3.Cross(b, ez)), det * Vec3.Dot(ex, Vec3.Cross(ey, b)));
        }

        /// <summary>
        /// Solve A * x = b, where b is a column vector. This is more efficient
        /// than computing the inverse in one-shot cases. Solve only the upper
        /// 2-by-2 matrix equation.
        /// </summary>
        /// <param name="b">The b.</param>
        /// <returns></returns>
        public Vec2 Solve22(Vec2 b)
        {
            Fix64 a11 = ex.X, a12 = ey.X, a21 = ex.Y, a22 = ey.Y;
            Fix64 det = a11 * a22 - a12 * a21;

            if (det != Fix64.Zero)
            {
                det = Fix64.One / det;
            }

            return new Vec2(det * (a22 * b.X - a12 * b.Y), det * (a11 * b.Y - a21 * b.X));
        }

        /// Get the inverse of this matrix as a 2-by-2.
        /// Returns the zero matrix if singular.
        public void GetInverse22(ref Mat33 M)
        {
            Fix64 a = ex.X, b = ey.X, c = ex.Y, d = ey.Y;
            Fix64 det = a * d - b * c;
            if (det != Fix64.Zero)
            {
                det = Fix64.One / det;
            }

            M.ex.X = det * d;
            M.ey.X = -det * b;
            M.ex.Z = Fix64.Zero;
            M.ex.Y = -det * c;
            M.ey.Y = det * a;
            M.ey.Z = Fix64.Zero;
            M.ez.X = Fix64.Zero;
            M.ez.Y = Fix64.Zero;
            M.ez.Z = Fix64.Zero;
        }

        /// Get the symmetric inverse of this matrix as a 3-by-3.
        /// Returns the zero matrix if singular.
        public void GetSymInverse33(ref Mat33 M)
        {
            Fix64 det = MathUtils.Dot(ex, MathUtils.Cross((Vec3)ey, ez));
            if (det != Fix64.Zero)
            {
                det = Fix64.One / det;
            }

            Fix64 a11 = ex.X, a12 = ey.X, a13 = ez.X;
            Fix64 a22 = ey.Y, a23 = ez.Y;
            Fix64 a33 = ez.Z;

            M.ex.X = det * (a22 * a33 - a23 * a23);
            M.ex.Y = det * (a13 * a23 - a12 * a33);
            M.ex.Z = det * (a12 * a23 - a13 * a22);

            M.ey.X = M.ex.Y;
            M.ey.Y = det * (a11 * a33 - a13 * a13);
            M.ey.Z = det * (a13 * a12 - a11 * a23);

            M.ez.X = M.ex.Z;
            M.ez.Y = M.ey.Z;
            M.ez.Z = det * (a11 * a22 - a12 * a12);
        }
    }
}
using FixedMath.Net;

namespace VelcroPhysics.Tools.Triangulation.Seidel
{
    internal class Point
    {
        // Pointers to next and previous points in Monontone Mountain
        public Point Next, Prev;

        public Fix64 X, Y;

        public Point(Fix64 x, Fix64 y)
        {
            X = x;
            Y = y;
            Next = null;
            Prev = null;
        }

        public static Point operator -(Point p1, Point p2)
        {
            return new Point(p1.X - p2.X, p1.Y - p2.Y);
        }

        public static Point operator +(Point p1, Point p2)
        {
            return new Point(p1.X + p2.X, p1.Y + p2.Y);
        }

        public static Point operator -(Point p1, Fix64 f)
        {
            return new Point(p1.X - f, p1.Y - f);
        }

        public static Point operator +(Point p1, Fix64 f)
        {
            return new Point(p1.X + f, p1.Y + f);
        }

        public Fix64 Cross(Point p)
        {
            return X * p.Y - Y * p.X;
        }

        public Fix64 Dot(Point p)
        {
            return X * p.X + Y * p.Y;
        }

        public bool Neq(Point p)
        {
            return p.X != X || p.Y != Y;
        }

        public Fix64 Orient2D(Point pb, Point pc)
        {
            Fix64 acx = X - pc.X;
            Fix64 bcx = pb.X - pc.X;
            Fix64 acy = Y - pc.Y;
            Fix64 bcy = pb.Y - pc.Y;
            return acx * bcy - acy * bcx;
        }
    }
}
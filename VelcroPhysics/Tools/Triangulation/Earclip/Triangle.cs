using FixedMath.Net;
using VelcroPhysics.Shared;

namespace VelcroPhysics.Tools.Triangulation.Earclip {
    public class Triangle : Vertices
    {
        //Constructor automatically fixes orientation to ccw
        public Triangle(Fix64 x1, Fix64 y1, Fix64 x2, Fix64 y2, Fix64 x3, Fix64 y3)
        {
            Fix64 cross = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1);
            if (cross > Fix64.Zero)
            {
                Add(new Vec2(x1, y1));
                Add(new Vec2(x2, y2));
                Add(new Vec2(x3, y3));
            }
            else
            {
                Add(new Vec2(x1, y1));
                Add(new Vec2(x3, y3));
                Add(new Vec2(x2, y2));
            }
        }

        public bool IsInside(Fix64 x, Fix64 y)
        {
            Vec2 a = this[0];
            Vec2 b = this[1];
            Vec2 c = this[2];

            if (x < a.X && x < b.X && x < c.X) return false;
            if (x > a.X && x > b.X && x > c.X) return false;
            if (y < a.Y && y < b.Y && y < c.Y) return false;
            if (y > a.Y && y > b.Y && y > c.Y) return false;

            Fix64 vx2 = x - a.X;
            Fix64 vy2 = y - a.Y;
            Fix64 vx1 = b.X - a.X;
            Fix64 vy1 = b.Y - a.Y;
            Fix64 vx0 = c.X - a.X;
            Fix64 vy0 = c.Y - a.Y;

            Fix64 dot00 = vx0 * vx0 + vy0 * vy0;
            Fix64 dot01 = vx0 * vx1 + vy0 * vy1;
            Fix64 dot02 = vx0 * vx2 + vy0 * vy2;
            Fix64 dot11 = vx1 * vx1 + vy1 * vy1;
            Fix64 dot12 = vx1 * vx2 + vy1 * vy2;
            Fix64 invDenom = Fix64.One / (dot00 * dot11 - dot01 * dot01);
            Fix64 u = (dot11 * dot02 - dot01 * dot12) * invDenom;
            Fix64 v = (dot00 * dot12 - dot01 * dot02) * invDenom;

            return ((u > Fix64.Zero) && (v > Fix64.Zero) && (u + v < Fix64.One));
        }
    }
}
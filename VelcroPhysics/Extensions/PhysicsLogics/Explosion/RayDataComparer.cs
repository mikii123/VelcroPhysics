using System.Collections.Generic;
using FixedMath.Net;

namespace VelcroPhysics.Extensions.PhysicsLogics.Explosion
{
    /// <summary>
    /// This is a comparer used for
    /// detecting angle difference between rays
    /// </summary>
    internal class RayDataComparer : IComparer<Fix64>
    {
        #region IComparer<Fix64> Members

        int IComparer<Fix64>.Compare(Fix64 a, Fix64 b)
        {
            Fix64 diff = (a - b);
            if (diff > Fix64.Zero)
                return 1;
            if (diff < Fix64.Zero)
                return -1;
            return 0;
        }

        #endregion
    }
}
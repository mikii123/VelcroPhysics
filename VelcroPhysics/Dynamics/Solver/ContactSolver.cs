//#define B2_DEBUG_SOLVER
/*
* Velcro Physics:
* Copyright (c) 2017 Ian Qvist
* 
* Original source Box2D:
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org 
* 
* This software is provided 'as-is', without any express or implied 
* warranty.  In no event will the authors be held liable for any damages 
* arising from the use of this software. 
* Permission is granted to anyone to use this software for any purpose, 
* including commercial applications, and to alter it and redistribute it 
* freely, subject to the following restrictions: 
* 1. The origin of this software must not be misrepresented; you must not 
* claim that you wrote the original software. If you use this software 
* in a product, an acknowledgment in the product documentation would be 
* appreciated but is not required. 
* 2. Altered source versions must be plainly marked as such, and must not be 
* misrepresented as being the original software. 
* 3. This notice may not be removed or altered from any source distribution. 
*/

using System;
using System.Diagnostics;
using FixedMath.Net;
using VelcroPhysics.Collision.ContactSystem;
using VelcroPhysics.Collision.Narrowphase;
using VelcroPhysics.Collision.Shapes;
using VelcroPhysics.Shared;
using VelcroPhysics.Shared.Optimization;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Dynamics.Solver
{
    public class ContactSolver
    {
        private Contact[] _contacts;
        private int _count;
        private ContactPositionConstraint[] _positionConstraints;
        private Position[] _positions;
        private TimeStep _step;
        private Velocity[] _velocities;
        public ContactVelocityConstraint[] VelocityConstraints;

        public void Reset(TimeStep step, int count, Contact[] contacts, Position[] positions, Velocity[] velocities)
        {
            _step = step;
            _count = count;
            _positions = positions;
            _velocities = velocities;
            _contacts = contacts;

            // grow the array
            if (VelocityConstraints == null || VelocityConstraints.Length < count)
            {
                VelocityConstraints = new ContactVelocityConstraint[count * 2];
                _positionConstraints = new ContactPositionConstraint[count * 2];

                for (int i = 0; i < VelocityConstraints.Length; i++)
                {
                    VelocityConstraints[i] = new ContactVelocityConstraint();
                }

                for (int i = 0; i < _positionConstraints.Length; i++)
                {
                    _positionConstraints[i] = new ContactPositionConstraint();
                }
            }

            // Initialize position independent portions of the constraints.
            for (int i = 0; i < _count; ++i)
            {
                Contact contact = contacts[i];

                Fixture fixtureA = contact.FixtureA;
                Fixture fixtureB = contact.FixtureB;
                Shape shapeA = fixtureA.Shape;
                Shape shapeB = fixtureB.Shape;
                Fix64 radiusA = shapeA.Radius;
                Fix64 radiusB = shapeB.Radius;
                Body bodyA = fixtureA.Body;
                Body bodyB = fixtureB.Body;
                Manifold manifold = contact.Manifold;

                int pointCount = manifold.PointCount;
                Debug.Assert(pointCount > 0);

                ContactVelocityConstraint vc = VelocityConstraints[i];
                vc.Friction = contact.Friction;
                vc.Restitution = contact.Restitution;
                vc.TangentSpeed = contact.TangentSpeed;
                vc.IndexA = bodyA.IslandIndex;
                vc.IndexB = bodyB.IslandIndex;
                vc.InvMassA = bodyA._invMass;
                vc.InvMassB = bodyB._invMass;
                vc.InvIA = bodyA._invI;
                vc.InvIB = bodyB._invI;
                vc.ContactIndex = i;
                vc.PointCount = pointCount;
                vc.K.SetZero();
                vc.NormalMass.SetZero();

                ContactPositionConstraint pc = _positionConstraints[i];
                pc.IndexA = bodyA.IslandIndex;
                pc.IndexB = bodyB.IslandIndex;
                pc.InvMassA = bodyA._invMass;
                pc.InvMassB = bodyB._invMass;
                pc.LocalCenterA = bodyA._sweep.LocalCenter;
                pc.LocalCenterB = bodyB._sweep.LocalCenter;
                pc.InvIA = bodyA._invI;
                pc.InvIB = bodyB._invI;
                pc.LocalNormal = manifold.LocalNormal;
                pc.LocalPoint = manifold.LocalPoint;
                pc.PointCount = pointCount;
                pc.RadiusA = radiusA;
                pc.RadiusB = radiusB;
                pc.Type = manifold.Type;

                for (int j = 0; j < pointCount; ++j)
                {
                    ManifoldPoint cp = manifold.Points[j];
                    VelocityConstraintPoint vcp = vc.Points[j];

                    if (Settings.EnableWarmstarting)
                    {
                        vcp.NormalImpulse = _step.dtRatio * cp.NormalImpulse;
                        vcp.TangentImpulse = _step.dtRatio * cp.TangentImpulse;
                    }
                    else
                    {
                        vcp.NormalImpulse = Fix64.Zero;
                        vcp.TangentImpulse = Fix64.Zero;
                    }

                    vcp.rA = Vec2.Zero;
                    vcp.rB = Vec2.Zero;
                    vcp.NormalMass = Fix64.Zero;
                    vcp.TangentMass = Fix64.Zero;
                    vcp.VelocityBias = Fix64.Zero;

                    pc.LocalPoints[j] = cp.LocalPoint;
                }
            }
        }

        /// <summary>
        /// Initialize position dependent portions of the velocity constraints.
        /// </summary>
        public void InitializeVelocityConstraints()
        {
            for (int i = 0; i < _count; ++i)
            {
                ContactVelocityConstraint vc = VelocityConstraints[i];
                ContactPositionConstraint pc = _positionConstraints[i];

                Fix64 radiusA = pc.RadiusA;
                Fix64 radiusB = pc.RadiusB;
                Manifold manifold = _contacts[vc.ContactIndex].Manifold;

                int indexA = vc.IndexA;
                int indexB = vc.IndexB;

                Fix64 mA = vc.InvMassA;
                Fix64 mB = vc.InvMassB;
                Fix64 iA = vc.InvIA;
                Fix64 iB = vc.InvIB;
                Vec2 localCenterA = pc.LocalCenterA;
                Vec2 localCenterB = pc.LocalCenterB;

                Vec2 cA = _positions[indexA].C;
                Fix64 aA = _positions[indexA].A;
                Vec2 vA = _velocities[indexA].V;
                Fix64 wA = _velocities[indexA].W;

                Vec2 cB = _positions[indexB].C;
                Fix64 aB = _positions[indexB].A;
                Vec2 vB = _velocities[indexB].V;
                Fix64 wB = _velocities[indexB].W;

                Debug.Assert(manifold.PointCount > 0);

                Transform xfA = new Transform();
                Transform xfB = new Transform();
                xfA.q.Set(aA);
                xfB.q.Set(aB);
                xfA.p = cA - MathUtils.Mul(xfA.q, localCenterA);
                xfB.p = cB - MathUtils.Mul(xfB.q, localCenterB);

                WorldManifold.Initialize(ref manifold, ref xfA, radiusA, ref xfB, radiusB, out Vec2 normal, out FixedArray2<Vec2> points, out _);

                vc.Normal = normal;

                int pointCount = vc.PointCount;
                for (int j = 0; j < pointCount; ++j)
                {
                    VelocityConstraintPoint vcp = vc.Points[j];

                    vcp.rA = points[j] - cA;
                    vcp.rB = points[j] - cB;

                    Fix64 rnA = MathUtils.Cross(vcp.rA, vc.Normal);
                    Fix64 rnB = MathUtils.Cross(vcp.rB, vc.Normal);

                    Fix64 kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

                    vcp.NormalMass = kNormal > Fix64.Zero ? Fix64.One / kNormal : Fix64.Zero;

                    Vec2 tangent = MathUtils.Cross(vc.Normal, Fix64.One);

                    Fix64 rtA = MathUtils.Cross(vcp.rA, tangent);
                    Fix64 rtB = MathUtils.Cross(vcp.rB, tangent);

                    Fix64 kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

                    vcp.TangentMass = kTangent > Fix64.Zero ? Fix64.One / kTangent : Fix64.Zero;

                    // Setup a velocity bias for restitution.
                    vcp.VelocityBias = Fix64.Zero;
                    Fix64 vRel = Vec2.Dot(vc.Normal, vB + MathUtils.Cross(wB, vcp.rB) - vA - MathUtils.Cross(wA, vcp.rA));
                    if (vRel < -Settings.VelocityThreshold)
                    {
                        vcp.VelocityBias = -vc.Restitution * vRel;
                    }
                }

                // If we have two points, then prepare the block solver.
                if (vc.PointCount == 2 && Settings.BlockSolve)
                {
                    VelocityConstraintPoint vcp1 = vc.Points[0];
                    VelocityConstraintPoint vcp2 = vc.Points[1];

                    Fix64 rn1A = MathUtils.Cross(vcp1.rA, vc.Normal);
                    Fix64 rn1B = MathUtils.Cross(vcp1.rB, vc.Normal);
                    Fix64 rn2A = MathUtils.Cross(vcp2.rA, vc.Normal);
                    Fix64 rn2B = MathUtils.Cross(vcp2.rB, vc.Normal);

                    Fix64 k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
                    Fix64 k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
                    Fix64 k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;

                    // Ensure a reasonable condition number.
                    Fix64 k_maxConditionNumber = new Fix64(1000);
                    if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12))
                    {
                        // K is safe to invert.
                        vc.K.ex = new Vec2(k11, k12);
                        vc.K.ey = new Vec2(k12, k22);
                        vc.NormalMass = vc.K.Inverse;
                    }
                    else
                    {
                        // The constraints are redundant, just use one.
                        // TODO_ERIN use deepest?
                        vc.PointCount = 1;
                    }
                }
            }
        }

        public void WarmStart()
        {
            // Warm start.
            for (int i = 0; i < _count; ++i)
            {
                ContactVelocityConstraint vc = VelocityConstraints[i];

                int indexA = vc.IndexA;
                int indexB = vc.IndexB;
                Fix64 mA = vc.InvMassA;
                Fix64 iA = vc.InvIA;
                Fix64 mB = vc.InvMassB;
                Fix64 iB = vc.InvIB;
                int pointCount = vc.PointCount;

                Vec2 vA = _velocities[indexA].V;
                Fix64 wA = _velocities[indexA].W;
                Vec2 vB = _velocities[indexB].V;
                Fix64 wB = _velocities[indexB].W;

                Vec2 normal = vc.Normal;
                Vec2 tangent = MathUtils.Cross(normal, Fix64.One);

                for (int j = 0; j < pointCount; ++j)
                {
                    VelocityConstraintPoint vcp = vc.Points[j];
                    Vec2 P = vcp.NormalImpulse * normal + vcp.TangentImpulse * tangent;
                    wA -= iA * MathUtils.Cross(vcp.rA, P);
                    vA -= mA * P;
                    wB += iB * MathUtils.Cross(vcp.rB, P);
                    vB += mB * P;
                }

                _velocities[indexA].V = vA;
                _velocities[indexA].W = wA;
                _velocities[indexB].V = vB;
                _velocities[indexB].W = wB;
            }
        }

        public void SolveVelocityConstraints()
        {
            for (int i = 0; i < _count; ++i)
            {
                ContactVelocityConstraint vc = VelocityConstraints[i];

                int indexA = vc.IndexA;
                int indexB = vc.IndexB;
                Fix64 mA = vc.InvMassA;
                Fix64 iA = vc.InvIA;
                Fix64 mB = vc.InvMassB;
                Fix64 iB = vc.InvIB;
                int pointCount = vc.PointCount;

                Vec2 vA = _velocities[indexA].V;
                Fix64 wA = _velocities[indexA].W;
                Vec2 vB = _velocities[indexB].V;
                Fix64 wB = _velocities[indexB].W;

                Vec2 normal = vc.Normal;
                Vec2 tangent = MathUtils.Cross(normal, Fix64.One);
                Fix64 friction = vc.Friction;

                Debug.Assert(pointCount == 1 || pointCount == 2);

                // Solve tangent constraints first because non-penetration is more important
                // than friction.
                for (int j = 0; j < pointCount; ++j)
                {
                    VelocityConstraintPoint vcp = vc.Points[j];

                    // Relative velocity at contact
                    Vec2 dv = vB + MathUtils.Cross(wB, vcp.rB) - vA - MathUtils.Cross(wA, vcp.rA);

                    // Compute tangent force
                    Fix64 vt = Vec2.Dot(dv, tangent) - vc.TangentSpeed;
                    Fix64 lambda = vcp.TangentMass * (-vt);

                    // b2Clamp the accumulated force
                    Fix64 maxFriction = friction * vcp.NormalImpulse;
                    Fix64 newImpulse = MathUtils.Clamp(vcp.TangentImpulse + lambda, -maxFriction, maxFriction);
                    lambda = newImpulse - vcp.TangentImpulse;
                    vcp.TangentImpulse = newImpulse;

                    // Apply contact impulse
                    Vec2 P = lambda * tangent;

                    vA -= mA * P;
                    wA -= iA * MathUtils.Cross(vcp.rA, P);

                    vB += mB * P;
                    wB += iB * MathUtils.Cross(vcp.rB, P);
                }

                // Solve normal constraints
                if (pointCount == 1 || Settings.BlockSolve == false)
                {
                    for (int j = 0; j < pointCount; ++j)
                    {
                        VelocityConstraintPoint vcp = vc.Points[j];

                        // Relative velocity at contact
                        Vec2 dv = vB + MathUtils.Cross(wB, vcp.rB) - vA - MathUtils.Cross(wA, vcp.rA);

                        // Compute normal impulse
                        Fix64 vn = Vec2.Dot(dv, normal);
                        Fix64 lambda = -vcp.NormalMass * (vn - vcp.VelocityBias);

                        // b2Clamp the accumulated impulse
                        Fix64 newImpulse = Fix64.Max(vcp.NormalImpulse + lambda, Fix64.Zero);
                        lambda = newImpulse - vcp.NormalImpulse;
                        vcp.NormalImpulse = newImpulse;

                        // Apply contact impulse
                        Vec2 P = lambda * normal;
                        vA -= mA * P;
                        wA -= iA * MathUtils.Cross(vcp.rA, P);

                        vB += mB * P;
                        wB += iB * MathUtils.Cross(vcp.rB, P);
                    }
                }
                else
                {
                    // Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
                    // Build the mini LCP for this contact patch
                    //
                    // vn = A * x + b, vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
                    //
                    // A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
                    // b = vn0 - velocityBias
                    //
                    // The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
                    // implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
                    // vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
                    // solution that satisfies the problem is chosen.
                    // 
                    // In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
                    // that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
                    //
                    // Substitute:
                    // 
                    // x = a + d
                    // 
                    // a := old total impulse
                    // x := new total impulse
                    // d := incremental impulse 
                    //
                    // For the current iteration we extend the formula for the incremental impulse
                    // to compute the new total impulse:
                    //
                    // vn = A * d + b
                    //    = A * (x - a) + b
                    //    = A * x + b - A * a
                    //    = A * x + b'
                    // b' = b - A * a;

                    VelocityConstraintPoint cp1 = vc.Points[0];
                    VelocityConstraintPoint cp2 = vc.Points[1];

                    Vec2 a = new Vec2(cp1.NormalImpulse, cp2.NormalImpulse);
                    Debug.Assert(a.X >= Fix64.Zero && a.Y >= Fix64.Zero);

                    // Relative velocity at contact
                    Vec2 dv1 = vB + MathUtils.Cross(wB, cp1.rB) - vA - MathUtils.Cross(wA, cp1.rA);
                    Vec2 dv2 = vB + MathUtils.Cross(wB, cp2.rB) - vA - MathUtils.Cross(wA, cp2.rA);

                    // Compute normal velocity
                    Fix64 vn1 = Vec2.Dot(dv1, normal);
                    Fix64 vn2 = Vec2.Dot(dv2, normal);

                    Vec2 b = Vec2.Zero;
                    b.X = vn1 - cp1.VelocityBias;
                    b.Y = vn2 - cp2.VelocityBias;
                    
                    // Compute b'
                    b -= MathUtils.Mul(ref vc.K, a);

                    for (;;)
                    {
                        //
                        // Case 1: vn = 0
                        //
                        // 0 = A * x + b'
                        //
                        // Solve for x:
                        //
                        // x = - inv(A) * b'
                        //
                        Vec2 x = -MathUtils.Mul(ref vc.NormalMass, b);

                        if (x.X >= Fix64.Zero && x.Y >= Fix64.Zero)
                        {
                            // Get the incremental impulse
                            Vec2 d = x - a;

                            // Apply incremental impulse
                            Vec2 P1 = d.X * normal;
                            Vec2 P2 = d.Y * normal;
                            vA -= mA * (P1 + P2);
                            wA -= iA * (MathUtils.Cross(cp1.rA, P1) + MathUtils.Cross(cp2.rA, P2));

                            vB += mB * (P1 + P2);
                            wB += iB * (MathUtils.Cross(cp1.rB, P1) + MathUtils.Cross(cp2.rB, P2));

                            // Accumulate
                            cp1.NormalImpulse = x.X;
                            cp2.NormalImpulse = x.Y;

#if B2_DEBUG_SOLVER
                            // Postconditions
                            dv1 = vB + MathUtils.Cross(wB, cp1.rB) - vA - MathUtils.Cross(wA, cp1.rA);
                            dv2 = vB + MathUtils.Cross(wB, cp2.rB) - vA - MathUtils.Cross(wA, cp2.rA);

                            // Compute normal velocity
                            vn1 = Vector2.Dot(dv1, normal);
                            vn2 = Vector2.Dot(dv2, normal);

                            Debug.Assert(Fix64.Abs(vn1 - cp1.VelocityBias) < k_errorTol);
                            Debug.Assert(Fix64.Abs(vn2 - cp2.VelocityBias) < k_errorTol);
#endif
                            break;
                        }

                        //
                        // Case 2: vn1 = 0 and x2 = 0
                        //
                        //   0 = a11 * x1 + a12 * 0 + b1' 
                        // vn2 = a21 * x1 + a22 * 0 + b2'
                        //
                        x.X = -cp1.NormalMass * b.X;
                        x.Y = Fix64.Zero;
                        vn1 = Fix64.Zero;
                        vn2 = vc.K.ex.Y * x.X + b.Y;

                        if (x.X >= Fix64.Zero && vn2 >= Fix64.Zero)
                        {
                            // Get the incremental impulse
                            Vec2 d = x - a;

                            // Apply incremental impulse
                            Vec2 P1 = d.X * normal;
                            Vec2 P2 = d.Y * normal;
                            vA -= mA * (P1 + P2);
                            wA -= iA * (MathUtils.Cross(cp1.rA, P1) + MathUtils.Cross(cp2.rA, P2));

                            vB += mB * (P1 + P2);
                            wB += iB * (MathUtils.Cross(cp1.rB, P1) + MathUtils.Cross(cp2.rB, P2));

                            // Accumulate
                            cp1.NormalImpulse = x.X;
                            cp2.NormalImpulse = x.Y;

#if B2_DEBUG_SOLVER
                            // Postconditions
                            dv1 = vB + MathUtils.Cross(wB, cp1.rB) - vA - MathUtils.Cross(wA, cp1.rA);

                            // Compute normal velocity
                            vn1 = Vector2.Dot(dv1, normal);

                            Debug.Assert(Fix64.Abs(vn1 - cp1.VelocityBias) < k_errorTol);
#endif
                            break;
                        }

                        //
                        // Case 3: vn2 = 0 and x1 = 0
                        //
                        // vn1 = a11 * 0 + a12 * x2 + b1' 
                        //   0 = a21 * 0 + a22 * x2 + b2'
                        //
                        x.X = Fix64.Zero;
                        x.Y = -cp2.NormalMass * b.Y;
                        vn1 = vc.K.ey.X * x.Y + b.X;
                        vn2 = Fix64.Zero;

                        if (x.Y >= Fix64.Zero && vn1 >= Fix64.Zero)
                        {
                            // Resubstitute for the incremental impulse
                            Vec2 d = x - a;

                            // Apply incremental impulse
                            Vec2 P1 = d.X * normal;
                            Vec2 P2 = d.Y * normal;
                            vA -= mA * (P1 + P2);
                            wA -= iA * (MathUtils.Cross(cp1.rA, P1) + MathUtils.Cross(cp2.rA, P2));

                            vB += mB * (P1 + P2);
                            wB += iB * (MathUtils.Cross(cp1.rB, P1) + MathUtils.Cross(cp2.rB, P2));

                            // Accumulate
                            cp1.NormalImpulse = x.X;
                            cp2.NormalImpulse = x.Y;

#if B2_DEBUG_SOLVER
                            // Postconditions
                            dv2 = vB + MathUtils.Cross(wB, cp2.rB) - vA - MathUtils.Cross(wA, cp2.rA);

                            // Compute normal velocity
                            vn2 = Vector2.Dot(dv2, normal);

                            Debug.Assert(Fix64.Abs(vn2 - cp2.VelocityBias) < k_errorTol);
#endif
                            break;
                        }

                        //
                        // Case 4: x1 = 0 and x2 = 0
                        // 
                        // vn1 = b1
                        // vn2 = b2;
                        x.X = Fix64.Zero;
                        x.Y = Fix64.Zero;
                        vn1 = b.X;
                        vn2 = b.Y;

                        if (vn1 >= Fix64.Zero && vn2 >= Fix64.Zero)
                        {
                            // Resubstitute for the incremental impulse
                            Vec2 d = x - a;

                            // Apply incremental impulse
                            Vec2 P1 = d.X * normal;
                            Vec2 P2 = d.Y * normal;
                            vA -= mA * (P1 + P2);
                            wA -= iA * (MathUtils.Cross(cp1.rA, P1) + MathUtils.Cross(cp2.rA, P2));

                            vB += mB * (P1 + P2);
                            wB += iB * (MathUtils.Cross(cp1.rB, P1) + MathUtils.Cross(cp2.rB, P2));

                            // Accumulate
                            cp1.NormalImpulse = x.X;
                            cp2.NormalImpulse = x.Y;

                            break;
                        }

                        // No solution, give up. This is hit sometimes, but it doesn't seem to matter.
                        break;
                    }
                }

                _velocities[indexA].V = vA;
                _velocities[indexA].W = wA;
                _velocities[indexB].V = vB;
                _velocities[indexB].W = wB;
            }
        }

        public void StoreImpulses()
        {
            for (int i = 0; i < _count; ++i)
            {
                ContactVelocityConstraint vc = VelocityConstraints[i];
                Manifold manifold = _contacts[vc.ContactIndex].Manifold;

                for (int j = 0; j < vc.PointCount; ++j)
                {
                    ManifoldPoint point = manifold.Points[j];
                    point.NormalImpulse = vc.Points[j].NormalImpulse;
                    point.TangentImpulse = vc.Points[j].TangentImpulse;
                    manifold.Points[j] = point;
                }

                _contacts[vc.ContactIndex].Manifold = manifold;
            }
        }

        public bool SolvePositionConstraints()
        {
            Fix64 minSeparation = Fix64.Zero;

            for (int i = 0; i < _count; ++i)
            {
                ContactPositionConstraint pc = _positionConstraints[i];

                int indexA = pc.IndexA;
                int indexB = pc.IndexB;
                Vec2 localCenterA = pc.LocalCenterA;
                Fix64 mA = pc.InvMassA;
                Fix64 iA = pc.InvIA;
                Vec2 localCenterB = pc.LocalCenterB;
                Fix64 mB = pc.InvMassB;
                Fix64 iB = pc.InvIB;
                int pointCount = pc.PointCount;

                Vec2 cA = _positions[indexA].C;
                Fix64 aA = _positions[indexA].A;

                Vec2 cB = _positions[indexB].C;
                Fix64 aB = _positions[indexB].A;

                // Solve normal constraints
                for (int j = 0; j < pointCount; ++j)
                {
                    Transform xfA = new Transform();
                    Transform xfB = new Transform();
                    xfA.q.Set(aA);
                    xfB.q.Set(aB);
                    xfA.p = cA - MathUtils.Mul(xfA.q, localCenterA);
                    xfB.p = cB - MathUtils.Mul(xfB.q, localCenterB);

                    PositionSolverManifold.Initialize(pc, xfA, xfB, j, out Vec2 normal, out Vec2 point, out Fix64 separation);

                    Vec2 rA = point - cA;
                    Vec2 rB = point - cB;

                    // Track max constraint error.
                    minSeparation = Fix64.Min(minSeparation, separation);

                    // Prevent large corrections and allow slop.
                    Fix64 C = MathUtils.Clamp(Settings.Baumgarte * (separation + Settings.LinearSlop), -Settings.MaxLinearCorrection, Fix64.Zero);

                    // Compute the effective mass.
                    Fix64 rnA = MathUtils.Cross(rA, normal);
                    Fix64 rnB = MathUtils.Cross(rB, normal);
                    Fix64 K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

                    // Compute normal impulse
                    Fix64 impulse = K > Fix64.Zero ? -C / K : Fix64.Zero;

                    Vec2 P = impulse * normal;

                    cA -= mA * P;
                    aA -= iA * MathUtils.Cross(rA, P);

                    cB += mB * P;
                    aB += iB * MathUtils.Cross(rB, P);
                }

                _positions[indexA].C = cA;
                _positions[indexA].A = aA;

                _positions[indexB].C = cB;
                _positions[indexB].A = aB;
            }

            // We can't expect minSpeparation >= -b2_linearSlop because we don't
            // push the separation above -b2_linearSlop.
            return minSeparation >= -Fix64.Three * Settings.LinearSlop;
        }

        // Sequential position solver for position constraints.
        public bool SolveTOIPositionConstraints(int toiIndexA, int toiIndexB)
        {
            Fix64 minSeparation = Fix64.Zero;

            for (int i = 0; i < _count; ++i)
            {
                ContactPositionConstraint pc = _positionConstraints[i];

                int indexA = pc.IndexA;
                int indexB = pc.IndexB;
                Vec2 localCenterA = pc.LocalCenterA;
                Vec2 localCenterB = pc.LocalCenterB;
                int pointCount = pc.PointCount;

                Fix64 mA = Fix64.Zero;
                Fix64 iA = Fix64.Zero;
                if (indexA == toiIndexA || indexA == toiIndexB)
                {
                    mA = pc.InvMassA;
                    iA = pc.InvIA;
                }

                Fix64 mB = Fix64.Zero;
                Fix64 iB = Fix64.Zero;
                if (indexB == toiIndexA || indexB == toiIndexB)
                {
                    mB = pc.InvMassB;
                    iB = pc.InvIB;
                }

                Vec2 cA = _positions[indexA].C;
                Fix64 aA = _positions[indexA].A;

                Vec2 cB = _positions[indexB].C;
                Fix64 aB = _positions[indexB].A;

                // Solve normal constraints
                for (int j = 0; j < pointCount; ++j)
                {
                    Transform xfA = new Transform();
                    Transform xfB = new Transform();
                    xfA.q.Set(aA);
                    xfB.q.Set(aB);
                    xfA.p = cA - MathUtils.Mul(xfA.q, localCenterA);
                    xfB.p = cB - MathUtils.Mul(xfB.q, localCenterB);

                    PositionSolverManifold.Initialize(pc, xfA, xfB, j, out Vec2 normal, out Vec2 point, out Fix64 separation);

                    Vec2 rA = point - cA;
                    Vec2 rB = point - cB;

                    // Track max constraint error.
                    minSeparation = Fix64.Min(minSeparation, separation);

                    // Prevent large corrections and allow slop.
                    Fix64 C = MathUtils.Clamp(Settings.Baumgarte * (separation + Settings.LinearSlop), -Settings.MaxLinearCorrection, Fix64.Zero);

                    // Compute the effective mass.
                    Fix64 rnA = MathUtils.Cross(rA, normal);
                    Fix64 rnB = MathUtils.Cross(rB, normal);
                    Fix64 K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

                    // Compute normal impulse
                    Fix64 impulse = K > Fix64.Zero ? -C / K : Fix64.Zero;

                    Vec2 P = impulse * normal;

                    cA -= mA * P;
                    aA -= iA * MathUtils.Cross(rA, P);

                    cB += mB * P;
                    aB += iB * MathUtils.Cross(rB, P);
                }

                _positions[indexA].C = cA;
                _positions[indexA].A = aA;

                _positions[indexB].C = cB;
                _positions[indexB].A = aB;
            }

            // We can't expect minSpeparation >= -b2_linearSlop because we don't
            // push the separation above -b2_linearSlop.
            return minSeparation >= -(Fix64.One + Fix64.Half) * Settings.LinearSlop;
        }
    }
}
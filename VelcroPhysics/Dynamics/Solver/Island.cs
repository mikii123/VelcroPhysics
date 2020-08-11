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
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Dynamics.Solver
{
    /// <summary>
    /// This is an internal class.
    /// </summary>
    public class Island
    {
        private static readonly Fix64 LinTolSqr = Settings.LinearSleepTolerance * Settings.LinearSleepTolerance;
        private static readonly Fix64 AngTolSqr = Settings.AngularSleepTolerance * Settings.AngularSleepTolerance;
        private ContactManager _contactManager;
        private Contact[] _contacts;
        private ContactSolver _contactSolver = new ContactSolver();
        private Stopwatch _watch = new Stopwatch();

        public Body[] Bodies;
        public Position[] Positions;
        public Velocity[] Velocities;
        public int BodyCapacity;
        public int BodyCount;
        public int ContactCapacity;
        public int ContactCount;

        public void Reset(int bodyCapacity, int contactCapacity, ContactManager contactManager)
        {
            BodyCapacity = bodyCapacity;
            ContactCapacity = contactCapacity;
            BodyCount = 0;
            ContactCount = 0;

            _contactManager = contactManager;

            if (Bodies == null || Bodies.Length < bodyCapacity)
            {
                Bodies = new Body[bodyCapacity];
                Velocities = new Velocity[bodyCapacity];
                Positions = new Position[bodyCapacity];
            }

            if (_contacts == null || _contacts.Length < contactCapacity)
            {
                _contacts = new Contact[contactCapacity * 2];
            }
        }

        public void Clear()
        {
            BodyCount = 0;
            ContactCount = 0;
        }

        public void Solve(ref TimeStep step, ref Vec2 gravity)
        {
            Fix64 h = step.dt;

            // Integrate velocities and apply damping. Initialize the body state.
            for (int i = 0; i < BodyCount; ++i)
            {
                Body b = Bodies[i];

                Vec2 c = b._sweep.C;
                Fix64 a = b._sweep.A;
                Vec2 v = b._linearVelocity;
                Fix64 w = b._angularVelocity;

                // Store positions for continuous collision.
                b._sweep.C0 = b._sweep.C;
                b._sweep.A0 = b._sweep.A;

                if (b.BodyType == BodyType.Dynamic)
                {
                    // Integrate velocities.

                    //Velcro: Only apply gravity if the body wants it.
                    if (b.IgnoreGravity)
                        v += h * (b._invMass * b._force);
                    else
                        v += h * (b.GravityScale * gravity + b._invMass * b._force);

                    w += h * b._invI * b._torque;

                    // Apply damping.
                    // ODE: dv/dt + c * v = 0
                    // Solution: v(t) = v0 * exp(-c * t)
                    // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
                    // v2 = exp(-c * dt) * v1
                    // Taylor expansion:
                    // v2 = (Fix64.One - c * dt) * v1
                    v *= MathUtils.Clamp(Fix64.One - h * b.LinearDamping, Fix64.Zero, Fix64.One);
                    w *= MathUtils.Clamp(Fix64.One - h * b.AngularDamping, Fix64.Zero, Fix64.One);
                }

                Positions[i].C = c;
                Positions[i].A = a;
                Velocities[i].V = v;
                Velocities[i].W = w;
            }

            // Solver data
            SolverData solverData = new SolverData();
            solverData.Step = step;
            solverData.Positions = Positions;
            solverData.Velocities = Velocities;

            _contactSolver.Reset(step, ContactCount, _contacts, Positions, Velocities);
            _contactSolver.InitializeVelocityConstraints();

            if (Settings.EnableWarmstarting)
            {
                _contactSolver.WarmStart();
            }

            // Solve velocity constraints.
            for (int i = 0; i < Settings.VelocityIterations; ++i)
            {
                _contactSolver.SolveVelocityConstraints();
            }

            // Store impulses for warm starting.
            _contactSolver.StoreImpulses();

            // Integrate positions
            for (int i = 0; i < BodyCount; ++i)
            {
                Vec2 c = Positions[i].C;
                Fix64 a = Positions[i].A;
                Vec2 v = Velocities[i].V;
                Fix64 w = Velocities[i].W;

                // Check for large velocities
                Vec2 translation = h * v;
                if (Vec2.Dot(translation, translation) > Settings.MaxTranslationSquared)
                {
                    Fix64 ratio = Settings.MaxTranslation / translation.Magnitude;
                    v *= ratio;
                }

                Fix64 rotation = h * w;
                if (rotation * rotation > Settings.MaxRotationSquared)
                {
                    Fix64 ratio = Settings.MaxRotation / Fix64.Abs(rotation);
                    w *= ratio;
                }

                // Integrate
                c += h * v;
                a += h * w;

                Positions[i].C = c;
                Positions[i].A = a;
                Velocities[i].V = v;
                Velocities[i].W = w;
            }

            // Solve position constraints
            bool positionSolved = false;
            for (int i = 0; i < Settings.PositionIterations; ++i)
            {
                if (_contactSolver.SolvePositionConstraints())
                {
                    // Exit early if the position errors are small.
                    positionSolved = true;
                    break;
                }
            }

            // Copy state buffers back to the bodies
            for (int i = 0; i < BodyCount; ++i)
            {
                Body body = Bodies[i];
                body._sweep.C = Positions[i].C;
                body._sweep.A = Positions[i].A;
                body._linearVelocity = Velocities[i].V;
                body._angularVelocity = Velocities[i].W;
                body.SynchronizeTransform();
            }

            Report(_contactSolver.VelocityConstraints);

            if (Settings.AllowSleep)
            {
                Fix64 minSleepTime = Fix64.MaxValue;

                for (int i = 0; i < BodyCount; ++i)
                {
                    Body b = Bodies[i];

                    if (b.BodyType == BodyType.Static)
                        continue;

                    if (!b.SleepingAllowed || b._angularVelocity * b._angularVelocity > AngTolSqr || Vec2.Dot(b._linearVelocity, b._linearVelocity) > LinTolSqr)
                    {
                        b.SleepTime = Fix64.Zero;
                        minSleepTime = Fix64.Zero;
                    }
                    else
                    {
                        b.SleepTime += h;
                        minSleepTime = Fix64.Min(minSleepTime, b.SleepTime);
                    }
                }

                if (minSleepTime >= Settings.TimeToSleep && positionSolved)
                {
                    for (int i = 0; i < BodyCount; ++i)
                    {
                        Body b = Bodies[i];
                        b.Awake = false;
                    }
                }
            }
        }

        internal void SolveTOI(ref TimeStep subStep, int toiIndexA, int toiIndexB)
        {
            Debug.Assert(toiIndexA < BodyCount);
            Debug.Assert(toiIndexB < BodyCount);

            // Initialize the body state.
            for (int i = 0; i < BodyCount; ++i)
            {
                Body b = Bodies[i];
                Positions[i].C = b._sweep.C;
                Positions[i].A = b._sweep.A;
                Velocities[i].V = b._linearVelocity;
                Velocities[i].W = b._angularVelocity;
            }

            _contactSolver.Reset(subStep, ContactCount, _contacts, Positions, Velocities);

            // Solve position constraints.
            for (int i = 0; i < Settings.TOIPositionIterations; ++i)
            {
                bool contactsOkay = _contactSolver.SolveTOIPositionConstraints(toiIndexA, toiIndexB);
                if (contactsOkay)
                {
                    break;
                }
            }

            // Leap of faith to new safe state.
            Bodies[toiIndexA]._sweep.C0 = Positions[toiIndexA].C;
            Bodies[toiIndexA]._sweep.A0 = Positions[toiIndexA].A;
            Bodies[toiIndexB]._sweep.C0 = Positions[toiIndexB].C;
            Bodies[toiIndexB]._sweep.A0 = Positions[toiIndexB].A;

            // No warm starting is needed for TOI events because warm
            // starting impulses were applied in the discrete solver.
            _contactSolver.InitializeVelocityConstraints();

            // Solve velocity constraints.
            for (int i = 0; i < Settings.TOIVelocityIterations; ++i)
            {
                _contactSolver.SolveVelocityConstraints();
            }

            // Don't store the TOI contact forces for warm starting
            // because they can be quite large.

            Fix64 h = subStep.dt;

            // Integrate positions.
            for (int i = 0; i < BodyCount; ++i)
            {
                Vec2 c = Positions[i].C;
                Fix64 a = Positions[i].A;
                Vec2 v = Velocities[i].V;
                Fix64 w = Velocities[i].W;

                // Check for large velocities
                Vec2 translation = h * v;
                if (Vec2.Dot(translation, translation) > Settings.MaxTranslationSquared)
                {
                    Fix64 ratio = Settings.MaxTranslation / translation.Magnitude;
                    v *= ratio;
                }

                Fix64 rotation = h * w;
                if (rotation * rotation > Settings.MaxRotationSquared)
                {
                    Fix64 ratio = Settings.MaxRotation / Fix64.Abs(rotation);
                    w *= ratio;
                }

                // Integrate
                c += h * v;
                a += h * w;

                Positions[i].C = c;
                Positions[i].A = a;
                Velocities[i].V = v;
                Velocities[i].W = w;

                // Sync bodies
                Body body = Bodies[i];
                body._sweep.C = c;
                body._sweep.A = a;
                body._linearVelocity = v;
                body._angularVelocity = w;
                body.SynchronizeTransform();
            }

            Report(_contactSolver.VelocityConstraints);
        }

        public void Add(Body body)
        {
            Debug.Assert(BodyCount < BodyCapacity);
            body.IslandIndex = BodyCount;
            Bodies[BodyCount++] = body;
        }

        public void Add(Contact contact)
        {
            Debug.Assert(ContactCount < ContactCapacity);
            _contacts[ContactCount++] = contact;
        }

        private void Report(ContactVelocityConstraint[] constraints)
        {
            if (_contactManager == null)
                return;

            for (int i = 0; i < ContactCount; ++i)
            {
                Contact c = _contacts[i];

                //Velcro optimization: We don't store the impulses and send it to the delegate. We just send the whole contact.
                //Velcro feature: added after collision
                c.FixtureA.AfterCollision?.Invoke(c.FixtureA, c.FixtureB, c, constraints[i]);

                c.FixtureB.AfterCollision?.Invoke(c.FixtureB, c.FixtureA, c, constraints[i]);

                _contactManager.PostSolve?.Invoke(c, constraints[i]);
            }
        }
    }
}
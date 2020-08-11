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
using System.Collections.Generic;
using System.Diagnostics;
using FixedMath.Net;
using VelcroPhysics.Collision.Broadphase;
using VelcroPhysics.Collision.ContactSystem;
using VelcroPhysics.Collision.Distance;
using VelcroPhysics.Collision.RayCast;
using VelcroPhysics.Collision.TOI;
using VelcroPhysics.Dynamics.Handlers;
using VelcroPhysics.Dynamics.Solver;
using VelcroPhysics.Extensions.Controllers.ControllerBase;
using VelcroPhysics.Shared;
using VelcroPhysics.Templates;

namespace VelcroPhysics.Dynamics
{
	/// <summary>
	///     The world class manages all physics entities, dynamic simulation,
	///     and asynchronous queries.
	/// </summary>
	public class World
	{
		#region PublicFields

		public List<Controller> ControllerList { get; }

		public List<BreakableBody> BreakableBodyList { get; }

		public Fix64 UpdateTime { get; private set; }

		public Fix64 ContinuousPhysicsTime { get; private set; }

		public Fix64 ControllersUpdateTime { get; private set; }

		public Fix64 AddRemoveTime { get; private set; }

		public Fix64 NewContactsTime { get; private set; }

		public Fix64 ContactsUpdateTime { get; private set; }

		public Fix64 SolveUpdateTime { get; private set; }

		/// <summary>
		///     Get the number of broad-phase proxies.
		/// </summary>
		/// <value>The proxy count.</value>
		public int ProxyCount => ContactManager.BroadPhase.ProxyCount;

		/// <summary>
		///     Get the contact manager for testing.
		/// </summary>
		/// <value>The contact manager.</value>
		public ContactManager ContactManager { get; }

		/// <summary>
		///     Get the world body list.
		/// </summary>
		/// <value>The head of the world body list.</value>
		public List<Body> BodyList { get; }

		/// <summary>
		///     Get the world contact list. With the returned contact, use Contact.GetNext to get
		///     the next contact in the world list. A null contact indicates the end of the list.
		/// </summary>
		/// <value>The head of the world contact list.</value>
		public List<Contact> ContactList => ContactManager.ContactList;

		/// <summary>
		///     If false, the whole simulation stops. It still processes added and removed geometries.
		/// </summary>
		public bool Enabled { get; set; }

		public Island Island { get; }

		/// <summary>
		///     Fires whenever a body has been added
		/// </summary>
		public BodyHandler BodyAdded;

		/// <summary>
		///     Fires whenever a body has been removed
		/// </summary>
		public BodyHandler BodyRemoved;

		/// <summary>
		///     Fires whenever a fixture has been added
		/// </summary>
		public FixtureHandler FixtureAdded;

		/// <summary>
		///     Fires whenever a fixture has been removed
		/// </summary>
		public FixtureHandler FixtureRemoved;

		/// <summary>
		///     Fires every time a controller is added to the World.
		/// </summary>
		public ControllerHandler ControllerAdded;

		/// <summary>
		///     Fires every time a controller is removed form the World.
		/// </summary>
		public ControllerHandler ControllerRemoved;

		/// <summary>
		///     Change the global gravity vector.
		/// </summary>
		/// <value>The gravity.</value>
		public Vec2 Gravity;

		#endregion

		#region PrivateFields

		private Fix64 _invDt0;
		private Body[] _stack = new Body[64];
		private bool _stepComplete;
		private HashSet<Body> _bodyAddList = new HashSet<Body>();
		private HashSet<Body> _bodyRemoveList = new HashSet<Body>();
		private Func<Fixture, bool> _queryAABBCallback;
		private Func<int, bool> _queryAABBCallbackWrapper;
		private Fixture _myFixture;
		private Vec2 _point1;
		private Vec2 _point2;
		private List<Fixture> _testPointAllFixtures;
		private Stopwatch _watch = new Stopwatch();
		private Func<Fixture, Vec2, Vec2, Fix64, Fix64> _rayCastCallback;
		private Func<RayCastInput, int, Fix64> _rayCastCallbackWrapper;

		#endregion

		#region OtherFields

		internal Queue<Contact> _contactPool = new Queue<Contact>(256);
		internal bool _worldHasNewFixture;

		internal int _bodyIdCounter;
		internal int _fixtureIdCounter;

		#endregion

		#region Constructors

		/// <summary>
		///     Initializes a new instance of the <see cref="World" /> class.
		/// </summary>
		public World(Vec2 gravity)
		{
			Island = new Island();
			Enabled = true;
			ControllerList = new List<Controller>();
			BreakableBodyList = new List<BreakableBody>();
			BodyList = new List<Body>(32);

			_queryAABBCallbackWrapper = QueryAABBCallbackWrapper;
			_rayCastCallbackWrapper = RayCastCallbackWrapper;

			ContactManager = new ContactManager(new DynamicTreeBroadPhase());
			Gravity = gravity;
		}

		#endregion

		#region PublicMethods

		/// <summary>
		///     Destroy a rigid body.
		///     Warning: This automatically deletes all associated shapes and joints.
		/// </summary>
		/// <param name="body">The body.</param>
		public void RemoveBody(Body body)
		{
			Debug.Assert(!_bodyRemoveList.Contains(body), "The body is already marked for removal. You are removing the body more than once.");

			if (!_bodyRemoveList.Contains(body))
			{
				_bodyRemoveList.Add(body);
			}
		}

		/// <summary>
		///     All adds and removes are cached by the World during a World step.
		///     To process the changes before the world updates again, call this method.
		/// </summary>
		public void ProcessChanges()
		{
			ProcessAddedBodies();
			ProcessRemovedBodies();
		}

		/// <summary>
		///     Take a time step. This performs collision detection, integration,
		///     and constraint solution.
		/// </summary>
		/// <param name="dt">The amount of time to simulate, this should not vary.</param>
		public void Step(Fix64 dt)
		{
			if (!Enabled)
			{
				return;
			}

			ProcessChanges();

			// If new fixtures were added, we need to find the new contacts.
			if (_worldHasNewFixture)
			{
				ContactManager.FindNewContacts();
				_worldHasNewFixture = false;
			}

			//Velcro only: moved position and velocity iterations into Settings.cs
			TimeStep step;
			step.inv_dt = dt > Fix64.Zero ? Fix64.One / dt : Fix64.Zero;
			step.dt = dt;
			step.dtRatio = _invDt0 * dt;

			//Update controllers
			for (int i = 0; i < ControllerList.Count; i++)
			{
				ControllerList[i].Update(dt);
			}

			// Update contacts. This is where some contacts are destroyed.
			ContactManager.Collide();

			// Integrate velocities, solve velocity constraints, and integrate positions.
			Solve(ref step);

			// Handle TOI events.
			if (Settings.ContinuousPhysics)
			{
				SolveTOI(ref step);
			}

			if (Settings.AutoClearForces)
			{
				ClearForces();
			}

			for (int i = 0; i < BreakableBodyList.Count; i++)
			{
				BreakableBodyList[i].Update();
			}

			_invDt0 = step.inv_dt;
		}

		/// <summary>
		///     Call this after you are done with time steps to clear the forces. You normally
		///     call this after each call to Step, unless you are performing sub-steps. By default,
		///     forces will be automatically cleared, so you don't need to call this function.
		/// </summary>
		public void ClearForces()
		{
			for (int i = 0; i < BodyList.Count; i++)
			{
				Body body = BodyList[i];
				body._force = Vec2.Zero;
				body._torque = Fix64.Zero;
			}
		}

		/// <summary>
		///     Query the world for all fixtures that potentially overlap the provided AABB.
		///     Inside the callback:
		///     Return true: Continues the query
		///     Return false: Terminate the query
		/// </summary>
		/// <param name="callback">A user implemented callback class.</param>
		/// <param name="aabb">The AABB query box.</param>
		public void QueryAABB(Func<Fixture, bool> callback, ref AABB aabb)
		{
			_queryAABBCallback = callback;
			ContactManager.BroadPhase.Query(_queryAABBCallbackWrapper, ref aabb);
			_queryAABBCallback = null;
		}

		/// <summary>
		///     Query the world for all fixtures that potentially overlap the provided AABB.
		///     Use the overload with a callback for filtering and better performance.
		/// </summary>
		/// <param name="aabb">The AABB query box.</param>
		/// <returns>A list of fixtures that were in the affected area.</returns>
		public List<Fixture> QueryAABB(ref AABB aabb)
		{
			var affected = new List<Fixture>();

			QueryAABB(
				fixture =>
				{
					affected.Add(fixture);
					return true;
				},
				ref aabb);

			return affected;
		}

		/// <summary>
		///     Ray-cast the world for all fixtures in the path of the ray. Your callback
		///     controls whether you get the closest point, any point, or n-points.
		///     The ray-cast ignores shapes that contain the starting point.
		///     Inside the callback:
		///     return -1: ignore this fixture and continue
		///     return 0: terminate the ray cast
		///     return fraction: clip the ray to this point
		///     return 1: don't clip the ray and continue
		/// </summary>
		/// <param name="callback">A user implemented callback class.</param>
		/// <param name="point1">The ray starting point.</param>
		/// <param name="point2">The ray ending point.</param>
		public void RayCast(Func<Fixture, Vec2, Vec2, Fix64, Fix64> callback, Vec2 point1, Vec2 point2)
		{
			RayCastInput input = new RayCastInput();
			input.MaxFraction = Fix64.One;
			input.Point1 = point1;
			input.Point2 = point2;

			_rayCastCallback = callback;
			ContactManager.BroadPhase.RayCast(_rayCastCallbackWrapper, ref input);
			_rayCastCallback = null;
		}

		public List<Fixture> RayCast(Vec2 point1, Vec2 point2)
		{
			var affected = new List<Fixture>();

			RayCast(
				(f, p, n, fr) =>
				{
					affected.Add(f);
					return Fix64.One;
				},
				point1,
				point2);

			return affected;
		}

		public void AddController(Controller controller)
		{
			Debug.Assert(!ControllerList.Contains(controller), "You are adding the same controller more than once.");

			controller.World = this;
			ControllerList.Add(controller);

			ControllerAdded?.Invoke(controller);
		}

		public void RemoveController(Controller controller)
		{
			Debug.Assert(
				ControllerList.Contains(controller),
				"You are removing a controller that is not in the simulation.");

			if (ControllerList.Contains(controller))
			{
				ControllerList.Remove(controller);

				ControllerRemoved?.Invoke(controller);
			}
		}

		public void AddBreakableBody(BreakableBody breakableBody)
		{
			BreakableBodyList.Add(breakableBody);
		}

		public void RemoveBreakableBody(BreakableBody breakableBody)
		{
			//The breakable body list does not contain the body you tried to remove.
			Debug.Assert(BreakableBodyList.Contains(breakableBody));

			BreakableBodyList.Remove(breakableBody);
		}

		public Fixture TestPoint(Vec2 point)
		{
			AABB aabb;
			Vec2 d = new Vec2(Fix64.Epsilon, Fix64.Epsilon);
			aabb.LowerBound = point - d;
			aabb.UpperBound = point + d;

			_myFixture = null;
			_point1 = point;

			// Query the world for overlapping shapes.
			QueryAABB(TestPointCallback, ref aabb);

			return _myFixture;
		}

		/// <summary>
		///     Returns a list of fixtures that are at the specified point.
		/// </summary>
		/// <param name="point">The point.</param>
		/// <returns></returns>
		public List<Fixture> TestPointAll(Vec2 point)
		{
			AABB aabb;
			Vec2 d = new Vec2(Fix64.Epsilon, Fix64.Epsilon);
			aabb.LowerBound = point - d;
			aabb.UpperBound = point + d;

			_point2 = point;
			_testPointAllFixtures = new List<Fixture>();

			// Query the world for overlapping shapes.
			QueryAABB(TestPointAllCallback, ref aabb);

			return _testPointAllFixtures;
		}

		/// Shift the world origin. Useful for large worlds.
		/// The body shift formula is: position -= newOrigin
		/// @param newOrigin the new origin with respect to the old origin
		/// Warning: Calling this method mid-update might cause a crash.
		public void ShiftOrigin(Vec2 newOrigin)
		{
			foreach (Body b in BodyList)
			{
				b._xf.p -= newOrigin;
				b._sweep.C0 -= newOrigin;
				b._sweep.C -= newOrigin;
			}

			ContactManager.BroadPhase.ShiftOrigin(newOrigin);
		}

		public void Clear()
		{
			ProcessChanges();

			for (int i = BodyList.Count - 1; i >= 0; i--)
			{
				RemoveBody(BodyList[i]);
			}

			for (int i = ControllerList.Count - 1; i >= 0; i--)
			{
				RemoveController(ControllerList[i]);
			}

			for (int i = BreakableBodyList.Count - 1; i >= 0; i--)
			{
				RemoveBreakableBody(BreakableBodyList[i]);
			}

			ProcessChanges();
		}

		#endregion

		#region PrivateMethods

		private void ProcessAddedBodies()
		{
			if (_bodyAddList.Count > 0)
			{
				foreach (Body body in _bodyAddList)
				{
					// Add to world list.
					BodyList.Add(body);

					BodyAdded?.Invoke(body);
				}

				_bodyAddList.Clear();
			}
		}

		private void ProcessRemovedBodies()
		{
			if (_bodyRemoveList.Count > 0)
			{
				foreach (Body body in _bodyRemoveList)
				{
					Debug.Assert(BodyList.Count > 0);

					// You tried to remove a body that is not contained in the BodyList.
					// Are you removing the body more than once?
					Debug.Assert(BodyList.Contains(body));

					// Delete the attached contacts.
					ContactEdge ce = body.ContactList;
					while (ce != null)
					{
						ContactEdge ce0 = ce;
						ce = ce.Next;
						ContactManager.Destroy(ce0.Contact);
					}

					body.ContactList = null;

					// Delete the attached fixtures. This destroys broad-phase proxies.
					for (int i = 0; i < body.FixtureList.Count; i++)
					{
						body.FixtureList[i].DestroyProxies(ContactManager.BroadPhase);
						body.FixtureList[i].Destroy();
					}

					body.FixtureList = null;

					// Remove world body list.
					BodyList.Remove(body);

					BodyRemoved?.Invoke(body);
				}

				_bodyRemoveList.Clear();
			}
		}

		private bool QueryAABBCallbackWrapper(int proxyId)
		{
			FixtureProxy proxy = ContactManager.BroadPhase.GetProxy(proxyId);
			return _queryAABBCallback(proxy.Fixture);
		}

		private Fix64 RayCastCallbackWrapper(RayCastInput rayCastInput, int proxyId)
		{
			FixtureProxy proxy = ContactManager.BroadPhase.GetProxy(proxyId);
			Fixture fixture = proxy.Fixture;
			int index = proxy.ChildIndex;
			RayCastOutput output;
			bool hit = fixture.RayCast(out output, ref rayCastInput, index);

			if (hit)
			{
				Fix64 fraction = output.Fraction;
				Vec2 point = (Fix64.One - fraction) * rayCastInput.Point1 + fraction * rayCastInput.Point2;
				return _rayCastCallback(fixture, point, output.Normal, fraction);
			}

			return rayCastInput.MaxFraction;
		}

		private void Solve(ref TimeStep step)
		{
			// Size the island for the worst case.
			Island.Reset(
				BodyList.Count,
				ContactManager.ContactList.Count,
				ContactManager);

			// Clear all the island flags.
			foreach (Body b in BodyList)
			{
				b._flags &= ~BodyFlags.IslandFlag;
			}

			foreach (Contact c in ContactManager.ContactList)
			{
				c._flags &= ~ContactFlags.IslandFlag;
			}

			// Build and simulate all awake islands.
			int stackSize = BodyList.Count;
			if (stackSize > _stack.Length)
			{
				_stack = new Body[Math.Max(_stack.Length * 2, stackSize)];
			}

			for (int index = BodyList.Count - 1; index >= 0; index--)
			{
				Body seed = BodyList[index];
				if ((seed._flags & BodyFlags.IslandFlag) == BodyFlags.IslandFlag)
				{
					continue;
				}

				if (seed.Awake == false || seed.Enabled == false)
				{
					continue;
				}

				// The seed can be dynamic or kinematic.
				if (seed.BodyType == BodyType.Static)
				{
					continue;
				}

				// Reset island and stack.
				Island.Clear();
				int stackCount = 0;
				_stack[stackCount++] = seed;

				seed._flags |= BodyFlags.IslandFlag;

				// Perform a depth first search (DFS) on the constraint graph.
				while (stackCount > 0)
				{
					// Grab the next body off the stack and add it to the island.
					Body b = _stack[--stackCount];
					Debug.Assert(b.Enabled);
					Island.Add(b);

					// Make sure the body is awake (without resetting sleep timer).
					b._flags |= BodyFlags.AwakeFlag;

					// To keep islands as small as possible, we don't
					// propagate islands across static bodies.
					if (b.BodyType == BodyType.Static)
					{
						continue;
					}

					// Search all contacts connected to this body.
					for (ContactEdge ce = b.ContactList; ce != null; ce = ce.Next)
					{
						Contact contact = ce.Contact;

						// Has this contact already been added to an island?
						if (contact.IslandFlag)
						{
							continue;
						}

						// Is this contact solid and touching?
						if (ce.Contact.Enabled == false || ce.Contact.IsTouching == false)
						{
							continue;
						}

						// Skip sensors.
						bool sensorA = contact.FixtureA.IsSensor;
						bool sensorB = contact.FixtureB.IsSensor;
						if (sensorA || sensorB)
						{
							continue;
						}

						Island.Add(contact);
						contact._flags |= ContactFlags.IslandFlag;

						Body other = ce.Other;

						// Was the other body already added to this island?
						if (other.IsIsland)
						{
							continue;
						}

						Debug.Assert(stackCount < stackSize);
						_stack[stackCount++] = other;

						other._flags |= BodyFlags.IslandFlag;
					}
				}

				Island.Solve(ref step, ref Gravity);

				// Post solve cleanup.
				for (int i = 0; i < Island.BodyCount; ++i)
				{
					// Allow static bodies to participate in other islands.
					Body b = Island.Bodies[i];
					if (b.BodyType == BodyType.Static)
					{
						b._flags &= ~BodyFlags.IslandFlag;
					}
				}
			}

			// Synchronize fixtures, check for out of range bodies.

			foreach (Body b in BodyList)
			{
				// If a body was not in an island then it did not move.
				if (!b.IsIsland)
				{
					continue;
				}

				if (b.BodyType == BodyType.Static)
				{
					continue;
				}

				// Update fixtures (for broad-phase).
				b.SynchronizeFixtures();
			}

			// Look for new contacts.
			ContactManager.FindNewContacts();
		}

		private void SolveTOI(ref TimeStep step)
		{
			Island.Reset(2 * Settings.MaxTOIContacts, Settings.MaxTOIContacts, ContactManager);

			if (_stepComplete)
			{
				for (int i = 0; i < BodyList.Count; i++)
				{
					BodyList[i]._flags &= ~BodyFlags.IslandFlag;
					BodyList[i]._sweep.Alpha0 = Fix64.Zero;
				}

				for (int i = 0; i < ContactManager.ContactList.Count; i++)
				{
					Contact c = ContactManager.ContactList[i];

					// Invalidate TOI
					c._flags &= ~ContactFlags.IslandFlag;
					c._flags &= ~ContactFlags.TOIFlag;
					c._toiCount = 0;
					c._toi = Fix64.One;
				}
			}

			// Find TOI events and solve them.
			for (;;)
			{
				// Find the first TOI.
				Contact minContact = null;
				Fix64 minAlpha = Fix64.One;

				for (int i = 0; i < ContactManager.ContactList.Count; i++)
				{
					Contact c = ContactManager.ContactList[i];

					// Is this contact disabled?
					if (c.Enabled == false)
					{
						continue;
					}

					// Prevent excessive sub-stepping.
					if (c._toiCount > Settings.MaxSubSteps)
					{
						continue;
					}

					Fix64 alpha;
					if (c.TOIFlag)
					{
						// This contact has a valid cached TOI.
						alpha = c._toi;
					}
					else
					{
						Fixture fA = c.FixtureA;
						Fixture fB = c.FixtureB;

						// Is there a sensor?
						if (fA.IsSensor || fB.IsSensor)
						{
							continue;
						}

						Body bA = fA.Body;
						Body bB = fB.Body;

						BodyType typeA = bA.BodyType;
						BodyType typeB = bB.BodyType;
						Debug.Assert(typeA == BodyType.Dynamic || typeB == BodyType.Dynamic);

						bool activeA = bA.Awake && typeA != BodyType.Static;
						bool activeB = bB.Awake && typeB != BodyType.Static;

						// Is at least one body active (awake and dynamic or kinematic)?
						if (activeA == false && activeB == false)
						{
							continue;
						}

						bool collideA = (bA.IsBullet || typeA != BodyType.Dynamic) && (fA.IgnoreCCDWith & fB.CollisionCategories) == 0 && !bA.IgnoreCCD;
						bool collideB = (bB.IsBullet || typeB != BodyType.Dynamic) && (fB.IgnoreCCDWith & fA.CollisionCategories) == 0 && !bB.IgnoreCCD;

						// Are these two non-bullet dynamic bodies?
						if (collideA == false && collideB == false)
						{
							continue;
						}

						// Compute the TOI for this contact.
						// Put the sweeps onto the same time interval.
						Fix64 alpha0 = bA._sweep.Alpha0;

						if (bA._sweep.Alpha0 < bB._sweep.Alpha0)
						{
							alpha0 = bB._sweep.Alpha0;
							bA._sweep.Advance(alpha0);
						}
						else if (bB._sweep.Alpha0 < bA._sweep.Alpha0)
						{
							alpha0 = bA._sweep.Alpha0;
							bB._sweep.Advance(alpha0);
						}

						Debug.Assert(alpha0 < Fix64.One);

						// Compute the time of impact in interval [0, minTOI]
						TOIInput input = new TOIInput();
						input.ProxyA = new DistanceProxy(fA.Shape, c.ChildIndexA);
						input.ProxyB = new DistanceProxy(fB.Shape, c.ChildIndexB);
						input.SweepA = bA._sweep;
						input.SweepB = bB._sweep;
						input.TMax = Fix64.One;

						TOIOutput output;
						TimeOfImpact.CalculateTimeOfImpact(ref input, out output);

						// Beta is the fraction of the remaining portion of the .
						Fix64 beta = output.T;
						if (output.State == TOIOutputState.Touching)
						{
							alpha = Fix64.Min(alpha0 + (Fix64.One - alpha0) * beta, Fix64.One);
						}
						else
						{
							alpha = Fix64.One;
						}

						c._toi = alpha;
						c._flags &= ~ContactFlags.TOIFlag;
					}

					if (alpha < minAlpha)
					{
						// This is the minimum TOI found so far.
						minContact = c;
						minAlpha = alpha;
					}
				}

				if (minContact == null || Fix64.One - new Fix64(10) * Fix64.Epsilon < minAlpha)
				{
					// No more TOI events. Done!
					_stepComplete = true;
					break;
				}

				// Advance the bodies to the TOI.
				Fixture fA1 = minContact.FixtureA;
				Fixture fB1 = minContact.FixtureB;
				Body bA0 = fA1.Body;
				Body bB0 = fB1.Body;

				Sweep backup1 = bA0._sweep;
				Sweep backup2 = bB0._sweep;

				bA0.Advance(minAlpha);
				bB0.Advance(minAlpha);

				// The TOI contact likely has some new contact points.
				minContact.Update(ContactManager);
				minContact._flags &= ~ContactFlags.TOIFlag;
				++minContact._toiCount;

				// Is the contact solid?
				if (minContact.Enabled == false || minContact.IsTouching == false)
				{
					// Restore the sweeps.
					minContact._flags &= ~ContactFlags.EnabledFlag;
					bA0._sweep = backup1;
					bB0._sweep = backup2;
					bA0.SynchronizeTransform();
					bB0.SynchronizeTransform();
					continue;
				}

				bA0.Awake = true;
				bB0.Awake = true;

				// Build the island
				Island.Clear();
				Island.Add(bA0);
				Island.Add(bB0);
				Island.Add(minContact);

				bA0._flags |= BodyFlags.IslandFlag;
				bB0._flags |= BodyFlags.IslandFlag;
				minContact._flags &= ~ContactFlags.IslandFlag;

				// Get contacts on bodyA and bodyB.
				Body[] bodies = { bA0, bB0 };
				for (int i = 0; i < 2; ++i)
				{
					Body body = bodies[i];
					if (body.BodyType == BodyType.Dynamic)
					{
						for (ContactEdge ce = body.ContactList; ce != null; ce = ce.Next)
						{
							Contact contact = ce.Contact;

							if (Island.BodyCount == Island.BodyCapacity)
							{
								break;
							}

							if (Island.ContactCount == Island.ContactCapacity)
							{
								break;
							}

							// Has this contact already been added to the island?
							if (contact.IslandFlag)
							{
								continue;
							}

							// Only add static, kinematic, or bullet bodies.
							Body other = ce.Other;
							if (other.BodyType == BodyType.Dynamic &&
								body.IsBullet == false && other.IsBullet == false)
							{
								continue;
							}

							// Skip sensors.
							if (contact.FixtureA.IsSensor || contact.FixtureB.IsSensor)
							{
								continue;
							}

							// Tentatively advance the body to the TOI.
							Sweep backup = other._sweep;
							if (!other.IsIsland)
							{
								other.Advance(minAlpha);
							}

							// Update the contact points
							contact.Update(ContactManager);

							// Was the contact disabled by the user?
							if (contact.Enabled == false)
							{
								other._sweep = backup;
								other.SynchronizeTransform();
								continue;
							}

							// Are there contact points?
							if (contact.IsTouching == false)
							{
								other._sweep = backup;
								other.SynchronizeTransform();
								continue;
							}

							// Add the contact to the island
							minContact._flags |= ContactFlags.IslandFlag;
							Island.Add(contact);

							// Has the other body already been added to the island?
							if (other.IsIsland)
							{
								continue;
							}

							// Add the other body to the island.
							other._flags |= BodyFlags.IslandFlag;

							if (other.BodyType != BodyType.Static)
							{
								other.Awake = true;
							}

							Island.Add(other);
						}
					}
				}

				TimeStep subStep;
				subStep.dt = (Fix64.One - minAlpha) * step.dt;
				subStep.inv_dt = Fix64.One / subStep.dt;
				subStep.dtRatio = Fix64.One;
				Island.SolveTOI(ref subStep, bA0.IslandIndex, bB0.IslandIndex);

				// Reset island flags and synchronize broad-phase proxies.
				for (int i = 0; i < Island.BodyCount; ++i)
				{
					Body body = Island.Bodies[i];
					body._flags &= ~BodyFlags.IslandFlag;

					if (body.BodyType != BodyType.Dynamic)
					{
						continue;
					}

					body.SynchronizeFixtures();

					// Invalidate all contact TOIs on this displaced body.
					for (ContactEdge ce = body.ContactList; ce != null; ce = ce.Next)
					{
						ce.Contact._flags &= ~ContactFlags.TOIFlag;
						ce.Contact._flags &= ~ContactFlags.IslandFlag;
					}
				}

				// Commit fixture proxy movements to the broad-phase so that new contacts are created.
				// Also, some contacts can be destroyed.
				ContactManager.FindNewContacts();

				if (Settings.EnableSubStepping)
				{
					_stepComplete = false;
					break;
				}
			}
		}

		private bool TestPointCallback(Fixture fixture)
		{
			bool inside = fixture.TestPoint(ref _point1);
			if (inside)
			{
				_myFixture = fixture;
				return false;
			}

			// Continue the query.
			return true;
		}

		private bool TestPointAllCallback(Fixture fixture)
		{
			bool inside = fixture.TestPoint(ref _point2);
			if (inside)
			{
				_testPointAllFixtures.Add(fixture);
			}

			// Continue the query.
			return true;
		}

		#endregion

		#region AllOtherMembers

		/// <summary>
		///     Add a rigid body.
		/// </summary>
		/// <returns></returns>
		internal void AddBody(Body body)
		{
			Debug.Assert(!_bodyAddList.Contains(body), "You are adding the same body more than once.");

			if (!_bodyAddList.Contains(body))
			{
				_bodyAddList.Add(body);
			}
		}

		internal Body CreateBody(BodyTemplate template)
		{
			Body b = new Body(this, template) { BodyId = _bodyIdCounter++ };

			AddBody(b);
			return b;
		}

		#endregion
	}
}

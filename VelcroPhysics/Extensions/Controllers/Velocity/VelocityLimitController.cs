using System;
using System.Collections.Generic;
using FixedMath.Net;
using VelcroPhysics.Dynamics;
using VelcroPhysics.Extensions.Controllers.ControllerBase;

namespace VelcroPhysics.Extensions.Controllers.Velocity
{
    /// <summary>
    /// Put a limit on the linear (translation - the move speed) and angular (rotation) velocity
    /// of bodies added to this controller.
    /// </summary>
    public class VelocityLimitController : Controller
    {
        private List<Body> _bodies = new List<Body>();
        private Fix64 _maxAngularSqared;
        private Fix64 _maxAngularVelocity;
        private Fix64 _maxLinearSqared;
        private Fix64 _maxLinearVelocity;
        public bool LimitAngularVelocity = true;
        public bool LimitLinearVelocity = true;

        /// <summary>
        /// Initializes a new instance of the <see cref="VelocityLimitController" /> class.
        /// Sets the max linear velocity to Settings.MaxTranslation
        /// Sets the max angular velocity to Settings.MaxRotation
        /// </summary>
        public VelocityLimitController()
            : base(ControllerType.VelocityLimitController)
        {
            MaxLinearVelocity = Settings.MaxTranslation;
            MaxAngularVelocity = Settings.MaxRotation;
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="VelocityLimitController" /> class.
        /// Pass in 0 or Fix64.MaxValue to disable the limit.
        /// maxAngularVelocity = 0 will disable the angular velocity limit.
        /// </summary>
        /// <param name="maxLinearVelocity">The max linear velocity.</param>
        /// <param name="maxAngularVelocity">The max angular velocity.</param>
        public VelocityLimitController(Fix64 maxLinearVelocity, Fix64 maxAngularVelocity)
            : base(ControllerType.VelocityLimitController)
        {
            if (maxLinearVelocity == Fix64.Zero || maxLinearVelocity == Fix64.MaxValue)
                LimitLinearVelocity = false;

            if (maxAngularVelocity == Fix64.Zero || maxAngularVelocity == Fix64.MaxValue)
                LimitAngularVelocity = false;

            MaxLinearVelocity = maxLinearVelocity;
            MaxAngularVelocity = maxAngularVelocity;
        }

        /// <summary>
        /// Gets or sets the max angular velocity.
        /// </summary>
        /// <value>The max angular velocity.</value>
        public Fix64 MaxAngularVelocity
        {
            get { return _maxAngularVelocity; }
            set
            {
                _maxAngularVelocity = value;
                _maxAngularSqared = _maxAngularVelocity * _maxAngularVelocity;
            }
        }

        /// <summary>
        /// Gets or sets the max linear velocity.
        /// </summary>
        /// <value>The max linear velocity.</value>
        public Fix64 MaxLinearVelocity
        {
            get { return _maxLinearVelocity; }
            set
            {
                _maxLinearVelocity = value;
                _maxLinearSqared = _maxLinearVelocity * _maxLinearVelocity;
            }
        }

        public override void Update(Fix64 dt)
        {
            foreach (Body body in _bodies)
            {
                if (!IsActiveOn(body))
                    continue;

                if (LimitLinearVelocity)
                {
                    //Translation
                    // Check for large velocities.
                    Fix64 translationX = dt * body._linearVelocity.X;
                    Fix64 translationY = dt * body._linearVelocity.Y;
                    Fix64 result = translationX * translationX + translationY * translationY;

                    if (result > dt * _maxLinearSqared)
                    {
                        Fix64 sq = Fix64.Sqrt(result);

                        Fix64 ratio = _maxLinearVelocity / sq;
                        body._linearVelocity.X *= ratio;
                        body._linearVelocity.Y *= ratio;
                    }
                }

                if (LimitAngularVelocity)
                {
                    //Rotation
                    Fix64 rotation = dt * body._angularVelocity;
                    if (rotation * rotation > _maxAngularSqared)
                    {
                        Fix64 ratio = _maxAngularVelocity / Fix64.Abs(rotation);
                        body._angularVelocity *= ratio;
                    }
                }
            }
        }

        public void AddBody(Body body)
        {
            _bodies.Add(body);
        }

        public void RemoveBody(Body body)
        {
            _bodies.Remove(body);
        }
    }
}
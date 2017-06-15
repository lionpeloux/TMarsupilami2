using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilami.MathLib
{
    /// <summary>
    /// This class offers methods to integrate frames given their velocity :
    /// dx/dt = Vx = VX_k dk
    /// d/ds (di) = Vθ x di
    /// t could be any parameter. Could be time of arc length.
    /// </summary>
    public static class Motion
    {

        // v and ϰ in local coordinates
        public static MFrame Move(MFrame frame, double dt, MVector Vx, MVector Vθ)
        {   
            // extension ratio
            var speed = Vx.Length();
            var ε = speed - 1;

            // unit tangent in LCS
            var t = Vx / speed;

            // extension
            var dl = speed * dt;

            // curvature binormal
            var κ1 = Vθ.X / speed;
            var κ2 = Vθ.Y / speed;
            var κb = new MVector(κ1, κ2, 0); // in LCS
            var κ = Math.Sqrt(κ1 * κ1 + κ2 * κ2);

            MVector dx;
            MFrame movedFrame = frame;

            // TRANSLATION
            if (κ == 0) // LINE
            {
                dx = dl * t;
            }
            else // CIRCLE
            {
                var r = 1 / κ;
                var b = κb / κ; // in LCS
                var n = MVector.CrossProduct(b, t); // in LCS
                var dα = dl / r; // sur le cercle
                dα = 2 * Math.Asin(0.5 * dl / r); // à la chorde
                dx = r * Math.Sin(dα) * t + r * (1 - Math.Cos(dα)) * n;
            }

            movedFrame.Origin += BasisChange.ToGlobal(dx, frame);

            // ROTATION
            var dβ = Vθ.Length();
            if (dβ != 0)
            {
                var axis = Vθ / dβ;
                movedFrame = Rotation.Rotate(movedFrame, dt * dβ, axis);
            }
            
            return movedFrame;
        }
    }
}

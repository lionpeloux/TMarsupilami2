using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilami.MathLib
{
    public static class Interpolation
    {

        public static void Quadratic(   double l0, double l1, double V0, double V1, double V2, 
                                        out double dV0, out double dV1, out double dV2,
                                        out double dV01, out double dV12,
                                        out double V01, out double V12
                                    )
        {
            var s = (l0 + l1);
            var q0 = l0 / s;
            var q1 = l1 / s;

            // derivative at mid points
            dV01 = (V1 - V0) / l0;  // s = -l0/2
            dV12 = (V2 - V1) / l1;  // s = +l1/2

            // derivative at vertices
            dV0 = (2 * q0 + q1) * dV01 - q0            * dV12;  // s = -l0
            dV1 = q1            * dV01 + q0            * dV12;  // s = 0
            dV2 = -q1           * dV01 + (q0 + 2 * q1) * dV12;  // s = +l1

            // value at mid points
            V01 =  (0.25 * (q0 + 2 * q1))   * V0 + (0.25 * (l0 + 2 * l1) / l1)  * V1 - (0.25 * q0 * (l0 / l1))   * V2;  // s = -l0/2
            V12 = -(0.25 * q1 * (l1 / l0))  * V0 + (0.25 * (2 * l0 + l1) / l0)  * V1 + (0.25 * (2 * q0 + q1))    * V2;  // s = +l1/2
        }

        public static void Quadratic(double l0, double l1, double V0, double V1, double V2,
                                        out double V01, out double V12
                                    )
        {
            var s = (l0 + l1);
            var q0 = l0 / s;
            var q1 = l1 / s;

            // value at mid points
            V01 = (0.25 * (q0 + 2 * q1)) * V0 + (0.25 * (l0 + 2 * l1) / l1) * V1 - (0.25 * q0 * (l0 / l1)) * V2;  // s = -l0/2
            V12 = -(0.25 * q1 * (l1 / l0)) * V0 + (0.25 * (2 * l0 + l1) / l0) * V1 + (0.25 * (2 * q0 + q1)) * V2;  // s = +l1/2
        }

        public static void Quadratic(   double l0, double l1, 
                                        MVector V0, MVector V1, MVector V2,
                                        out MVector dV0, out MVector dV1, out MVector dV2,
                                        out MVector dV01, out MVector dV12,
                                        out MVector V01, out MVector V12
                                    )
        {
            var s = (l0 + l1);
            var q0 = l0 / s;
            var q1 = l1 / s;

            // derivative at mid points
            dV01 = (V1 - V0) / l0;  // s = -l0/2
            dV12 = (V2 - V1) / l1;  // s = +l1/2

            // derivative at vertices
            dV0 = (2 * q0 + q1) * dV01 - q0 * dV12;  // s = -l0
            dV1 = q1 * dV01 + q0 * dV12;  // s = 0
            dV2 = -q1 * dV01 + (q0 + 2 * q1) * dV12;  // s = +l1

            // value at mid points
            V01 = (0.25 * (q0 + 2 * q1)) * V0 + (0.25 * (l0 + 2 * l1) / l1) * V1 - (0.25 * q0 * (l0 / l1)) * V2;  // s = -l0/2
            V12 = -(0.25 * q1 * (l1 / l0)) * V0 + (0.25 * (2 * l0 + l1) / l0) * V1 + (0.25 * (2 * q0 + q1)) * V2;  // s = +l1/2
        }
    }
}

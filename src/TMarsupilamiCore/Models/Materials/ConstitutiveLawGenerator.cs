using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilamiCore.Materials
{

    /// <summary>
    /// The type of a constitutive law. Predefined laws can be generated through the static class ConstitutiveLawGenerator.
    /// </summary>
    /// <remarks>Users can define any kind of law via a lamda expression. Such a law should be marked as "Custom".</remarks>
    public enum ConstitutiveLawType
    {
        SymConstant,

        SymLinear,
        AsymLinear,

        SymBiLinear,
        AsymBiLinear,

        SymMultiLinear,
        AsymMultiLinear,

        Custom,
    }

    /// <summary>
    /// Defines a generic law : x -> F(x)
    /// </summary>
    /// <param name="x">Input value : x.</param>
    /// <param name="y">Output value : y=F(x).</param>
    public delegate void Law_F(double x, out double y);

    /// <summary>
    /// Defines a generic law  F : x -> F(x), F'(x)
    /// </summary>
    /// <param name="x">Input value : x.</param>
    /// <param name="y">Output value : y=F(x).</param>
    /// <param name="dy">First derivative : dy/dx = F'(x).</param>
    public delegate void Law_FdF(double x, out double y, out double dy);

    /// <summary>
    /// This is a utility class to 
    /// </summary>
    public static class ConstitutiveLawGenerator
    {
        /// <summary>
        /// A symetric constant law.
        /// </summary>
        /// <param name="y0">The constant value of the law.</param>
        /// <param name="F">The law | F : x -> F(x).</param>
        /// <param name="dF">The first derivative of the law | dF : x -> F'(x).</param>
        /// <param name="FdF">A function that evaluates both the law and its first derivative | FdF : x -> (F(x), F'(x)).</param>
        public static ConstitutiveLawType Make_Sym_Constant(double y0, out Law_F F, out Law_F dF, out Law_FdF FdF)
        {
            F = (double x, out double y) => {
                y = y0;
            };

            dF = (double x, out double dy) => {
                dy = 0;
            };

            FdF = (double x, out double y, out double dy) => {
                y = y0;
                dy = 0;
            };

            return ConstitutiveLawType.SymConstant;
        }

        /// <summary>
        /// A symetric linear law.
        /// </summary>
        /// <param name="dy0">The constant slope of the law.</param>
        /// <param name="F">The law | F : x -> F(x).</param>
        /// <param name="dF">The first derivative of the law | dF : x -> F'(x).</param>
        /// <param name="FdF">A function that evaluates both the law and its first derivative | FdF : x -> (F(x), F'(x)).</param>
        public static ConstitutiveLawType Make_Sym_Linear(double dy0, out Law_F F, out Law_F dF, out Law_FdF FdF)
        {
            F = (double x, out double y) => {
                y = dy0 * x;
            };

            dF = (double x, out double dy) => {
                dy = dy0;
            };

            FdF = (double x, out double y, out double dy) => {
                y = dy0 * x;
                dy = dy0;
            };

            return ConstitutiveLawType.SymLinear;
        }

        /// <summary>
        /// Asymmetric linear law | F(x<x0)=_dy0*x | F(x>=x0)=dy0*x
        /// </summary>
        /// <param name="x0">The abscisse where the slope changes from _dy0 to dy0</param>
        /// <param name="_dy0">The slope for x < x0</param>
        /// <param name="dy0">The slope for x >= x0</param>
        /// <param name="F">The law | F : x -> F(x).</param>
        /// <param name="dF">The first derivative of the law | dF : x -> F'(x).</param>
        /// <param name="FdF">A function that evaluates both the law and its first derivative | FdF : x -> (F(x), F'(x)).</param>
        public static ConstitutiveLawType Make_ASym_Linear(double x0, double _dy0, double dy0, out Law_F F, out Law_F dF, out Law_FdF FdF)
        {
            F = (double x, out double y) => {
                if (x < x0)
                {
                    y = _dy0 * x;
                }
                else
                {
                    y = dy0 * x;
                }         
            };

            dF = (double x, out double dy) => {
                if(x < x0)
                {
                    dy = _dy0;
                }
                else
                {
                    dy = dy0;
                }
            };

            FdF = (double x, out double y, out double dy) => {
                if (x < x0)
                {
                    y = _dy0 * x;
                    dy = _dy0;
                }
                else
                {
                    y = dy0 * x;
                    dy = dy0;
                }
            };

            return ConstitutiveLawType.AsymLinear;
        }
    }
}

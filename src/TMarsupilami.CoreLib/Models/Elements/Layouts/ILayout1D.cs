using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.MathLib;
using TMarsupilamiCore.Elements;
using TMarsupilamiCore.Materials;
using TMarsupilamiCore.Sections;

namespace TMarsupilami.CoreLib.Element
{
    interface ILayout1D : ILayout
    {
        /// <summary>
        /// Returns true if the element loops over itself (Nv = Nn).
        /// Returns false otherwise (Nv = Nn-1).
        /// </summary>
        bool IsClosed { get; }

        /// <summary>
        /// Returns true if all edge sections are uniform across the element.
        /// </summary>
        /// <remarks>Use this property for fast implementation flavors.</remarks>
        bool HasUniformSection { get; }

        /// <summary>
        /// Returns true if all edge materials are uniform across the element.
        /// </summary>
        /// <remarks>Use this property for fast implementation flavors.</remarks>
        bool HasUniformMaterial { get; }

        /// <summary>
        /// Gets the number of element vertices.
        /// </summary>
        int Nv { get; }

        /// <summary>
        /// Gets the number of element edges.
        /// </summary>
        int Ne { get; }

        /// <summary>
        /// Returns the array of element edge sections.
        /// </summary>
        Section[] Sections { get; }
             
        /// <summary>
        /// Returns the array of element edge materials.
        /// </summary>
        Material[] Materials { get; }
           
        ///// <summary>
        ///// Gets the internal force at a given vertex.
        ///// </summary>
        ///// <param name="index">The vertex index.</param>
        ///// <param name="F">A tuple with the left and right values of the internal force, as it might be discontinuous.</param>
        ///// <param name="cs">The coordinate system in which the result is given. Optional parameter default to "Global".</param>
        ///// <returns>True on success. False if the evaluation fails.</returns>
        //bool Fint(int index, out Tuple<MVector, MVector> F, CoordinateSystem cs = CoordinateSystem.Global);

        ///// <summary>
        ///// Gets the internal moment at a given vertex.
        ///// </summary>
        ///// <param name="index">The vertex index.</param>
        ///// <param name="M">A tuple with the left and right values of the internal moment, as it might be discontinuous.</param>
        ///// <param name="cs">The coordinate system in which the result is given. Optional parameter default to "Global".</param>
        ///// <returns>True on success. False if the evaluation fails.</returns>
        //bool Mint(int index, out Tuple<MVector, MVector> M, CoordinateSystem cs = CoordinateSystem.Global);
    }
}

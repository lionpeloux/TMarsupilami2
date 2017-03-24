using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.MathLib;
using TMarsupilamiCore.Materials;
using TMarsupilamiCore.Sections;

namespace TMarsupilamiCore.Elements
{
    enum ElementType
    {

    }

    enum SpatialDimension
    {
        One, // beam, cable, truss, tie, strut
        Two, // membrane, plate, shell
    }

    /// <summary>
    ///  Types of coordinate systems. 
    ///  Can be either the material coordinate system (t, d1, d2) or the world coordinate system (x, y, z).
    /// </summary>
    public enum CoordinateSystem
    {
        Material,
        Global,
    }

    /// <summary>
    /// Labels of material coordinate system axis.
    /// </summary>
    public enum MaterialAxis
    {
        d1,
        d2,
        t,
    }

    /// <summary>
    /// Labels of global coordinate system axis.
    /// </summary>
    public enum GlobalAxis
    {
        x,
        y,
        z
    }


    public abstract class Element_1D
    {
        protected int nv;
        protected int ne;
        protected Material[] materials;
        protected Section[] sections;

        // Beam

        protected MFrame[] mFrames;

        // Or cable, bar, 

        protected MPoint[]  x;
        protected MVector[] t;

        #region PROPERTIES

        /// <summary>
        /// Gets the number of element vertices.
        /// </summary>
        public int Nv { get; protected set; }

        /// <summary>
        /// Gets the number of element edges.
        /// </summary>
        public int Ne { get; protected set; }

        /// <summary>
        /// Returns true if the element loops over itself (Nv = Nn).
        /// Returns false otherwise (Nv = Nn-1).
        /// </summary>
        public bool IsClosed { get; protected set; }

        /// <summary>
        /// Returns true if all edge sections are uniform across the element.
        /// </summary>
        /// <remarks>Use this property for fast implementation flavors.</remarks>
        public bool HasUniformSection { get; protected set; }

        /// <summary>
        /// Returns true if all edge materials are uniform across the element.
        /// </summary>
        /// <remarks>Use this property for fast implementation flavors.</remarks>
        public bool HasUniformMaterial { get; protected set; }
  
        /// <summary>
        /// Gets the array of edge matrials.
        /// </summary>
        public Material[] Materials { get; protected set; }

        /// <summary>
        /// Gets the array of edge sections.
        /// </summary>
        public Section[] Sections { get; protected set; }

        /// <summary>
        /// Gets the internal force at at arclength s.
        /// </summary>
        /// <param name="s">The arclength.</param>
        /// <returns>The internal force applied from the right to the left.</returns>
        public abstract MVector F(double s);

        /// <summary>
        /// Gets the internal force at at arclength s.
        /// </summary>
        /// <param name="s">The arclength.</param>
        /// <returns>The internal force applied from the right to the left.</returns>
        public abstract MVector F(double s);

        // This is the "PPCD" of all elements
        // Just what is needeed / the minimal representation shared by any element
        // + some instance methods that returns 

        /* Un élément peu être traité par plusieurs modèles :
         *   - un cable peut aussi bien être traité par un modèle qui ne prend en compte que 
         *     un comportement non linéaire axial
         *   - mais il peut aussi bien être traité par un modèle de poutre avec une inertie de flexion nulle
         *   - La non linéarité axiale peut se gérer au niveau du modèle mais aussi de manière détrounée
         *     en jouant sur la loi de comportement du matériau associé (par exemple un module nul pour une compression)
         *    
         *  De même, un même modèle peut se décliner dans des versions "spécialisées" pour des raisons de performance et 
         *  qui s'appliquent à des cas typiques et courant.
         *   - Par exemple, si la poutre a un materiau et une section uniforme on pourra mettre en cache ES et EI1, EI2.
         *   - Autre exemple, si la poutre est linéaire rectiligne au repos, les courbures au repos sont nulles, ce qui allège de bcp les calculs.
         */

        // On adopte la représenation la plus minimale possible pour un élément. 
        // 1D (cable, bar, beam) ou 2D (membrane ou shell)

        // attention, le cas de la poutre 3pts est problématique.
        // en faite, il n'y a pas de torsion sous des conditions très spécifiques et en l'absence de 
        // connexions qui transmettent du moment.
        // en pratique, on peut toujours définir une material frame à l'aide du transport parallel (pas de torsion).
        // Mais rappelons nous que courbure + moment => 

        protected MPoint[]    x;        // => pour tous 
        protected MVector[]   t;        // => par pour le cable, la chaine, la membrane
        protected MVector[]   d1, d2;   // => par pour le cable, la chaine, la membrane

        // EXTERNAL FORCES & MOMENTS
        protected MVector[] _Fext_g;    // (Fx, Fy, Fz) given in the global coordinate system (x,y,z)
        protected MVector[] _Mext_m;    // (M1, M2, Q) given in the material coordinate system(d1, d2, t)
        protected MVector[] _fext_g;    // linear external moment
        protected MVector[] _mext_m;    // linear external moment

        // REACTION MOMENT AND FORCES
        // take care about signs : these are the forces and moments applied
        // by the beam to the supports. 
        protected MVector[] _Fr_g;      // (Frx, Fry, Frz) given in the global coordinate system
        protected MVector[] _Mr_m;      // (Mr1, Mr2, Qr) given in the material coordinate system 

        // ============= Normalement on peut s'arréter ici car on a toutes les informations utiles 
        // pour déduire les effors internes sous certaines actions extérieures.

        // Un element doit pouvoir rendre compte de ses proprités géométriques sous un certain chargement.
        // Ces fonctions sont implémentées dans des sous types spécialisés. 
        // Par exemple un cable renvera un tableau de valeurs nulles sous Get_κb()
        public abstract MVector[] Get_κb();
        public abstract MVector[] Get_e(Point[] x);

        


        private Vector[] κb_l, κb_r;     // curvature

        private double[] N;              // Internal axial force at each edge(Ne).
        private Vector[] M_l, M_r;       // Left/Rigth internal bending moment at vertices (Nn).
        private Vector[] Q_l, Q_r;       // Internal torisonal moment à each node

        /// <summary>
        /// Left/Rigth internal torsional moment at vertices (Nn).
        /// </summary>


        // Ces informations sont propres au solver ??
        // => description "performantielle " du Model, un peu comme un cahier des charges
        // décrire les phénomènes que l'élément peut/doit capturer (Traction/Comp , Flexion / Torsion)
        // une base commune de représentation ?
        // une base commune (mais spécialisée par élément) pour calculer les efforts internes et les réactions.
        // en prenant en compte les discontinuitées.

        // Puis il y a une classe de contexte qui fait le dispatch entre le model et le solver pour 
        // instancier les bons éléments. Le mappage des éléments du model vers les élemtns du solver n'est pas forcément unique.
        // on peut avoir plusieurs modèles de poutres qui sont censer prendre en compte les mêmes phénomènes.
        // Il y a donc un choix de l'utilisateur qui doit être pris en compte. Par exemple, dans le cas de la torsion, le modèle à 4Dof ou à 6DOF.


        /// <summary>
        /// Returns true if all edge materials are uniform across the element.
        /// </summary>
        /// <remarks>Use this property for fast implementation flavors.</remarks>
        public bool HasRotationalDof { get; set; }

        /// <summary>
        /// Gets the number of degrees of freedom of this element.
        /// </summary>
        public int DofCount { get; set; }


        // Centerline ? Material Frames
        // Ghost Nodes vs Handle Nodes
        // Has local axes ?

        /*
         * Je dois trouver un formalisme pour faire fonctionner ensemble 
         * des models d'éléments qui n'ont pas le même ordre (3Dof, 4Dof, peut-être 6Dof).
         *
         **/


        // un cable 
        /// <summary>
        /// un cable / une chaine / possède :
        ///     * une centerline
        ///     * pas de tangente aux noeuds(non définie aux noeuds car pas de courbure)
        ///       mais on peut en définir un sur chaque edge
        ///       
        ///     pas de repère materiel d1, d2)
        ///     
        /// </summary>



        /// <summary>
        /// une beam avec torsion possède :
        ///     une centerline avec des fictious verticies
        ///     une tangente à chaque vertex
        ///     un repère materiel d1, d2) à chaque vertex
        /// </summary>

        #endregion

    }
}

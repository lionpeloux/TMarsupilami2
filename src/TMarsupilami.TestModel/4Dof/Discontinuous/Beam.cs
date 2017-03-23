using Rhino.Geometry;
using System;
using TMarsupilami.Gh;
using TMarsupilami.MathLib;
using TMarsupilami.TestModel.Dof4.Discontinuous;

namespace TMarsupilami.TestModel.Dof4.Discontinuous
{
    /// <summary>
    /// Abstract base class for every beam element.
    /// </summary>
    public abstract class Beam : Element
    {
        #region PROPERTIES
        /// <summary>
        /// Beam type.
        /// </summary>
        public BeamType BeamType { get; protected set; }

        #endregion

        #region CONSTRUCTORS
        protected Beam(int id, string name, BeamType type,
                        SectionProperty[] section_prop, MaterialProperty material_prop,
                        bool is_closed, MFrame[] mframe_0, MFrame[] mframe_i)
            : base(id, name, ElementType.Beam, section_prop, material_prop, is_closed, mframe_0, mframe_i)
        {
            BeamType = type;
        }

        public static Beam CreateCurvedBeam(int id, string name, SectionProperty[] section_prop, MaterialProperty material_prop, MFrame[] mframe_0, MFrame[] mframe_i)
        {
            Beam beam = new CurvedBeam(id, name, section_prop, material_prop, mframe_0, mframe_i);
            return beam;
        }
        #endregion

        protected override void SetRestConfig(ref MFrame[] mframe)
        {
            for (int i = 0; i < Nn; i++)
            {
                x_0[i] = mframe[i].Origin;
            }

            // i = 0
            l_0[0] = x_0[0].DistanceTo(x_0[1]);
            ll_0[0] = x_0[0].DistanceTo(x_0[2]);
            e_0[0] = new MVector(x_0[1] - x_0[0]);
            t_0[0] = mframe[0].ZAxis;
            κb_0[0] = -2 / (l_0[0] * l_0[0]) * MVector.CrossProduct(e_0[0], t_0[0]);
            _mframe_0[0] = mframe[0];

            // i = 1, ..., Ne-2
            for (int i = 1; i < Ne - 1; i++)
            {
                l_0[i] = x_0[i].DistanceTo(x_0[i + 1]);
                ll_0[i] = x_0[i].DistanceTo(x_0[i + 2]);
                e_0[i] = new MVector(x_0[i + 1] - x_0[i]);
                t_0[i] = l_0[i] / (l_0[i - 1] * ll_0[i - 1]) * e_0[i - 1] + l_0[i - 1] / (l_0[i] * ll_0[i - 1]) * e_0[i];
                κb_0[i] = 2 * MVector.CrossProduct(e_0[i - 1], e_0[i]) / (l_0[i - 1] * l_0[i] * ll_0[i - 1]);
                //_mframe_0[i] = mframe[i].ParallelTransport(x_0[i], t_0[i]); // enforce d3 colinear to t by parallel transport    

                //_mframe_0[i] = (mframe[i].Cast()).ParallelTransport(x_0[i].Cast(), t_0[i].Cast()).Cast();
                ParallelTransportation.ZPT_Rotation(mframe[i], mframe[i].ZAxis, x_0[i], t_0[i], ref _mframe_0[i]);

            }

            // i = Ne - 1
            l_0[Ne - 1] = x_0[Nn - 2].DistanceTo(x_0[Nn - 1]);
            e_0[Ne - 1] = new MVector(x_0[Nn - 1] - x_0[Nn - 2]);
            t_0[Ne - 1] = l_0[Ne - 1] / (l_0[Ne - 2] * ll_0[Ne - 2]) * e_0[Ne - 2] + l_0[Ne - 2] / (l_0[Ne - 1] * ll_0[Ne - 2]) * e_0[Ne - 1];
            κb_0[Ne - 1] = 2 * MVector.CrossProduct(e_0[Ne - 2], e_0[Ne - 1]) / (l_0[Ne - 2] * l_0[Ne - 1] * ll_0[Ne - 2]);

            //_mframe_0[Ne - 1] = mframe[Nn - 2].ParallelTransport(x_0[Nn - 2], t_0[Nn - 2]); // enforce d3 colinear to t by parallel transport    

            //_mframe_0[Ne - 1] = (mframe[Nn - 2].Cast()).ParallelTransport(x_0[Nn - 2].Cast(), t_0[Nn - 2].Cast()).Cast();
            ParallelTransportation.ZPT_Rotation(mframe[Nn - 2], mframe[Nn - 2].ZAxis, x_0[Nn - 2], t_0[Nn - 2], ref _mframe_0[Ne - 1]);

            // i = Ne
            t_0[Nn - 1] = mframe[Nn - 1].ZAxis;
            κb_0[Nn - 1] = 2 / (l_0[Ne - 1] * l_0[Ne - 1]) * MVector.CrossProduct(e_0[Ne - 1], t_0[Nn - 1]);
            _mframe_0[Nn - 1] = mframe[Nn - 1];

            for (int i = 0; i < Nn; i++)
            {
                κ1_0[i] = κb_0[i] * _mframe_0[i].XAxis;
                κ2_0[i] = κb_0[i] * _mframe_0[i].YAxis;
            }
            for (int i = 0; i < Ne; i++)
            {
                //twist_0[i] = _mframe_0[i].GetTwistAngle(_mframe_0[i + 1]);
                //twist_0[i] = (_mframe_0[i].Cast()).GetTwistAngle(_mframe_0[i + 1].Cast());
                twist_0[i] = -Rotation.ZAngle_Rotation(_mframe_0[i], _mframe_0[i].ZAxis, _mframe_0[i + 1], _mframe_0[i + 1].ZAxis);

                τ_0[i] = twist_0[i] / l_0[i];
            }



        }
        protected override void SetInitialConfig(ref MFrame[] mframe)
        {
            // The initial set of material frames impose the starting configuration.
            // Thus ti is not interpolated but immediately given by mframe ZAxis.

            // x, t
            for (int i = 0; i < Nn; i++)
            {
                x[i] = mframe[i].Origin;
                MaterialFrame[i] = mframe[i];
                t[i] = mframe[i].ZAxis;
            }

            // i = 0
            l[0] = x[0].DistanceTo(x[1]);
            ll[0] = x[0].DistanceTo(x[2]);
            e[0] = new MVector(x[1] - x[0]);

            // i = 1, ..., Ne-2
            for (int i = 1; i < Ne - 1; i++)
            {
                l[i] = x[i].DistanceTo(x[i + 1]);
                ll[i] = x[i].DistanceTo(x[i + 2]);
                e[i] = new MVector(x[i + 1] - x[i]);
            }

            // i = Ne - 1
            l[Ne - 1] = x[Nn - 2].DistanceTo(x[Nn - 1]);
            e[Ne - 1] = new MVector(x[Nn - 1] - x[Nn - 2]);
        }

        public void SetInitialConfig2(ref MFrame[] mframe)
        {
            // The initial set of material frames impose the starting configuration.
            // Thus ti is not interpolated but immediately given by mframe ZAxis.

            // on initialise x et MaterialFrame avec les données de départ (on en fait une copie)
            for (int i = 0; i < Nn; i++)
            {
                x[i] = mframe[i].Origin;
                MaterialFrame[i] = mframe[i];
            }

        }

    }


    /// <summary>
    /// CurvedBeam : the more generic type of beam that accounts for both non vanishing curvature and twist
    /// in the beam rest configuration.
    /// </summary>
    public class CurvedBeam : Beam
    {
        // open beam
        // uniform section properties
        // curved and twisted shape in rest configuration 

        #region FIELDS
        private double[] _ES;      // section [m2]
        private double[] _EI1;     // area moment of inertia [m4] around first material axis (d1)
        private double[] _EI2;     // area moment of inertia [m4] around second material axis (d2)
        private double[] _GJ;      // area moment of inertia [m4] around third material axis (d3)    

        private int nn_h;                       // number of handle nodes
        private int ne_h;                       // number of handle edges
        private int nn_g;                       // number of ghost nodes

        public MVector[] t_g;             // tangent vector at a ghost point
        public MVector[] t_h_l, t_h_r;    // tangent vector at left/right of a handle point

        public MVector[] κb_g, κb_h_l, κb_h_r;
        public double[] κ1_g, κ2_g;
        public double[] κ1_h_l, κ1_h_r;
        public double[] κ2_h_l, κ2_h_r;

        public double[] M1_h_l, M1_h_r, M2_h_l, M2_h_r;  // bending moment around d1 material axis : M1 = EI1.(κ1-κ1_0)
        public double[] M1_g, M2_g;  // bending moment around d2 material axis : M2 = EI2.(κ2-κ2_0)
        public MVector[] M_g, M_h_l, M_h_r;

        public double[] Q_l, Q_r;   // torsion moment at left/rigth of each node, ghost and handle

        public MVector[] V_M_g, V_M_h_l, V_M_h_r; // shear (du au moment de flexion) aux ghost et à droite / gauche des handles
        public MVector[] V_Q_g, V_Q_h_l, V_Q_h_r; // shear (du au moment de torsion) aux ghost et à droite / gauche des handles

        public double[] N_l, N_r;   // normal force at left/rigth of each node, ghost and handle


        #endregion

        #region PROPERTIES
        /// <summary>
        /// Axial stiffness.
        /// </summary>
        public double[] ES
        {
            get { return _ES; }
            protected set { _ES = value; }
        }

        /// <summary>
        /// Bending stiffness around first material axis (d1).
        /// </summary>
        public double[] EI1
        {
            get { return _EI1; }
            protected set { _EI1 = value; }
        }

        /// <summary>
        /// Bending stiffness around second material axis (d2).
        /// </summary>
        public double[] EI2
        {
            get { return _EI2; }
            protected set { _EI2 = value; }
        }

        /// <summary>
        /// Torsional stiffness around third material axis (d3).
        /// </summary>
        public double[] GJ
        {
            get { return _GJ; }
            protected set { _GJ = value; }
        }
        #endregion

        #region CONSTRUCTORS
        public CurvedBeam(int id, string name, SectionProperty[] section_prop, MaterialProperty material_prop,
                    MFrame[] mframe_0, MFrame[] mframe_i)
            : base(id, name, BeamType.Curved, section_prop, material_prop, false, mframe_0, mframe_i)
        {
            // Le nombre de mframe doit être impair
            ne_h = Ne / 2;
            nn_h = ne_h + 1;
            nn_g = nn_h - 1;

            t_g = new MVector[nn_g];   // tangente à un ghost node
            t_h_l = new MVector[nn_h];   // tangente à gauche d'un handle node (non défini en 0)
            t_h_r = new MVector[nn_h];   // tangente à droite d'un handle node (non défini en nn_h-1)

            κb_g = new MVector[nn_g];     // courbure 3pts aux ghost nodes
            κ1_g = new double[nn_g];
            κ2_g = new double[nn_g];

            κb_h_l = new MVector[nn_h];   // courbure à gauche d'un handle node (non défini en 0)
            κb_h_r = new MVector[nn_h];   // courbure à droute d'un handle node (non défini en nn_h-1)
            κ1_h_l = new double[nn_h];
            κ1_h_r = new double[nn_h];
            κ2_h_l = new double[nn_h];
            κ2_h_r = new double[nn_h];

            M_g = new MVector[nn_g];   // moment à un ghost node
            M1_g = new double[nn_g];
            M2_g = new double[nn_g];

            M_h_l = new MVector[nn_h];   // moment à gauche d'un handle node
            M1_h_l = new double[nn_h];
            M2_h_l = new double[nn_h];

            M_h_r = new MVector[nn_h];   // moment à gauche d'un handle node
            M1_h_r = new double[nn_h];
            M2_h_r = new double[nn_h];

            Q_l = new double[Nn];
            Q_r = new double[Nn];

            V_M_g = new MVector[nn_g];
            V_M_h_l = new MVector[nn_h];
            V_M_h_r = new MVector[nn_h];

            V_Q_g = new MVector[nn_g];
            V_Q_h_l = new MVector[nn_h];
            V_Q_h_r = new MVector[nn_h];

            N_l = new double[Nn];
            N_r = new double[Nn];

            ES = new double[Ne];
            EI1 = new double[Ne];
            EI2 = new double[Ne];
            GJ = new double[Ne];

            for (int i = 0; i < Ne; i++)
            {
                ES[i] = material_prop.E * section_prop[i].S;
                EI1[i] = material_prop.E * section_prop[i].I1;
                EI2[i] = material_prop.E * section_prop[i].I2;
                GJ[i] = material_prop.G * section_prop[i].J;
            }

            // REST CONFIGURATION
            // pour l'instant, on assume que la courbure est continue dans la configuration au repos
            // il faudra faire la mise à jour de cette fonction pour traitre le cas général
            SetRestConfig(ref mframe_0);

            // Ici, il faut commencer à traiter la discontinuité de courbure.
            SetInitialConfig2(ref mframe_i);

            // Make sure l[i] is computed pour la conversion des efforts linéiques en efforts ponctuels
            GetCenterlineProperties();
        }
        #endregion

        public override string ToString()
        {
            return "[BEAM] : curved beam";
        }

        // GEOMETRY
        /// <summary>
        /// Compute the centerlines properties assumming Mext = 0 and ends are pinned
        /// Constraints may be applied separately to enforced t[0] or t[Nn-1]
        /// </summary>
        public void GetCenterlineProperties()
        {

            // GET GHOST PROPERTIES WITH CIRCLE 3PTS
            for (int i = 0; i < nn_g; i++)
            {
                OsculatingCircle.Circle3Pts(x[2 * i], x[2 * i + 1], x[2 * i + 2],
                    ref κb_g[i],
                    ref e[2 * i], ref e[2 * i + 1],
                    ref l[2 * i], ref l[2 * i + 1],
                    ref d3_mid[2 * i], ref d3_mid[2 * i + 1],
                    ref t_h_r[i], ref t_g[i], ref t_h_l[i + 1]
                    );
                t[2 * i + 1] = t_g[i];
            }

            // GET TANGENT AT HANDLE NODES
            t[0] = e[0] / l[0];
            for (int i = 1; i < nn_h - 1; i++)
            {
                t[2 * i] = t_h_r[i] + t_h_l[i];
                t[2 * i].Normalize();
            }
            t[Nn - 1] = e[Ne - 1] / l[Ne - 1];
        }
        public void GetCurvatureBinormal()
        {
            // Cette fonction n'a plus vraiment de sens
            // Il faut la dispatcher ailleur
            // Elle ne fait plus que le calcul des courbures et tangentes en 
            // début /fin de poutre

            double κ, α;
            Transform xform;

            // i = 0
            // κb[0] = 2 / (l[0] * l[0]) * MVector.CrossProduct(t[0], e[0]);
            // κb is given by applied external moment
            κb_h_r[0] = (-(Mext[0].X - Mr[0].X) / EI1[0]) * MaterialFrame[0].XAxis + (-(Mext[0].Y - Mr[0].Y) / EI2[0]) * MaterialFrame[0].YAxis;
            κ = κb_h_r[0].Length();
            
            if (κ > 0)
            {
                α = -Math.Asin(κ * l[0] / 2);
                xform = Transform.Rotation(α, κb_h_r[0].Cast(), new Point3d());
                //t[0].Transform(xform);

                //var tmp1 = t[0].Cast();
                //tmp1.Transform(xform);
                //t[0] = tmp1.Cast();

                Rotation.Rotate(ref t[0], α, κb_h_r[0] / κ);
            }
            

            // i = Nn-1
            //κb[Nn - 1] = 2 / (l[Ne - 1] * l[Ne - 1]) * MVector.CrossProduct(e[Ne - 1], t[Nn - 1]);
            // κb is given by applied external moment
            κb_h_l[nn_h - 1] = ((Mext[Nn - 1].X - Mr[Nn - 1].X) / EI1[Ne - 1]) * MaterialFrame[Nn - 1].XAxis + ((Mext[Nn - 1].Y - Mr[Nn - 1].Y) / EI2[Ne - 1]) * MaterialFrame[Nn - 1].YAxis;
            κ = κb_h_l[nn_h - 1].Length();
            

            if (κ > 0)
            {
                α = Math.Asin(κ * l[Ne - 1] / 2);
                xform = Transform.Rotation(α, κb_h_l[nn_h - 1].Cast(), new Point3d());
                //t[Nn - 1].Transform(xform);

                //var tmp2 = t[Nn - 1].Cast();
                //tmp2.Transform(xform);
                //t[Nn - 1] = tmp2.Cast();

                Rotation.Rotate(ref t[Nn - 1], α, κb_h_l[nn_h - 1] / κ);
            }
            


        }
        public void UpdateMaterialFrame()
        {
            // i = 0, ..., Nn-1
            for (int i = 0; i < Nn; i++)
            {
                //MaterialFrame[i] = MaterialFrame[i].ParallelTransport(x[i], t[i]); // enforce d3 colinear to t by parallel transport   

                //MaterialFrame[i] = (MaterialFrame[i].Cast()).ParallelTransport(x[i].Cast(), t[i].Cast()).Cast();
                ParallelTransportation.ZPT_Rotation(MaterialFrame[i], MaterialFrame[i].ZAxis, x[i], t[i], ref MaterialFrame[i]);
            }
        }

        // MOMENTS
        /// <summary>
        /// M : M1, M2, κ1, κ2
        /// M is given at each frame/node i
        /// </summary>
        public override void GetBendingMoment()
        {
            MVector d1, d2;

            // GHOST
            for (int i = 0; i < nn_g; i++)
            {
                d1 = MaterialFrame[2 * i + 1].XAxis;
                d2 = MaterialFrame[2 * i + 1].YAxis;

                // attention, EI doit etre uniforme sur [2i,2i+1]
                κ1_g[i] = κb_g[i] * d1;
                κ2_g[i] = κb_g[i] * d2;
                M1_g[i] = EI1[2 * i] * (κ1_g[i] - κ1_0[2 * i + 1]);
                M2_g[i] = EI2[2 * i] * (κ2_g[i] - κ2_0[2 * i + 1]);
                M_g[i] = M1_g[i] * d1 + M2_g[i] * d2;
            }

            // HANDLE

            // i = 0
            d1 = MaterialFrame[0].XAxis;
            d2 = MaterialFrame[0].YAxis;

            κ1_h_r[0] = κb_h_r[0] * d1;
            κ2_h_r[0] = κb_h_r[0] * d2;

            // prendre en compte l'état zéro avec différence left/right
            M1_h_r[0] = EI1[0] * (κ1_h_r[0] - κ1_0[0]);
            M2_h_r[0] = EI2[0] * (κ2_h_r[0] - κ2_0[0]);
            M_h_r[0] = M1_h_r[0] * d1 + M2_h_r[0] * d2;

            // i = 1, ..., nn_h-1
            for (int i = 1; i < nn_h - 1; i++)
            {
                // courbures à gauche et à droite des handle points (à évacuer)
                MVector κb_l = 2 / (l[2 * i - 1] * l[2 * i - 1]) * MVector.CrossProduct(e[2 * i - 1], t[2 * i]);
                MVector κb_r = 2 / (l[2 * i] * l[2 * i]) * MVector.CrossProduct(t[2 * i], e[2 * i]);

                d1 = MaterialFrame[2 * i].XAxis;
                d2 = MaterialFrame[2 * i].YAxis;

                double κb1_l = κb_l * d1;
                double κb2_l = κb_l * d2;
                double κb1_r = κb_r * d1;
                double κb2_r = κb_r * d2;

                // prendre en compte l'état zéro avec différence left/right
                double κb1_l_0 = κ1_0[2 * i];
                double κb2_l_0 = κ2_0[2 * i];
                double κb1_r_0 = κ1_0[2 * i];
                double κb2_r_0 = κ2_0[2 * i];

                // left/right moment that verify the static dondition -Ml + Mr + Mext - Mr = 0
                MVector M;
                M = 0.5 * (EI1[2 * i - 1] * (κb1_l - κb1_l_0) + EI1[2 * i] * (κb1_r - κb1_r_0)) * d1;
                M += 0.5 * (EI2[2 * i - 1] * (κb2_l - κb2_l_0) + EI2[2 * i] * (κb2_r - κb2_r_0)) * d2;

                MVector dM = 0.5 * ((Mext[2 * i].X - Mr[2 * i].X) * d1 + (Mext[2 * i].Y - Mr[2 * i].Y) * d2);
                M_h_l[i] = M + dM;
                M_h_r[i] = M - dM;

                // compute left curvatures from moments
                κ1_h_l[i] = (M_h_l[i] * d1) / EI1[2 * i - 1];
                κ2_h_l[i] = (M_h_l[i] * d2) / EI2[2 * i - 1];
                κb_h_l[i] = κ1_h_l[i] * d1 + κ2_h_l[i] * d2;

                // compute right curvatures from moments
                κ1_h_r[i] = (M_h_r[i] * d1) / EI1[2 * i];
                κ2_h_r[i] = (M_h_r[i] * d2) / EI2[2 * i];
                κb_h_r[i] = κ1_h_r[i] * d1 + κ2_h_r[i] * d2;
            }

            // i = nn_h - 1
            int n = nn_h - 1;
            d1 = MaterialFrame[Nn - 1].XAxis;
            d2 = MaterialFrame[Nn - 1].YAxis;

            κ1_h_l[n] = κb_h_l[n] * d1;
            κ2_h_l[n] = κb_h_l[n] * d2;

            // prendre en compte l'état zéro avec différence left/right
            M1_h_l[n] = EI1[Ne - 1] * (κ1_h_l[n] - κ1_0[Nn - 1]);
            M2_h_l[n] = EI2[Ne - 1] * (κ2_h_l[n] - κ2_0[Nn - 1]);
            M_h_l[n] = M1_h_l[n] * d1 + M2_h_l[n] * d2;

            // calcul des courbures mid edge.
            // ici, faire le calcul avec l'interpolation parabolique
            for (int i = 0; i < ne_h; i++)
            {
                κb_mid[2 * i] = 0.5 * (κb_h_r[i] + κb_g[i]);
                κb_mid[2 * i + 1] = 0.5 * (κb_g[i] + κb_h_l[i + 1]);
            }
        }

        /// <summary>
        /// Q : twist, τ
        /// Q is given at each mid-edge i+1/2
        /// </summary>
        public override void GetTwistingMoment()
        {
            for (int i = 0; i < Ne; i++)
            {
                //twist[i] = MaterialFrame[i].GetTwistAngle(MaterialFrame[i + 1]);
                //twist[i] = (MaterialFrame[i].Cast()).GetTwistAngle(MaterialFrame[i + 1].Cast());
                twist[i] = -Rotation.ZAngle_Rotation(MaterialFrame[i], MaterialFrame[i].ZAxis, MaterialFrame[i + 1], MaterialFrame[i + 1].ZAxis);

                τ[i] = twist[i] / l[i];

                // introduire un twist lié à un qext ?
                Q[i] = GJ[i] * (twist[i] - twist_0[i]) / l[i];
            }

            MVector M_mid;
            double Q_mid, dQ, m3, κM;

            //Interpolation du moment de torsion aux noeuds avec prise en compte de la courbure
            for (int i = 0; i < ne_h; i++)
            {
                // dQ est évalué à partir de Q' + κ1M2 - κ2M1 + m3 = 0 (à l'équilibre statique)
                // et donc Q' = dQ/ds = -m3 - (κ1M2 - κ2M1)
                // on remarque que κb x M = (κ1M2 - κ2M1) * d3

                // 2i
                m3 = mext[2 * i].Z;
                M_mid = 0.5 * (M_h_r[i] + M_g[i]);
                κM = MVector.CrossProduct(κb_mid[2 * i], M_mid) * d3_mid[2 * i];

                Q_mid = Q[2 * i];
                dQ = -0.5 * l[2 * i] * (m3 + κM);
                Q_r[2 * i] = Q_mid - dQ;
                Q_l[2 * i + 1] = Q_mid + dQ;

                // 2i + 1
                m3 = mext[2 * i + 1].Z;
                M_mid = 0.5 * (M_g[i] + M_h_l[i + 1]);
                κM = MVector.CrossProduct(κb_mid[2 * i + 1], M_mid) * d3_mid[2 * i + 1];

                Q_mid = Q[2 * i + 1];
                dQ = -0.5 * l[2 * i + 1] * (m3 + κM);
                Q_r[2 * i + 1] = Q_mid - dQ;
                Q_l[2 * i + 2] = Q_mid + dQ;
            }
        }

        /// <summary>
        /// Rθ_int = Rθ_Q + Rθ_M : is the resulting twisting moment acting on beam nodes due to internal forces and moments.
        /// Rθ_int is given at each node frame/node i.
        /// </summary>
        public override void GetInternalNodalMoment()
        {
            // TWISTING MOMENT | torsion contribution (Q)
            Rθ_Q[0] = Q[0];
            Rθ_Q[1] = -Q[0];
            for (int i = 1; i < Ne - 1; i++)
            {
                Rθ_Q[i] += Q[i];
                Rθ_Q[i + 1] = -Q[i];
            }
            Rθ_Q[Nn - 2] += Q[Ne - 1];
            Rθ_Q[Nn - 1] = -Q[Ne - 1];

            // TWISTING MOMENT | bending contribution (M)
            double m3, κM, dRθ;

            // i = 0
            m3 = mext[0].Z;
            κM = MVector.CrossProduct(κb_h_r[0], M_h_r[0]) * MaterialFrame[0].ZAxis;
            dRθ = 0.5 * (κM + m3) * l[0];
            Rθ_M[0] = dRθ;

            // i = 1
            m3 = mext[0].Z;
            κM = MVector.CrossProduct(κb_mid[0], 0.5 * (M_h_r[0] + M_g[0])) * d3_mid[0];
            dRθ = 0.5 * (κM + m3) * l[0];
            Rθ_M[1] = dRθ;

            for (int i = 0; i < ne_h - 1; i++)
            {
                // 2i + 1
                m3 = mext[2 * i + 1].Z;
                κM = MVector.CrossProduct(κb_mid[2 * i + 1], 0.5 * (M_g[i] + M_h_l[i + 1])) * d3_mid[2 * i + 1];
                dRθ = 0.5 * (κM + m3) * l[2 * i + 1];

                Rθ_M[2 * i + 1] += dRθ;
                Rθ_M[2 * i + 2] = dRθ;

                // 2i + 2
                m3 = mext[2 * i + 2].Z;
                κM = MVector.CrossProduct(κb_mid[2 * i + 2], 0.5 * (M_h_r[i + 1] + M_g[i + 1])) * d3_mid[2 * i + 2];
                dRθ = 0.5 * (κM + m3) * l[2 * i + 2];

                Rθ_M[2 * i + 2] += dRθ;
                Rθ_M[2 * i + 3] = dRθ;
            }

            // i = Nn-2
            m3 = mext[Ne - 1].Z;
            κM = MVector.CrossProduct(κb_mid[Ne - 1], 0.5 * (M_g[nn_g - 1] + M_h_l[nn_h - 1])) * d3_mid[Ne - 1];
            dRθ = 0.5 * (κM + m3) * l[Ne - 1];
            Rθ_M[Nn - 2] += dRθ;

            // i = Nn-1
            m3 = mext[Ne - 1].Z;
            κM = MVector.CrossProduct(κb_h_l[nn_h - 1], M_h_l[nn_h - 1]) * MaterialFrame[Nn - 1].ZAxis;
            dRθ = 0.5 * (κM + m3) * l[Ne - 1];
            Rθ_M[Nn - 1] = dRθ;

            // RESULTING TWISTING MOMENT
            for (int i = 0; i < Nn; i++)
            {
                Rθ_int[i] = Rθ_Q[i] + Rθ_M[i];
            }
        }

        public override void GetResultantNodalMoment()
        {
            // RESULTING TWISTING MOMENT
            for (int i = 0; i < Nn; i++)
            {
                double Qext = Mext[i].Z;
                double Qr = Mr[i].Z;
                Rθ[i] = -Qr + Qext + Rθ_int[i];
            }
        }

        // FORCES
        /// <summary>
        /// N : axial force
        /// N is given for each mid edge i+1/2. Its size is Ne.         
        /// </summary>
        public override void GetAxialForce()
        {
            for (int i = 0; i < Ne; i++)
            {
                //double εε = l[i] * l[0] * (κ1[i] * κ1[i] + κ2[i] * κ2[i]) / 24; // second ordre avec la courbure
                //ε[i] = l[i] / l_0[i] - 1 + εε;
                ε[i] = l[i] / l_0[i] - 1;

                // dN est évalué à partir de N' + κ1T2 - κ2T1 + f3 = 0 (à l'équilibre statique)
                // et donc N' = dN/ds = -m3 + κ1T2 - κ2T1
                var Nmid = ES[i] * ε[i];
                N[i] = Nmid;
            }
        }

        /// <summary>
        /// V : shear force
        /// V_M is the bending contribution
        /// V_Q is the twisting contribution
        /// V, V_M and V_Q are given for each mid edge i+1/2. Their size is Ne.
        /// </summary>
        public override void GetShearForce()
        {
            for (int i = 0; i < ne_h; i++)
            {
                // ici, on peut calculer un shear à droite et à gauche des handle
                // et un shear aux ghost
                // le calcul est fait à mi portée
                MVector V_MQ;

                // calcul des dérivées :
                var M0 = M_h_r[i];
                var M1 = M_g[i];
                var M2 = M_h_l[i + 1];

                var l0 = l[2 * i];
                var l1 = l[2 * i + 1];

                var dM01 = (M1 - M0) / l0; // dérivée en i+1/2 selon interpolation parabolique
                var dM12 = (M2 - M1) / l1; // dérivée en i+3/2 selon interpolation parabolique

                V_M[2 * i] = MVector.CrossProduct(d3_mid[2 * i], dM01 + mext[2 * i]);
                V_Q[2 * i] = Q[2 * i] * κb_mid[2 * i];
                V_MQ = -(τ[2 * i] / 2) * (M0 + M1); // manque le terme -τM
                V[2 * i] = V_M[2 * i] + V_Q[2 * i];

                V_M[2 * i + 1] = MVector.CrossProduct(d3_mid[2 * i + 1], dM12 + mext[2 * i + 1]);
                V_Q[2 * i + 1] = Q[2 * i + 1] * κb_mid[2 * i + 1];
                V_MQ = -(τ[2 * i + 1] / 2) * (M1 + M2); // manque le terme -τM
                V[2 * i + 1] = V_M[2 * i + 1] + V_Q[2 * i + 1];
            }


            // Interpolation de l'effort normal aux noeuds avec prise en compte de la courbure
            for (int i = 0; i < Ne; i++)
            {
                // dN est évalué à partir de N' + κ1T2 - κ2T1 + f3 = 0 (à l'équilibre statique)
                // et donc N' = dN/ds = -m3 - (κ1T2 - κ2T1)
                // on remarque que κb x T = (κ1T2 - κ2T1) * d3
                var Nmid = N[i];
                var f3 = fext[i] * d3_mid[i];
                var κT = MVector.CrossProduct(κb_mid[i], V[i]) * d3_mid[i];

                var dN = 0.5 * l[i] * (-f3 - κT);
                N_r[i] = Nmid - dN;
                N_l[i + 1] = Nmid + dN;
            }
        }

        /// <summary>
        /// Rx_int = Rx_axial + Rx_shear_M + Rx_shear_Q : is the resulting force acting on beam nodes due to internal forces and moments.
        /// Rx_int is given at each node frame/node i.
        /// </summary>
        /// 
        public override void GetInternalNodalForce()
        {
            // AXIAL FORCE : attention à ne pas répéter cette opération pour les variations sur θ

            // A revoir avec les nouvelles méthodes d'interpolation des efforts aux noeuds au début/fin de poutre
            // Ne prend pas en compte les efforts linéiques
            Rx_axial[0] = ((N[0] * d3_mid[0] + V_M[0] + V_Q[0]) * t[0]) * t[0];
            Rx_axial[1] = -N[0] * d3_mid[0];
            for (int i = 1; i < Ne - 1; i++)
            {
                MVector force = N[i] * d3_mid[i];
                Rx_axial[i] += force;
                Rx_axial[i + 1] = -force;
            }
            Rx_axial[Nn - 2] += N[Ne - 1] * d3_mid[Ne - 1];
            Rx_axial[Nn - 1] = -((N[Ne - 1] * d3_mid[Ne - 1] + V_M[Ne - 1] + V_Q[Ne - 1]) * t[Nn - 1]) * t[Nn - 1];

            // SHEAR FORCE | bending contribution
            // A revoir avec les nouvelles méthodes d'interpolation des efforts aux noeuds au début/fin de poutre
            // Ne prend pas en compte les efforts linéiques
            Rx_shear_M[0] = (N[0] * d3_mid[0]) - ((N[0] * d3_mid[0] + V_M[0]) * t[0]) * t[0];
            for (int i = 0; i < Ne; i++)
            {
                Rx_shear_M[i] += V_M[i];
                Rx_shear_M[i + 1] = -V_M[i];
            }
            Rx_shear_M[Nn - 1] += -N[Ne - 1] * d3_mid[Ne - 1] + ((N[Ne - 1] * d3_mid[Ne - 1] + V_M[Ne - 1]) * t[Nn - 1]) * t[Nn - 1];

            // SHEAR FORCE | twisting contribution
            Rx_shear_Q[0] = -(V_Q[0] * t[0]) * t[0];
            for (int i = 0; i < Ne; i++)
            {
                Rx_shear_Q[i] += V_Q[i];
                Rx_shear_Q[i + 1] = -V_Q[i];
            }
            Rx_shear_Q[Nn - 1] += (V_Q[nn_h - 1] * t[Nn - 1]) * t[Nn - 1];

            // RESULTANT FORCE
            for (int i = 0; i < Nn; i++)
            {
                Rx_int[i] = Rx_axial[i] + Rx_shear_M[i] + Rx_shear_Q[i];
            }
        }

        public override void GetResultantNodalForce()
        {
            // RESULTANT FORCE
            // ici, prendre en compte l'effort tranchant linéique extérieur
            // celui si créé un déséquilibre qui engendre un moment supplémentaire.
            for (int i = 0; i < Nn; i++)
            {
                Rx[i] = -Fr[i] + Fext[i] + Rx_int[i];
            }
        }

        // ENERGIES
        protected override double GetAxialElasticEnergy()
        {
            double E_axial = 0;
            for (int i = 0; i < Ne; i++)
            {
                E_axial += ES[i] * Math.Pow(ε[i], 2) * l[i];
            }
            E_axial = E_axial / 2;
            return E_axial;
        }
        protected override double GetBendingElasticEnergy()
        {
            double E_bending = 0;
            E_bending += (EI1[0] * Math.Pow((κ1[0] - κ1_0[0]), 2) + EI2[0] * Math.Pow((κ2[0] - κ2_0[0]), 2)) * (l[0] / 4);
            for (int i = 1; i < Nn - 1; i++)
            {
                E_bending += (EI1[i] * Math.Pow((κ1[i] - κ1_0[i]), 2) + EI2[i] * Math.Pow((κ2[i] - κ2_0[i]), 2)) * (l[i - 1] + l[i]) / 4;
            }
            E_bending += (EI1[Ne - 1] * Math.Pow((κ1[Nn - 1] - κ1_0[Nn - 1]), 2) + EI2[Ne - 1] * Math.Pow((κ2[Nn - 1] - κ2_0[Nn - 1]), 2)) * (l[Ne - 1] / 4);
            return E_bending;
        }
        protected override double GetTwistingElasticEnergy()
        {
            double E_twisting = 0;
            for (int i = 0; i < Ne; i++)
            {
                E_twisting += GJ[i] * Math.Pow((τ[i] - τ_0[i]), 2) * l[i];
            }
            E_twisting = E_twisting / 2;
            return E_twisting;
        }

        public void GetShearForceAtNodes()
        {
            // interpolation de l'effort tranchant aux noeuds
            // non nécessaire pour la DR => à décplacer dans l'affichage
            for (int i = 0; i < ne_h; i++)
            {
                // calcul des dérivées :
                var M0 = M_h_r[i];
                var M1 = M_g[i];
                var M2 = M_h_l[i + 1];

                var l0 = l[2 * i];
                var l1 = l[2 * i + 1];

                var dM0 = -(2 * l0 + l1) / (l0 * (l0 + l1)) * M0 + (l0 + l1) / (l0 * l1) * M1 - l0 / (l1 * (l0 + l1)) * M2;
                var dM1 = -l1 / (l0 * (l1 + l0)) * M0 - (l0 - l1) / (l0 * l1) * M1 + l0 / (l1 * (l0 + l1)) * M2;
                var dM2 = l1 / (l0 * (l0 + l1)) * M0 - (l1 + l0) / (l0 * l1) * M1 + (2 * l1 + l0) / (l1 * (l1 + l0)) * M2;


                V_M_h_r[i] = MVector.CrossProduct(MaterialFrame[2 * i].ZAxis, dM0 + mext[2 * i]);
                V_M_g[i] = MVector.CrossProduct(MaterialFrame[2 * i + 1].ZAxis, dM1 + mext[2 * i]);
                V_M_h_l[i + 1] = MVector.CrossProduct(MaterialFrame[2 * i + 2].ZAxis, dM2 + mext[2 * i + 1]);


                V_Q_h_r[i] = Q_r[2 * i] * κb_h_r[i];
                V_Q_g[i] = 0.5 * (Q_l[2 * i + 1] + Q_r[2 * i + 1]) * κb_g[i];
                V_Q_h_l[i + 1] = Q_l[2 * i + 2] * κb_h_l[i + 1]; ;

                V_M_h_r[i] += V_Q_h_r[i];
                V_M_g[i] += V_Q_g[i];
                V_M_h_l[i + 1] += V_Q_h_l[i + 1];
            }
        }
        public MVector GetReactionForce(Boundary boundary)
        {
            if (boundary == Boundary.Start)
            {
                return -(Rx_axial[0] + Rx_shear_M[0] + Rx_shear_Q[0]);
            }
            else
            {
                return -(Rx_axial[Nn - 1] + Rx_shear_M[Nn - 1] + Rx_shear_Q[Nn - 1]);
            }

        }
        public MVector GetReactionMoment(Boundary boundary)
        {
            if (boundary == Boundary.Start)
            {
                return -(M_h_r[0] + Q_r[0] * MaterialFrame[0].ZAxis);
            }
            else
            {
                return (M_h_l[nn_h - 1] + Q_l[Nn - 1] * MaterialFrame[Nn - 1].ZAxis);
            }
        }
        public MVector[] GetMext(CoordinateSystem cs)
        {
            if (cs == CoordinateSystem.Material)
            {
                return this.Mext;
            }
            else
            {
                MVector[] Mext = new MVector[Nn];
                for (int i = 0; i < Nn; i++)
                {
                    Mext[i] = this.Mext[i].X * MaterialFrame[i].XAxis
                                + this.Mext[i].Y * MaterialFrame[i].YAxis
                                + this.Mext[i].Z * MaterialFrame[i].ZAxis;
                }

                return Mext;
            }
        }
        public MVector[] Getmext(CoordinateSystem cs)
        {
            if (cs == CoordinateSystem.Material)
            {
                return this.mext;
            }
            else
            {
                MVector[] mext = new MVector[Ne];
                for (int i = 0; i < Ne; i++)
                {
                    mext[i] = this.mext[i].X * MaterialFrame[i].XAxis
                                + this.mext[i].Y * MaterialFrame[i].YAxis
                                + this.mext[i].Z * MaterialFrame[i].ZAxis;
                }

                return mext;
            }
        }
        public MVector[] GetFext(CoordinateSystem cs)
        {
            if (cs == CoordinateSystem.Global)
            {
                return this.Fext;
            }
            else
            {
                MVector[] Fext = new MVector[Nn];
                for (int i = 0; i < Nn; i++)
                {
                    Fext[i].X = this.Fext[i] * MaterialFrame[i].XAxis;
                    Fext[i].Y = this.Fext[i] * MaterialFrame[i].YAxis;
                    Fext[i].Z = this.Fext[i] * MaterialFrame[i].ZAxis;
                }
                return Fext;
            }
        }
        public MVector[] Getfext(CoordinateSystem cs)
        {
            if (cs == CoordinateSystem.Global)
            {
                return this.fext;
            }
            else
            {
                MVector[] Fext = new MVector[Nn];
                for (int i = 0; i < Nn; i++)
                {
                    Fext[i].X = this.Fext[i] * MaterialFrame[i].XAxis;
                    Fext[i].Y = this.Fext[i] * MaterialFrame[i].YAxis;
                    Fext[i].Z = this.Fext[i] * MaterialFrame[i].ZAxis;
                }
                return Fext;
            }
        }

        public MVector[] GetFr(CoordinateSystem cs)
        {
            if (cs == CoordinateSystem.Global)
            {
                return this.Fr;
            }
            else
            {
                MVector[] Fr = new MVector[Nn];
                for (int i = 0; i < Nn; i++)
                {
                    Fr[i].X = this.Fr[i] * MaterialFrame[i].XAxis;
                    Fr[i].Y = this.Fr[i] * MaterialFrame[i].YAxis;
                    Fr[i].Z = this.Fr[i] * MaterialFrame[i].ZAxis;
                }
                return Fr;
            }
        }
        public MVector[] GetMr(CoordinateSystem cs)
        {
            if (cs == CoordinateSystem.Material)
            {
                return this.Mr;
            }
            else
            {
                MVector[] Mr = new MVector[Nn];
                for (int i = 0; i < Nn; i++)
                {
                    Mr[i] = this.Mr[i].X * MaterialFrame[i].XAxis
                                + this.Mr[i].Y * MaterialFrame[i].YAxis
                                + this.Mr[i].Z * MaterialFrame[i].ZAxis;
                }
                return Mr;
            }
        }


        // DR SPECIFIC (A DEPLACER DANS UNE INTERFACE)


        // DR LUMPED MASS
        public void Update_lm_x(ref double[] lm_x)
        {
            double lm;
            lm_x[0] = 0.0;
            for (int i = 0; i < Nn - 1; i++)
            {
                lm = 0.5 * (ES[i] / l_0[i] + 1.5 * Math.Abs(N[i]) / l[i]);
                lm_x[i] += lm;
                lm_x[i + 1] = lm;
            }
        }
        public void Update_lm_θ(ref double[] lm_θ)
        {
            //Rhino.RhinoApp.WriteLine("update lm_θ");
            double lm;
            lm_θ[0] = 0.0;
            for (int i = 0; i < Ne; i++)
            {
                lm = 0.5 * (GJ[i] / l_0[i] + 1.5 * Math.Abs(Q[i]) / l[i]);
                lm_θ[i] += lm;
                lm_θ[i + 1] = lm;
            }
        }
    }



}

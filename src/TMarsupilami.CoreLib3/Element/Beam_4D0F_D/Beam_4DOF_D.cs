using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.Event;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib3
{
    public enum Boundary
    {
        Start,
        End,
    }


    public sealed class Beam_4DOF_D : Beam
    {
        // Fext_g : beam load buffer 0
        // fext_g : beam load buffer 1
        // Mext_g : beam load buffer 2
        // mext_g : beam load buffer 3
        // Fext_m : beam load buffer 4
        // fext_m : beam load buffer 5
        // Mext_m : beam load buffer 6
        // mext_m : beam load buffer 7

        public MFrame[] MaterialFrames
        {
            get { return mframes; }
        }

        #region FIELDS
        private int ne_h;                // number of handle edges

        // REST CONFIGURATION      
        private double[] l_0;            // rest length of edges
        private double[] κ1_0, κ2_0;     // material curvature in rest configuration
        private double[] twist_0, τ_0;   // twist angle and rate of twist (τ) in rest configuration

        // ACTUAL CONFIGURATION
        private MPoint[] x;                 // centerline : x[i]
        //private MVector[] e;                // edge : e[i] = x[i+1] - x[i]
        //private double[] l;                 // edge length : l[i] = |e[i]|
        private double[] ll;                // ll[i] = |e[i] + e[i+1]|

        private MVector[] t;                // tangent vector at a handle vertex
        private MVector[] d3_mid;           // tangent vector at 
        private MVector[] t_g;              // tangent vector at a ghost vertex
        private MVector[] t_h_l, t_h_r;     // tangent vector at left/right of a handle vertex

        private double[] ε;                 // axial strain : ε = l[i]/l[i]_0 - 1

        private MVector[] κb;               // curvature binormal at ghost vertex
        private MVector[] κb_mid;           // curvature binormal at mid-edge
        private MVector[] κb_g;             // curvature at a ghost vertex
        private MVector[] κb_h_l, κb_h_r;   // curvature at a left/right of a handle vertex

        private double[] κ1, κ2;            // material curvature : κ1 =  κb.d1 | κ2 =  κb.d1
        private double[] κ1_g, κ2_g;
        private double[] κ1_h_l, κ1_h_r;
        private double[] κ2_h_l, κ2_h_r;

        public double[] twist, τ;           // twist angle and rate of twist : τ[i] = twist[i] / l[i]

        // INTERNAL FORCES & MOMENTS
        private double[] N;                                 // axial force : N = ES.ε
        private double[] N_l, N_r;                          // axial force at left/rigth of each node, ghost and handle

        private MVector[] V_M;                              // shear force (bending contribution) : V_M = M'
        private MVector[] V_Q;                              // shear force (torsion contribution) : V_Q = Qκb
        private MVector[] V;                                // total shear force : V = V_M + V_Q
        private MVector[] V_M_g, V_M_h_l, V_M_h_r;          // shear (du au moment de flexion) aux ghost et à droite / gauche des handles
        private MVector[] V_Q_g, V_Q_h_l, V_Q_h_r;          // shear (du au moment de torsion) aux ghost et à droite / gauche des handles

        private double[] M1_h_l, M1_h_r, M2_h_l, M2_h_r;    // bending moment around d1 material axis : M1 = EI1.(κ1-κ1_0)
        private double[] M1_g, M2_g;                        // bending moment around d2 material axis : M2 = EI2.(κ2-κ2_0)
        private MVector[] M_g, M_h_l, M_h_r;

        private double[] Q;                                 // torsional moment : Q = GJ.(τ-τ_0)
        private double[] Q_l, Q_r;                          // torsion moment at left/rigth of each node, ghost and handle

        // RESULTANTE FORCES (DR SPECIFIC ?? => si oui, à déplacer)
        private MVector[] Rint_x_axial, Rint_x_shear_M, Rint_x_shear_Q;
        private double[] Rint_θ_Q, Rint_θ_M;
        
        #endregion

        // pour l'instant, on passe l'ensemble des frames et des sections
        public Beam_4DOF_D(IEnumerable<MFrame> restFrames, IEnumerable<MFrame> actualFrames, IEnumerable<Section> sections, IEnumerable<Material> materials, bool isClosed = false)
            :base(isClosed)
        {

            Update_Loads = new Cluster();

            // DEEP COPY of FRAMES
            mframes_0 = restFrames.ToArray();
            mframes_i = actualFrames.ToArray();
            mframes = actualFrames.ToArray();

            if (mframes_0.Length != mframes_i.Length)
                throw new ArgumentOutOfRangeException("actualFrames", "actualFrames and restFrames must have the same number of items.");

            // TOPOLOGY
            //if (mframes.Length % 2 != 0)
            //    throw new ArgumentOutOfRangeException("Frames", "actualFrames and restFrames must have an even number of items.");

            if (IsClosed)
            {
                ne = mframes.Length;
                nv_g = ne / 2;
                nv_h = ne/2;
                nv = nv_g + nv_h;

                ne_h = nv_g;
            }
            else
            {
                ne = mframes.Length - 1;
                nv_g = ne / 2;
                nv_h = nv_g + 1;
                nv = nv_g + nv_h;

                ne_h = nv_g;
            }

            // DEEP COPY
            this.sections = sections.ToArray();
            this.materials = materials.ToArray();

            // EXTERNAL FORCES & MOMENTS
            Fext_g = new MVector[nv_h];
            Mext_m = new MVector[nv_h];
            fext_g = new MVector[nv_g]; // pas bien pris en compte pour le moment
            mext_m = new MVector[nv_g];   

            // REACTION MOMENT AND FORCES
            Fr_g = new MVector[nv_h];
            Mr_m = new MVector[nv_h];

            // APPLIED LOADS
            loadManager = new BeamLoadManager(this);

            // KINEMATIC CAPABILITIES
            TranslationalDOF = 3;
            RotationalDOF = 1;

            // MECHANICAL CAPABILITIES 
            IsCompressionCapable = true;
            IsTractionCapable = true;
            IsShearCapable = true;
            IsBendingCapable = true;
            IsTorsionCapable = true;

            // DYNAMIC CONFIGURATION
            lm_x = new double[nv];
            lm_θ = new double[nv];
            ld_x = new double[nv];
            ld_θ = new double[nv];
            v_x = new MVector[nv];
            v_θ = new MVector[nv];
            a_x = new MVector[nv];
            a_θ = new MVector[nv];

            // REST CONFIGURATION
            l_0 = new double[ne];
            κ1_0 = new double[nv];
            κ2_0 = new double[nv];
            twist_0 = new double[ne];
            τ_0 = new double[ne];

            // DEFORMED CONFIGURATION
            x = new MPoint[nv];
            e = new MVector[ne];
            l = new double[ne];
            ll = new double[ne - 1];

            t = new MVector[nv];
            d3_mid = new MVector[ne];
            t_g = new MVector[nv_g];   // tangente à un ghost node
            t_h_l = new MVector[nv_h];   // tangente à gauche d'un handle node (non défini en 0)
            t_h_r = new MVector[nv_h];   // tangente à droite d'un handle node (non défini en nv_h-1)

            ε = new double[ne];

            κb = new MVector[nv];
            κb_mid = new MVector[ne];

            κb_g = new MVector[nv_g];     // courbure 3pts aux ghost nodes
            κ1_g = new double[nv_g];
            κ2_g = new double[nv_g];

            κb_h_l = new MVector[nv_h];   // courbure à gauche d'un handle node (non défini en 0)
            κb_h_r = new MVector[nv_h];   // courbure à droute d'un handle node (non défini en nv_h-1)
            κ1_h_l = new double[nv_h];
            κ1_h_r = new double[nv_h];
            κ2_h_l = new double[nv_h];
            κ2_h_r = new double[nv_h];

            κ1 = new double[nv];
            κ2 = new double[nv];
            twist = new double[ne];
            τ = new double[ne];

            // INTERNAL FORCES & MOMENTS
            N = new double[ne];
            N_l = new double[nv];
            N_r = new double[nv];

            V = new MVector[ne];

            V_M = new MVector[ne];
            V_M_g = new MVector[nv_g];
            V_M_h_l = new MVector[nv_h];
            V_M_h_r = new MVector[nv_h];

            V_Q = new MVector[ne];
            V_Q_g = new MVector[nv_g];
            V_Q_h_l = new MVector[nv_h];
            V_Q_h_r = new MVector[nv_h];

            M_g = new MVector[nv_g];   // moment à un ghost node
            M1_g = new double[nv_g];
            M2_g = new double[nv_g];

            M_h_l = new MVector[nv_h];   // moment à gauche d'un handle node
            M1_h_l = new double[nv_h];
            M2_h_l = new double[nv_h];

            M_h_r = new MVector[nv_h];   // moment à gauche d'un handle node
            M1_h_r = new double[nv_h];
            M2_h_r = new double[nv_h];

            Q = new double[ne];
            Q_l = new double[nv];
            Q_r = new double[nv];

            // DR RESULTANT
            R_x = new MVector[nv];
            Rint_x = new MVector[nv];
            Rint_x_axial = new MVector[nv];
            Rint_x_shear_M = new MVector[nv];
            Rint_x_shear_Q = new MVector[nv];
            R_θ = new double[nv];
            Rint_θ = new double[nv];
            Rint_θ_Q = new double[nv];
            Rint_θ_M = new double[nv];

            ES = new double[ne];
            EI1 = new double[ne];
            EI2 = new double[ne];
            GJ = new double[ne];

            for (int i = 0; i < ne; i++)
            {
                var section = this.sections[i];
                var material = this.materials[0];

                ES[i] = material.E * section.S;
                EI1[i] = material.E * section.I1;
                EI2[i] = material.E * section.I2;
                GJ[i] = material.G * section.J;
            }

            // REST CONFIGURATION
            // pour l'instant, on assume que la courbure est continue dans la configuration au repos
            // il faudra faire la mise à jour de cette fonction pour traitre le cas général
            SetRestConfig(mframes_0);

            // Ici, il faut commencer à traiter la discontinuité de courbure.
            SetInitialConfig(mframes);

            // Make sure l[i] is computed pour la conversion des efforts linéiques en efforts ponctuels
            UpdateCenterlineProperties();
        }


        public void SetRestConfig(MFrame[] restFrames)
        {
            var x_0 = new MPoint[nv];       // centerline
            var e_0 = new MVector[ne];      // edge
            var ll_0 = new double[ne];      
            var t_0 = new MVector[nv];      // tangent
            var κb_0 = new MVector[nv];     // curvature binormal
            var frames_0 = new MFrame[nv];

            for (int i = 0; i < nv; i++)
            {
                x_0[i] = restFrames[i].Origin;
            }

            // i = 0
            l_0[0] = x_0[0].DistanceTo(x_0[1]);
            ll_0[0] = x_0[0].DistanceTo(x_0[2]);
            e_0[0] = new MVector(x_0[1] - x_0[0]);
            t_0[0] = restFrames[0].ZAxis;
            κb_0[0] = -2 / (l_0[0] * l_0[0]) * MVector.CrossProduct(e_0[0], t_0[0]);
            frames_0[0] = restFrames[0];

            // i = 1, ..., ne-2
            for (int i = 1; i < ne - 1; i++)
            {
                l_0[i] = x_0[i].DistanceTo(x_0[i + 1]);
                ll_0[i] = x_0[i].DistanceTo(x_0[i + 2]);
                e_0[i] = new MVector(x_0[i + 1] - x_0[i]);
                t_0[i] = l_0[i] / (l_0[i - 1] * ll_0[i - 1]) * e_0[i - 1] + l_0[i - 1] / (l_0[i] * ll_0[i - 1]) * e_0[i];
                κb_0[i] = 2 * MVector.CrossProduct(e_0[i - 1], e_0[i]) / (l_0[i - 1] * l_0[i] * ll_0[i - 1]);
                ParallelTransportation.ZPT_Rotation(restFrames[i], restFrames[i].ZAxis, x_0[i], t_0[i], ref frames_0[i]); // enforce d3 colinear to t by parallel transport    
            }

            // i = ne - 1
            l_0[ne - 1] = x_0[nv - 2].DistanceTo(x_0[nv - 1]);
            e_0[ne - 1] = new MVector(x_0[nv - 1] - x_0[nv - 2]);
            t_0[ne - 1] = l_0[ne - 1] / (l_0[ne - 2] * ll_0[ne - 2]) * e_0[ne - 2] + l_0[ne - 2] / (l_0[ne - 1] * ll_0[ne - 2]) * e_0[ne - 1];
            κb_0[ne - 1] = 2 * MVector.CrossProduct(e_0[ne - 2], e_0[ne - 1]) / (l_0[ne - 2] * l_0[ne - 1] * ll_0[ne - 2]);
            ParallelTransportation.ZPT_Rotation(restFrames[nv - 2], restFrames[nv - 2].ZAxis, x_0[nv - 2], t_0[nv - 2], ref frames_0[ne - 1]); // enforce d3 colinear to t by parallel transport   

            // i = ne
            t_0[nv - 1] = restFrames[nv - 1].ZAxis;
            κb_0[nv - 1] = 2 / (l_0[ne - 1] * l_0[ne - 1]) * MVector.CrossProduct(e_0[ne - 1], t_0[nv - 1]);
            frames_0[nv - 1] = restFrames[nv - 1];

            for (int i = 0; i < nv; i++)
            {
                κ1_0[i] = κb_0[i] * frames_0[i].XAxis;
                κ2_0[i] = κb_0[i] * frames_0[i].YAxis;
            }
            for (int i = 0; i < ne; i++)
            {
                twist_0[i] = -Rotation.ZAngle_Rotation(frames_0[i], frames_0[i].ZAxis, frames_0[i + 1], frames_0[i + 1].ZAxis);
                τ_0[i] = twist_0[i] / l_0[i];
            }
        }
        public void SetInitialConfig(MFrame[] initialFrames)
        {
            // The initial set of material frames impose the starting configuration.
            // Thus ti is not interpolated but immediately given by mframe ZAxis.

            // on initialise x et MaterialFrame avec les donvées de départ (on en fait une copie)
            for (int i = 0; i < nv; i++)
            {
                x[i] = initialFrames[i].Origin;
                mframes[i] = initialFrames[i];
            }
        }

        public override void Move(MVector[] dx)
        {
            // x = x + dx
            for (int i = 0; i < nv; i++)
            {
                x[i] = mframes[i].Origin + dx[i];
            }
        }
        public override void Move(double[] dθ)
        {
            // θ = θ + dθ
            for (int i = 0; i < nv; i++)
            {
                mframes[i].ZRotate(dθ[i]);
            }

            Update_Loads.Call();
        }

        // GEOMETRY
        /// <summary>
        /// Compute the centerlines properties assumming Mext_m = 0 and ends are pinved
        /// Constraints may be applied separately to enforced t[0] or t[nv-1]
        /// </summary>
        public override void UpdateCenterlineProperties()
        {
            // GET GHOST PROPERTIES WITH CIRCLE 3PTS
            for (int i = 0; i < nv_g; i++)
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
            for (int i = 1; i < nv_h - 1; i++)
            {
                t[2 * i] = t_h_r[i] + t_h_l[i];
                t[2 * i].Normalize();
            }
            t[nv - 1] = e[ne - 1] / l[ne - 1];
        }
        public override void UpdateCurvatureBinormal()
        {
            // Cette fonction n'a plus vraiment de sens
            // Il faut la dispatcher ailleur
            // Elle ne fait plus que le calcul des courbures et tangentes en 
            // début /fin de poutre

            double κ, α;

            // i = 0
            // κb[0] = 2 / (l[0] * l[0]) * MVector.CrossProduct(t[0], e[0]);
            // κb is given by applied external moment
            κb_h_r[0] = (-(Mext_m[0].X - Mr_m[0].X) / EI1[0]) * mframes[0].XAxis + (-(Mext_m[0].Y - Mr_m[0].Y) / EI2[0]) * mframes[0].YAxis;
            κ = κb_h_r[0].Length();

            if (κ > 0)
            {
                α = -Math.Asin(κ * l[0] / 2);
                Rotation.Rotate(ref t[0], α, κb_h_r[0] / κ);
            }

            // i = nv-1
            //κb[nv - 1] = 2 / (l[ne - 1] * l[ne - 1]) * MVector.CrossProduct(e[ne - 1], t[nv - 1]);
            // κb is given by applied external moment
            κb_h_l[nv_h - 1] = ((Mext_m[nv_h - 1].X - Mr_m[nv_h - 1].X) / EI1[ne - 1]) * mframes[nv - 1].XAxis + ((Mext_m[nv_h - 1].Y - Mr_m[nv_h - 1].Y) / EI2[ne - 1]) * mframes[nv - 1].YAxis;
            κ = κb_h_l[nv_h - 1].Length();

            if (κ > 0)
            {
                α = Math.Asin(κ * l[ne - 1] / 2);
                Rotation.Rotate(ref t[nv - 1], α, κb_h_l[nv_h - 1] / κ);
            }
        }
        public override void UpdateMaterialFrame()
        {
            // i = 0, ..., nv-1
            for (int i = 0; i < nv; i++)
            {
                ParallelTransportation.ZPT_Rotation(mframes[i], mframes[i].ZAxis, x[i], t[i], ref mframes[i]);
            }
        }

        // MOMENTS
        /// <summary>
        /// M : M1, M2, κ1, κ2
        /// M is given at each frame/node i
        /// </summary>
        public override void UpdateBendingMoment()
        {
            MVector d1, d2;

            // GHOST
            for (int i = 0; i < nv_g; i++)
            {
                d1 = mframes[2 * i + 1].XAxis;
                d2 = mframes[2 * i + 1].YAxis;

                // attention, EI doit etre uniforme sur [2i,2i+1]
                κ1_g[i] = κb_g[i] * d1;
                κ2_g[i] = κb_g[i] * d2;
                M1_g[i] = EI1[2 * i] * (κ1_g[i] - κ1_0[2 * i + 1]);
                M2_g[i] = EI2[2 * i] * (κ2_g[i] - κ2_0[2 * i + 1]);
                M_g[i] = M1_g[i] * d1 + M2_g[i] * d2;
            }

            // HANDLE

            // i = 0
            d1 = mframes[0].XAxis;
            d2 = mframes[0].YAxis;

            κ1_h_r[0] = κb_h_r[0] * d1;
            κ2_h_r[0] = κb_h_r[0] * d2;

            // prendre en compte l'état zéro avec différence left/right
            M1_h_r[0] = EI1[0] * (κ1_h_r[0] - κ1_0[0]);
            M2_h_r[0] = EI2[0] * (κ2_h_r[0] - κ2_0[0]);
            M_h_r[0] = M1_h_r[0] * d1 + M2_h_r[0] * d2;

            // i = 1, ..., nv_h-1
            for (int i = 1; i < nv_h - 1; i++)
            {
                // courbures à gauche et à droite des handle points (à évacuer)
                MVector κb_l = 2 / (l[2 * i - 1] * l[2 * i - 1]) * MVector.CrossProduct(e[2 * i - 1], t[2 * i]);
                MVector κb_r = 2 / (l[2 * i] * l[2 * i]) * MVector.CrossProduct(t[2 * i], e[2 * i]);

                d1 = mframes[2 * i].XAxis;
                d2 = mframes[2 * i].YAxis;

                double κb1_l = κb_l * d1;
                double κb2_l = κb_l * d2;
                double κb1_r = κb_r * d1;
                double κb2_r = κb_r * d2;

                // prendre en compte l'état zéro avec différence left/right
                double κb1_l_0 = κ1_0[2 * i];
                double κb2_l_0 = κ2_0[2 * i];
                double κb1_r_0 = κ1_0[2 * i];
                double κb2_r_0 = κ2_0[2 * i];

                // left/right moment that verify the static dondition -Ml + Mr_m + Mext_m - Mr_m = 0
                MVector M;
                M = 0.5 * (EI1[2 * i - 1] * (κb1_l - κb1_l_0) + EI1[2 * i] * (κb1_r - κb1_r_0)) * d1;
                M += 0.5 * (EI2[2 * i - 1] * (κb2_l - κb2_l_0) + EI2[2 * i] * (κb2_r - κb2_r_0)) * d2;

                MVector dM = 0.5 * ((Mext_m[i].X - Mr_m[i].X) * d1 + (Mext_m[i].Y - Mr_m[i].Y) * d2);
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

            // i = nv_h - 1
            int n = nv_h - 1;
            d1 = mframes[nv - 1].XAxis;
            d2 = mframes[nv - 1].YAxis;

            κ1_h_l[n] = κb_h_l[n] * d1;
            κ2_h_l[n] = κb_h_l[n] * d2;

            // prendre en compte l'état zéro avec différence left/right
            M1_h_l[n] = EI1[ne - 1] * (κ1_h_l[n] - κ1_0[nv - 1]);
            M2_h_l[n] = EI2[ne - 1] * (κ2_h_l[n] - κ2_0[nv - 1]);
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
        public override void UpdateTwistingMoment()
        {
            for (int i = 0; i < Ne; i++)
            {
                //twist[i] = MaterialFrame[i].GetTwistAngle(MaterialFrame[i + 1]);
                //twist[i] = (MaterialFrame[i].Cast()).GetTwistAngle(MaterialFrame[i + 1].Cast());
                twist[i] = -Rotation.ZAngle_Rotation(mframes[i], mframes[i].ZAxis, mframes[i + 1], mframes[i + 1].ZAxis);

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
                m3 = mext_m[i].Z;
                M_mid = 0.5 * (M_h_r[i] + M_g[i]);
                κM = MVector.CrossProduct(κb_mid[2 * i], M_mid) * d3_mid[2 * i];

                Q_mid = Q[2 * i];
                dQ = -0.5 * l[2 * i] * (m3 + κM);
                Q_r[2 * i] = Q_mid - dQ;
                Q_l[2 * i + 1] = Q_mid + dQ;

                // 2i + 1
                m3 = mext_m[i].Z;
                M_mid = 0.5 * (M_g[i] + M_h_l[i + 1]);
                κM = MVector.CrossProduct(κb_mid[2 * i + 1], M_mid) * d3_mid[2 * i + 1];

                Q_mid = Q[2 * i + 1];
                dQ = -0.5 * l[2 * i + 1] * (m3 + κM);
                Q_r[2 * i + 1] = Q_mid - dQ;
                Q_l[2 * i + 2] = Q_mid + dQ;
            }
        }

        /// <summary>
        /// Q : twist, τ
        /// Q is given at each mid-edge i+1/2
        /// </summary>
        public override void UpdateInternalNodalMoment()
        {
            // TWISTING MOMENT | torsion contribution (Q)
            Rint_θ_Q[0] = Q[0];
            Rint_θ_Q[1] = -Q[0];
            for (int i = 1; i < ne - 1; i++)
            {
                Rint_θ_Q[i] += Q[i];
                Rint_θ_Q[i + 1] = -Q[i];
            }
            Rint_θ_Q[nv - 2] += Q[ne - 1];
            Rint_θ_Q[nv - 1] = -Q[ne - 1];

            // TWISTING MOMENT | bending contribution (M)
            double m3, κM, dRθ;

            // i = 0
            m3 = mext_m[0].Z;
            κM = MVector.CrossProduct(κb_h_r[0], M_h_r[0]) * mframes[0].ZAxis;
            dRθ = 0.5 * (κM + m3) * l[0];
            Rint_θ_M[0] = dRθ;

            // i = 1
            m3 = mext_m[0].Z;
            κM = MVector.CrossProduct(κb_mid[0], 0.5 * (M_h_r[0] + M_g[0])) * d3_mid[0];
            dRθ = 0.5 * (κM + m3) * l[0];
            Rint_θ_M[1] = dRθ;

            for (int i = 0; i < ne_h - 1; i++)
            {
                // 2i + 1
                m3 = mext_m[i].Z;
                κM = MVector.CrossProduct(κb_mid[2 * i + 1], 0.5 * (M_g[i] + M_h_l[i + 1])) * d3_mid[2 * i + 1];
                dRθ = 0.5 * (κM + m3) * l[2 * i + 1];

                Rint_θ_M[2 * i + 1] += dRθ;
                Rint_θ_M[2 * i + 2] = dRθ;

                // 2i + 2
                m3 = mext_m[i + 1].Z;
                κM = MVector.CrossProduct(κb_mid[2 * i + 2], 0.5 * (M_h_r[i + 1] + M_g[i + 1])) * d3_mid[2 * i + 2];
                dRθ = 0.5 * (κM + m3) * l[2 * i + 2];

                Rint_θ_M[2 * i + 2] += dRθ;
                Rint_θ_M[2 * i + 3] = dRθ;
            }

            // i = nv-2
            m3 = mext_m[nv_g - 1].Z;
            κM = MVector.CrossProduct(κb_mid[ne - 1], 0.5 * (M_g[nv_g - 1] + M_h_l[nv_h - 1])) * d3_mid[ne - 1];
            dRθ = 0.5 * (κM + m3) * l[ne - 1];
            Rint_θ_M[nv - 2] += dRθ;

            // i = nv-1
            m3 = mext_m[nv_g - 1].Z;
            κM = MVector.CrossProduct(κb_h_l[nv_h - 1], M_h_l[nv_h - 1]) * mframes[nv - 1].ZAxis;
            dRθ = 0.5 * (κM + m3) * l[ne - 1];
            Rint_θ_M[nv - 1] = dRθ;

            // RESULTING TWISTING MOMENT
            for (int i = 0; i < nv; i++)
            {
                Rint_θ[i] = Rint_θ_Q[i] + Rint_θ_M[i];
            }
        }

        public override void UpdateResultantNodalMoment()
        {
            // RESULTING TWISTING MOMENT
            //for (int i = 0; i < nv; i++)
            //{
            //    double Qext = Mext_m[i].Z;
            //    double Qr = Mr_m[i].Z;
            //    R_θ[i] = -Qr + Qext + Rint_θ[i];
            //}

            int index;
            double Qext, Qr;

            // HANDLES

            Qext = Mext_m[0].Z + 0.0 * mext_m[0].Z * l[0];
            Qr = Mr_m[0].Z;
            R_θ[0] = -Qr + Qext + Rint_θ[0];

            for (int i = 1; i < nv_h-1; i++)
            {
                index = 2 * i;
                Qext = Mext_m[i].Z + 0.0 * (mext_m[i - 1].Z * l[index - 1] + mext_m[i].Z * l[index]);
                Qr = Mr_m[i].Z;
                R_θ[index] = -Qr + Qext + Rint_θ[index];
            }

            Qext = Mext_m[nv_h - 1].Z + 0.0 * mext_m[nv_g - 1].Z * l[2 * nv_g - 1];
            Qr = Mr_m[nv_h - 1].Z;
            R_θ[nv - 1] = -Qr + Qext + Rint_θ[nv - 1];

            // GHOSTS (no Qr and Qext)

            for (int i = 0; i < nv_g; i++)
            {
                index = 2 * i + 1;
                Qext = 0.0 * mext_m[i].Z * (l[index - 1] + l[index]);
                R_θ[index] = Qext + Rint_θ[index];
            }
        }

        // FORCES
        /// <summary>
        /// N : axial force
        /// N is given for each mid edge i+1/2. Its size is ne.         
        /// </summary>
        public override void UpdateAxialForce()
        {
            for (int i = 0; i < ne; i++)
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
        /// V, V_M and V_Q are given for each mid edge i+1/2. Their size is ne.
        /// </summary>
        public override void UpdateShearForce()
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

                V_M[2 * i] = MVector.CrossProduct(d3_mid[2 * i], dM01 + mext_m[i]);
                V_Q[2 * i] = Q[2 * i] * κb_mid[2 * i];
                V_MQ = -(τ[2 * i] / 2) * (M0 + M1); // manque le terme -τM
                V[2 * i] = V_M[2 * i] + V_Q[2 * i];

                V_M[2 * i + 1] = MVector.CrossProduct(d3_mid[2 * i + 1], dM12 + mext_m[i]);
                V_Q[2 * i + 1] = Q[2 * i + 1] * κb_mid[2 * i + 1];
                V_MQ = -(τ[2 * i + 1] / 2) * (M1 + M2); // manque le terme -τM
                V[2 * i + 1] = V_M[2 * i + 1] + V_Q[2 * i + 1];
            }


            // Interpolation de l'effort normal aux noeuds avec prise en compte de la courbure
            //for (int i = 0; i < ng; i++)
            //{
            //    // dN est évalué à partir de N' + κ1T2 - κ2T1 + f3 = 0 (à l'équilibre statique)
            //    // et donc N' = dN/ds = -m3 - (κ1T2 - κ2T1)
            //    // on remarque que κb x T = (κ1T2 - κ2T1) * d3
            //    var Nmid = N[i];
            //    var f3 = fext_g[i] * d3_mid[i];
            //    var κT = MVector.CrossProduct(κb_mid[i], V[i]) * d3_mid[i];

            //    var dN = 0.5 * l[i] * (-f3 - κT);
            //    N_r[i] = Nmid - dN;
            //    N_l[i + 1] = Nmid + dN;
            //}

            for (int i = 0; i < 2*nv_g; i++)
            {
                // dN est évalué à partir de N' + κ1T2 - κ2T1 + f3 = 0 (à l'équilibre statique)
                // et donc N' = dN/ds = -m3 - (κ1T2 - κ2T1)
                // on remarque que κb x T = (κ1T2 - κ2T1) * d3
                var Nmid = N[i];
                var f3 = fext_g[i / 2] * d3_mid[i];
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
        public override void UpdateInternalNodalForce()
        {
            // AXIAL FORCE : attention à ne pas répéter cette opération pour les variations sur θ

            // A revoir avec les nouvelles méthodes d'interpolation des efforts aux noeuds au début/fin de poutre
            // ne prend pas en compte les efforts linéiques
            Rint_x_axial[0] = ((N[0] * d3_mid[0] + V_M[0] + V_Q[0]) * t[0]) * t[0];
            Rint_x_axial[1] = -N[0] * d3_mid[0];
            for (int i = 1; i < ne - 1; i++)
            {
                MVector force = N[i] * d3_mid[i];
                Rint_x_axial[i] += force;
                Rint_x_axial[i + 1] = -force;
            }
            Rint_x_axial[nv - 2] += N[ne - 1] * d3_mid[ne - 1];
            Rint_x_axial[nv - 1] = -((N[ne - 1] * d3_mid[ne - 1] + V_M[ne - 1] + V_Q[ne - 1]) * t[nv - 1]) * t[nv - 1];

            // SHEAR FORCE | bending contribution
            // A revoir avec les nouvelles méthodes d'interpolation des efforts aux noeuds au début/fin de poutre
            // ne prend pas en compte les efforts linéiques
            Rint_x_shear_M[0] = (N[0] * d3_mid[0]) - ((N[0] * d3_mid[0] + V_M[0]) * t[0]) * t[0];
            for (int i = 0; i < ne; i++)
            {
                Rint_x_shear_M[i] += V_M[i];
                Rint_x_shear_M[i + 1] = -V_M[i];
            }
            Rint_x_shear_M[nv - 1] += -N[ne - 1] * d3_mid[ne - 1] + ((N[ne - 1] * d3_mid[ne - 1] + V_M[ne - 1]) * t[nv - 1]) * t[nv - 1];

            // SHEAR FORCE | twisting contribution
            Rint_x_shear_Q[0] = -(V_Q[0] * t[0]) * t[0];
            for (int i = 0; i < ne; i++)
            {
                Rint_x_shear_Q[i] += V_Q[i];
                Rint_x_shear_Q[i + 1] = -V_Q[i];
            }
            Rint_x_shear_Q[nv - 1] += (V_Q[nv_h - 1] * t[nv - 1]) * t[nv - 1];

            // RESULTANT FORCE
            for (int i = 0; i < nv; i++)
            {
                Rint_x[i] = Rint_x_axial[i] + Rint_x_shear_M[i] + Rint_x_shear_Q[i];
            }
        }

        public override void UpdateResultantNodalForce()
        {
            // RESULTANT FORCE
            // ici, prendre en compte l'effort tranchant linéique extérieur
            // celui si créé un déséquilibre qui engendre un moment supplémentaire.
            //for (int i = 0; i < nv; i++)
            //{
            //    R_x[i] = -Fr_g[i] + Fext_g[i] + Rint_x[i];
            //}

            int index;
            MVector Fext, Fr;

            //for (int i = 0; i < nv_h; i++)
            //{
            //    index = 2 * i;
            //    R_x[index] = -Fr_g[index] + Fext_g[i] + Rint_x[index];
            //}



            // HANDLES
            Fext = Fext_g[0] + 0.0 * fext_g[0] * l[0];
            Fr = Fr_g[0];
            R_x[0] = -Fr + Fext + Rint_x[0];

            for (int i = 1; i < nv_h - 1; i++)
            {
                index = 2 * i;
                Fext = Fext_g[i] + 0.0 * (fext_g[i - 1] * l[index - 1] + fext_g[i] * l[index]);
                Fr = Fr_g[i];
                R_x[index] = -Fr + Fext + Rint_x[index];
            }

            Fext = Fext_g[nv_h - 1] + 0.0 * fext_g[nv_g - 1] * l[2 * nv_g - 1];
            Fr = Fr_g[nv_h - 1];
            R_x[nv - 1] = -Fr + Fext + Rint_x[nv - 1];

            // GHOSTS

            //for (int i = 0; i < nv_g; i++)
            //{
            //    index = 2 * i + 1;
            //    R_x[index] = -Fr_g[index] + Rint_x[index];
            //}

            for (int i = 0; i < nv_g; i++)
            {
                index = 2 * i + 1;
                Fext = 0.5 * fext_g[i] * (l[index - 1] + l[index]);
                R_x[index] = Fext + Rint_x[index];
            }
        }

        // ENERGIES
        protected double UpdateAxialElasticEnergy()
        {
            double E_axial = 0;
            for (int i = 0; i < ne; i++)
            {
                E_axial += ES[i] * Math.Pow(ε[i], 2) * l[i];
            }
            E_axial = E_axial / 2;
            return E_axial;
        }
        protected double UpdateBendingElasticEnergy()
        {
            double E_bending = 0;
            E_bending += (EI1[0] * Math.Pow((κ1[0] - κ1_0[0]), 2) + EI2[0] * Math.Pow((κ2[0] - κ2_0[0]), 2)) * (l[0] / 4);
            for (int i = 1; i < nv - 1; i++)
            {
                E_bending += (EI1[i] * Math.Pow((κ1[i] - κ1_0[i]), 2) + EI2[i] * Math.Pow((κ2[i] - κ2_0[i]), 2)) * (l[i - 1] + l[i]) / 4;
            }
            E_bending += (EI1[ne - 1] * Math.Pow((κ1[nv - 1] - κ1_0[nv - 1]), 2) + EI2[ne - 1] * Math.Pow((κ2[nv - 1] - κ2_0[nv - 1]), 2)) * (l[ne - 1] / 4);
            return E_bending;
        }
        protected double UpdateTwistingElasticEnergy()
        {
            double E_twisting = 0;
            for (int i = 0; i < ne; i++)
            {
                E_twisting += GJ[i] * Math.Pow((τ[i] - τ_0[i]), 2) * l[i];
            }
            E_twisting = E_twisting / 2;
            return E_twisting;
        }

        public void UpdateShearForceAtNodes()
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


                V_M_h_r[i] = MVector.CrossProduct(mframes[2 * i].ZAxis, dM0 + mext_m[i]);
                V_M_g[i] = MVector.CrossProduct(mframes[2 * i + 1].ZAxis, dM1 + mext_m[i]);
                V_M_h_l[i + 1] = MVector.CrossProduct(mframes[2 * i + 2].ZAxis, dM2 + mext_m[i]);


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
                return -(Rint_x_axial[0] + Rint_x_shear_M[0] + Rint_x_shear_Q[0]);
            }
            else
            {
                return -(Rint_x_axial[nv - 1] + Rint_x_shear_M[nv - 1] + Rint_x_shear_Q[nv - 1]);
            }

        }
        public MVector GetReactionMoment(Boundary boundary)
        {
            if (boundary == Boundary.Start)
            {
                return -(M_h_r[0] + Q_r[0] * mframes[0].ZAxis);
            }
            else
            {
                return (M_h_l[nv_h - 1] + Q_l[nv - 1] * mframes[nv - 1].ZAxis);
            }
        }
        public MVector[] GetMext(CoordinateSystem cs)
        {
            if (cs == CoordinateSystem.Material)
            {
                return this.Mext_m;
            }
            else
            {
                MVector[] Mext_m = new MVector[nv];
                for (int i = 0; i < Mext_m.Length; i++)
                {
                    Mext_m[i] = this.Mext_m[i].X * mframes[i].XAxis
                                + this.Mext_m[i].Y * mframes[i].YAxis
                                + this.Mext_m[i].Z * mframes[i].ZAxis;
                }

                return Mext_m;
            }
        }
        public MVector[] Getmext(CoordinateSystem cs)
        {
            if (cs == CoordinateSystem.Material)
            {
                return this.mext_m;
            }
            else
            {
                MVector[] mext_m = new MVector[ne];
                for (int i = 0; i < mext_m.Length; i++)
                {
                    mext_m[i] = this.mext_m[i].X * mframes[i].XAxis
                                + this.mext_m[i].Y * mframes[i].YAxis
                                + this.mext_m[i].Z * mframes[i].ZAxis;
                }

                return mext_m;
            }
        }
        public MVector[] GetFext(CoordinateSystem cs)
        {
            if (cs == CoordinateSystem.Global)
            {
                return this.Fext_g;
            }
            else
            {
                MVector[] Fext_g = new MVector[nv];
                for (int i = 0; i < nv; i++)
                {
                    Fext_g[i].X = this.Fext_g[i] * mframes[i].XAxis;
                    Fext_g[i].Y = this.Fext_g[i] * mframes[i].YAxis;
                    Fext_g[i].Z = this.Fext_g[i] * mframes[i].ZAxis;
                }
                return Fext_g;
            }
        }
        public MVector[] Getfext(CoordinateSystem cs)
        {
            if (cs == CoordinateSystem.Global)
            {
                return this.fext_g;
            }
            else
            {
                MVector[] Fext_g = new MVector[nv];
                for (int i = 0; i < nv; i++)
                {
                    Fext_g[i].X = this.Fext_g[i] * mframes[i].XAxis;
                    Fext_g[i].Y = this.Fext_g[i] * mframes[i].YAxis;
                    Fext_g[i].Z = this.Fext_g[i] * mframes[i].ZAxis;
                }
                return Fext_g;
            }
        }

        public MVector[] GetFr(CoordinateSystem cs)
        {
            if (cs == CoordinateSystem.Global)
            {
                return this.Fr_g;
            }
            else
            {
                MVector[] Fr_g = new MVector[nv];
                for (int i = 0; i < Fr_g.Length; i++)
                {
                    Fr_g[i].X = this.Fr_g[i] * mframes[i].XAxis;
                    Fr_g[i].Y = this.Fr_g[i] * mframes[i].YAxis;
                    Fr_g[i].Z = this.Fr_g[i] * mframes[i].ZAxis;
                }
                return Fr_g;
            }
        }
        public MVector[] GetMr(CoordinateSystem cs)
        {
            if (cs == CoordinateSystem.Material)
            {
                return this.Mr_m;
            }
            else
            {
                MVector[] Mr_m = new MVector[nv];
                for (int i = 0; i < Mr_m.Length; i++)
                {
                    Mr_m[i] = this.Mr_m[i].X * mframes[i].XAxis
                                + this.Mr_m[i].Y * mframes[i].YAxis
                                + this.Mr_m[i].Z * mframes[i].ZAxis;
                }
                return Mr_m;
            }
        }

        // doit être customizable
        public override void Update_lm_x(ref double[] lm_x)
        {
            double lm;
            lm_x[0] = 0.0;
            for (int i = 0; i < nv - 1; i++)
            {
                lm = 0.5 * (ES[i] / l_0[i] + 1.5 * Math.Abs(N[i]) / l[i]);
                lm_x[i] += lm;
                lm_x[i + 1] = lm;
            }
        }
        public override void Update_lm_θ(ref double[] lm_θ)
        {
            double lm;
            lm_θ[0] = 0.0;
            for (int i = 0; i < ne; i++)
            {
                lm = 0.5 * (GJ[i] / l_0[i] + 1.5 * Math.Abs(Q[i]) / l[i]);
                lm_θ[i] += lm;
                lm_θ[i + 1] = lm;
            }
        }

    }
}

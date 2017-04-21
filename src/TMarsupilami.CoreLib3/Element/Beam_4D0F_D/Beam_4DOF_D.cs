﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.Event;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib3
{
   


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

        // REST CONFIGURATION      
        private double[] l_0;                   // rest length of edges
        private double[] τ_0;                   // twist angle and rate of twist (τ) in rest configuration

        private MVector[] κb_l_0;             // curvature at a ghost vertex
        private MVector[] κb_g_0;             // curvature at a ghost vertex
        private MVector[] κb_r_0;             // curvature at a ghost vertex


        public MVector[] t_mid;           // tangent vector at mid (u = t3_mid = ei/|ei|)
        public MFrame[] mframes_mid;

        private double[] ε;                 // axial strain : ε = l[i]/l[i]_0 - 1
        private double[] ε_l;                          // torsion moment at left/rigth of each node, ghost and handle
        private double[] ε_g;
        private double[] ε_r;                           // torsion moment at left/rigth of each node, ghost and handle

        private MVector[] κb_mid;           // curvature binormal at mid-edge
        private MVector[] κb_l;             // curvature at a ghost vertex
        private MVector[] κb_g;             // curvature at a ghost vertex
        private MVector[] κb_r;             // curvature at a ghost vertex

        private double[] τ;           // twist angle and rate of twist : τ[i] = twist[i] / l[i]
        private double[] τ_mid;                                 // torsional moment : Q = GJ.(τ-τ_0)
        private double[] τ_l;                         // torsion moment at left/rigth of each node, ghost and handle
        private double[] τ_g;                         // torsion moment at left/rigth of each node, ghost and handle
        private double[] τ_r;                         // torsion moment at left/rigth of each node, ghost and handle

        // INTERNAL FORCES & MOMENTS
        private double[] N_mid;                                 // axial force : N = ES.ε
        private double[] N_l;
        private double[] N_g;
        private double[] N_r;

        public MVector[] V_mid;
        public MVector[] V_l;
        public MVector[] V_g;
        public MVector[] V_r;

        private MVector[] M_l;
        private MVector[] M_g;
        private MVector[] M_r;


        public double[] Q_mid;                                 // torsional moment : Q = GJ.(τ-τ_0)
        public double[] Q_l, Q_r;                          // torsion moment at left/rigth of each node, ghost and handle

        // RESULTANTE FORCES (DR SPECIFIC ?? => si oui, à déplacer)

        private MVector[] dθ;

        #endregion

        // pour l'instant, on passe l'ensemble des frames et des sections
        public Beam_4DOF_D(IEnumerable<MFrame> restFrames, IEnumerable<MFrame> actualFrames, IEnumerable<Section> sections, IEnumerable<Material> materials, bool isClosed = false)
            :base(isClosed)
        {           
            // KINEMATIC CAPABILITIES
            TranslationalDOF = 3;
            RotationalDOF = 1;

            // MECHANICAL CAPABILITIES 
            IsCompressionCapable = true;
            IsTractionCapable = true;
            IsShearCapable = true;
            IsBendingCapable = true;
            IsTorsionCapable = true;

            // AUTOMATIC CREATION OF GHOST VERTICES
            CreateGhostVertices(restFrames, actualFrames, isClosed);

            // TOPOLOGY
            if (IsClosed)
            {
                ne = mframes.Length;
                nv_g = ne / 2;
                nv_h = ne / 2;
                nv = nv_g + nv_h;
            }
            else
            {
                ne = mframes.Length - 1;
                nv_g = ne / 2;
                nv_h = nv_g + 1;
                nv = nv_g + nv_h;
            }

            // PREFORM CHECKS
            if (mframes_0.Length != mframes_i.Length)
                throw new ArgumentOutOfRangeException("actualFrames", "actualFrames and restFrames must have the same number of items.");

            // INSTANCIATE INTERNAL ARRAYS
            CreateInternalArrays(nv, nv_h, nv_g, ne);

            // DEEP COPY
            this.sections = sections.ToArray();
            this.materials = materials.ToArray();

            for (int i = 0; i < nv_g; i++)
            {
                var section = this.sections[0];
                var material = this.materials[0];

                ES[i] = material.E * section.S;
                EI1[i] = material.E * section.I1;
                EI2[i] = material.E * section.I2;
                GJ[i] = material.G * section.J;
            }

            // SET CONFIGURATIONS
            SetRestConfig(mframes_0);
            SetInitialConfig(mframes);
        }
        private void CreateGhostVertices(IEnumerable<MFrame> restFrames, IEnumerable<MFrame> actualFrames, bool isClosed)
        {
            var frames = restFrames.ToArray();
            int nv = frames.Length;
            int ne = IsClosed ? nv : nv - 1;

            var x = new MPoint[nv];
            var e = new MVector[ne];
            var u = new MVector[ne];
            var t = new MVector[nv];
            var κb = new MVector[nv];
            var l = new double[ne];
            var τ = new double[ne];

            // Interpolate restFrames
            Centerline.GetCurvature(frames, x, e, u, l, t, κb, isClosed);
            Centerline.ZAlignFrames(frames, t);
            Centerline.GetTwist(frames, l, τ, isClosed);
            mframes_0 = Centerline.Refine(frames, κb, τ, isClosed, 1);

            // Interpolate actualFrames
            frames = actualFrames.ToArray();
            Centerline.GetCurvature(frames, x, e, u, l, t, κb, isClosed);
            Centerline.ZAlignFrames(frames, t);
            Centerline.GetTwist(frames, l, τ, isClosed);
            mframes = Centerline.Refine(frames, κb, τ, isClosed, 1);
            mframes_i = mframes.DeepCopy();
        }
        private void CreateInternalArrays(int nv, int nv_h, int nv_g, int ne)
        {
            // SECTION & MATERIAL 
            ES = new double[nv_g];
            EI1 = new double[nv_g];
            EI2 = new double[nv_g];
            GJ = new double[nv_g];

            // EXTERNAL FORCES & MOMENTS
            Fext_g = new MVector[nv_h];
            Mext_m = new MVector[nv_h];
            fext_g = new MVector[nv_g];
            mext_m = new MVector[nv_g];

            // REACTION MOMENT AND FORCES
            Fr_g = new MVector[nv_h];
            Mr_m = new MVector[nv_h];

            // APPLIED LOADS
            loadManager = new BeamLoadManager(this);

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
            κb_g_0 = new MVector[nv_g];        // tangente à un ghost node
            κb_l_0 = new MVector[nv_h];      // tangente à gauche d'un handle node (non défini en 0)
            κb_r_0 = new MVector[nv_h];      // tangente à droite d'un handle node (non défini en nv_h-1)
            τ_0 = new double[ne];

            // DEFORMED CONFIGURATION
            x = new MPoint[nv];
            e = new MVector[ne];
            l = new double[ne];

            t = new MVector[nv];
            t_mid = new MVector[ne];
            mframes_mid = new MFrame[ne];

            ε = new double[ne];
            ε_l = new double[nv_h];
            ε_g = new double[nv_g];
            ε_r = new double[nv_h];

            κb_mid = new MVector[ne];
            κb_l = new MVector[nv_h];       
            κb_g = new MVector[nv_g];       
            κb_r = new MVector[nv_h];       

            τ = new double[ne];
            τ_mid = new double[ne];
            τ_l = new double[nv_h];
            τ_g = new double[nv_g];
            τ_r = new double[nv_h];

            // INTERNAL FORCES & MOMENTS
            N_mid = new double[ne];
            N_l = new double[nv_h];
            N_g = new double[nv_g];
            N_r = new double[nv_h];

            V_mid = new MVector[ne];
            V_l = new MVector[nv_h];
            V_g = new MVector[nv_g];
            V_r = new MVector[nv_h];

            M_g = new MVector[nv_g];   // moment à un ghost node
            M_l = new MVector[nv_h];   // moment à gauche d'un handle node
            M_r = new MVector[nv_h];   // moment à gauche d'un handle node

            Q_mid = new double[ne];
            Q_l = new double[nv];
            Q_r = new double[nv];

            // DR RESULTANT
            R_x = new MVector[nv];
            Rint_x = new MVector[nv];


            R_θ = new MVector[nv];
            Rint_θ = new MVector[nv];

            // internal rotation to realign frames with the centerline at each x step.
            dθ = new MVector[nv];
    }

        public override string ToString()
        {
            return "Beam_4DOF_D";
        }

        private void SetRestConfig(MFrame[] restFrames)
        {
            for (int i = 0; i < nv; i++)
            {
                x[i] = restFrames[i].Origin;
            }
            Centerline.GetCurvature(x, e, t_mid, l_0, t, κb_g_0, IsClosed);
            Centerline.GetTwist(restFrames, l_0, τ_0, IsClosed);

            int index;
            MVector κb, κb_l, κb_r, d1, d2;

            // HANDLE

            // i = 0
            κb_r = (2 / l_0[0]) * MVector.CrossProduct(t[0], t_mid[0]);
            d1 = restFrames[0].XAxis;
            d2 = restFrames[0].YAxis;
            κb_r_0[0].X = κb_r * d1;
            κb_r_0[0].Y = κb_r * d2;

            for (int i = 1; i < nv_h-1; i++)
            {
                index = 2 * i;
                κb_l = (2 / l_0[index - 1]) * MVector.CrossProduct(t_mid[index - 1], t[index]);
                κb_r = (2 / l_0[index]) * MVector.CrossProduct(t[index], t_mid[index]);

                d1 = restFrames[index].XAxis;
                d2 = restFrames[index].YAxis;

                // WARNING : given in the material frame coordinate system
                κb_l_0[i].X = κb_l * d1;
                κb_l_0[i].Y = κb_l * d2;
                κb_r_0[i].X = κb_r * d1;
                κb_r_0[i].Y = κb_r * d2;
            }

            // i = nv_h-1
            κb_l = 2 / (l_0[ne - 1]) * MVector.CrossProduct(t_mid[ne - 1], t[nv - 1]);
            d1 = mframes[nv - 1].XAxis;
            d2 = mframes[nv - 1].YAxis;
            κb_l_0[nv_h - 1].X = κb_l * d1;
            κb_l_0[nv_h - 1].Y = κb_l * d2;

            // GHOST
            for (int i = 0; i < nv_g; i++)
            {
                index = 2 * i + 1;
                κb = κb_g_0[i];

                d1 = restFrames[index].XAxis;
                d2 = restFrames[index].YAxis;

                // WARNING : given in the material frame coordinate system
                κb_g_0[i].X = κb * d1;
                κb_g_0[i].Y = κb * d2;
                κb_g_0[i].Z = 0;
            }
        }
        private void SetInitialConfig(MFrame[] initialFrames)
        {
            // The initial set of material frames impose the starting configuration.
            // Thus ti is not interpolated but immediately given by mframe ZAxis.

            // on initialise x et MaterialFrame avec les donvées de départ (on en fait une copie)
            for (int i = 0; i < nv; i++)
            {
                x[i] = initialFrames[i].Origin;
                mframes[i] = initialFrames[i];
            }
            Centerline.GetCurvature(x, e, t_mid, l, t, κb_g, IsClosed);
            Centerline.GetTwist(mframes, l, τ, IsClosed);
        }

        public override void Init()
        {
        }

        public override void Move_x(MVector[] dx)
        {
            // x = x + dx
            for (int i = 0; i < nv; i++)
            {
                x[i] = mframes[i].Origin + dx[i];
            }
            OnFramesTranslated(dx);

            Centerline.GetCurvature(x, e, t_mid, l, t, κb_g, IsClosed);
            if (!IsClosed) // assumes that edges are free (default) before imposing any tangent constraint
            {
                t[0] = e[0] / l[0];
                t[nv - 1] = e[ne - 1] / l[ne - 1];
            }

            // call for tangent constraint
            OnTangentsUpdated(t);

            // realign material frames to stick to the centerline (this is a rotation by dθ[i] = MVector.CrossProduct(mframes[i].ZAxis, t[i]);
            // if the tangent vector has not changed, the frame remains unchanged.
            for (int i = 0; i < nv; i++)
            {
                dθ[i] = MVector.CrossProduct(mframes[i].ZAxis, t[i]);
                ParallelTransportation.ZPT_Rotation(mframes[i], mframes[i].ZAxis, x[i], t[i], ref mframes[i]);
            }

            //OnFramesRotated(dθ);

            Centerline.GetTwist(mframes, l, τ, IsClosed);
        }
        public override void Move_θ(MVector[] dθ)
        {
            // θ = θ + dθ
            for (int i = 0; i < nv; i++)
            {
                mframes[i].ZRotate(dθ[i].Z);
            }
            //OnFramesRotated(dθ);
            Centerline.GetTwist(mframes, l, τ, IsClosed);
        }

        public override void Calculate_x()
        {
            UpdateBendingMoment();
            UpdateTwistingMoment();

            UpdateShearForce();
            UpdateAxialForce();

            UpdateInternalNodalMoment();
            UpdateInternalNodalForce();

            UpdateResultantNodalMoment();
            UpdateResultantNodalForce();
        }
        public override void Calculate_θ()
        {
            UpdateBendingMoment();
            UpdateTwistingMoment();

            UpdateShearForce();
            UpdateAxialForce();

            UpdateInternalNodalMoment();
            UpdateInternalNodalForce();

            UpdateResultantNodalMoment();
            UpdateResultantNodalForce();
        }

        public void ClearAppliedReactions()
        {
            for (int i = 0; i < nv_h; i++)
            {
                Fext_g[i] = MVector.Zero;
                Mext_m[i] = MVector.Zero;
            }
            for (int i = 0; i < nv_g; i++)
            {
                fext_g[i] = MVector.Zero;
                mext_m[i] = MVector.Zero;
            }
        }

        // MOMENTS

        private void UpdateBendingMoment()
        {
            MVector d1, d2;
            MVector κb, κb_0;
            MVector κbl, κbl_0, κbr, κbr_0;
            double dM1, dM2, M1, M2;
            MVector dM, Mmean;

            // HANDLE

            // i = 0
            κbr = (2 / l[0]) * MVector.CrossProduct(t[0], t_mid[0]);
            κbr_0 = κb_r_0[0];
            d1 = mframes[0].XAxis;
            d2 = mframes[0].YAxis;
            M1 = EI1[0] * (κbr * d1 - κbr_0.X);
            M2 = EI2[0] * (κbr * d2 - κbr_0.Y);
            M_r[0] = M1 * d1 + M2 * d2;
            κb_r[0] = κbr;

            // i = 1, ..., nv_h-1
            for (int i = 1; i < nv_h - 1; i++)
            {
                // left curvature
                κbl = (2 / l[2 * i - 1]) * MVector.CrossProduct(t_mid[2 * i - 1], t[2 * i]);
                κbl_0 = κb_l_0[i];

                // right curvature
                κbr = (2 / l[2 * i]) * MVector.CrossProduct(t[2 * i], t_mid[2 * i]);
                κbr_0 = κb_r_0[i];

                d1 = mframes[2 * i].XAxis;
                d2 = mframes[2 * i].YAxis;

                // mean moment 
                M1 = 0.5 * (EI1[i - 1] * (κbl * d1 - κbl_0.X) + EI1[i] * (κbr * d1 - κbr_0.X));
                M2 = 0.5 * (EI2[i - 1] * (κbl * d2 - κbl_0.Y) + EI2[i] * (κbr * d2 - κbr_0.Y));
                Mmean = M1 * d1 + M2 * d2;

                // left & right moments that satisfy the static condition -Ml + Mr + Mext_m + Mr_m = 0
                dM1 = 0.5 * (Mext_m[i].X + Mr_m[i].X);
                dM2 = 0.5 * (Mext_m[i].Y + Mr_m[i].Y);
                dM = dM1 * d1 + dM2 * d2;
                M_l[i] = Mmean + dM;
                M_r[i] = Mmean - dM;

                // left curvatures from moment
                var κ1_l = (M1 + dM1) / EI1[i - 1];
                var κ2_l = (M2 + dM2) / EI2[i - 1];
                κb_l[i] = κ1_l * d1 + κ2_l * d2;

                // right curvatures from moment
                var κ1_h_r = (M1 - dM1) / EI1[i];
                var κ2_h_r = (M2 - dM2) / EI2[i];
                κb_r[i] = κ1_h_r * d1 + κ2_h_r * d2;
            }

            // i = nv_h - 1
            κbl = (2 / l[ne - 1]) * MVector.CrossProduct(t_mid[ne - 1], t[nv - 1]);
            κbl_0 = κb_l_0[nv_h - 1];
            d1 = mframes[nv - 1].XAxis;
            d2 = mframes[nv - 1].YAxis;
            M1 = EI1[nv_g - 1] * (κbl * d1 - κbl_0.X);
            M2 = EI2[nv_g - 1] * (κbl * d2 - κbl_0.Y);
            M_l[nv_h - 1] = M1 * d1 + M2 * d2;
            κb_l[nv_h - 1] = κbl;

            // GHOST
            for (int i = 0; i < nv_g; i++)
            {
                κb = κb_g[i];
                κb_0 = κb_g_0[i];
                d1 = mframes[2 * i + 1].XAxis;
                d2 = mframes[2 * i + 1].YAxis;

                M1 = EI1[i] * (κb * d1 - κb_0.X);
                M2 = EI2[i] * (κb * d2 - κb_0.Y);
                M_g[i] = M1 * d1 + M2 * d2;

                κb_mid[2 * i] = 0.5 * (κb_r[i] + κb_g[i]);
                κb_mid[2 * i] = κb_mid[2 * i] - (κb_mid[2 * i] * t_mid[2 * i]) * t_mid[2 * i]; // make sure kb is perpendicular to d3

                κb_mid[2 * i + 1] = 0.5 * (κb_g[i] + κb_l[i + 1]);
                κb_mid[2 * i + 1] = κb_mid[2 * i + 1] - (κb_mid[2 * i + 1] * t_mid[2 * i + 1]) * t_mid[2 * i + 1]; // make sure kb is perpendicular to d3

            }
        }

        /// <summary>
        /// Q : twist, τ
        /// Q is given at each mid-edge i+1/2
        /// </summary>
        private void UpdateTwistingMoment()
        {
            // interpolation de l'effort tranchant aux noeuds
            // non nécessaire pour la DR => à décplacer dans l'affichage
            for (int i = 0; i < nv_g; i++)
            {
                var l0 = l[2 * i];
                var l1 = l[2 * i + 1];

                var GJ = this.GJ[i];
                var m3 = mext_m[i].Z;

                var κM0 = MVector.CrossProduct(κb_r[i], M_r[i]) * mframes[2 * i].ZAxis;
                var κM1 = MVector.CrossProduct(κb_g[i], M_g[i]) * mframes[2 * i + 1].ZAxis;
                var κM2 = MVector.CrossProduct(κb_l[i + 1], M_l[i + 1]) * mframes[2 * i + 2].ZAxis;

                // 0-1
                var τ01 = τ[2 * i];
                var Q01 = GJ * (τ01 - τ_0[2 * i]);
                var dQ01 = (l0 / 4) * (κM0 + κM1) + (l0 / 2) * m3;
                var dτ01 = dQ01 / GJ;

                // 1-2
                var τ12 = τ[2 * i + 1];
                var dQ12 = (l1 / 4) * (κM1 + κM2) + (l1 / 2) * m3;
                var Q12 = GJ * (τ12 - τ_0[2 * i + 1]);
                var dτ12 = dQ12 / GJ;

                // 0
                Q_mid[2 * i] = Q01;
                Q_r[2 * i] = Q01 + dQ01;
                Q_l[2 * i + 1] = Q01 - dQ01;
                τ_r[i] = τ01 + dτ01;

                // 1
                τ_g[i] = 0.5 * ((τ01 - dτ01) + (τ12 + dτ12));

                // 2
                Q_mid[2 * i + 1] = Q12;
                Q_r[2 * i + 1] = Q12 + dQ12;
                Q_l[2 * i + 2] = Q12 - dQ12;
                τ_l[i + 1] = τ12 - dτ12;
            }

            // Calcul les frames au milieu des edges
            // GJ * dθ_mid = li/8 *(3*Qr + Ql)

            for (int i = 0; i < ne; i++)
            {
                var dθ_mid = l[i] * (0.5 * τ[i] + (Q_l[i + 1] - Q_r[i]) / (8 * GJ[i / 2]));
                ParallelTransportation.ZPT_Rotation(mframes[i], mframes[i].ZAxis, 0.5 * (x[i] + x[i + 1]), t_mid[i], ref mframes_mid[i]); // la position ne tient pas compte de la courbure. Mais ce n'est pas nécessaire pour la suite
                mframes_mid[i].ZRotate(dθ_mid);
            }

        }

        /// <summary>
        /// Q : twist, τ
        /// Q is given at each mid-edge i+1/2
        /// </summary>
        private void UpdateInternalNodalMoment()
        {
            Rint_θ[0] = MVector.Zero;
            for (int i = 0; i < nv_g; i++)
            {
                var l0 = l[2 * i];
                var l1 = l[2 * i + 1];

                // au repos, nécessairement τ_0(s) est uniforme sur [x_h_r, x_g, x_h_l]
                // et donc on a bien Q' = GJτ' = -(κbxM + m).d3 sur cet intervalle (et non Q' = GJ(τ'-τ_0')
                //var κM0 = MVector.CrossProduct(κb_h_r[i], M_h_r[i]) * mframes[2 * i].ZAxis;
                //var κM1 = MVector.CrossProduct(κb_g[i], M_g[i]) * mframes[2 * i + 1].ZAxis;
                //var κM2 = MVector.CrossProduct(κb_h_l[i + 1], M_h_l[i + 1]) * mframes[2 * i + 2].ZAxis;

                Rint_θ[2 * i].X += M_r[i].X;
                Rint_θ[2 * i].Y += M_r[i].Y;
                //Rint_θ[2 * i].Z += Q_mid[2 * i] + l0 / 4 * (κM0 + κM1);
                Rint_θ[2 * i].Z += Q_r[2 * i];


                Rint_θ[2 * i + 1].X = 0;
                Rint_θ[2 * i + 1].Y = 0;
                //Rint_θ[2 * i + 1].Z = Q_mid[2 * i + 1] - Q_mid[2 * i] + l0 / 4 * (κM0 + κM1) + l1 / 4 * (κM1 + κM2);
                Rint_θ[2 * i + 1].Z = -Q_l[2 * i + 1] + Q_r[2 * i + 1];

                Rint_θ[2 * i + 2].X = -M_l[i + 1].X;
                Rint_θ[2 * i + 2].Y = -M_l[i + 1].Y;
                //Rint_θ[2 * i + 2].Z = -Q_mid[2 * i + 1] + l1 / 4 * (κM1 + κM2);
                Rint_θ[2 * i + 2].Z = -Q_l[2 * i + 2];

            }

            //Rint_θ[0] = MVector.Zero;
            //for (int i = 0; i < nv_g; i++)
            //{
            //    var l0 = l[2 * i];
            //    var l1 = l[2 * i + 1];

            //    // au repos, nécessairement τ_0(s) est uniforme sur [x_h_r, x_g, x_h_l]
            //    // et donc on a bien Q' = GJτ' = -(κbxM + m).d3 sur cet intervalle (et non Q' = GJ(τ'-τ_0')
            //    var M01 = 0.5 * (M_h_r[i] + M_g[i]);            // a corriger avec l'interpolation parablolique
            //    var M12 = 0.5 * (M_g[i] + M_h_l[i + 1]);        // a corriger avec l'interpolation parablolique

            //    var R0 = M01 + Q_mid[2 * i] * t_mid[2 * i];
            //    //var R1 = -M01 - Q_mid[2 * i] * t_mid[2 * i] + M12 + Q_mid[2 * i + 1] * t_mid[2 * i + 1];
            //    var R2 = M12 + Q_mid[2 * i + 1] * t_mid[2 * i + 1];

            //    Rint_θ[2 * i].X += M_h_r[i].X;
            //    Rint_θ[2 * i].Y += M_h_r[i].Y;
            //    Rint_θ[2 * i].Z += R0 * mframes[2 * i].ZAxis;

            //    Rint_θ[2 * i + 1].X = 0;
            //    Rint_θ[2 * i + 1].Y = 0;
            //    Rint_θ[2 * i + 1].Z = (-R0+R2) * mframes[2 * i+1].ZAxis;

            //    Rint_θ[2 * i + 2].X = -M_h_l[i + 1].X;
            //    Rint_θ[2 * i + 2].Y = -M_h_l[i + 1].Y;
            //    Rint_θ[2 * i + 2].Z = -R2 * mframes[2 * i + 2].ZAxis;
            //}
        }

        private void UpdateResultantNodalMoment()
        {
            // ADD APPLIED LOADS TO INTERNAL RESULTANT
            R_θ[0] = Rint_θ[0] + Mext_m[0] + (0.5 * l[0]) * mext_m[0];
            for (int i = 1; i < nv_h - 1; i++)
            {
                R_θ[2 * i] = Rint_θ[2 * i]
                           + Mr_m[i] + Mext_m[i];
                           //+ 0.0 * (mext_m[i - 1] * l[2 * i - 1] + mext_m[i] * l[2 * i]);
            }
            R_θ[nv - 1] = Rint_θ[nv - 1] + Mext_m[nv_h - 1] + (0.5 * l[2 * nv_g - 1]) * mext_m[nv_g - 1];

            for (int i = 0; i < nv_g; i++)
            {
                R_θ[2 * i + 1] = Rint_θ[2 * i + 1];
                    //+ (0.0 * (l[2 * i] + l[2 * i + 1])) * mext_m[i];
            }

            OnReactionMomentUpdating(Mr_m, R_θ);
        }

        // FORCES       
        private void UpdateShearForce()
        {
            MVector M0, M1, M2, Vmid, V;
            MVector dM01, dM12, dM0, dM1, dM2, M01, M12;
            MFrame mframe;
            double l0, l1;

            for (int i = 0; i < nv_g; i++)
            {
                // calcul des dérivées :
                M0 = M_r[i];
                M1 = M_g[i];
                M2 = M_l[i + 1];

                l0 = l[2 * i];
                l1 = l[2 * i + 1];

                Interpolation.Quadratic(l0, l1, M0, M1, M2, out dM0, out dM1, out dM2, out dM01, out dM12, out M01, out M12);

                M01 = 0.5 * (M0 + M1);
                M12 = 0.5 * (M1 + M2);

                //parabolic interpolation of M
                dM0 = -(2 * l0 + l1) / (l0 * (l0 + l1)) * M0 + (l0 + l1) / (l0 * l1) * M1 - l0 / (l1 * (l0 + l1)) * M2;
                dM1 = -l1 / (l0 * (l1 + l0)) * M0 - (l0 - l1) / (l0 * l1) * M1 + l0 / (l1 * (l0 + l1)) * M2;
                dM2 = l1 / (l0 * (l0 + l1)) * M0 - (l1 + l0) / (l0 * l1) * M1 + (2 * l1 + l0) / (l1 * (l1 + l0)) * M2;

                var GJ = this.GJ[i];
                var m = this.mext_m[i];

                mframe = mframes[2 * i + 1];
                var Vm = -m.Y * mframe.XAxis + m.X * mframe.YAxis;

                // At mid
                Vmid = MVector.CrossProduct(t_mid[2 * i], dM01);        // d3 x M'
                Vmid += Vm;                                             // d3 x m
                Vmid += Q_mid[2 * i] * κb_mid[2 * i];                   // Qκb
                Vmid += -τ[2 * i] * M01;                                // -τM
                V_mid[2 * i] = Vmid - (Vmid * t_mid[2 * i]) * t_mid[2 * i];

                Vmid = MVector.CrossProduct(t_mid[2 * i + 1], dM12);    // d3 x M'
                Vmid += Vm;                                             // d3 x m
                Vmid += Q_mid[2 * i + 1] * κb_mid[2 * i + 1];           // Qκb
                Vmid += - τ[2 * i + 1] * M12;                           // -τM
                V_mid[2 * i + 1] = Vmid - (Vmid * t_mid[2 * i + 1]) * t_mid[2 * i + 1];
            }
        }
        private void UpdateAxialForce()
        {
            for (int i = 0; i < nv_g; i++)
            {
                // 0-1-2
                var ES = this.ES[i];
                var f3 = fext_g[i] * mframes[2 * i + 1].ZAxis;

                // 0-1
                var l0 = l[2 * i];
                var ε01 = l0 / l_0[2 * i] - 1;
                var N01 = ES * ε01;
                var κT01 = MVector.CrossProduct(κb_mid[2 * i], V_mid[2 * i]) * t_mid[2 * i];
                var dN01 = (l0 / 2) * κT01;
                var dε01 = dN01 / ES;

                // 0-2
                var l1 = l[2 * i + 1];
                var ε12 = l1 / l_0[2 * i + 1] - 1;
                var N12 = ES * ε12;
                var κT12 = MVector.CrossProduct(κb_mid[2 * i + 1], V_mid[2 * i + 1]) * t_mid[2 * i + 1];
                var dN12 = (l1 / 2) * κT12;
                var dε12 = dN12 / ES;

                // 0
                ε[2 * i] = ε01;
                N_mid[2 * i] = N01; 
                N_r[i] = N01 + dN01;
                ε_r[i] = ε01 + dε01;

                // 1
                ε_g[i] = 0.5 * ((ε01 - dε01) + (ε12 + dε12)); // make sure ε is continuous over ]0,1,2[
                N_g[i] = 0.5 * ((N01 - dN01) + (N12 + dN12));

                // 2
                ε[2 * i + 1] = ε12;
                N_mid[2 * i + 1] = ES * ε12;
                N_l[i + 1] = N12 - dN12;
                ε_l[i + 1] = ε12 - dε12;

                // ================================================================

                var κN0 = N_r[i] * MVector.CrossProduct(κb_r[i], mframes[2 * i].ZAxis);
                var κN1 = N_g[i] * MVector.CrossProduct(κb_g[i], mframes[2 * i + 1].ZAxis);
                var κN2 = N_l[i + 1] * MVector.CrossProduct(κb_l[i + 1], mframes[2 * i + 2].ZAxis);

                var τT01 = τ[2 * i] * MVector.CrossProduct(t_mid[2 * i], V_mid[2 * i]);
                var τT12 = τ[2 * i + 1] * MVector.CrossProduct(t_mid[2 * i + 1], V_mid[2 * i + 1]);

                var dV01 = (l0 / 4) * (κN0 + κN1) + (l0 / 2) * τT01;
                var dV12 = (l1 / 4) * (κN1 + κN2) + (l1 / 2) * τT12;

                // 2i
                var Vr = V_mid[2 * i] + dV01;
                var Vr1 = Vr * mframes_mid[2 * i].XAxis;
                var Vr2 = Vr * mframes_mid[2 * i].YAxis;
                V_r[i] = Vr1 * mframes[2 * i].XAxis + Vr2 * mframes[2 * i].YAxis;

                // 2i+1
                var Vgl = 0.5 * (V_mid[2 * i] - dV01);
                var Vgl1 = Vgl * mframes_mid[2 * i].XAxis;
                var Vgl2 = Vgl * mframes_mid[2 * i].YAxis;

                var Vgr = 0.5 * (V_mid[2 * i + 1] + dV12);
                var Vgr1 = Vgr * mframes_mid[2 * i + 1].XAxis;
                var Vgr2 = Vgr * mframes_mid[2 * i + 1].YAxis;

                V_g[i] = (Vgl1 + Vgr1) * mframes[2 * i + 1].XAxis + (Vgl2 + Vgr2) * mframes[2 * i + 1].YAxis;

                // 2i+2
                var Vl = V_mid[2 * i + 1] - dV12;
                var Vl1 = Vl * mframes_mid[2 * i + 1].XAxis;
                var Vl2 = Vl * mframes_mid[2 * i + 1].YAxis;
                V_l[i + 1] = Vl1 * mframes[2 * i + 2].XAxis + Vl2 * mframes[2 * i + 2].YAxis;
            }
        }
        private void UpdateInternalNodalForce()
        {
            // AXIAL FORCE : attention à ne pas répéter cette opération pour les variations sur θ

            // A revoir avec les nouvelles méthodes d'interpolation des efforts aux noeuds au début/fin de poutre
            // ne prend pas en compte les efforts linéiques
            //Rint_x_axial[0] = ((N_mid[0] * t_mid[0] + V_mid[0]) * t[0]) * t[0];
            //Rint_x_axial[0] = N_mid[0] * t_mid[0];
            //Rint_x_axial[0] = N_r[0];

            //Rint_x_axial[1] = -N_mid[0] * t_mid[0];
            //for (int i = 1; i < ne - 1; i++)
            //{
            //    MVector force = N_mid[i] * t_mid[i];
            //    Rint_x_axial[i] += force;
            //    Rint_x_axial[i + 1] = -force;
            //}
            //Rint_x_axial[nv - 2] += N_mid[ne - 1] * t_mid[ne - 1];
            //Rint_x_axial[nv - 1] = -((N_mid[ne - 1] * t_mid[ne - 1] + V_M[ne - 1] + V_Q[ne - 1]) * t[nv - 1]) * t[nv - 1];

            //// SHEAR FORCE | bending contribution
            //// A revoir avec les nouvelles méthodes d'interpolation des efforts aux noeuds au début/fin de poutre
            //// ne prend pas en compte les efforts linéiques
            //Rint_x_shear_M[0] = (N_mid[0] * t_mid[0]) - ((N_mid[0] * t_mid[0] + V_M[0]) * t[0]) * t[0];
            //for (int i = 0; i < ne; i++)
            //{
            //    Rint_x_shear_M[i] += V_M[i];
            //    Rint_x_shear_M[i + 1] = -V_M[i];
            //}
            //Rint_x_shear_M[nv - 1] += -N_mid[ne - 1] * t_mid[ne - 1] + ((N_mid[ne - 1] * t_mid[ne - 1] + V_M[ne - 1]) * t[nv - 1]) * t[nv - 1];

            //// SHEAR FORCE | twisting contribution
            //Rint_x_shear_Q[0] = -(V_Q[0] * t[0]) * t[0];
            //for (int i = 0; i < ne; i++)
            //{
            //    Rint_x_shear_Q[i] += V_Q[i];
            //    Rint_x_shear_Q[i + 1] = -V_Q[i];
            //}
            //Rint_x_shear_Q[nv - 1] += (V_Q[nv_h - 1] * t[nv - 1]) * t[nv - 1];

            //// RESULTANT FORCE
            //for (int i = 0; i < nv; i++)
            //{
            //    Rint_x[i] = Rint_x_axial[i] + Rint_x_shear_M[i] + Rint_x_shear_Q[i];
            //}

            //RESULTANT FORCE
            Rint_x[0] = N_mid[0] * t[0] + V_mid[0];
            for (int i = 1; i < nv - 1; i++)
            {
                var N = -N_mid[i - 1] * t_mid[i - 1] + N_mid[i] * t_mid[i];
                Rint_x[i] = N - V_mid[i - 1] + V_mid[i];
            }
            Rint_x[nv - 1] = -N_mid[ne - 1] * t[nv - 1] - V_mid[ne - 1];

            //Rint_x[0] = MVector.Zero;
            //for (int i = 0; i < nv_g; i++)
            //{
            //    var l0 = l[2 * i];
            //    var l1 = l[2 * i + 1];

            //    var κV0 = MVector.CrossProduct(κb_h_r[i], V_r[2 * i]) * mframes[2 * i].ZAxis;
            //    var κV1 = MVector.CrossProduct(κb_g[i], 0.5 * (V_l[2 * i + 1] + V_r[2 * i + 1])) * mframes[2 * i + 1].ZAxis;
            //    var κV2 = MVector.CrossProduct(κb_h_l[i + 1], V_l[2 * i + 2]) * mframes[2 * i + 2].ZAxis;

            //    Rint_x[2 * i] += (N_mid[2 * i] + l0 / 4 * (κV0 + κV1)) * mframes[2 * i].ZAxis;
            //    Rint_x[2 * i] += V_mid[2 * i];

            //    Rint_x[2 * i + 1] = (-N_mid[2 * i] + N_mid[2 * i + 1] + l0 / 4 * (κV0 + κV1) + l1 / 4 * (κV1 + κV2)) * mframes[2 * i + 1].ZAxis;
            //    Rint_x[2 * i + 1] += -V_mid[2 * i] + V_mid[2 * i + 1];

            //    Rint_x[2 * i + 2] = (-N_mid[2 * i + 1] + l1 / 4 * (κV1 + κV2)) * mframes[2 * i].ZAxis;
            //    Rint_x[2 * i + 2] += -V_mid[2 * i + 1];
            //}

        }
        private void UpdateResultantNodalForce()
        {
            // RESULTANT FORCE

            // ADD APPLIED LOADS TO INTERNAL RESULTANT
            R_x[0] = Rint_x[0] + Fext_g[0] + (0.5 * l[0]) * fext_g[0];
            for (int i = 1; i < nv_h - 1; i++)
            {
                R_x[2 * i] = Rint_x[2 * i] 
                           + Fr_g[i] + Fext_g[i] 
                           + 0.5 * (fext_g[i - 1] * l[2 * i - 1] + fext_g[i] * l[2 * i]);
            }
            R_x[nv - 1] = Rint_x[nv - 1] + Fext_g[nv_h - 1] + (0.5 * l[2 * nv_g - 1]) * fext_g[nv_g - 1];
            for (int i = 0; i < nv_g; i++)
            {
                R_x[2 * i + 1] = Rint_x[2 * i + 1] + (0.5 * (l[2 * i] + l[2 * i + 1])) * fext_g[i];
            }

            OnReactionForceUpdating(Fr_g, R_x);
        }

        // ENERGIES
        public double GetAxialElasticEnergy()
        {
            double E_axial = 0;
            for (int i = 0; i < ne; i++)
            {
                E_axial += ES[i] * Math.Pow(ε[i], 2) * l[i];
            }
            E_axial = E_axial / 2;
            return E_axial;
        }
        public double GetBendingElasticEnergy()
        {
            double E_bending = 0;
            //E_bending += (EI1[0] * Math.Pow((κ1[0] - κ1_0[0]), 2) + EI2[0] * Math.Pow((κ2[0] - κ2_0[0]), 2)) * (l[0] / 4);
            //for (int i = 1; i < nv - 1; i++)
            //{
            //    E_bending += (EI1[i/2] * Math.Pow((κ1[i] - κ1_0[i]), 2) + EI2[i/2] * Math.Pow((κ2[i] - κ2_0[i]), 2)) * (l[i - 1] + l[i]) / 4;
            //}
            //E_bending += (EI1[nv_g - 1] * Math.Pow((κ1[nv - 1] - κ1_0[nv - 1]), 2) + EI2[nv_g - 1] * Math.Pow((κ2[nv - 1] - κ2_0[nv - 1]), 2)) * (l[ne - 1] / 4);
            return E_bending;
        }
        public double GetTwistingElasticEnergy()
        {
            double E_twisting = 0;
            for (int i = 0; i < ne; i++)
            {
                double τ2 = (τ[i] - τ_0[i]) * (τ[i] - τ_0[i]);
                E_twisting += GJ[i/2] * Math.Pow((τ[i] - τ_0[i]), 2) * l[i];
            }
            E_twisting = E_twisting / 2;
            return E_twisting;
        }

        public void Get2_Q(out CMoment[] Ql, out CMoment[] Qr, out CMoment[] Qmid)
        {
            if (IsClosed)
                throw new NotImplementedException();

            Ql = new CMoment[nv];
            Qr = new CMoment[nv];
            Qmid = new CMoment[ne];

            for (int i = 0; i < nv; i++)
            {
                var frame = mframes[i];
                Ql[i] = new CMoment(Q_l[i] * frame.ZAxis, frame);
                Qr[i] = new CMoment(Q_r[i] * frame.ZAxis, frame);
            }
            for (int i = 0; i < nv_g; i++)
            {
                var frame = mframes[2 * i + 1];
                Qmid[2 * i] = new CMoment(Q_mid[2 * i] * t_mid[2 * i], frame);
                Qmid[2 * i + 1] = new CMoment(Q_mid[2 * i + 1] * t_mid[2 * i + 1], frame);
            }
        }
        public void Get_Q(out CMoment[] Ql, out CMoment[] Qr, out CMoment[] Qmid)
        {
            MVector[] Ql_base, Qr_base, Qmid_base;
            this.Get_V(out Ql_base, out Qr_base, out Qmid_base);

            Ql = new CMoment[nv];
            Qr = new CMoment[nv];
            Qmid = new CMoment[ne];

            for (int i = 0; i < nv; i++)
            {
                var frame = mframes[i];
                Ql[i] = new CMoment(Ql_base[i], frame);
                Qr[i] = new CMoment(Qr_base[i], frame);
            }
            for (int i = 0; i < nv_g; i++)
            {
                var frame = mframes[2 * i + 1];
                Qmid[2 * i] = new CMoment(Qmid_base[2 * i], frame);
                Qmid[2 * i + 1] = new CMoment(Qmid_base[2 * i + 1], frame);
            }
        }
        public void Get_Q(out MVector[] Ql, out MVector[] Qr, out MVector[] Qmid)
        {
            if (IsClosed)
                throw new NotImplementedException();

            Ql = new MVector[Nv];
            Qr = new MVector[Nv];
            Qmid = new MVector[Ne];
         
            double l0, l1;
            double τmean, τ0, τ1, τ2, τmid;

            Ql[0] = MVector.Zero;
            for (int i = 0; i < nv_g; i++)
            {
                l0 = l[2 * i];
                l1 = l[2 * i + 1];

                var GJ = this.GJ[i];
                var m3 = mext_m[i].Z;

                // au repos, nécessairement τ_0(s) est uniforme sur [x_h_r, x_g, x_h_l]
                // et donc on a bien Q' = GJτ' = -(κbxM + m).d3 sur cet intervalle (et non Q' = GJ(τ'-τ_0')
                var dτ0 = -(1 / GJ) * (MVector.CrossProduct(κb_r[i], M_r[i]) * mframes[2 * i].ZAxis + m3);
                var dτ1 = -(1 / GJ) * (MVector.CrossProduct(κb_g[i], M_g[i]) * mframes[2 * i + 1].ZAxis + m3);
                var dτ2 = -(1 / GJ) * (MVector.CrossProduct(κb_l[i + 1], M_l[i + 1]) * mframes[2 * i + 2].ZAxis + m3);

                // 0-1
                τmean = τ[2 * i];
                τ0 = τmean - l0 / 6 * (2 * dτ0 + dτ1);
                τ1 = τmean + l0 / 6 * (dτ0 + 2 * dτ1);
                τmid = τmean + l0 / 24 * (dτ0 - dτ1);
                Qr[2 * i] = (GJ * (τ0 - τ_0[2 * i])) * mframes[2 * i].ZAxis;
                Qmid[2 * i] = (GJ * (τmid - τ_0[2 * i])) * t_mid[2 * i]; // better to create mid frames
                Ql[2 * i + 1] = (GJ * (τ1 - τ_0[2 * i])) * mframes[2 * i + 1].ZAxis;

                // 1-2
                τmean = τ[2 * i + 1];
                τ1 = τmean - l1 / 6 * (2 * dτ1 + dτ2);
                τ2 = τmean + l1 / 6 * (dτ1 + 2 * dτ2);
                τmid = τmean + l1 / 24 * (dτ1 - dτ2);
                Qr[2 * i + 1] = (GJ * (τ1 - τ_0[2 * i + 1])) * mframes[2 * i + 1].ZAxis;
                Qmid[2 * i + 1] = (GJ * (τmid - τ_0[2 * i + 1])) * t_mid[2 * i + 1]; ; // better to create mid frames
                Ql[2 * i + 2] = (GJ * (τ2 - τ_0[2 * i + 1])) * mframes[2 * i + 2].ZAxis; ;
            }
            Qr[nv - 1] = MVector.Zero;


        }

        public void Get_M(out CMoment[] Ml, out CMoment[] Mr)
        {
            Ml = new CMoment[nv];
            Mr = new CMoment[nv];

            for (int i = 0; i < nv_g; i++)
            {
                Ml[2 * i + 1] = new CMoment(M_g[i], mframes[2 * i + 1]);
                Mr[2 * i + 1] = new CMoment(M_g[i], mframes[2 * i + 1]);
            }
            for (int i = 0; i < nv_h; i++)
            {
                Ml[2 * i] = new CMoment(M_l[i], mframes[2 * i]);
                Mr[2 * i] = new CMoment(M_r[i], mframes[2 * i]);
            }
        }
        public void Get_M(out MVector[] Ml, out MVector[] Mr)
        {
            Ml = new MVector[nv];
            Mr = new MVector[nv];

            for (int i = 0; i < nv_g; i++)
            {
                Ml[2 * i + 1] = M_g[i];
                Mr[2 * i + 1] = M_g[i];
            }
            for (int i = 0; i < nv_h; i++)
            {
                Ml[2 * i] = M_l[i];
                Mr[2 * i] = M_r[i];
            }
        }
        public void Diagram_M(out MPoint[] startPoints, out MPoint[] endPoints_1, out MPoint[] endPoints_2, double scale, Configuration config)
        {
            if (IsClosed)
                throw new NotImplementedException();

            if (config == Configuration.Rest)
                Diagram_vector_12(M_l, M_g, M_r, mframes, mframes_0, scale, out startPoints, out endPoints_1, out endPoints_2);
            else if (config == Configuration.Initial)
                Diagram_vector_12(M_l, M_g, M_r, mframes, mframes_i, scale, out startPoints, out endPoints_1, out endPoints_2);
            else
                Diagram_vector_12(M_l, M_g, M_r, mframes, mframes, scale, out startPoints, out endPoints_1, out endPoints_2);
        }

        public void Get_V(out CForce[] Vl, out CForce[] Vr, out CForce[] Vmid)
        {
            Vl = new CForce[nv];
            Vr = new CForce[nv];
            Vmid = new CForce[ne];

            for (int i = 0; i < nv_g; i++)
            {
                Vl[2 * i + 1] = new CForce(V_g[i], mframes[2 * i + 1]);
                Vr[2 * i + 1] = new CForce(V_g[i], mframes[2 * i + 1]);

                Vmid[2 * i] = new CForce(V_mid[2 * i], mframes_mid[2 * i]);
                Vmid[2 * i + 1] = new CForce(V_mid[2 * i + 1], mframes_mid[2 * i + 1]);
            }
            for (int i = 0; i < nv_h; i++)
            {
                Vl[2 * i] = new CForce(V_l[i], mframes[2 * i]);
                Vr[2 * i] = new CForce(V_r[i], mframes[2 * i]);
            }
        }
        public void Get_V(out MVector[] Vl, out MVector[] Vr, out MVector[] Vmid)
        {
            Vl = new MVector[nv];
            Vr = new MVector[nv];
            Vmid = new MVector[ne];

            for (int i = 0; i < nv_g; i++)
            {
                Vl[2 * i + 1] = V_g[i];
                Vr[2 * i + 1] = V_g[i];

                Vmid[2 * i] = V_mid[2 * i];
                Vmid[2 * i + 1] = V_mid[2 * i + 1];
            }
            for (int i = 0; i < nv_h; i++)
            {
                Vl[2 * i] = V_l[i];
                Vr[2 * i] = V_r[i];
            }
        }
        public void Diagram_V(out MPoint[] startPoints, out MPoint[] endPoints_1, out MPoint[] endPoints_2, double scale, Configuration config)
        {
            if (IsClosed)
                throw new NotImplementedException();

            if (config == Configuration.Rest)
                Diagram_vector_12(V_l, V_g, V_r, mframes, mframes_0, scale, out startPoints, out endPoints_1, out endPoints_2);
            else if (config == Configuration.Initial)
                Diagram_vector_12(V_l, V_g, V_r, mframes, mframes_i, scale, out startPoints, out endPoints_1, out endPoints_2);
            else
                Diagram_vector_12(V_l, V_g, V_r, mframes, mframes, scale, out startPoints, out endPoints_1, out endPoints_2);
        }

        public void Get_N(out CForce[] Nl, out CForce[] Nr, out CForce[] Nmid)
        {
            Nl = new CForce[nv];
            Nr = new CForce[nv];
            Nmid = new CForce[ne];

            for (int i = 0; i < nv_g; i++)
            {
                var d3 = mframes[2 * i + 1].ZAxis;
                Nl[2 * i + 1] = new CForce(N_g[i] * d3, mframes[2 * i + 1]);
                Nr[2 * i + 1] = new CForce(N_g[i] * d3, mframes[2 * i + 1]);

                Nmid[2 * i] = new CForce(N_mid[2 * i] * t_mid[2 * i], mframes_mid[2 * i]);
                Nmid[2 * i + 1] = new CForce(N_mid[2 * i + 1] * t_mid[2 * i + 1], mframes_mid[2 * i + 1]);
            }
            for (int i = 0; i < nv_h; i++)
            {
                var d3 = mframes[2 * i].ZAxis;
                Nl[2 * i] = new CForce(N_l[i] * d3, mframes[2 * i]);
                Nr[2 * i] = new CForce(N_r[i] * d3, mframes[2 * i]);
            }
        }
        public void Get_N(out MVector[] Nl, out MVector[] Nr, out MVector[] Nmid)
        {
            Nl = new MVector[nv];
            Nr = new MVector[nv];
            Nmid = new MVector[ne];

            for (int i = 0; i < nv_g; i++)
            {
                var d3 = mframes[2 * i + 1].ZAxis;
                Nl[2 * i + 1] = N_g[i] * d3;
                Nr[2 * i + 1] = N_g[i] * d3;

                Nmid[2 * i] = N_mid[2 * i] * t_mid[2 * i];
                Nmid[2 * i + 1] = N_mid[2 * i + 1] * t_mid[2 * i + 1];
            }
            for (int i = 0; i < nv_h; i++)
            {
                var d3 = mframes[2 * i].ZAxis;
                Nl[2 * i] = N_l[i] * d3;
                Nr[2 * i] = N_r[i] * d3;
            }
        }
        public void Diagram_N(out MPoint[] startPoints, out MPoint[] endPoints, double scale, Configuration config, Axis axis)
        {
            if (IsClosed)
                throw new NotImplementedException();

            if (config == Configuration.Rest)
                Diagram_scalar(N_l, N_g, N_r, mframes_0, scale, axis, out startPoints, out endPoints);
            else if (config == Configuration.Initial)
                Diagram_scalar(N_l, N_g, N_r, mframes_i, scale, axis, out startPoints, out endPoints);
            else
                Diagram_scalar(N_l, N_g, N_r, mframes, scale, axis, out startPoints, out endPoints);
        }

        public void Get_κ(out MVector[] κbl, out MVector[] κbr, out MVector[] κbmid)
        {
            κbl = new MVector[nv];
            κbr = new MVector[nv];
            κbmid = κb_mid.DeepCopy();

            for (int i = 0; i < nv_g; i++)
            {
                κbl[2 * i + 1] = κb_g[i];
                κbr[2 * i + 1] = κb_g[i];
            }
            for (int i = 0; i < nv_h; i++)
            {
                κbl[2 * i] = κb_l[i];
                κbr[2 * i] = κb_r[i];
            }
        }
        public void Diagram_κ(out MPoint[] startPoints, out MPoint[] endPoints_1, out MPoint[] endPoints_2, double scale, Configuration config)
        {
            if (IsClosed)
                throw new NotImplementedException();

            if (config == Configuration.Rest)
                Diagram_vector_12(κb_l, κb_g, κb_r, mframes, mframes_0, scale, out startPoints, out endPoints_1, out endPoints_2);
            else if (config == Configuration.Initial)
                Diagram_vector_12(κb_l, κb_g, κb_r, mframes, mframes_i, scale, out startPoints, out endPoints_1, out endPoints_2);
            else
                Diagram_vector_12(κb_l, κb_g, κb_r, mframes, mframes, scale, out startPoints, out endPoints_1, out endPoints_2);
        }

        public void Get_τ(out double[] τl, out double[] τr, out double[] τmid)
        {
            τl = new double[nv];
            τr = new double[nv];
            τmid = τ.ToArray();

            for (int i = 0; i < nv_g; i++)
            {
                τl[2 * i + 1] = τ_g[i];
                τr[2 * i + 1] = τ_g[i];
            }
            for (int i = 0; i < nv_h; i++)
            {
                τl[2 * i] = τ_l[i];
                τr[2 * i] = τ_r[i];
            }
        }
        public void Diagram_τ(out MPoint[] startPoints, out MPoint[] endPoints, double scale, Configuration config, Axis axis)
        {
            if (IsClosed)
                throw new NotImplementedException();

            if (config == Configuration.Rest)
                Diagram_scalar(τ_l, τ_g, τ_r, mframes_0, scale, axis, out startPoints, out endPoints);
            else if (config == Configuration.Initial)
                Diagram_scalar(τ_l, τ_g, τ_r, mframes_i, scale, axis, out startPoints, out endPoints);
            else
                Diagram_scalar(τ_l, τ_g, τ_r, mframes, scale, axis, out startPoints, out endPoints);
        }

        public void Get_ε(out double[] εl, out double[] εr, out double[] εmid)
        {
            εl = new double[nv];
            εr = new double[nv];
            εmid = ε.ToArray();

            for (int i = 0; i < nv_g; i++)
            {
                εl[2 * i + 1] = ε_g[i];
                εr[2 * i + 1] = ε_g[i];
            }
            for (int i = 0; i < nv_h; i++)
            {
                εl[2 * i] = ε_l[i];
                εr[2 * i] = ε_r[i];
            }
        }
        public void Diagram_ε(out MPoint[] startPoints, out MPoint[] endPoints, double scale, Configuration config, Axis axis)
        {
            if (IsClosed)
                throw new NotImplementedException();

            if (config == Configuration.Rest)
                Diagram_scalar(ε_l, ε_g, ε_r, mframes_0, scale, axis, out startPoints, out endPoints);
            else if (config == Configuration.Initial)
                Diagram_scalar(ε_l, ε_g, ε_r, mframes_i, scale, axis, out startPoints, out endPoints);
            else
                Diagram_scalar(ε_l, ε_g, ε_r, mframes, scale, axis, out startPoints, out endPoints);
        }

        private void Diagram_scalar(double[] vl, double[] vg, double[] vr, MFrame[] displayFrames, double scale, Axis axis, out MPoint[] startPoints, out MPoint[] endPoints)
        {
            if (IsClosed)
                throw new NotImplementedException();

            int nh = vl.Length;
            int ng = vg.Length;

            startPoints = new MPoint[2 * nv_h + nv_g];
            endPoints = new MPoint[2 * nv_h + nv_g];

            var dir = displayFrames.GetAxis(axis);

            MPoint P;

            for (int i = 0; i < nv_g; i++)
            {
                var ig = 3 * i + 2; // ghost index in startPoints & endPoints
                P = displayFrames[2 * i + 1].Origin;
                startPoints[ig] = P;
                endPoints[ig] = P + scale * vg[i] * dir[2 * i + 1];
            }
            for (int i = 0; i < nv_h; i++)
            {
                P = displayFrames[2 * i].Origin;

                var il = 3 * i;     // handle left index in startPoints & endPoints
                startPoints[il] = P;
                endPoints[il] = P + scale * vl[i] * dir[2 * i];

                var ir = il + 1;    // handle right index in startPoints & endPoints
                startPoints[ir] = P;
                endPoints[ir] = P + scale * vr[i] * dir[2 * i];
            }
        }
        private void Diagram_vector_12(MVector[] vl, MVector[] vg, MVector[] vr, MFrame[] valueFrames, MFrame[] displayFrames, double scale, out MPoint[] startPoints, out MPoint[] endPoints_1, out MPoint[] endPoints_2)
        {
            if (IsClosed)
                throw new NotImplementedException();

            int nh = vl.Length;
            int ng = vg.Length;

            startPoints = new MPoint[2 * nv_h + nv_g];
            endPoints_1 = new MPoint[2 * nv_h + nv_g];
            endPoints_2 = new MPoint[2 * nv_h + nv_g];


            var d1 = displayFrames.GetAxis(Axis.d1);
            var d2 = displayFrames.GetAxis(Axis.d2);

            MPoint P;

            for (int i = 0; i < nv_g; i++)
            {
                var ig = 3 * i + 2; // ghost index in startPoints & endPoints
                P = displayFrames[2 * i + 1].Origin;
                startPoints[ig] = P;
                endPoints_1[ig] = P + scale * (vg[i] * valueFrames[2 * i + 1].XAxis) * d1[2 * i + 1];
                endPoints_2[ig] = P + scale * (vg[i] * valueFrames[2 * i + 1].YAxis) * d2[2 * i + 1];
            }
            for (int i = 0; i < nv_h; i++)
            {
                P = displayFrames[2 * i].Origin;

                var il = 3 * i;     // handle left index in startPoints & endPoints
                startPoints[il] = P;
                endPoints_1[il] = P + scale * (vl[i] * valueFrames[2 * i].XAxis) * d1[2 * i];
                endPoints_2[il] = P + scale * (vl[i] * valueFrames[2 * i].YAxis) * d2[2 * i];

                var ir = il + 1;    // handle right index in startPoints & endPoints
                startPoints[ir] = P;
                endPoints_1[ir] = P + scale * (vr[i] * valueFrames[2 * i].XAxis) * d1[2 * i];
                endPoints_2[ir] = P + scale * (vr[i] * valueFrames[2 * i].YAxis) * d2[2 * i];
            }
        }

        // doit être customizable
        public override void Update_lm_x(ref double[] lm_x)
        {
            double lm;
            lm_x[0] = 0.0;
            for (int i = 0; i < nv - 1; i++)
            {
                lm = 0.5 * (ES[i/2] / l_0[i] + 1.5 * Math.Abs(N_mid[i]) / l[i]);
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
                lm = 0.5 * (GJ[i/2] / l_0[i] + 1.5 * Math.Abs(Q_mid[i]) / l[i]);
                lm_θ[i] += lm;
                lm_θ[i + 1] = lm;
            }
        }

    }
}

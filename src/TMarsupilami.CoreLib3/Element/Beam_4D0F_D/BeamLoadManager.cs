using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.MathLib;

namespace TMarsupilami.CoreLib3
{
    public class BeamLoadManager
    {
        private Beam Beam { get; set; }
        private List<BeamVectorLoad> StaticLoads { get; set; }
        private bool[] IsStaticLoadBufferActive { get; set; } // wether a buffer is filled with loads or not.

        private MVector[] ptr_Fext_g, ptr_fext_g; // pointers to Fext and fext beam loads in GCS
        private MVector[] ptr_Mext_m, ptr_mext_m; // pointers to Mext and mext beam loads in MCS

        // Fext_g : beam load buffer 0
        // fext_g : beam load buffer 1
        // Mext_g : beam load buffer 2
        // mext_g : beam load buffer 3
        // Fext_m : beam load buffer 4
        // fext_m : beam load buffer 5
        // Mext_m : beam load buffer 6
        // mext_m : beam load buffer 7
        private MVector[][] StaticLoadBuffers { get; set; }

        public BeamLoadManager(Beam beam)
        {
            Beam = beam;
            StaticLoads = new List<BeamVectorLoad>();

            IsStaticLoadBufferActive = new bool[8];
            StaticLoadBuffers = new MVector[8][];
            for (int i = 0; i < 4; i++)
            {
                StaticLoadBuffers[2 * i] = new MVector[Beam.Nvh];
                StaticLoadBuffers[2 * i + 1] = new MVector[Beam.Nvg];
            }

            ptr_Fext_g = beam.Fext_g;
            ptr_Mext_m = beam.Mext_m;
            ptr_fext_g = beam.fext_g;
            ptr_mext_m = beam.mext_m;
        }
        public void Fill(IEnumerable<BeamVectorLoad> loads)
        {
            Clear();

            // dispatch loads in to lists
            foreach (var load in loads)
            {
                if (load.IsStatic)
                {
                    StaticLoads.Add(load);
                    AddToLoadBuffer(load);
                }
                else
                {
                    //DynamicLoads.Add(load);
                }
            }

            // Active les forces suiveuses
            if (IsStaticLoadBufferActive[4])
            {
                //Beam.Update_Loads.Subscribe(Update_ptr_Fext);
                Beam.FramesRotated += Update_ptr_Fext;
            }

            // Active les moments fixes
            if (IsStaticLoadBufferActive[2])
            {
                //Beam.Update_Loads.Subscribe(Update_ptr_Mext);
                Beam.FramesRotated += Update_ptr_Mext;
            }

            Update();
        }
        public void Update()
        {
            // IN GLOBAL CS
            for (int i = 0; i < ptr_Fext_g.Length; i++)
            {
                ptr_Fext_g[i] = StaticLoadBuffers[0][i];
            }
            for (int i = 0; i < ptr_fext_g.Length; i++)
            {
                ptr_fext_g[i] = StaticLoadBuffers[1][i];
            }

            //// IN MATERIAL CS
            for (int i = 0; i < ptr_Mext_m.Length; i++)
            {
                ptr_Mext_m[i] = StaticLoadBuffers[6][i];
            }
            for (int i = 0; i < ptr_mext_m.Length; i++)
            {
                ptr_mext_m[i] = StaticLoadBuffers[7][i];
            }
        }

        // dans l'hypothèse ou Fext_m est actif, il faut faire un update des efforts du repère materiel vers le repère global
        public void Update_ptr_Fext(MVector[] dθ)
        {
            for (int i = 0; i < ptr_Fext_g.Length; i++)
            {
                int index = 2 * i;

                // convert Fext_m to Global CS
                var Fext_m = StaticLoadBuffers[4][i];
                var Fext_m_to_g = Fext_m.X * Beam.ActualConfiguration[index].XAxis
                                    + Fext_m.Y * Beam.ActualConfiguration[index].YAxis
                                    + Fext_m.Z * Beam.ActualConfiguration[index].ZAxis;

                ptr_Fext_g[i] = StaticLoadBuffers[0][i] + Fext_m_to_g;
            }
        }

        // dans l'hypothèse ou Mext_g est actif, il faut faire un update des efforts du repère global vers le repère materiel
        public void Update_ptr_Mext(MVector[] dθ)
        {
            for (int i = 0; i < ptr_Mext_m.Length; i++)
            {
                int index = 2 * i;

                // convert Fext_m to Global CS
                var Mext_g = StaticLoadBuffers[3][i];
                var Mext_g_to_m = new MVector(  Mext_g * Beam.ActualConfiguration[index].XAxis,
                                                Mext_g * Beam.ActualConfiguration[index].YAxis,
                                                Mext_g * Beam.ActualConfiguration[index].ZAxis
                                             );

                ptr_Mext_m[i] = StaticLoadBuffers[4][i] + Mext_g_to_m;
            }
        }

        private void Clear()
        {
            // Clear lists
            StaticLoads.Clear();

            // Clear LoadBuffers
            var zero = MVector.Zero;
            for (int i = 0; i < StaticLoadBuffers.Length; i++)
            {
                MVector.Set(zero, ref StaticLoadBuffers[i]);
                IsStaticLoadBufferActive[i] = false;
            }
        }

        // ici, on peut traiter pour chaque model comment on transmet la charge au buffer.
        // à transférer vers l'objet/class model mécanique.
        // par exemple un modèle qui ne prend pas en compte les charges linéique, ou qui peut linéariser des charges ponctuelles.
        private void AddToLoadBuffer(BeamVectorLoad load)
        {
            var index = GetLoadBufferIndex(load);
            IsStaticLoadBufferActive[index] = true;
            MVector.Add(load.Value, ref StaticLoadBuffers[index]);
        }
        private static int GetLoadBufferIndex(BeamVectorLoad load)
        {
            return GetLoadBufferIndex(load.Type, load.IsDistributed, load.IsGlobal);
        }
        private static int GetLoadBufferIndex(VectorLoadType type, bool isDistributed, bool isGlobal)
        {
            var index = -1;

            if (isGlobal)
            {
                if (type == VectorLoadType.Force)
                {
                    if (!isDistributed)
                    {
                        index = 0;      // Fext_g
                    }
                    else
                    {
                        index = 1;      // fext_g
                    }
                }
                else
                {
                    if (!isDistributed)
                    {
                        index = 2;      // Mext_g
                    }
                    else
                    {
                        index = 3;      // mext_g
                    }
                }
            }
            else
            {
                if (type == VectorLoadType.Force)
                {
                    if (!isDistributed)
                    {
                        index = 4;      // Fext_m
                    }
                    else
                    {
                        index = 5;      // fext_m
                    }
                }
                else
                {
                    if (!isDistributed)
                    {
                        index = 6;      // Mext_m
                    }
                    else
                    {
                        index = 7;      // mext_m
                    }
                }
            }

            return index;
        }
    }
}

using System;
using Newtonsoft.Json;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Runtime.Serialization;
using TMarsupilami.CoreLib3;
using TMarsupilami.MathLib;

namespace TMarsupilami.BenchProblem
{
    /// <summary>
    /// Class that defines a test problem with a single beam with end conditions (free, pin, clamped)
    /// TO be serialized or desirialized in a JSON file and benchmarked outside from rhino.
    /// </summary>
    public class SingleBeamProblem
    {
        // Support Conditions
        public int Start { get; set; }  // 0,1,2 (FEE, PIN, CLAMP)
        public int End { get;  set; }   // 0,1,2 (FEE, PIN, CLAMP)

        // Configurations
        public List<MFrame> RestConfiguration { get;  set; }
        public List<MFrame> ActualConfiguration { get;  set; }

        // Rectangular Cross-Section
        public double b1 { get; set; }
        public double b2 { get; set; }

        // Material
        public double E { get; set; }
        public double G { get; set; }

        public SingleBeamProblem()
        {
            this.RestConfiguration = new List<MFrame>();
            this.ActualConfiguration = new List<MFrame>();
        }
        public SingleBeamProblem(IEnumerable<MFrame> restConfig, IEnumerable<MFrame> actualConfig, int start, int end, double b1, double b2, double E, double G)
        {
            this.RestConfiguration = restConfig.ToList();
            this.ActualConfiguration = actualConfig.ToList();
            this.Start = start;
            this.End = end;
            this.b1 = b1;
            this.b2 = b2;
            this.E = E;
            this.G = G;
        }

        public static string Serialize(SingleBeamProblem problem, string path)
        {
            string output = JsonConvert.SerializeObject(problem, Formatting.Indented);
            File.WriteAllText(path, output);
            return output;
        }
        public static SingleBeamProblem DeSerialize(string path)
        {
            var json = File.ReadAllText(path);
            SingleBeamProblem problem = JsonConvert.DeserializeObject<SingleBeamProblem>(json);
            return problem;
        }


        public static SingleBeamProblem Relax(SingleBeamProblem problem, int iteration_max, bool display=false)
        {
            var Ec_x_lim = 1e-10;
            var Ec_θ_lim = 1e-6;

            int n = problem.RestConfiguration.Count;
            var sections = new List<Section>();
            var materials = new List<Material>();

            for (int i = 0; i < n - 1; i++)
            {
                sections.Add(Section.RectangularSection(problem.b1, problem.b2));
            }

            materials.Add(new Material(StandardMaterials.GFRP));

            // INIT
            var beam = new Beam_4DOF_D(problem.RestConfiguration, problem.ActualConfiguration, sections, materials);
            var  elements = new Beam[1] { beam };

            var bc_list = new List<Support>();
            switch (problem.Start)
            {
                case (int)SupportCondition.Free:
                    break;
                case (int)SupportCondition.Clamped:
                    Support.AddClampedSupport(elements[0], Boundary.Start);
                    break;
                default:
                    Support.AddPinnedSupport(elements[0], Boundary.Start);
                    break;
            }
            switch (problem.End)
            {
                case (int)SupportCondition.Free:
                    break;
                case (int)SupportCondition.Clamped:
                    Support.AddClampedSupport(elements[0], Boundary.End);
                    break;
                default:
                    Support.AddPinnedSupport(elements[0], Boundary.End);
                    break;
            }


            // Add Some Loadds
            int nh_mid = elements[0].Nvh / 2;
            var loads = new List<BeamVectorLoad>();
            loads.Add(BeamVectorLoad.Create_Fext(new MVector(0,0,100), nh_mid, beam, true));
            //loads.Add(BeamVectorLoad.Create_Mext(M, nh_mid, beam, false));

            //int ng_mid = elements[0].Nvg / 2;
            //for (int i = 0; i < ng_mid; i++)
            //{
            //    loads.Add(BeamVectorLoad.Create_fext(f, i, beam, true));
            //    loads.Add(BeamVectorLoad.Create_mext(m, i, beam, false));
            //}

            beam.Load(loads);

            var solver = new KDRSolver(elements, bc_list, new List<Link>(), iteration_max, Ec_x_lim, Ec_θ_lim);
            if(display)
            {
                solver.OnEnergyPeak_x += OnKineticEnergyPeak_x;
                solver.OnConvergence += OnConvergence;
                solver.OnNotConvergence += OnNotConvergence;
            }
            else
            {
                solver.OnEnergyPeak_x += Nothing;
                solver.OnConvergence += Nothing;
                solver.OnNotConvergence += Nothing;
            }
            

            var watch = new Stopwatch();
            watch.Start();
            solver.Run(iteration_max);
            watch.Stop();
            Console.WriteLine(("Elasped = " + watch.ElapsedMilliseconds));
     

            SingleBeamProblem pb = new SingleBeamProblem(beam.RestConfiguration, beam.ActualConfiguration, problem.Start, problem.End, problem.b1, problem.b2, problem.E, problem.G);
            return pb;
        }

        private static void OnKineticEnergyPeak_x(KDRSolver solver)
        {
            Console.WriteLine("EC_x[" + solver.NumberOfKineticPeaks_x + "] = " + string.Format("{0:E2}", solver.Ec_x));
        }
        private static void OnConvergence(KDRSolver solver)
        {
            Console.WriteLine("Ec_x[CVG = " + solver.CurrentIteration_x + "] = " + string.Format("{0:E8}", solver.Ec_x));
            Console.WriteLine("Ec_θ[CVG = " + solver.CurrentIteration_θ + "] = " + string.Format("{0:E8}", solver.Ec_θ));
        }
        private static void OnNotConvergence(KDRSolver solver)
        {
            Console.WriteLine("Ec_x[ENDED = " + solver.CurrentIteration_x + "] = " + string.Format("{0:E8}", solver.Ec_x));
            Console.WriteLine("Ec_θ[ENDED = " + solver.CurrentIteration_θ + "] = " + string.Format("{0:E8}", solver.Ec_θ));
        }

        private static void Nothing(KDRSolver solver)
        {
        }
    }
}
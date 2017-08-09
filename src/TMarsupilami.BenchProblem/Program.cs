using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Newtonsoft.Json;
using TMarsupilami.MathLib;
using System.IO;
using TMarsupilami.CoreLib3;

namespace TMarsupilami.BenchProblem
{
    class Program
    {
        public static SingleBeamProblem WriteTestProblem(string dir, string filename = "input")
        {
            int ns = 10;
            var restConfiguration = new MFrame[ns + 1];
            var actualConfiguration = new MFrame[ns + 1];

            double L0 = 10;
            var elongation = 1.2;

            for (int i = 0; i < ns + 1; i++)
            {
                var x = i * (L0 / ns);
                var P_r = new MPoint(elongation * x, 0, 0);
                var P_i = new MPoint(x, 0, 0);

                restConfiguration[i] = new MFrame(P_r, new MVector(0, 0, 1), new MVector(0, 1, 0), new MVector(1, 0, 0));
                actualConfiguration[i] = new MFrame(P_i, new MVector(0, 0, 1), new MVector(0, 1, 0), new MVector(1, 0, 0));
            }


            var pb = new SingleBeamProblem(restConfiguration, actualConfiguration, 2, 1, 0.1, 0.2, 0, 0);
            SingleBeamProblem.Serialize(pb, dir + filename + ".json");
            return pb;
        }
        static void Main(string[] args)
        {
            Console.WriteLine("Pouet");

            SingleBeamProblem pb;

            string dir = @"C:\Users\Véronique\Lionel\Github\TMP\";
            string pbName = "test";

            pb = WriteTestProblem(dir, "test");

        
            // load a SingleBeamProblem
            pb = SingleBeamProblem.DeSerialize(dir + pbName + ".json");
            var res = SingleBeamProblem.Relax(pb, 10000, true);

            // write results
            SingleBeamProblem.Serialize(res, dir + pbName + "_res.json");


            //SingleBeamProblem.Serialize(pb, dir  + "input.json");
            ////var res = SingleBeamProblem.Relax(pb, 10000, true);
            ////res = SingleBeamProblem.Relax(pb, 10000, false);
            ////SingleBeamProblem.Serialize(res, dir + "output.json");
            

           Console.ReadLine();


            

        
        }

        
    }
}

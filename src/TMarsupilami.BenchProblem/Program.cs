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
        static void Main(string[] args)
        {
            Console.WriteLine("Pouet");

            int ns = 10;
            var restConfiguration = new MFrame[ns+1];
            var actualConfiguration = new MFrame[ns+1]; 

            double L0 = 10;
            var elongation = 1.2;

            for (int i = 0; i < ns + 1; i++)
            {
                var x = i*(L0/ns);
                var P_r = new MPoint(elongation*x, 0, 0);
                var P_i = new MPoint(x, 0, 0);

                restConfiguration[i] = new MFrame(P_r, new MVector(0, 0, 1), new MVector(0, 1, 0), new MVector(1, 0, 0));
                actualConfiguration[i] = new MFrame(P_i, new MVector(0, 0, 1), new MVector(0, 1, 0), new MVector(1, 0, 0));
            }


            var pb = new SingleBeamProblem(restConfiguration, actualConfiguration, 1, 1, 0.1, 0.2, 20e9, 5e9);
            string dir = @"C:\Users\Admin\Desktop\";

            SingleBeamProblem.Serialize(pb, dir  + "input.json");
            var res = SingleBeamProblem.Relax(pb, 10000, true);
            res = SingleBeamProblem.Relax(pb, 10000, false);
            SingleBeamProblem.Serialize(res, dir + "output.json");
            
            //string output = JsonConvert.SerializeObject(pb, Formatting.Indented);


            
            //File.WriteAllText(path, output);

            //var json = File.ReadAllText(path);
            //Console.Write(json);
            //SingleBeamProblem pb2 = JsonConvert.DeserializeObject<SingleBeamProblem>(json);
            //SingleBeamProblem pb2 = JsonConvert.DeserializeObject<SingleBeamProblem>(json, 
            //    new JsonSerializerSettings
            //    {
            //        ConstructorHandling = ConstructorHandling.AllowNonPublicDefaultConstructor
            //    });
            ////using (StreamReader file = File.OpenText(path))
            //{
            //    JsonSerializer serializer = new JsonSerializer();
            //    pb2 = (SingleBeamProblem)serializer.Deserialize(file, typeof(SingleBeamProblem));
            //}

            //for (int i = 0; i < pb2.ActualConfiguration.Count; i++)
            //{

            //    Console.WriteLine("[" + i + "] = " + pb2.ActualConfiguration[i].ToString());
            //}

            Console.ReadLine();


            

        
        }

        
    }
}

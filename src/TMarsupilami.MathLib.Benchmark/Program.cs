using BenchmarkDotNet.Running;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace TMarsupilamiMath.Benchmark
{
    class Program
    {
        static void Main(string[] args)
        {
            var summary = BenchmarkRunner.Run<Arithmetic_noloop>();
            //var summary = BenchmarkRunner.Run<Arithmetic_loop>();

            //Console.WriteLine("IsHardwareAccelerated = " + Vector.IsHardwareAccelerated);
            //Console.WriteLine("Vector<int>.Count = " + Vector<int>.Count);
            //Console.WriteLine("Vector<float>.Count = " + Vector<float>.Count);
            //Console.WriteLine("Vector<double>.Count = " + Vector<double>.Count);
            //Console.ReadLine();
            //var obj = new Arithmetic_loop();
            //obj.Sqrt();
        }
    }
}

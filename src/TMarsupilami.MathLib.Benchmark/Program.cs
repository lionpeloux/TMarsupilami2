using BenchmarkDotNet.Reports;
using BenchmarkDotNet.Running;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using TMarsupilami.MathLib.Benchmark;

namespace TMarsupilami.MathLib.Benchmark
{
    class Program
    {
        static void Main(string[] args)
        {
            Summary sum;
            //var summary = BenchmarkRunner.Run<Arithmetic_noloop>();
            //var summary = BenchmarkRunner.Run<Bench_Rotation>();
            //sum = BenchmarkRunner.Run<Bench_Rotation>();
            sum = BenchmarkRunner.Run<Bench_ParallelTransport>();

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

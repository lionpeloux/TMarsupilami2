using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Attributes.Exporters;
using BenchmarkDotNet.Attributes.Jobs;
using System;
using System.Collections.Generic;

namespace TMarsupilami.MathLib.Benchmark
{
       //[MarkdownExporter, AsciiDocExporter, HtmlExporter, CsvExporter, LegacyJitX86Job, LegacyJitX64Job, RyuJitX64Job]
    [HtmlExporter, CsvExporter, RyuJitX64Job]
    public class Bench_Arithmetic
    {
        int n;
        double a, b, c;

        public Bench_Arithmetic()
        {
            n = 10*1000*1000;
            a = Math.Sqrt(Math.E*Math.Log(7));
            b = Math.Sqrt(Math.PI/2.75);
            c = 0;
        }

        [Benchmark(Baseline = true)]
        public double Add()
        {
            for (int i = 0; i < n; i++)
            {
                c = a + b;
            }
            return c;
        }

        [Benchmark]
        public double Sub()
        {
            for (int i = 0; i < n; i++)
            {
                c = a - b;
            }
            return c;
        }

        [Benchmark]
        public double Mul()
        {
            for (int i = 0; i < n; i++)
            {
                c = a * b;
            }
            return c;
        }

        [Benchmark]
        public double Inv()
        {
            for (int i = 0; i < n; i++)
            {
                c = (1 / a);
            }
            return c;
        }

        [Benchmark]
        public double Div()
        {
            for (int i = 0; i < n; i++)
            {
                c = a / b;
            }
            return c;
        }

        [Benchmark]
        public double Sqrt()
        {
            for (int i = 0; i < n; i++)
            {
                c = Math.Sqrt(a);
            }
            return c;
        }

    }

    [HtmlExporter, CsvExporter, LegacyJitX64Job]
    public class Arithmetic_noloop
    {
        int n;
        double a, b, c;

        public Arithmetic_noloop()
        {
            n = 10 * 1000 * 1000;
            a = Math.Sqrt(Math.E * Math.Log(7));
            b = Math.Sqrt(Math.PI / 2.75);
            c = 0;
        }

        [Benchmark(Baseline = true)]
        public double Add()
        {
            return a + b;
        }

        [Benchmark]
        public double Sub()
        {
            return a - b;
        }

        [Benchmark]
        public double Mul()
        {
            return a * b;
        }

        [Benchmark]
        public double Inv()
        {
            return 1/a;
        }

        [Benchmark]
        public double Div()
        {
            return a/b;
        }

        [Benchmark]
        public double Sqrt()
        {
            return Math.Sqrt(a);
        }

    }

}
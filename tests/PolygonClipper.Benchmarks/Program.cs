// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using BenchmarkDotNet.Columns;
using BenchmarkDotNet.Configs;
using BenchmarkDotNet.Exporters;
using BenchmarkDotNet.Jobs;
using BenchmarkDotNet.Loggers;
using BenchmarkDotNet.Running;

namespace SixLabors.PolygonClipper.Benchmarks;

internal sealed class Program
{
    public static void Main(string[] args)
        => BenchmarkSwitcher
        .FromAssembly(typeof(Program).Assembly)
        .Run(args, new ShortRunConfig());
}

public class ShortRunConfig : ManualConfig
{
    public ShortRunConfig()
    {
        this.AddLogger(ConsoleLogger.Default);

        this.AddColumnProvider(DefaultColumnProviders.Instance);

        this.AddExporter(DefaultExporters.Html, DefaultExporters.Csv);

        this.AddJob(Job.ShortRun);
    }
}

// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Collections;

namespace SixLabors.PolygonClipper;

internal abstract class PolyPathBase : IEnumerable
{
    protected PolyPathBase(PolyPathBase? parent = null) => this.Parent = parent;

    public int Count => this.Children.Count;

    public bool IsHole => this.GetIsHole();

    protected PolyPathBase? Parent { get; set; }

    protected List<PolyPathBase> Children { get; } = [];

    public IEnumerator GetEnumerator() => new NodeEnumerator(this.Children);

    public abstract PolyPathBase AddChild(Contour path);

    public void Clear() => this.Children.Clear();

    private int GetLevel()
    {
        int result = 0;
        PolyPathBase? current = this.Parent;
        while (current != null)
        {
            result++;
            current = current.Parent;
        }

        return result;
    }

    private bool GetIsHole()
    {
        int level = this.GetLevel();
        return level != 0 && (level & 1) == 0;
    }

    private sealed class NodeEnumerator : IEnumerator
    {
        private int position = -1;
        private readonly List<PolyPathBase> nodes;

        public NodeEnumerator(List<PolyPathBase> nodes) => this.nodes = [.. nodes];

        public object Current
        {
            get
            {
                if (this.position < 0 || this.position >= this.nodes.Count)
                {
                    throw new InvalidOperationException();
                }

                return this.nodes[this.position];
            }
        }

        public bool MoveNext()
        {
            this.position++;
            return this.position < this.nodes.Count;
        }

        public void Reset() => this.position = -1;
    }
}

internal class PolyPath : PolyPathBase
{
    public PolyPath(PolyPathBase? parent = null)
        : base(parent)
    {
    }

    public Contour? Contour { get; private set; }

    public override PolyPathBase AddChild(Contour path)
    {
        PolyPathBase child = new PolyPath(this);
        (child as PolyPath)!.Contour = path;
        this.Children.Add(child);
        return child;
    }
}

internal sealed class PolyTree : PolyPath
{
    public PolyTree()
        : base(null)
    {
    }
}

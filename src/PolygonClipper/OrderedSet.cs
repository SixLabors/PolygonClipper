// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Represents a sorted set that supports duplicate keys and stable iteration order.
/// </summary>
/// <remarks>
/// Ordering is defined by the supplied comparer. Duplicate keys are permitted and are
/// ordered by insertion sequence, which provides deterministic iteration order.
///
/// Performance: insertion, removal, and lookups are O(log n) due to the red-black tree
/// backing store, while neighbor traversal (Prev/Next) is O(1) via an in-order linked list.
/// Enumeration is O(n) and allocates no per-element memory.
/// </remarks>
/// <typeparam name="T">The element type.</typeparam>
public sealed class OrderedSet<T> : IEnumerable<T>
{
    private Node? root;
    private Node? minNode;
    private Node? maxNode;

    /// <summary>
    /// Initializes a new instance of the <see cref="OrderedSet{T}"/> class.
    /// </summary>
    public OrderedSet()
        : this(Comparer<T>.Default)
    {
    }

    /// <summary>
    /// Initializes a new instance of the <see cref="OrderedSet{T}"/> class with a custom comparer.
    /// </summary>
    /// <param name="comparer">The comparer used to order items.</param>
    public OrderedSet(IComparer<T> comparer)
        => this.Comparer = comparer ?? throw new ArgumentNullException(nameof(comparer));

    /// <summary>
    /// Gets the comparer used to order items.
    /// </summary>
    public IComparer<T> Comparer { get; }

    /// <summary>
    /// Gets the number of items in the set.
    /// </summary>
    public int Count { get; private set; }

    internal Node? MinNode => this.minNode;

    internal Node? MaxNode => this.maxNode;

    /// <summary>
    /// Gets the smallest value in the set.
    /// </summary>
    public T Min => this.minNode == null ? throw new InvalidOperationException("The set is empty.") : this.minNode.Value;

    /// <summary>
    /// Gets the largest value in the set.
    /// </summary>
    public T Max => this.maxNode == null ? throw new InvalidOperationException("The set is empty.") : this.maxNode.Value;

    /// <summary>
    /// Removes all items from the set.
    /// </summary>
    public void Clear()
    {
        this.root = null;
        this.minNode = null;
        this.maxNode = null;
        this.Count = 0;
    }

    /// <summary>
    /// Determines whether the set contains the specified value.
    /// </summary>
    /// <param name="value">The value to locate.</param>
    /// <returns><see langword="true"/> if the value exists; otherwise, <see langword="false"/>.</returns>
    public bool Contains(in T value) => this.FindNode(value) != null;

    /// <summary>
    /// Inserts a value into the set.
    /// </summary>
    /// <param name="value">The value to insert.</param>
    public void Add(in T value) => _ = this.AddNode(value);

    /// <summary>
    /// Removes the specified value from the set.
    /// </summary>
    /// <param name="value">The value to remove.</param>
    /// <returns><see langword="true"/> if a matching value was removed; otherwise, <see langword="false"/>.</returns>
    public bool Remove(in T value)
    {
        Node? node = this.FindNode(value);
        if (node == null)
        {
            return false;
        }

        this.RemoveNode(node);
        return true;
    }

    /// <summary>
    /// Attempts to get the smallest value in the set.
    /// </summary>
    /// <param name="value">When this method returns, contains the smallest value.</param>
    /// <returns><see langword="true"/> if the set is non-empty; otherwise, <see langword="false"/>.</returns>
    public bool TryGetMin(out T value)
    {
        if (this.minNode == null)
        {
            value = default!;
            return false;
        }

        value = this.minNode.Value;
        return true;
    }

    /// <summary>
    /// Attempts to get the largest value in the set.
    /// </summary>
    /// <param name="value">When this method returns, contains the largest value.</param>
    /// <returns><see langword="true"/> if the set is non-empty; otherwise, <see langword="false"/>.</returns>
    public bool TryGetMax(out T value)
    {
        if (this.maxNode == null)
        {
            value = default!;
            return false;
        }

        value = this.maxNode.Value;
        return true;
    }

    /// <summary>
    /// Attempts to get the first value greater than or equal to the specified value.
    /// </summary>
    /// <param name="value">The value to compare.</param>
    /// <param name="result">When this method returns, contains the matching value.</param>
    /// <returns><see langword="true"/> if a value was found; otherwise, <see langword="false"/>.</returns>
    public bool TryGetLowerBound(in T value, out T result)
    {
        Node? node = this.LowerBoundNode(value);
        if (node == null)
        {
            result = default!;
            return false;
        }

        result = node.Value;
        return true;
    }

    /// <summary>
    /// Attempts to get the first value strictly greater than the specified value.
    /// </summary>
    /// <param name="value">The value to compare.</param>
    /// <param name="result">When this method returns, contains the matching value.</param>
    /// <returns><see langword="true"/> if a value was found; otherwise, <see langword="false"/>.</returns>
    public bool TryGetUpperBound(in T value, out T result)
    {
        Node? node = this.UpperBoundNode(value);
        if (node == null)
        {
            result = default!;
            return false;
        }

        result = node.Value;
        return true;
    }

    /// <summary>
    /// Returns an enumerator that iterates through the set in ascending order.
    /// </summary>
    /// <returns>An enumerator for the set.</returns>
    public Enumerator GetEnumerator() => new(this);

    /// <inheritdoc />
    IEnumerator<T> IEnumerable<T>.GetEnumerator() => this.GetEnumerator();

    /// <inheritdoc />
    IEnumerator IEnumerable.GetEnumerator() => this.GetEnumerator();

    internal Node AddNode(in T value)
    {
        Node node = new(value);
        if (this.root == null)
        {
            node.IsRed = false;
            this.root = node;
            this.minNode = node;
            this.maxNode = node;
            this.Count = 1;
            return node;
        }

        Node? current = this.root;
        Node? parent = null;
        Node? predecessor = null;
        Node? successor = null;
        int compare = 0;

        while (current != null)
        {
            parent = current;
            compare = this.Comparer.Compare(value, current.Value);
            if (compare < 0)
            {
                // Track the next-larger node so we can link the in-order list in O(1).
                successor = current;
                current = current.Left;
            }
            else
            {
                // Track the next-smaller node so we can link the in-order list in O(1).
                predecessor = current;
                current = current.Right;
            }
        }

        node.Parent = parent;
        if (compare < 0)
        {
            parent!.Left = node;
        }
        else
        {
            parent!.Right = node;
        }

        // Link the in-order list before rebalancing so neighbor access remains valid.
        this.LinkNode(node, predecessor, successor);
        this.InsertFixup(node);
        this.Count++;
        return node;
    }

    internal void RemoveNode(Node node)
    {
        // Keep the in-order list consistent before we alter tree structure.
        this.UnlinkNode(node);

        Node? y = node;
        Node? x;
        Node? xParent;
        bool yOriginalRed = y.IsRed;

        if (node.Left == null)
        {
            x = node.Right;
            xParent = node.Parent;
            this.Transplant(node, node.Right);
        }
        else if (node.Right == null)
        {
            x = node.Left;
            xParent = node.Parent;
            this.Transplant(node, node.Left);
        }
        else
        {
            // Replace with the successor to preserve in-order ordering.
            y = Minimum(node.Right);
            yOriginalRed = y.IsRed;
            x = y.Right;
            if (y.Parent == node)
            {
                xParent = y;
                if (x != null)
                {
                    x.Parent = y;
                }
            }
            else
            {
                xParent = y.Parent;
                this.Transplant(y, y.Right);
                y.Right = node.Right;
                y.Right.Parent = y;
            }

            this.Transplant(node, y);
            y.Left = node.Left;
            y.Left.Parent = y;
            y.IsRed = node.IsRed;
        }

        if (!yOriginalRed)
        {
            this.DeleteFixup(x, xParent);
        }

        this.Count--;
        node.Parent = null;
        node.Left = null;
        node.Right = null;
        node.Previous = null;
        node.Next = null;
    }

    internal Node? LowerBoundNode(in T value)
    {
        Node? current = this.root;
        Node? candidate = null;
        while (current != null)
        {
            int compare = this.Comparer.Compare(current.Value, value);
            if (compare < 0)
            {
                current = current.Right;
            }
            else
            {
                candidate = current;
                current = current.Left;
            }
        }

        return candidate;
    }

    internal Node? UpperBoundNode(in T value)
    {
        Node? current = this.root;
        Node? candidate = null;
        while (current != null)
        {
            int compare = this.Comparer.Compare(current.Value, value);
            if (compare <= 0)
            {
                current = current.Right;
            }
            else
            {
                candidate = current;
                current = current.Left;
            }
        }

        return candidate;
    }

    internal Node? FindNode(in T value)
    {
        Node? current = this.root;
        Node? candidate = null;
        while (current != null)
        {
            int compare = this.Comparer.Compare(current.Value, value);
            if (compare < 0)
            {
                current = current.Right;
            }
            else if (compare > 0)
            {
                current = current.Left;
            }
            else
            {
                candidate = current;
                current = current.Left;
            }
        }

        return candidate;
    }

    private static Node Minimum(Node node)
    {
        while (node.Left != null)
        {
            node = node.Left;
        }

        return node;
    }

    private void LinkNode(Node node, Node? predecessor, Node? successor)
    {
        node.Previous = predecessor;
        node.Next = successor;
        if (predecessor != null)
        {
            predecessor.Next = node;
        }
        else
        {
            this.minNode = node;
        }

        if (successor != null)
        {
            successor.Previous = node;
        }
        else
        {
            this.maxNode = node;
        }
    }

    private void UnlinkNode(Node node)
    {
        if (node.Previous != null)
        {
            node.Previous.Next = node.Next;
        }
        else
        {
            this.minNode = node.Next;
        }

        if (node.Next != null)
        {
            node.Next.Previous = node.Previous;
        }
        else
        {
            this.maxNode = node.Previous;
        }
    }

    private void InsertFixup(Node node)
    {
        // Restore red-black properties after insert.
        while (node.Parent != null && node.Parent.IsRed)
        {
            Node parent = node.Parent;
            Node grandParent = parent.Parent!;
            if (parent == grandParent.Left)
            {
                Node? uncle = grandParent.Right;
                if (IsRed(uncle))
                {
                    parent.IsRed = false;
                    uncle!.IsRed = false;
                    grandParent.IsRed = true;
                    node = grandParent;
                }
                else
                {
                    if (node == parent.Right)
                    {
                        node = parent;
                        this.RotateLeft(node);
                        parent = node.Parent!;
                        grandParent = parent.Parent!;
                    }

                    parent.IsRed = false;
                    grandParent.IsRed = true;
                    this.RotateRight(grandParent);
                }
            }
            else
            {
                Node? uncle = grandParent.Left;
                if (IsRed(uncle))
                {
                    parent.IsRed = false;
                    uncle!.IsRed = false;
                    grandParent.IsRed = true;
                    node = grandParent;
                }
                else
                {
                    if (node == parent.Left)
                    {
                        node = parent;
                        this.RotateRight(node);
                        parent = node.Parent!;
                        grandParent = parent.Parent!;
                    }

                    parent.IsRed = false;
                    grandParent.IsRed = true;
                    this.RotateLeft(grandParent);
                }
            }
        }

        this.root!.IsRed = false;
    }

    private void DeleteFixup(Node? node, Node? parent)
    {
        // Restore red-black properties after delete.
        while (node != this.root && IsBlack(node))
        {
            if (parent == null)
            {
                break;
            }

            if (node == parent.Left)
            {
                Node? sibling = parent.Right;
                if (IsRed(sibling))
                {
                    sibling!.IsRed = false;
                    parent.IsRed = true;
                    this.RotateLeft(parent);
                    sibling = parent.Right;
                }

                if (IsBlack(sibling?.Left) && IsBlack(sibling?.Right))
                {
                    if (sibling != null)
                    {
                        sibling.IsRed = true;
                    }

                    node = parent;
                    parent = node.Parent;
                }
                else
                {
                    if (IsBlack(sibling?.Right))
                    {
                        if (sibling?.Left != null)
                        {
                            sibling.Left.IsRed = false;
                        }

                        if (sibling != null)
                        {
                            sibling.IsRed = true;
                            this.RotateRight(sibling);
                        }

                        sibling = parent.Right;
                    }

                    if (sibling != null)
                    {
                        sibling.IsRed = parent.IsRed;
                    }

                    parent.IsRed = false;
                    if (sibling?.Right != null)
                    {
                        sibling.Right.IsRed = false;
                    }

                    this.RotateLeft(parent);
                    node = this.root;
                    parent = null;
                }
            }
            else
            {
                Node? sibling = parent.Left;
                if (IsRed(sibling))
                {
                    sibling!.IsRed = false;
                    parent.IsRed = true;
                    this.RotateRight(parent);
                    sibling = parent.Left;
                }

                if (IsBlack(sibling?.Left) && IsBlack(sibling?.Right))
                {
                    if (sibling != null)
                    {
                        sibling.IsRed = true;
                    }

                    node = parent;
                    parent = node.Parent;
                }
                else
                {
                    if (IsBlack(sibling?.Left))
                    {
                        if (sibling?.Right != null)
                        {
                            sibling.Right.IsRed = false;
                        }

                        if (sibling != null)
                        {
                            sibling.IsRed = true;
                            this.RotateLeft(sibling);
                        }

                        sibling = parent.Left;
                    }

                    if (sibling != null)
                    {
                        sibling.IsRed = parent.IsRed;
                    }

                    parent.IsRed = false;
                    if (sibling?.Left != null)
                    {
                        sibling.Left.IsRed = false;
                    }

                    this.RotateRight(parent);
                    node = this.root;
                    parent = null;
                }
            }
        }

        if (node != null)
        {
            node.IsRed = false;
        }
    }

    private void RotateLeft(Node node)
    {
        // Standard red-black left rotation.
        Node? right = node.Right;
        node.Right = right!.Left;
        if (right.Left != null)
        {
            right.Left.Parent = node;
        }

        right.Parent = node.Parent;
        if (node.Parent == null)
        {
            this.root = right;
        }
        else if (node == node.Parent.Left)
        {
            node.Parent.Left = right;
        }
        else
        {
            node.Parent.Right = right;
        }

        right.Left = node;
        node.Parent = right;
    }

    private void RotateRight(Node node)
    {
        // Standard red-black right rotation.
        Node? left = node.Left;
        node.Left = left!.Right;
        if (left.Right != null)
        {
            left.Right.Parent = node;
        }

        left.Parent = node.Parent;
        if (node.Parent == null)
        {
            this.root = left;
        }
        else if (node == node.Parent.Right)
        {
            node.Parent.Right = left;
        }
        else
        {
            node.Parent.Left = left;
        }

        left.Right = node;
        node.Parent = left;
    }

    private void Transplant(Node node, Node? replacement)
    {
        // Replace node with replacement while preserving parent links.
        if (node.Parent == null)
        {
            this.root = replacement;
        }
        else if (node == node.Parent.Left)
        {
            node.Parent.Left = replacement;
        }
        else
        {
            node.Parent.Right = replacement;
        }

        if (replacement != null)
        {
            replacement.Parent = node.Parent;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsRed(Node? node) => node != null && node.IsRed;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsBlack(Node? node) => node == null || !node.IsRed;

    /// <summary>
    /// Enumerates the set in ascending order.
    /// </summary>
    public struct Enumerator : IEnumerator<T>
    {
        private readonly OrderedSet<T> set;
        private Node? current;
        private bool started;

        internal Enumerator(OrderedSet<T> set)
        {
            this.set = set;
            this.current = null;
            this.started = false;
        }

        /// <inheritdoc />
        public T Current => this.current!.Value;

        /// <inheritdoc />
        object IEnumerator.Current => this.Current!;

        /// <inheritdoc />
        public bool MoveNext()
        {
            if (!this.started)
            {
                this.current = this.set.minNode;
                this.started = true;
            }
            else if (this.current != null)
            {
                this.current = this.current.Next;
            }

            return this.current != null;
        }

        /// <inheritdoc />
        public void Reset()
        {
            this.current = null;
            this.started = false;
        }

        /// <inheritdoc />
        public void Dispose()
        {
        }
    }

    /// <summary>
    /// Represents a node stored in the ordered set.
    /// </summary>
    internal sealed class Node
    {
        internal Node(T value)
        {
            this.Value = value;
        }

        internal Node? Parent { get; set; }

        internal Node? Left { get; set; }

        internal Node? Right { get; set; }

        internal bool IsRed { get; set; } = true;

        /// <summary>
        /// Gets the value stored in the node.
        /// </summary>
        public T Value { get; }

        /// <summary>
        /// Gets or sets the previous node in sorted order.
        /// </summary>
        public Node? Previous { get; internal set; }

        /// <summary>
        /// Gets or sets the next node in sorted order.
        /// </summary>
        public Node? Next { get; internal set; }
    }
}

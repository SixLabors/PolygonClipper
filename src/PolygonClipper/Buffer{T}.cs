// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Buffers;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// A disposable buffer that is backed by an array pool.
/// </summary>
/// <typeparam name="T">The type of buffer element.</typeparam>
internal ref struct Buffer<T>
    where T : unmanaged
{
    private readonly byte[] buffer;
    private readonly Span<T> span;
    private bool isDisposed;

    /// <summary>
    /// Initializes a new instance of the <see cref="Buffer{T}"/> struct.
    /// </summary>
    /// <param name="length">The number of elements to rent. Their contents are not initialized.</param>
    public Buffer(int length)
    {
        Guard.MustBeGreaterThanOrEqualTo(length, 0, nameof(length));
        int itemSizeBytes = Unsafe.SizeOf<T>();
        int bufferSizeInBytes = length * itemSizeBytes;
        this.buffer = ArrayPool<byte>.Shared.Rent(bufferSizeInBytes);

        // The buffer owns the rented bytes until Dispose. A direct span cast
        // keeps this lifetime without allocating a memory manager for each rental.
        this.span = MemoryMarshal.Cast<byte, T>(this.buffer.AsSpan())[..length];

        this.isDisposed = false;
    }

    /// <summary>
    /// Gets the requested elements from the rented array.
    /// </summary>
    /// <returns>The elements. Use them only until the buffer is disposed.</returns>
    public readonly Span<T> GetSpan()
    {
        if (this.buffer is null)
        {
            ThrowObjectDisposedException();
        }

        return this.span;
    }

    /// <summary>
    /// Returns the rented array to the pool. Further calls have no effect.
    /// </summary>
    public void Dispose()
    {
        if (this.isDisposed)
        {
            return;
        }

        ArrayPool<byte>.Shared.Return(this.buffer);
        this.isDisposed = true;
    }

    /// <summary>
    /// Reports access to a buffer without a rented array.
    /// </summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void ThrowObjectDisposedException() => throw new ObjectDisposedException("Buffer<T>");
}

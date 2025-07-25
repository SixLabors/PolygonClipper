﻿// Copyright © Joerg Battermann 2014, Matt Hunt 2017

using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text.Json.Serialization;
using GeoJson.Converters;

namespace GeoJson.Geometry
{
    /// <summary>
    /// Defines the LineString type.
    /// </summary>
    /// <remarks>
    /// See https://tools.ietf.org/html/rfc7946#section-3.1.4
    /// </remarks>
    public class LineString : GeoJSONObject, IGeometryObject, IEqualityComparer<LineString>, IEquatable<LineString>
    {
        public LineString()
        {

        }

        /// <summary>
        /// Initializes a new <see cref="LineString" /> from a 2-d array of <see cref="double" />s
        /// that matches the "coordinates" field in the JSON representation.
        /// </summary>
        //[JsonConstructor]
        public LineString(IEnumerable<IEnumerable<double>> coordinates)
        : this(coordinates?.Select(latLongAlt => (IPosition)latLongAlt.ToPosition())
               ?? throw new ArgumentNullException(nameof(coordinates)))
        {
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="LineString" /> class.
        /// </summary>
        /// <param name="coordinates">The coordinates.</param>
        public LineString(IEnumerable<IPosition> coordinates)
        {
            this.Coordinates = new ReadOnlyCollection<IPosition>(
                coordinates?.ToArray() ?? throw new ArgumentNullException(nameof(coordinates)));

            if (this.Coordinates.Count < 2)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(coordinates),
                    "According to the GeoJSON v1.0 spec a LineString must have at least two or more positions.");
            }
        }

        [JsonPropertyName("type")]
        [JsonIgnore(Condition = JsonIgnoreCondition.Never)]
        [JsonConverter(typeof(JsonStringEnumConverter))]
        public override GeoJSONObjectType Type => GeoJSONObjectType.LineString;

        /// <summary>
        /// The positions of the line string.
        /// </summary>
        [JsonPropertyName("coordinates")]
        [JsonConverter(typeof(PositionEnumerableConverter))]
        public ReadOnlyCollection<IPosition> Coordinates { get; set;  }

        /// <summary>
        /// Determines whether this instance has its first and last coordinate at the same position and thereby is closed.
        /// </summary>
        /// <returns>
        /// <c>true</c> if this instance is closed; otherwise, <c>false</c>.
        /// </returns>
        public bool IsClosed()
        {
            IPosition firstCoordinate = this.Coordinates[0];
            IPosition lastCoordinate = this.Coordinates[this.Coordinates.Count - 1];

            return firstCoordinate.Longitude.Equals(lastCoordinate.Longitude)
                   && firstCoordinate.Latitude.Equals(lastCoordinate.Latitude)
                   && Nullable.Equals(firstCoordinate.Altitude, lastCoordinate.Altitude);
        }

        /// <summary>
        /// Determines whether this LineString is a LinearRing.
        /// </summary>
        /// <remarks>
        /// See https://tools.ietf.org/html/rfc7946#section-3.1.1
        /// </remarks>
        /// <returns>
        /// <c>true</c> if it is a linear ring; otherwise, <c>false</c>.
        /// </returns>
        public bool IsLinearRing()
        {
            return this.Coordinates.Count >= 4 && this.IsClosed();
        }

        #region IEqualityComparer, IEquatable

        /// <summary>
        /// Determines whether the specified object is equal to the current object
        /// </summary>
        public override bool Equals(object obj)
        {
            return this.Equals(this, obj as LineString);
        }

        /// <summary>
        /// Determines whether the specified object is equal to the current object
        /// </summary>
        public bool Equals(LineString other)
        {
            return this.Equals(this, other);
        }

        /// <summary>
        /// Determines whether the specified object instances are considered equal
        /// </summary>
        public bool Equals(LineString left, LineString right)
        {
            if (base.Equals(left, right))
            {
                return left.Coordinates.SequenceEqual(right.Coordinates);
            }
            return false;
        }

        /// <summary>
        /// Determines whether the specified object instances are considered equal
        /// </summary>
        public static bool operator ==(LineString left, LineString right)
        {
            if (ReferenceEquals(left, right))
            {
                return true;
            }
            if (right is null)
            {
                return false;
            }
            return left != null && left.Equals(right);
        }

        /// <summary>
        /// Determines whether the specified object instances are not considered equal
        /// </summary>
        public static bool operator !=(LineString left, LineString right)
        {
            return !(left == right);
        }

        /// <summary>
        /// Returns the hash code for this instance
        /// </summary>
        public override int GetHashCode()
        {
            int hash = base.GetHashCode();
            foreach (IPosition? item in this.Coordinates)
            {
                hash = (hash * 397) ^ item.GetHashCode();
            }
            return hash;
        }

        /// <summary>
        /// Returns the hash code for the specified object
        /// </summary>
        public int GetHashCode(LineString other)
        {
            return other.GetHashCode();
        }

        #endregion
    }
}

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
    /// Defines the MultiLineString type.
    /// </summary>
    /// <remarks>
    /// See https://tools.ietf.org/html/rfc7946#section-3.1.5
    /// </remarks>
    public class MultiLineString : GeoJSONObject, IGeometryObject, IEqualityComparer<MultiLineString>, IEquatable<MultiLineString>
    {
        public MultiLineString()
        {

        }

        /// <summary>
        /// Initializes a new instance of the <see cref="MultiLineString" /> class.
        /// </summary>
        /// <param name="coordinates">The coordinates.</param>
        public MultiLineString(IEnumerable<LineString> coordinates)
        {
            this.Coordinates =new ReadOnlyCollection<LineString>(
                coordinates?.ToArray() ?? Array.Empty<LineString>());
        }

        /// <summary>
        /// Initializes a new <see cref="MultiLineString" /> from a 3-d array
        /// of <see cref="double" />s that matches the "coordinates" field in the JSON representation.
        /// </summary>
        //[JsonConstructor]
        public MultiLineString(IEnumerable<IEnumerable<IEnumerable<double>>> coordinates)
            : this(coordinates?.Select(line => new LineString(line))
                   ?? throw new ArgumentNullException(nameof(coordinates)))
        {
        }

        [JsonPropertyName("type")]
        [JsonIgnore(Condition = JsonIgnoreCondition.Never)]
        [JsonConverter(typeof(JsonStringEnumConverter))]
        public override GeoJSONObjectType Type => GeoJSONObjectType.MultiLineString;

        /// <summary>
        /// The collection of line strings of this <see cref="MultiLineString"/>.
        /// </summary>
        [JsonPropertyName("coordinates")]
        [JsonConverter(typeof(LineStringEnumerableConverter))]
        public ReadOnlyCollection<LineString> Coordinates { get; set; }

        #region IEqualityComparer, IEquatable

        /// <summary>
        /// Determines whether the specified object is equal to the current object
        /// </summary>
        public override bool Equals(object obj)
        {
            return this.Equals(this, obj as MultiLineString);
        }

        /// <summary>
        /// Determines whether the specified object is equal to the current object
        /// </summary>
        public bool Equals(MultiLineString other)
        {
            return this.Equals(this, other);
        }

        /// <summary>
        /// Determines whether the specified object instances are considered equal
        /// </summary>
        public bool Equals(MultiLineString left, MultiLineString right)
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
        public static bool operator ==(MultiLineString left, MultiLineString right)
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
        public static bool operator !=(MultiLineString left, MultiLineString right)
        {
            return !(left == right);
        }

        /// <summary>
        /// Returns the hash code for this instance
        /// </summary>
        public override int GetHashCode()
        {
            int hash = base.GetHashCode();
            foreach (LineString? item in this.Coordinates)
            {
                hash = (hash * 397) ^ item.GetHashCode();
            }
            return hash;
        }

        /// <summary>
        /// Returns the hash code for the specified object
        /// </summary>
        public int GetHashCode(MultiLineString other)
        {
            return other.GetHashCode();
        }

        #endregion
    }
}
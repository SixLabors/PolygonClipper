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
    /// Defines the GeometryCollection type.
    /// </summary>
    /// <remarks>
    /// See https://tools.ietf.org/html/rfc7946#section-3.1.8
    /// </remarks>
    public class GeometryCollection : GeoJSONObject, IGeometryObject, IEqualityComparer<GeometryCollection>, IEquatable<GeometryCollection>
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="GeometryCollection" /> class.
        /// </summary>
        public GeometryCollection() : this(Array.Empty<IGeometryObject>())
        {
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="GeometryCollection" /> class.
        /// </summary>
        /// <param name="geometries">The geometries contained in this GeometryCollection.</param>
        public GeometryCollection(IEnumerable<IGeometryObject> geometries)
        {
            this.Geometries = new ReadOnlyCollection<IGeometryObject>(
                geometries?.ToArray() ?? throw new ArgumentNullException(nameof(geometries)));
        }

        [JsonPropertyName("type")]
        [JsonIgnore(Condition = JsonIgnoreCondition.Never)]
        [JsonConverter(typeof(JsonStringEnumConverter))]
        public override GeoJSONObjectType Type => GeoJSONObjectType.GeometryCollection;

        /// <summary>
        /// Gets the list of Polygons enclosed in this MultiPolygon.
        /// </summary>
        [JsonPropertyName("geometries")]
        [JsonConverter(typeof(GeometryEnumerableConverter))]
        public ReadOnlyCollection<IGeometryObject> Geometries { get; set; }

        #region IEqualityComparer, IEquatable

        /// <summary>
        /// Determines whether the specified object is equal to the current object
        /// </summary>
        public override bool Equals(object obj)
        {
            return this.Equals(this, obj as GeometryCollection);
        }

        /// <summary>
        /// Determines whether the specified object is equal to the current object
        /// </summary>
        public bool Equals(GeometryCollection other)
        {
            return this.Equals(this, other);
        }

        /// <summary>
        /// Determines whether the specified object instances are considered equal
        /// </summary>
        public bool Equals(GeometryCollection left, GeometryCollection right)
        {
            if (base.Equals(left, right))
            {
                return left.Geometries.SequenceEqual(right.Geometries);
            }
            return false;
        }

        /// <summary>
        /// Determines whether the specified object instances are considered equal
        /// </summary>
        public static bool operator ==(GeometryCollection left, GeometryCollection right)
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
        public static bool operator !=(GeometryCollection left, GeometryCollection right)
        {
            return !(left == right);
        }

        /// <summary>
        /// Returns the hash code for this instance
        /// </summary>
        public override int GetHashCode()
        {
            int hash = base.GetHashCode();
            foreach (IGeometryObject item in this.Geometries)
            {
                hash = (hash * 397) ^ item.GetHashCode();
            }
            return hash;
        }

        /// <summary>
        /// Returns the hash code for the specified object
        /// </summary>
        public int GetHashCode(GeometryCollection other)
        {
            return other.GetHashCode();
        }

        #endregion
    }
}

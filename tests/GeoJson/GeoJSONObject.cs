﻿// Copyright © Joerg Battermann 2014, Matt Hunt 2017

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json.Serialization;
using GeoJson.Converters;
using GeoJson.CoordinateReferenceSystem;

namespace GeoJson
{
    /// <summary>
    ///     Base class for all IGeometryObject implementing types
    /// </summary>
    public abstract class GeoJSONObject : IGeoJSONObject, IEqualityComparer<GeoJSONObject>, IEquatable<GeoJSONObject>
    {
        internal static readonly DoubleTenDecimalPlaceComparer DoubleComparer = new();

        /// <summary>
        ///     Gets or sets the (optional)
        ///     <see cref="https://tools.ietf.org/html/rfc7946#section-5">Bounding Boxes</see>.
        /// </summary>
        /// <value>
        ///     The value of <see cref="BoundingBoxes" /> must be a 2*n array where n is the number of dimensions represented in
        ///     the
        ///     contained geometries, with the lowest values for all axes followed by the highest values.
        ///     The axes order of a bbox follows the axes order of geometries.
        ///     In addition, the coordinate reference system for the bbox is assumed to match the coordinate reference
        ///     system of the GeoJSON object of which it is a member.
        /// </value>
        [JsonPropertyName("bbox")]
        [JsonConverter(typeof(BoundingBoxConverter))]
        [JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
        public double[] BoundingBoxes { get; set; }

        /// <summary>
        ///     Gets or sets the (optional)
        ///     <see cref="https://tools.ietf.org/html/rfc7946#section-4">
        ///         Coordinate Reference System
        ///         Object.
        ///     </see>
        /// </summary>
        /// <value>
        ///     The Coordinate Reference System Objects.
        /// </value>
        [JsonPropertyName("crs")]
        [JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingDefault)]
        [JsonConverter(typeof(CrsConverter))]
        public ICRSObject CRS { get; set; }

        /// <summary>
        ///     The (mandatory) type of the
        ///     <see cref="https://tools.ietf.org/html/rfc7946#section-3">GeoJSON Object</see>.
        /// </summary>
        [JsonPropertyName("type")]
        [JsonIgnore(Condition = JsonIgnoreCondition.Never)]
        [JsonConverter(typeof(JsonStringEnumConverter))]
        public abstract GeoJSONObjectType Type { get; }


#region IEqualityComparer, IEquatable

        /// <summary>
        /// Determines whether the specified object is equal to the current object
        /// </summary>
        public override bool Equals(object obj)
        {
            return this.Equals(this, obj as GeoJSONObject);
        }

        /// <summary>
        /// Determines whether the specified object is equal to the current object
        /// </summary>
        public bool Equals(GeoJSONObject other)
        {
            return this.Equals(this, other);
        }

        /// <summary>
        /// Determines whether the specified object instances are considered equal
        /// </summary>
        public bool Equals(GeoJSONObject left, GeoJSONObject right)
        {
            if (ReferenceEquals(left, right))
            {
                return true;
            }
            if (right is null)
            {
                return false;
            }

            if (left.Type != right.Type)
            {
                return false;
            }

            if (!Equals(left.CRS, right.CRS))
            {
                return false;
            }

            bool leftIsNull = left.BoundingBoxes is null;
            bool rightIsNull = right.BoundingBoxes is null;
            bool bothAreMissing = leftIsNull && rightIsNull;

            if (bothAreMissing || leftIsNull != rightIsNull)
            {
                return bothAreMissing;
            }

            return left.BoundingBoxes.SequenceEqual(right.BoundingBoxes, DoubleComparer);
        }

        /// <summary>
        /// Determines whether the specified object instances are considered equal
        /// </summary>
        public static bool operator ==(GeoJSONObject left, GeoJSONObject right)
        {
            if (ReferenceEquals(left, right))
            {
                return true;
            }
            if (right is null)
            {
                return false;
            }
            return left.Equals(right);
        }

        /// <summary>
        /// Determines whether the specified object instances are not considered equal
        /// </summary>
        public static bool operator !=(GeoJSONObject left, GeoJSONObject right)
        {
            return !(left == right);
        }

        /// <summary>
        /// Returns the hash code for this instance
        /// </summary>
        public override int GetHashCode()
        {
            return ((int)this.Type).GetHashCode();
        }

        /// <summary>
        /// Returns the hash code for the specified object
        /// </summary>
        public int GetHashCode(GeoJSONObject obj)
        {
            return obj.GetHashCode();
        }

#endregion
    }
}

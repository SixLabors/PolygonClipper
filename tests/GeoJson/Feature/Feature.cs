﻿// Copyright © Joerg Battermann 2014, Matt Hunt 2017

using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text.Json.Serialization;
using GeoJson.Converters;
using GeoJson.Geometry;

namespace GeoJson.Feature
{
    /// <summary>
    /// A GeoJSON Feature Object; generic version for strongly typed <see cref="Geometry"/>
    /// and <see cref="Properties"/>
    /// </summary>
    /// <remarks>
    /// See https://tools.ietf.org/html/rfc7946#section-3.2
    /// </remarks>
    public class Feature<TGeometry, TProps> : GeoJSONObject, IEquatable<Feature<TGeometry, TProps>>
        where TGeometry : IGeometryObject
    {
        private string _id;
        private bool _idHasValue = false;
        private TGeometry _geometry;
        private bool _geometryHasValue = false;
        private TProps _properties;
        private bool _propertiesHasValue = false;

        public Feature()
        {

        }

        public Feature(TGeometry geometry, TProps properties, string id = null)
        {
            this.Geometry = geometry;
            this.Properties = properties;
            this.Id = id;
        }

        public Feature(IGeometryObject geometry, TProps properties, string id = null)
        {
            this.Geometry = (TGeometry)geometry;
            this.Properties = properties;
            this.Id = id;
        }

        [JsonPropertyName("type")]
        [JsonIgnore(Condition = JsonIgnoreCondition.Never)]
        [JsonConverter(typeof(JsonStringEnumConverter))]
        public override GeoJSONObjectType Type => GeoJSONObjectType.Feature;

        [JsonPropertyName( "id")]
        [JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
        public string Id { 
            get
            {
                return this._id;
            }
#if NET5_0_OR_GREATER
            init
            {
                if (this._idHasValue) throw new InvalidOperationException("Id property already set, is read only");

                this._id = value;
                this._idHasValue = true;
            }
#else
            set
            {
                if (_idHasValue) throw new InvalidOperationException("Id property already set, is read only");

                _id = value;
                _idHasValue = true;
            }
#endif
        }

        [JsonPropertyName("geometry")]
        [JsonConverter(typeof(GeometryConverter))]
        public TGeometry Geometry { 
            get
            {
                return this._geometry;
            }
#if NET5_0_OR_GREATER
            init
            {
                if (this._geometryHasValue) throw new InvalidOperationException("Geometry property already set, is read only");

                this._geometry = value;
                this._geometryHasValue = true;
            }
#else
            set
            {
                if (_geometryHasValue) throw new InvalidOperationException("Geometry property already set, is read only");

                _geometry = value;
                _geometryHasValue = true;
            }
#endif
        }

        [JsonPropertyName("properties")]
        public TProps Properties { 
            get
            {
                return this._properties;
            }
#if NET5_0_OR_GREATER
            init
            {
                if (this._propertiesHasValue) throw new InvalidOperationException("Geometry property already set, is read only");

                this._properties = value;
                this._propertiesHasValue = true;
            }
#else
            set
            {
                if (_propertiesHasValue) throw new InvalidOperationException("Geometry property already set, is read only");

                _properties = value;
                _propertiesHasValue = true;
            }
#endif
        }

        /// <summary>
        /// Equality comparer.
        /// </summary>
        /// <remarks>
        /// In contrast to <see cref="Feature.Equals(Feature)"/>, this implementation returns true only
        /// if <see cref="Id"/> and <see cref="Properties"/> are also equal. See
        /// <a href="https://github.com/GeoJSON-Net/GeoJSON.Text/issues/80">#80</a> for discussion. The rationale
        /// here is that a user explicitly specifying the property type most probably cares about the properties
        /// equality.
        /// </remarks>
        /// <param name="other"></param>
        /// <returns></returns>
        public bool Equals(Feature<TGeometry, TProps> other)
        {
            if (other is null) return false;
            if (ReferenceEquals(this, other)) return true;
            return base.Equals(other)
                   && string.Equals(this.Id, other.Id)
                   && EqualityComparer<TGeometry>.Default.Equals(this.Geometry, other.Geometry)
                   && EqualityComparer<TProps>.Default.Equals(this.Properties, other.Properties);
        }

        public override bool Equals(object obj)
        {
            if (obj is null) return false;
            if (ReferenceEquals(this, obj)) return true;
            if (obj.GetType() != this.GetType()) return false;
            return this.Equals((Feature<TGeometry, TProps>) obj);
        }

        public override int GetHashCode()
        {
            unchecked
            {
                int hashCode = base.GetHashCode();
                hashCode = (hashCode * 397) ^ (this.Id != null ? this.Id.GetHashCode() : 0);
                hashCode = (hashCode * 397) ^ EqualityComparer<TGeometry>.Default.GetHashCode(this.Geometry);
                hashCode = (hashCode * 397) ^ EqualityComparer<TProps>.Default.GetHashCode(this.Properties);
                return hashCode;
            }
        }

        public static bool operator ==(Feature<TGeometry, TProps> left, Feature<TGeometry, TProps> right)
        {
            return object.Equals(left, right);
        }

        public static bool operator !=(Feature<TGeometry, TProps> left, Feature<TGeometry, TProps> right)
        {
            return !object.Equals(left, right);
        }
    }


    /// <summary>
    /// A GeoJSON Feature Object.
    /// </summary>
    /// <remarks>
    /// See https://tools.ietf.org/html/rfc7946#section-3.2
    /// </remarks>
    public class Feature : Feature<IGeometryObject>
    {
        public Feature()
        {

        }

        public Feature(IGeometryObject geometry, IDictionary<string, object> properties = null, string id = null)
            : base(geometry, properties, id)
        {
        }

        public Feature(IGeometryObject geometry, object properties, string id = null)
            : base(geometry, properties, id)
        {
        }
    }


    /// <summary>
    /// Typed GeoJSON Feature class
    /// </summary>
    /// <remarks>Returns correctly typed Geometry property</remarks>
    /// <typeparam name="TGeometry"></typeparam>
    public class Feature<TGeometry> : Feature<TGeometry, IDictionary<string, object>>, IEquatable<Feature<TGeometry>> where TGeometry : IGeometryObject
    {
        public Feature()
        {

        }

        /// <summary>
        /// Initializes a new instance of the <see cref="Feature" /> class.
        /// </summary>
        /// <param name="geometry">The Geometry Object.</param>
        /// <param name="properties">The properties.</param>
        /// <param name="id">The (optional) identifier.</param>
        public Feature(TGeometry geometry, IDictionary<string, object> properties = null, string id = null)
        : base(geometry, properties ?? new Dictionary<string, object>(), id)
        {
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="Feature" /> class.
        /// </summary>
        /// <param name="geometry">The Geometry Object.</param>
        /// <param name="properties">The properties.</param>
        /// <param name="id">The (optional) identifier.</param>
        public Feature(IGeometryObject geometry, IDictionary<string, object> properties = null, string id = null)
        : base((TGeometry)geometry, properties ?? new Dictionary<string, object>(), id)
        {
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="Feature" /> class.
        /// </summary>
        /// <param name="geometry">The Geometry Object.</param>
        /// <param name="properties">
        /// Class used to fill feature properties. Any public member will be added to feature
        /// properties
        /// </param>
        /// <param name="id">The (optional) identifier.</param>
        public Feature(TGeometry geometry, object properties, string id = null)
        : this(geometry, GetDictionaryOfPublicProperties(properties), id)
        {
        }


        private static Dictionary<string, object> GetDictionaryOfPublicProperties(object properties)
        {
            if (properties == null)
            {
                return new Dictionary<string, object>();
            }
            return properties.GetType().GetTypeInfo().DeclaredProperties
                .Where(propertyInfo => propertyInfo.GetMethod.IsPublic)
                .ToDictionary(propertyInfo => propertyInfo.Name,
                    propertyInfo => propertyInfo.GetValue(properties, null));
        }

        public bool Equals(Feature<TGeometry> other)
        {
            if (other is null) return false;
            if (ReferenceEquals(this, other)) return true;

            if (this.Geometry == null && other.Geometry == null)
            {
                return true;
            }

            if (this.Geometry == null && other.Geometry != null)
            {
                return false;
            }

            if (this.Geometry == null)
            {
                return false;
            }

            return EqualityComparer<TGeometry>.Default.Equals(this.Geometry, other.Geometry);
        }

        public override bool Equals(object obj)
        {
            if (obj is null) return false;
            if (ReferenceEquals(this, obj)) return true;
            return obj.GetType() == this.GetType() && this.Equals((Feature<TGeometry>) obj);
        }

        public override int GetHashCode()
        {
            return this.Geometry.GetHashCode();
        }

        public static bool operator ==(Feature<TGeometry> left, Feature<TGeometry> right)
        {
            return left?.Equals(right) ?? right is null;
        }

        public static bool operator !=(Feature<TGeometry> left, Feature<TGeometry> right)
        {
            return !(left?.Equals(right) ?? right is null);
        }
    }
}

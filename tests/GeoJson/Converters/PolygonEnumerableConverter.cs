﻿// Copyright © Joerg Battermann 2014, Matt Hunt 2017

using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Text.Json;
using System.Text.Json.Serialization;
using GeoJson.Geometry;

namespace GeoJson.Converters
{
    /// <summary>
    /// Converter to read and write the <see cref="IEnumerable{MultiPolygon}" /> type.
    /// </summary>
    public class PolygonEnumerableConverter : JsonConverter<IReadOnlyCollection<Polygon>>
    {

        private static readonly LineStringEnumerableConverter PolygonConverter = new();
        /// <summary>
        ///     Determines whether this instance can convert the specified object type.
        /// </summary>
        /// <param name="objectType">Type of the object.</param>
        /// <returns>
        ///     <c>true</c> if this instance can convert the specified object type; otherwise, <c>false</c>.
        /// </returns>
        public override bool CanConvert(Type objectType)
        {
            return true || typeof(IReadOnlyCollection<Polygon>).IsAssignableFromType(objectType);
        }

        /// <summary>
        ///     Reads the JSON representation of the object.
        /// </summary>
        /// <param name="reader">The <see cref="T:Newtonsoft.Json.JsonReader" /> to read from.</param>
        /// <param name="objectType">Type of the object.</param>
        /// <param name="existingValue">The existing value of object being read.</param>
        /// <param name="serializer">The calling serializer.</param>
        /// <returns>
        ///     The object value.
        /// </returns>
        public override IReadOnlyCollection<Polygon> Read(
            ref Utf8JsonReader reader,
            Type type,
            JsonSerializerOptions options)
        {

            switch (reader.TokenType)
            {
                case JsonTokenType.Null:
                    return null;
                case JsonTokenType.StartArray:
                    break;
                default:
                    throw new InvalidOperationException("Incorrect json type");
            }

            int startDepth = reader.CurrentDepth;
            List<Polygon>? result = new();
            while (reader.Read())
            {
                if (JsonTokenType.EndArray == reader.TokenType && reader.CurrentDepth == startDepth)
                {
                    return new ReadOnlyCollection<Polygon>(result);
                }
                if (reader.TokenType == JsonTokenType.StartArray)
                {
                    result.Add(new Polygon(PolygonConverter.Read(
                        ref reader,
                        typeof(IEnumerable<LineString>),
                        options)));
                }
            }

            throw new JsonException($"expected null, object or array token but received {reader.TokenType}");
        }

        /// <summary>
        ///     Writes the JSON representation of the object.
        /// </summary>
        /// <param name="writer">The <see cref="T:Newtonsoft.Json.JsonWriter" /> to write to.</param>
        /// <param name="value">The value.</param>
        /// <param name="serializer">The calling serializer.</param>
        public override void Write(
            Utf8JsonWriter writer,
            IReadOnlyCollection<Polygon> value,
            JsonSerializerOptions options)
        {
            writer.WriteStartArray();
            foreach (Polygon? polygon in value)
            {
                PolygonConverter.Write(writer, polygon.Coordinates, options);
            }
            writer.WriteEndArray();
        }
    }
}
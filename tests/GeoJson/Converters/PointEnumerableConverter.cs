// Copyright � Joerg Battermann 2014, Matt Hunt 2017

using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Text.Json;
using System.Text.Json.Serialization;
using GeoJson.Geometry;

namespace GeoJson.Converters
{
    /// <summary>
    /// Converter to read and write the <see cref="IEnumerable{Point}" /> type.
    /// </summary>
    public class PointEnumerableConverter : JsonConverter<ReadOnlyCollection<Point>>
    {
        private static readonly PositionConverter PositionConverter = new();
        /// <inheritdoc />
        public override void Write(
            Utf8JsonWriter writer,
            ReadOnlyCollection<Point> value,
            JsonSerializerOptions options)
        {
            writer.WriteStartArray();
            foreach (Point? point in value)
            {
                PositionConverter.Write(writer, point.Coordinates, options);
            }
            writer.WriteEndArray();
        }

        /// <inheritdoc />
        public override ReadOnlyCollection<Point> Read(
            ref Utf8JsonReader reader,
            Type type,
            JsonSerializerOptions options)
        {
            switch (reader.TokenType)
            {
                // This converter has to read something starting with an array
                case JsonTokenType.StartArray:
                    break;
                case JsonTokenType.Null:
                    return null;
                default:
                    throw new InvalidOperationException("Incorrect json type");
            }

            int startDepth = reader.CurrentDepth;
            List<Point>? result = new();
            List<double> numbers = new();
            while (reader.Read())
            {
                if (JsonTokenType.EndArray == reader.TokenType && reader.CurrentDepth == startDepth)
                {
                    return new ReadOnlyCollection<Point>(result);
                }
                if(JsonTokenType.EndArray == reader.TokenType)
                {
                    result.Add(new Point(numbers.ToPosition()));

                    // We have finished reading this internal point array, clear so we can read next (If needed)
                    numbers.Clear();
                }
                if(reader.TokenType == JsonTokenType.Number)
                {
                    numbers.Add(reader.GetDouble());
                }
            }

            throw new JsonException($"expected null, object or array token but received {reader.TokenType}");
        }

        /// <inheritdoc />
        public override bool CanConvert(Type objectType)
        {
            return objectType == typeof(ReadOnlyCollection<Point>);
        }
    }
}
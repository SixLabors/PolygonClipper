// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper.Tests;

public class PolygonStrokerTests
{
    [Fact]
    public void ProcessPath_ClosedSquare_ReturnsGeometry()
    {
        const double width = 2D;

        Contour square =
        [
            new Vertex(0, 0),
            new Vertex(10, 0),
            new Vertex(10, 10),
            new Vertex(0, 10),
            new Vertex(0, 0)
        ];

        Polygon result = PolygonStroker.Stroke([square], width);

        Assert.True(result.Count > 0);
        Assert.True(result[0].Count >= 4);
    }

    [Fact]
    public void ProcessPath_DegenerateTwoPointPath_ReturnsPointCap()
    {
        const double width = 4D;

        Vertex[] points =
        [
            new(5, 5),
            new(5, 5)
        ];

        Polygon result = PolygonStroker.Stroke([new Contour(points.Length) { points[0], points[1] }], width);
        Assert.True(result.Count > 0);
        Contour contour = result[0];

        Assert.Equal(4, contour.Count);
    }

    [Fact]
    public void ProcessPolygonAndClip_SimplePolygon_ReturnsNonEmptyResult()
    {
        Polygon input =
        [[
            new Vertex(0, 0),
            new Vertex(20, 0),
            new Vertex(20, 20),
            new Vertex(0, 20),
            new Vertex(0, 0)
        ]];

        Polygon result = PolygonStroker.Stroke(input, width: 2D);

        Assert.True(result.Count > 0);
    }

    [Fact]
    public void Stroke_OpenPolylineWithThreeVertices_IsNotForcedClosed()
    {
        const double width = 2D;

        StrokeOptions options = new()
        {
            LineCap = LineCap.Butt,
            LineJoin = LineJoin.Miter,
            InnerJoin = InnerJoin.Miter
        };

        Polygon open =
        [[
            new Vertex(0, 0),
            new Vertex(12, 0),
            new Vertex(12, 9)
        ]];

        Polygon closed =
        [[
            new Vertex(0, 0),
            new Vertex(12, 0),
            new Vertex(12, 9),
            new Vertex(0, 0)
        ]];

        Polygon openResult = PolygonStroker.Stroke(open, width, options);
        Polygon closedResult = PolygonStroker.Stroke(closed, width, options);

        AssertPolygonIsValid(openResult);
        AssertPolygonIsValid(closedResult);

        double openArea = ComputeTotalAbsoluteArea(openResult);
        double closedArea = ComputeTotalAbsoluteArea(closedResult);
        Assert.True(
            Math.Abs(openArea - closedArea) > 1E-6D,
            $"Open and closed path strokes should differ. openArea={openArea:R}, closedArea={closedArea:R}.");
    }

    [Fact]
    public void ProcessPolygonAndClip_FoldBackOpenPath_ProducesValidStroke()
    {
        const double width = 12D;
        StrokeOptions options = new() { NormalizeOutput = true };

        Contour inputContour = CreateFoldBackPolyline();
        if (inputContour[0] != inputContour[^1])
        {
            inputContour.Add(inputContour[0]);
        }

        Polygon input = [inputContour];
        Polygon actual = PolygonStroker.Stroke(input, width, options);

        AssertPolygonIsValid(actual);
        AssertStrokeCoversInputCenterline(input, actual, samplesPerSegment: 3);
    }

    [Fact]
    public void ProcessPolygonAndClip_FigureNinePath_ProducesValidStroke()
    {
        const double width = 10D;
        StrokeOptions options = new() { NormalizeOutput = true };

        Contour inputContour = CreateFigureNinePolyline(72);
        if (inputContour[0] != inputContour[^1])
        {
            inputContour.Add(inputContour[0]);
        }

        Polygon input = [inputContour];
        Polygon actual = PolygonStroker.Stroke(input, width, options);

        AssertPolygonIsValid(actual);
        AssertStrokeCoversInputCenterline(input, actual, samplesPerSegment: 3);
    }

    [Theory (Skip = "For profiling only.")]
    [InlineData(101)]
    [InlineData(301)]
    [InlineData(1001)]
    public void ProcessPolygonAndClip_CompoundGearBenchmarkInput_ProducesStroke(int toothCount)
    {
        const double width = 12D;
        StrokeOptions options = new()
        {
            LineJoin = LineJoin.Round,
            LineCap = LineCap.Round
        };

        Polygon input = BuildCompoundGearPolygon(toothCount);
        Polygon actual = PolygonStroker.Stroke(input, width, options);

        Assert.True(actual.Count > 0);
        for (int i = 0; i < actual.Count; i++)
        {
            Assert.True(actual[i].Count >= 3, $"Contour {i} must have at least 3 vertices.");
        }
    }

    [Fact]
    public void ProcessPolygonAndClip_CompoundCircleGlyphPath_ProducesValidStroke()
    {
        const double width = 5D;

        Polygon input =
[
    [
        new Vertex(1201.9033203125, 717.2041015625),
        new Vertex(1201.8572998046875, 720.0596923828125),
        new Vertex(1201.7196044921875, 722.8502197265625),
        new Vertex(1201.489990234375, 725.5755004882812),
        new Vertex(1201.16845703125, 728.235595703125),
        new Vertex(1200.755126953125, 730.83056640625),
        new Vertex(1200.2498779296875, 733.3604125976562),
        new Vertex(1199.65283203125, 735.8250732421875),
        new Vertex(1198.9638671875, 738.224609375),
        new Vertex(1198.1846923828125, 740.5499267578125),
        new Vertex(1197.31689453125, 742.7918701171875),
        new Vertex(1196.3603515625, 744.9505615234375),
        new Vertex(1195.315185546875, 747.02587890625),
        new Vertex(1194.181396484375, 749.0179443359375),
        new Vertex(1192.958984375, 750.9266357421875),
        new Vertex(1191.6478271484375, 752.7520751953125),
        new Vertex(1190.248046875, 754.494140625),
        new Vertex(1188.7607421875, 756.14599609375),
        new Vertex(1187.1868896484375, 757.70068359375),
        new Vertex(1185.5263671875, 759.1580200195312),
        new Vertex(1183.779541015625, 760.518310546875),
        new Vertex(1181.9461669921875, 761.7813720703125),
        new Vertex(1180.0262451171875, 762.9471435546875),
        new Vertex(1178.019775390625, 764.015869140625),
        new Vertex(1175.9267578125, 764.9873046875),
        new Vertex(1173.747802734375, 765.8525390625),
        new Vertex(1171.4833984375, 766.602294921875),
        new Vertex(1169.133544921875, 767.2367553710938),
        new Vertex(1166.6982421875, 767.755859375),
        new Vertex(1164.177490234375, 768.1596069335938),
        new Vertex(1161.5712890625, 768.447998046875),
        new Vertex(1158.879638671875, 768.62109375),
        new Vertex(1156.1025390625, 768.6787109375),
        new Vertex(1153.24365234375, 768.62109375),
        new Vertex(1150.4779052734375, 768.447998046875),
        new Vertex(1147.8048095703125, 768.1596069335938),
        new Vertex(1145.224853515625, 767.755859375),
        new Vertex(1142.73779296875, 767.2367553710938),
        new Vertex(1140.3436279296875, 766.602294921875),
        new Vertex(1138.0423583984375, 765.8524780273438),
        new Vertex(1135.833984375, 764.9873046875),
        new Vertex(1133.718017578125, 764.015380859375),
        new Vertex(1131.6939697265625, 762.945068359375),
        new Vertex(1129.76171875, 761.7765502929688),
        new Vertex(1127.92138671875, 760.509765625),
        new Vertex(1126.1728515625, 759.144775390625),
        new Vertex(1124.5162353515625, 757.681396484375),
        new Vertex(1122.951416015625, 756.1198120117188),
        new Vertex(1121.478515625, 754.4599609375),
        new Vertex(1120.096435546875, 752.7092895507812),
        new Vertex(1118.803955078125, 750.8753051757812),
        new Vertex(1117.6011962890625, 748.9581298828125),
        new Vertex(1116.48828125, 746.95751953125),
        new Vertex(1115.465087890625, 744.8736572265625),
        new Vertex(1114.531494140625, 742.7064208984375),
        new Vertex(1113.6876220703125, 740.4559326171875),
        new Vertex(1112.93359375, 738.1220703125),
        new Vertex(1112.268798828125, 735.7145385742188),
        new Vertex(1111.6923828125, 733.2428588867188),
        new Vertex(1111.2049560546875, 730.7072143554688),
        new Vertex(1110.805908203125, 728.107421875),
        new Vertex(1110.49560546875, 725.443603515625),
        new Vertex(1110.27392578125, 722.715576171875),
        new Vertex(1110.1409912109375, 719.9235229492188),
        new Vertex(1110.0966796875, 717.0673828125),
        new Vertex(1110.1756591796875, 713.2979125976562),
        new Vertex(1110.412841796875, 709.6461791992188),
        new Vertex(1110.8079833984375, 706.1117553710938),
        new Vertex(1111.361328125, 702.69482421875),
        new Vertex(1112.07275390625, 699.3953857421875),
        new Vertex(1112.942138671875, 696.2135009765625),
        new Vertex(1113.9696044921875, 693.1491088867188),
        new Vertex(1115.1552734375, 690.2021484375),
        new Vertex(1115.8079833984375, 688.778076171875),
        new Vertex(1116.5015869140625, 687.39404296875),
        new Vertex(1117.2362060546875, 686.0501098632812),
        new Vertex(1118.0113525390625, 684.7461547851562),
        new Vertex(1118.82763671875, 683.4823608398438),
        new Vertex(1119.6845703125, 682.258544921875),
        new Vertex(1120.58251953125, 681.0748291015625),
        new Vertex(1121.521240234375, 679.93115234375),
        new Vertex(1122.5008544921875, 678.8275146484375),
        new Vertex(1123.521240234375, 677.763916015625),
        new Vertex(1124.58251953125, 676.7404174804688),
        new Vertex(1125.6846923828125, 675.7569580078125),
        new Vertex(1126.8277587890625, 674.8135986328125),
        new Vertex(1128.0115966796875, 673.91015625),
        new Vertex(1129.236328125, 673.046875),
        new Vertex(1130.501953125, 672.2236328125),
        new Vertex(1131.8084716796875, 671.4454956054688),
        new Vertex(1133.1556396484375, 670.7175903320312),
        new Vertex(1134.5439453125, 670.0398559570312),
        new Vertex(1135.9727783203125, 669.412353515625),
        new Vertex(1137.4427490234375, 668.8350219726562),
        new Vertex(1138.9534912109375, 668.3079223632812),
        new Vertex(1140.5050048828125, 667.8309936523438),
        new Vertex(1142.097412109375, 667.404296875),
        new Vertex(1143.730712890625, 667.0277099609375),
        new Vertex(1145.40478515625, 666.7014770507812),
        new Vertex(1147.119873046875, 666.4253540039062),
        new Vertex(1148.8756103515625, 666.199462890625),
        new Vertex(1152.5098876953125, 665.8982543945312),
        new Vertex(1156.3076171875, 665.7978515625),
        new Vertex(1159.9429931640625, 665.8971557617188),
        new Vertex(1163.431884765625, 666.1952514648438),
        new Vertex(1165.1214599609375, 666.418701171875),
        new Vertex(1166.774658203125, 666.69189453125),
        new Vertex(1168.3909912109375, 667.0147094726562),
        new Vertex(1169.970947265625, 667.38720703125),
        new Vertex(1171.5142822265625, 667.8093872070312),
        new Vertex(1173.02099609375, 668.28125),
        new Vertex(1174.4910888671875, 668.802734375),
        new Vertex(1175.924560546875, 669.3739013671875),
        new Vertex(1177.321533203125, 669.9947509765625),
        new Vertex(1178.6820068359375, 670.665283203125),
        new Vertex(1180.0057373046875, 671.3854370117188),
        new Vertex(1181.29296875, 672.1552734375),
        new Vertex(1182.5426025390625, 672.9700927734375),
        new Vertex(1183.75390625, 673.8253173828125),
        new Vertex(1184.9267578125, 674.7207641601562),
        new Vertex(1186.06103515625, 675.656494140625),
        new Vertex(1187.1568603515625, 676.6326904296875),
        new Vertex(1188.21435546875, 677.6491088867188),
        new Vertex(1189.2332763671875, 678.7058715820312),
        new Vertex(1190.2138671875, 679.802978515625),
        new Vertex(1191.156005859375, 680.9403686523438),
        new Vertex(1192.0595703125, 682.1181030273438),
        new Vertex(1192.9246826171875, 683.336181640625),
        new Vertex(1193.75146484375, 684.5945434570312),
        new Vertex(1194.539794921875, 685.8932495117188),
        new Vertex(1195.28955078125, 687.2322998046875),
        new Vertex(1196.0009765625, 688.611572265625),
        new Vertex(1196.673828125, 690.03125),
        new Vertex(1197.8994140625, 692.9755249023438),
        new Vertex(1198.9617919921875, 696.0490112304688),
        new Vertex(1199.8604736328125, 699.251708984375),
        new Vertex(1200.595947265625, 702.583740234375),
        new Vertex(1201.16796875, 706.044921875),
        new Vertex(1201.5765380859375, 709.6354370117188),
        new Vertex(1201.821533203125, 713.3551025390625),
    ],
    [
        new Vertex(1122.3330078125, 717.2041015625),
        new Vertex(1122.3896484375, 720.3294067382812),
        new Vertex(1122.5594482421875, 723.347900390625),
        new Vertex(1122.842529296875, 726.2596435546875),
        new Vertex(1123.23876953125, 729.064453125),
        new Vertex(1123.7481689453125, 731.7625122070312),
        new Vertex(1124.3709716796875, 734.353759765625),
        new Vertex(1125.10693359375, 736.8381958007812),
        new Vertex(1125.9560546875, 739.2158203125),
        new Vertex(1126.424072265625, 740.3613891601562),
        new Vertex(1126.9222412109375, 741.4738159179688),
        new Vertex(1127.450439453125, 742.5531616210938),
        new Vertex(1128.009033203125, 743.599365234375),
        new Vertex(1128.59765625, 744.612548828125),
        new Vertex(1129.2164306640625, 745.5924682617188),
        new Vertex(1129.865478515625, 746.5393676757812),
        new Vertex(1130.544677734375, 747.453125),
        new Vertex(1131.254150390625, 748.3338012695312),
        new Vertex(1131.99365234375, 749.1813354492188),
        new Vertex(1132.7633056640625, 749.9957885742188),
        new Vertex(1133.563232421875, 750.777099609375),
        new Vertex(1134.3931884765625, 751.5252685546875),
        new Vertex(1135.25341796875, 752.2404174804688),
        new Vertex(1136.1439208984375, 752.92236328125),
        new Vertex(1137.064453125, 753.5712890625),
        new Vertex(1138.0159912109375, 754.183837890625),
        new Vertex(1138.9993896484375, 754.7568969726562),
        new Vertex(1140.014404296875, 755.2904052734375),
        new Vertex(1141.0614013671875, 755.784423828125),
        new Vertex(1142.1400146484375, 756.2388305664062),
        new Vertex(1143.25048828125, 756.6538696289062),
        new Vertex(1144.392578125, 757.029296875),
        new Vertex(1145.566650390625, 757.365234375),
        new Vertex(1146.7724609375, 757.66162109375),
        new Vertex(1148.010009765625, 757.9185791015625),
        new Vertex(1149.279296875, 758.1358642578125),
        new Vertex(1150.580322265625, 758.313720703125),
        new Vertex(1153.27783203125, 758.5508422851562),
        new Vertex(1156.1025390625, 758.6298828125),
        new Vertex(1158.942138671875, 758.55078125),
        new Vertex(1161.650390625, 758.313720703125),
        new Vertex(1162.9552001953125, 758.1358642578125),
        new Vertex(1164.2271728515625, 757.9185180664062),
        new Vertex(1165.46630859375, 757.66162109375),
        new Vertex(1166.672607421875, 757.365234375),
        new Vertex(1167.8460693359375, 757.029296875),
        new Vertex(1168.9866943359375, 756.6538696289062),
        new Vertex(1170.0943603515625, 756.2388916015625),
        new Vertex(1171.1693115234375, 755.784423828125),
        new Vertex(1172.21142578125, 755.2904052734375),
        new Vertex(1173.220703125, 754.7568969726562),
        new Vertex(1174.197021484375, 754.183837890625),
        new Vertex(1175.140625, 753.5712890625),
        new Vertex(1176.0526123046875, 752.92236328125),
        new Vertex(1176.9345703125, 752.2404174804688),
        new Vertex(1177.7862548828125, 751.5252685546875),
        new Vertex(1178.607666015625, 750.777099609375),
        new Vertex(1179.3990478515625, 749.995849609375),
        new Vertex(1180.16015625, 749.1812744140625),
        new Vertex(1180.8912353515625, 748.3338012695312),
        new Vertex(1181.592041015625, 747.453125),
        new Vertex(1182.2626953125, 746.5393676757812),
        new Vertex(1182.903076171875, 745.5924682617188),
        new Vertex(1183.513427734375, 744.6124877929688),
        new Vertex(1184.093505859375, 743.599365234375),
        new Vertex(1184.6435546875, 742.5531616210938),
        new Vertex(1185.163330078125, 741.4738159179688),
        new Vertex(1185.65283203125, 740.3613891601562),
        new Vertex(1186.1123046875, 739.2158203125),
        new Vertex(1186.9454345703125, 736.8381958007812),
        new Vertex(1187.6673583984375, 734.353759765625),
        new Vertex(1188.2784423828125, 731.762451171875),
        new Vertex(1188.7783203125, 729.064453125),
        new Vertex(1189.1671142578125, 726.2596435546875),
        new Vertex(1189.44482421875, 723.347900390625),
        new Vertex(1189.6114501953125, 720.3294067382812),
        new Vertex(1189.6669921875, 717.2041015625),
        new Vertex(1189.5404052734375, 712.4995727539062),
        new Vertex(1189.16064453125, 708.058837890625),
        new Vertex(1188.8759765625, 705.9375),
        new Vertex(1188.52783203125, 703.8820190429688),
        new Vertex(1188.1165771484375, 701.8924560546875),
        new Vertex(1187.641845703125, 699.968994140625),
        new Vertex(1187.1038818359375, 698.1113891601562),
        new Vertex(1186.502685546875, 696.3197631835938),
        new Vertex(1185.8382568359375, 694.5941162109375),
        new Vertex(1185.1103515625, 692.9343872070312),
        new Vertex(1184.3193359375, 691.340576171875),
        new Vertex(1183.4649658203125, 689.8128051757812),
        new Vertex(1182.54736328125, 688.3510131835938),
        new Vertex(1181.56640625, 686.955078125),
        new Vertex(1181.05126953125, 686.2842407226562),
        new Vertex(1180.5184326171875, 685.634765625),
        new Vertex(1179.9678955078125, 685.0065307617188),
        new Vertex(1179.3997802734375, 684.399658203125),
        new Vertex(1178.81396484375, 683.81396484375),
        new Vertex(1178.2103271484375, 683.2496948242188),
        new Vertex(1177.5889892578125, 682.7066650390625),
        new Vertex(1176.949951171875, 682.184814453125),
        new Vertex(1176.2933349609375, 681.6844482421875),
        new Vertex(1175.6190185546875, 681.205322265625),
        new Vertex(1174.9268798828125, 680.7474365234375),
        new Vertex(1174.2171630859375, 680.3108520507812),
        new Vertex(1173.48974609375, 679.8956298828125),
        new Vertex(1172.7447509765625, 679.5016479492188),
        new Vertex(1171.201416015625, 678.777587890625),
        new Vertex(1169.58740234375, 678.1387329101562),
        new Vertex(1167.902587890625, 677.5850219726562),
        new Vertex(1166.14697265625, 677.1165771484375),
        new Vertex(1164.320556640625, 676.7332763671875),
        new Vertex(1162.423583984375, 676.43505859375),
        new Vertex(1160.45556640625, 676.2221069335938),
        new Vertex(1158.4169921875, 676.0943603515625),
        new Vertex(1156.3076171875, 676.0517578125),
        new Vertex(1153.4664306640625, 676.1297607421875),
        new Vertex(1150.7532958984375, 676.3637084960938),
        new Vertex(1149.445068359375, 676.5391235351562),
        new Vertex(1148.1685791015625, 676.7534790039062),
        new Vertex(1146.9241943359375, 677.0069580078125),
        new Vertex(1145.7119140625, 677.29931640625),
        new Vertex(1144.53173828125, 677.6307373046875),
        new Vertex(1143.3834228515625, 678.0010375976562),
        new Vertex(1142.267333984375, 678.410400390625),
        new Vertex(1141.18310546875, 678.8587646484375),
        new Vertex(1140.131103515625, 679.3460693359375),
        new Vertex(1139.1109619140625, 679.8724365234375),
        new Vertex(1138.1229248046875, 680.4376831054688),
        new Vertex(1137.1669921875, 681.0419921875),
        new Vertex(1136.2421875, 681.6821899414062),
        new Vertex(1135.347412109375, 682.355224609375),
        new Vertex(1134.48291015625, 683.0610961914062),
        new Vertex(1133.6485595703125, 683.7999267578125),
        new Vertex(1132.844482421875, 684.5714111328125),
        new Vertex(1132.0704345703125, 685.3758544921875),
        new Vertex(1131.3265380859375, 686.213134765625),
        new Vertex(1130.613037109375, 687.083251953125),
        new Vertex(1129.9295654296875, 687.9862670898438),
        new Vertex(1129.2762451171875, 688.9219970703125),
        new Vertex(1128.6531982421875, 689.890625),
        new Vertex(1128.0601806640625, 690.8921508789062),
        new Vertex(1127.4974365234375, 691.926513671875),
        new Vertex(1126.96484375, 692.99365234375),
        new Vertex(1126.46240234375, 694.09375),
        new Vertex(1125.990234375, 695.2265625),
        new Vertex(1125.133056640625, 697.5812377929688),
        new Vertex(1124.3902587890625, 700.0480346679688),
        new Vertex(1123.7615966796875, 702.626953125),
        new Vertex(1123.247314453125, 705.318115234375),
        new Vertex(1122.8472900390625, 708.121337890625),
        new Vertex(1122.5616455078125, 711.0368041992188),
        new Vertex(1122.39013671875, 714.0643310546875),
    ],
];

        Polygon actual = PolygonStroker.Stroke(input, width);

        AssertPolygonIsValid(actual);
        AssertStrokeCoversInputCenterline(input, actual, samplesPerSegment: 4);
        AssertImplicitSeamsAreClosed(input, actual, width);
        AssertCompoundCenterRemainsOutside(input, actual);
    }

    private static void AssertPolygonIsValid(Polygon polygon)
    {
        Assert.True(polygon.Count > 0);

        for (int contourIndex = 0; contourIndex < polygon.Count; contourIndex++)
        {
            Contour contour = polygon[contourIndex];
            Assert.True(contour.Count >= 3, $"Contour {contourIndex} must have at least 3 vertices.");

            int edgeCount = contour.Count;
            if (edgeCount > 1 && contour[0] == contour[^1])
            {
                edgeCount--;
            }

            Assert.True(edgeCount >= 3, $"Contour {contourIndex} must have at least 3 edge vertices.");
            for (int i = 0; i < edgeCount; i++)
            {
                Assert.NotEqual(contour[i], contour[(i + 1) % edgeCount]);
            }

            double signedArea2 = 0D;
            for (int i = 0; i < edgeCount; i++)
            {
                signedArea2 += Vertex.Cross(contour[i], contour[(i + 1) % edgeCount]);
            }

            Assert.True(Math.Abs(signedArea2) > 1E-10D, $"Contour {contourIndex} is degenerate.");

            for (int i = 0; i < edgeCount; i++)
            {
                Vertex a1 = contour[i];
                Vertex a2 = contour[(i + 1) % edgeCount];

                for (int j = i + 1; j < edgeCount; j++)
                {
                    if (j == i + 1 || (i == 0 && j == edgeCount - 1))
                    {
                        continue;
                    }

                    Vertex b1 = contour[j];
                    Vertex b2 = contour[(j + 1) % edgeCount];

                    int intersections = PolygonUtilities.FindIntersection(a1, a2, b1, b2, out Vertex pi0, out Vertex pi1);
                    Assert.True(intersections == 0, $"Contour {contourIndex} self-intersects between edges {i} and {j} at {pi0} / {pi1}.");
                }
            }
        }
    }

    private static double ComputeTotalAbsoluteArea(Polygon polygon)
    {
        double total = 0D;
        for (int i = 0; i < polygon.Count; i++)
        {
            total += Math.Abs(ComputeSignedArea(polygon[i]));
        }

        return total;
    }

    private static double ComputeSignedArea(Contour contour)
    {
        int count = contour.Count;
        if (count < 3)
        {
            return 0D;
        }

        double area = 0D;
        Vertex previous = contour[count - 1];
        for (int i = 0; i < count; i++)
        {
            Vertex current = contour[i];
            area += (previous.Y + current.Y) * (previous.X - current.X);
            previous = current;
        }

        return area * 0.5D;
    }

    private static void AssertStrokeCoversInputCenterline(Polygon input, Polygon stroked, int samplesPerSegment)
    {
        Assert.True(samplesPerSegment > 0);

        Box2 bounds = stroked.GetBoundingBox();
        double epsilon = Math.Max(bounds.Max.X - bounds.Min.X, bounds.Max.Y - bounds.Min.Y) * 1E-12D;

        for (int contourIndex = 0; contourIndex < input.Count; contourIndex++)
        {
            Contour contour = input[contourIndex];
            if (contour.Count < 2)
            {
                continue;
            }

            bool isClosed = contour.Count > 1 && contour[0] == contour[^1];
            int vertexCount = isClosed ? contour.Count - 1 : contour.Count;
            int segmentCount = isClosed ? vertexCount : vertexCount - 1;

            for (int segmentIndex = 0; segmentIndex < segmentCount; segmentIndex++)
            {
                Vertex start = contour[segmentIndex];
                Vertex end = contour[(segmentIndex + 1) % vertexCount];
                if (start == end)
                {
                    continue;
                }

                for (int sample = 1; sample <= samplesPerSegment; sample++)
                {
                    double t = sample / (double)(samplesPerSegment + 1);
                    Vertex point = new(
                        start.X + ((end.X - start.X) * t),
                        start.Y + ((end.Y - start.Y) * t));

                    Containment containment = ClassifyPoint(stroked, point, epsilon);
                    Assert.True(
                        containment != Containment.Outside,
                        $"Stroke misses input centerline at contour {contourIndex}, segment {segmentIndex}, t={t:F3}, point=({point.X:F6},{point.Y:F6}).");
                }
            }
        }
    }

    private static Contour CreateFoldBackPolyline()
    {
        Contour contour =
        [
            new Vertex(-40, -20),
            new Vertex(40, -20),
            new Vertex(40, 20),
            new Vertex(-20, 20),
            new Vertex(-20, -5),
            new Vertex(25, -5),
            new Vertex(25, 35),
            new Vertex(-35, 35),
            new Vertex(-35, 5),
            new Vertex(10, 5),
            new Vertex(10, 50),
            new Vertex(-50, 50)
        ];

        return contour;
    }

    private static Contour CreateFigureNinePolyline(int loopSegments)
    {
        Contour contour = new(loopSegments + 16);

        // Tail of the "9".
        contour.Add(new Vertex(0, -40));
        contour.Add(new Vertex(0, -34));
        contour.Add(new Vertex(0, -28));
        contour.Add(new Vertex(0, -22));
        contour.Add(new Vertex(0, -16));
        contour.Add(new Vertex(0, -10));

        // Closed loop section represented as an open polyline traversal.
        double centerX = 0D;
        double centerY = 10D;
        double radius = 22D;
        double startAngle = -Math.PI * 0.6;
        double endAngle = startAngle + (Math.PI * 2D * 0.95D);
        double angleStep = (endAngle - startAngle) / loopSegments;

        for (int i = 0; i <= loopSegments; i++)
        {
            double angle = startAngle + (i * angleStep);
            double x = centerX + (Math.Cos(angle) * radius);
            double y = centerY + (Math.Sin(angle) * radius);
            contour.Add(new Vertex(x, y));
        }

        return contour;
    }

    private static Polygon BuildCompoundGearPolygon(int toothCount)
    {
        if (toothCount < 3)
        {
            throw new ArgumentOutOfRangeException(nameof(toothCount), "Tooth count must be >= 3.");
        }

        Contour outer = BuildGearContour(toothCount, 120D, 104D, 0D, 0D, clockwise: false);
        Contour inner = BuildGearContour(toothCount, 62D, 50D, 0D, 0D, clockwise: true);
        return [outer, inner];
    }

    private static Contour BuildGearContour(
        int toothCount,
        double outerRadius,
        double innerRadius,
        double centerX,
        double centerY,
        bool clockwise)
    {
        int vertexCount = toothCount * 2;
        double angleStep = Math.PI / toothCount;
        Contour contour = new(vertexCount + 1);

        for (int i = 0; i < vertexCount; i++)
        {
            double radius = (i & 1) == 0 ? outerRadius : innerRadius;
            double angle = i * angleStep;
            contour.Add(new Vertex(
                centerX + (Math.Cos(angle) * radius),
                centerY + (Math.Sin(angle) * radius)));
        }

        contour.Add(contour[0]);
        bool shouldBeCounterClockwise = !clockwise;
        if (contour.IsCounterClockwise() != shouldBeCounterClockwise)
        {
            contour.Reverse();
        }

        return contour;
    }

    private static Contour CreateCompoundCircleGlyphPath(
        double centerX,
        double centerY,
        double outerRadius,
        double innerRadius,
        int outerVertices,
        int innerVertices)
    {
        if (outerVertices < 8 || innerVertices < 8)
        {
            throw new ArgumentOutOfRangeException(nameof(outerVertices), "Vertex counts must be >= 8.");
        }

        Contour contour = new(outerVertices + innerVertices + 8);

        // Outer loop (counter-clockwise), explicitly closed.
        for (int i = 0; i < outerVertices; i++)
        {
            double angle = (i * Math.PI * 2D) / outerVertices;
            contour.Add(new Vertex(
                centerX + (Math.Cos(angle) * outerRadius),
                centerY + (Math.Sin(angle) * outerRadius)));
        }

        contour.Add(contour[0]);

        // Bridge to the inner loop start point.
        Vertex innerStart = new(centerX + innerRadius, centerY);
        contour.Add(innerStart);

        // Inner loop (clockwise) to emulate a glyph hole inside one compound contour.
        for (int i = innerVertices - 1; i >= 0; i--)
        {
            double angle = (i * Math.PI * 2D) / innerVertices;
            contour.Add(new Vertex(
                centerX + (Math.Cos(angle) * innerRadius),
                centerY + (Math.Sin(angle) * innerRadius)));
        }

        contour.Add(innerStart);

        // Bridge back to outer start and close the whole compound contour.
        Vertex outerStart = contour[0];
        contour.Add(outerStart);
        if (contour[0] != contour[^1])
        {
            contour.Add(contour[0]);
        }

        return contour;
    }

    private static void AssertCoverageEquivalent(Polygon expected, Polygon actual, int sampleCountPerAxis)
    {
        Assert.True(sampleCountPerAxis > 0);

        Box2 expectedBounds = expected.GetBoundingBox();
        Box2 actualBounds = actual.GetBoundingBox();
        Box2 bounds = expectedBounds.Add(actualBounds);

        double width = bounds.Max.X - bounds.Min.X;
        double height = bounds.Max.Y - bounds.Min.Y;
        Assert.True(width > 0D);
        Assert.True(height > 0D);

        double epsilon = Math.Max(width, height) * 1E-10D;

        int mismatches = 0;
        List<Vertex> mismatchSamples = [];
        int maxLoggedMismatches = 8;

        for (int y = 0; y < sampleCountPerAxis; y++)
        {
            double py = bounds.Min.Y + (((y + 0.5D) / sampleCountPerAxis) * height);
            for (int x = 0; x < sampleCountPerAxis; x++)
            {
                double px = bounds.Min.X + (((x + 0.5D) / sampleCountPerAxis) * width);
                Vertex point = new(px, py);

                Containment expectedContainment = ClassifyPoint(expected, point, epsilon);
                Containment actualContainment = ClassifyPoint(actual, point, epsilon);

                if (expectedContainment == Containment.OnBoundary || actualContainment == Containment.OnBoundary)
                {
                    continue;
                }

                if (expectedContainment != actualContainment)
                {
                    mismatches++;
                    if (mismatchSamples.Count < maxLoggedMismatches)
                    {
                        mismatchSamples.Add(point);
                    }
                }
            }
        }

        string message =
            $"Coverage mismatch: {mismatches} sampled points differed. " +
            $"Example samples: {string.Join("; ", mismatchSamples.Select(static p => $"({p.X:F4},{p.Y:F4})"))}";
        Assert.True(mismatches == 0, message);
    }

    private static Containment ClassifyPoint(Polygon polygon, in Vertex point, double epsilon)
    {
        bool inside = false;
        for (int i = 0; i < polygon.Count; i++)
        {
            Containment contourContainment = ClassifyPointInContour(polygon[i], point, epsilon);
            if (contourContainment == Containment.OnBoundary)
            {
                return Containment.OnBoundary;
            }

            if (contourContainment == Containment.Inside)
            {
                inside = !inside;
            }
        }

        return inside ? Containment.Inside : Containment.Outside;
    }

    private static Containment ClassifyPointInContour(Contour contour, in Vertex point, double epsilon)
    {
        int count = contour.Count;
        if (count < 3)
        {
            return Containment.Outside;
        }

        int last = count;
        if (contour[0] == contour[^1])
        {
            last--;
        }

        if (last < 3)
        {
            return Containment.Outside;
        }

        bool inside = false;
        Vertex previous = contour[last - 1];
        for (int i = 0; i < last; i++)
        {
            Vertex current = contour[i];
            if (IsPointOnSegment(point, previous, current, epsilon))
            {
                return Containment.OnBoundary;
            }

            bool intersects = ((previous.Y > point.Y) != (current.Y > point.Y)) &&
                              (point.X < (((current.X - previous.X) * (point.Y - previous.Y)) / (current.Y - previous.Y)) + previous.X);

            if (intersects)
            {
                inside = !inside;
            }

            previous = current;
        }

        return inside ? Containment.Inside : Containment.Outside;
    }

    private static bool IsPointOnSegment(in Vertex point, in Vertex a, in Vertex b, double epsilon)
    {
        double abX = b.X - a.X;
        double abY = b.Y - a.Y;
        double apX = point.X - a.X;
        double apY = point.Y - a.Y;

        double cross = (abX * apY) - (abY * apX);
        if (Math.Abs(cross) > epsilon)
        {
            return false;
        }

        double dot = (apX * abX) + (apY * abY);
        if (dot < -epsilon)
        {
            return false;
        }

        double lengthSq = (abX * abX) + (abY * abY);
        return dot <= lengthSq + epsilon;
    }

    private static void AssertImplicitSeamsAreClosed(Polygon input, Polygon output, double strokeWidth)
    {
        Box2 bounds = output.GetBoundingBox();
        double epsilon = Math.Max(bounds.Max.X - bounds.Min.X, bounds.Max.Y - bounds.Min.Y) * 1E-12D;
        double closeThreshold = Math.Max(strokeWidth * 2D, 1E-3D);

        for (int contourIndex = 0; contourIndex < input.Count; contourIndex++)
        {
            Contour contour = input[contourIndex];
            if (contour.Count < 3 || contour[0] == contour[^1])
            {
                continue;
            }

            Vertex first = contour[0];
            Vertex last = contour[^1];
            double dx = first.X - last.X;
            double dy = first.Y - last.Y;
            if ((dx * dx) + (dy * dy) > closeThreshold * closeThreshold)
            {
                continue;
            }

            double seamLength = Math.Sqrt((dx * dx) + (dy * dy));
            if (seamLength <= 1E-12D)
            {
                continue;
            }

            double normalX = -dy / seamLength;
            double normalY = dx / seamLength;
            double lateral = strokeWidth * 0.35D;
            double[] lateralFactors = [-1D, 0D, 1D];

            for (int i = 1; i <= 4; i++)
            {
                double t = i / 5D;
                double baseX = (first.X * (1D - t)) + (last.X * t);
                double baseY = (first.Y * (1D - t)) + (last.Y * t);

                for (int j = 0; j < lateralFactors.Length; j++)
                {
                    double factor = lateralFactors[j];
                    Vertex sample = new(
                        baseX + (normalX * lateral * factor),
                        baseY + (normalY * lateral * factor));

                    Containment containment = ClassifyPoint(output, sample, epsilon);
                    Assert.True(
                        containment != Containment.Outside,
                        $"Implicit seam remained open at contour {contourIndex}, t={t:F2}, lateral={factor:F1}, point=({sample.X:F6},{sample.Y:F6}).");
                }
            }
        }
    }

    private static void AssertCompoundCenterRemainsOutside(Polygon input, Polygon output)
    {
        Box2 inputBounds = input.GetBoundingBox();
        Vertex center = new(
            (inputBounds.Min.X + inputBounds.Max.X) * 0.5D,
            (inputBounds.Min.Y + inputBounds.Max.Y) * 0.5D);

        Box2 outputBounds = output.GetBoundingBox();
        double epsilon = Math.Max(outputBounds.Max.X - outputBounds.Min.X, outputBounds.Max.Y - outputBounds.Min.Y) * 1E-12D;

        Containment evenOdd = ClassifyPoint(output, center, epsilon);
        Containment nonZero = ClassifyPointNonZero(output, center, epsilon);

        Assert.True(
            evenOdd != Containment.Inside,
            $"Compound center is filled under even-odd at ({center.X:F6},{center.Y:F6}).");
        Assert.True(
            nonZero != Containment.Inside,
            $"Compound center is filled under non-zero at ({center.X:F6},{center.Y:F6}).");
    }

    private static Containment ClassifyPointNonZero(Polygon polygon, in Vertex point, double epsilon)
    {
        int winding = 0;
        for (int i = 0; i < polygon.Count; i++)
        {
            winding += ComputeContourWindingContribution(polygon[i], point, epsilon, out bool onBoundary);
            if (onBoundary)
            {
                return Containment.OnBoundary;
            }
        }

        return winding == 0 ? Containment.Outside : Containment.Inside;
    }

    private static int ComputeContourWindingContribution(Contour contour, in Vertex point, double epsilon, out bool onBoundary)
    {
        onBoundary = false;

        int count = contour.Count;
        if (count < 3)
        {
            return 0;
        }

        int last = contour[0] == contour[^1] ? count - 1 : count;
        if (last < 3)
        {
            return 0;
        }

        int winding = 0;
        Vertex previous = contour[last - 1];
        for (int i = 0; i < last; i++)
        {
            Vertex current = contour[i];
            if (IsPointOnSegment(point, previous, current, epsilon))
            {
                onBoundary = true;
                return 0;
            }

            if (previous.Y <= point.Y)
            {
                if (current.Y > point.Y && IsLeft(previous, current, point) > epsilon)
                {
                    winding++;
                }
            }
            else if (current.Y <= point.Y && IsLeft(previous, current, point) < -epsilon)
            {
                winding--;
            }

            previous = current;
        }

        return winding;
    }

    private static double IsLeft(in Vertex a, in Vertex b, in Vertex p)
        => ((b.X - a.X) * (p.Y - a.Y)) - ((p.X - a.X) * (b.Y - a.Y));

    private enum Containment
    {
        Outside,
        Inside,
        OnBoundary
    }
}

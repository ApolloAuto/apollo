import argparse
from bokeh.plotting import figure, output_file, show
import common.proto_utils as proto_utils
from modules.map.proto import map_pb2


def draw(map_pb, plot):
    for lane in map_pb.lane:
        for curve in lane.left_boundary.curve.segment:
            if curve.HasField('line_segment'):
                x = []
                y = []
                for p in curve.line_segment.point:
                    x.append(p.x)
                    y.append(p.y)
                plot.line(x, y, line_width=2)
        for curve in lane.right_boundary.curve.segment:
            if curve.HasField('line_segment'):
                x = []
                y = []
                for p in curve.line_segment.point:
                    x.append(p.x)
                    y.append(p.y)
                plot.line(x, y, line_width=2)


def load_map_data(map_file):
    map_pb = map_pb2.Map()
    proto_utils.get_pb_from_file(map_file, map_pb)
    return map_pb


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="HDMapViewer is a tool to display hdmap.",
        prog="hdmapviewer.py")

    parser.add_argument(
        "-m", "--map", action="store", type=str, required=True,
        help="Specify the HDMap file in txt or binary format")

    args = parser.parse_args()
    map_pb = load_map_data(args.map)

    output_file("hdmap.html")
    plot = figure(sizing_mode='scale_both', match_aspect=True)

    draw(map_pb, plot)
    show(plot)

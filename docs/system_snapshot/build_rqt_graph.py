#!/usr/bin/env python3
r"""Build a Graphviz rqt_graph equivalent from `ros2 node info` dumps.

Reads every node_info/*.txt file produced by:
    for n in $(ros2 node list); do
        ros2 node info "$n" > node_info/${n//\//_}.txt
    done
and emits ros_graph.gv (Graphviz DOT) with:
  - one box per node
  - one ellipse per topic
  - one edge node→topic for each publisher
  - one edge topic→node for each subscriber
Filters out the noise topics (/parameter_events, /rosout, parameter services)
to match what rqt_graph's "hide debug" defaults look like.

Render with:
    dot -Tpng ros_graph.gv -o ros_graph.png
    dot -Tpdf ros_graph.gv -o ros_graph.pdf
"""

import os
import re
import sys

NODE_INFO_DIR = os.path.join(os.path.dirname(__file__), 'node_info')
OUT_GV = os.path.join(os.path.dirname(__file__), 'ros_graph.gv')

NOISE_TOPIC_RE = re.compile(
    r'^(/parameter_events|/rosout|.*/get_parameter|.*/set_parameter|'
    r'.*/list_parameters|.*/describe_parameters|.*/get_type_description|'
    r'.*/transition_event)$'
)
NOISE_NODE_RE = re.compile(
    r'^/(transform_listener_impl_.*|.*_private_.*|moveit_\d+|rqt_gui_cpp_node_\d+)$'
)


def parse_node_info(path):
    """Return (node_name, {'pub': set(topics), 'sub': set(topics)})."""
    pubs, subs = set(), set()
    section = None
    name = None
    with open(path) as f:
        for raw in f:
            line = raw.rstrip()
            if not line:
                continue
            if line.startswith('/') and not line.startswith('  '):
                name = line.strip()
                continue
            stripped = line.strip()
            if stripped == 'Subscribers:':
                section = 'sub'
                continue
            if stripped == 'Publishers:':
                section = 'pub'
                continue
            if stripped.endswith(':') and not line.startswith('    '):
                section = None
                continue
            if section is None:
                continue
            m = re.match(r'\s+([^:]+):\s', line)
            if not m:
                continue
            topic = m.group(1).strip()
            if NOISE_TOPIC_RE.match(topic):
                continue
            (pubs if section == 'pub' else subs).add(topic)
    return name, {'pub': pubs, 'sub': subs}


def main():
    if not os.path.isdir(NODE_INFO_DIR):
        sys.stderr.write(f'no {NODE_INFO_DIR}\n')
        sys.exit(1)
    nodes = {}
    for fn in sorted(os.listdir(NODE_INFO_DIR)):
        if not fn.endswith('.txt'):
            continue
        name, io = parse_node_info(os.path.join(NODE_INFO_DIR, fn))
        if name is None or NOISE_NODE_RE.match(name):
            continue
        nodes[name] = io

    topics = set()
    for io in nodes.values():
        topics.update(io['pub'])
        topics.update(io['sub'])

    with open(OUT_GV, 'w') as out:
        out.write('digraph rosgraph {\n')
        out.write('  rankdir=LR;\n')
        out.write('  node [fontname="Helvetica" fontsize=10];\n')
        out.write('  edge [fontname="Helvetica" fontsize=8];\n\n')
        out.write('  // === nodes ===\n')
        for name in sorted(nodes):
            label = name.replace('"', '\\"')
            out.write(f'  "{name}" [shape=box style="rounded,filled" '
                      f'fillcolor="#cfe8ff" label="{label}"];\n')
        out.write('\n  // === topics ===\n')
        for t in sorted(topics):
            label = t.replace('"', '\\"')
            out.write(f'  "{t}" [shape=ellipse style=filled '
                      f'fillcolor="#ffe6cc" label="{label}"];\n')
        out.write('\n  // === edges ===\n')
        for name, io in sorted(nodes.items()):
            for t in sorted(io['pub']):
                out.write(f'  "{name}" -> "{t}";\n')
            for t in sorted(io['sub']):
                out.write(f'  "{t}" -> "{name}";\n')
        out.write('}\n')
    print(f'wrote {OUT_GV} '
          f'({len(nodes)} nodes, {len(topics)} topics)')


if __name__ == '__main__':
    main()

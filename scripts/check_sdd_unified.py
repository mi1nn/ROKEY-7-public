#!/usr/bin/env python3
"""Validate the editable 00 proposal against JSON/CSV and byte-preserved historical pages.

This is E1 artifact verification, not ROS/runtime or physical-safety validation.
"""
import argparse
import collections
import copy
import csv
import json
import math
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
DESIGN = ROOT / 'docs/design'
CHUNKS = re.compile(rb'<diagram\b.*?</diagram>', re.S)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def normalized_root(page):
    """draw.io page duplication changes cell IDs but preserves SDD identities."""
    root = copy.deepcopy(page.find('./mxGraphModel/root'))
    aliases = {root[0].get('id'): '0', root[1].get('id'): '1'}
    queries = {'read_mission_state': 'mission', 'read_robot_status': 'robot',
               'read_target_states': 'targets', 'read_reports': 'reports', 'read_site_events': 'events'}
    for obj in root.findall('object'):
        for attr, prefix in [('sddNodeId', 'node-'), ('sddInterfaceId', 'if-'),
                             ('webPanel', 'panel-'), ('hostId', 'host-')]:
            if obj.get(attr):
                aliases[obj.get('id')] = prefix + obj.get(attr)
        if obj.get('queryFunction'):
            aliases[obj.get('id')] = 'query-' + queries[obj.get('queryFunction')]
    require(len(set(aliases.values())) == len(aliases), 'duplicate semantic cell ID')
    for entry in root:
        entry.set('id', aliases.get(entry.get('id'), entry.get('id')))
        cell = entry if entry.tag == 'mxCell' else entry.find('mxCell')
        for attr in ('parent', 'source', 'target'):
            if cell.get(attr) in aliases:
                cell.set(attr, aliases[cell.get(attr)])
    return root


def check_graph(data, xml, page_index=0):
    pages = ET.fromstring(xml)
    variant = data['metadata']['placement_variants']['three_pc']
    require(len(pages) == 13 and pages[0].get('name') == '00 통합 시스템 설계도'
            and pages[1].get('name') == variant['page_name'], 'page count/name')
    root = normalized_root(pages[page_index])
    objects = root.findall('object')
    actual_nodes = [o.get('sddNodeId') for o in objects if o.get('sddNodeId')]
    expected_nodes = {n['id'] for n in data['nodes']}
    require(set(actual_nodes) == expected_nodes and len(actual_nodes) == len(expected_nodes), 'logical node coverage')
    interfaces = {i['id']: i for i in data['interfaces']}
    actual_if = [o.get('sddInterfaceId') for o in objects if o.get('sddInterfaceId')]
    require(set(actual_if) == set(interfaces) and len(actual_if) == len(interfaces), 'interface coverage')
    require('IF-040' not in interfaces and {i['id'] for i in data['retired_interfaces']} == {'IF-040'}, 'retired IF-040')
    edges = [(o, o.find('mxCell')) for o in objects if o.find('mxCell').get('edge') == '1']
    adjacency = collections.defaultdict(set)
    for o, cell in edges:
        adjacency[cell.get('source')].add(cell.get('target'))
    for iid, spec in interfaces.items():
        require(set(spec['sources'] + spec['targets']) <= expected_nodes, iid + ' unknown node')
        selected = [(o, c) for o, c in edges if o.get('interfaceRef') == iid]
        if iid == 'IF-081':
            actual = {(c.get('source')[5:], c.get('target')[5:]) for o, c in selected}
        else:
            ins = {c.get('source')[5:] for o, c in selected if o.get('direction') == 'IN'}
            outs = {c.get('target')[5:] for o, c in selected if o.get('direction') == 'OUT'}
            actual = {(a, b) for a in ins for b in outs}
            port = next(o for o in objects if o.get('sddInterfaceId') == iid)
            require(port.get('endpoint') == spec['endpoint'] and port.get('interfaceType') == spec['type'], iid + ' label metadata')
        require(actual == {(a, b) for a in spec['sources'] for b in spec['targets']}, iid + ' source/target coverage')
    paths = [
        ['node-central.coordinator', 'if-IF-071', 'node-central.operations', 'if-IF-085', 'node-central.db', 'query-mission', 'node-central.web', 'panel-disaster'],
        ['node-central.web', 'panel-refresh'],
        ['node-cam.capture', 'if-IF-074', 'node-cam.video', 'if-IF-082', 'panel-webcam'],
        ['node-amr1.mapping', 'if-IF-010', 'node-central.operations', 'if-IF-085', 'node-central.db', 'query-robot', 'node-central.web', 'panel-map'],
        ['node-amr1.executor', 'if-IF-011', 'node-central.operations', 'if-IF-085', 'node-central.db', 'query-robot', 'node-central.web', 'panel-robots'],
        ['node-central.coordinator', 'if-IF-070', 'node-central.operations', 'if-IF-085', 'node-central.db', 'query-targets', 'node-central.web', 'panel-targets'],
        ['node-cam.quake', 'if-IF-076', 'node-central.coordinator', 'if-IF-072', 'node-central.operations', 'if-IF-085', 'node-central.db', 'query-reports', 'node-central.web', 'panel-events'],
        ['panel-commands', 'node-central.web', 'if-IF-080', 'node-central.operations', 'if-IF-079', 'node-central.coordinator'],
    ]
    for path in paths:
        for a, b in zip(path, path[1:]):
            require(b in adjacency[a], 'web provenance: ' + a + ' -> ' + b)
    require(len([o for o in objects if o.get('webPanel')]) == 8, 'web panels')
    hosts = {o.get('hostId'): o.get('id') for o in objects if o.get('hostId')}
    expected_hosts = {'BROWSER', 'PC1', 'PC2', 'PC3', 'PC4', 'AMR1', 'AMR2'}
    require(set(hosts) == expected_hosts, 'host swimlanes')
    entries = {e.get('id'): e for e in root}

    def physical_host(iid):
        seen = set()
        while iid not in ('0', '1'):
            require(iid not in seen, 'cyclic container parent')
            seen.add(iid)
            entry = entries[iid]
            if entry.get('hostId') in expected_hosts - {'BROWSER'}:
                return entry.get('hostId')
            cell = entry if entry.tag == 'mxCell' else entry.find('mxCell')
            iid = cell.get('parent')
        return None

    overrides = variant['node_host_overrides'] if page_index else {}
    for node in data['nodes']:
        require(physical_host('node-' + node['id']) == overrides.get(node['id'], node['host']),
                'node placement: ' + node['id'])
    if page_index:
        require(variant['physical_pc_count'] == 4 and pages[1].get('physicalPCCount') == '4', 'physical PC count')
        require(variant['application_pc_count'] == 3 and variant['dedicated_discovery_pc_count'] == 1
                and pages[1].get('applicationPCCount') == '3' and pages[1].get('discoveryPCCount') == '1', 'application/discovery PC count')
        require(variant['discovery_host'] == 'PC4' and physical_host('node-central.discovery') == 'PC4', 'dedicated discovery host')
        require(variant['browser_host'] == 'PC3' and physical_host(hosts['BROWSER']) == 'PC3', 'local browser host')
    # The existing ExecuteTask/ObserveTarget wires are the mandatory recheck path.
    for a, b in [('node-central.coordinator', 'if-IF-021'), ('if-IF-021', 'node-amr1.executor'),
                 ('node-amr1.executor', 'if-IF-023'), ('if-IF-023', 'node-amr1.perception')]:
        require(b in adjacency[a], 'mandatory AMR1 recheck wire')
    require(data['metadata']['completion_requirement']['mode'] == 'EXIT_CANDIDATE_AND_FRESH_AMR1_FIELD_RECHECK', 'latest PM completion requirement')
    require(not data['metadata']['completion_requirement']['candidate_is_completion'], 'candidate must not equal completion')
    return root, edges


def check_geometry(root, edges):
    cells = {}
    for entry in root:
        cell = entry if entry.tag == 'mxCell' else entry.find('mxCell')
        if cell is not None:
            cells[entry.get('id')] = cell
    rectangles = {}

    def rectangle(iid):
        if iid in rectangles:
            return rectangles[iid]
        cell = cells[iid]
        geo = cell.find('mxGeometry')
        x, y = float(geo.get('x', 0)), float(geo.get('y', 0))
        parent = cell.get('parent')
        if parent not in (None, '0', '1'):
            px, py, _, _ = rectangle(parent)
            x, y = x + px, y + py
        rectangles[iid] = (x, y, float(geo.get('width', 0)), float(geo.get('height', 0)))
        return rectangles[iid]

    obstacle_ids = [iid for iid, c in cells.items() if c.get('vertex') == '1' and iid.startswith(('node-', 'if-'))]
    spans = collections.defaultdict(list)
    for obj, cell in edges:
        style = dict(s.split('=', 1) for s in cell.get('style').split(';') if '=' in s)
        require(style.get('jumpStyle') == 'arc', 'crossing jump style')
        a, b = cell.get('source'), cell.get('target')
        ax, ay, aw, ah = rectangle(a)
        bx, by, bw, bh = rectangle(b)
        points = [(ax + aw * float(style['exitX']), ay + ah * float(style['exitY']))]
        points += [(float(p.get('x')), float(p.get('y'))) for p in cell.findall('./mxGeometry/Array/mxPoint')]
        points.append((bx + bw * float(style['entryX']), by + bh * float(style['entryY'])))
        for start, end in zip(points, points[1:]):
            if math.dist(start, end) < 1e-6:
                continue
            horizontal = math.isclose(start[1], end[1], abs_tol=1e-6)
            require(horizontal or math.isclose(start[0], end[0], abs_tol=1e-6), 'diagonal line')
            key = ('h', round(start[1], 6)) if horizontal else ('v', round(start[0], 6))
            lo, hi = sorted((start[0], end[0]) if horizontal else (start[1], end[1]))
            spans[key].append((lo, hi, obj.get('id'), obj.get('interfaceRef') or obj.get('id')))
            for obstacle in obstacle_ids:
                if obstacle in (a, b):
                    continue
                x, y, w, h = rectangle(obstacle)
                hit = (y + 1e-6 < start[1] < y + h - 1e-6 and min(start[0], end[0]) < x + w and max(start[0], end[0]) > x) if horizontal else (x + 1e-6 < start[0] < x + w - 1e-6 and min(start[1], end[1]) < y + h and max(start[1], end[1]) > y)
                require(not hit, 'line crosses node/label: ' + obstacle)
    for key, values in spans.items():
        for index, (lo, hi, edge, group) in enumerate(values):
            for ll, hh, other, other_group in values[index + 1:]:
                require(edge == other or group == other_group or min(hi, hh) - max(lo, ll) <= 1e-6, 'unrelated collinear overlap: ' + edge + '/' + other)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', help='Git ref for exact preservation of historical 01-11 pages')
    parser.add_argument('--self-test', action='store_true', help='also reject intentionally missing nodes/IFs/links and wrong completion mode')
    args = parser.parse_args()
    data = json.loads((DESIGN / 'sdd-node-contracts-v0.1.json').read_text())
    xml = (DESIGN / 'rokey7-sdd.drawio').read_bytes()
    for page_index in (0, 1):
        root, edges = check_graph(data, xml, page_index)
        check_geometry(root, edges)
    expected = {(i['id'], a, b, i['endpoint'], i['type']) for i in data['interfaces'] for a in i['sources'] for b in i['targets']}
    rows = list(csv.DictReader((DESIGN / 'sdd-node-connections-v0.1.csv').open(encoding='utf-8-sig')))
    got = {(r['interface_id'], r['source_node_id'], r['target_node_id'], r['endpoint'], r['type']) for r in rows}
    require(got == expected and len(rows) == len(expected), 'CSV exact JSON source/target/type relation')
    md = (DESIGN / 'sdd-node-interface-spec-v0.1.md').read_text()
    require(set(re.findall(r'^\| (IF-\d{3}) \|', md, re.M)) == {i['id'] for i in data['interfaces']}, 'Markdown IF coverage')
    for node in data['nodes']:
        require('| ' + node['id'] + ' | ' + node['host'] + ' | ' + node['name'] + ' |' in md, 'Markdown node metadata')
    base = args.base or data['metadata']['repository_head']
    original = subprocess.check_output(['git', 'show', base + ':docs/design/rokey7-sdd.drawio'], cwd=ROOT)
    require(CHUNKS.findall(xml)[2:] == CHUNKS.findall(original)[1:], '01-11 not byte-identical to base')
    if args.self_test:
        # These mutations exercise coverage and latest-decision checks, not runtime behavior.
        for mutation in ('node', 'interface', 'edge', 'completion', 'placement', 'browser', 'discovery', 'pc_count'):
            tree = ET.fromstring(xml)
            page_index = 1 if mutation in ('placement', 'browser', 'discovery', 'pc_count') else 0
            entry_root = tree[page_index].find('./mxGraphModel/root')
            changed = copy.deepcopy(data)
            if mutation == 'completion':
                changed['metadata']['completion_requirement']['mode'] = 'NO_RECHECK'
            elif mutation == 'pc_count':
                tree[1].set('physicalPCCount', '3')
            elif mutation == 'discovery':
                pc3 = next(o for o in entry_root.findall('object') if o.get('hostId') == 'PC3').get('id')
                next(o for o in entry_root.findall('object') if o.get('sddNodeId') == 'central.discovery').find('mxCell').set('parent', pc3)
            elif mutation in ('placement', 'browser'):
                key = 'sddNodeId' if mutation == 'placement' else 'hostId'
                value = 'central.operations' if mutation == 'placement' else 'BROWSER'
                next(o for o in entry_root.findall('object') if o.get(key) == value).find('mxCell').set('parent', entry_root[1].get('id'))
            else:
                key = {'node': 'sddNodeId', 'interface': 'sddInterfaceId', 'edge': 'interfaceRef'}[mutation]
                entry_root.remove(next(o for o in entry_root.findall('object') if o.get(key)))
            try:
                check_graph(changed, ET.tostring(tree), page_index)
            except ValueError:
                pass
            else:
                raise ValueError('negative case accepted: ' + mutation)
        print('PASS self-test: missing node/interface/edge, wrong completion/count/PC3 placement and misplaced Discovery rejected')
    print(f'PASS XML/JSON/CSV/Markdown; pages=13; nodes={len(data["nodes"])}; active_IF={len(data["interfaces"])}; pairs={len(expected)}; web=8/page; hosts=7/7; physical_PC=4; application_PC=3; dedicated_discovery_PC=1; original_pages=11; unrelated_overlap=0')


if __name__ == '__main__':
    main()

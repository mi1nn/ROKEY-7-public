"""rescue_interfaces IDL == SDD v0.1 JSON (docs/design/sdd-node-contracts-v0.1.json), without ROS.

The JSON is the single source. Each phase-1 .msg/.srv/.action must list the same
fields in the same order with the mapped ROS type, plus one string constant per
enum value (`<FIELD>_<VALUE>`). Building the package (rosidl) is a separate E3 check.
"""
import json
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
JSON_PATH = ROOT / 'docs' / 'design' / 'sdd-node-contracts-v0.1.json'
PKG = ROOT / 'src' / 'rescue_interfaces'

# Phase 1 adopted 2026-10-10 (contracts.md C-10). RecheckObservation/PassageAssessment
# are pulled in because TaskResult/RouteCandidate/ObserveTarget embed them.
MESSAGES = ('Envelope', 'ObservationTime', 'FrameRef', 'EvidenceRef', 'RobotStatus', 'RobotPose', 'Heartbeat',
            'CommandLease', 'TaskAdmission', 'TaskResult', 'OperatorApproval', 'RouteCandidate', 'SiteEvent',
            'TargetObservation', 'TargetLocation', 'MapState', 'ReportReceipt', 'RecheckObservation',
            'PassageAssessment')
SERVICES = ('RequestStop', 'RegisterAuthority', 'ResetStop', 'SubmitOperatorCommand')
ACTIONS = ('ExecuteTask', 'ObserveTarget')
SECTIONS = {'msg': (None,), 'srv': ('Request', 'Response'), 'action': ('Goal', 'Result', 'Feedback')}
ENUM = re.compile(r'[A-Z][A-Z0-9_]*(/[A-Z][A-Z0-9_]*)*')


def spec_fields(text):
    """'name:type(A/B), ... . note' -> [(name, type, enum values)]."""
    out = []
    for part in re.split(r',\s+', text.strip()):
        m = re.fullmatch(r'([a-z][a-z0-9_]*):\s*([^.(]+?)\s*(?:\(([^)]*)\))?\s*(\..*)?', part, re.S)
        if not m:
            raise ValueError('unparsed field: ' + part)
        name, kind, paren, tail = m.groups()
        values = tuple(paren.split('/')) if paren and ENUM.fullmatch(paren) else ()
        out.append((name, ros_type(kind), values))
        if tail:
            break
    return out


def ros_type(kind):
    array = '[]' if kind.endswith('[]') else ''
    base = kind[:-2] if array else kind
    if base in ('UUID', 'UUID string'):
        base = 'string'
    return base + array


def idl_sections(path):
    """IDL text -> [([(type, name)], {constant: value})] per '---' section."""
    sections = [([], {})]
    for line in path.read_text(encoding='utf-8').splitlines():
        line = line.split('#', 1)[0].strip()
        if not line:
            continue
        if line == '---':
            sections.append(([], {}))
            continue
        kind, rest = line.split(None, 1)
        if '=' in rest:
            name, value = (s.strip() for s in rest.split('=', 1))
            sections[-1][1][name] = value.strip('"')
        else:
            sections[-1][0].append((kind, rest))
    return sections


def expected(text):
    fields = spec_fields(text)
    constants = {f'{name.upper()}_{v}': v for name, _, values in fields for v in values}
    return [(kind, name) for name, kind, _ in fields], constants


@unittest.skipUnless((ROOT / 'src').is_dir(), 'source checkout absent (installed-wheel run): IDL check NOT_RUN here')
class InterfaceSchemaTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.spec = json.loads(JSON_PATH.read_text(encoding='utf-8'))

    def check(self, folder, name, parts):
        path = PKG / folder / f'{name}.{folder}'
        self.assertTrue(path.is_file(), path)
        got = idl_sections(path)
        self.assertEqual(len(got), len(parts), path)
        for (fields, constants), part in zip(got, parts):
            want_fields, want_constants = expected(part)
            self.assertEqual(fields, want_fields, path)
            self.assertEqual(constants, want_constants, path)

    def test_messages(self):
        for name in MESSAGES:
            self.check('msg', name, [self.spec['message_schemas'][name]])

    def test_services_and_actions(self):
        for folder, names, key in (('srv', SERVICES, 'services'), ('action', ACTIONS, 'actions')):
            for name in names:
                self.check(folder, name, [self.spec[key][name][s] for s in SECTIONS[folder]])

    def test_only_phase1_files_and_types_resolve(self):
        files = {p.relative_to(PKG).as_posix() for d in SECTIONS for p in (PKG / d).glob('*.' + d)}
        want = ({f'msg/{n}.msg' for n in MESSAGES} | {f'srv/{n}.srv' for n in SERVICES}
                | {f'action/{n}.action' for n in ACTIONS})
        self.assertEqual(files, want)
        primitive = {'bool', 'string', 'int64', 'uint64', 'uint32', 'float64'}
        for rel in files:
            for fields, _ in idl_sections(PKG / rel):
                for kind, _ in fields:
                    base = re.sub(r'\[\d*\]$', '', kind)
                    self.assertTrue(base in primitive or '/' in base or base in MESSAGES, (rel, kind))
        cmake = (PKG / 'CMakeLists.txt').read_text(encoding='utf-8')
        for rel in files:
            self.assertIn(f'"{rel}"', cmake)


if __name__ == '__main__':
    unittest.main()

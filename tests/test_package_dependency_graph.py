"""colcon orders the workspace by package.xml dependencies; a cycle makes the
whole build fail before compiling (#100 BUILD-01). Checks the repository's own
packages only, with the standard library."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

SRC = Path(__file__).resolve().parents[1] / 'src'
DEPEND_TAGS = ('depend', 'build_depend', 'buildtool_depend', 'exec_depend', 'build_export_depend',
               'test_depend')


def repository_graph():
    manifests = {}
    for path in sorted(SRC.glob('*/package.xml')):
        root = ET.parse(path).getroot()
        manifests[root.findtext('name')] = root
    return {name: sorted({e.text.strip() for tag in DEPEND_TAGS for e in root.iter(tag)} & manifests.keys())
            for name, root in manifests.items()}


def find_cycle(graph):
    state, stack = {}, []

    def visit(name):
        state[name] = 'open'
        stack.append(name)
        for dep in graph[name]:
            if state.get(dep) == 'open':
                return stack[stack.index(dep):] + [dep]
            if dep not in state:
                cycle = visit(dep)
                if cycle:
                    return cycle
        stack.pop()
        state[name] = 'done'
        return None

    for name in graph:
        if name not in state:
            cycle = visit(name)
            if cycle:
                return cycle
    return None


class PackageDependencyGraphTests(unittest.TestCase):
    def test_repository_packages_have_no_dependency_cycle(self):
        graph = repository_graph()
        self.assertIn('rescue_coordination', graph)  # the scan found the packages
        self.assertIsNone(find_cycle(graph))

    def test_cycle_finder_reports_a_cycle(self):
        self.assertEqual(['a', 'b', 'a'], find_cycle({'a': ['b'], 'b': ['a']}))
        self.assertIsNone(find_cycle({'a': ['b'], 'b': []}))


if __name__ == '__main__':
    unittest.main()

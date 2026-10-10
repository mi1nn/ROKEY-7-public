"""Profile v2 mission areas -> coordination MissionSite, and shared vocabularies (OPS-01)."""
from dataclasses import replace
import math
from pathlib import Path
import unittest

import rescue_contracts
from rescue_bringup.profile import load_deployment
from rescue_coordination import planning, search
from rescue_coordination.planning import CoordinationError
from rescue_coordination.search import MissionSite

EXAMPLE = Path(__file__).resolve().parents[1] / 'config' / 'deployment.v2.example.json'


def example_areas():
    return load_deployment(EXAMPLE).mission.search_areas


def yaws(areas, value=0.5):
    return {area.id: value for area in areas}


class SharedVocabularyTests(unittest.TestCase):
    def test_task_kinds_come_from_contracts(self):
        self.assertIs(planning.TASK_KINDS, rescue_contracts.TASK_KINDS)
        self.assertIs(planning.TARGET_TASK_KINDS, rescue_contracts.TARGET_TASK_KINDS)

    def test_roles_come_from_contracts(self):
        self.assertIs(search.ROLES, rescue_contracts.ROBOT_ROLES)


class MissionSiteLoaderTests(unittest.TestCase):
    def test_profile_areas_become_sites_in_order(self):
        areas = example_areas()
        sites = search.mission_sites(areas, entry_yaws=yaws(areas))
        self.assertEqual([a.id for a in areas], [s.site_id for s in sites])
        for area, site in zip(areas, sites):
            self.assertIsInstance(site, MissionSite)
            self.assertEqual(area.mission_xy, site.mission_xy)
            self.assertEqual(area.basis.kind, site.basis)
            self.assertEqual((area.basis.map_session_id, area.basis.map_revision,
                              area.basis.transform_revision),
                             (site.map_session_id, site.map_revision, site.transform_revision))
            self.assertEqual(0.5, site.entry_yaw)
            self.assertTrue(site.bound_to_map())

    def test_entry_yaw_is_per_area(self):
        areas = example_areas()
        given = {areas[0].id: -1.0, areas[1].id: 2.0}
        sites = search.mission_sites(areas, entry_yaws=given)
        self.assertEqual([-1.0, 2.0], [s.entry_yaw for s in sites])

    def test_missing_extra_or_nonfinite_entry_yaw_rejected(self):
        areas = example_areas()
        for given in ({areas[0].id: 0.0},
                      yaws(areas) | {'11111111-1111-4111-8111-1111111111ff': 0.0},
                      yaws(areas, math.nan)):
            with self.subTest(given=given), self.assertRaises(CoordinationError) as caught:
                search.mission_sites(areas, entry_yaws=given)
            self.assertEqual('INVALID_INPUT', caught.exception.code)

    def test_duplicate_or_empty_areas_rejected(self):
        areas = example_areas()
        for given in ((), (areas[0], areas[0])):
            with self.subTest(n=len(given)), self.assertRaises(CoordinationError):
                search.mission_sites(given, entry_yaws=yaws(given))

    def test_basis_kind_is_kept_not_judged(self):
        # Executability is SearchPolicy.executable_bases' call, never the loader's.
        areas = example_areas()
        prior = search.mission_sites(areas, entry_yaws=yaws(areas))
        self.assertEqual({'PRIOR_MAP'}, {s.basis for s in prior})
        bound = replace(areas[0], basis=replace(areas[0].basis, kind='MAP_REVISION_BOUND'))
        self.assertEqual('MAP_REVISION_BOUND',
                         search.mission_sites((bound,), entry_yaws=yaws((bound,)))[0].basis)


if __name__ == '__main__':
    unittest.main()

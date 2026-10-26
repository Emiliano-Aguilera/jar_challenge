"""Compare saved coverage evaluations without confusing prediction with observation."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path


def seen(route, seconds, target_set):
    return max((event['visible'][target_set] for event in route['curve']
                if event['elapsed'] <= seconds + 1e-9), default=0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--before', type=Path, required=True)
    parser.add_argument('--after', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    if not args.output.resolve().is_relative_to(root / 'build'):
        parser.error('comparison output must stay under build/')
    before, after = (json.loads(path.read_text()) for path in (args.before, args.after))
    assert before['camera'] == after['camera']
    assert before['mission_parameters'] == after['mission_parameters']
    order_parameters = {'order_multiview', 'order_baseline_m', 'order_block_candidates'}
    assert all(after['settings'][key] == value for key, value in before['settings'].items()
               if key not in order_parameters)
    old = {row['case']: row for row in before['cases']}
    assert set(old) == {row['case'] for row in after['cases']}
    interval = after['settings']['order_check_interval_s']
    rows = []
    for row in after['cases']:
        reference = old[row['case']]
        assert row['status'] == reference['status'] == 'accepted'
        assert row['start'] == reference['start']
        assert row['counts'] == reference['counts']
        poses = lambda case: Counter((p['x'], p['y'], p['yaw']) for p in case['poses'])
        assert poses(row) == poses(reference), row['case']
        assert row['mission_skipped'] == reference['mission_skipped'] == []
        assert row['mission_lost_dense'] == row['mission_lost_sampled'] == 0
        for route in ('intended', 'mission'):
            assert row[route]['final_visible'] == reference[route]['final_visible']
            assert row[route]['duration'] <= reference[route]['duration'] + 1e-9
            for check in range(1, math.ceil(reference[route]['duration'] / interval) + 1):
                seconds = check * interval
                for target_set in row[route]['final_visible']:
                    assert seen(row[route], seconds, target_set) >= seen(reference[route], seconds, target_set), (
                        row['case'], route, seconds, target_set)
            for seconds, snapshot in reference[route]['budgets'].items():
                for target_set, values in snapshot.items():
                    assert row[route]['budgets'][seconds][target_set]['visible'] >= values['visible'], (
                        row['case'], route, seconds, target_set)
        rows.append(dict(case=row['case'], before_s=reference['intended']['duration'],
                         after_s=row['intended']['duration'],
                         saved_s=reference['intended']['duration'] - row['intended']['duration'],
                         dense_300_gain=(row['intended']['budgets']['300.0']['dense_all']['visible']
                                         - reference['intended']['budgets']['300.0']['dense_all']['visible']),
                         planning_sample_s=row['timings']['planning']))
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'comparison.json').write_text(json.dumps(dict(cases=rows,
        limits='Grid predictions; no detector confirmation, collision or score guarantee.'), indent=2)+'\n')
    print(f'{len(rows)} cases preserve all views/final visibility, mission identity and every predicted clock check.')
    print(f"{sum(row['saved_s'] > 1e-9 for row in rows)} shorter; maximum saving {max(row['saved_s'] for row in rows):.2f} s.")


if __name__ == '__main__':
    main()

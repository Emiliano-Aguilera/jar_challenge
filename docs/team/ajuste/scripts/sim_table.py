"""One line per simulator run, from runs downloaded with notes/AlsinaBautista/bajar_corrida.sh.

Usage, from the repo root:  python3 docs/team/ajuste/scripts/sim_table.py 41937524342=A 41937530493=B
Each argument is RUN_ID=label; the runs are read from build/sim/RUN_ID/.
Counts are log lines in equipo.log (MPPI failures, "Collision Ahead", recoveries, trips cancelled).
"""
import collections
import glob
import json
import os
import sys

PATTERNS = {'mppi_fail': 'Optimizer fail', 'coll_ahead': 'Collision Ahead', 'spin': 'Running spin',
            'backup': 'Running backup', 'cancel': 'Cancelo el viaje', 'no_progress': 'Failed to make progress',
            'start_lethal': 'Starting point in lethal'}
COLUMNS = ['map', 'label', 'rep', 'rtf', 'score', 'victims', 'last_report_s', 'end', 'end_s', 'fast_bonus',
           'return', 'collisions', 'close_calls'] + list(PATTERNS)
rows = []
for arg in sys.argv[1:]:
    run_id, label = arg.split('=', 1)
    for d in glob.glob(f'build/sim/{run_id}/lote-*/corridas/*'):
        if not os.path.exists(f'{d}/resultado.json'):
            continue
        meta, res = json.load(open(f'{d}/meta.json')), json.load(open(f'{d}/resultado.json'))
        hits = json.load(open(f'{d}/choques.json')) if os.path.exists(f'{d}/choques.json') else {}
        score = res.get('puntaje', {})
        count = collections.Counter()
        for line in open(f'{d}/equipo.log', errors='replace') if os.path.exists(f'{d}/equipo.log') else []:
            for key, text in PATTERNS.items():
                count[key] += text in line
        victims = score.get('por_victima', [])
        rows.append({'map': meta.get('mapa'), 'label': label, 'rep': meta.get('repeticion'), 'rtf': meta.get('rtf'),
                     'score': score.get('total'), 'victims': f"{sum(v['puntos'] > 0 for v in victims)}/{len(victims)}",
                     'last_report_s': res.get('tiempo_ultimo_reporte_que_puntua_s'), 'end': res.get('termino_por'),
                     'end_s': res.get('duracion_s'), 'fast_bonus': score.get('bono_rapido'),
                     'return': score.get('bono_regreso'), 'collisions': hits.get('collisions'),
                     'close_calls': hits.get('close_calls'), **count})
rows.sort(key=lambda r: (r['map'] or '', r['label'], r['rep'] or 0))
print('\t'.join(COLUMNS))
for r in rows:
    print('\t'.join(str(r.get(c, '')) for c in COLUMNS))
